#include "task/task_scheduler.h"

#include <spdlog/spdlog.h>

#include <magic_enum/magic_enum.hpp>

#include "communication/communication_events.h"

namespace plane::task {

TaskScheduler::TaskScheduler(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                             TaskExecutionEngineApi* engine)
    : event_bus_(event_bus), config_(config), engine_(engine) {}

TaskScheduler::~TaskScheduler() { stop(); }

std::string TaskScheduler::name() const { return "任务调度器"; }

bool TaskScheduler::init() {
    SPDLOG_INFO("模块初始化完成: {}", name());
    return true;
}

bool TaskScheduler::start() {
    using plane::communication::ReceivedControl;
    using ReceivedTask = ReceivedControl<plane::model::TaskInfo>;
    using ReceivedCommand = ReceivedControl<plane::model::Command>;
    {
        std::lock_guard<std::mutex> lock(dispatch_mutex_);
        running_ = true;
    }
    event_bus_.subscribe<ReceivedTask>(
        plane::communication::events::kTaskInfoReceived,
        [this](const ReceivedTask& request) { on_task_info(request.value, request.received_at); });
    event_bus_.subscribe<ReceivedCommand>(
        plane::communication::events::kCommandReceived,
        [this](const ReceivedCommand& request) { on_command(request.value, request.received_at); });
    event_bus_.subscribe<plane::flight_control::ManualTakeoverEvent>(
        plane::flight_control::events::kManualTakeover,
        [this](const plane::flight_control::ManualTakeoverEvent& event) { on_manual_takeover(event); });
    event_bus_.subscribe<TaskEngineResult>(events::kEngineResult,
                                           [this](const TaskEngineResult& result) { on_engine_result(result); });
    event_bus_.subscribe<TaskProgress>(events::kExecutionProgress,
                                       [this](const TaskProgress& progress) { on_execution_progress(progress); });
    SPDLOG_INFO("模块启动完成: {}", name());
    return true;
}

void TaskScheduler::stop() {
    std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex_);
    running_ = false;
    std::lock_guard<std::mutex> lock(mutex_);
    clear_control_locked();
}

void TaskScheduler::release() {}

void TaskScheduler::set_communication(plane::communication::CommunicationApi& communication) {
    communication_ = &communication;
}

plane::model::TaskState TaskScheduler::current_task_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return task_state_;
}

bool TaskScheduler::prepare_control(ControlTime received_at) {
    if (!running_ || received_at <= last_takeover_finished_at_) {
        return false;
    }
    return engine_ == nullptr || (engine_->control_can_advance() && engine_->abort());
}

void TaskScheduler::clear_control_locked() {
    control_ = ControlKind::None;
    execution_id_ = 0;
    command_id_.clear();
    task_state_ = plane::model::TaskState{};
}

void TaskScheduler::on_task_info(const plane::model::TaskInfo& task, ControlTime received_at) {
    std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex_);
    if (task.taskAction == plane::model::TaskAction::Stop) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (control_ != ControlKind::Task || task_state_.taskId != task.taskId) {
                SPDLOG_WARN("忽略不匹配的任务停止: taskId={}", task.taskId);
                return;
            }
        }
        if (prepare_control(received_at)) {
            SPDLOG_INFO("任务中止: taskId={}", task.taskId);
            std::lock_guard<std::mutex> lock(mutex_);
            clear_control_locked();
        }
        return;
    }

    if (!prepare_control(received_at)) {
        SPDLOG_WARN("忽略接管核对或清理期间收到的任务: taskId={}", task.taskId);
        return;
    }
    const auto execution_id = ++next_execution_id_;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (control_ == ControlKind::Task) {
            SPDLOG_INFO("任务被新任务替换: taskId={}", task_state_.taskId);
        }
        clear_control_locked();
        control_ = ControlKind::Task;
        execution_id_ = execution_id;
        task_state_.taskId = task.taskId;
        task_state_.status = plane::model::TaskStatus::Pending;
    }
    if (engine_ == nullptr) {
        SPDLOG_WARN("执行引擎未接入，任务保持待执行: taskId={}", task.taskId);
        return;
    }
    std::string error;
    if (engine_->start(execution_id, task, error)) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (execution_id_ == execution_id) {
            task_state_.status = plane::model::TaskStatus::InProgress;
        }
        SPDLOG_INFO("任务接收并启动: taskId={}", task.taskId);
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        clear_control_locked();
    }
    report_error("任务启动失败: " + error);
}

void TaskScheduler::on_command(const plane::model::Command& command, ControlTime received_at) {
    std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex_);
    if (!prepare_control(received_at)) {
        SPDLOG_WARN("忽略接管核对或清理期间收到的指令: commandId={}", command.commandId);
        return;
    }
    const auto execution_id = ++next_execution_id_;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (control_ == ControlKind::Task) {
            SPDLOG_INFO("任务被独立指令替换: taskId={}", task_state_.taskId);
        }
        clear_control_locked();
        control_ = ControlKind::Command;
        execution_id_ = execution_id;
        command_id_ = command.commandId;
    }
    std::string error;
    if (engine_ != nullptr && engine_->start_command(execution_id, command, error)) {
        SPDLOG_INFO("指令已派发: commandId={}", command.commandId);
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        clear_control_locked();
    }
    if (engine_ == nullptr) {
        error = "执行引擎未接入";
    }
    const std::string detail = "指令派发失败: " + command.commandId + " (" + error + ")";
    report_error(detail);
    send_alert(detail);
}

void TaskScheduler::on_manual_takeover(const plane::flight_control::ManualTakeoverEvent& event) {
    std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex_);
    if (!running_) {
        return;
    }

    plane::model::TaskState aborted_task;
    std::string aborted_command_id;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (control_ == ControlKind::None || execution_id_ != event.execution_id) {
            return;
        }
        if (control_ == ControlKind::Task) {
            aborted_task = task_state_;
            aborted_task.status = plane::model::TaskStatus::Aborted;
        } else {
            aborted_command_id = command_id_;
        }
    }

    if (engine_ != nullptr) {
        engine_->abort(true);
    }
    last_takeover_finished_at_ = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        clear_control_locked();
    }

    std::string detail = "已切入接管模式 " + event.flight_mode;
    if (aborted_task.status == plane::model::TaskStatus::Aborted) {
        detail += "，当前任务已中止（taskId=" + aborted_task.taskId + "）";
        if (communication_ != nullptr && !communication_->sendToStation(aborted_task)) {
            SPDLOG_ERROR("接管中止状态发送失败: taskId={}", aborted_task.taskId);
        }
    } else {
        detail += "，当前指令已中止（commandId=" + aborted_command_id + "）";
    }
    SPDLOG_WARN("{}", detail);
    send_alert(detail, "manual_takeover");
}

void TaskScheduler::on_engine_result(const TaskEngineResult& result) {
    bool command_failed = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (control_ == ControlKind::None || execution_id_ != result.execution_id) {
            return;
        }
        if (control_ == ControlKind::Task) {
            SPDLOG_INFO("任务结束: taskId={} status={} detail={}", task_state_.taskId,
                        std::string(magic_enum::enum_name(result.status)), result.detail);
        } else if (result.status == plane::model::TaskStatus::Failed) {
            command_failed = true;
        } else {
            SPDLOG_INFO("指令执行结束: status={} detail={}", std::string(magic_enum::enum_name(result.status)),
                        result.detail);
        }
        clear_control_locked();
    }
    if (command_failed) {
        const std::string detail = "指令执行失败: " + result.detail;
        report_error(detail);
        send_alert(detail);
    }
}

void TaskScheduler::on_execution_progress(const TaskProgress& progress) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (control_ != ControlKind::Task || execution_id_ != progress.execution_id) {
        return;
    }
    task_state_.currentBehaviorTree = progress.currentBehaviorTree;
    task_state_.currentNode = progress.currentNode;
}

void TaskScheduler::send_alert(const std::string& detail, const std::string& type) {
    if (communication_ == nullptr) {
        SPDLOG_ERROR("通信模块未接入，告警未发送: {}", detail);
        return;
    }
    plane::model::Alert alert{type, detail};
    if (!communication_->sendToStation(alert)) {
        SPDLOG_ERROR("告警发送失败: {}", detail);
    }
}

void TaskScheduler::report_error(const std::string& detail) const {
    SPDLOG_ERROR("任务调度器错误: {}", detail);
    TaskSchedulerError event;
    event.detail = detail;
    event_bus_.publish(events::kError, event);
}

}  // namespace plane::task
