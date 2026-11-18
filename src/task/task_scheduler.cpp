#include "task/task_scheduler.h"

#include <optional>
#include <utility>

#include <magic_enum/magic_enum.hpp>
#include <spdlog/spdlog.h>

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
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (control_ == ControlKind::SelfCheck) {
            SPDLOG_WARN("飞机自检中，拒绝任务: taskId={}", task.taskId);
            return;
        }
    }
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
    const std::string detail = "任务启动失败: " + error;
    on_engine_result({execution_id, plane::model::TaskStatus::Failed, detail});
    report_error(detail);
}

void TaskScheduler::on_command(const plane::model::Command& command, ControlTime received_at) {
    using plane::model::CommandType;
    std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex_);
    if (!running_) {
        return;
    }
    const bool self_check = command.type == CommandType::SelfCheck;
    const bool priority = command.type == CommandType::Disarm || command.type == CommandType::StopMotor;
    ControlKind current;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current = control_;
    }
    if (self_check && current != ControlKind::None) {
        report_self_check_result({command.commandId, false, "飞机正在执行任务或指令，无法开始自检"});
        return;
    }
    if (current == ControlKind::SelfCheck && !priority) {
        const std::string detail = "飞机自检中，拒绝指令（commandId=" + command.commandId + "）";
        SPDLOG_WARN("{}", detail);
        send_alert(detail);
        return;
    }
    if (!prepare_control(received_at)) {
        if (self_check) {
            report_self_check_result({command.commandId, false, "飞控接管核对或清理中，无法开始自检"});
        } else {
            SPDLOG_WARN("忽略接管核对或清理期间收到的指令: commandId={}", command.commandId);
        }
        return;
    }
    const auto execution_id = ++next_execution_id_;
    std::optional<std::string> interrupted_self_check;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (control_ == ControlKind::Task) {
            SPDLOG_INFO("任务被独立指令替换: taskId={}", task_state_.taskId);
        } else if (control_ == ControlKind::SelfCheck) {
            interrupted_self_check = command_id_;
        }
        clear_control_locked();
        control_ = self_check ? ControlKind::SelfCheck : ControlKind::Command;
        execution_id_ = execution_id;
        command_id_ = command.commandId;
    }
    std::string error;
    const bool started = engine_ != nullptr && engine_->start_command(execution_id, command, error);
    if (interrupted_self_check) {
        report_self_check_result({*interrupted_self_check, false,
                                  "自检被 " + std::string(magic_enum::enum_name(command.type)) +
                                      " 指令中止，未确认恢复上锁"});
    }
    if (started) {
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
    if (self_check) {
        report_self_check_result({command.commandId, false, "自检派发失败：" + error});
        return;
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
    bool aborted_self_check = false;
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
            aborted_self_check = control_ == ControlKind::SelfCheck;
            if (aborted_self_check) {
                clear_control_locked();
            }
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
        report_task_state(aborted_task);
    } else {
        detail += "，当前指令已中止（commandId=" + aborted_command_id + "）";
    }
    SPDLOG_WARN("{}", detail);
    send_alert(detail, "manual_takeover");
    if (aborted_self_check) {
        report_self_check_result({aborted_command_id, false,
                                  "飞控切入接管模式 " + event.flight_mode + "，自检中止，未确认恢复上锁"});
    }
}

void TaskScheduler::on_engine_result(const TaskEngineResult& result) {
    bool command_failed = false;
    std::optional<plane::model::TaskState> finished_task;
    std::optional<plane::model::SelfCheckResult> self_check_result;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (control_ == ControlKind::None || execution_id_ != result.execution_id) {
            return;
        }
        if (control_ == ControlKind::Task) {
            task_state_.status = result.status;
            finished_task = task_state_;
            SPDLOG_INFO("任务结束: taskId={} status={} detail={}", task_state_.taskId,
                        std::string(magic_enum::enum_name(result.status)), result.detail);
        } else if (control_ == ControlKind::SelfCheck) {
            const bool success = result.status == plane::model::TaskStatus::Completed;
            std::string detail = result.detail;
            if (detail.empty()) {
                detail = success ? "已确认解锁并恢复上锁" : "自检执行失败，未确认恢复上锁";
            }
            self_check_result = plane::model::SelfCheckResult{command_id_, success, std::move(detail)};
        } else if (result.status == plane::model::TaskStatus::Failed) {
            command_failed = true;
        } else {
            SPDLOG_INFO("指令执行结束: status={} detail={}", std::string(magic_enum::enum_name(result.status)),
                        result.detail);
        }
        clear_control_locked();
    }
    if (finished_task) {
        report_task_state(*finished_task);
    } else if (self_check_result) {
        report_self_check_result(*self_check_result);
    } else if (command_failed) {
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

void TaskScheduler::report_task_state(const plane::model::TaskState& state) {
    if (communication_ == nullptr || !communication_->sendToStation(state)) {
        SPDLOG_ERROR("任务状态发送失败: taskId={} status={}", state.taskId, magic_enum::enum_name(state.status));
    }
}

void TaskScheduler::report_self_check_result(const plane::model::SelfCheckResult& result) {
    if (result.success) {
        SPDLOG_INFO("自检通过: commandId={} detail={}", result.commandId, result.detail);
    } else {
        SPDLOG_WARN("自检失败: commandId={} detail={}", result.commandId, result.detail);
    }
    if (communication_ == nullptr || !communication_->sendToStation(result)) {
        SPDLOG_ERROR("自检结果发送失败: commandId={}", result.commandId);
    }
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
