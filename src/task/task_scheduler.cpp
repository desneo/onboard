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

bool TaskScheduler::init() { return true; }

bool TaskScheduler::start() {
    event_bus_.subscribe<plane::model::TaskInfo>(plane::communication::events::kTaskInfoReceived,
                                                 [this](const plane::model::TaskInfo& task) { on_task_info(task); });
    event_bus_.subscribe<plane::model::Command>(plane::communication::events::kCommandReceived,
                                                [this](const plane::model::Command& command) { on_command(command); });
    event_bus_.subscribe<TaskEngineResult>(events::kEngineResult,
                                           [this](const TaskEngineResult& result) { on_engine_result(result); });
    event_bus_.subscribe<TaskProgress>(events::kExecutionProgress,
                                       [this](const TaskProgress& progress) { on_execution_progress(progress); });
    return true;
}

void TaskScheduler::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    control_ = ControlKind::None;
    task_state_ = plane::model::TaskState{};
}

void TaskScheduler::release() {}

void TaskScheduler::set_communication(plane::communication::CommunicationApi& communication) {
    communication_ = &communication;
}

plane::model::TaskState TaskScheduler::current_task_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return task_state_;
}

void TaskScheduler::on_task_info(const plane::model::TaskInfo& task) {
    if (task.taskAction == plane::model::TaskAction::Stop) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (control_ != ControlKind::Task || task_state_.taskId != task.taskId) {
                SPDLOG_WARN("忽略不匹配的任务停止: taskId={}", task.taskId);
                return;
            }
            SPDLOG_INFO("任务中止: taskId={}", task_state_.taskId);
            control_ = ControlKind::None;
            task_state_ = plane::model::TaskState{};
        }
        if (engine_ != nullptr) {
            engine_->abort();
        }
        return;
    }

    bool replaced_task = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        replaced_task = control_ == ControlKind::Task;
        if (replaced_task) {
            SPDLOG_INFO("任务被新任务替换: taskId={}", task_state_.taskId);
        }
        control_ = ControlKind::Task;
        task_state_ = plane::model::TaskState{};
        task_state_.taskId = task.taskId;
        task_state_.status = plane::model::TaskStatus::Pending;
    }
    if (replaced_task && engine_ != nullptr) {
        engine_->abort();
    }
    if (engine_ == nullptr) {
        SPDLOG_WARN("执行引擎未接入，任务保持待执行: taskId={}", task.taskId);
        return;
    }
    std::string error;
    if (engine_->start(task, error)) {
        std::lock_guard<std::mutex> lock(mutex_);
        task_state_.status = plane::model::TaskStatus::InProgress;
        SPDLOG_INFO("任务接收并启动: taskId={}", task.taskId);
    } else {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            control_ = ControlKind::None;
            task_state_ = plane::model::TaskState{};
        }
        report_error("任务启动失败: " + error);
    }
}

void TaskScheduler::on_command(const plane::model::Command& command) {
    bool had_control = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        had_control = control_ != ControlKind::None;
        if (control_ == ControlKind::Task) {
            SPDLOG_INFO("任务被独立指令替换: taskId={}", task_state_.taskId);
        }
        control_ = ControlKind::Command;
        task_state_ = plane::model::TaskState{};
    }
    if (had_control && engine_ != nullptr) {
        engine_->abort();
    }
    if (engine_ == nullptr) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            control_ = ControlKind::None;
        }
        const std::string detail = "指令派发失败: " + command.commandId + " (执行引擎未接入)";
        report_error(detail);
        send_alert(detail);
        return;
    }
    std::string error;
    if (engine_->start_command(command, error)) {
        SPDLOG_INFO("指令已派发: commandId={}", command.commandId);
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        control_ = ControlKind::None;
    }
    const std::string detail = "指令派发失败: " + command.commandId + " (" + error + ")";
    report_error(detail);
    send_alert(detail);
}

void TaskScheduler::on_engine_result(const TaskEngineResult& result) {
    bool command_failed = false;
    std::string failure_detail;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (control_ == ControlKind::Task) {
            SPDLOG_INFO("任务结束: taskId={} status={} detail={}", task_state_.taskId,
                        std::string(magic_enum::enum_name(result.status)), result.detail);
            control_ = ControlKind::None;
            task_state_ = plane::model::TaskState{};
            return;
        }
        if (control_ == ControlKind::Command) {
            control_ = ControlKind::None;
            if (result.status == plane::model::TaskStatus::Failed) {
                command_failed = true;
                failure_detail = result.detail;
            } else {
                SPDLOG_INFO("指令执行结束: status={} detail={}", std::string(magic_enum::enum_name(result.status)),
                            result.detail);
            }
        } else {
            SPDLOG_WARN("忽略非当前执行的引擎结果");
            return;
        }
    }
    if (command_failed) {
        const std::string detail = "指令执行失败: " + failure_detail;
        report_error(detail);
        send_alert(detail);
    }
}

void TaskScheduler::on_execution_progress(const TaskProgress& progress) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (control_ != ControlKind::Task) {
        return;
    }
    task_state_.currentBehaviorTree = progress.currentBehaviorTree;
    task_state_.currentNode = progress.currentNode;
}

void TaskScheduler::send_alert(const std::string& detail) {
    if (communication_ == nullptr) {
        SPDLOG_ERROR("通信模块未接入，告警未发送: {}", detail);
        return;
    }
    plane::model::Alert alert;
    alert.type = "command_dispatch_failed";
    alert.detail = detail;
    if (!communication_->sendToStation(plane::communication::OutgoingMessage{alert})) {
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
