#include "task/task_scheduler.h"

#include <spdlog/spdlog.h>

#include <magic_enum/magic_enum.hpp>

#include "communication/communication_events.h"

namespace plane::task {

TaskScheduler::TaskScheduler(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                             plane::flight_control::FlightControlApi& flight_control,
                             plane::core::SystemParamApi& system_param, TaskExecutionEngineApi* engine)
    : event_bus_(event_bus),
      config_(config),
      flight_control_(flight_control),
      system_param_(system_param),
      engine_(engine) {}

TaskScheduler::~TaskScheduler() { stop(); }

std::string TaskScheduler::name() const { return "Task Scheduler"; }

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
                spdlog::warn("忽略不匹配的任务停止: taskId={}", task.taskId);
                return;
            }
            spdlog::info("任务中止: taskId={}", task_state_.taskId);
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
            spdlog::info("任务被新任务替换: taskId={}", task_state_.taskId);
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
        spdlog::warn("执行引擎未接入，任务保持 Pending: taskId={}", task.taskId);
        return;
    }
    std::string error;
    if (engine_->start(task, error)) {
        std::lock_guard<std::mutex> lock(mutex_);
        task_state_.status = plane::model::TaskStatus::InProgress;
        spdlog::info("任务接收并启动: taskId={}", task.taskId);
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
    bool replaced_task = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        replaced_task = control_ == ControlKind::Task;
        if (replaced_task) {
            spdlog::info("任务被独立指令替换: taskId={}", task_state_.taskId);
        }
        control_ = ControlKind::Command;
        task_state_ = plane::model::TaskState{};
    }
    if (replaced_task && engine_ != nullptr) {
        engine_->abort();
    }
    dispatch_command(command);
}

void TaskScheduler::on_engine_result(const TaskEngineResult& result) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (control_ != ControlKind::Task) {
        spdlog::warn("忽略非当前任务的引擎结果");
        return;
    }
    spdlog::info("任务结束: taskId={}, status={}, detail={}", task_state_.taskId,
                 std::string(magic_enum::enum_name(result.status)), result.detail);
    control_ = ControlKind::None;
    task_state_ = plane::model::TaskState{};
}

void TaskScheduler::on_execution_progress(const TaskProgress& progress) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (control_ != ControlKind::Task) {
        return;
    }
    task_state_.currentBehaviorTree = progress.currentBehaviorTree;
    task_state_.currentNode = progress.currentNode;
}

bool TaskScheduler::dispatch_command(const plane::model::Command& command) {
    std::string error;
    bool ok = false;
    switch (command.type) {
        case plane::model::CommandType::Arm:
            ok = flight_control_.arm(error);
            break;
        case plane::model::CommandType::Disarm:
            ok = flight_control_.disarm(error);
            break;
        case plane::model::CommandType::Takeoff: {
            float height = command.takeoffHeight_m;
            if (height <= 0.0F) {
                height = static_cast<float>(system_param_.current().safeHeight_m);
                spdlog::info("Takeoff 未指定高度，使用安全高度: {} m", height);
            }
            ok = flight_control_.takeoff(height, error);
            break;
        }
        case plane::model::CommandType::Land:
            ok = flight_control_.land(error);
            break;
        case plane::model::CommandType::RTL:
            ok = flight_control_.return_to_launch(error);
            break;
        case plane::model::CommandType::Hover:
            ok = flight_control_.hold(error);
            break;
        case plane::model::CommandType::Waypoint:
            if (command.waypoints.empty()) {
                error = "航点为空";
                break;
            }
            ok = flight_control_.goto_waypoint(command.waypoints.front(), error);
            break;
        case plane::model::CommandType::Route:
            if (command.waypoints.empty()) {
                error = "航点为空";
                break;
            }
            ok = flight_control_.execute_route(command.waypoints, error);
            break;
        case plane::model::CommandType::StopMotor:
            ok = flight_control_.stop_motor(error);
            break;
    }
    if (ok) {
        spdlog::info("指令已派发: {}", command.commandId);
        return true;
    }
    const std::string detail = "指令派发失败: " + command.commandId + " (" + error + ")";
    report_error(detail);
    send_alert(detail);
    return false;
}

void TaskScheduler::send_alert(const std::string& detail) {
    if (communication_ == nullptr) {
        spdlog::error("通信模块未接入，告警未发送: {}", detail);
        return;
    }
    plane::model::Alert alert;
    alert.type = "command_dispatch_failed";
    alert.detail = detail;
    if (!communication_->sendToStation(plane::communication::OutgoingMessage{alert})) {
        spdlog::error("告警发送失败: {}", detail);
    }
}

void TaskScheduler::report_error(const std::string& detail) const {
    spdlog::error("TaskScheduler: {}", detail);
    TaskSchedulerError event;
    event.detail = detail;
    event_bus_.publish(events::kError, event);
}

}  // namespace plane::task
