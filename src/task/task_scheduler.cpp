#include "task/task_scheduler.h"

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

bool TaskScheduler::prepare_control(ControlTime received_at, const std::string& interruption) {
    if (!running_ || received_at <= last_takeover_finished_at_) {
        return false;
    }
    std::uint64_t previous_execution;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        previous_execution = execution_id_;
    }
    // abort 会等待执行线程退出；结果回调仍须能取得状态锁并认领自然结束。
    if (engine_ != nullptr && (!engine_->control_can_advance() || !engine_->abort())) {
        return false;
    }
    finish_control({previous_execution, plane::model::TaskStatus::Aborted, interruption, {}, {}});
    return true;
}

void TaskScheduler::clear_control_locked() {
    control_ = ControlKind::None;
    execution_id_ = 0;
    command_id_.clear();
    control_type_.clear();
    current_tree_.clear();
    current_node_.clear();
    task_state_ = plane::model::TaskState{};
}

bool TaskScheduler::claim_control_locked(std::uint64_t execution_id, ControlSnapshot& snapshot) {
    if (control_ == ControlKind::None || execution_id == 0 || execution_id_ != execution_id) {
        return false;
    }
    snapshot = {control_, task_state_, command_id_, control_type_, current_tree_, current_node_};
    clear_control_locked();
    return true;
}

bool TaskScheduler::finish_control(const TaskEngineResult& result) {
    ControlSnapshot snapshot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!claim_control_locked(result.execution_id, snapshot)) {
            return false;
        }
    }
    if (!result.currentBehaviorTree.empty()) snapshot.current_tree = result.currentBehaviorTree;
    if (!result.currentNode.empty()) snapshot.current_node = result.currentNode;
    report_finished(std::move(snapshot), result.status, result.detail);
    return true;
}

void TaskScheduler::report_finished(ControlSnapshot snapshot, plane::model::TaskStatus status,
                                    const std::string& result_detail) {
    using plane::model::TaskStatus;
    const bool task = snapshot.kind == ControlKind::Task;
    const bool self_check = snapshot.kind == ControlKind::SelfCheck;
    std::string reason = result_detail;
    if (reason.empty()) {
        if (self_check) {
            reason = status == TaskStatus::Completed ? "已确认解锁并恢复上锁" : "自检执行失败，未确认恢复上锁";
        } else {
            reason = status == TaskStatus::Completed ? "执行完成" :
                     status == TaskStatus::Aborted ? "执行已中止" : "执行失败，未提供详情";
        }
    }
    std::string detail = task ? "任务结束：taskId=" + snapshot.task_state.taskId + " taskCode=" + snapshot.type
                              : "指令结束：commandId=" + snapshot.command_id + " type=" + snapshot.type;
    detail += " status=" + std::string(magic_enum::enum_name(status));
    if (!snapshot.current_tree.empty()) detail += " currentBehaviorTree=" + snapshot.current_tree;
    if (!snapshot.current_node.empty()) detail += " currentNode=" + snapshot.current_node;
    detail += "，" + reason;
    SPDLOG_INFO("{}", detail);
    send_alert(detail, task ? "task_finished" : "command_finished");
    if (task) {
        snapshot.task_state.status = status;
        snapshot.task_state.currentBehaviorTree = snapshot.current_tree;
        snapshot.task_state.currentNode = snapshot.current_node;
        report_task_state(snapshot.task_state);
    } else if (self_check) {
        report_self_check_result({snapshot.command_id, status == TaskStatus::Completed, reason});
    } else if (status == TaskStatus::Failed) {
        const std::string failure = "指令执行失败：commandId=" + snapshot.command_id + " type=" + snapshot.type +
                                    "，" + reason;
        report_error(failure);
        send_alert(failure);
    }
}

void TaskScheduler::report_rejected_command(const plane::model::Command& command, const std::string& reason) {
    if (command.type == plane::model::CommandType::SelfCheck) {
        report_self_check_result({command.commandId, false, reason});
    }
    const std::string detail = "指令未开始：commandId=" + command.commandId + " type=" +
                               std::string(magic_enum::enum_name(command.type)) + " status=Failed，" + reason;
    send_alert(detail, "command_finished");
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
        if (prepare_control(received_at, "收到任务停止请求")) {
            SPDLOG_INFO("任务中止: taskId={}", task.taskId);
        }
        return;
    }

    if (!prepare_control(received_at, "被新任务替换（taskId=" + task.taskId + "）")) {
        SPDLOG_WARN("忽略接管核对或清理期间收到的任务: taskId={}", task.taskId);
        return;
    }
    const auto execution_id = ++next_execution_id_;
    const std::string type(magic_enum::enum_name(task.taskCode));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        control_ = ControlKind::Task;
        execution_id_ = execution_id;
        control_type_ = type;
        task_state_.taskId = task.taskId;
        task_state_.status = plane::model::TaskStatus::Pending;
    }
    // 引擎可在 start 返回前同步发布结果，开始事件必须先入队。
    send_alert("开始执行任务：taskId=" + task.taskId + " taskCode=" + type +
                   (task.targetTrackId.empty() ? "" : " targetTrackId=" + task.targetTrackId), "task_started");
    std::string error;
    if (engine_ != nullptr && engine_->start(execution_id, task, error)) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (execution_id_ == execution_id) {
            task_state_.status = plane::model::TaskStatus::InProgress;
        }
        SPDLOG_INFO("任务接收并启动: taskId={}", task.taskId);
        return;
    }
    if (engine_ == nullptr) error = "执行引擎未接入";
    const std::string detail = "任务启动失败: " + error;
    on_engine_result({execution_id, plane::model::TaskStatus::Failed, detail, {}, {}});
    report_error(detail);
}

void TaskScheduler::on_command(const plane::model::Command& command, ControlTime received_at) {
    using plane::model::CommandType;
    std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex_);
    if (!running_) return;
    const bool self_check = command.type == CommandType::SelfCheck;
    const bool priority = command.type == CommandType::Disarm || command.type == CommandType::StopMotor;
    ControlKind current;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current = control_;
    }
    if (self_check && current != ControlKind::None) {
        report_rejected_command(command, "飞机正在执行任务或指令，无法开始自检");
        return;
    }
    if (current == ControlKind::SelfCheck && !priority) {
        const std::string detail = "飞机自检中，拒绝指令（commandId=" + command.commandId + "）";
        SPDLOG_WARN("{}", detail);
        send_alert(detail);
        report_rejected_command(command, detail);
        return;
    }
    const std::string type(magic_enum::enum_name(command.type));
    const std::string interruption = current == ControlKind::SelfCheck
        ? "自检被 " + type + " 指令中止，未确认恢复上锁"
        : "被新指令替换（commandId=" + command.commandId + " type=" + type + "）";
    if (!prepare_control(received_at, interruption)) {
        if (self_check) {
            report_rejected_command(command, "飞控接管核对或清理中，无法开始自检");
        } else {
            SPDLOG_WARN("忽略接管核对或清理期间收到的指令: commandId={}", command.commandId);
        }
        return;
    }
    const auto execution_id = ++next_execution_id_;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        control_ = self_check ? ControlKind::SelfCheck : ControlKind::Command;
        execution_id_ = execution_id;
        command_id_ = command.commandId;
        control_type_ = type;
    }
    send_alert("开始执行指令：commandId=" + command.commandId + " type=" + type, "command_started");
    std::string error;
    if (engine_ != nullptr && engine_->start_command(execution_id, command, error)) {
        SPDLOG_INFO("指令已派发: commandId={}", command.commandId);
        return;
    }
    if (engine_ == nullptr) error = "执行引擎未接入";
    const std::string detail = self_check ? "自检派发失败：" + error : "指令派发失败：" + error;
    on_engine_result({execution_id, plane::model::TaskStatus::Failed, detail, {}, {}});
}

void TaskScheduler::on_manual_takeover(const plane::flight_control::ManualTakeoverEvent& event) {
    std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex_);
    if (!running_) return;
    ControlSnapshot snapshot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // 先认领终态，再等待执行线程退出，防止自检的同步结果重复上报。
        if (!claim_control_locked(event.execution_id, snapshot)) return;
    }
    if (engine_ != nullptr) engine_->abort(true);
    last_takeover_finished_at_ = std::chrono::steady_clock::now();
    std::string detail = "检测到接管：previous=" + event.previous_flight_mode + " value=" + event.flight_mode +
                         "，停止自动控制";
    if (snapshot.kind == ControlKind::Task) {
        detail += "，当前任务已中止（taskId=" + snapshot.task_state.taskId + "）";
    } else {
        detail += "，当前指令已中止（commandId=" + snapshot.command_id + "）";
    }
    SPDLOG_WARN("{}", detail);
    send_alert(detail, "manual_takeover");
    const std::string reason = snapshot.kind == ControlKind::SelfCheck
        ? "飞控切入接管模式 " + event.flight_mode + "，自检中止，未确认恢复上锁"
        : "人工接管（previous=" + event.previous_flight_mode + " value=" + event.flight_mode + "），停止自动控制";
    report_finished(std::move(snapshot), plane::model::TaskStatus::Aborted, reason);
}

void TaskScheduler::on_engine_result(const TaskEngineResult& result) { finish_control(result); }

void TaskScheduler::on_execution_progress(const TaskProgress& progress) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (control_ == ControlKind::None || execution_id_ != progress.execution_id) return;
    current_tree_ = progress.currentBehaviorTree;
    current_node_ = progress.currentNode;
    if (control_ == ControlKind::Task) {
        task_state_.currentBehaviorTree = progress.currentBehaviorTree;
        task_state_.currentNode = progress.currentNode;
    }
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
