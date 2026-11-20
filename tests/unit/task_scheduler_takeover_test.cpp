#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include "communication/communication_events.h"
#include "task/task_scheduler.h"

namespace {

using namespace plane;
using Clock = std::chrono::steady_clock;

void require(bool condition, const char* detail) {
    if (!condition) {
        throw std::runtime_error(detail);
    }
}

class CapturedLogs {
   public:
    CapturedLogs() : output_(&buffer_), previous_(spdlog::default_logger()) {
        auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(output_);
        auto logger = std::make_shared<spdlog::logger>("takeover-test", sink);
        logger->set_pattern("%v");
        // 关闭级别触发刷新，仅显式 flush 才将捕获内容标记为已刷新。
        logger->flush_on(spdlog::level::off);
        spdlog::set_default_logger(logger);
    }

    ~CapturedLogs() { spdlog::set_default_logger(previous_); }

    std::string text() const { return buffer_.str(); }
    std::string flushed_text() const { return buffer_.flushed; }
    std::size_t flush_count() const { return buffer_.flushes; }

    std::size_t takeover_count() const {
        const auto logs = text();
        std::size_t count = 0;
        for (auto pos = logs.find("检测到接管："); pos != std::string::npos;
             pos = logs.find("检测到接管：", pos + 1)) {
            ++count;
        }
        return count;
    }

   private:
    class CaptureBuffer final : public std::stringbuf {
       public:
        std::string flushed;
        std::size_t flushes = 0;

       protected:
        int sync() override {
            flushed = str();
            ++flushes;
            return 0;
        }
    };

    CaptureBuffer buffer_;
    std::ostream output_;
    std::shared_ptr<spdlog::logger> previous_;
};

class TestEngine final : public task::TaskExecutionEngineApi {
   public:
    bool start(std::uint64_t execution_id, const model::TaskInfo&, std::string& error) override {
        if (!task_start_error.empty()) {
            error = task_start_error;
            return false;
        }
        starts.push_back(execution_id);
        if (task_start_hook) task_start_hook(execution_id);
        return true;
    }

    bool start_command(std::uint64_t execution_id, const model::Command& command, std::string& error) override {
        if (command.type == model::CommandType::SelfCheck && !self_check_start_error.empty()) {
            error = self_check_start_error;
            return false;
        }
        if (!command_start_error.empty()) {
            error = command_start_error;
            return false;
        }
        starts.push_back(execution_id);
        commands.push_back(command.type);
        if (command_start_hook) command_start_hook(execution_id);
        return true;
    }

    bool abort(bool acknowledge_takeover) override {
        if (acknowledge_takeover) {
            ++takeover_aborts;
            gate_open = true;
            return true;
        }
        ++aborts;
        if (abort_hook) {
            auto hook = std::move(abort_hook);
            abort_hook = {};
            hook();
        }
        if (takeover_during_abort) {
            takeover_during_abort = false;
            gate_open = false;
        }
        return gate_open;
    }

    bool control_can_advance() const override { return gate_open; }

    std::vector<std::uint64_t> starts;
    std::vector<model::CommandType> commands;
    std::string task_start_error;
    std::string self_check_start_error;
    std::string command_start_error;
    std::function<void(std::uint64_t)> task_start_hook;
    std::function<void(std::uint64_t)> command_start_hook;
    std::function<void()> abort_hook;
    bool gate_open = true;
    bool takeover_during_abort = false;
    int aborts = 0;
    int takeover_aborts = 0;
};

class TestCommunication final : public communication::CommunicationApi {
   public:
    bool sendToStation(const communication::OutgoingMessage& message) override {
        if (const auto* alert = std::get_if<model::Alert>(&message)) {
            alerts.push_back(*alert);
        }
        if (const auto* state = std::get_if<model::TaskState>(&message)) {
            states.push_back(*state);
        }
        if (const auto* result = std::get_if<model::SelfCheckResult>(&message)) {
            self_check_results.push_back(*result);
        }
        return true;
    }

    bool sendToPlane(const std::string&, const communication::OutgoingMessage&) override { return false; }
    bool sendToAll(const communication::OutgoingMessage&) override { return false; }
    bool queryAgent(const std::string&, model::Message&, std::string&, int) override { return false; }

    std::vector<model::Alert> alerts_of(const std::string& type) const {
        std::vector<model::Alert> selected;
        for (const auto& alert : alerts) {
            if (alert.type == type) selected.push_back(alert);
        }
        return selected;
    }

    std::vector<model::Alert> alerts;
    std::vector<model::TaskState> states;
    std::vector<model::SelfCheckResult> self_check_results;
};


std::string expect_lifecycle(const TestCommunication& communication, bool task, const std::string& id,
                             const std::string& status, const std::string& reason) {
    const std::string kind = task ? "task" : "command";
    const std::string start_prefix = task ? "开始执行任务：taskId=" + id + " "
                                         : "开始执行指令：commandId=" + id + " ";
    const std::string end_prefix = task ? "任务结束：taskId=" + id + " " : "指令结束：commandId=" + id + " ";
    std::vector<model::Alert> lifecycle;
    for (const auto& alert : communication.alerts) {
        if ((alert.type == kind + "_started" && alert.detail.rfind(start_prefix, 0) == 0) ||
            (alert.type == kind + "_finished" && alert.detail.rfind(end_prefix, 0) == 0)) {
            lifecycle.push_back(alert);
        }
    }
    require(lifecycle.size() == 2 && lifecycle.front().type == kind + "_started" &&
                lifecycle.back().type == kind + "_finished", "生命周期必须恰好开始、结束各一次，且顺序正确");
    require(lifecycle.back().detail.find("status=" + status) != std::string::npos &&
                lifecycle.back().detail.find(reason) != std::string::npos, "生命周期结束状态或原因不完整");
    return lifecycle.back().detail;
}

void task_scheduler_reports_takeover_once() {
    CapturedLogs logs;
    core::AppConfig config;
    core::EventBus bus;
    TestEngine engine;
    TestCommunication communication;
    task::TaskScheduler scheduler(bus, config, &engine);
    scheduler.set_communication(communication);
    require(scheduler.start(), "任务调度器启动失败");

    auto send_task = [&](const std::string& id) {
        model::TaskInfo task;
        task.taskId = id;
        task.taskCode = model::TaskCode::AntiUav;
        task.taskAction = model::TaskAction::Start;
        bus.publish(communication::events::kTaskInfoReceived, communication::ReceivedControl<model::TaskInfo>{
                                                               task, Clock::now()});
    };
    auto send_command = [&](const std::string& id, Clock::time_point received_at) {
        model::Command command;
        command.commandId = id;
        command.type = model::CommandType::Route;
        bus.publish(communication::events::kCommandReceived, communication::ReceivedControl<model::Command>{
                                                              command, received_at});
    };
    auto takeover = [&](std::uint64_t execution_id, const std::string& mode, const std::string& previous) {
        bus.publish(flight_control::events::kManualTakeover,
                    flight_control::ManualTakeoverEvent{execution_id, mode, previous});
    };

    send_task("task-1");
    require(engine.starts.size() == 1, "未启动第一个任务");
    const auto old_execution = engine.starts.back();
    bus.publish(task::events::kExecutionProgress, task::TaskProgress{old_execution, "AntiUav", "Midcourse"});
    const auto received_before_cleanup = Clock::now();

    engine.gate_open = false;
    send_command("during-hold-check", Clock::now());
    require(engine.starts.size() == 1, "核对 Hold 期间启动了新指令");

    takeover(old_execution, "Hold", "Offboard");
    takeover(old_execution, "Stabilized", "Hold");
    takeover(old_execution, "Posctl", "Stabilized");
    require(engine.takeover_aborts == 1, "快速切档造成重复中止");
    const auto first_takeovers = communication.alerts_of("manual_takeover");
    require(first_takeovers.size() == 1 &&
                first_takeovers.front().detail.find("previous=Offboard value=Hold") != std::string::npos &&
                first_takeovers.front().detail.find("停止自动控制") != std::string::npos &&
                first_takeovers.front().detail.find("当前任务已中止") != std::string::npos &&
                first_takeovers.front().detail.find("taskId=task-1") != std::string::npos,
            "接管告警未保留首次前后模式、停止自动控制操作与被中止任务");
    require(logs.takeover_count() == 1 &&
                logs.text().find(first_takeovers.front().detail) != std::string::npos,
            "接管操作日志应与告警内容一致，快速切档不得重复记录");
    require(communication.states.size() == 1 && communication.states.front().status == model::TaskStatus::Aborted &&
                communication.states.front().taskId == "task-1" &&
                communication.states.front().currentNode == "Midcourse" &&
                scheduler.current_task_state().status == model::TaskStatus::None,
            "任务没有恰好上报一次 Aborted 并恢复到无任务状态");

    expect_lifecycle(communication, true, "task-1", "Aborted", "currentNode=Midcourse");
    send_command("buffered-command", received_before_cleanup);
    require(engine.starts.size() == 1, "清理前收到的指令清理后仍被执行");
    send_command("new-command", Clock::now());
    require(engine.starts.size() == 2, "清理后未启动新指令");
    const auto current_execution = engine.starts.back();
    bus.publish(task::events::kEngineResult, task::TaskEngineResult{old_execution, model::TaskStatus::Failed, "迟到"});
    takeover(old_execution, "Hold", "Offboard");

    // 新指令替换与飞控接管同时发生时，飞控先取得旧执行的中止权。
    engine.takeover_during_abort = true;
    send_command("racing-command", Clock::now());
    require(engine.starts.size() == 2, "接管抢在替换完成前发生时仍启动了新指令");
    takeover(current_execution, "Stabilized", "Mission");
    takeover(current_execution, "Posctl", "Stabilized");
    const auto all_takeovers = communication.alerts_of("manual_takeover");
    require(engine.takeover_aborts == 2 && all_takeovers.size() == 2 && communication.states.size() == 1 &&
                all_takeovers.back().detail.find("previous=Mission value=Stabilized") != std::string::npos &&
                all_takeovers.back().detail.find("停止自动控制") != std::string::npos &&
                all_takeovers.back().detail.find("当前指令已中止") != std::string::npos &&
                all_takeovers.back().detail.find("commandId=new-command") != std::string::npos,
            "独立指令的接管告警错误");
    require(logs.takeover_count() == 2 &&
                logs.text().find(all_takeovers.back().detail) != std::string::npos,
            "独立指令应只记录一次与告警一致的接管操作日志，迟到事件不得重复记录");

    expect_lifecycle(communication, false, "new-command", "Aborted", "人工接管");
    send_task("task-2");
    require(engine.starts.size() == 3 && scheduler.current_task_state().taskId == "task-2",
            "完成两次接管后未正常启动新任务");
}

void task_scheduler_arbitrates_self_check() {
    using model::CommandType;
    using model::TaskStatus;
    core::AppConfig config;
    core::EventBus bus;
    TestEngine engine;
    TestCommunication communication;
    task::TaskScheduler scheduler(bus, config, &engine);
    scheduler.set_communication(communication);
    require(scheduler.start(), "任务调度器启动失败");

    auto send_task = [&](const std::string& id) {
        model::TaskInfo request;
        request.taskId = id;
        request.taskCode = model::TaskCode::AntiUav;
        request.taskAction = model::TaskAction::Start;
        bus.publish(communication::events::kTaskInfoReceived,
                    communication::ReceivedControl<model::TaskInfo>{request, Clock::now()});
    };
    auto send_command = [&](const std::string& id, CommandType type) {
        model::Command request;
        request.commandId = id;
        request.type = type;
        bus.publish(communication::events::kCommandReceived,
                    communication::ReceivedControl<model::Command>{request, Clock::now()});
    };
    auto finish = [&](std::uint64_t id, TaskStatus status, const std::string& detail = {}) {
        bus.publish(task::events::kEngineResult, task::TaskEngineResult{id, status, detail});
    };
    auto expect_result = [&](std::size_t count, const std::string& id, bool success, const std::string& detail) {
        const auto& results = communication.self_check_results;
        require(results.size() == count, "自检结果应只上报一次");
        const auto& result = results.back();
        require(result.commandId == id && result.success == success && result.detail.find(detail) != std::string::npos,
                "自检结果的 commandId、状态或原因错误");
    };

    send_task("task-active");
    const auto aborts_before_rejection = engine.aborts;
    send_command("check-busy", CommandType::SelfCheck);
    require(engine.starts.size() == 1 && engine.aborts == aborts_before_rejection &&
                scheduler.current_task_state().taskId == "task-active",
            "自检打断了当前任务");
    expect_result(1, "check-busy", false, "正在执行");
    finish(engine.starts.back(), TaskStatus::Completed);

    send_command("check-pass", CommandType::SelfCheck);
    const auto pass_execution = engine.starts.back();
    const auto aborts_before_conflict = engine.aborts;
    send_command("check-concurrent", CommandType::SelfCheck);
    send_command("takeoff-concurrent", CommandType::Takeoff);
    send_task("task-concurrent");
    require(engine.starts.size() == 2 && engine.aborts == aborts_before_conflict &&
                engine.commands.back() == CommandType::SelfCheck &&
                scheduler.current_task_state().status == TaskStatus::None,
            "自检期间启动了其他自动控制，或错误上报了任务状态");
    expect_result(2, "check-concurrent", false, "正在执行");
    finish(pass_execution, TaskStatus::Completed);
    finish(pass_execution, TaskStatus::Failed, "旧执行的迟到结果");
    expect_result(3, "check-pass", true, "恢复上锁");

    send_command("check-failed", CommandType::SelfCheck);
    finish(engine.starts.back(), TaskStatus::Failed, "PX4 拒绝解锁");
    expect_result(4, "check-failed", false, "PX4 拒绝解锁");

    engine.self_check_start_error = "自检行为树创建失败";
    send_command("check-not-started", CommandType::SelfCheck);
    expect_result(5, "check-not-started", false, engine.self_check_start_error);
    engine.self_check_start_error.clear();

    send_command("check-replaced", CommandType::SelfCheck);
    auto old_execution = engine.starts.back();
    send_command("disarm-priority", CommandType::Disarm);
    require(engine.starts.back() != old_execution && engine.commands.back() == CommandType::Disarm,
            "显式 Disarm 未抢占自检");
    finish(old_execution, TaskStatus::Completed);
    expect_result(6, "check-replaced", false, "Disarm 指令中止，未确认恢复上锁");
    finish(engine.starts.back(), TaskStatus::Completed);

    send_command("check-stopped", CommandType::SelfCheck);
    old_execution = engine.starts.back();
    send_command("stop-motor-priority", CommandType::StopMotor);
    require(engine.starts.back() != old_execution && engine.commands.back() == CommandType::StopMotor,
            "显式 StopMotor 未抢占自检");
    finish(old_execution, TaskStatus::Completed);
    expect_result(7, "check-stopped", false, "StopMotor 指令中止，未确认恢复上锁");
    finish(engine.starts.back(), TaskStatus::Completed);

    send_command("check-takeover", CommandType::SelfCheck);
    old_execution = engine.starts.back();
    engine.gate_open = false;
    const flight_control::ManualTakeoverEvent takeover{old_execution, "Stabilized", "Hold"};
    bus.publish(flight_control::events::kManualTakeover, takeover);
    bus.publish(flight_control::events::kManualTakeover, takeover);
    finish(old_execution, TaskStatus::Completed);
    expect_result(8, "check-takeover", false, "Stabilized，自检中止，未确认恢复上锁");
    require(engine.takeover_aborts == 1 && communication.alerts_of("manual_takeover").size() == 1,
            "自检被人工接管时未保留接管告警，或重复中止");
    const auto self_check_ends = communication.alerts_of("command_finished");
    require(!self_check_ends.empty() &&
                self_check_ends.back().detail.find("commandId=check-takeover") != std::string::npos &&
                self_check_ends.back().detail.find("status=Aborted") != std::string::npos,
            "自检接管没有上报对应指令的单次中止结果");

    expect_lifecycle(communication, false, "check-pass", "Completed", "已确认解锁并恢复上锁");
    expect_lifecycle(communication, false, "check-failed", "Failed", "PX4 拒绝解锁");
    expect_lifecycle(communication, false, "check-not-started", "Failed", "自检行为树创建失败");
    expect_lifecycle(communication, false, "check-replaced", "Aborted", "Disarm 指令中止，未确认恢复上锁");
    expect_lifecycle(communication, false, "check-stopped", "Aborted", "StopMotor 指令中止，未确认恢复上锁");
    expect_lifecycle(communication, false, "check-takeover", "Aborted", "自检中止，未确认恢复上锁");
    for (const std::string id : {"check-busy", "check-concurrent"}) {
        std::size_t starts = 0;
        std::size_t rejections = 0;
        for (const auto& alert : communication.alerts) {
            if (alert.type == "command_started" && alert.detail.find("commandId=" + id + " ") != std::string::npos) {
                ++starts;
            }
            if (alert.type == "command_finished" &&
                alert.detail.find("指令未开始：commandId=" + id + " ") == 0) {
                ++rejections;
            }
        }
        require(starts == 0 && rejections == 1, "被拒绝的自检不应伪报开始，结果摘要只能上报一次");
    }
    send_task("task-after-check");
    require(scheduler.current_task_state().taskId == "task-after-check", "自检中止后不能启动新任务");
}

void task_scheduler_reports_task_failure_once() {
    core::AppConfig config;
    core::EventBus bus;
    TestEngine engine;
    TestCommunication communication;
    task::TaskScheduler scheduler(bus, config, &engine);
    scheduler.set_communication(communication);
    require(scheduler.start(), "任务调度器启动失败");

    auto send_task = [&](const std::string& task_id) {
        const model::TaskInfo request{task_id, model::TaskCode::AntiUav, "target", model::TaskAction::Start};
        bus.publish(communication::events::kTaskInfoReceived,
                    communication::ReceivedControl<model::TaskInfo>{request, Clock::now()});
    };

    send_task("task-failed");
    require(engine.starts.size() == 1, "未启动待验证的任务");
    const auto failed_execution = engine.starts.back();
    bus.publish(task::events::kExecutionProgress, task::TaskProgress{failed_execution, "AntiUav", "Midcourse"});
    const task::TaskEngineResult failure{failed_execution, model::TaskStatus::Failed, "中制导配置校验失败"};
    bus.publish(task::events::kEngineResult, failure);
    bus.publish(task::events::kEngineResult, failure);
    require(communication.states.size() == 1 && communication.states.back().taskId == "task-failed" &&
                communication.states.back().status == model::TaskStatus::Failed &&
                communication.states.back().currentBehaviorTree == "AntiUav" &&
                communication.states.back().currentNode == "Midcourse" &&
                scheduler.current_task_state().status == model::TaskStatus::None,
            "中制导失败后应上报一次完整的任务失败状态并恢复空闲");

    send_task("task-next");
    bus.publish(task::events::kEngineResult, failure);
    require(communication.states.size() == 1 && scheduler.current_task_state().taskId == "task-next" &&
                scheduler.current_task_state().status == model::TaskStatus::InProgress,
            "迟到的任务失败结果影响了新任务");

    engine.task_start_error = "AntiUav 行为树创建失败";
    send_task("task-not-started");
    require(communication.states.size() == 3 && communication.states[1].taskId == "task-next" &&
                communication.states[1].status == model::TaskStatus::Aborted &&
                communication.states.back().taskId == "task-not-started" &&
                communication.states.back().status == model::TaskStatus::Failed &&
                scheduler.current_task_state().status == model::TaskStatus::None,
            "任务启动失败时未上报任务失败状态并恢复空闲");
}


void task_scheduler_reports_complete_lifecycles() {
    using model::CommandType;
    using model::TaskStatus;
    core::AppConfig config;
    core::EventBus bus;
    TestEngine engine;
    TestCommunication communication;
    task::TaskScheduler scheduler(bus, config, &engine);
    scheduler.set_communication(communication);
    require(scheduler.start(), "任务调度器启动失败");
    auto send_task = [&](const std::string& id, model::TaskAction action = model::TaskAction::Start) {
        const model::TaskInfo request{id, model::TaskCode::AntiUav, "target-1", action};
        bus.publish(communication::events::kTaskInfoReceived,
                    communication::ReceivedControl<model::TaskInfo>{request, Clock::now()});
    };
    auto send_command = [&](const std::string& id, CommandType type) {
        model::Command command;
        command.commandId = id;
        command.type = type;
        bus.publish(communication::events::kCommandReceived,
                    communication::ReceivedControl<model::Command>{command, Clock::now()});
    };
    auto finish = [&](std::uint64_t execution, TaskStatus status, const std::string& detail = {}) {
        bus.publish(task::events::kEngineResult, task::TaskEngineResult{execution, status, detail});
    };

    send_command("route-completed", CommandType::Route);
    auto execution = engine.starts.back();
    bus.publish(task::events::kExecutionProgress, task::TaskProgress{execution, "cmd-Route", "Mission"});
    finish(execution, TaskStatus::Completed);
    finish(execution, TaskStatus::Failed, "迟到的失败结果");
    const auto route_end = expect_lifecycle(communication, false, "route-completed", "Completed", "执行完成");
    require(route_end.find("type=Route") != std::string::npos &&
                route_end.find("currentBehaviorTree=cmd-Route currentNode=Mission") != std::string::npos,
            "独立指令终态丢失类型或节点上下文");

    send_command("arm-failed", CommandType::Arm);
    execution = engine.starts.back();
    const auto failure = task::TaskEngineResult{execution, TaskStatus::Failed, "PX4 拒绝解锁：CommandDenied",
                                                 "cmd-Arm", "Arm"};
    bus.publish(task::events::kEngineResult, failure);
    bus.publish(task::events::kEngineResult, failure);
    const auto arm_end = expect_lifecycle(communication, false, "arm-failed", "Failed", "CommandDenied");
    require(arm_end.find("currentNode=Arm") != std::string::npos &&
                communication.alerts_of("command_dispatch_failed").size() == 1,
            "指令失败缺少节点或既有失败告警重复");

    send_task("task-stopped");
    const auto stopped_execution = engine.starts.back();
    send_task("task-stopped", model::TaskAction::Stop);
    finish(stopped_execution, TaskStatus::Completed);
    expect_lifecycle(communication, true, "task-stopped", "Aborted", "收到任务停止请求");
    send_task("task-replaced");
    const auto replaced_execution = engine.starts.back();
    send_task("task-new");
    finish(replaced_execution, TaskStatus::Failed, "旧执行迟到");
    expect_lifecycle(communication, true, "task-replaced", "Aborted", "被新任务替换");
    send_command("land-replacement", CommandType::Land);
    expect_lifecycle(communication, true, "task-new", "Aborted", "被新指令替换");
    send_task("task-command-replacement");
    expect_lifecycle(communication, false, "land-replacement", "Aborted", "被新任务替换");
    finish(engine.starts.back(), TaskStatus::Completed);
    expect_lifecycle(communication, true, "task-command-replacement", "Completed", "taskCode=AntiUav");

    engine.command_start_error = "指令行为树创建失败";
    send_command("dispatch-rejected", CommandType::Takeoff);
    expect_lifecycle(communication, false, "dispatch-rejected", "Failed", "指令行为树创建失败");
    engine.command_start_error.clear();
    engine.task_start_error = "AntiUav 行为树创建失败";
    send_task("task-dispatch-rejected");
    expect_lifecycle(communication, true, "task-dispatch-rejected", "Failed", "AntiUav 行为树创建失败");
    require(scheduler.current_task_state().status == TaskStatus::None, "派发失败后残留调度状态");
}

void task_scheduler_handles_synchronous_results_and_abort_races() {
    using model::CommandType;
    using model::TaskStatus;
    core::AppConfig config;
    core::EventBus bus;
    TestEngine engine;
    TestCommunication communication;
    task::TaskScheduler scheduler(bus, config, &engine);
    scheduler.set_communication(communication);
    require(scheduler.start(), "任务调度器启动失败");
    auto finish = [&](std::uint64_t id, TaskStatus status) {
        bus.publish(task::events::kEngineResult, task::TaskEngineResult{id, status, "同步执行结果"});
    };
    auto send_task = [&](const std::string& id, model::TaskAction action = model::TaskAction::Start) {
        const model::TaskInfo request{id, model::TaskCode::AntiUav, "target-1", action};
        bus.publish(communication::events::kTaskInfoReceived,
                    communication::ReceivedControl<model::TaskInfo>{request, Clock::now()});
    };
    engine.task_start_hook = [&](std::uint64_t id) { finish(id, TaskStatus::Completed); };
    send_task("task-immediate");
    engine.task_start_hook = {};
    expect_lifecycle(communication, true, "task-immediate", "Completed", "同步执行结果");
    require(scheduler.current_task_state().status == TaskStatus::None, "同步任务终态后残留 Pending/InProgress");
    engine.command_start_hook = [&](std::uint64_t id) { finish(id, TaskStatus::Failed); };
    model::Command command;
    command.commandId = "command-immediate";
    command.type = CommandType::Arm;
    bus.publish(communication::events::kCommandReceived,
                communication::ReceivedControl<model::Command>{command, Clock::now()});
    engine.command_start_hook = {};
    expect_lifecycle(communication, false, "command-immediate", "Failed", "同步执行结果");

    send_task("task-finish-before-replace");
    auto previous = engine.starts.back();
    engine.abort_hook = [&]() { finish(previous, TaskStatus::Completed); };
    send_task("task-finish-before-stop");
    expect_lifecycle(communication, true, "task-finish-before-replace", "Completed", "同步执行结果");
    previous = engine.starts.back();
    engine.abort_hook = [&]() { finish(previous, TaskStatus::Failed); };
    send_task("task-finish-before-stop", model::TaskAction::Stop);
    expect_lifecycle(communication, true, "task-finish-before-stop", "Failed", "同步执行结果");
    require(scheduler.current_task_state().status == TaskStatus::None, "停止与自然结果竞争后没有恢复空闲");
}

void task_scheduler_flushes_terminal_summaries() {
    CapturedLogs logs;
    core::AppConfig config;
    core::EventBus bus;
    TestEngine engine;
    TestCommunication communication;
    task::TaskScheduler scheduler(bus, config, &engine);
    scheduler.set_communication(communication);
    require(scheduler.start(), "任务调度器启动失败");

    struct Outcome {
        model::TaskStatus value;
        const char* text;
    };
    const Outcome outcomes[] = {{model::TaskStatus::Completed, "Completed"},
                                {model::TaskStatus::Failed, "Failed"},
                                {model::TaskStatus::Aborted, "Aborted"}};
    for (const std::string kind : {"task", "command", "self-check"}) {
        for (const auto& outcome : outcomes) {
            const auto id = kind + "-flush-" + outcome.text;
            if (kind == "task") {
                const model::TaskInfo request{id, model::TaskCode::AntiUav, "target", model::TaskAction::Start};
                bus.publish(communication::events::kTaskInfoReceived,
                            communication::ReceivedControl<model::TaskInfo>{request, Clock::now()});
            } else {
                model::Command request;
                request.commandId = id;
                request.type = kind == "self-check" ? model::CommandType::SelfCheck : model::CommandType::Route;
                bus.publish(communication::events::kCommandReceived,
                            communication::ReceivedControl<model::Command>{request, Clock::now()});
            }
            const auto flushes_before = logs.flush_count();
            const task::TaskEngineResult result{engine.starts.back(), outcome.value, "结束摘要即时刷新测试",
                                                kind == "task" ? "AntiUav" : "cmd-test", "测试节点"};
            bus.publish(task::events::kEngineResult, result);
            require(logs.flush_count() == flushes_before + 1,
                    "任务、指令和自检的每种终态应主动刷新一次结束摘要");
            const auto summary = (kind == "task" ? "任务结束：taskId=" : "指令结束：commandId=") + id;
            const auto flushed = logs.flushed_text();
            const auto summary_at = flushed.find(summary);
            require(summary_at != std::string::npos &&
                        flushed.find("status=" + std::string(outcome.text), summary_at) != std::string::npos &&
                        flushed.find("结束摘要即时刷新测试", summary_at) != std::string::npos,
                    "显式刷新必须发生在包含执行标识、终态和原因的结束摘要之后");
            bus.publish(task::events::kEngineResult, result);
            require(logs.flush_count() == flushes_before + 1, "迟到的重复终态不应重复输出或刷新结束摘要");
        }
    }
}

}  // namespace

int main() {
    try {
        task_scheduler_reports_takeover_once();
        task_scheduler_arbitrates_self_check();
        task_scheduler_reports_task_failure_once();
        task_scheduler_reports_complete_lifecycles();
        task_scheduler_handles_synchronous_results_and_abort_races();
        task_scheduler_flushes_terminal_summaries();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
