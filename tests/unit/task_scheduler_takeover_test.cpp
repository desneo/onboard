#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

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

class TestEngine final : public task::TaskExecutionEngineApi {
   public:
    bool start(std::uint64_t execution_id, const model::TaskInfo&, std::string&) override {
        starts.push_back(execution_id);
        return true;
    }

    bool start_command(std::uint64_t execution_id, const model::Command& command, std::string& error) override {
        if (command.type == model::CommandType::SelfCheck && !self_check_start_error.empty()) {
            error = self_check_start_error;
            return false;
        }
        starts.push_back(execution_id);
        commands.push_back(command.type);
        return true;
    }

    bool abort(bool acknowledge_takeover) override {
        if (acknowledge_takeover) {
            ++takeover_aborts;
            gate_open = true;
            return true;
        }
        ++aborts;
        if (takeover_during_abort) {
            takeover_during_abort = false;
            gate_open = false;
        }
        return gate_open;
    }

    bool control_can_advance() const override { return gate_open; }

    std::vector<std::uint64_t> starts;
    std::vector<model::CommandType> commands;
    std::string self_check_start_error;
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

    std::vector<model::Alert> alerts;
    std::vector<model::TaskState> states;
    std::vector<model::SelfCheckResult> self_check_results;
};

void task_scheduler_reports_takeover_once() {
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
    auto takeover = [&](std::uint64_t execution_id, const std::string& mode) {
        bus.publish(flight_control::events::kManualTakeover, flight_control::ManualTakeoverEvent{execution_id, mode});
    };

    send_task("task-1");
    require(engine.starts.size() == 1, "未启动第一个任务");
    const auto old_execution = engine.starts.back();
    bus.publish(task::events::kExecutionProgress, task::TaskProgress{old_execution, "AntiUav", "Midcourse"});
    const auto received_before_cleanup = Clock::now();

    engine.gate_open = false;
    send_command("during-hold-check", Clock::now());
    require(engine.starts.size() == 1, "核对 Hold 期间启动了新指令");

    takeover(old_execution, "Hold");
    takeover(old_execution, "Stabilized");
    takeover(old_execution, "Posctl");
    require(engine.takeover_aborts == 1, "快速切档造成重复中止");
    require(communication.alerts.size() == 1 && communication.alerts.front().type == "manual_takeover" &&
                communication.alerts.front().detail.find("Hold") != std::string::npos &&
                communication.alerts.front().detail.find("taskId=task-1") != std::string::npos,
            "接管告警未保留首次模式与被中止任务");
    require(communication.states.size() == 1 && communication.states.front().status == model::TaskStatus::Aborted &&
                communication.states.front().taskId == "task-1" &&
                communication.states.front().currentNode == "Midcourse" &&
                scheduler.current_task_state().status == model::TaskStatus::None,
            "任务没有恰好上报一次 Aborted 并恢复到无任务状态");

    send_command("buffered-command", received_before_cleanup);
    require(engine.starts.size() == 1, "清理前收到的指令清理后仍被执行");
    send_command("new-command", Clock::now());
    require(engine.starts.size() == 2, "清理后未启动新指令");
    const auto current_execution = engine.starts.back();
    bus.publish(task::events::kEngineResult, task::TaskEngineResult{old_execution, model::TaskStatus::Failed, "迟到"});
    takeover(old_execution, "Hold");

    // 新指令替换与飞控接管同时发生时，飞控先取得旧执行的中止权。
    engine.takeover_during_abort = true;
    send_command("racing-command", Clock::now());
    require(engine.starts.size() == 2, "接管抢在替换完成前发生时仍启动了新指令");
    takeover(current_execution, "Stabilized");
    require(engine.takeover_aborts == 2 && communication.alerts.size() == 2 && communication.states.size() == 1 &&
                communication.alerts.back().type == "manual_takeover" &&
                communication.alerts.back().detail.find("commandId=new-command") != std::string::npos,
            "独立指令的接管告警错误");

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
    const flight_control::ManualTakeoverEvent takeover{old_execution, "Stabilized"};
    bus.publish(flight_control::events::kManualTakeover, takeover);
    bus.publish(flight_control::events::kManualTakeover, takeover);
    finish(old_execution, TaskStatus::Completed);
    expect_result(8, "check-takeover", false, "Stabilized，自检中止，未确认恢复上锁");
    require(engine.takeover_aborts == 1 && communication.alerts.back().type == "manual_takeover",
            "自检被人工接管时未保留接管告警，或重复中止");

    send_task("task-after-check");
    require(scheduler.current_task_state().taskId == "task-after-check", "自检中止后不能启动新任务");
}

}  // namespace

int main() {
    try {
        task_scheduler_reports_takeover_once();
        task_scheduler_arbitrates_self_check();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
