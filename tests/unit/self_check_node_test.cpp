#include <iostream>
#include <stdexcept>
#include <string>

#include "task/nodes/self_check_node.h"

namespace {

using BT::NodeStatus;
using plane::flight_control::FlightStatus;
using plane::flight_control::SelfCheckArmResult;
using plane::task::SelfCheckNode;
using namespace std::chrono_literals;

SelfCheckNode::Clock::time_point test_time;

SelfCheckNode::Clock::time_point now() noexcept { return test_time; }

class FakeFlightControl final : public plane::flight_control::FlightControlApi {
   public:
    FlightStatus status;
    SelfCheckArmResult arm_result = SelfCheckArmResult::Accepted;
    bool delay_arm_ack = false;
    bool disarm_ok = true;
    bool confirm_disarm_on_dispatch = false;
    bool can_advance = true;
    int arm_count = 0;
    int disarm_count = 0;

    SelfCheckArmResult self_check_arm(const FlightStatus&, const std::atomic<bool>*, std::string& error) override {
        ++arm_count;
        error = arm_result == SelfCheckArmResult::Rejected ? "CommandDenied" : "Timeout";
        if (delay_arm_ack) {
            test_time += 1ms;
            status.armed = true;
            status.armed_updated = now();
            test_time += 4s;
            status.ground_updated = now();
        }
        return arm_result;
    }
    bool self_check_disarm(const FlightStatus&, const std::atomic<bool>*, std::string& error) override {
        ++disarm_count;
        if (confirm_disarm_on_dispatch) {
            test_time += 1ms;
            status.armed = false;
            status.armed_updated = now();
        }
        error = "CommandDenied";
        return disarm_ok;
    }

    bool control_can_advance() const override { return can_advance; }
    bool arm(std::string&) override { throw std::runtime_error("自检绕过了解锁校验"); }
    bool disarm(std::string&) override { throw std::runtime_error("自检绕过了上锁校验"); }
    bool takeoff(float, std::string&) override { throw std::runtime_error("自检下发了起飞"); }
    bool land(std::string&) override { return false; }
    bool return_to_launch(std::string&) override { return false; }
    bool hold(std::string&) override { return false; }
    bool stop_motor(std::string&) override { throw std::runtime_error("自检下发了停桨"); }
    bool goto_waypoint(const plane::model::GeoPoint&, std::string&) override { return false; }
    bool execute_route(const std::vector<plane::model::GeoPoint>&, std::string&) override { return false; }
    bool set_ned_velocity(const plane::model::NedVelocity&, std::string&) override { return false; }
    bool set_ned_velocity_acceleration(const plane::algorithm::VelocityAccelerationCommand&, double,
                                       std::string&) override { return false; }
    bool set_attitude(const plane::flight_control::AttitudeSetpoint&, std::string&) override { return false; }
    bool get_cached_flight_control_params(plane::flight_control::FlightControlParams&, std::string&) const override {
        return false;
    }
    void clear_offboard() override {}
    std::string link_state() const override { return status.connected ? "connected" : "disconnected"; }
    FlightStatus flight_status() const override { return status; }
};

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

BT::NodeConfig node_config() {
    BT::NodeConfig config;
    config.blackboard = BT::Blackboard::create();
    return config;
}

struct Scenario {
    FakeFlightControl flight;
    std::atomic<bool> ticking{true};
    BT::NodeConfig config = node_config();
    SelfCheckNode node{"SelfCheck", config, &flight, &ticking, &now};

    Scenario() {
        test_time = SelfCheckNode::Clock::time_point{10s};
        auto& status = flight.status;
        status.connected = true;
        status.on_ground = true;
        status.ground_updated = now();
        status.mode_updated = now();
        status.armed_updated = now();
        status.flight_mode = "Posctl";
        status.link_generation = 1;
    }

    void report_armed(bool armed) {
        test_time += 1ms;
        flight.status.armed = armed;
        flight.status.armed_updated = now();
        flight.status.mode_updated = now();
    }

    std::string detail() const { return config.blackboard->get<std::string>(SelfCheckNode::kResultDetail); }
};

void success_requires_both_telemetry_updates() {
    Scenario scenario;
    require(scenario.node.executeTick() == NodeStatus::RUNNING, "首次下发解锁后等待遥测");
    require(scenario.node.executeTick() == NodeStatus::RUNNING && scenario.flight.disarm_count == 0,
            "解锁回执不能代替实际解锁遥测");
    scenario.report_armed(true);
    require(scenario.node.executeTick() == NodeStatus::RUNNING && scenario.flight.disarm_count == 1,
            "确认解锁后立即下发上锁");
    require(scenario.node.executeTick() == NodeStatus::RUNNING, "上锁回执不能代替实际恢复上锁");
    scenario.report_armed(false);
    require(scenario.node.executeTick() == NodeStatus::SUCCESS && scenario.flight.arm_count == 1 &&
                scenario.detail() == "已确认解锁并恢复上锁",
            "成功依赖两次遥测确认且只请求一次解锁");
}

void invalid_initial_state_blocks_arm() {
    auto check = [](auto configure, const char* reason) {
        Scenario scenario;
        configure(scenario.flight.status);
        require(scenario.node.executeTick() == NodeStatus::FAILURE && scenario.flight.arm_count == 0 &&
                    scenario.detail().find(reason) != std::string::npos,
                std::string("不合格初始状态拒绝且返回原因 ") + reason);
    };
    check([](FlightStatus& status) { status.armed = true; }, "已解锁");
    check([](FlightStatus& status) { status.in_air = true; }, "地面");
    check([](FlightStatus& status) { status.flight_mode = "Takeoff"; }, "飞行模式");
    check([](FlightStatus& status) { status.armed_updated -= 4s; }, "遥测");
    check([](FlightStatus& status) { status.connected = false; }, "飞控未连接");
}

void stale_arm_sample_does_not_confirm_success() {
    Scenario scenario;
    scenario.flight.delay_arm_ack = true;
    require(scenario.node.executeTick() == NodeStatus::RUNNING &&
                scenario.node.executeTick() == NodeStatus::RUNNING && scenario.flight.disarm_count == 0,
            "收到延迟飞控回执时必须重新确认解锁状态");
    scenario.report_armed(true);
    require(scenario.node.executeTick() == NodeStatus::RUNNING && scenario.flight.disarm_count == 1,
            "新的解锁遥测完成确认");
}

void arm_rejection_and_timeout_have_distinct_cleanup() {
    {
        Scenario scenario;
        scenario.flight.arm_result = SelfCheckArmResult::Rejected;
        require(scenario.node.executeTick() == NodeStatus::FAILURE &&
                    scenario.detail().find("CommandDenied") != std::string::npos &&
                    scenario.flight.disarm_count == 0,
                "明确拒绝时保留飞控返回且无需清理");
    }
    {
        Scenario scenario;
        require(scenario.node.executeTick() == NodeStatus::RUNNING, "等待解锁确认");
        test_time += 3100ms;
        scenario.flight.status.ground_updated = now();
        require(scenario.node.executeTick() == NodeStatus::RUNNING && scenario.flight.disarm_count == 1,
                "解锁确认超时后在地面执行普通上锁");
        scenario.report_armed(false);
        require(scenario.node.executeTick() == NodeStatus::FAILURE &&
                    scenario.detail() == "等待飞控确认解锁超时",
                "上锁得到确认仍保留原始自检失败原因");
    }
    {
        Scenario scenario;
        scenario.flight.arm_result = SelfCheckArmResult::Uncertain;
        scenario.flight.confirm_disarm_on_dispatch = true;
        require(scenario.node.executeTick() == NodeStatus::FAILURE &&
                    scenario.detail().find("Timeout") != std::string::npos &&
                    scenario.flight.disarm_count == 1,
                "解锁请求可能已生效时尝试上锁并保留飞控超时");
    }
}

void disarm_failure_and_lost_connection_report_unknown_state() {
    {
        Scenario scenario;
        scenario.flight.disarm_ok = false;
        require(scenario.node.executeTick() == NodeStatus::RUNNING, "等待解锁");
        scenario.report_armed(true);
        require(scenario.node.executeTick() == NodeStatus::RUNNING, "等待上锁确认");
        test_time += 3100ms;
        scenario.flight.status.ground_updated = now();
        require(scenario.node.executeTick() == NodeStatus::FAILURE &&
                    scenario.detail().find("CommandDenied") != std::string::npos &&
                    scenario.detail().find("确认上锁超时") != std::string::npos &&
                    scenario.detail().find("未确认恢复上锁") != std::string::npos,
                "上锁拒绝且无有效遥测时回报最终状态未知");
    }
    {
        Scenario scenario;
        require(scenario.node.executeTick() == NodeStatus::RUNNING, "等待解锁");
        ++scenario.flight.status.link_generation;
        require(scenario.node.executeTick() == NodeStatus::FAILURE &&
                    scenario.detail().find("飞控连接已中断或变更") != std::string::npos &&
                    scenario.flight.disarm_count == 0,
                "重连后不会向新连接补发旧自检动作");
    }
}

void shutdown_restore_respects_control_and_ground() {
    auto check = [](bool can_advance, bool on_ground, int expected_disarm) {
        Scenario scenario;
        require(scenario.node.executeTick() == NodeStatus::RUNNING, "等待解锁");
        scenario.flight.can_advance = can_advance;
        scenario.flight.status.on_ground = on_ground;
        scenario.flight.confirm_disarm_on_dispatch = true;
        scenario.ticking = false;
        scenario.node.restore_on_shutdown();
        require(scenario.flight.disarm_count == expected_disarm, "停机上锁仅在安全条件下执行");
    };
    check(true, true, 1);
    check(false, true, 0);
    check(true, false, 0);
}

}  // namespace

int main() {
    try {
        success_requires_both_telemetry_updates();
        invalid_initial_state_blocks_arm();
        stale_arm_sample_does_not_confirm_success();
        arm_rejection_and_timeout_have_distinct_cleanup();
        disarm_failure_and_lost_connection_report_unknown_state();
        shutdown_restore_respects_control_and_ground();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
