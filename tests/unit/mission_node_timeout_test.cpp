#include <spdlog/spdlog.h>

#include <cstdint>
#include <stdexcept>
#include <string>

#include "task/nodes/mission_node.h"

namespace {

std::int64_t test_now_ms = 1000;

class FakeFlightControl final : public plane::flight_control::FlightControlApi {
   public:
    plane::flight_control::FlightStatus status;
    bool dispatch_ok = true;
    int route_calls = 0;
    int waypoint_calls = 0;

    bool arm(std::string&) override { return false; }
    bool disarm(std::string&) override { return false; }
    bool takeoff(float, std::string&) override { return false; }
    bool land(std::string&) override { return false; }
    bool return_to_launch(std::string&) override { return false; }
    bool hold(std::string&) override { return false; }
    bool stop_motor(std::string&) override { return false; }
    bool goto_waypoint(const plane::model::GeoPoint&, std::string&) override {
        ++waypoint_calls;
        return dispatch_ok;
    }
    bool execute_route(const std::vector<plane::model::GeoPoint>&, std::string&) override {
        ++route_calls;
        return dispatch_ok;
    }
    bool set_ned_velocity(const plane::model::NedVelocity&, std::string&) override { return false; }
    bool set_attitude(const plane::flight_control::AttitudeSetpoint&, std::string&) override { return false; }
    bool get_cached_flight_control_params(plane::flight_control::FlightControlParams&, std::string&) const override {
        return false;
    }
    void clear_offboard() override {}
    std::string link_state() const override { return "connected"; }
    plane::flight_control::FlightStatus flight_status() const override { return status; }
};

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void route_waits_until_finished() {
    test_now_ms = 1000;
    plane::core::AppConfig config;
    config.bt_node_timeout_ms = 30000;
    plane::task::CommandContext context;
    context.type = plane::model::CommandType::Route;
    context.waypoints.resize(3);
    FakeFlightControl flight;
    plane::task::MissionNode node("Mission", BT::NodeConfig{}, &flight, nullptr, &config, &context);

    require(node.executeTick() == BT::NodeStatus::RUNNING, "航线等待首次进度");
    test_now_ms += 31000;
    require(node.executeTick() == BT::NodeStatus::RUNNING, "航线等待首次进度时不得超时");
    flight.status.mission_total = 3;
    flight.status.mission_current = 1;
    require(node.executeTick() == BT::NodeStatus::RUNNING, "航线超过普通节点超时后继续飞行");
    test_now_ms += 24 * 60 * 60 * 1000;
    require(node.executeTick() == BT::NodeStatus::RUNNING, "航线没有固定执行时限");
    flight.status.mission_current = 3;
    require(node.executeTick() == BT::NodeStatus::SUCCESS, "实际任务进度完成后航线成功");
    require(flight.route_calls == 1, "等待过程中只下发一次航线");
}

void waypoint_keeps_timeout() {
    test_now_ms = 1000;
    plane::core::AppConfig config;
    config.bt_node_timeout_ms = 30000;
    plane::task::CommandContext context;
    context.type = plane::model::CommandType::Waypoint;
    context.waypoints.resize(1);
    FakeFlightControl flight;
    flight.status.mission_total = 1;
    plane::task::MissionNode node("Mission", BT::NodeConfig{}, &flight, nullptr, &config, &context);

    require(node.executeTick() == BT::NodeStatus::RUNNING, "航点下发后等待完成");
    test_now_ms += 29000;
    require(node.executeTick() == BT::NodeStatus::RUNNING, "航点在超时前保持运行");
    test_now_ms += 2000;
    require(node.executeTick() == BT::NodeStatus::FAILURE, "航点仍遵守普通节点超时");
    require(flight.waypoint_calls == 1 && flight.route_calls == 0, "单航点分发保持不变");
}

void route_can_be_halted() {
    test_now_ms = 1000;
    plane::core::AppConfig config;
    plane::task::CommandContext context;
    context.type = plane::model::CommandType::Route;
    context.waypoints.resize(2);
    FakeFlightControl flight;
    flight.status.mission_total = 2;
    plane::task::MissionNode node("Mission", BT::NodeConfig{}, &flight, nullptr, &config, &context);

    require(node.executeTick() == BT::NodeStatus::RUNNING, "航线启动");
    test_now_ms += 60000;
    require(node.executeTick() == BT::NodeStatus::RUNNING, "长航线继续运行");
    node.haltNode();
    require(node.status() == BT::NodeStatus::IDLE, "无超时航线仍支持中止");
    require(node.executeTick() == BT::NodeStatus::RUNNING && flight.route_calls == 2, "中止后可重新启动航线");
}

void route_dispatch_failure_is_preserved() {
    plane::core::AppConfig config;
    plane::task::CommandContext context;
    context.type = plane::model::CommandType::Route;
    context.waypoints.resize(1);
    FakeFlightControl flight;
    flight.dispatch_ok = false;
    plane::task::MissionNode node("Mission", BT::NodeConfig{}, &flight, nullptr, &config, &context);
    require(node.executeTick() == BT::NodeStatus::FAILURE, "取消超时后仍报告飞控分发失败");
}

}  // namespace

// 测试目标替换时间源，验证长时间运行而无需真实等待或连接飞控。
namespace plane::core {
std::int64_t now_ms() { return test_now_ms; }
}  // namespace plane::core

int main() {
    try {
        route_waits_until_finished();
        waypoint_keeps_timeout();
        route_can_be_halted();
        route_dispatch_failure_is_preserved();
    } catch (const std::exception& error) {
        SPDLOG_ERROR("航线超时回归测试失败: {}", error.what());
        return 1;
    }
    SPDLOG_INFO("航线超时回归测试通过");
    return 0;
}
