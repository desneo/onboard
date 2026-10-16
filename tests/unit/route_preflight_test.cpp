#include <spdlog/spdlog.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "core/system_param_api.h"
#include "state/state_store_api.h"
#include "task/command_tree_plan.h"
#include "task/nodes/adjust_height_node.h"
#include "task/nodes/takeoff_node.h"

namespace {

std::int64_t test_now_ms = 1000;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class FakeFlightControl final : public plane::flight_control::FlightControlApi {
   public:
    plane::flight_control::FlightStatus status;
    plane::model::NedVelocity last_velocity;
    plane::model::GeoPoint last_waypoint;
    int velocity_calls = 0;
    int clear_calls = 0;
    int waypoint_calls = 0;
    int hold_calls = 0;
    float takeoff_height_m = 0.0F;

    bool arm(std::string&) override { return true; }
    bool disarm(std::string&) override { return true; }
    bool takeoff(float height_m, std::string&) override {
        takeoff_height_m = height_m;
        return true;
    }
    bool land(std::string&) override { return true; }
    bool return_to_launch(std::string&) override { return true; }
    bool hold(std::string&) override {
        ++hold_calls;
        return true;
    }
    bool stop_motor(std::string&) override { return true; }
    bool goto_waypoint(const plane::model::GeoPoint& point, std::string&) override {
        last_waypoint = point;
        ++waypoint_calls;
        status.mission_current = 0;
        status.mission_total = 1;
        return true;
    }
    bool execute_route(const std::vector<plane::model::GeoPoint>&, std::string&) override { return true; }
    bool set_ned_velocity(const plane::model::NedVelocity& velocity, std::string&) override {
        last_velocity = velocity;
        ++velocity_calls;
        return true;
    }
    void clear_ned_velocity() override { ++clear_calls; }
    std::string link_state() const override { return "connected"; }
    plane::flight_control::FlightStatus flight_status() const override { return status; }
};

class FakeStateStore final : public plane::state::StateStoreApi {
   public:
    plane::model::VehicleState state;

    plane::model::VehicleState own_state() const override { return state; }
    std::vector<plane::model::VehicleState> peer_states() const override { return {}; }
    bool peer_state(const std::string&, plane::model::VehicleState&) const override { return false; }
    plane::state::GcsLinkState gcs_link() const override { return {}; }
    void update_own_state(const plane::model::VehicleState& value) override { state = value; }
    void update_peer_state(const plane::model::VehicleState&) override {}
    void notify_station_seen() override {}
};

class FakeSystemParam final : public plane::core::SystemParamApi {
   public:
    plane::model::SystemParam value;

    plane::model::SystemParam current() const override { return value; }
};

void route_tree_depends_on_flight_state() {
    plane::flight_control::FlightStatus status;
    require(plane::task::command_node_names(plane::model::CommandType::Route, status) ==
                std::vector<std::string>({"Arm", "Takeoff", "AdjustHeight", "Mission"}),
            "未解锁且未起飞的航线节点错误");

    status.armed = true;
    require(plane::task::command_node_names(plane::model::CommandType::Route, status) ==
                std::vector<std::string>({"Takeoff", "AdjustHeight", "Mission"}),
            "已解锁但未起飞的航线节点错误");

    status.in_air = true;
    require(plane::task::command_node_names(plane::model::CommandType::Route, status) ==
                std::vector<std::string>({"AdjustHeight", "Mission"}),
            "已起飞的航线节点错误");
}

void height_adjust_uses_relative_height() {
    test_now_ms = 1000;
    plane::core::AppConfig config;
    config.bt_height_adjust_tolerance_m = 1.0;
    plane::task::CommandContext context;
    context.type = plane::model::CommandType::Route;
    context.waypoints.resize(1);
    context.waypoints.front().height = 20.0F;
    // 海拔目标刻意低于当前海拔；若误用 altitude，控制方向会与相对高度相反。
    context.waypoints.front().altitude = 90.0F;
    FakeFlightControl flight;
    flight.status.in_air = true;
    FakeStateStore states;
    states.state.geoPos.latitude = 32.217857;
    states.state.geoPos.longitude = 119.105878;
    states.state.geoPos.height = 5.0F;
    states.state.geoPos.altitude = 105.0F;
    states.state.flightMode = "Hold";
    plane::task::AdjustHeightNode node("AdjustHeight", BT::NodeConfig{}, &flight, &states, &config, &context);

    require(node.executeTick() == BT::NodeStatus::RUNNING, "相对高度调整应进入运行状态");
    require(flight.waypoint_calls == 1, "高度调整应只下发一个临时航点");
    require(flight.velocity_calls == 0, "高度调整不得使用 Offboard 速度控制");
    require(std::fabs(flight.last_waypoint.latitude - states.state.geoPos.latitude) < 0.0000001,
            "临时航点应使用当前位置纬度");
    require(std::fabs(flight.last_waypoint.longitude - states.state.geoPos.longitude) < 0.0000001,
            "临时航点应使用当前位置经度");
    require(std::fabs(flight.last_waypoint.height - 20.0F) < 0.001F, "临时航点应使用首航点相对高度");
    require(std::fabs(flight.last_waypoint.altitude - 105.0F) < 0.001F,
            "构造临时航点时不得把首航点海拔当成相对高度");

    states.state.geoPos.height = 19.2F;
    states.state.geoPos.altitude = 119.2F;
    states.state.flightMode = "Mission";
    flight.status.mission_current = 1;
    require(node.executeTick() == BT::NodeStatus::RUNNING, "临时航点完成后应等待退出 Mission");
    states.state.flightMode = "Hold";
    require(node.executeTick() == BT::NodeStatus::SUCCESS, "退出临时 Mission 后高度调整应完成");
}

void height_adjust_supports_descent_and_halt() {
    plane::core::AppConfig config;
    plane::task::CommandContext context;
    context.type = plane::model::CommandType::Route;
    context.waypoints.resize(1);
    context.waypoints.front().height = 20.0F;
    FakeFlightControl flight;
    flight.status.in_air = true;
    FakeStateStore states;
    states.state.geoPos.latitude = 32.217857;
    states.state.geoPos.longitude = 119.105878;
    states.state.geoPos.height = 30.0F;
    plane::task::AdjustHeightNode node("AdjustHeight", BT::NodeConfig{}, &flight, &states, &config, &context);

    require(node.executeTick() == BT::NodeStatus::RUNNING, "下降调整应进入运行状态");
    require(std::fabs(flight.last_waypoint.height - 20.0F) < 0.001F, "下降也应使用目标相对高度航点");
    node.haltNode();
    require(flight.hold_calls == 1, "高度调整中止后应切换 Hold 停止临时任务");
}

void height_adjust_timeout_stops_temporary_mission() {
    test_now_ms = 1000;
    plane::core::AppConfig config;
    config.bt_height_adjust_timeout_ms = 1000;
    plane::task::CommandContext context;
    context.type = plane::model::CommandType::Route;
    context.waypoints.resize(1);
    context.waypoints.front().height = 20.0F;
    FakeFlightControl flight;
    flight.status.in_air = true;
    FakeStateStore states;
    states.state.geoPos.height = 5.0F;
    plane::task::AdjustHeightNode node("AdjustHeight", BT::NodeConfig{}, &flight, &states, &config, &context);

    require(node.executeTick() == BT::NodeStatus::RUNNING, "超时测试应先进入运行状态");
    test_now_ms += 1001;
    require(node.executeTick() == BT::NodeStatus::FAILURE, "高度调整超时应失败");
    require(flight.hold_calls == 1, "高度调整超时后应切换 Hold 停止临时任务");
}

void takeoff_checks_dispatched_relative_height() {
    plane::core::AppConfig config;
    config.bt_takeoff_tolerance_m = 1.0;
    plane::task::CommandContext context;
    context.takeoffHeight_m = 20.0F;
    FakeFlightControl flight;
    flight.status.in_air = true;
    FakeStateStore states;
    states.state.geoPos.height = 12.0F;
    FakeSystemParam params;
    params.value.safeHeight_m = 10.0;
    plane::task::TakeoffNode node("Takeoff", BT::NodeConfig{}, &flight, &states, &config, &params, &context);

    require(node.executeTick() == BT::NodeStatus::RUNNING, "未达到实际起飞目标高度时不得完成");
    require(std::fabs(flight.takeoff_height_m - 20.0F) < 0.001F, "应下发上下文中的起飞高度");
    states.state.geoPos.height = 19.1F;
    require(node.executeTick() == BT::NodeStatus::SUCCESS, "达到实际起飞目标高度后应完成");
}

}  // namespace

namespace plane::core {
std::int64_t now_ms() { return test_now_ms; }
}  // namespace plane::core

int main() {
    try {
        route_tree_depends_on_flight_state();
        height_adjust_uses_relative_height();
        height_adjust_supports_descent_and_halt();
        height_adjust_timeout_stops_temporary_mission();
        takeoff_checks_dispatched_relative_height();
    } catch (const std::exception& error) {
        SPDLOG_ERROR("航线起飞与高度调整测试失败: {}", error.what());
        return 1;
    }
    SPDLOG_INFO("航线起飞与高度调整测试通过");
    return 0;
}
