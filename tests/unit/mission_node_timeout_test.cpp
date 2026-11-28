#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include <cstdint>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "task/nodes/mission_node.h"

namespace {

std::int64_t test_now_ms = 1000;

class FakeFlightControl final : public plane::flight_control::FlightControlApi {
   public:
    plane::flight_control::FlightStatus status;
    bool dispatch_ok = true;
    std::string dispatch_error = "PX4 拒绝航线：Denied";
    int route_calls = 0;
    int waypoint_calls = 0;

    bool arm(std::string&) override { return false; }
    bool disarm(std::string&) override { return false; }
    bool takeoff(float, std::string&) override { return false; }
    bool land(std::string&) override { return false; }
    bool return_to_launch(std::string&) override { return false; }
    bool hold(std::string&) override { return false; }
    bool stop_motor(std::string&) override { return false; }
    bool goto_waypoint(const plane::model::GeoPoint&, std::string& error) override {
        ++waypoint_calls;
        if (!dispatch_ok) error = dispatch_error;
        return dispatch_ok;
    }
    bool execute_route(const std::vector<plane::model::GeoPoint>&, std::string& error) override {
        ++route_calls;
        if (!dispatch_ok) error = dispatch_error;
        return dispatch_ok;
    }
    bool set_ned_velocity(const plane::model::NedVelocity&, std::string&) override { return false; }
    bool set_ned_velocity_acceleration(const plane::algorithm::VelocityAccelerationCommand&, double,
                                       std::string&) override { return false; }
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

void run_ticks(plane::task::MissionNode& node, int count) {
    for (int i = 0; i < count; ++i) {
        require(node.executeTick() == BT::NodeStatus::RUNNING, "航线应保持运行");
    }
}

void route_waits_until_finished() {
    test_now_ms = 1000;
    plane::core::AppConfig config;
    config.bt_update_rate_hz = 20;
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
    config.bt_update_rate_hz = 20;
    config.bt_node_timeout_ms = 30000;
    plane::task::CommandContext context;
    context.type = plane::model::CommandType::Waypoint;
    context.waypoints.resize(1);
    FakeFlightControl flight;
    flight.status.mission_total = 1;
    BT::NodeConfig node_config;
    node_config.blackboard = BT::Blackboard::create();
    plane::task::MissionNode node("Mission", node_config, &flight, nullptr, &config, &context);

    require(node.executeTick() == BT::NodeStatus::RUNNING, "航点下发后等待完成");
    test_now_ms += 29000;
    require(node.executeTick() == BT::NodeStatus::RUNNING, "航点在超时前保持运行");
    test_now_ms += 2000;
    require(node.executeTick() == BT::NodeStatus::FAILURE, "航点仍遵守普通节点超时");
    require(flight.waypoint_calls == 1 && flight.route_calls == 0, "单航点分发保持不变");
    std::string detail;
    require(node_config.blackboard->get(plane::task::FlightActionNode::kFailureDetail, detail) &&
                detail == "节点 Mission 超时（30000 毫秒）", "节点超时的具体详情没有传给执行引擎");
    node.haltNode();
    flight.status.mission_current = 1;
    require(node.executeTick() == BT::NodeStatus::SUCCESS &&
                node_config.blackboard->get(plane::task::FlightActionNode::kFailureDetail, detail) && detail.empty(),
            "重新执行时没有清除前次节点失败原因");
}

void route_can_be_halted(std::ostringstream& output) {
    output.str({});
    test_now_ms = 1000;
    plane::core::AppConfig config;
    config.bt_update_rate_hz = 2;
    plane::task::CommandContext context;
    context.type = plane::model::CommandType::Route;
    context.waypoints.resize(2);
    FakeFlightControl flight;
    flight.status.mission_total = 2;
    plane::task::MissionNode node("Mission", BT::NodeConfig{}, &flight, nullptr, &config, &context);
    FakeFlightControl other_flight;
    plane::task::MissionNode other("OtherMission", BT::NodeConfig{}, &other_flight, nullptr, &config, &context);

    run_ticks(node, 1);
    test_now_ms += 60000;
    run_ticks(node, 8);
    run_ticks(other, 1);
    require(output.str().empty(), "节点独立计数，累计墙钟时间和其他节点的 tick 均不触发进度日志");
    const std::string progress = "节点 Mission 等待航点进度遥测\n";
    run_ticks(node, 1);
    require(output.str() == progress, "2 Hz 时第 10 次 tick 应输出进度日志");
    run_ticks(node, 1);
    node.haltNode();
    require(node.status() == BT::NodeStatus::IDLE, "无超时航线仍支持中止");
    output.str({});
    run_ticks(node, 9);
    require(flight.route_calls == 2 && output.str().empty(), "中止后重新启动航线，前 9 次 tick 不输出进度日志");
    run_ticks(node, 1);
    require(output.str() == progress, "重新开始后第 10 次 tick 输出进度日志");
}

void route_dispatch_failure_is_preserved() {
    plane::core::AppConfig config;
    config.bt_update_rate_hz = 20;
    plane::task::CommandContext context;
    context.type = plane::model::CommandType::Route;
    context.waypoints.resize(1);
    FakeFlightControl flight;
    flight.dispatch_ok = false;
    BT::NodeConfig node_config;
    node_config.blackboard = BT::Blackboard::create();
    plane::task::MissionNode node("Mission", node_config, &flight, nullptr, &config, &context);
    require(node.executeTick() == BT::NodeStatus::FAILURE, "取消超时后仍报告飞控分发失败");
    std::string detail;
    require(node_config.blackboard->get(plane::task::FlightActionNode::kFailureDetail, detail) &&
                detail.find("Mission") != std::string::npos && detail.find(flight.dispatch_error) != std::string::npos,
            "飞控派发失败的具体原因没有传给执行引擎");
}

void route_logs_received_waypoint_progress(std::ostringstream& output) {
    output.str({});
    test_now_ms = 1000;
    plane::core::AppConfig config;
    config.bt_update_rate_hz = 20;
    plane::task::CommandContext context;
    context.type = plane::model::CommandType::Route;
    context.waypoints.resize(100);
    FakeFlightControl flight;
    flight.status.connected = true;
    // 航线启动后可以先缓存航点总数，收到回调前的 0/100 只是初始值。
    flight.status.mission_total = 100;
    plane::task::MissionNode node("Mission", BT::NodeConfig{}, &flight, nullptr, &config, &context);

    run_ticks(node, 99);
    require(output.str().empty(), "20 Hz 时前 99 次 tick 不应输出航线进度日志");
    run_ticks(node, 1);
    require(output.str() == "节点 Mission 等待航点进度遥测\n", "第 100 次 tick 应提示等待进度遥测");

    flight.status.mission_progress_received = true;
    run_ticks(node, 100);
    require(output.str().find("航线飞行中，航点进度 1/100\n") != std::string::npos,
            "首条有效回调的零基索引 0 应显示为 1/100");

    flight.status.mission_current = 9;
    run_ticks(node, 100);
    require(output.str().find("航线飞行中，航点进度 10/100\n") != std::string::npos,
            "当前航点应以一基序号和总数显示");

    flight.status.mission_current = 99;
    run_ticks(node, 100);
    require(output.str().find("航线飞行中，航点进度 100/100\n") != std::string::npos,
            "航向最后一个航点时仍应运行并显示 100/100");

    flight.status.mission_current = 100;
    require(node.executeTick() == BT::NodeStatus::SUCCESS, "索引等于总数时航线应完成");
    const auto messages = output.str();
    require(messages.find("节点 Mission 执行成功\n") != std::string::npos &&
                messages.find("101/100") == std::string::npos,
            "航线完成时应直接记录成功");
    require(messages.find("elapsed_ms") == std::string::npos && messages.find("tick_count") == std::string::npos,
            "航线运行和完成日志应省略耗时和 tick 次数");
}

}  // namespace

// 测试目标替换时间源，验证长时间运行而无需真实等待或连接飞控。
namespace plane::core {
std::int64_t now_ms() { return test_now_ms; }
}  // namespace plane::core

int main() {
    std::ostringstream output;
    const auto previous_logger = spdlog::default_logger();
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(output);
    auto logger = std::make_shared<spdlog::logger>("mission_node_timeout", sink);
    logger->set_pattern("%v");
    logger->set_level(spdlog::level::info);
    spdlog::set_default_logger(logger);
    try {
        route_waits_until_finished();
        waypoint_keeps_timeout();
        route_can_be_halted(output);
        route_dispatch_failure_is_preserved();
        route_logs_received_waypoint_progress(output);
    } catch (const std::exception& error) {
        spdlog::set_default_logger(previous_logger);
        SPDLOG_ERROR("航线超时回归测试失败: {}", error.what());
        return 1;
    }
    spdlog::set_default_logger(previous_logger);
    SPDLOG_INFO("航线超时回归测试通过");
    return 0;
}
