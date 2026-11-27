#include "task/nodes/mission_node.h"

namespace plane::task {

bool MissionNode::timeout_enabled() const {
    return command_context_ == nullptr || command_context_->type != plane::model::CommandType::Route;
}

bool MissionNode::dispatch(std::string& error) {
    if (command_context_ == nullptr || command_context_->waypoints.empty()) {
        error = "航点为空";
        return false;
    }
    if (command_context_->type == plane::model::CommandType::Route) {
        return flight_control_->execute_route(command_context_->waypoints, error);
    }
    return flight_control_->goto_waypoint(command_context_->waypoints.front(), error);
}

BT::NodeStatus MissionNode::checkFinish() {
    const plane::flight_control::FlightStatus status = flight_control_->flight_status();
    if (status.mission_total <= 0) {
        return BT::NodeStatus::RUNNING;  // 任务尚未开始（上传/启动中）
    }
    return status.mission_current >= status.mission_total ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void MissionNode::log_running_progress() const {
    const auto status = flight_control_->flight_status();
    if (!status.mission_progress_received || status.mission_total <= 0 || status.mission_current < 0) {
        SPDLOG_INFO("节点 {} 等待航点进度遥测", name());
        return;
    }
    if (status.mission_current < status.mission_total) {
        const bool is_route = command_context_->type == plane::model::CommandType::Route;
        SPDLOG_INFO("{}飞行中，航点进度 {}/{}", is_route ? "航线" : "单航点", status.mission_current + 1,
                    status.mission_total);
    }
}

}  // namespace plane::task
