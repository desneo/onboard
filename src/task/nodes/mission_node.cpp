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

}  // namespace plane::task
