#include "task/nodes/midcourse_node.h"

#include <spdlog/spdlog.h>

namespace plane::task {

bool MidcourseNode::dispatch(std::string& error) {
    return controller_->begin(context_->targetTrackId, error);
}

BT::NodeStatus MidcourseNode::checkFinish() {
    std::string error;
    if (controller_->tick(error) == MidcourseController::TickResult::Failure) {
        SPDLOG_ERROR("中段制导失败: {}", error);
        controller_->stop();
        flight_control_->clear_offboard();
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

void MidcourseNode::onHalted() {
    FlightActionNode::onHalted();
    controller_->stop();
    flight_control_->clear_offboard();
}

bool MidcourseNode::timeout_enabled() const { return false; }

}  // namespace plane::task
