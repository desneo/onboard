#include "task/nodes/midcourse_node.h"

#include <spdlog/spdlog.h>

namespace plane::task {

bool MidcourseNode::dispatch(std::string& error) {
    if (controller_->begin(context_->targetTrackId, context_->targetHistoryStartRevision, error)) {
        return true;
    }
    record_failure(error);
    return false;
}

BT::NodeStatus MidcourseNode::checkFinish() {
    std::string error;
    if (controller_->tick(error) == MidcourseController::TickResult::Failure) {
        SPDLOG_ERROR("中段制导失败: {}", error);
        record_failure(error);
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
