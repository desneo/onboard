#include "task/nodes/land_node.h"

namespace plane::task {

bool LandNode::dispatch(std::string& error) { return flight_control_->land(error); }

BT::NodeStatus LandNode::checkFinish() {
    return flight_control_->flight_status().in_air ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
}

}  // namespace plane::task
