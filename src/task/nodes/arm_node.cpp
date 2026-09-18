#include "task/nodes/arm_node.h"

namespace plane::task {

bool ArmNode::dispatch(std::string& error) { return flight_control_->arm(error); }

BT::NodeStatus ArmNode::check() {
    return flight_control_->flight_status().armed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

}  // namespace plane::task
