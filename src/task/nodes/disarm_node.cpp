#include "task/nodes/disarm_node.h"

namespace plane::task {

bool DisarmNode::dispatch(std::string& error) { return flight_control_->disarm(error); }

BT::NodeStatus DisarmNode::check() {
    return flight_control_->flight_status().armed ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
}

}  // namespace plane::task
