#include "task/nodes/rtl_node.h"

namespace plane::task {

bool RtlNode::dispatch(std::string& error) { return flight_control_->return_to_launch(error); }

BT::NodeStatus RtlNode::check() {
    return flight_control_->flight_status().in_air ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
}

}  // namespace plane::task
