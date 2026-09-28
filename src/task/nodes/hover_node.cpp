#include "task/nodes/hover_node.h"

namespace plane::task {

bool HoverNode::dispatch(std::string& error) { return flight_control_->hold(error); }

BT::NodeStatus HoverNode::checkFinish() { return BT::NodeStatus::SUCCESS; }

}  // namespace plane::task
