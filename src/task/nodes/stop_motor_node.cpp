#include "task/nodes/stop_motor_node.h"

namespace plane::task {

bool StopMotorNode::dispatch(std::string& error) { return flight_control_->stop_motor(error); }

BT::NodeStatus StopMotorNode::checkFinish() { return BT::NodeStatus::SUCCESS; }

}  // namespace plane::task
