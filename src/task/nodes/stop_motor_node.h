#pragma once

#include "task/nodes/flight_action_node.h"

namespace plane::task {

class StopMotorNode final : public FlightActionNode {
   public:
    using FlightActionNode::FlightActionNode;
    static BT::PortsList providedPorts() { return {}; }

   protected:
    bool dispatch(std::string& error) override;
    BT::NodeStatus checkFinish() override;
};

}  // namespace plane::task
