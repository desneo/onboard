#pragma once

#include "task/nodes/flight_action_node.h"

namespace plane::task {

class DisarmNode final : public FlightActionNode {
   public:
    using FlightActionNode::FlightActionNode;
    static BT::PortsList providedPorts() { return {}; }

   protected:
    bool dispatch(std::string& error) override;
    BT::NodeStatus check() override;
};

}  // namespace plane::task
