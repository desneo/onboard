#pragma once

#include "task/nodes/flight_action_node.h"

namespace plane::task {

class RtlNode final : public FlightActionNode {
   public:
    using FlightActionNode::FlightActionNode;
    static BT::PortsList providedPorts() { return {}; }

   protected:
    bool dispatch(std::string& error) override;
    BT::NodeStatus checkFinish() override;
    void log_running_progress() const override { log_altitude_and_mode_progress(); }
};

}  // namespace plane::task
