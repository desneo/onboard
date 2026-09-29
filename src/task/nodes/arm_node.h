#pragma once

#include "task/nodes/flight_action_node.h"

namespace plane::task {

class ArmNode final : public FlightActionNode {
   public:
    using FlightActionNode::FlightActionNode;
    static BT::PortsList providedPorts() { return {}; }

   protected:
    bool dispatch(std::string& error) override;
    BT::NodeStatus checkFinish() override;

   private:
    static constexpr std::int64_t kArmSpinUpMs = 1000;
    std::int64_t armed_at_ms_ = 0;
};

}  // namespace plane::task
