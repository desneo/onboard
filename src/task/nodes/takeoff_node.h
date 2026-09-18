#pragma once

#include "core/system_param_api.h"
#include "task/nodes/flight_action_node.h"

namespace plane::task {

class TakeoffNode final : public FlightActionNode {
   public:
    TakeoffNode(const std::string& name, const BT::NodeConfig& config,
                plane::flight_control::FlightControlApi* flight_control, plane::state::StateStoreApi* state_store,
                const plane::core::AppConfig* app_config, const plane::core::SystemParamApi* system_param)
        : FlightActionNode(name, config, flight_control, state_store, app_config), system_param_(system_param) {}

    static BT::PortsList providedPorts() { return {}; }

   protected:
    bool dispatch(std::string& error) override;
    BT::NodeStatus check() override;

   private:
    const plane::core::SystemParamApi* system_param_;
};

}  // namespace plane::task
