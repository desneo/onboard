#pragma once

#include "core/system_param_api.h"
#include "task/command_context.h"
#include "task/nodes/flight_action_node.h"

namespace plane::task {

class TakeoffNode final : public FlightActionNode {
   public:
    TakeoffNode(const std::string& name, const BT::NodeConfig& config,
                plane::flight_control::FlightControlApi* flight_control, plane::state::StateStoreApi* state_store,
                const plane::core::AppConfig* app_config, const plane::core::SystemParamApi* system_param,
                const CommandContext* command_context = nullptr, bool log_position_progress = false)
        : FlightActionNode(name, config, flight_control, state_store, app_config),
          system_param_(system_param),
          command_context_(command_context),
          log_position_progress_(log_position_progress) {}

    static BT::PortsList providedPorts() { return {}; }

   protected:
    bool dispatch(std::string& error) override;
    BT::NodeStatus checkFinish() override;
    void log_running_progress() const override;

   private:
    const plane::core::SystemParamApi* system_param_;
    const CommandContext* command_context_;
    const bool log_position_progress_;
    float target_relative_height_m_ = 0.0F;
    bool takeoff_mode_seen_ = false;
};

}  // namespace plane::task
