#pragma once

#include "task/command_context.h"
#include "task/nodes/flight_action_node.h"

namespace plane::task {

/// 航点/航线指令节点：下发任务并依据任务进度判定完成。
class MissionNode final : public FlightActionNode {
   public:
    MissionNode(const std::string& name, const BT::NodeConfig& config,
                plane::flight_control::FlightControlApi* flight_control, plane::state::StateStoreApi* state_store,
                const plane::core::AppConfig* app_config, const CommandContext* command_context)
        : FlightActionNode(name, config, flight_control, state_store, app_config), command_context_(command_context) {}

    static BT::PortsList providedPorts() { return {}; }

   protected:
    bool dispatch(std::string& error) override;
    BT::NodeStatus check() override;

   private:
    const CommandContext* command_context_;
};

}  // namespace plane::task
