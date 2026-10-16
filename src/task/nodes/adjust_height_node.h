#pragma once

#include "task/command_context.h"
#include "task/nodes/flight_action_node.h"

namespace plane::task {

/// 航线开始前的相对高度调整节点；通过当前经纬度的临时航点调整，只使用
/// GeoPoint.height，不使用海拔 altitude。
class AdjustHeightNode final : public FlightActionNode {
   public:
    AdjustHeightNode(const std::string& name, const BT::NodeConfig& config,
                     plane::flight_control::FlightControlApi* flight_control, plane::state::StateStoreApi* state_store,
                     const plane::core::AppConfig* app_config, const CommandContext* command_context)
        : FlightActionNode(name, config, flight_control, state_store, app_config), command_context_(command_context) {}

    static BT::PortsList providedPorts() { return {}; }

   protected:
    bool dispatch(std::string& error) override;
    BT::NodeStatus checkFinish() override;
    std::int64_t node_timeout_ms() const override;
    void onTimeout() override;
    void onHalted() override;

   private:
    void stop_temporary_mission();

    const CommandContext* command_context_;
    float target_relative_height_m_ = 0.0F;
    bool already_at_target_ = false;
    bool temporary_mission_active_ = false;
    bool mission_mode_seen_ = false;
};

}  // namespace plane::task
