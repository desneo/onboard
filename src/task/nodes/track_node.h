#pragma once

#include <cstdint>

#include "perception/perception_api.h"
#include "target/target_store_api.h"
#include "task/nodes/approach_controller.h"
#include "task/nodes/flight_action_node.h"
#include "task/suicide_task_context.h"

namespace plane::task {

/// 单目标跟踪：丢失超限则失败，稳定达到时长后进入导引（SPEC §6-5）。
class TrackNode final : public FlightActionNode {
   public:
    TrackNode(const std::string& name, const BT::NodeConfig& config,
              plane::flight_control::FlightControlApi* flight_control, plane::state::StateStoreApi* state_store,
              plane::perception::PerceptionApi* perception, plane::target::TargetStoreApi* target_store,
              const plane::core::AppConfig* app_config, const SuicideTaskContext* context)
        : FlightActionNode(name, config, flight_control, state_store, app_config),
          controller_(flight_control, state_store, target_store, app_config),
          perception_(perception),
          target_store_(target_store),
          app_config_(app_config),
          context_(context) {}

    static BT::PortsList providedPorts() { return {}; }

   protected:
    bool dispatch(std::string& error) override;
    BT::NodeStatus check() override;
    std::int64_t node_timeout_ms() const override;

   private:
    ApproachController controller_;
    plane::perception::PerceptionApi* perception_;
    plane::target::TargetStoreApi* target_store_;
    const plane::core::AppConfig* app_config_;
    const SuicideTaskContext* context_;
    std::int64_t lost_since_ms_ = 0;
    std::int64_t stable_since_ms_ = 0;
};

}  // namespace plane::task
