#pragma once

#include "perception/perception_api.h"
#include "target/target_store_api.h"
#include "task/nodes/approach_controller.h"
#include "task/nodes/flight_action_node.h"
#include "task/suicide_task_context.h"

namespace plane::task {

/// 目标检测：启动检测、继续逼近，检测到目标即阶段完成（SPEC §6-4）。
class DetectNode final : public FlightActionNode {
   public:
    DetectNode(const std::string& name, const BT::NodeConfig& config,
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
    BT::NodeStatus checkFinish() override;
    void onHalted() override;
    std::int64_t node_timeout_ms() const override;

   private:
    ApproachController controller_;
    plane::perception::PerceptionApi* perception_;
    plane::target::TargetStoreApi* target_store_;
    const plane::core::AppConfig* app_config_;
    const SuicideTaskContext* context_;
};

}  // namespace plane::task
