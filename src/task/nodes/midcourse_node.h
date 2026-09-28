#pragma once

#include "task/nodes/approach_controller.h"
#include "task/nodes/flight_action_node.h"
#include "task/suicide_task_context.h"

namespace plane::task {

/// 中段制导：Offboard 速度逼近最新 GCS 目标，进入检测距离即完成（SPEC §6-3）。
class MidcourseNode final : public FlightActionNode {
   public:
    MidcourseNode(const std::string& name, const BT::NodeConfig& config,
                  plane::flight_control::FlightControlApi* flight_control, plane::state::StateStoreApi* state_store,
                  plane::target::TargetStoreApi* target_store, const plane::core::AppConfig* app_config,
                  const SuicideTaskContext* context)
        : FlightActionNode(name, config, flight_control, state_store, app_config),
          controller_(flight_control, state_store, target_store, app_config),
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
    const plane::core::AppConfig* app_config_;
    const SuicideTaskContext* context_;
};

}  // namespace plane::task
