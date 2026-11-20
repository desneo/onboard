#pragma once

#include "task/nodes/flight_action_node.h"
#include "task/nodes/midcourse_controller.h"
#include "task/suicide_task_context.h"

namespace plane::task {

/// 中制导：持续执行聚合算法闭环和 Offboard，直到失败或被外部中止。
class MidcourseNode final : public FlightActionNode {
   public:
    MidcourseNode(const std::string& name, const BT::NodeConfig& config,
                  plane::flight_control::FlightControlApi* flight_control, plane::state::StateStoreApi* state_store,
                  const plane::core::AppConfig* app_config, const SuicideTaskContext* context,
                  MidcourseController* controller)
        : FlightActionNode(name, config, flight_control, state_store, app_config),
          context_(context),
          controller_(controller) {}

    static BT::PortsList providedPorts() { return {}; }

   protected:
    bool dispatch(std::string& error) override;
    BT::NodeStatus checkFinish() override;
    void onHalted() override;
    bool timeout_enabled() const override;

   private:
    const SuicideTaskContext* context_;
    MidcourseController* controller_;
};

}  // namespace plane::task
