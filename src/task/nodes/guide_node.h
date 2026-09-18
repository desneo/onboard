#pragma once

#include <cstdint>

#include "perception/perception_api.h"
#include "target/target_store_api.h"
#include "task/nodes/flight_action_node.h"

namespace plane::task {

/// 导引撞击：启动导引算法，按遥测异常判定撞击完成（SPEC §6-6）。
class GuideNode final : public FlightActionNode {
   public:
    GuideNode(const std::string& name, const BT::NodeConfig& config,
              plane::flight_control::FlightControlApi* flight_control, plane::state::StateStoreApi* state_store,
              plane::perception::PerceptionApi* perception, plane::target::TargetStoreApi* target_store,
              const plane::core::AppConfig* app_config)
        : FlightActionNode(name, config, flight_control, state_store, app_config),
          perception_(perception),
          target_store_(target_store),
          app_config_(app_config) {}

    static BT::PortsList providedPorts() { return {}; }

   protected:
    bool dispatch(std::string& error) override;
    BT::NodeStatus check() override;
    void onHalted() override;
    std::int64_t node_timeout_ms() const override;

   private:
    plane::perception::PerceptionApi* perception_;
    plane::target::TargetStoreApi* target_store_;
    const plane::core::AppConfig* app_config_;
    std::int64_t lost_since_ms_ = 0;
};

}  // namespace plane::task
