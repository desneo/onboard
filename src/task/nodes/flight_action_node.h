#pragma once

#include <behaviortree_cpp/action_node.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <cstdint>
#include <string>

#include "core/config.h"
#include "flight_control/flight_control_api.h"
#include "state/state_store_api.h"

namespace plane::task {

/// 飞控动作节点基类：分发一次、轮询完成、统一超时（SPEC §5.8 以实际状态判定）。
class FlightActionNode : public BT::StatefulActionNode {
   public:
    FlightActionNode(const std::string& name, const BT::NodeConfig& config,
                     plane::flight_control::FlightControlApi* flight_control, plane::state::StateStoreApi* state_store,
                     const plane::core::AppConfig* app_config)
        : BT::StatefulActionNode(name, config),
          flight_control_(flight_control),
          state_store_(state_store),
          app_config_(app_config) {}

   protected:
    BT::NodeStatus onStart() override {
        start_ms_ = now_ms();
        if (config().blackboard) {
            config().blackboard->set("current_node", std::string(name()));
        }
        std::string error;
        if (!dispatch(error)) {
            spdlog::error("节点 {} 分发失败: {}", name(), error);
            return BT::NodeStatus::FAILURE;
        }
        return check();
    }

    BT::NodeStatus onRunning() override {
        if (now_ms() - start_ms_ > app_config_->bt_node_timeout_ms) {
            spdlog::error("节点 {} 超时 ({} ms)", name(), app_config_->bt_node_timeout_ms);
            return BT::NodeStatus::FAILURE;
        }
        return check();
    }

    void onHalted() override { spdlog::info("节点 {} 已中止", name()); }

    virtual bool dispatch(std::string& error) = 0;
    virtual BT::NodeStatus check() = 0;

    plane::flight_control::FlightControlApi* flight_control_;
    plane::state::StateStoreApi* state_store_;
    const plane::core::AppConfig* app_config_;

   private:
    std::int64_t now_ms() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }
    std::int64_t start_ms_ = 0;
};

}  // namespace plane::task
