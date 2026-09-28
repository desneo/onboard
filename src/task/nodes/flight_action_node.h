#pragma once

#include <behaviortree_cpp/action_node.h>
#include <spdlog/spdlog.h>

#include <cstdint>
#include <string>

#include "core/config.h"
#include "core/time_utils.h"
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
        start_ms_ = plane::core::now_ms();
        if (config().blackboard) {
            config().blackboard->set("current_node", std::string(name()));
        }
        std::string error;
        if (!dispatch(error)) {
            SPDLOG_ERROR("节点 {} 分发失败: {}", name(), error);
            return BT::NodeStatus::FAILURE;
        }
        return checkFinish();
    }

    BT::NodeStatus onRunning() override {
        if (plane::core::now_ms() - start_ms_ > node_timeout_ms()) {
            SPDLOG_ERROR("节点 {} 超时 ({} ms)", name(), node_timeout_ms());
            return BT::NodeStatus::FAILURE;
        }
        return checkFinish();
    }

    void onHalted() override { SPDLOG_INFO("节点 {} 已中止", name()); }

    /// 节点超时；默认取全局配置，长阶段节点可覆写（M10）。
    virtual std::int64_t node_timeout_ms() const { return app_config_->bt_node_timeout_ms; }

    virtual bool dispatch(std::string& error) = 0;
    virtual BT::NodeStatus checkFinish() = 0;

    plane::flight_control::FlightControlApi* flight_control_;
    plane::state::StateStoreApi* state_store_;
    const plane::core::AppConfig* app_config_;

   private:
    std::int64_t start_ms_ = 0;
};

}  // namespace plane::task
