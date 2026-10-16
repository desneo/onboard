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

/// 飞控动作节点基类：分发一次、轮询完成，默认统一超时（SPEC §5.8 以实际状态判定）。
class FlightActionNode : public BT::StatefulActionNode {
   public:
    FlightActionNode(const std::string& name, const BT::NodeConfig& config,
                     plane::flight_control::FlightControlApi* flight_control, plane::state::StateStoreApi* state_store,
                     const plane::core::AppConfig* app_config)
        : BT::StatefulActionNode(name, config),
          flight_control_(flight_control),
          state_store_(state_store),
          app_config_(app_config) {}

    /// 该节点被 tick 的次数（首帧 onStart + 之后每次 onRunning）。
    int tick_count() const noexcept { return tick_count_; }

   protected:
    BT::NodeStatus onStart() override {
        ++tick_count_;
        start_ms_ = plane::core::now_ms();
        last_running_log_ms_ = start_ms_;
        if (config().blackboard) {
            config().blackboard->set("current_node", std::string(name()));
        }
        std::string error;
        if (!dispatch(error)) {
            SPDLOG_ERROR("节点 {} 分发失败: {}", name(), error);
            return BT::NodeStatus::FAILURE;
        }
        return report_status(checkFinish());
    }

    BT::NodeStatus onRunning() override {
        ++tick_count_;
        if (timeout_enabled() && plane::core::now_ms() - start_ms_ > node_timeout_ms()) {
            SPDLOG_ERROR("节点 {} 超时 ({} ms)", name(), node_timeout_ms());
            return BT::NodeStatus::FAILURE;
        }
        return report_status(checkFinish());
    }

    void onHalted() override { SPDLOG_INFO("节点 {} 已中止", name()); }

    /// 默认启用超时；需要持续等待实际完成的节点可覆写。
    virtual bool timeout_enabled() const { return true; }

    /// 节点超时；默认取全局配置，长阶段节点可覆写（M10）。
    virtual std::int64_t node_timeout_ms() const { return app_config_->bt_node_timeout_ms; }

    virtual bool dispatch(std::string& error) = 0;
    virtual BT::NodeStatus checkFinish() = 0;

    plane::flight_control::FlightControlApi* flight_control_;
    plane::state::StateStoreApi* state_store_;
    const plane::core::AppConfig* app_config_;

   private:
    /// 统一打印节点结果：RUNNING 每 5 秒一次心跳，SUCCESS/FAILURE 各打印一次。
    BT::NodeStatus report_status(BT::NodeStatus status) {
        switch (status) {
            case BT::NodeStatus::RUNNING: {
                const std::int64_t now = plane::core::now_ms();
                if (now - last_running_log_ms_ >= kRunningLogIntervalMs) {
                    last_running_log_ms_ = now;
                    SPDLOG_INFO("节点 {} 运行中 (已运行 {} ms, tick={})", name(), now - start_ms_, tick_count_);
                }
                break;
            }
            case BT::NodeStatus::SUCCESS:
                SPDLOG_INFO("节点 {} 执行成功 (tick={})", name(), tick_count_);
                break;
            case BT::NodeStatus::FAILURE:
                SPDLOG_ERROR("节点 {} 执行失败 (tick={})", name(), tick_count_);
                break;
            default:
                break;
        }
        return status;
    }

    static constexpr std::int64_t kRunningLogIntervalMs = 5000;
    std::int64_t start_ms_ = 0;
    std::int64_t last_running_log_ms_ = 0;
    int tick_count_ = 0;
};

}  // namespace plane::task
