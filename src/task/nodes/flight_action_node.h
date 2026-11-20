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

    static constexpr char kFailureDetail[] = "failure_detail";

   protected:
    BT::NodeStatus onStart() override {
        dispatched_ = false;
        failure_detail_.clear();
        if (config().blackboard) config().blackboard->set(kFailureDetail, std::string{});
        return start_action();
    }

    BT::NodeStatus onRunning() override {
        if (!flight_control_->control_can_advance()) {
            return BT::NodeStatus::RUNNING;
        }
        if (!dispatched_) {
            return start_action();
        }
        ++tick_count_;
        if (timeout_enabled() && plane::core::now_ms() - start_ms_ > node_timeout_ms()) {
            record_failure("节点 " + name() + " 超时（" + std::to_string(node_timeout_ms()) + " 毫秒）");
            SPDLOG_ERROR("{}", failure_detail_);
            onTimeout();
            return BT::NodeStatus::FAILURE;
        }
        return report_status(checkFinish());
    }

    void onHalted() override { SPDLOG_INFO("节点 {} 已中止", name()); }

    /// 默认启用超时；需要持续等待实际完成的节点可覆写。
    virtual bool timeout_enabled() const { return true; }

    /// 节点超时时的资源清理钩子。
    virtual void onTimeout() {}

    /// 节点超时；默认取全局配置，长阶段节点可覆写（M10）。
    virtual std::int64_t node_timeout_ms() const { return app_config_->bt_node_timeout_ms; }

    virtual bool dispatch(std::string& error) = 0;
    virtual BT::NodeStatus checkFinish() = 0;

    /// 保存业务根因，供执行引擎在失败时上报；日志不承担跨模块传递详情。
    void record_failure(const std::string& detail) {
        failure_detail_ = detail;
        if (config().blackboard) config().blackboard->set(kFailureDetail, detail);
    }

    plane::flight_control::FlightControlApi* flight_control_;
    plane::state::StateStoreApi* state_store_;
    const plane::core::AppConfig* app_config_;

   private:
    BT::NodeStatus start_action() {
        if (!flight_control_->control_can_advance()) {
            return BT::NodeStatus::RUNNING;
        }
        ++tick_count_;
        start_ms_ = plane::core::now_ms();
        last_running_log_ms_ = start_ms_;
        if (config().blackboard) {
            config().blackboard->set("current_node", std::string(name()));
        }
        std::string error;
        dispatched_ = dispatch(error);
        if (!flight_control_->control_can_advance()) {
            return BT::NodeStatus::RUNNING;
        }
        if (!dispatched_) {
            record_failure("节点 " + name() + " 分发失败：" + (error.empty() ? "未提供原因" : error));
            SPDLOG_ERROR("{}", failure_detail_);
            return BT::NodeStatus::FAILURE;
        }
        return report_status(checkFinish());
    }

    /// 统一打印节点结果：RUNNING 每 5 秒一次心跳，SUCCESS/FAILURE 各打印一次。
    BT::NodeStatus report_status(BT::NodeStatus status) {
        switch (status) {
            case BT::NodeStatus::RUNNING: {
                const std::int64_t now = plane::core::now_ms();
                if (now - last_running_log_ms_ >= kRunningLogIntervalMs) {
                    last_running_log_ms_ = now;
                    SPDLOG_INFO("节点 {} 运行中（elapsed_ms={} tick_count_={}）", name(), now - start_ms_, tick_count_);
                }
                break;
            }
            case BT::NodeStatus::SUCCESS:
                SPDLOG_INFO("节点 {} 执行成功（tick_count_={}）", name(), tick_count_);
                break;
            case BT::NodeStatus::FAILURE:
                if (failure_detail_.empty()) record_failure("节点 " + name() + " 执行失败");
                SPDLOG_ERROR("节点 {} 执行失败（tick_count_={}）", name(), tick_count_);
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
    bool dispatched_ = false;
    std::string failure_detail_;
};

}  // namespace plane::task
