#include "task/nodes/guide_node.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <string>

namespace plane::task {
namespace {

std::int64_t clock_now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace

bool GuideNode::dispatch(std::string& error) {
    if (!perception_->start_guidance(error)) {
        return false;
    }
    spdlog::info("图像导引已启动");
    return true;
}

BT::NodeStatus GuideNode::check() {
    const std::string link = flight_control_->link_state();
    if (link != "connected") {
        spdlog::error("导引阶段飞控连接异常: {}", link);
        return BT::NodeStatus::FAILURE;
    }
    const auto status = flight_control_->flight_status();
    const auto own = state_store_->own_state();
    if (!status.in_air || !status.armed) {
        spdlog::info("检测到落地/碰撞，判定撞击完成");
        return BT::NodeStatus::SUCCESS;
    }
    if (own.geoPos.height <= static_cast<float>(app_config_->suicide_impact_height_m)) {
        spdlog::info("检测到高度异常 ({} m)，判定撞击完成", own.geoPos.height);
        return BT::NodeStatus::SUCCESS;
    }
    if (own.vd_m_s >= static_cast<float>(app_config_->suicide_impact_vd_m_s)) {
        spdlog::info("检测到下坠速度异常 ({} m/s)，判定撞击完成", own.vd_m_s);
        return BT::NodeStatus::SUCCESS;
    }
    const plane::target::TrackingState tracking = target_store_->tracking_state();
    const std::int64_t now = clock_now_ms();
    if (!tracking.valid || tracking.lost) {
        if (lost_since_ms_ == 0) {
            lost_since_ms_ = now;
            spdlog::warn("导引阶段跟踪丢失，等待恢复");
        }
        if (now - lost_since_ms_ >= app_config_->suicide_track_lost_timeout_ms) {
            spdlog::error("导引阶段跟踪丢失超过 {} ms，任务失败", app_config_->suicide_track_lost_timeout_ms);
            return BT::NodeStatus::FAILURE;
        }
    } else {
        lost_since_ms_ = 0;
    }
    return BT::NodeStatus::RUNNING;
}

void GuideNode::onHalted() {
    FlightActionNode::onHalted();
    perception_->stop_guidance();
}

std::int64_t GuideNode::node_timeout_ms() const { return app_config_->suicide_phase_timeout_ms; }

}  // namespace plane::task
