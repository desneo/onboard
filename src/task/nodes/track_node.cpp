#include "task/nodes/track_node.h"

#include <spdlog/spdlog.h>

#include <chrono>

namespace plane::task {
namespace {

std::int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace

bool TrackNode::dispatch(std::string&) {
    spdlog::info("单目标跟踪监视开始");
    return true;
}

BT::NodeStatus TrackNode::check() {
    plane::model::GcsTarget gcs_target;
    if (controller_.resolve_target(context_->targetTrackId, gcs_target)) {
        std::string error;
        if (!controller_.approach(gcs_target, app_config_->suicide_track_approach_speed_m_s, error)) {
            spdlog::error("跟踪阶段逼近速度下发失败: {}", error);
            return BT::NodeStatus::FAILURE;
        }
    }
    const plane::target::TrackingState state = target_store_->tracking_state();
    const std::int64_t now = now_ms();
    if (!state.valid) {
        if (lost_since_ms_ == 0) {
            lost_since_ms_ = now;
        }
        if (now - lost_since_ms_ >= app_config_->suicide_track_lost_timeout_ms) {
            spdlog::error("跟踪未建立且超时");
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }
    if (state.lost) {
        stable_since_ms_ = 0;
        if (lost_since_ms_ == 0) {
            lost_since_ms_ = now;
            spdlog::warn("跟踪目标丢失，等待恢复");
        }
        if (now - lost_since_ms_ >= app_config_->suicide_track_lost_timeout_ms) {
            spdlog::error("跟踪丢失超过 {} ms，任务失败", app_config_->suicide_track_lost_timeout_ms);
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }
    lost_since_ms_ = 0;
    if (stable_since_ms_ == 0) {
        stable_since_ms_ = now;
    }
    if (now - stable_since_ms_ >= app_config_->suicide_guidance_start_stable_ms) {
        spdlog::info("跟踪稳定，进入导引阶段");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

std::int64_t TrackNode::node_timeout_ms() const { return app_config_->suicide_phase_timeout_ms; }

}  // namespace plane::task
