#include "task/nodes/adjust_height_node.h"

#include <spdlog/spdlog.h>

#include <cmath>

namespace plane::task {

bool AdjustHeightNode::dispatch(std::string& error) {
    already_at_target_ = false;
    temporary_mission_active_ = false;
    mission_mode_seen_ = false;
    if (command_context_ == nullptr || command_context_->waypoints.empty()) {
        error = "航线首航点为空";
        return false;
    }
    target_relative_height_m_ = command_context_->waypoints.front().height;
    if (target_relative_height_m_ <= 0.0F) {
        error = "首航点相对高度必须大于 0";
        return false;
    }
    const auto state = state_store_->own_state();
    const float error_m = target_relative_height_m_ - state.geoPos.height;
    const float tolerance_m = static_cast<float>(app_config_->bt_height_adjust_tolerance_m);
    if (std::fabs(error_m) <= tolerance_m) {
        already_at_target_ = true;
        SPDLOG_INFO("相对高度已在容差内: geoPos.height={:.1f} target_relative_height_m_={:.1f}", state.geoPos.height,
                    target_relative_height_m_);
        return true;
    }
    if (!flight_control_->flight_status().in_air) {
        error = "飞机未起飞，无法调整高度";
        return false;
    }

    plane::model::GeoPoint temporary_waypoint = state.geoPos;
    temporary_waypoint.height = target_relative_height_m_;
    SPDLOG_INFO(
        "通过临时航点调整相对高度: latitude={:.7f} longitude={:.7f} geoPos.height={:.1f} "
        "target_relative_height_m_={:.1f} error_m={:.1f}",
        temporary_waypoint.latitude, temporary_waypoint.longitude, state.geoPos.height, target_relative_height_m_,
        error_m);
    if (!flight_control_->goto_waypoint(temporary_waypoint, error)) {
        return false;
    }
    temporary_mission_active_ = true;
    return true;
}

BT::NodeStatus AdjustHeightNode::checkFinish() {
    if (already_at_target_) {
        return BT::NodeStatus::SUCCESS;
    }

    const auto state = state_store_->own_state();
    if (state.flightMode == "Mission") {
        mission_mode_seen_ = true;
    }
    const float error_m = target_relative_height_m_ - state.geoPos.height;
    const float tolerance_m = static_cast<float>(app_config_->bt_height_adjust_tolerance_m);
    if (!flight_control_->flight_status().in_air) {
        SPDLOG_ERROR("调整高度期间飞机离地状态丢失");
        stop_temporary_mission();
        return BT::NodeStatus::FAILURE;
    }

    const plane::flight_control::FlightStatus status = flight_control_->flight_status();
    const bool mission_finished = status.mission_total > 0 && status.mission_current >= status.mission_total;
    const bool height_reached = std::fabs(error_m) <= tolerance_m;
    if (!mission_mode_seen_ || !mission_finished || !height_reached || state.flightMode == "Mission") {
        return BT::NodeStatus::RUNNING;
    }

    temporary_mission_active_ = false;
    SPDLOG_INFO("临时航点高度调整完成: geoPos.height={:.1f} target_relative_height_m_={:.1f} flightMode={}",
                state.geoPos.height, target_relative_height_m_, state.flightMode);
    return BT::NodeStatus::SUCCESS;
}

std::int64_t AdjustHeightNode::node_timeout_ms() const { return app_config_->bt_height_adjust_timeout_ms; }

void AdjustHeightNode::onTimeout() { stop_temporary_mission(); }

void AdjustHeightNode::onHalted() {
    FlightActionNode::onHalted();
    stop_temporary_mission();
}

void AdjustHeightNode::stop_temporary_mission() {
    if (!temporary_mission_active_) {
        return;
    }
    temporary_mission_active_ = false;
    std::string error;
    if (!flight_control_->hold(error)) {
        SPDLOG_ERROR("停止临时高度航点失败: {}", error);
    }
}

}  // namespace plane::task
