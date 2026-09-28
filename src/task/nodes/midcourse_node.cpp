#include "task/nodes/midcourse_node.h"

#include <spdlog/spdlog.h>

namespace plane::task {

bool MidcourseNode::dispatch(std::string& error) {
    plane::model::GcsTarget target;
    if (!controller_.resolve_target(context_->targetTrackId, target)) {
        error = context_->targetTrackId.empty() ? "无法确定 GCS 目标（targetTrackId 为空且 GCS 目标数不为 1）"
                                                : "GCS 目标不存在: " + context_->targetTrackId;
        return false;
    }
    SPDLOG_INFO("中段制导目标: trackId={}, lat={}, lon={}, height={}", target.trackId, target.geoPos.latitude,
                target.geoPos.longitude, target.geoPos.height);
    return true;
}

BT::NodeStatus MidcourseNode::checkFinish() {
    plane::model::GcsTarget target;
    if (!controller_.resolve_target(context_->targetTrackId, target)) {
        SPDLOG_ERROR("中段制导目标不可用");
        return BT::NodeStatus::FAILURE;
    }
    const double distance = controller_.horizontal_distance_m(target);
    if (distance <= app_config_->suicide_detection_start_distance_m) {
        SPDLOG_INFO("中段制导到达检测启动距离: {:.1f} m", distance);
        return BT::NodeStatus::SUCCESS;
    }
    std::string error;
    if (!controller_.approach(target, app_config_->suicide_approach_speed_m_s, error)) {
        SPDLOG_ERROR("中段制导速度下发失败 (距离 {:.1f} m): {}", distance, error);
        return BT::NodeStatus::FAILURE;
    }
    SPDLOG_DEBUG("中段制导: 距离 {:.1f} m", distance);
    return BT::NodeStatus::RUNNING;
}

void MidcourseNode::onHalted() {
    FlightActionNode::onHalted();
    flight_control_->clear_ned_velocity();
}

std::int64_t MidcourseNode::node_timeout_ms() const { return app_config_->suicide_phase_timeout_ms; }

}  // namespace plane::task
