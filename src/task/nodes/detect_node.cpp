#include "task/nodes/detect_node.h"

#include <spdlog/spdlog.h>

#include <opencv2/core.hpp>
#include <vector>

namespace plane::task {

bool DetectNode::dispatch(std::string& error) {
    if (!perception_->start_detection(error)) {
        return false;
    }
    SPDLOG_INFO("目标检测已启动");
    return true;
}

BT::NodeStatus DetectNode::check() {
    plane::model::GcsTarget gcs_target;
    if (controller_.resolve_target(context_->targetTrackId, gcs_target)) {
        std::string approach_error;
        if (!controller_.approach(gcs_target, app_config_->suicide_approach_speed_m_s, approach_error)) {
            SPDLOG_ERROR("检测阶段逼近速度下发失败: {}", approach_error);
            return BT::NodeStatus::FAILURE;
        }
    }
    const auto targets = target_store_->vision_targets();
    if (targets.empty()) {
        return BT::NodeStatus::RUNNING;
    }
    SPDLOG_INFO("检测到目标: {} 个", targets.size());
    return BT::NodeStatus::SUCCESS;
}

void DetectNode::onHalted() {
    FlightActionNode::onHalted();
    perception_->stop_detection();
}

std::int64_t DetectNode::node_timeout_ms() const { return app_config_->suicide_phase_timeout_ms; }

}  // namespace plane::task
