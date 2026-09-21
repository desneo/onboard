#include "task/nodes/detect_node.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <opencv2/core.hpp>
#include <vector>

namespace plane::task {
namespace {

double center_distance_squared(const plane::model::DetectTarget& target, double center_x, double center_y) {
    const double dx = target.bbox.x + target.bbox.width / 2.0 - center_x;
    const double dy = target.bbox.y + target.bbox.height / 2.0 - center_y;
    return dx * dx + dy * dy;
}

plane::model::DetectTarget select_target(const std::vector<plane::model::DetectTarget>& targets, const cv::Mat* frame,
                                         const std::string& rule) {
    if (rule == "first") {
        return targets.front();
    }
    if (rule == "highest_confidence") {
        return *std::max_element(targets.begin(), targets.end(),
                                 [](const auto& a, const auto& b) { return a.confidence < b.confidence; });
    }
    if (frame == nullptr || frame->empty()) {
        return targets.front();
    }
    const double center_x = frame->cols / 2.0;
    const double center_y = frame->rows / 2.0;
    return *std::min_element(targets.begin(), targets.end(), [&](const auto& a, const auto& b) {
        return center_distance_squared(a, center_x, center_y) < center_distance_squared(b, center_x, center_y);
    });
}

}  // namespace

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
    cv::Mat frame;
    const bool has_frame = perception_->latest_frame(frame);
    const plane::model::DetectTarget selected =
        select_target(targets, has_frame ? &frame : nullptr, app_config_->suicide_target_select_rule);
    std::string error;
    if (!perception_->select_target(selected, error)) {
        SPDLOG_ERROR("建立跟踪失败: {}", error);
        return BT::NodeStatus::FAILURE;
    }
    SPDLOG_INFO("已选定跟踪目标: type={}, confidence={:.2f}", selected.type, selected.confidence);
    return BT::NodeStatus::SUCCESS;
}

void DetectNode::onHalted() {
    FlightActionNode::onHalted();
    perception_->stop_detection();
}

std::int64_t DetectNode::node_timeout_ms() const { return app_config_->suicide_phase_timeout_ms; }

}  // namespace plane::task
