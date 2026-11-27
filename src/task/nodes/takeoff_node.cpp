#include "task/nodes/takeoff_node.h"

#include <spdlog/spdlog.h>

namespace plane::task {

bool TakeoffNode::dispatch(std::string& error) {
    takeoff_mode_seen_ = false;
    target_relative_height_m_ = static_cast<float>(system_param_->current().safeHeight_m);
    if (command_context_ != nullptr && command_context_->takeoffHeight_m > 0.0F) {
        target_relative_height_m_ = command_context_->takeoffHeight_m;
    }
    return flight_control_->takeoff(target_relative_height_m_, error);
}

BT::NodeStatus TakeoffNode::checkFinish() {
    const auto status = flight_control_->flight_status();
    if (status.flight_mode == "Takeoff") {
        takeoff_mode_seen_ = true;
    }
    if (!takeoff_mode_seen_ || status.flight_mode != "Hold" ||
        !status.in_air || status.on_ground || !status.landed_in_air) {
        return BT::NodeStatus::RUNNING;
    }
    const auto state = state_store_->own_state();
    if (!log_position_progress_) {
        SPDLOG_INFO("起飞完成: geoPos.height={:.1f} target_relative_height_m_={:.1f} flight_mode={}",
                    state.geoPos.height, target_relative_height_m_, status.flight_mode);
    } else if (has_loggable_altitude(state)) {
        SPDLOG_INFO("起飞完成，geoPos.height={:.1f} geoPos.altitude={:.1f} target_relative_height_m_={:.1f} flight_mode={}",
                    state.geoPos.height, state.geoPos.altitude, target_relative_height_m_, status.flight_mode);
    } else {
        SPDLOG_INFO("起飞完成，位置遥测不可用，target_relative_height_m_={:.1f} flight_mode={}",
                    target_relative_height_m_, status.flight_mode);
    }
    return BT::NodeStatus::SUCCESS;
}

void TakeoffNode::log_running_progress() const {
    if (!log_position_progress_) return;

    const auto state = state_store_->own_state();
    if (has_loggable_altitude(state)) {
        SPDLOG_INFO("节点 {} 运行中，geoPos.height={:.1f} geoPos.altitude={:.1f}", name(), state.geoPos.height,
                    state.geoPos.altitude);
    } else {
        SPDLOG_INFO("节点 {} 运行中，位置遥测不可用", name());
    }
}

}  // namespace plane::task
