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
    SPDLOG_INFO("起飞完成: geoPos.height={:.1f} target_relative_height_m_={:.1f} flight_mode={}",
                state_store_->own_state().geoPos.height, target_relative_height_m_, status.flight_mode);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace plane::task
