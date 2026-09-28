#include "task/nodes/takeoff_node.h"

namespace plane::task {

bool TakeoffNode::dispatch(std::string& error) {
    float height = static_cast<float>(system_param_->current().safeHeight_m);
    if (command_context_ != nullptr && command_context_->takeoffHeight_m > 0.0F) {
        height = command_context_->takeoffHeight_m;
    }
    return flight_control_->takeoff(height, error);
}

BT::NodeStatus TakeoffNode::checkFinish() {
    const auto status = flight_control_->flight_status();
    if (!status.in_air) {
        return BT::NodeStatus::RUNNING;
    }
    const auto target = static_cast<float>(system_param_->current().safeHeight_m);
    const auto tolerance = static_cast<float>(app_config_->bt_takeoff_tolerance_m);
    return state_store_->own_state().geoPos.height >= target - tolerance ? BT::NodeStatus::SUCCESS
                                                                         : BT::NodeStatus::RUNNING;
}

}  // namespace plane::task
