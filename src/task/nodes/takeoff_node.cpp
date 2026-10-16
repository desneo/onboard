#include "task/nodes/takeoff_node.h"

namespace plane::task {

bool TakeoffNode::dispatch(std::string& error) {
    target_relative_height_m_ = static_cast<float>(system_param_->current().safeHeight_m);
    if (command_context_ != nullptr && command_context_->takeoffHeight_m > 0.0F) {
        target_relative_height_m_ = command_context_->takeoffHeight_m;
    }
    return flight_control_->takeoff(target_relative_height_m_, error);
}

BT::NodeStatus TakeoffNode::checkFinish() {
    const auto status = flight_control_->flight_status();
    if (!status.in_air) {
        return BT::NodeStatus::RUNNING;
    }
    const auto tolerance = static_cast<float>(app_config_->bt_takeoff_tolerance_m);
    return state_store_->own_state().geoPos.height >= target_relative_height_m_ - tolerance ? BT::NodeStatus::SUCCESS
                                                                                            : BT::NodeStatus::RUNNING;
}

}  // namespace plane::task
