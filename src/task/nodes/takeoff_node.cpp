#include "task/nodes/takeoff_node.h"

namespace plane::task {

bool TakeoffNode::dispatch(std::string& error) {
    const auto height = static_cast<float>(app_config_->system_param.safeHeight_m);
    return flight_control_->takeoff(height, error);
}

BT::NodeStatus TakeoffNode::check() {
    const auto status = flight_control_->flight_status();
    if (!status.in_air) {
        return BT::NodeStatus::RUNNING;
    }
    const auto target = static_cast<float>(app_config_->system_param.safeHeight_m);
    const auto tolerance = static_cast<float>(app_config_->bt_takeoff_tolerance_m);
    return state_store_->own_state().geoPos.height >= target - tolerance ? BT::NodeStatus::SUCCESS
                                                                         : BT::NodeStatus::RUNNING;
}

}  // namespace plane::task
