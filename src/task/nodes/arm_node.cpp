#include "task/nodes/arm_node.h"

#include "core/time_utils.h"

namespace plane::task {

using plane::core::now_ms;

bool ArmNode::dispatch(std::string& error) { return flight_control_->arm(error); }

BT::NodeStatus ArmNode::checkFinish() {
    if (!flight_control_->flight_status().armed) {
        return BT::NodeStatus::RUNNING;
    }
    const std::int64_t now = now_ms();
    if (armed_at_ms_ == 0) {
        armed_at_ms_ = now;
    }
    // 解锁后保持 1 秒，让桨叶启动，再判定成功。
    return now - armed_at_ms_ >= kArmSpinUpMs ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

}  // namespace plane::task
