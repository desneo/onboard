#include "task/nodes/self_check_node.h"

#include <spdlog/spdlog.h>

#include <thread>
#include <utility>

namespace plane::task {

using plane::flight_control::FlightControlApi;
using plane::flight_control::FlightStatus;
using plane::flight_control::SelfCheckArmResult;
using plane::flight_control::self_check_ground_error;
using plane::flight_control::self_check_telemetry_fresh;

SelfCheckNode::SelfCheckNode(const std::string& name, const BT::NodeConfig& config, FlightControlApi* flight_control,
                             const std::atomic<bool>* ticking, Clock::time_point (*now)() noexcept)
    : BT::StatefulActionNode(name, config), flight_control_(flight_control), ticking_(ticking), now_(now) {}

BT::NodeStatus SelfCheckNode::onStart() {
    phase_ = Phase::Ready;
    needs_restore_ = false;
    failure_.clear();
    if (config().blackboard) {
        config().blackboard->set("current_node", std::string(name()));
    }
    return onRunning();
}

BT::NodeStatus SelfCheckNode::start_arm() {
    initial_status_ = flight_control_->flight_status();
    const auto now = now_();
    const auto ground_error = self_check_ground_error(initial_status_, now);
    if (!ground_error.empty()) {
        return fail("解锁前检查失败，" + ground_error);
    }
    if (!self_check_telemetry_fresh(initial_status_.armed_updated, now)) {
        return fail("解锁前检查失败，缺少有效的解锁状态遥测");
    }
    if (initial_status_.armed) {
        return fail("解锁前检查失败，飞机已解锁");
    }
    if (!*ticking_) {
        return BT::NodeStatus::RUNNING;
    }
    SPDLOG_INFO("自检已确认飞机在地面且已上锁，准备解锁");
    phase_ = Phase::ConfirmArm;
    requested_at_ = now_();
    needs_restore_ = true;
    std::string error;
    const auto arm_result = flight_control_->self_check_arm(initial_status_, ticking_, error);
    deadline_ = now_() + kConfirmationTimeout;
    if (arm_result == SelfCheckArmResult::Rejected) {
        needs_restore_ = false;
        return fail("解锁失败，" + error);
    }
    if (arm_result == SelfCheckArmResult::Uncertain) {
        failure_ = "解锁失败，" + error;
        if (*ticking_ && flight_control_->control_can_advance()) {
            return start_disarm(flight_control_->flight_status());
        }
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SelfCheckNode::onRunning() {
    if (!*ticking_ || !flight_control_->control_can_advance()) {
        return BT::NodeStatus::RUNNING;
    }
    if (phase_ == Phase::Ready) {
        return start_arm();
    }
    const auto status = flight_control_->flight_status();
    const auto now = now_();
    if (phase_ == Phase::ConfirmDisarm) {
        return confirm_disarm(status, now);
    }
    if (const auto error = restore_error(status, now); !error.empty()) {
        return fail("解锁确认中断，" + error);
    }
    if (armed_sample_received(status, true, now)) {
        SPDLOG_INFO("自检已确认实际解锁，准备上锁");
        return start_disarm(status);
    }
    if (failure_.empty() && now >= deadline_) {
        failure_ = "等待飞控确认解锁超时";
    }
    return failure_.empty() ? BT::NodeStatus::RUNNING : start_disarm(status);
}

BT::NodeStatus SelfCheckNode::start_disarm(const FlightStatus& status, bool shutdown) {
    if (const auto error = restore_error(status, now_()); !error.empty()) {
        return fail("无法执行上锁，" + error);
    }
    phase_ = Phase::ConfirmDisarm;
    requested_at_ = now_();
    std::string error;
    if (!flight_control_->self_check_disarm(status, shutdown ? nullptr : ticking_, error)) {
        if (!failure_.empty()) {
            failure_ += "，";
        }
        failure_ += "上锁失败，" + error;
    }
    deadline_ = now_() + kConfirmationTimeout;
    return confirm_disarm(flight_control_->flight_status(), now_());
}

BT::NodeStatus SelfCheckNode::confirm_disarm(const FlightStatus& status, Clock::time_point now) {
    if (const auto error = restore_error(status, now); !error.empty()) {
        return fail("上锁确认中断，" + error);
    }
    if (armed_sample_received(status, false, now)) {
        needs_restore_ = false;
        SPDLOG_INFO("自检已确认实际恢复上锁");
        return result();
    }
    if (now >= deadline_) {
        return fail("等待飞控确认上锁超时");
    }
    return BT::NodeStatus::RUNNING;
}

bool SelfCheckNode::armed_sample_received(const FlightStatus& status, bool armed, Clock::time_point now) const {
    return status.armed == armed && status.armed_updated > requested_at_ && status.armed_updated <= deadline_ &&
           self_check_telemetry_fresh(status.armed_updated, now);
}

std::string SelfCheckNode::restore_error(const FlightStatus& status, Clock::time_point now) const {
    if (status.link_generation != initial_status_.link_generation) {
        return "飞控连接已中断或变更";
    }
    if (status.flight_mode != initial_status_.flight_mode) {
        return "飞行模式已变更，flightMode=" + status.flight_mode;
    }
    if (!flight_control_->control_can_advance()) {
        return "飞控控制权已移交";
    }
    return self_check_ground_error(status, now, false);
}

BT::NodeStatus SelfCheckNode::fail(std::string reason) {
    if (!failure_.empty()) {
        failure_ += "，";
    }
    failure_ += std::move(reason);
    return result();
}

BT::NodeStatus SelfCheckNode::result() {
    if (failure_.empty()) {
        write_detail("已确认解锁并恢复上锁");
        return BT::NodeStatus::SUCCESS;
    }
    write_detail(needs_restore_ ? failure_ + "，未确认恢复上锁" : failure_);
    return BT::NodeStatus::FAILURE;
}

void SelfCheckNode::write_detail(const std::string& detail) {
    if (config().blackboard) {
        config().blackboard->set(kResultDetail, detail);
    }
}

void SelfCheckNode::restore_on_shutdown() {
    if (!needs_restore_) {
        return;
    }
    const auto flight_status = flight_control_->flight_status();
    if (phase_ == Phase::ConfirmDisarm) {
        confirm_disarm(flight_status, now_());
        if (!needs_restore_) {
            return;
        }
    }
    auto status = start_disarm(flight_status, true);
    while (status == BT::NodeStatus::RUNNING) {
        status = confirm_disarm(flight_control_->flight_status(), now_());
        if (status == BT::NodeStatus::RUNNING) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    if (needs_restore_) {
        SPDLOG_ERROR("停机时自检未确认恢复上锁，detail={}", failure_);
    }
}

}  // namespace plane::task
