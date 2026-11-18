#pragma once

#include <behaviortree_cpp/action_node.h>

#include <atomic>
#include <chrono>
#include <string>

#include "flight_control/flight_control_api.h"

namespace plane::task {

class SelfCheckNode final : public BT::StatefulActionNode {
   public:
    using Clock = std::chrono::steady_clock;
    static constexpr auto kConfirmationTimeout = std::chrono::seconds(3);
    static constexpr char kResultDetail[] = "self_check_detail";

    SelfCheckNode(const std::string& name, const BT::NodeConfig& config,
                  plane::flight_control::FlightControlApi* flight_control, const std::atomic<bool>* ticking,
                  Clock::time_point (*now)() noexcept = &Clock::now);

    static BT::PortsList providedPorts() { return {}; }

    void restore_on_shutdown();

   private:
    enum class Phase { Ready, ConfirmArm, ConfirmDisarm };

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {}

    BT::NodeStatus start_arm();
    BT::NodeStatus start_disarm(const plane::flight_control::FlightStatus& status, bool shutdown = false);
    BT::NodeStatus confirm_disarm(const plane::flight_control::FlightStatus& status, Clock::time_point now);
    bool armed_sample_received(const plane::flight_control::FlightStatus& status, bool armed,
                               Clock::time_point now) const;
    BT::NodeStatus result();
    std::string restore_error(const plane::flight_control::FlightStatus& status, Clock::time_point now) const;
    BT::NodeStatus fail(std::string reason);
    void write_detail(const std::string& detail);

    plane::flight_control::FlightControlApi* flight_control_;
    const std::atomic<bool>* ticking_;
    Clock::time_point (*now_)() noexcept;
    Phase phase_ = Phase::Ready;
    plane::flight_control::FlightStatus initial_status_;
    Clock::time_point requested_at_;
    Clock::time_point deadline_;
    bool needs_restore_ = false;
    std::string failure_;
};

}  // namespace plane::task
