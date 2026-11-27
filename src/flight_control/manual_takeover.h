#pragma once

#include <mavsdk/plugins/telemetry/telemetry.h>

#include <chrono>
#include <optional>

namespace plane::flight_control {

/// 根据同一飞控连接上的模式变化判定接管，调用方负责串行化和关联本次动作的遥测。
class ManualTakeover {
   public:
    using Mode = mavsdk::Telemetry::FlightMode;
    using Clock = std::chrono::steady_clock;

    enum class Action { Other, Takeoff, Mission, Hold };

    struct Evidence {
        bool mission_complete = false;
    };

    struct Result {
        std::optional<Mode> takeover;
        bool pending = false;
        Mode previous = Mode::Unknown;
    };

    void start() {
        finish();
        active_ = true;
    }

    void finish() {
        active_ = false;
        taken_over_ = false;
        action_ = Action::Other;
        hold_check_.reset();
        clear_takeoff();
    }

    void reset_link() {
        mode_ = Mode::Unknown;
        action_ = Action::Other;
        hold_check_.reset();
        requested_hold_until_.reset();
        clear_takeoff();
    }

    void expect(Action action, Clock::time_point now = Clock::now()) {
        action_ = action == Action::Hold ? Action::Other : action;
        if (action == Action::Hold) {
            if (mode_ != Mode::Hold) {
                requested_hold_until_ = now + kHoldReportTime;
            }
        } else if (action == Action::Takeoff) {
            takeoff_expected_ = true;
            takeoff_handoff_until_.reset();
        } else if (action == Action::Other) {
            clear_takeoff();
        }
    }

    void action_failed(Action action) {
        if (action_ == action) {
            action_ = Action::Other;
        }
        if (action == Action::Takeoff) {
            clear_takeoff();
        } else if (action == Action::Hold) {
            requested_hold_until_.reset();
        }
    }

    std::optional<Clock::time_point> deadline() const {
        if (hold_check_) {
            return hold_check_->until;
        }
        return std::nullopt;
    }

    Result observe(Mode mode, Evidence evidence, Clock::time_point now) {
        if (mode == Mode::Unknown || mode == mode_) {
            return update(evidence, now);
        }

        const Mode previous = mode_;
        mode_ = mode;
        const bool requested_hold =
            mode == Mode::Hold && requested_hold_until_ && now < *requested_hold_until_;
        requested_hold_until_.reset();
        if (!active_ || taken_over_ || previous == Mode::Unknown) {
            return {};
        }
        if (mode == Mode::Stabilized || mode == Mode::Posctl) {
            return take_over(mode, previous);
        }

        if (mode == Mode::Hold && !hold_check_ && !requested_hold) {
            const Action reason = expected_hold(previous, now);
            if (reason == Action::Other) {
                return take_over(mode, previous);
            }
            if (reason == Action::Takeoff) {
                // 飞控完成起飞后自动切回 Hold 属于正常交接，完成与否由 Takeoff 节点按实际模式
                // 和离地状态判定，不能在这里用短时间窗口核对起飞遥测。
                clear_takeoff();
                return {};
            }
            hold_check_ = HoldCheck{reason, now + kHoldCheckTime, previous};
        } else if (mode == Mode::Offboard && previous == Mode::Takeoff && takeoff_expected_) {
            // PX4 起飞完成的模式切换可能晚于机载首次进入 Offboard。
            takeoff_handoff_until_ = now + kTakeoffHandoffTime;
        } else if (mode != Mode::Takeoff && !hold_check_) {
            clear_takeoff();
        }
        return update(evidence, now);
    }

    Result update(Evidence evidence, Clock::time_point now) {
        if (!hold_check_) {
            return {};
        }
        const Action reason = hold_check_->reason;
        if (action_ != reason || now >= hold_check_->until) {
            return take_over(Mode::Hold, hold_check_->previous);
        }
        if (reason != Action::Mission || !evidence.mission_complete) {
            return {std::nullopt, true};
        }
        action_ = Action::Other;
        hold_check_.reset();
        return {};
    }

   private:
    static constexpr auto kHoldCheckTime = std::chrono::milliseconds(300);
    static constexpr auto kHoldReportTime = std::chrono::milliseconds(1500);
    static constexpr auto kTakeoffHandoffTime = std::chrono::seconds(1);

    struct HoldCheck {
        Action reason;
        Clock::time_point until;
        Mode previous;
    };

    Action expected_hold(Mode previous, Clock::time_point now) const {
        if (previous == Mode::Mission && action_ == Action::Mission) {
            return Action::Mission;
        }
        if (takeoff_expected_ &&
            (previous == Mode::Takeoff ||
             (previous == Mode::Offboard && takeoff_handoff_until_ && now < *takeoff_handoff_until_))) {
            return Action::Takeoff;
        }
        return Action::Other;
    }

    void clear_takeoff() {
        takeoff_expected_ = false;
        takeoff_handoff_until_.reset();
    }

    Result take_over(Mode mode, Mode previous) {
        taken_over_ = true;
        hold_check_.reset();
        return {mode, false, previous};
    }

    Mode mode_ = Mode::Unknown;
    Action action_ = Action::Other;
    bool active_ = false;
    bool taken_over_ = false;
    bool takeoff_expected_ = false;
    std::optional<HoldCheck> hold_check_;
    std::optional<Clock::time_point> requested_hold_until_;
    std::optional<Clock::time_point> takeoff_handoff_until_;
};

}  // namespace plane::flight_control
