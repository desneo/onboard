#include "flight_control/manual_takeover.h"

#include <iostream>
#include <stdexcept>

namespace {

using plane::flight_control::ManualTakeover;
using Mode = ManualTakeover::Mode;
using Action = ManualTakeover::Action;
using namespace std::chrono_literals;

constexpr ManualTakeover::Clock::time_point kStart{};

void require(bool condition, const char* detail) {
    if (!condition) {
        throw std::runtime_error(detail);
    }
}

void position_baseline_and_rapid_switches() {
    ManualTakeover detector;
    detector.observe(Mode::Posctl, {}, kStart);
    detector.start();
    require(!detector.observe(Mode::Posctl, {}, kStart).takeover, "飞前 Position 应允许执行任务");
    require(!detector.observe(Mode::Offboard, {}, kStart + 50ms).takeover, "进入 Offboard 不应中止");
    auto result = detector.observe(Mode::Hold, {}, kStart + 100ms);
    require(result.takeover == Mode::Hold && result.previous == Mode::Offboard,
            "Offboard 非预期进入 Loiter 应立即中止");
    require(!detector.observe(Mode::Stabilized, {}, kStart + 150ms).takeover, "快速切换不得重复中止");
    require(!detector.observe(Mode::Posctl, {}, kStart + 200ms).takeover, "拨回 Position 不得重复中止");

    detector.finish();
    detector.start();
    require(!detector.observe(Mode::Posctl, {}, kStart + 250ms).takeover, "新任务应接受接管后的 Position 基线");
    detector.observe(Mode::Mission, {}, kStart + 300ms);
    result = detector.observe(Mode::Posctl, {}, kStart + 350ms);
    require(result.takeover == Mode::Posctl && result.previous == Mode::Mission,
            "新任务中再进入 Position 应再次接管");
}

void hold_completed_mission_and_expiry() {
    ManualTakeover detector;
    detector.observe(Mode::Posctl, {}, kStart);
    detector.start();
    detector.expect(Action::Mission);
    detector.observe(Mode::Mission, {}, kStart + 50ms);
    auto result = detector.observe(Mode::Hold, {}, kStart + 100ms);
    require(result.pending && !result.takeover && detector.deadline() == kStart + 400ms,
            "航点完成证据可晚于 Hold，核对期间应暂停下发");
    result = detector.update({true}, kStart + 250ms);
    require(!result.pending && !result.takeover, "本次航点完成后正常进入 Hold 不应中止");

    detector.expect(Action::Mission);
    detector.observe(Mode::Mission, {}, kStart + 300ms);
    require(detector.observe(Mode::Hold, {}, kStart + 350ms).pending, "新航线需要本次航点完成证据");
    require(detector.observe(Mode::Mission, {}, kStart + 400ms).pending,
            "快速拨回后应继续核对首次 Loiter 是否正常");
    result = detector.update({}, kStart + 650ms);
    require(result.takeover == Mode::Hold && result.previous == Mode::Mission && !result.pending,
            "核对超时应保留首次 Mission 到 Hold 的模式，不能被快速拨回覆盖");
    require(!detector.observe(Mode::Hold, {}, kStart + 700ms).takeover,
            "核对超时接管后不得重复中止");
}

void pilot_mode_overrides_pending_hold() {
    ManualTakeover detector;
    detector.observe(Mode::Mission, {}, kStart);
    detector.start();
    detector.expect(Action::Mission);
    require(detector.observe(Mode::Hold, {}, kStart + 50ms).pending, "应先核对本次航点是否完成");
    auto result = detector.observe(Mode::Stabilized, {}, kStart + 100ms);
    require(result.takeover == Mode::Stabilized && result.previous == Mode::Hold && !result.pending,
            "核对期间切到 Stabilized 必须立即中止并记录 Hold 到 Stabilized");
    require(!detector.update({true}, kStart + 150ms).takeover, "迟到的航点状态不得影响接管");
}

void failed_action_preserves_pending_hold_transition() {
    ManualTakeover detector;
    detector.observe(Mode::Mission, {}, kStart);
    detector.start();
    detector.expect(Action::Mission);
    require(detector.observe(Mode::Hold, {}, kStart + 50ms).pending, "应先核对本次航点是否完成");
    detector.observe(Mode::ReturnToLaunch, {}, kStart + 100ms);
    detector.action_failed(Action::Mission);
    const auto result = detector.update({true}, kStart + 150ms);
    require(result.takeover == Mode::Hold && result.previous == Mode::Mission && !result.pending,
            "航点动作失败后应立即按首次 Mission 到 Hold 接管，不被后续返航模式覆盖");
}

void allow_owned_hold_and_takeoff_handoff() {
    ManualTakeover detector;
    detector.observe(Mode::Posctl, {}, kStart);
    detector.start();
    detector.expect(Action::Hold, kStart);
    require(!detector.observe(Mode::Hold, {}, kStart + 50ms).takeover, "机载主动 Hold 应正常完成");
    detector.finish();
    detector.start();
    detector.expect(Action::Takeoff);
    detector.observe(Mode::Takeoff, {}, kStart + 100ms);
    const auto result = detector.observe(Mode::Hold, {}, kStart + 200ms);
    require(!result.pending && !result.takeover, "起飞结束切回 Hold 属于正常交接，不得暂停或中止");
    require(!detector.observe(Mode::Offboard, {}, kStart + 350ms).takeover, "起飞交接后应能进入 Offboard");

    detector.finish();
    detector.start();
    detector.expect(Action::Takeoff);
    detector.observe(Mode::Takeoff, {}, kStart + 400ms);
    detector.observe(Mode::Offboard, {}, kStart + 500ms);
    require(!detector.observe(Mode::Hold, {}, kStart + 550ms).takeover, "起飞完成覆盖首次 Offboard 时应允许交接");
    detector.observe(Mode::Offboard, {}, kStart + 600ms);
    require(detector.observe(Mode::Hold, {}, kStart + 650ms).takeover == Mode::Hold,
            "起飞正常 Hold 只应豁免一次");

    detector.finish();
    detector.start();
    detector.expect(Action::Takeoff);
    detector.observe(Mode::Takeoff, {}, kStart + 700ms);
    detector.observe(Mode::Offboard, {}, kStart + 750ms);
    require(detector.observe(Mode::Hold, {}, kStart + 1750ms).takeover == Mode::Hold,
            "起飞交接结束后应及时恢复对非预期 Loiter 的检测");
}

void unrelated_and_reconnected_modes() {
    ManualTakeover detector;
    detector.start();
    require(!detector.observe(Mode::Posctl, {}, kStart).takeover, "首个有效模式只建立基线");
    detector.observe(Mode::ReturnToLaunch, {}, kStart + 50ms);
    require(!detector.observe(Mode::Land, {}, kStart + 100ms).takeover, "正常返航降落无需中止");
    detector.reset_link();
    require(!detector.observe(Mode::Stabilized, {}, kStart + 150ms).takeover, "重连首帧只建立基线");
    detector.observe(Mode::Unknown, {}, kStart + 200ms);
    const auto result = detector.observe(Mode::Posctl, {}, kStart + 250ms);
    require(result.takeover == Mode::Posctl && result.previous == Mode::Stabilized,
            "Unknown 不应遮蔽两个有效模式间的切换或覆盖前序模式");
}

void delayed_owned_hold_after_replacement() {
    ManualTakeover detector;
    detector.observe(Mode::Offboard, {}, kStart);
    detector.start();
    detector.expect(Action::Hold, kStart);
    detector.finish();
    detector.start();
    require(!detector.observe(Mode::Hold, {}, kStart + 200ms).takeover, "旧执行主动 Hold 的迟到遥测应正常处理");
    detector.observe(Mode::Offboard, {}, kStart + 300ms);
    require(detector.observe(Mode::Hold, {}, kStart + 400ms).takeover == Mode::Hold,
            "旧执行主动 Hold 只应匹配一次");
}

}  // namespace

int main() {
    try {
        position_baseline_and_rapid_switches();
        hold_completed_mission_and_expiry();
        pilot_mode_overrides_pending_hold();
        failed_action_preserves_pending_hold_transition();
        allow_owned_hold_and_takeoff_handoff();
        unrelated_and_reconnected_modes();
        delayed_owned_hold_after_replacement();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
