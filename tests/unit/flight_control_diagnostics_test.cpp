#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "flight_control/flight_control_diagnostics.h"

namespace {

using plane::flight_control::FlightControlDiagnostics;
using plane::model::Alert;
using Clock = FlightControlDiagnostics::Clock;
using namespace std::chrono_literals;

void require(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

void require_alert(const std::optional<Alert>& alert, const std::string& type, const std::string& text) {
    require(alert && alert->type == type && alert->detail.find(text) != std::string::npos,
            "诊断报文类型或详情错误：" + type + "，" + text);
}

void status_text_preserves_severity_and_full_text() {
    FlightControlDiagnostics diagnostics;
    diagnostics.link(1, true);
    const auto now = Clock::time_point{} + 10s;
    const std::string text = "Preflight Fail: " + std::string(180, 'x') + "\n传感器未就绪";
    const std::vector<std::string> types{"Info", "Notice", "Warning", "Error", "Critical", "Alert", "Emergency"};
    for (const auto& type : types) {
        const auto alert = diagnostics.status_text(1, type, text, now);
        require_alert(alert, "flight_control_status_text", "type=" + type);
        require(alert->detail == "飞控 STATUSTEXT：type=" + type + " text=" + text,
                "STATUSTEXT 原文被截断、改写或丢失级别");
    }
    require(!diagnostics.status_text(1, "Debug", text, now), "Debug 不应上报");
    require(!diagnostics.status_text(1, "Unknown", text, now), "无效的 STATUSTEXT 级别不应上报");
    require(!diagnostics.status_text(1, "Warning", "", now), "空 STATUSTEXT 不应上报");
}

void status_text_uses_a_five_second_window_per_message() {
    FlightControlDiagnostics diagnostics;
    diagnostics.link(2, true);
    const auto now = Clock::time_point{} + 20s;
    require(diagnostics.status_text(2, "Warning", "GPS failed", now).has_value(), "首份文本未上报");
    require(!diagnostics.status_text(2, "Warning", "GPS failed", now + 4999ms), "五秒内重复文本未抑制");
    require(diagnostics.status_text(2, "Error", "GPS failed", now + 4999ms).has_value(), "不同级别被错误去重");
    require(diagnostics.status_text(2, "Warning", "EKF failed", now + 4999ms).has_value(), "不同文本被错误去重");
    require(diagnostics.status_text(2, "Warning", "GPS failed", now + 5s).has_value(),
            "五秒后未重新上报，或被抑制消息错误延长了窗口");
}

void state_reports_valid_baselines_and_changes() {
    FlightControlDiagnostics diagnostics;
    const auto initial_link = diagnostics.link(0, false);
    require_alert(initial_link, "flight_control_state_changed", "fcLinkState=disconnected");
    require(initial_link->detail.find("previous=") == std::string::npos, "初始连接被伪装成状态切换");
    require(!diagnostics.link(0, false), "重复连接状态未抑制");
    require_alert(diagnostics.link(1, true), "flight_control_state_changed", "previous=disconnected value=connected");
    require(!diagnostics.flight_mode(1, "Unknown"), "Unknown 模式不应建立有效基线");
    const auto mode = diagnostics.flight_mode(1, "Posctl");
    require_alert(mode, "flight_control_state_changed", "flightMode=Posctl");
    require(mode->detail.find("previous=") == std::string::npos, "首次有效模式被伪装成切换");
    require(!diagnostics.flight_mode(1, "Posctl"), "重复模式未抑制");
    require_alert(diagnostics.flight_mode(1, "Offboard"), "flight_control_state_changed", "previous=Posctl value=Offboard");
    require_alert(diagnostics.armed(1, false), "flight_control_state_changed", "armed=false");
    require(!diagnostics.armed(1, false), "重复解锁状态未抑制");
    require_alert(diagnostics.armed(1, true), "flight_control_state_changed", "armed previous=false value=true");
    require_alert(diagnostics.inair(1, false), "flight_control_state_changed", "inair=false");
    require(!diagnostics.inair(1, false), "重复离地状态未抑制");
    require_alert(diagnostics.inair(1, true), "flight_control_state_changed", "inair previous=false value=true");
}

void reconnect_resets_baselines_and_text_suppression() {
    FlightControlDiagnostics diagnostics;
    const auto now = Clock::time_point{} + 30s;
    diagnostics.link(3, true);
    diagnostics.flight_mode(3, "Hold");
    diagnostics.armed(3, false);
    diagnostics.inair(3, false);
    diagnostics.status_text(3, "Info", "Ready", now);
    require_alert(diagnostics.link(3, false), "flight_control_state_changed", "previous=connected value=disconnected");
    require(!diagnostics.status_text(3, "Info", "Ready", now + 1s), "断开期间仍上报 STATUSTEXT");
    require_alert(diagnostics.link(3, true), "flight_control_state_changed", "previous=disconnected value=connected");
    require_alert(diagnostics.flight_mode(3, "Hold"), "flight_control_state_changed", "flightMode=Hold");
    require_alert(diagnostics.armed(3, false), "flight_control_state_changed", "armed=false");
    require_alert(diagnostics.inair(3, false), "flight_control_state_changed", "inair=false");
    require(diagnostics.status_text(3, "Info", "Ready", now + 1s).has_value(), "重连未清空文本去重窗口");
    diagnostics.link(4, true);
    require(!diagnostics.link(3, false), "旧连接的断开事件影响新连接");
    require(!diagnostics.status_text(3, "Error", "Old failure", now + 2s), "旧连接 STATUSTEXT 未过滤");
    require(!diagnostics.flight_mode(3, "Mission"), "旧连接模式未过滤");
    require(!diagnostics.armed(3, true) && !diagnostics.inair(3, true), "旧连接遥测未过滤");
    require_alert(diagnostics.flight_mode(4, "Hold"), "flight_control_state_changed", "flightMode=Hold");
    require(diagnostics.status_text(4, "Info", "Ready", now + 2s).has_value(), "新系统未重置文本去重窗口");
}

void offboard_faults_report_only_edges_and_keep_causes() {
    FlightControlDiagnostics diagnostics;
    diagnostics.link(5, true);
    const std::string velocity = "Offboard 速度";
    const std::string attitude = "Offboard Attitude";
    const std::string first_error = "Offboard 速度设定值下发失败: ConnectionError";
    const std::string second_error = "Offboard 速度设定值下发失败: NoSystem";
    require(!diagnostics.offboard_result(5, velocity, ""), "正常周期发送不应上报");
    require_alert(diagnostics.offboard_result(5, velocity, first_error), "flight_control_command_failed", first_error);
    for (int i = 0; i < 100; ++i) {
        require(!diagnostics.offboard_result(5, velocity, first_error), "重复 Offboard 失败造成刷屏");
    }
    require_alert(diagnostics.offboard_result(5, velocity, second_error), "flight_control_command_failed", second_error);
    require_alert(diagnostics.offboard_result(5, attitude, "Attitude 下发失败: ConnectionError"),
                  "flight_control_command_failed", "ConnectionError");
    require_alert(diagnostics.offboard_result(5, velocity, ""), "offboard_send_recovered", velocity);
    require(!diagnostics.offboard_result(5, velocity, ""), "Offboard 恢复被重复上报");
    require(!diagnostics.offboard_result(5, attitude, "Attitude 下发失败: ConnectionError"),
            "速度恢复错误清除了姿态故障");
    diagnostics.clear_offboard();
    require(!diagnostics.offboard_result(5, attitude, ""), "主动清理被误报为发送恢复");
    require(diagnostics.offboard_result(5, attitude, "Attitude 下发失败: ConnectionError").has_value(),
            "新 Offboard 会话的首次失败未上报");
    diagnostics.link(6, true);
    require(!diagnostics.offboard_result(5, velocity, first_error), "旧连接 Offboard 结果未过滤");
    require(!diagnostics.offboard_result(6, attitude, ""), "重连后残留了旧故障");
    diagnostics.link(6, false);
    require(diagnostics.offboard_result(6, velocity, "飞控未连接").has_value(), "未连接前置失败未上报");
    require(!diagnostics.offboard_result(6, velocity, "飞控未连接"), "未连接前置失败未降噪");
}

void concurrent_texts_do_not_duplicate_the_same_event() {
    FlightControlDiagnostics diagnostics;
    diagnostics.link(7, true);
    const auto now = Clock::time_point{} + 50s;
    std::atomic<int> reports{0};
    std::vector<std::thread> threads;
    for (int i = 0; i < 8; ++i) {
        threads.emplace_back([&]() {
            for (int j = 0; j < 100; ++j) {
                if (diagnostics.status_text(7, "Warning", "Duplicate PX4 warning", now)) ++reports;
            }
        });
    }
    for (auto& thread : threads) thread.join();
    require(reports == 1, "并发 STATUSTEXT 去重失效");
}

void invalid_new_inputs_do_not_recover_when_old_setpoints_are_sent() {
    FlightControlDiagnostics diagnostics;
    diagnostics.link(8, true);
    const std::string operation = "Offboard 速度";
    const std::string error = "Offboard 速度、加速度或航向必须为有限数";
    diagnostics.offboard_input_validity(8, operation, false);
    require(diagnostics.offboard_result(8, operation, error).has_value(), "非法设定值首错未上报");
    for (int i = 0; i < 100; ++i) {
        require(!diagnostics.offboard_result(8, operation, ""), "旧设定值成功发送被误报为非法输入恢复");
        diagnostics.offboard_input_validity(8, operation, false);
        require(!diagnostics.offboard_result(8, operation, error), "非法输入与旧设定值周期发送造成故障刷屏");
    }
    diagnostics.offboard_input_validity(8, operation, true);
    require_alert(diagnostics.offboard_result(8, operation, ""), "offboard_send_recovered", operation);
    require(!diagnostics.offboard_result(8, operation, ""), "合法输入恢复后重复上报");
    diagnostics.offboard_input_validity(8, operation, false);
    diagnostics.offboard_result(8, operation, error);
    diagnostics.clear_offboard();
    require(!diagnostics.offboard_result(8, operation, ""), "会话结束后非法输入锁存未清理");
}

}  // namespace

int main() {
    status_text_preserves_severity_and_full_text();
    status_text_uses_a_five_second_window_per_message();
    state_reports_valid_baselines_and_changes();
    reconnect_resets_baselines_and_text_suppression();
    offboard_faults_report_only_edges_and_keep_causes();
    concurrent_texts_do_not_duplicate_the_same_event();
    invalid_new_inputs_do_not_recover_when_old_setpoints_are_sent();
    return 0;
}
