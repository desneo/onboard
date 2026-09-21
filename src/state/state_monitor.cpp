#include "state/state_monitor.h"

#include <spdlog/spdlog.h>

#include "state/state_events.h"

namespace plane::state {

StateMonitor::StateMonitor(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                           plane::communication::CommunicationApi& communication)
    : event_bus_(event_bus), config_(config), communication_(communication) {}

StateMonitor::~StateMonitor() { stop(); }

std::string StateMonitor::name() const { return "StateMonitor"; }

bool StateMonitor::init() { return true; }

bool StateMonitor::start() {
    event_bus_.subscribe<plane::model::VehicleState>(
        events::kOwnStateChanged, [this](const plane::model::VehicleState& state) { on_own_state(state); });
    return true;
}

void StateMonitor::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    low_voltage_latched_ = false;
}

void StateMonitor::release() {}

void StateMonitor::on_own_state(const plane::model::VehicleState& state) {
    if (state.fcLinkState != "connected") {
        std::lock_guard<std::mutex> lock(mutex_);
        low_voltage_latched_ = false;
        return;
    }
    const double threshold = config_.alert_voltage_threshold_v;
    const double voltage = static_cast<double>(state.voltage_v);
    bool trigger = false;
    bool recovered = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (voltage < threshold && !low_voltage_latched_) {
            low_voltage_latched_ = true;
            trigger = true;
        } else if (voltage >= threshold && low_voltage_latched_) {
            low_voltage_latched_ = false;
            recovered = true;
        }
    }
    if (trigger) {
        const std::string detail =
            "voltage=" + std::to_string(voltage) + "V < threshold=" + std::to_string(threshold) + "V";
        SPDLOG_WARN("低电压告警: {}", detail);
        send_alert(detail);
    } else if (recovered) {
        SPDLOG_INFO("电压恢复正常: {} V", voltage);
    }
}

void StateMonitor::send_alert(const std::string& detail) {
    plane::model::Alert alert;
    alert.type = "low_voltage";
    alert.detail = detail;
    if (!communication_.sendToStation(plane::communication::OutgoingMessage{alert})) {
        SPDLOG_ERROR("低电压告警发送失败: {}", detail);
    }
}

}  // namespace plane::state
