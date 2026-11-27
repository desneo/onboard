#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

#include "state/state_events.h"
#include "state/state_monitor.h"

namespace {

class TestCommunication final : public plane::communication::CommunicationApi {
   public:
    bool sendToStation(const plane::communication::OutgoingMessage& message) override {
        alerts.push_back(std::get<plane::model::Alert>(message));
        return accept_alert;
    }

    bool sendToPlane(const std::string&, const plane::communication::OutgoingMessage&) override { return false; }
    bool sendToAll(const plane::communication::OutgoingMessage&) override { return false; }
    bool queryAgent(const std::string&, plane::model::Message&, std::string&, int) override { return false; }

    bool accept_alert = false;
    std::vector<plane::model::Alert> alerts;
};

}  // namespace

int main() {
    std::ostringstream output;
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(output);
    auto logger = std::make_shared<spdlog::logger>("低电压日志测试", sink);
    logger->set_level(spdlog::level::info);
    logger->set_pattern("%v");
    spdlog::set_default_logger(logger);

    plane::core::EventBus event_bus;
    plane::core::AppConfig config;
    config.is_sim = false;
    config.alert_voltage_threshold_v = 21.0;
    TestCommunication communication;
    plane::state::StateMonitor monitor(event_bus, config, communication);
    monitor.start();
    output.str("");

    plane::model::VehicleState state;
    state.fcLinkState = "connected";
    // 原始电压低于阈值，舍入后与阈值相同。
    state.voltage_v = 20.9990234375F;
    event_bus.publish(plane::state::events::kOwnStateChanged, state);
    event_bus.publish(plane::state::events::kOwnStateChanged, state);

    state.voltage_v = 25.095001220703125F;
    event_bus.publish(plane::state::events::kOwnStateChanged, state);

    communication.accept_alert = true;
    state.voltage_v = 0.0F;
    event_bus.publish(plane::state::events::kOwnStateChanged, state);

    const std::string expected_log =
        "低电压告警: voltage=21.00V threshold=21.00V\n"
        "低电压告警发送失败: voltage=21.00V threshold=21.00V\n"
        "电压恢复正常: voltage=25.10V\n"
        "低电压告警: voltage=0.00V threshold=21.00V\n";
    if (output.str() != expected_log) {
        std::cerr << "低电压日志与预期不符\n" << output.str();
        return 1;
    }
    if (communication.alerts.size() != 2 || communication.alerts[0].type != "low_voltage" ||
        communication.alerts[0].detail != "voltage=20.999023V < threshold=21.000000V" ||
        communication.alerts[1].type != "low_voltage" ||
        communication.alerts[1].detail != "voltage=0.000000V < threshold=21.000000V") {
        std::cerr << "低电压告警报文与预期不符\n";
        return 1;
    }
    return 0;
}
