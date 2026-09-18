#pragma once

#include <mutex>
#include <string>

#include "communication/communication_api.h"
#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"

namespace plane::state {

/// 安全监控：第一版仅低电压告警，边沿触发，仅飞控连接时评估（SPEC §10）。
class StateMonitor final : public plane::core::IModule {
   public:
    StateMonitor(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                 plane::communication::CommunicationApi& communication);
    ~StateMonitor() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

   private:
    void on_own_state(const plane::model::VehicleState& state);
    void send_alert(const std::string& detail);

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    plane::communication::CommunicationApi& communication_;
    std::mutex mutex_;
    bool low_voltage_latched_ = false;
};

}  // namespace plane::state
