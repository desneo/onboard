#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"
#include "state/state_store_api.h"
#include "state/state_types.h"

namespace plane::state {

/// UAV 状态管理：本机、其他无人机与地面站链路的统一存储与查询（ARCHITECTURE §11）。
class StateStore final : public plane::core::IModule, public StateStoreApi {
   public:
    StateStore(plane::core::EventBus& event_bus, const plane::core::AppConfig& config);
    ~StateStore() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    plane::model::VehicleState own_state() const override;
    std::vector<plane::model::VehicleState> peer_states() const override;
    bool peer_state(const std::string& plane_id, plane::model::VehicleState& out) const override;
    GcsLinkState gcs_link() const override;

    void update_own_state(const plane::model::VehicleState& state) override;
    void update_peer_state(const plane::model::VehicleState& state) override;
    void notify_station_seen() override;

   private:
    void tick_loop();

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    mutable std::mutex mutex_;
    plane::model::VehicleState own_;
    std::unordered_map<std::string, plane::model::VehicleState> peers_;
    bool gcs_connected_ = false;
    std::int64_t gcs_last_seen_ms_ = 0;
    std::thread tick_;
    std::atomic<bool> running_{false};
};

}  // namespace plane::state
