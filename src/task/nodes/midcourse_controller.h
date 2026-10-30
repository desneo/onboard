#pragma once

#include <memory>
#include <string>

#include "core/config.h"
#include "core/event_bus.h"
#include "flight_control/flight_control_api.h"
#include "state/state_store_api.h"
#include "target/target_store_api.h"

namespace plane::task {

/// 中制导会话：管理目标估计器、规划器及雷达更新与行为树 tick 的并发。
class MidcourseController {
   public:
    enum class TickResult { Running, Failure };

    MidcourseController(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                        plane::flight_control::FlightControlApi& flight_control,
                        plane::state::StateStoreApi& state_store, plane::target::TargetStoreApi& target_store);
    ~MidcourseController();

    void start();
    bool begin(const std::string& requested_track_id, std::string& error);
    TickResult tick(std::string& error);
    void stop();

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace plane::task
