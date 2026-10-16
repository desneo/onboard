#pragma once

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"
#include "flight_control/flight_control_api.h"
#include "state/state_store_api.h"

namespace plane::flight_control {

/// 飞控接入：MAVSDK 连接管理、遥测聚合与能力分发（ARCHITECTURE §13）。
class FlightControlModule final : public plane::core::IModule, public FlightControlApi {
   public:
    FlightControlModule(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                        plane::state::StateStoreApi& state_store);
    ~FlightControlModule() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    bool arm(std::string& error) override;
    bool disarm(std::string& error) override;
    bool takeoff(float height_m, std::string& error) override;
    bool land(std::string& error) override;
    bool return_to_launch(std::string& error) override;
    bool hold(std::string& error) override;
    bool stop_motor(std::string& error) override;
    bool goto_waypoint(const plane::model::GeoPoint& point, std::string& error) override;
    bool execute_route(const std::vector<plane::model::GeoPoint>& points, std::string& error) override;
    bool set_ned_velocity(const plane::model::NedVelocity& velocity, std::string& error) override;
    void clear_ned_velocity() override;

    std::string link_state() const override;
    plane::flight_control::FlightStatus flight_status() const override;

   private:
    struct TelemetryCache {
        double latitude_deg = 0.0;
        double longitude_deg = 0.0;
        float relative_altitude_m = 0.0F;
        float absolute_altitude_m = 0.0F;
        double home_latitude_deg = 0.0;
        double home_longitude_deg = 0.0;
        float home_relative_altitude_m = 0.0F;
        float home_absolute_altitude_m = 0.0F;
        float roll_deg = 0.0F;
        float pitch_deg = 0.0F;
        float yaw_deg = 0.0F;
        float vn_m_s = 0.0F;
        float ve_m_s = 0.0F;
        float vd_m_s = 0.0F;
        float airspeed_m_s = 0.0F;
        float voltage_v = 0.0F;
        mavsdk::Telemetry::FlightMode flight_mode = mavsdk::Telemetry::FlightMode::Unknown;
        bool armed = false;
        bool in_air = false;
        int mission_current = 0;
        int mission_total = 0;
        float takeoff_altitude_m = 0.0F;
    };

    std::string connection_url() const;
    void monitor_loop();
    void state_loop();
    void offboard_loop();
    void attach_system(const std::shared_ptr<mavsdk::System>& system);
    void detach_system();
    bool ensure_connected(std::string& error) const;
    bool upload_and_start_mission(const std::vector<plane::model::GeoPoint>& points, bool fly_through,
                                  std::string& error);
    void report_error(const std::string& detail) const;

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    plane::state::StateStoreApi& state_store_;

    std::unique_ptr<mavsdk::Mavsdk> mavsdk_;
    std::shared_ptr<mavsdk::System> system_;
    std::unique_ptr<mavsdk::Action> action_;
    std::unique_ptr<mavsdk::Telemetry> telemetry_;
    std::unique_ptr<mavsdk::Mission> mission_;
    std::unique_ptr<mavsdk::Offboard> offboard_;

    mutable std::mutex cache_mutex_;
    TelemetryCache cache_;
    std::atomic<bool> connected_{false};

    std::mutex offboard_mutex_;
    mavsdk::Offboard::VelocityNedYaw setpoint_;
    bool offboard_active_ = false;

    std::thread monitor_thread_;
    std::thread state_thread_;
    std::thread offboard_thread_;
    std::atomic<bool> running_{false};
    std::int64_t last_wait_log_ms_ = 0;
};

}  // namespace plane::flight_control
