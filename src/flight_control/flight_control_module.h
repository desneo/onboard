#pragma once

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"
#include "flight_control/flight_control_api.h"
#include "flight_control/flight_control_diagnostics.h"
#include "flight_control/flight_control_events.h"
#include "flight_control/manual_takeover.h"
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

    void begin_control(std::uint64_t execution_id) override;
    bool end_control(std::uint64_t execution_id, bool acknowledge_takeover = false) override;
    bool control_can_advance() const override;

    bool arm(std::string& error) override;
    bool disarm(std::string& error) override;
    SelfCheckArmResult self_check_arm(const FlightStatus& expected, const std::atomic<bool>* active,
                                     std::string& error) override;
    bool self_check_disarm(const FlightStatus& expected, const std::atomic<bool>* active, std::string& error) override;
    bool takeoff(float height_m, std::string& error) override;
    bool land(std::string& error) override;
    bool return_to_launch(std::string& error) override;
    bool hold(std::string& error) override;
    bool stop_motor(std::string& error) override;
    bool goto_waypoint(const plane::model::GeoPoint& point, std::string& error) override;
    bool execute_route(const std::vector<plane::model::GeoPoint>& points, std::string& error) override;
    bool set_ned_velocity(const plane::model::NedVelocity& velocity, std::string& error) override;
    bool set_ned_velocity_acceleration(const plane::algorithm::VelocityAccelerationCommand& command,
                                       double current_yaw_deg, std::string& error) override;
    bool set_attitude(const AttitudeSetpoint& setpoint, std::string& error) override;
    bool get_cached_flight_control_params(FlightControlParams& params, std::string& error) const override;
    void clear_offboard() override;

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
        mavsdk::Telemetry::LandedState landed_state = mavsdk::Telemetry::LandedState::Unknown;
        int mission_current = 0;
        int mission_total = 0;
        float takeoff_altitude_m = 0.0F;
        ManualTakeover::Clock::time_point position_updated{};
        ManualTakeover::Clock::time_point mission_progress_updated{};
        ManualTakeover::Clock::time_point armed_updated{};
        ManualTakeover::Clock::time_point ground_updated{};
        ManualTakeover::Clock::time_point mode_updated{};
    };

    struct SelfCheckAction {
        std::uint64_t link_generation;
        std::string flight_mode;
        bool arming;
        const std::atomic<bool>* active;
        bool allowed_send = false;
        std::string blocked_reason;
    };

    std::string connection_url() const;
    void monitor_loop();
    void state_loop();
    void offboard_loop();
    void manual_takeover_loop();
    void attach_system(const std::shared_ptr<mavsdk::System>& system);
    void detach_system();
    void request_landed_state_stream_if_needed();
    void refresh_flight_control_params_cache_if_needed();
    void invalidate_flight_control_params_cache(const std::string& reason);
    bool ensure_connected(std::string& error, const std::string& operation);
    FlightStatus flight_status_locked() const;
    std::optional<plane::model::Alert> update_link_state_locked(bool connected);
    bool allow_outgoing_message(const mavlink_message_t& message);
    std::string self_check_action_error_locked(const SelfCheckAction& action) const;
    SelfCheckArmResult run_self_check_action(const FlightStatus& expected, bool arming,
                                            const std::atomic<bool>* active, std::string& error);
    bool wait_for_control(std::string& error);
    bool expect_action(ManualTakeover::Action action, std::string& error, float takeoff_height_m = 0.0F,
                       int mission_total = 0);
    void action_failed(ManualTakeover::Action action);
    void update_takeover_evidence();
    ManualTakeover::Evidence takeover_evidence_locked() const;
    void apply_takeover_result_locked(const ManualTakeover::Result& result);
    bool upload_and_start_mission(const std::vector<plane::model::GeoPoint>& points, bool fly_through,
                                  std::string& error);
    bool set_velocity_setpoint(mavsdk::Offboard::VelocityNedYaw velocity,
                              std::optional<mavsdk::Offboard::AccelerationNed> acceleration, std::string& error);
    mavsdk::Offboard::Result send_velocity_setpoint_locked();
    void publish_diagnostic(const std::optional<plane::model::Alert>& alert) const;
    void report_command_started(const std::string& operation) const;
    void report_command_failed(const std::string& detail) const;
    void report_offboard_error(const std::string& operation, const std::string& error, bool invalid_input = false);
    void report_offboard_result(const std::string& operation, mavsdk::Offboard::Result result);
    void report_error(const std::string& detail) const;

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    plane::state::StateStoreApi& state_store_;
    FlightControlDiagnostics diagnostics_;

    std::unique_ptr<mavsdk::Mavsdk> mavsdk_;
    std::shared_ptr<mavsdk::System> system_;
    std::unique_ptr<mavsdk::Action> action_;
    std::unique_ptr<mavsdk::Telemetry> telemetry_;
    std::unique_ptr<mavsdk::Mission> mission_;
    std::unique_ptr<mavsdk::Offboard> offboard_;
    std::unique_ptr<mavsdk::Param> param_;

    mutable std::mutex cache_mutex_;
    TelemetryCache cache_;
    std::shared_ptr<mavsdk::MavlinkPassthrough> self_check_link_;
    std::uint64_t link_generation_ = 0;
    ManualTakeover::Clock::time_point last_landed_state_stream_request_{};
    std::optional<SelfCheckAction> self_check_action_;
    std::atomic<bool> connected_{false};
    std::atomic<std::uint64_t> connection_generation_{0};

    mutable std::mutex control_mutex_;
    std::condition_variable control_cv_;
    ManualTakeover manual_takeover_;
    std::uint64_t execution_id_ = 0;
    ManualTakeover::Clock::time_point takeoff_requested_at_{};
    ManualTakeover::Clock::time_point mission_requested_at_{};
    float control_takeoff_height_m_ = 0.0F;
    int control_mission_total_ = 0;
    std::optional<ManualTakeoverEvent> takeover_event_;
    enum class ControlGate { Open, CheckingHold, Takeover };
    std::atomic<ControlGate> control_gate_{ControlGate::Open};

    mutable std::mutex flight_control_params_mutex_;
    FlightControlParams flight_control_params_cache_;
    bool flight_control_params_ready_ = false;
    std::string flight_control_params_error_ = "尚未连接飞控并读取 PX4 参数";
    std::int64_t last_flight_control_params_attempt_ms_ = 0;

    std::mutex offboard_mutex_;
    enum class OffboardSetpointKind { None, VelocityNed, Attitude };
    mavsdk::Offboard::VelocityNedYaw velocity_setpoint_;
    std::optional<mavsdk::Offboard::AccelerationNed> acceleration_setpoint_;
    mavsdk::Offboard::Attitude attitude_setpoint_;
    OffboardSetpointKind offboard_setpoint_kind_ = OffboardSetpointKind::None;
    bool offboard_active_ = false;
    std::int64_t last_offboard_start_ms_ = 0;

    std::thread monitor_thread_;
    std::thread state_thread_;
    std::thread offboard_thread_;
    std::thread manual_takeover_thread_;
    std::atomic<bool> running_{false};
    std::int64_t last_wait_log_ms_ = 0;
};

}  // namespace plane::flight_control
