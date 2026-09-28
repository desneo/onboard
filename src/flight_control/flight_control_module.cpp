#include "flight_control/flight_control_module.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <magic_enum/magic_enum.hpp>

#include "core/time_utils.h"
#include "flight_control/flight_control_events.h"

namespace plane::flight_control {

using plane::core::now_ms;

namespace {

constexpr int kMonitorIntervalMs = 200;
constexpr int kStateIntervalMs = 100;
constexpr int kOffboardIntervalMs = 50;
constexpr int kWaitLogIntervalMs = 5000;

template <typename Result>
std::string result_text(Result result) {
    return std::string(magic_enum::enum_name(result));
}

}  // namespace

FlightControlModule::FlightControlModule(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                                         plane::state::StateStoreApi& state_store)
    : event_bus_(event_bus), config_(config), state_store_(state_store) {}

FlightControlModule::~FlightControlModule() { stop(); }

std::string FlightControlModule::name() const { return "Flight Control"; }

bool FlightControlModule::init() { return true; }

std::string FlightControlModule::connection_url() const {
    if (config_.fc_link_type == "serial") {
        return "serial://" + config_.fc_serial_device + ":" + std::to_string(config_.fc_serial_baud);
    }
    return config_.fc_udp_url;
}

bool FlightControlModule::start() {
    mavsdk_ = std::make_unique<mavsdk::Mavsdk>(mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::CompanionComputer});
    mavsdk_->set_heartbeat_timeout_s(static_cast<double>(config_.fc_heartbeat_timeout_ms) / 1000.0);

    const std::string url = connection_url();
    const mavsdk::ConnectionResult result = mavsdk_->add_any_connection(url);
    if (result != mavsdk::ConnectionResult::Success) {
        report_error("飞控连接初始化失败: " + url + " (" + result_text(result) + ")");
        return false;
    }
    SPDLOG_INFO("飞控连接参数: {} (心跳超时 {} ms)", url, config_.fc_heartbeat_timeout_ms);

    running_ = true;
    monitor_thread_ = std::thread([this]() { monitor_loop(); });
    state_thread_ = std::thread([this]() { state_loop(); });
    offboard_thread_ = std::thread([this]() { offboard_loop(); });
    return true;
}

void FlightControlModule::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    clear_ned_velocity();
    if (offboard_thread_.joinable()) {
        offboard_thread_.join();
    }
    if (state_thread_.joinable()) {
        state_thread_.join();
    }
    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }
    detach_system();
    mavsdk_.reset();
}

void FlightControlModule::release() {}

void FlightControlModule::monitor_loop() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kMonitorIntervalMs));
        const std::vector<std::shared_ptr<mavsdk::System>> systems = mavsdk_->systems();
        if (!system_) {
            for (const auto& candidate : systems) {
                if (candidate->has_autopilot()) {
                    attach_system(candidate);
                    break;
                }
            }
            if (!system_) {
                const std::int64_t now = now_ms();
                if (now - last_wait_log_ms_ >= kWaitLogIntervalMs) {
                    last_wait_log_ms_ = now;
                    SPDLOG_INFO("等待飞控连接: {}", connection_url());
                }
            }
            continue;
        }
        bool found = false;
        for (const auto& candidate : systems) {
            if (candidate.get() == system_.get()) {
                found = true;
                break;
            }
        }
        if (!found) {
            detach_system();
        }
    }
}

void FlightControlModule::attach_system(const std::shared_ptr<mavsdk::System>& system) {
    system_ = system;
    action_ = std::make_unique<mavsdk::Action>(system_);
    telemetry_ = std::make_unique<mavsdk::Telemetry>(system_);
    mission_ = std::make_unique<mavsdk::Mission>(system_);
    offboard_ = std::make_unique<mavsdk::Offboard>(system_);

    telemetry_->subscribe_position([this](mavsdk::Telemetry::Position value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.latitude_deg = value.latitude_deg;
        cache_.longitude_deg = value.longitude_deg;
        cache_.relative_altitude_m = value.relative_altitude_m;
        cache_.absolute_altitude_m = value.absolute_altitude_m;
    });
    telemetry_->subscribe_home([this](mavsdk::Telemetry::Position value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.home_latitude_deg = value.latitude_deg;
        cache_.home_longitude_deg = value.longitude_deg;
        cache_.home_relative_altitude_m = value.relative_altitude_m;
        cache_.home_absolute_altitude_m = value.absolute_altitude_m;
    });
    telemetry_->subscribe_attitude_euler([this](mavsdk::Telemetry::EulerAngle value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.roll_deg = value.roll_deg;
        cache_.pitch_deg = value.pitch_deg;
        cache_.yaw_deg = value.yaw_deg;
    });
    telemetry_->subscribe_velocity_ned([this](mavsdk::Telemetry::VelocityNed value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.vn_m_s = value.north_m_s;
        cache_.ve_m_s = value.east_m_s;
        cache_.vd_m_s = value.down_m_s;
    });
    telemetry_->subscribe_fixedwing_metrics([this](mavsdk::Telemetry::FixedwingMetrics value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.airspeed_m_s = value.airspeed_m_s;
    });
    telemetry_->subscribe_battery([this](mavsdk::Telemetry::Battery value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.voltage_v = value.voltage_v;
    });
    telemetry_->subscribe_flight_mode([this](mavsdk::Telemetry::FlightMode value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.flight_mode = value;
    });
    telemetry_->subscribe_armed([this](bool value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.armed = value;
    });
    telemetry_->subscribe_in_air([this](bool value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.in_air = value;
    });
    mission_->subscribe_mission_progress([this](mavsdk::Mission::MissionProgress value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.mission_current = value.current;
        cache_.mission_total = value.total;
    });
    system_->subscribe_is_connected([this](bool connected) {
        connected_ = connected;
        SPDLOG_INFO("飞控连接{}", connected ? "建立" : "异常");
    });

    SPDLOG_INFO("飞控系统已接入: system_id={}", system_->get_system_id());
}

void FlightControlModule::detach_system() {
    {
        std::lock_guard<std::mutex> lock(offboard_mutex_);
        offboard_active_ = false;
        offboard_.reset();
    }
    connected_ = false;
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.armed = false;
        cache_.in_air = false;
        cache_.mission_current = 0;
        cache_.mission_total = 0;
    }
    mission_.reset();
    telemetry_.reset();
    action_.reset();
    if (system_) {
        SPDLOG_INFO("飞控系统已移除");
        system_.reset();
    }
}

void FlightControlModule::state_loop() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kStateIntervalMs));
        plane::model::VehicleState state;
        state.planeId = config_.plane_id;
        {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            state.geoPos.latitude = cache_.latitude_deg;
            state.geoPos.longitude = cache_.longitude_deg;
            state.geoPos.height = cache_.relative_altitude_m;
            state.geoPos.altitude = cache_.absolute_altitude_m;
            state.startPos.latitude = cache_.home_latitude_deg;
            state.startPos.longitude = cache_.home_longitude_deg;
            state.startPos.height = cache_.home_relative_altitude_m;
            state.startPos.altitude = cache_.home_absolute_altitude_m;
            state.roll_deg = cache_.roll_deg;
            state.pitch_deg = cache_.pitch_deg;
            state.yaw_deg = cache_.yaw_deg;
            state.vn_m_s = cache_.vn_m_s;
            state.ve_m_s = cache_.ve_m_s;
            state.vd_m_s = cache_.vd_m_s;
            state.airspeed_m_s = cache_.airspeed_m_s;
            state.voltage_v = cache_.voltage_v;
            state.flightMode = std::string(magic_enum::enum_name(cache_.flight_mode));
            state.fcLinkState = connected_ ? "connected" : "disconnected";
        }
        state_store_.update_own_state(state);
    }
}

void FlightControlModule::offboard_loop() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kOffboardIntervalMs));
        std::lock_guard<std::mutex> lock(offboard_mutex_);
        if (offboard_active_ && offboard_) {
            offboard_->set_velocity_ned(setpoint_);
        }
    }
}

bool FlightControlModule::ensure_connected(std::string& error) const {
    if (!system_ || !action_ || !connected_) {
        error = "飞控未连接";
        return false;
    }
    return true;
}

bool FlightControlModule::arm(std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    const mavsdk::Action::Result result = action_->arm();
    if (result != mavsdk::Action::Result::Success) {
        error = "Arm 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    return true;
}

bool FlightControlModule::disarm(std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    const mavsdk::Action::Result result = action_->disarm();
    if (result != mavsdk::Action::Result::Success) {
        error = "Disarm 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    return true;
}

bool FlightControlModule::takeoff(float height_m, std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    mavsdk::Action::Result result = action_->set_takeoff_altitude(height_m);
    if (result != mavsdk::Action::Result::Success) {
        error = "设置起飞高度失败: " + result_text(result);
        report_error(error);
        return false;
    }
    result = action_->takeoff();
    if (result != mavsdk::Action::Result::Success) {
        error = "Takeoff 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.takeoff_altitude_m = height_m;
    }
    return true;
}

bool FlightControlModule::land(std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    const mavsdk::Action::Result result = action_->land();
    if (result != mavsdk::Action::Result::Success) {
        error = "Land 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    return true;
}

bool FlightControlModule::return_to_launch(std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    const mavsdk::Action::Result result = action_->return_to_launch();
    if (result != mavsdk::Action::Result::Success) {
        error = "RTL 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    return true;
}

bool FlightControlModule::hold(std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    const mavsdk::Action::Result result = action_->hold();
    if (result != mavsdk::Action::Result::Success) {
        error = "Hold 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    return true;
}

bool FlightControlModule::stop_motor(std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    const mavsdk::Action::Result result = action_->kill();
    if (result != mavsdk::Action::Result::Success) {
        error = "StopMotor 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    return true;
}

bool FlightControlModule::goto_waypoint(const plane::model::GeoPoint& point, std::string& error) {
    return upload_and_start_mission({point}, error);
}

bool FlightControlModule::execute_route(const std::vector<plane::model::GeoPoint>& points, std::string& error) {
    return upload_and_start_mission(points, error);
}

bool FlightControlModule::upload_and_start_mission(const std::vector<plane::model::GeoPoint>& points,
                                                   std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    if (points.empty()) {
        error = "航点列表为空";
        return false;
    }
    mavsdk::Mission::MissionPlan plan;
    for (const auto& point : points) {
        mavsdk::Mission::MissionItem item;
        item.latitude_deg = point.latitude;
        item.longitude_deg = point.longitude;
        item.relative_altitude_m = point.height;
        item.speed_m_s = static_cast<float>(config_.fc_route_speed_m_s);
        item.is_fly_through = true;
        plan.mission_items.push_back(item);
    }
    mavsdk::Mission::Result result = mission_->upload_mission(plan);
    if (result != mavsdk::Mission::Result::Success) {
        error = "上传航线失败: " + result_text(result);
        report_error(error);
        return false;
    }
    result = mission_->start_mission();
    if (result != mavsdk::Mission::Result::Success) {
        error = "启动航线失败: " + result_text(result);
        report_error(error);
        return false;
    }
    return true;
}

bool FlightControlModule::set_ned_velocity(const plane::model::NedVelocity& velocity, std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    mavsdk::Offboard::VelocityNedYaw setpoint;
    setpoint.north_m_s = static_cast<float>(velocity.north_m_s);
    setpoint.east_m_s = static_cast<float>(velocity.east_m_s);
    setpoint.down_m_s = static_cast<float>(velocity.down_m_s);
    setpoint.yaw_deg = static_cast<float>(velocity.yaw_deg);

    std::lock_guard<std::mutex> lock(offboard_mutex_);
    setpoint_ = setpoint;
    if (offboard_active_) {
        return true;
    }
    mavsdk::Offboard::Result result = offboard_->set_velocity_ned(setpoint_);
    if (result != mavsdk::Offboard::Result::Success) {
        error = "Offboard 设定值下发失败: " + result_text(result);
        report_error(error);
        return false;
    }
    result = offboard_->start();
    if (result != mavsdk::Offboard::Result::Success) {
        error = "Offboard 启动失败: " + result_text(result);
        report_error(error);
        return false;
    }
    offboard_active_ = true;
    SPDLOG_INFO("Offboard 已启动");
    return true;
}

void FlightControlModule::clear_ned_velocity() {
    std::lock_guard<std::mutex> lock(offboard_mutex_);
    if (!offboard_active_) {
        return;
    }
    offboard_active_ = false;
    if (offboard_) {
        offboard_->stop();
        SPDLOG_INFO("Offboard 已停止");
    }
}

std::string FlightControlModule::link_state() const { return connected_ ? "connected" : "disconnected"; }

plane::flight_control::FlightStatus FlightControlModule::flight_status() const {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    plane::flight_control::FlightStatus status;
    status.armed = cache_.armed;
    status.in_air = cache_.in_air;
    status.mission_current = cache_.mission_current;
    status.mission_total = cache_.mission_total;
    status.takeoff_altitude_m = cache_.takeoff_altitude_m;
    return status;
}

void FlightControlModule::report_error(const std::string& detail) const {
    FlightControlError event;
    event.detail = detail;
    event_bus_.publish(events::kError, event);
}

}  // namespace plane::flight_control
