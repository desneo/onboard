#include "flight_control/flight_control_module.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <cmath>
#include <future>
#include <magic_enum/magic_enum.hpp>
#include <utility>

#include "core/time_utils.h"
#include "flight_control/attitude_conversion.h"
#include "flight_control/flight_control_events.h"

namespace plane::flight_control {

using plane::core::now_ms;

namespace {

constexpr int kMonitorIntervalMs = 200;
constexpr int kStateIntervalMs = 100;
constexpr int kOffboardIntervalMs = 50;
constexpr int kOffboardRestartIntervalMs = 1000;
constexpr int kWaitLogIntervalMs = 5000;
constexpr int kFlightControlParamsRetryIntervalMs = 5000;

template <typename Result>
std::string result_text(Result result) {
    return std::string(magic_enum::enum_name(result));
}

/// 以 promise/future 包裹 MAVSDK 异步接口，阻塞直到回调返回真实结果
/// （与 MAVSDK 自身阻塞版实现一致，见官方 action_impl.cpp）。
template <typename Result, typename Register>
Result await_async(Register&& register_callback) {
    std::promise<Result> promise;
    std::future<Result> future = promise.get_future();
    register_callback([&promise](Result result) { promise.set_value(result); });
    return future.get();
}

}  // namespace

FlightControlModule::FlightControlModule(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                                         plane::state::StateStoreApi& state_store)
    : event_bus_(event_bus), config_(config), state_store_(state_store) {}

FlightControlModule::~FlightControlModule() { stop(); }

std::string FlightControlModule::name() const { return "飞行控制"; }

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
    SPDLOG_INFO("飞控连接参数: url={} fc_heartbeat_timeout_ms={}", url, config_.fc_heartbeat_timeout_ms);

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
    clear_offboard();
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
            continue;
        }
        refresh_flight_control_params_cache_if_needed();
    }
}

void FlightControlModule::attach_system(const std::shared_ptr<mavsdk::System>& system) {
    system_ = system;
    // subscribe_is_connected 只保证通知后续变化；接入时系统可能已经连接，
    // 因此必须先同步当前状态，避免 connected_ 永久保留默认 false。
    connected_ = system_->is_connected();
    action_ = std::make_unique<mavsdk::Action>(system_);
    telemetry_ = std::make_unique<mavsdk::Telemetry>(system_);
    mission_ = std::make_unique<mavsdk::Mission>(system_);
    offboard_ = std::make_unique<mavsdk::Offboard>(system_);
    param_ = std::make_unique<mavsdk::Param>(system_);

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
        mavsdk::Telemetry::FlightMode previous = mavsdk::Telemetry::FlightMode::Unknown;
        {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            previous = cache_.flight_mode;
            cache_.flight_mode = value;
        }
        if (value != previous) {
            SPDLOG_INFO("飞控飞行模式变化: previous={} value={}", magic_enum::enum_name(previous),
                        magic_enum::enum_name(value));
        }
    });
    telemetry_->subscribe_armed([this](bool value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        // SPDLOG_INFO("解锁状态变化: {}", value ? "解锁" : "上锁");
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
        if (!connected) {
            invalidate_flight_control_params_cache("飞控连接中断");
        } else {
            std::lock_guard<std::mutex> lock(flight_control_params_mutex_);
            last_flight_control_params_attempt_ms_ = 0;
        }
        SPDLOG_INFO("飞控连接{}", connected ? "建立" : "异常");
    });

    SPDLOG_INFO("飞控系统已接入: system_id={} connected={}", system_->get_system_id(), connected_ ? "true" : "false");
    refresh_flight_control_params_cache_if_needed();
}

void FlightControlModule::detach_system() {
    {
        std::lock_guard<std::mutex> lock(offboard_mutex_);
        offboard_active_ = false;
        offboard_setpoint_kind_ = OffboardSetpointKind::None;
        last_offboard_start_ms_ = 0;
        offboard_.reset();
    }
    connected_ = false;
    invalidate_flight_control_params_cache("飞控系统已移除");
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.armed = false;
        cache_.in_air = false;
        cache_.mission_current = 0;
        cache_.mission_total = 0;
    }
    mission_.reset();
    param_.reset();
    telemetry_.reset();
    action_.reset();
    if (system_) {
        SPDLOG_INFO("飞控系统已移除");
        system_.reset();
    }
}

void FlightControlModule::invalidate_flight_control_params_cache(const std::string& reason) {
    std::lock_guard<std::mutex> lock(flight_control_params_mutex_);
    flight_control_params_ready_ = false;
    flight_control_params_error_ = reason;
    last_flight_control_params_attempt_ms_ = 0;
}

void FlightControlModule::refresh_flight_control_params_cache_if_needed() {
    if (!connected_ || !param_) {
        return;
    }
    const std::int64_t now = now_ms();
    {
        std::lock_guard<std::mutex> lock(flight_control_params_mutex_);
        if (flight_control_params_ready_ ||
            (last_flight_control_params_attempt_ms_ > 0 &&
             now - last_flight_control_params_attempt_ms_ < kFlightControlParamsRetryIntervalMs)) {
            return;
        }
        last_flight_control_params_attempt_ms_ = now;
    }

    std::string error;
    auto read_positive = [this, &error](const char* name, double& out) {
        const auto [result, value] = param_->get_param_float(name);
        if (result != mavsdk::Param::Result::Success) {
            error = std::string("读取 PX4 参数失败: ") + name + " (" + result_text(result) + ")";
            return false;
        }
        if (!std::isfinite(value) || value <= 0.0F) {
            error = std::string("PX4 参数必须为有限正数: ") + name;
            return false;
        }
        out = value;
        return true;
    };

    FlightControlParams value;
    bool ok = read_positive("MPC_XY_VEL_MAX", value.mpc_xy_vel_max) &&
              read_positive("MPC_Z_VEL_MAX_UP", value.mpc_z_vel_max_up) &&
              read_positive("MPC_Z_VEL_MAX_DN", value.mpc_z_vel_max_dn) &&
              read_positive("MPC_ACC_HOR_MAX", value.mpc_acc_hor_max);
    if (ok && !connected_) {
        ok = false;
        error = "读取 PX4 参数期间飞控连接中断";
    }
    {
        std::lock_guard<std::mutex> lock(flight_control_params_mutex_);
        if (ok) {
            flight_control_params_cache_ = value;
            flight_control_params_ready_ = true;
            flight_control_params_error_.clear();
        } else {
            flight_control_params_ready_ = false;
            flight_control_params_error_ = error;
        }
    }
    if (!ok) {
        SPDLOG_WARN("飞控系统参数缓存失败，{} 毫秒后重试: {}", kFlightControlParamsRetryIntervalMs, error);
        return;
    }
    SPDLOG_INFO(
        "飞控系统参数已缓存: MPC_XY_VEL_MAX={:.2f} MPC_Z_VEL_MAX_UP={:.2f} "
        "MPC_Z_VEL_MAX_DN={:.2f} MPC_ACC_HOR_MAX={:.2f}",
        value.mpc_xy_vel_max, value.mpc_z_vel_max_up, value.mpc_z_vel_max_dn, value.mpc_acc_hor_max);
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
            if (offboard_setpoint_kind_ == OffboardSetpointKind::VelocityNed) {
                offboard_->set_velocity_ned(velocity_setpoint_);
            } else if (offboard_setpoint_kind_ == OffboardSetpointKind::Attitude) {
                offboard_->set_attitude(attitude_setpoint_);
            }
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

namespace {

/// 执行一次 Action 异步调用并返回真实结果。
template <typename Register>
mavsdk::Action::Result await_action(Register&& register_callback) {
    return await_async<mavsdk::Action::Result>(std::forward<Register>(register_callback));
}

}  // namespace

bool FlightControlModule::arm(std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    const auto result = await_action([this](mavsdk::Action::ResultCallback cb) { action_->arm_async(std::move(cb)); });
    if (result != mavsdk::Action::Result::Success) {
        error = "下发飞控Arm 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    SPDLOG_INFO("下发飞控指令成功: 解锁（Arm）");
    return true;
}

bool FlightControlModule::disarm(std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    const auto result =
        await_action([this](mavsdk::Action::ResultCallback cb) { action_->disarm_async(std::move(cb)); });
    if (result != mavsdk::Action::Result::Success) {
        error = "下发飞控Disarm 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    SPDLOG_INFO("下发飞控指令成功: 上锁（Disarm）");
    return true;
}

bool FlightControlModule::takeoff(float height_m, std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    const auto set_result = await_action([this, height_m](mavsdk::Action::ResultCallback cb) {
        action_->set_takeoff_altitude_async(height_m, std::move(cb));
    });
    if (set_result != mavsdk::Action::Result::Success) {
        error = "下发飞控SetTakeoffAltitude 失败: " + result_text(set_result);
        report_error(error);
        return false;
    }
    const auto takeoff_result =
        await_action([this](mavsdk::Action::ResultCallback cb) { action_->takeoff_async(std::move(cb)); });
    if (takeoff_result != mavsdk::Action::Result::Success) {
        error = "下发飞控Takeoff 失败: " + result_text(takeoff_result);
        report_error(error);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.takeoff_altitude_m = height_m;
    }
    SPDLOG_INFO("下发飞控指令成功: 起飞（Takeoff） height_m={:.1f}", height_m);
    return true;
}

bool FlightControlModule::land(std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    const auto result = await_action([this](mavsdk::Action::ResultCallback cb) { action_->land_async(std::move(cb)); });
    if (result != mavsdk::Action::Result::Success) {
        error = "下发飞控Land 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    SPDLOG_INFO("下发飞控指令成功: 降落（Land）");
    return true;
}

bool FlightControlModule::return_to_launch(std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    const auto result =
        await_action([this](mavsdk::Action::ResultCallback cb) { action_->return_to_launch_async(std::move(cb)); });
    if (result != mavsdk::Action::Result::Success) {
        error = "下发飞控ReturnToLaunch 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    SPDLOG_INFO("下发飞控指令成功: 返航（ReturnToLaunch）");
    return true;
}

bool FlightControlModule::hold(std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    const auto result = await_action([this](mavsdk::Action::ResultCallback cb) { action_->hold_async(std::move(cb)); });
    if (result != mavsdk::Action::Result::Success) {
        error = "下发飞控Hold 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    SPDLOG_INFO("下发飞控指令成功: 保持（Hold）");
    return true;
}

bool FlightControlModule::stop_motor(std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    const auto result = await_action([this](mavsdk::Action::ResultCallback cb) { action_->kill_async(std::move(cb)); });
    if (result != mavsdk::Action::Result::Success) {
        error = "下发飞控StopMotor 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    SPDLOG_INFO("下发飞控指令成功: 停桨（StopMotor）");
    return true;
}

bool FlightControlModule::goto_waypoint(const plane::model::GeoPoint& point, std::string& error) {
    return upload_and_start_mission({point}, false, error);
}

bool FlightControlModule::execute_route(const std::vector<plane::model::GeoPoint>& points, std::string& error) {
    return upload_and_start_mission(points, true, error);
}

bool FlightControlModule::upload_and_start_mission(const std::vector<plane::model::GeoPoint>& points, bool fly_through,
                                                   std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    if (points.empty()) {
        error = "航点列表为空";
        return false;
    }
    // MAVSDK 在新任务上传/启动完成前仍可能保留上一条任务的完成进度。
    // 先清空缓存，避免 MissionNode 将旧的 current == total 误判为本次任务已完成。
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.mission_current = 0;
        cache_.mission_total = 0;
    }
    mavsdk::Mission::MissionPlan plan;
    for (const auto& point : points) {
        mavsdk::Mission::MissionItem item;
        item.latitude_deg = point.latitude;
        item.longitude_deg = point.longitude;
        item.relative_altitude_m = point.height;
        item.speed_m_s = static_cast<float>(config_.fc_route_speed_m_s);
        item.is_fly_through = fly_through;
        plan.mission_items.push_back(item);
    }
    const auto upload_result =
        await_async<mavsdk::Mission::Result>([this, plan](mavsdk::Mission::ResultCallback cb) mutable {
            mission_->upload_mission_async(std::move(plan), std::move(cb));
        });
    if (upload_result != mavsdk::Mission::Result::Success) {
        error = "下发飞控UploadMission 失败: " + result_text(upload_result);
        report_error(error);
        return false;
    }
    SPDLOG_INFO("下发飞控指令成功: 上传任务（UploadMission） points.size={} fc_route_speed_m_s={:.1f} fly_through={}",
                points.size(), config_.fc_route_speed_m_s, fly_through ? "true" : "false");
    const auto start_result = await_async<mavsdk::Mission::Result>(
        [this](mavsdk::Mission::ResultCallback cb) { mission_->start_mission_async(std::move(cb)); });
    if (start_result != mavsdk::Mission::Result::Success) {
        error = "下发飞控StartMission 失败: " + result_text(start_result);
        report_error(error);
        return false;
    }
    // start_mission_async 返回前可能还没有收到新任务的首个进度回调。此处再次
    // 建立本次任务的初始进度，确保紧随其后的行为树 tick 必须等待新任务推进。
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.mission_current = 0;
        cache_.mission_total = static_cast<int>(points.size());
    }
    SPDLOG_INFO("下发飞控指令成功: 启动任务（StartMission）");
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
    velocity_setpoint_ = setpoint;
    offboard_setpoint_kind_ = OffboardSetpointKind::VelocityNed;
    if (offboard_active_) {
        return true;
    }
    const mavsdk::Offboard::Result result = offboard_->set_velocity_ned(velocity_setpoint_);
    if (result != mavsdk::Offboard::Result::Success) {
        error = "Offboard 设定值下发失败: " + result_text(result);
        report_error(error);
        return false;
    }
    const auto start_result = await_async<mavsdk::Offboard::Result>(
        [this](mavsdk::Offboard::ResultCallback cb) { offboard_->start_async(std::move(cb)); });
    if (start_result != mavsdk::Offboard::Result::Success) {
        error = "下发飞控OffboardStart 失败: " + result_text(start_result);
        report_error(error);
        return false;
    }
    offboard_active_ = true;
    last_offboard_start_ms_ = now_ms();
    SPDLOG_INFO(
        "下发飞控指令成功: Offboard 速度启动（north_m_s={:.2f} east_m_s={:.2f} "
        "down_m_s={:.2f} yaw_deg={:.1f}）",
        velocity_setpoint_.north_m_s, velocity_setpoint_.east_m_s, velocity_setpoint_.down_m_s,
        velocity_setpoint_.yaw_deg);
    return true;
}

bool FlightControlModule::set_attitude(const AttitudeSetpoint& command, std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    EulerAttitudeSetpoint converted;
    if (!convert_attitude_setpoint(command, converted, error)) {
        return false;
    }

    mavsdk::Offboard::Attitude setpoint;
    setpoint.roll_deg = converted.roll_deg;
    setpoint.pitch_deg = converted.pitch_deg;
    setpoint.yaw_deg = converted.yaw_deg;
    setpoint.thrust_value = converted.thrust;

    bool telemetry_offboard = false;
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        telemetry_offboard = cache_.flight_mode == mavsdk::Telemetry::FlightMode::Offboard;
    }

    std::lock_guard<std::mutex> lock(offboard_mutex_);
    attitude_setpoint_ = setpoint;
    offboard_setpoint_kind_ = OffboardSetpointKind::Attitude;
    const mavsdk::Offboard::Result result = offboard_->set_attitude(attitude_setpoint_);
    if (result != mavsdk::Offboard::Result::Success) {
        error = "Offboard Attitude 设定值下发失败: " + result_text(result);
        report_error(error);
        return false;
    }
    const std::int64_t now = now_ms();
    if (offboard_active_ && telemetry_offboard) {
        return true;
    }
    if (offboard_active_ && now - last_offboard_start_ms_ < kOffboardRestartIntervalMs) {
        return true;
    }
    const bool restarting = offboard_active_;
    const auto start_result = await_async<mavsdk::Offboard::Result>(
        [this](mavsdk::Offboard::ResultCallback cb) { offboard_->start_async(std::move(cb)); });
    if (start_result != mavsdk::Offboard::Result::Success) {
        error = "下发飞控 OffboardStart 失败: " + result_text(start_result);
        report_error(error);
        return false;
    }
    offboard_active_ = true;
    last_offboard_start_ms_ = now;
    SPDLOG_INFO(
        "下发飞控指令成功: Offboard 姿态{}（roll_deg={:.2f} pitch_deg={:.2f} "
        "yaw_deg={:.2f} thrust_value={:.3f}）",
        restarting ? "重新启动" : "启动", attitude_setpoint_.roll_deg, attitude_setpoint_.pitch_deg,
        attitude_setpoint_.yaw_deg, attitude_setpoint_.thrust_value);
    return true;
}

bool FlightControlModule::get_cached_flight_control_params(FlightControlParams& params, std::string& error) const {
    std::lock_guard<std::mutex> lock(flight_control_params_mutex_);
    if (!flight_control_params_ready_) {
        error = "飞控系统参数缓存未就绪: " + flight_control_params_error_;
        return false;
    }
    params = flight_control_params_cache_;
    return true;
}

void FlightControlModule::clear_offboard() {
    std::lock_guard<std::mutex> lock(offboard_mutex_);
    offboard_setpoint_kind_ = OffboardSetpointKind::None;
    last_offboard_start_ms_ = 0;
    if (!offboard_active_) {
        return;
    }
    offboard_active_ = false;
    if (offboard_) {
        const auto stop_result = await_async<mavsdk::Offboard::Result>(
            [this](mavsdk::Offboard::ResultCallback cb) { offboard_->stop_async(std::move(cb)); });
        if (stop_result != mavsdk::Offboard::Result::Success) {
            report_error("下发飞控OffboardStop 失败: " + result_text(stop_result));
        } else {
            SPDLOG_INFO("下发飞控指令成功: Offboard 停止");
        }
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
    SPDLOG_ERROR("飞控错误: {}", detail);
    FlightControlError event;
    event.detail = detail;
    event_bus_.publish(events::kError, event);
}

}  // namespace plane::flight_control
