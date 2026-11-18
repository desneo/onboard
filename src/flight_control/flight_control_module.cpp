#include "flight_control/flight_control_module.h"

#include <mavsdk/mavlink_include.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <magic_enum/magic_enum.hpp>
#include <utility>

#include "algorithm/midcourse_alg.h"
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
constexpr double kLandedStateRateHz = 2.0;
// 场景 02 的机载 MAVLink 链路会按带宽压低遥测速率。
constexpr double kSimLandedStateRateHz = 10.0;
constexpr auto kLandedStateStreamRetryInterval = std::chrono::seconds(5);
constexpr double kPi = 3.14159265358979323846;

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

bool is_flight_output(const mavlink_message_t& message) {
    switch (message.msgid) {
        case MAVLINK_MSG_ID_COMMAND_LONG:
        case MAVLINK_MSG_ID_COMMAND_INT: {
            const auto command = message.msgid == MAVLINK_MSG_ID_COMMAND_LONG
                                     ? mavlink_msg_command_long_get_command(&message)
                                     : mavlink_msg_command_int_get_command(&message);
            switch (command) {
                case MAV_CMD_NAV_TAKEOFF:
                case MAV_CMD_NAV_LAND:
                case MAV_CMD_NAV_RETURN_TO_LAUNCH:
                case MAV_CMD_COMPONENT_ARM_DISARM:
                case MAV_CMD_DO_SET_MODE:
                case MAV_CMD_DO_SET_STANDARD_MODE:
                case MAV_CMD_DO_REPOSITION:
                case MAV_CMD_DO_FLIGHTTERMINATION:
                case MAV_CMD_DO_PAUSE_CONTINUE:
                case MAV_CMD_MISSION_START:
                    return true;
                default:
                    return false;
            }
        }
        case MAVLINK_MSG_ID_SET_MODE:
        case MAVLINK_MSG_ID_MISSION_COUNT:
        case MAVLINK_MSG_ID_MISSION_ITEM:
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
        // Takeoff 在切换模式前会先写入 PX4 的起飞高度。
        case MAVLINK_MSG_ID_PARAM_SET:
        case MAVLINK_MSG_ID_PARAM_EXT_SET:
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
        case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
            return true;
        default:
            return false;
    }
}

}  // namespace

FlightControlModule::FlightControlModule(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                                         plane::state::StateStoreApi& state_store)
    : event_bus_(event_bus), config_(config), state_store_(state_store) {}

FlightControlModule::~FlightControlModule() { stop(); }

std::string FlightControlModule::name() const { return "飞行控制"; }

bool FlightControlModule::init() {
    SPDLOG_INFO("模块初始化完成: {}", name());
    return true;
}

std::string FlightControlModule::connection_url() const {
    if (config_.fc_link_type == "serial") {
        return "serial://" + config_.fc_serial_device + ":" + std::to_string(config_.fc_serial_baud);
    }
    return config_.fc_udp_url;
}

bool FlightControlModule::start() {
    // PX4 SITL 将 MAVSDK 的 GroundStation 心跳作为通过 GCS 预检的依据。真实机仍以
    // CompanionComputer 身份接入，避免仿真专用身份进入实机链路。
    const mavsdk::ComponentType component_type =
        config_.is_sim ? mavsdk::ComponentType::GroundStation : mavsdk::ComponentType::CompanionComputer;
    mavsdk_ = std::make_unique<mavsdk::Mavsdk>(mavsdk::Mavsdk::Configuration{component_type});
    mavsdk_->intercept_outgoing_messages_async(
        [this](mavlink_message_t& message) { return allow_outgoing_message(message); });
    SPDLOG_INFO("飞控会话身份: {}{}", config_.is_sim ? "GroundStation" : "CompanionComputer",
                config_.is_sim ? "（仿真提供 GCS 心跳）" : "（实机机载计算机）");
    mavsdk_->set_heartbeat_timeout_s(static_cast<double>(config_.fc_heartbeat_timeout_ms) / 1000.0);

    const std::string url = connection_url();
    const mavsdk::ConnectionResult result = mavsdk_->add_any_connection(url);
    if (result != mavsdk::ConnectionResult::Success) {
        report_error("飞控连接初始化失败: " + url + " (" + result_text(result) + ")");
        return false;
    }
    SPDLOG_INFO("飞控连接参数: url={} fc_heartbeat_timeout_ms={}", url, config_.fc_heartbeat_timeout_ms);

    running_ = true;
    manual_takeover_thread_ = std::thread([this]() { manual_takeover_loop(); });
    monitor_thread_ = std::thread([this]() { monitor_loop(); });
    state_thread_ = std::thread([this]() { state_loop(); });
    offboard_thread_ = std::thread([this]() { offboard_loop(); });
    SPDLOG_INFO("模块启动完成: {}", name());
    return true;
}

void FlightControlModule::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(control_mutex_);
        control_cv_.notify_all();
    }
    if (manual_takeover_thread_.joinable()) {
        manual_takeover_thread_.join();
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

void FlightControlModule::begin_control(std::uint64_t execution_id) {
    std::lock_guard<std::mutex> lock(control_mutex_);
    execution_id_ = execution_id;
    control_takeoff_height_m_ = 0.0F;
    control_mission_total_ = 0;
    manual_takeover_.start();
}

bool FlightControlModule::end_control(std::uint64_t execution_id, bool acknowledge_takeover) {
    std::unique_lock<std::mutex> lock(control_mutex_);
    control_cv_.wait(lock, [this, execution_id]() {
        return !running_ || execution_id_ != execution_id || control_gate_ != ControlGate::CheckingHold;
    });
    if (execution_id_ != execution_id) {
        return true;
    }
    if (control_gate_ == ControlGate::Takeover && !acknowledge_takeover) {
        return false;
    }
    execution_id_ = 0;
    manual_takeover_.finish();
    takeover_event_.reset();
    control_gate_ = ControlGate::Open;
    return true;
}

bool FlightControlModule::control_can_advance() const { return control_gate_ == ControlGate::Open; }

bool FlightControlModule::wait_for_control(std::string& error) {
    if (control_can_advance()) {
        return true;
    }
    std::unique_lock<std::mutex> lock(control_mutex_);
    control_cv_.wait(lock, [this]() { return !running_ || control_gate_ != ControlGate::CheckingHold; });
    if (control_can_advance()) {
        return true;
    }
    error = "当前执行因飞控模式切换已中止";
    return false;
}

bool FlightControlModule::expect_action(ManualTakeover::Action action, std::string& error, float takeoff_height_m,
                                        int mission_total) {
    std::unique_lock<std::mutex> lock(control_mutex_);
    control_cv_.wait(lock, [this]() { return !running_ || control_gate_ != ControlGate::CheckingHold; });
    if (!control_can_advance()) {
        error = "当前执行因飞控模式切换已中止";
        return false;
    }
    if (execution_id_ != 0) {
        const auto now = ManualTakeover::Clock::now();
        if (action == ManualTakeover::Action::Takeoff) {
            takeoff_requested_at_ = now;
            control_takeoff_height_m_ = takeoff_height_m;
        } else if (action == ManualTakeover::Action::Mission) {
            mission_requested_at_ = now;
            control_mission_total_ = mission_total;
        }
        manual_takeover_.expect(action, now);
    }
    return true;
}

void FlightControlModule::action_failed(ManualTakeover::Action action) {
    std::lock_guard<std::mutex> lock(control_mutex_);
    manual_takeover_.action_failed(action);
    if (control_gate_ == ControlGate::CheckingHold) {
        apply_takeover_result_locked(manual_takeover_.update(takeover_evidence_locked(), ManualTakeover::Clock::now()));
    }
}

ManualTakeover::Evidence FlightControlModule::takeover_evidence_locked() const {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return {control_takeoff_height_m_ > 0.0F && cache_.position_updated >= takeoff_requested_at_ && cache_.in_air &&
                cache_.relative_altitude_m >= control_takeoff_height_m_ - config_.bt_takeoff_tolerance_m,
            control_mission_total_ > 0 && cache_.mission_progress_updated >= mission_requested_at_ &&
                cache_.mission_total == control_mission_total_ && cache_.mission_current >= cache_.mission_total};
}

void FlightControlModule::apply_takeover_result_locked(const ManualTakeover::Result& result) {
    if (control_gate_ == ControlGate::Takeover) {
        return;
    }
    if (result.takeover && execution_id_ != 0) {
        control_gate_ = ControlGate::Takeover;
        takeover_event_ = ManualTakeoverEvent{execution_id_, std::string(magic_enum::enum_name(*result.takeover))};
    } else {
        control_gate_ = result.pending ? ControlGate::CheckingHold : ControlGate::Open;
    }
    control_cv_.notify_all();
}

void FlightControlModule::update_takeover_evidence() {
    if (control_gate_ != ControlGate::CheckingHold) {
        return;
    }
    std::lock_guard<std::mutex> lock(control_mutex_);
    apply_takeover_result_locked(manual_takeover_.update(takeover_evidence_locked(), ManualTakeover::Clock::now()));
}

void FlightControlModule::manual_takeover_loop() {
    std::unique_lock<std::mutex> lock(control_mutex_);
    while (running_) {
        if (takeover_event_) {
            auto event = std::move(*takeover_event_);
            takeover_event_.reset();
            lock.unlock();
            event_bus_.publish(events::kManualTakeover, event);
            lock.lock();
        } else if (const auto deadline = manual_takeover_.deadline();
                   deadline && control_gate_ == ControlGate::CheckingHold) {
            if (control_cv_.wait_until(lock, *deadline) == std::cv_status::timeout) {
                apply_takeover_result_locked(
                    manual_takeover_.update(takeover_evidence_locked(), ManualTakeover::Clock::now()));
            }
        } else {
            control_cv_.wait(lock);
        }
    }
}

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
        request_landed_state_stream_if_needed();
        refresh_flight_control_params_cache_if_needed();
    }
}

void FlightControlModule::attach_system(const std::shared_ptr<mavsdk::System>& system) {
    system_ = system;
    const auto generation = ++connection_generation_;
    // subscribe_is_connected 只保证通知后续变化；接入时系统可能已经连接，
    // 因此必须先同步当前状态，避免 connected_ 永久保留默认 false。
    action_ = std::make_unique<mavsdk::Action>(system_);
    telemetry_ = std::make_unique<mavsdk::Telemetry>(system_);
    mission_ = std::make_unique<mavsdk::Mission>(system_);
    {
        std::lock_guard<std::mutex> lock(offboard_mutex_);
        offboard_ = std::make_unique<mavsdk::Offboard>(system_);
    }
    param_ = std::make_unique<mavsdk::Param>(system_);
    auto self_check_link = std::make_shared<mavsdk::MavlinkPassthrough>(system_);
    const bool connected = system_->is_connected();
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        self_check_link_ = std::move(self_check_link);
        update_link_state_locked(connected);
    }

    telemetry_->subscribe_position([this, generation](mavsdk::Telemetry::Position value) {
        if (connection_generation_ != generation) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            cache_.latitude_deg = value.latitude_deg;
            cache_.longitude_deg = value.longitude_deg;
            cache_.relative_altitude_m = value.relative_altitude_m;
            cache_.absolute_altitude_m = value.absolute_altitude_m;
            cache_.position_updated = ManualTakeover::Clock::now();
        }
        update_takeover_evidence();
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
    telemetry_->subscribe_flight_mode([this, generation](mavsdk::Telemetry::FlightMode value) {
        mavsdk::Telemetry::FlightMode previous;
        {
            std::lock_guard<std::mutex> lock(control_mutex_);
            if (!connected_ || connection_generation_ != generation) {
                return;
            }
            const auto now = ManualTakeover::Clock::now();
            {
                std::lock_guard<std::mutex> cache_lock(cache_mutex_);
                previous = cache_.flight_mode;
                cache_.flight_mode = value;
                cache_.mode_updated = now;
            }
            apply_takeover_result_locked(manual_takeover_.observe(value, takeover_evidence_locked(), now));
        }
        if (value != previous) {
            SPDLOG_INFO("飞控飞行模式变化: previous={} value={}", magic_enum::enum_name(previous),
                        magic_enum::enum_name(value));
        }
    });
    telemetry_->subscribe_armed([this, generation](bool value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        if (!connected_ || connection_generation_ != generation) {
            return;
        }
        cache_.armed = value;
        cache_.armed_updated = ManualTakeover::Clock::now();
    });
    telemetry_->subscribe_landed_state([this, generation](mavsdk::Telemetry::LandedState value) {
        mavsdk::Telemetry::LandedState previous;
        {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            if (!connected_ || connection_generation_ != generation) {
                return;
            }
            previous = cache_.landed_state;
            cache_.landed_state = value;
            cache_.ground_updated = ManualTakeover::Clock::now();
        }
        if (value != previous) {
            SPDLOG_INFO("飞控着地状态变化：landed_state={}", magic_enum::enum_name(value));
        }
    });
    telemetry_->subscribe_in_air([this, generation](bool value) {
        if (connection_generation_ != generation) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            cache_.in_air = value;
        }
        update_takeover_evidence();
    });
    mission_->subscribe_mission_progress([this, generation](mavsdk::Mission::MissionProgress value) {
        if (connection_generation_ != generation) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            cache_.mission_current = value.current;
            cache_.mission_total = value.total;
            cache_.mission_progress_updated = ManualTakeover::Clock::now();
        }
        update_takeover_evidence();
    });
    system_->subscribe_is_connected([this, generation](bool connected) {
        {
            std::lock_guard<std::mutex> lock(control_mutex_);
            if (connection_generation_ != generation) {
                return;
            }
            {
                std::lock_guard<std::mutex> cache_lock(cache_mutex_);
                update_link_state_locked(connected);
            }
            if (!connected) {
                if (const auto deadline = manual_takeover_.deadline()) {
                    apply_takeover_result_locked(manual_takeover_.update({}, *deadline));
                }
                std::lock_guard<std::mutex> cache_lock(cache_mutex_);
                cache_.position_updated = {};
                cache_.mission_progress_updated = {};
            }
            manual_takeover_.reset_link();
            apply_takeover_result_locked({});
        }
        if (!connected) {
            invalidate_flight_control_params_cache("飞控连接中断");
        } else {
            std::lock_guard<std::mutex> lock(flight_control_params_mutex_);
            last_flight_control_params_attempt_ms_ = 0;
        }
        SPDLOG_INFO("飞控连接{}", connected ? "建立" : "异常");
    });

    SPDLOG_INFO("飞控系统已接入: system_id={} connected={}", system_->get_system_id(), connected_ ? "true" : "false");
    request_landed_state_stream_if_needed();
    refresh_flight_control_params_cache_if_needed();
}

void FlightControlModule::detach_system() {
    std::shared_ptr<mavsdk::MavlinkPassthrough> self_check_link;
    {
        std::lock_guard<std::mutex> lock(control_mutex_);
        ++connection_generation_;
        {
            std::lock_guard<std::mutex> cache_lock(cache_mutex_);
            update_link_state_locked(false);
            self_check_link = std::move(self_check_link_);
        }
        if (const auto deadline = manual_takeover_.deadline()) {
            apply_takeover_result_locked(manual_takeover_.update({}, *deadline));
        }
        manual_takeover_.reset_link();
        apply_takeover_result_locked({});
    }
    {
        std::lock_guard<std::mutex> lock(offboard_mutex_);
        offboard_active_ = false;
        offboard_setpoint_kind_ = OffboardSetpointKind::None;
        acceleration_setpoint_.reset();
        last_offboard_start_ms_ = 0;
        offboard_.reset();
    }
    invalidate_flight_control_params_cache("飞控系统已移除");
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.armed = false;
        cache_.in_air = false;
        cache_.mission_current = 0;
        cache_.mission_total = 0;
        cache_.position_updated = {};
        cache_.mission_progress_updated = {};
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

void FlightControlModule::update_link_state_locked(bool connected) {
    if (connected_ == connected) {
        return;
    }
    connected_ = connected;
    ++link_generation_;
    cache_.landed_state = mavsdk::Telemetry::LandedState::Unknown;
    cache_.armed_updated = {};
    cache_.ground_updated = {};
    cache_.mode_updated = {};
    last_landed_state_stream_request_ = {};
}

void FlightControlModule::request_landed_state_stream_if_needed() {
    const auto now = ManualTakeover::Clock::now();
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        if (!connected_ || self_check_action_) {
            return;
        }
        if (last_landed_state_stream_request_ != ManualTakeover::Clock::time_point{} &&
            (self_check_telemetry_fresh(cache_.ground_updated, now) ||
             now - last_landed_state_stream_request_ < kLandedStateStreamRetryInterval)) {
            return;
        }
        last_landed_state_stream_request_ = now;
    }
    const double rate_hz = config_.is_sim ? kSimLandedStateRateHz : kLandedStateRateHz;
    telemetry_->set_rate_landed_state_async(rate_hz, [](mavsdk::Telemetry::Result result) {
        if (result != mavsdk::Telemetry::Result::Success) {
            SPDLOG_WARN("请求飞控着地遥测失败（{}）", result_text(result));
        }
    });
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
        if (offboard_active_ && offboard_ && control_can_advance()) {
            if (offboard_setpoint_kind_ == OffboardSetpointKind::VelocityNed) {
                send_velocity_setpoint_locked();
            } else if (offboard_setpoint_kind_ == OffboardSetpointKind::Attitude) {
                offboard_->set_attitude(attitude_setpoint_);
            }
        }
    }
}

bool FlightControlModule::ensure_connected(std::string& error) {
    if (!wait_for_control(error)) {
        return false;
    }
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

SelfCheckArmResult FlightControlModule::self_check_arm(const FlightStatus& expected, const std::atomic<bool>* active,
                                                     std::string& error) {
    return run_self_check_action(expected, true, active, error);
}

bool FlightControlModule::self_check_disarm(const FlightStatus& expected, const std::atomic<bool>* active,
                                           std::string& error) {
    return run_self_check_action(expected, false, active, error) == SelfCheckArmResult::Accepted;
}

SelfCheckArmResult FlightControlModule::run_self_check_action(const FlightStatus& expected, bool arming,
                                                             const std::atomic<bool>* active, std::string& error) {
    std::shared_ptr<mavsdk::MavlinkPassthrough> link;
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        if (self_check_action_) {
            error = "自检飞控指令仍在下发";
            return SelfCheckArmResult::Rejected;
        }
        SelfCheckAction action{expected.link_generation, expected.flight_mode, arming, active, false, {}};
        error = self_check_action_error_locked(action);
        if (!error.empty()) {
            return SelfCheckArmResult::Rejected;
        }
        link = self_check_link_;
        self_check_action_ = std::move(action);
    }

    // MAVSDK Action::Arm 可能先切换飞行模式，自检仅发送常规解锁或上锁指令。
    mavsdk::MavlinkPassthrough::CommandLong command{};
    command.command = MAV_CMD_COMPONENT_ARM_DISARM;
    command.target_sysid = link->get_target_sysid();
    command.target_compid = link->get_target_compid();
    command.param1 = arming ? 1.0F : 0.0F;
    command.param2 = 0.0F;
    const auto result = link->send_command_long(command);

    SelfCheckAction action = [&]() {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        SelfCheckAction completed = std::move(*self_check_action_);
        self_check_action_.reset();
        return completed;
    }();
    using Result = mavsdk::MavlinkPassthrough::Result;
    if (action.allowed_send && result == Result::Success) {
        SPDLOG_INFO("自检飞控指令已确认（{}）", arming ? "Arm" : "Disarm");
        return SelfCheckArmResult::Accepted;
    }

    error = arming ? "飞控自检解锁指令失败" : "飞控自检上锁指令失败";
    error += "（" + result_text(result) + "）";
    if (!action.blocked_reason.empty()) {
        error += "，" + action.blocked_reason;
    }
    report_error(error);
    if (!action.allowed_send || result == Result::CommandDenied || result == Result::CommandTemporarilyRejected ||
        result == Result::CommandUnsupported || result == Result::CommandFailed) {
        return SelfCheckArmResult::Rejected;
    }
    return SelfCheckArmResult::Uncertain;
}

std::string FlightControlModule::self_check_action_error_locked(const SelfCheckAction& action) const {
    if (action.active && !action.active->load()) {
        return "自检已中止";
    }
    if (!control_can_advance()) {
        return "飞控控制权不可用";
    }
    const FlightStatus status = flight_status_locked();
    if (!status.connected || !self_check_link_) {
        return "飞控未连接";
    }
    if (status.link_generation != action.link_generation) {
        return "飞控连接已变化";
    }
    const auto now = ManualTakeover::Clock::now();
    if (auto error = self_check_ground_error(status, now, action.arming); !error.empty()) {
        return error;
    }
    if (status.mode_updated != ManualTakeover::Clock::time_point{} && status.flight_mode != action.flight_mode) {
        return "自检期间飞行模式已变化（flight_mode=" + status.flight_mode + "）";
    }
    if (action.arming && (!self_check_telemetry_fresh(status.armed_updated, now) || status.armed)) {
        return "未确认飞机处于上锁状态";
    }
    return {};
}

bool FlightControlModule::allow_outgoing_message(const mavlink_message_t& message) {
    std::optional<float> arm_param;
    if (message.msgid == MAVLINK_MSG_ID_COMMAND_LONG &&
        mavlink_msg_command_long_get_command(&message) == MAV_CMD_COMPONENT_ARM_DISARM) {
        arm_param = mavlink_msg_command_long_get_param1(&message);
    }
    if (!arm_param) {
        return control_can_advance() || !is_flight_output(message);
    }

    std::lock_guard<std::mutex> lock(cache_mutex_);
    if (!self_check_action_) {
        return control_can_advance();
    }
    auto& action = *self_check_action_;
    if (*arm_param != (action.arming ? 1.0F : 0.0F) ||
        mavlink_msg_command_long_get_param2(&message) != 0.0F || !action.blocked_reason.empty()) {
        return false;
    }
    action.blocked_reason = self_check_action_error_locked(action);
    if (!action.blocked_reason.empty()) {
        return false;
    }
    action.allowed_send = true;
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
    if (!expect_action(ManualTakeover::Action::Takeoff, error, height_m) || !ensure_connected(error)) {
        return false;
    }
    const auto takeoff_result =
        await_action([this](mavsdk::Action::ResultCallback cb) { action_->takeoff_async(std::move(cb)); });
    if (takeoff_result != mavsdk::Action::Result::Success) {
        action_failed(ManualTakeover::Action::Takeoff);
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
    if (!ensure_connected(error) || !expect_action(ManualTakeover::Action::Hold, error)) {
        return false;
    }
    const auto result = await_action([this](mavsdk::Action::ResultCallback cb) { action_->hold_async(std::move(cb)); });
    if (result != mavsdk::Action::Result::Success) {
        action_failed(ManualTakeover::Action::Hold);
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
        cache_.mission_progress_updated = {};
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
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.mission_current = 0;
        cache_.mission_total = 0;
        cache_.mission_progress_updated = {};
    }
    if (!ensure_connected(error) ||
        !expect_action(ManualTakeover::Action::Mission, error, 0.0F, static_cast<int>(points.size()))) {
        return false;
    }
    const auto start_result = await_async<mavsdk::Mission::Result>(
        [this](mavsdk::Mission::ResultCallback cb) { mission_->start_mission_async(std::move(cb)); });
    if (start_result != mavsdk::Mission::Result::Success) {
        action_failed(ManualTakeover::Action::Mission);
        error = "下发飞控StartMission 失败: " + result_text(start_result);
        report_error(error);
        return false;
    }
    // start_mission_async 返回前可能还没有收到新任务的首个进度回调。此处再次
    // 建立本次任务的初始进度，确保紧随其后的行为树 tick 必须等待新任务推进。
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        if (cache_.mission_progress_updated == ManualTakeover::Clock::time_point{}) {
            cache_.mission_current = 0;
            cache_.mission_total = static_cast<int>(points.size());
        }
    }
    SPDLOG_INFO("下发飞控指令成功: 启动任务（StartMission）");
    return true;
}

bool FlightControlModule::set_ned_velocity(const plane::model::NedVelocity& velocity, std::string& error) {
    return set_velocity_setpoint({static_cast<float>(velocity.north_m_s), static_cast<float>(velocity.east_m_s),
                                  static_cast<float>(velocity.down_m_s), static_cast<float>(velocity.yaw_deg)},
                                 std::nullopt, error);
}

bool FlightControlModule::set_ned_velocity_acceleration(const plane::algorithm::VelocityAccelerationCommand& command,
                                                        double current_yaw_deg, std::string& error) {
    const double yaw_deg = command.yaw_rad ? *command.yaw_rad * (180.0 / kPi) : current_yaw_deg;
    const std::array values{command.vn_m_s, command.ve_m_s, command.vd_m_s, command.an_m_s2,
                            command.ae_m_s2, command.ad_m_s2, yaw_deg};
    if (!std::all_of(values.begin(), values.end(), [](double value) {
            return std::abs(value) <= std::numeric_limits<float>::max();
        })) {
        error = "Offboard 速度、加速度或航向超出有限浮点范围";
        return false;
    }
    const mavsdk::Offboard::VelocityNedYaw velocity{static_cast<float>(command.vn_m_s),
                                                   static_cast<float>(command.ve_m_s),
                                                   static_cast<float>(command.vd_m_s), static_cast<float>(yaw_deg)};
    const mavsdk::Offboard::AccelerationNed acceleration{static_cast<float>(command.an_m_s2),
                                                         static_cast<float>(command.ae_m_s2),
                                                         static_cast<float>(command.ad_m_s2)};
    return set_velocity_setpoint(velocity, acceleration, error);
}

mavsdk::Offboard::Result FlightControlModule::send_velocity_setpoint_locked() {
    if (!acceleration_setpoint_) {
        return offboard_->set_velocity_ned(velocity_setpoint_);
    }
    // PX4 以 NaN 表示未指定位置。MAVSDK 使用 PositionNedYaw 中的 yaw_deg。
    constexpr float unset_position = std::numeric_limits<float>::quiet_NaN();
    const mavsdk::Offboard::PositionNedYaw position{unset_position, unset_position, unset_position,
                                                   velocity_setpoint_.yaw_deg};
    return offboard_->set_position_velocity_acceleration_ned(position, velocity_setpoint_, *acceleration_setpoint_);
}

bool FlightControlModule::set_velocity_setpoint(mavsdk::Offboard::VelocityNedYaw velocity,
                                               std::optional<mavsdk::Offboard::AccelerationNed> acceleration,
                                               std::string& error) {
    if (!wait_for_control(error)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(offboard_mutex_);
    if (!offboard_ || !connected_) {
        error = "飞控未连接";
        return false;
    }
    velocity_setpoint_ = velocity;
    acceleration_setpoint_ = acceleration;
    offboard_setpoint_kind_ = OffboardSetpointKind::VelocityNed;
    if (offboard_active_) {
        return true;
    }
    const mavsdk::Offboard::Result result = send_velocity_setpoint_locked();
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
    if (acceleration) {
        SPDLOG_INFO(
            "下发飞控指令成功: Offboard 速度及加速度启动（north_m_s={:.2f} east_m_s={:.2f} "
            "down_m_s={:.2f} north_m_s2={:.2f} east_m_s2={:.2f} down_m_s2={:.2f} yaw_deg={:.1f}）",
            velocity.north_m_s, velocity.east_m_s, velocity.down_m_s, acceleration->north_m_s2,
            acceleration->east_m_s2, acceleration->down_m_s2, velocity.yaw_deg);
    } else {
        SPDLOG_INFO(
            "下发飞控指令成功: Offboard 速度启动（north_m_s={:.2f} east_m_s={:.2f} "
            "down_m_s={:.2f} yaw_deg={:.1f}）",
            velocity.north_m_s, velocity.east_m_s, velocity.down_m_s, velocity.yaw_deg);
    }
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
    if (!offboard_ || !connected_) {
        error = "飞控未连接";
        return false;
    }
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
    std::string error;
    wait_for_control(error);
    std::lock_guard<std::mutex> lock(offboard_mutex_);
    offboard_setpoint_kind_ = OffboardSetpointKind::None;
    acceleration_setpoint_.reset();
    last_offboard_start_ms_ = 0;
    const bool stop_required = offboard_active_ || (offboard_ && offboard_->is_active());
    offboard_active_ = false;
    if (stop_required && offboard_) {
        if (!expect_action(ManualTakeover::Action::Hold, error)) {
            // 析构只撤销 MAVSDK 内部定时发送，保留飞手选定的飞行模式。
            offboard_.reset();
            if (connected_ && system_) {
                offboard_ = std::make_unique<mavsdk::Offboard>(system_);
            }
            return;
        }
        const auto stop_result = await_async<mavsdk::Offboard::Result>(
            [this](mavsdk::Offboard::ResultCallback cb) { offboard_->stop_async(std::move(cb)); });
        if (stop_result != mavsdk::Offboard::Result::Success) {
            action_failed(ManualTakeover::Action::Hold);
            report_error("下发飞控OffboardStop 失败: " + result_text(stop_result));
        } else {
            SPDLOG_INFO("下发飞控指令成功: Offboard 停止");
        }
    }
}

std::string FlightControlModule::link_state() const { return connected_ ? "connected" : "disconnected"; }

plane::flight_control::FlightStatus FlightControlModule::flight_status() const {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return flight_status_locked();
}

FlightStatus FlightControlModule::flight_status_locked() const {
    FlightStatus status;
    status.armed = cache_.armed;
    status.in_air = cache_.in_air;
    status.mission_current = cache_.mission_current;
    status.mission_total = cache_.mission_total;
    status.takeoff_altitude_m = cache_.takeoff_altitude_m;
    status.connected = connected_;
    status.on_ground = cache_.landed_state == mavsdk::Telemetry::LandedState::OnGround;
    status.armed_updated = cache_.armed_updated;
    status.ground_updated = cache_.ground_updated;
    status.mode_updated = cache_.mode_updated;
    status.flight_mode = magic_enum::enum_name(cache_.flight_mode);
    status.link_generation = link_generation_;
    return status;
}

void FlightControlModule::report_error(const std::string& detail) const {
    SPDLOG_ERROR("飞控错误: {}", detail);
    FlightControlError event;
    event.detail = detail;
    event_bus_.publish(events::kError, event);
}

}  // namespace plane::flight_control
