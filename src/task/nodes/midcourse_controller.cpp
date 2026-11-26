#include "task/nodes/midcourse_controller.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <magic_enum/magic_enum.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "algorithm/midcourse_alg.h"
#include "communication/communication_events.h"
#include "core/time_utils.h"

namespace plane::task {
namespace {

constexpr std::int64_t kProgressLogIntervalMs = 5000;
constexpr std::int64_t kUpdateLogIntervalMs = 1000;
constexpr double kPi = 3.14159265358979323846;

std::string geo_text(const plane::model::GeoPoint& value) {
    return fmt::format("{{latitude={} longitude={} height={} altitude={}}}", value.latitude, value.longitude,
                       value.height, value.altitude);
}

std::string measurement_text(const plane::algorithm::TargetMeasurement& value) {
    return fmt::format("{{geo={} timestamp_ms={}}}", geo_text(value.geo), value.timestamp_ms);
}

std::string history_text(const std::vector<plane::algorithm::TargetMeasurement>& history) {
    std::string text = "[";
    for (const auto& measurement : history) {
        if (text.size() > 1) text += ", ";
        text += measurement_text(measurement);
    }
    return text + "]";
}

std::string command_text(const plane::algorithm::MidcourseCommand& command) {
    if (const auto* value = std::get_if<plane::algorithm::VelocityAccelerationCommand>(&command.value)) {
        return fmt::format(
            "VelocityAccelerationCommand{{vn_m_s={} ve_m_s={} vd_m_s={} an_m_s2={} ae_m_s2={} ad_m_s2={} yaw_rad={}}}",
            value->vn_m_s, value->ve_m_s, value->vd_m_s, value->an_m_s2, value->ae_m_s2, value->ad_m_s2,
            value->yaw_rad ? fmt::format("{}", *value->yaw_rad) : "nullopt");
    }
    if (const auto* value = std::get_if<plane::algorithm::AttitudeCommand>(&command.value)) {
        return fmt::format("AttitudeCommand{{quat_body_to_ned=[{}, {}, {}, {}] thrust_body=[{}, {}, {}]}}",
                           value->quat_body_to_ned[0], value->quat_body_to_ned[1], value->quat_body_to_ned[2],
                           value->quat_body_to_ned[3], value->thrust_body[0], value->thrust_body[1], value->thrust_body[2]);
    }
    return "<无指令>";
}

std::string output_text(const plane::algorithm::MidCourseOutput& output) {
    return fmt::format("{{cmd={{mode={} value={}}} eta_seconds={} following={} terminal_confirmed={}}}",
                       magic_enum::enum_name(output.cmd.mode), command_text(output.cmd), output.eta_seconds,
                       output.following, output.terminal_confirmed);
}

}  // namespace

struct MidcourseController::Impl {
    struct AlgorithmSession {
        plane::algorithm::MidcourseAlgorithm algorithm;
        std::optional<std::int64_t> last_update_log_ms;
    };

    Impl(plane::core::EventBus& event_bus_value, const plane::core::AppConfig& config_value,
         plane::flight_control::FlightControlApi& flight_control_value, plane::state::StateStoreApi& state_store_value,
         plane::target::TargetStoreApi& target_store_value)
        : event_bus(event_bus_value),
          config(config_value),
          flight_control(flight_control_value),
          state_store(state_store_value),
          target_store(target_store_value) {}

    void on_target(const plane::model::GcsTarget& target) {
        std::lock_guard<std::mutex> lock(mutex);
        if (!session || locked_track_id.empty() || target.trackId != locked_track_id ||
            target.timestamp_ms <= last_measurement_timestamp_ms) {
            return;
        }
        pending_targets.push_back(target);
    }

    bool flush_pending_targets_locked(std::string& error) {
        std::sort(pending_targets.begin(), pending_targets.end(),
                  [](const auto& first, const auto& second) { return first.timestamp_ms < second.timestamp_ms; });
        for (const auto& target : pending_targets) {
            if (target.timestamp_ms <= last_measurement_timestamp_ms) continue;
            const plane::algorithm::TargetMeasurement measurement{target.geoPos, target.timestamp_ms};
            SPDLOG_INFO("中制导算法 MidcourseAlgorithm::updateTarget 输入: measurement={}",
                        measurement_text(measurement));
            error.clear();
            const bool updated = session->algorithm.updateTarget(measurement, error);
            SPDLOG_INFO("中制导算法 MidcourseAlgorithm::updateTarget 输出: 返回值={} error={}", updated, error);
            if (!updated) {
                error = "中制导 updateTarget 失败: " + error;
                pending_targets.clear();
                return false;
            }
            last_measurement_timestamp_ms = target.timestamp_ms;
            if (!target_update_logged) {
                target_update_logged = true;
                SPDLOG_INFO("中制导算法首次增量量测更新: trackId={} timestamp_ms={}", target.trackId,
                            target.timestamp_ms);
            }
        }
        pending_targets.clear();
        return true;
    }

    bool resolve_track_locked(std::string& error) {
        if (!locked_track_id.empty()) {
            return true;
        }
        if (!requested_track_id.empty()) {
            locked_track_id = requested_track_id;
            return true;
        }
        const auto targets = target_store.gcs_targets();
        if (targets.empty()) {
            return false;
        }
        if (targets.size() != 1) {
            error = "targetTrackId 为空且当前靶机目标数不为 1";
            return false;
        }
        locked_track_id = targets.front().trackId;
        target_store.lock_required_gcs_target(locked_track_id);
        SPDLOG_INFO("中制导自动锁定唯一靶机目标: trackId={}", locked_track_id);
        return true;
    }

    bool check_target_timeout_locked(std::string& error) const {
        std::optional<std::int64_t> received_ms;
        if (!locked_track_id.empty()) {
            received_ms = target_store.gcs_target_updates_after(locked_track_id, target_revision).last_received_ms;
        }
        const auto age_ms = plane::core::now_ms() - received_ms.value_or(start_ms);
        if (age_ms <= config.midcourse_target_timeout_ms) {
            return true;
        }
        const char* reason = !received_ms ? "本次会话从未收到" : "目标更新中断";
        error = fmt::format("中制导目标雷达信息超时: trackId={} 未更新时长={}毫秒 target_timeout_ms={}（{}）",
                            locked_track_id.empty() ? "<自动>" : locked_track_id, age_ms,
                            config.midcourse_target_timeout_ms, reason);
        return false;
    }

    bool initialize_locked(std::string& error) {
        if (!effective_config) {
            auto algorithm_config = config.midcourse_algorithm;
            if (config.midcourse_use_px4_limits) {
                plane::flight_control::FlightControlParams flight_params;
                if (!flight_control.get_cached_flight_control_params(flight_params, error)) {
                    return false;
                }
                algorithm_config.max_horizontal_speed_m_s = flight_params.mpc_xy_vel_max;
                algorithm_config.max_climb_speed_m_s = flight_params.mpc_z_vel_max_up;
                algorithm_config.max_descent_speed_m_s = flight_params.mpc_z_vel_max_dn;
                algorithm_config.cmd_accel_max_m_s2 = flight_params.mpc_acc_hor_max;
            }
            const std::string validation_error = algorithm_config.validate();
            if (!validation_error.empty()) {
                error = "中制导配置校验失败: " + validation_error;
                return false;
            }
            effective_config = algorithm_config;
        }
        const bool resolved = resolve_track_locked(error);
        if (!error.empty() || !check_target_timeout_locked(error) || !resolved) {
            return false;
        }
        if (initialized) {
            return true;
        }
        const std::vector<plane::model::GcsTarget> history =
            target_store.gcs_target_updates_after(locked_track_id, target_revision).history;
        const auto required_history_size = static_cast<std::size_t>(config.midcourse_target_history_size);
        if (history.size() < required_history_size) {
            return false;
        }
        const auto first = history.end() - static_cast<std::ptrdiff_t>(required_history_size);
        std::vector<plane::model::GcsTarget> init_history(first, history.end());
        for (std::size_t index = 1; index < init_history.size(); ++index) {
            if (init_history[index].timestamp_ms <= init_history[index - 1].timestamp_ms) {
                error = "中制导初始化历史时间戳未严格递增";
                return false;
            }
        }
        std::vector<plane::algorithm::TargetMeasurement> measurements;
        measurements.reserve(init_history.size());
        for (const auto& target : init_history) {
            measurements.push_back({target.geoPos, target.timestamp_ms});
        }
        const plane::model::VehicleState own = state_store.own_state();
        const auto& algorithm_config = *effective_config;
        if (!check_target_timeout_locked(error)) {
            return false;
        }
        SPDLOG_INFO(
            "中制导算法 MidcourseAlgorithm::init 输入: history={} ned_ref={} trackId={} use_px4_limits={} "
            "max_horizontal_speed_m_s={} max_climb_speed_m_s={} max_descent_speed_m_s={} "
            "vehicle_mass_kg={} max_thrust_n={} v_cruise_m_s={} approach_brake_k={} tau_ff_s={} r_min_m={} "
            "cmd_accel_max_m_s2={} update_dt_s={} target_invalid_grace_s={} control_output_mode={} "
            "hover_brake_accel_m_s2={} yaw_mode={} yaw_rate_max_deg_s={} enable_internal_terminal_confirmation={} "
            "half_fov_deg={} r_capture_min_m={} r_capture_max_m={} r_capture_nominal_m={} capture_discount={} "
            "sigma_dot_max_deg_s={} speed_ratio={} tol_pos_m={} tol_vel_m={} tol_accel_m_s2={} min_altitude_m={} "
            "use_cost_search={} w_time={} w_speed={} w_accel={} replan_period_s={} replan_freeze_s={} "
            "track_gain_k={} track_gain_d={} track_max_corr={} recovery_gain_k={} recovery_margin_m={} "
            "recovery_exit_hold_s={} recovery_vz_deadband_m_s={} recovery_altitude_tolerance_m={} "
            "follow_gain_k={} follow_filter_a={} follow_vel_tau_s={} "
            "confirm_boresight_deg={} confirm_rate_deg_s={} confirm_hold_s={}",
            history_text(measurements), geo_text(own.startPos), locked_track_id, config.midcourse_use_px4_limits,
            algorithm_config.max_horizontal_speed_m_s, algorithm_config.max_climb_speed_m_s,
            algorithm_config.max_descent_speed_m_s, algorithm_config.vehicle_mass_kg, algorithm_config.max_thrust_n,
            algorithm_config.v_cruise_m_s, algorithm_config.approach_brake_k, algorithm_config.tau_ff_s,
            algorithm_config.r_min_m, algorithm_config.cmd_accel_max_m_s2, algorithm_config.update_dt_s,
            algorithm_config.target_invalid_grace_s, static_cast<int>(algorithm_config.control_output_mode),
            algorithm_config.hover_brake_accel_m_s2, algorithm_config.yaw_mode,
            algorithm_config.yaw_rate_max_deg_s, algorithm_config.enable_internal_terminal_confirmation,
            algorithm_config.half_fov_deg, algorithm_config.r_capture_min_m, algorithm_config.r_capture_max_m,
            algorithm_config.r_capture_nominal_m, algorithm_config.capture_discount,
            algorithm_config.sigma_dot_max_deg_s, algorithm_config.speed_ratio, algorithm_config.tol_pos_m,
            algorithm_config.tol_vel_m, algorithm_config.tol_accel_m_s2, algorithm_config.min_altitude_m,
            algorithm_config.use_cost_search, algorithm_config.w_time, algorithm_config.w_speed,
            algorithm_config.w_accel, algorithm_config.replan_period_s, algorithm_config.replan_freeze_s,
            algorithm_config.track_gain_k, algorithm_config.track_gain_d, algorithm_config.track_max_corr,
            algorithm_config.recovery_gain_k, algorithm_config.recovery_margin_m,
            algorithm_config.recovery_exit_hold_s, algorithm_config.recovery_vz_deadband_m_s,
            algorithm_config.recovery_altitude_tolerance_m,
            algorithm_config.follow_gain_k, algorithm_config.follow_filter_a,
            algorithm_config.follow_vel_tau_s, algorithm_config.confirm_boresight_deg,
            algorithm_config.confirm_rate_deg_s, algorithm_config.confirm_hold_s);
        error.clear();
        const bool initialized_algorithm = session->algorithm.init(measurements, own.startPos, algorithm_config, error);
        SPDLOG_INFO("中制导算法 MidcourseAlgorithm::init 输出: 返回值={} error={}", initialized_algorithm, error);
        if (!initialized_algorithm) {
            error = "中制导 init 失败: " + error;
            return false;
        }
        last_measurement_timestamp_ms = init_history.back().timestamp_ms;
        initialized = true;
        SPDLOG_INFO("中制导初始化完成: trackId={} history_size={} update_dt_s={:.3f}", locked_track_id,
                    init_history.size(), algorithm_config.update_dt_s);
        return true;
    }

    plane::core::EventBus& event_bus;
    const plane::core::AppConfig& config;
    plane::flight_control::FlightControlApi& flight_control;
    plane::state::StateStoreApi& state_store;
    plane::target::TargetStoreApi& target_store;
    std::mutex mutex;
    bool subscribed = false;
    bool initialized = false;
    bool target_update_logged = false;
    std::string requested_track_id;
    std::string locked_track_id;
    std::int64_t start_ms = 0;
    std::uint64_t target_revision = 0;
    std::int64_t last_progress_log_ms = 0;
    std::int64_t last_measurement_timestamp_ms = 0;
    std::vector<plane::model::GcsTarget> pending_targets;
    std::optional<plane::algorithm::MidCourseConfig> effective_config;
    std::optional<AlgorithmSession> session;
};

MidcourseController::MidcourseController(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                                         plane::flight_control::FlightControlApi& flight_control,
                                         plane::state::StateStoreApi& state_store,
                                         plane::target::TargetStoreApi& target_store)
    : impl_(std::make_unique<Impl>(event_bus, config, flight_control, state_store, target_store)) {}

MidcourseController::~MidcourseController() { stop(); }

void MidcourseController::start() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->subscribed) {
        return;
    }
    impl_->event_bus.subscribe<plane::model::GcsTarget>(
        plane::communication::events::kGcsTargetReceived,
        [impl = impl_.get()](const plane::model::GcsTarget& target) { impl->on_target(target); });
    impl_->subscribed = true;
}

bool MidcourseController::begin(const std::string& requested_track_id, std::string& error) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->config.midcourse_target_timeout_ms <= 0) {
        error = "中制导配置非法: target_timeout_ms 必须为正整数";
        return false;
    }
    if (impl_->config.midcourse_target_history_size < plane::core::kMinMidcourseTargetHistorySize ||
        impl_->config.midcourse_target_history_size > plane::core::kMaxMidcourseTargetHistorySize) {
        error = "中制导配置非法: target_history_size 必须为 5 到 30 的整数";
        return false;
    }
    if (impl_->config.midcourse_init_timeout_ms <= 0 || impl_->config.bt_tick_interval_ms <= 0) {
        error = "中制导配置非法";
        return false;
    }
    impl_->initialized = false;
    impl_->target_update_logged = false;
    impl_->requested_track_id = requested_track_id;
    impl_->locked_track_id.clear();
    impl_->pending_targets.clear();
    impl_->target_revision = impl_->target_store.gcs_target_revision();
    impl_->start_ms = plane::core::now_ms();
    impl_->last_progress_log_ms = impl_->start_ms - kProgressLogIntervalMs;
    impl_->last_measurement_timestamp_ms = 0;
    impl_->effective_config.reset();
    impl_->session.emplace();
    SPDLOG_INFO("中制导会话启动: requested_track_id={} target_history_size={} target_timeout_ms={}",
                requested_track_id.empty() ? "<自动>" : requested_track_id,
                impl_->config.midcourse_target_history_size, impl_->config.midcourse_target_timeout_ms);
    return true;
}

MidcourseController::TickResult MidcourseController::tick(std::string& error) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->session) {
        error = "中制导会话未启动";
        return TickResult::Failure;
    }
    {
        std::string init_error;
        if (!impl_->initialize_locked(init_error)) {
            if (!init_error.empty()) {
                error = init_error;
                return TickResult::Failure;
            }
            const std::int64_t now = plane::core::now_ms();
            if (now - impl_->start_ms > impl_->config.midcourse_init_timeout_ms) {
                const std::size_t count = impl_->locked_track_id.empty()
                                              ? 0
                                              : impl_->target_store
                                                    .gcs_target_updates_after(impl_->locked_track_id,
                                                                              impl_->target_revision)
                                                    .history.size();
                error = fmt::format("等待中制导 {} 帧雷达历史超时，当前 {}/{}",
                                    impl_->config.midcourse_target_history_size, count,
                                    impl_->config.midcourse_target_history_size);
                return TickResult::Failure;
            }
            if (now - impl_->last_progress_log_ms >= kProgressLogIntervalMs) {
                impl_->last_progress_log_ms = now;
                const std::size_t count = impl_->locked_track_id.empty()
                                              ? 0
                                              : impl_->target_store
                                                    .gcs_target_updates_after(impl_->locked_track_id,
                                                                              impl_->target_revision)
                                                    .history.size();
                SPDLOG_INFO(
                    "等待中制导雷达历史: trackId={} history_size={}/{} elapsed_ms={} remaining_ms={}",
                    impl_->locked_track_id.empty() ? "<自动>" : impl_->locked_track_id, count,
                    impl_->config.midcourse_target_history_size, now - impl_->start_ms,
                    impl_->config.midcourse_init_timeout_ms - (now - impl_->start_ms));
            }
            return TickResult::Running;
        }
    }

    const plane::model::VehicleState own = impl_->state_store.own_state();
    if (!std::isfinite(static_cast<double>(own.yaw_deg))) {
        error = "中制导本机实际航向 yaw_deg 必须为有限数";
        return TickResult::Failure;
    }
    const plane::algorithm::MidcourseVehicleState vehicle{
        own.geoPos, own.startPos, own.vn_m_s, own.ve_m_s, own.vd_m_s,
        static_cast<double>(own.yaw_deg) * kPi / 180.0};
    if (!impl_->check_target_timeout_locked(error)) {
        return TickResult::Failure;
    }
    plane::algorithm::MidCourseOutput output;
    error.clear();
    const bool updated = impl_->session->algorithm.update(vehicle, output, error);
    const std::int64_t now = plane::core::now_ms();
    auto& last_update_log_ms = impl_->session->last_update_log_ms;
    if (!updated || !last_update_log_ms || now - *last_update_log_ms >= kUpdateLogIntervalMs) {
        SPDLOG_INFO(
            "中制导算法 MidcourseAlgorithm::update 输入: "
            "vehicle={{geoPos={} startPos={} vn_m_s={} ve_m_s={} vd_m_s={} yaw_rad={}}}",
            geo_text(vehicle.geoPos), geo_text(vehicle.startPos), vehicle.vn_m_s, vehicle.ve_m_s, vehicle.vd_m_s,
            *vehicle.yaw_rad);
        SPDLOG_INFO("中制导算法 MidcourseAlgorithm::update 输出: 返回值={} output={} error={}", updated,
                    updated ? output_text(output) : "<无有效输出>", error);
        last_update_log_ms = now;
    }
    if (!updated) {
        error = "中制导 update 失败: " + error;
        return TickResult::Failure;
    }
    if (!impl_->check_target_timeout_locked(error)) {
        return TickResult::Failure;
    }
    const auto* velocity = std::get_if<plane::algorithm::VelocityAccelerationCommand>(&output.cmd.value);
    const auto* attitude = std::get_if<plane::algorithm::AttitudeCommand>(&output.cmd.value);
    bool sent;
    if (output.cmd.mode == plane::algorithm::MidcourseControlMode::VelocityAcceleration && velocity) {
        sent = impl_->flight_control.set_ned_velocity_acceleration(*velocity, own.yaw_deg, error);
    } else if (output.cmd.mode == plane::algorithm::MidcourseControlMode::Attitude && attitude) {
        sent = impl_->flight_control.set_attitude(*attitude, error);
    } else {
        error = "中制导算法输出模式与指令类型不一致";
        return TickResult::Failure;
    }
    if (!sent) {
        error = "中制导飞控指令执行失败: " + error;
        return TickResult::Failure;
    }
    if (!impl_->flush_pending_targets_locked(error)) {
        return TickResult::Failure;
    }
    return TickResult::Running;
}

void MidcourseController::stop() {
    if (!impl_) {
        return;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->initialized = false;
    impl_->target_update_logged = false;
    impl_->requested_track_id.clear();
    impl_->locked_track_id.clear();
    impl_->pending_targets.clear();
    impl_->last_measurement_timestamp_ms = 0;
    impl_->effective_config.reset();
    impl_->session.reset();
}

}  // namespace plane::task
