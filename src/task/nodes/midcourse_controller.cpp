#include "task/nodes/midcourse_controller.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "communication/communication_events.h"
#include "core/time_utils.h"
#include "task/nodes/midcourse_algorithm_adapter.h"

namespace plane::task {
namespace {

constexpr std::size_t kRequiredHistorySize = 20;
constexpr std::int64_t kProgressLogIntervalMs = 5000;

MidcourseGeoPoint to_midcourse_geo(const plane::model::GeoPoint& value) {
    MidcourseGeoPoint result;
    result.latitude = value.latitude;
    result.longitude = value.longitude;
    result.height = value.height;
    result.altitude = value.altitude;
    return result;
}

MidcourseMeasurement to_measurement(const plane::target::GcsTarget& target) {
    MidcourseMeasurement measurement;
    measurement.geo = to_midcourse_geo(target.geoPos);
    measurement.timestamp_ms = target.timestamp_ms;
    return measurement;
}

double median_interval_s(const std::vector<plane::target::GcsTarget>& history) {
    std::vector<std::int64_t> intervals;
    intervals.reserve(history.size() - 1);
    for (std::size_t index = 1; index < history.size(); ++index) {
        intervals.push_back(history[index].timestamp_ms - history[index - 1].timestamp_ms);
    }
    std::sort(intervals.begin(), intervals.end());
    return static_cast<double>(intervals[intervals.size() / 2]) / 1000.0;
}

}  // namespace

struct MidcourseController::Impl {
    Impl(plane::core::EventBus& event_bus_value, const plane::core::AppConfig& config_value,
         plane::flight_control::FlightControlApi& flight_control_value, plane::state::StateStoreApi& state_store_value,
         plane::target::TargetStoreApi& target_store_value)
        : event_bus(event_bus_value),
          config(config_value),
          flight_control(flight_control_value),
          state_store(state_store_value),
          target_store(target_store_value) {}

    void on_target(const plane::target::GcsTarget& target) {
        std::lock_guard<std::mutex> lock(mutex);
        if (!active || locked_track_id.empty() || target.trackId != locked_track_id ||
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
            if (!algorithm->update_target(to_measurement(target), error)) {
                error = "目标估计器 update 失败: " + error;
                pending_targets.clear();
                return false;
            }
            last_measurement_timestamp_ms = target.timestamp_ms;
            if (!estimator_update_logged) {
                estimator_update_logged = true;
                SPDLOG_INFO("中制导目标估计器首次增量更新: trackId={} timestamp_ms={}", target.trackId,
                            target.timestamp_ms);
            } else {
                SPDLOG_DEBUG("中制导目标估计更新: trackId={} timestamp_ms={}", target.trackId, target.timestamp_ms);
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
        SPDLOG_INFO("中制导自动锁定唯一靶机目标: trackId={}", locked_track_id);
        return true;
    }

    bool initialize_locked(std::string& error) {
        if (initialized) {
            return true;
        }
        if (!resolve_track_locked(error)) {
            return false;
        }
        if (!planner_configured) {
            plane::flight_control::FlightControlParams flight_params;
            if (!flight_control.get_cached_flight_control_params(flight_params, error)) {
                return false;
            }
            MidcourseAlgorithmConfig algorithm_config;
            algorithm_config.max_horizontal_speed_m_s = flight_params.mpc_xy_vel_max;
            algorithm_config.max_climb_speed_m_s = flight_params.mpc_z_vel_max_up;
            algorithm_config.max_descent_speed_m_s = flight_params.mpc_z_vel_max_dn;
            algorithm_config.max_acceleration_m_s2 = flight_params.mpc_acc_hor_max;
            algorithm_config.update_dt_s = static_cast<double>(config.bt_tick_interval_ms) / 1000.0;
            algorithm_config.target_invalid_grace_s = config.midcourse_target_invalid_grace_s;
            algorithm_config.r_min_m = config.midcourse_r_min_m;
            if (!algorithm->set_config(algorithm_config, error)) {
                error = "中制导 setConfig 失败: " + error;
                return false;
            }
            planner_configured = true;
            SPDLOG_INFO("中制导规划器配置完成: trackId={} update_dt_s={:.3f}", locked_track_id,
                        algorithm_config.update_dt_s);
        }
        const std::vector<plane::target::GcsTarget> history = target_store.gcs_target_history(locked_track_id);
        if (history.size() < kRequiredHistorySize) {
            return false;
        }
        const auto first = history.end() - static_cast<std::ptrdiff_t>(kRequiredHistorySize);
        std::vector<plane::target::GcsTarget> init_history(first, history.end());
        for (std::size_t index = 1; index < init_history.size(); ++index) {
            if (init_history[index].timestamp_ms <= init_history[index - 1].timestamp_ms) {
                error = "中制导初始化历史时间戳未严格递增";
                return false;
            }
        }
        const double estimator_dt_s = median_interval_s(init_history);
        if (estimator_dt_s <= 0.0) {
            error = "中制导初始化历史周期非法";
            return false;
        }
        std::vector<MidcourseMeasurement> measurements;
        measurements.reserve(init_history.size());
        for (const auto& target : init_history) {
            measurements.push_back(to_measurement(target));
        }
        const plane::model::VehicleState own = state_store.own_state();
        if (!algorithm->init_estimator(measurements, to_midcourse_geo(own.startPos), estimator_dt_s, error)) {
            error = "目标估计器 init 失败: " + error;
            return false;
        }
        last_measurement_timestamp_ms = init_history.back().timestamp_ms;
        initialized = true;
        SPDLOG_INFO("中制导初始化完成: trackId={} history_size={} estimator_dt_s={:.3f} update_dt_s={:.3f}",
                    locked_track_id, init_history.size(), estimator_dt_s,
                    static_cast<double>(config.bt_tick_interval_ms) / 1000.0);
        return true;
    }

    plane::core::EventBus& event_bus;
    const plane::core::AppConfig& config;
    plane::flight_control::FlightControlApi& flight_control;
    plane::state::StateStoreApi& state_store;
    plane::target::TargetStoreApi& target_store;
    std::mutex mutex;
    bool subscribed = false;
    bool active = false;
    bool planner_configured = false;
    bool initialized = false;
    bool estimator_update_logged = false;
    std::string requested_track_id;
    std::string locked_track_id;
    std::int64_t start_ms = 0;
    std::int64_t last_progress_log_ms = 0;
    std::int64_t last_measurement_timestamp_ms = 0;
    std::vector<plane::target::GcsTarget> pending_targets;
    std::unique_ptr<MidcourseAlgorithmAdapter> algorithm;
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
    impl_->event_bus.subscribe<plane::target::GcsTarget>(
        plane::communication::events::kGcsTargetReceived,
        [impl = impl_.get()](const plane::target::GcsTarget& target) { impl->on_target(target); });
    impl_->subscribed = true;
}

bool MidcourseController::begin(const std::string& requested_track_id, std::string& error) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->config.midcourse_init_timeout_ms <= 0 || impl_->config.bt_tick_interval_ms <= 0 ||
        impl_->config.midcourse_target_invalid_grace_s < 0.0 || impl_->config.midcourse_r_min_m <= 0.0) {
        error = "中制导配置非法";
        return false;
    }
    impl_->active = true;
    impl_->planner_configured = false;
    impl_->initialized = false;
    impl_->estimator_update_logged = false;
    impl_->requested_track_id = requested_track_id;
    impl_->locked_track_id.clear();
    impl_->pending_targets.clear();
    impl_->start_ms = plane::core::now_ms();
    impl_->last_progress_log_ms = impl_->start_ms - kProgressLogIntervalMs;
    impl_->last_measurement_timestamp_ms = 0;
    impl_->algorithm = std::make_unique<MidcourseAlgorithmAdapter>();
    SPDLOG_INFO("中制导会话启动: requested_track_id={}", requested_track_id.empty() ? "<自动>" : requested_track_id);
    return true;
}

MidcourseController::TickResult MidcourseController::tick(std::string& error) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->active) {
        error = "中制导会话未启动";
        return TickResult::Failure;
    }
    if (!impl_->initialized) {
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
                                              : impl_->target_store.gcs_target_history(impl_->locked_track_id).size();
                error = "等待中制导 20 帧雷达历史超时，当前 " + std::to_string(count) + "/20";
                return TickResult::Failure;
            }
            if (now - impl_->last_progress_log_ms >= kProgressLogIntervalMs) {
                impl_->last_progress_log_ms = now;
                const std::size_t count = impl_->locked_track_id.empty()
                                              ? 0
                                              : impl_->target_store.gcs_target_history(impl_->locked_track_id).size();
                SPDLOG_INFO("等待中制导雷达历史: trackId={} history_size={}/20 elapsed_ms={} remaining_ms={}",
                            impl_->locked_track_id.empty() ? "<自动>" : impl_->locked_track_id, count,
                            now - impl_->start_ms, impl_->config.midcourse_init_timeout_ms - (now - impl_->start_ms));
            }
            return TickResult::Running;
        }
    }

    const plane::model::VehicleState own = impl_->state_store.own_state();
    MidcourseVehicleState vehicle;
    vehicle.geo = to_midcourse_geo(own.geoPos);
    vehicle.start = to_midcourse_geo(own.startPos);
    vehicle.vn_m_s = own.vn_m_s;
    vehicle.ve_m_s = own.ve_m_s;
    vehicle.vd_m_s = own.vd_m_s;
    MidcourseAttitudeOutput output;
    if (!impl_->algorithm->update(vehicle, output, error)) {
        error = "中制导 update 失败: " + error;
        return TickResult::Failure;
    }
    plane::flight_control::AttitudeSetpoint setpoint;
    setpoint.quat_body_to_ned = output.quat_body_to_ned;
    setpoint.thrust_body = output.thrust_body;
    if (!impl_->flight_control.set_attitude(setpoint, error)) {
        error = "中制导飞控指令执行失败: " + error;
        return TickResult::Failure;
    }
    if (!impl_->flush_pending_targets_locked(error)) {
        return TickResult::Failure;
    }
    SPDLOG_DEBUG("中制导执行周期: eta_seconds={:.3f} following={}", output.eta_seconds,
                 output.following ? "true" : "false");
    return TickResult::Running;
}

void MidcourseController::stop() {
    if (!impl_) {
        return;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->active = false;
    impl_->planner_configured = false;
    impl_->initialized = false;
    impl_->estimator_update_logged = false;
    impl_->requested_track_id.clear();
    impl_->locked_track_id.clear();
    impl_->pending_targets.clear();
    impl_->last_measurement_timestamp_ms = 0;
    impl_->algorithm.reset();
}

}  // namespace plane::task
