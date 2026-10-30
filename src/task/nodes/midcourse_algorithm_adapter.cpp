#include "task/nodes/midcourse_algorithm_adapter.h"

#include <spdlog/spdlog.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <variant>

#include "algorithm/midcourse_alg.h"

namespace plane::task {
namespace {

std::atomic<std::uint64_t> next_session_id{0};

plane::model::GeoPoint to_algorithm_geo(const MidcourseGeoPoint& value) {
    plane::model::GeoPoint result;
    result.latitude = value.latitude;
    result.longitude = value.longitude;
    result.height = value.height;
    result.altitude = value.altitude;
    return result;
}

plane::algorithm::TargetMeasurement to_algorithm_measurement(const MidcourseMeasurement& value) {
    plane::algorithm::TargetMeasurement result;
    result.geo = to_algorithm_geo(value.geo);
    result.timestamp_ms = value.timestamp_ms;
    return result;
}

const char* control_mode_name(plane::algorithm::MidcourseControlMode mode) {
    switch (mode) {
        case plane::algorithm::MidcourseControlMode::VelocityAcceleration:
            return "VelocityAcceleration";
        case plane::algorithm::MidcourseControlMode::Attitude:
            return "Attitude";
    }
    return "Unknown";
}

const char* bool_name(bool value) { return value ? "true" : "false"; }

}  // namespace

struct MidcourseAlgorithmAdapter::Impl {
    Impl() : session_id(++next_session_id) {}

    plane::algorithm::TargetEstimator estimator;
    plane::algorithm::MidcoursePlanner planner;
    plane::algorithm::TargetEstimate estimate;
    const std::uint64_t session_id;
    std::uint64_t call_sequence = 0;
    std::uint64_t estimator_update_sequence = 0;
    std::uint64_t planner_update_sequence = 0;
};

MidcourseAlgorithmAdapter::MidcourseAlgorithmAdapter() : impl_(std::make_unique<Impl>()) {
    SPDLOG_INFO("中制导会话创建: session={}", impl_->session_id);
}

MidcourseAlgorithmAdapter::~MidcourseAlgorithmAdapter() {
    SPDLOG_INFO("中制导会话销毁: session={} calls={} estimator_updates={} planner_updates={}", impl_->session_id,
                impl_->call_sequence, impl_->estimator_update_sequence, impl_->planner_update_sequence);
}

bool MidcourseAlgorithmAdapter::init_estimator(const std::vector<MidcourseMeasurement>& history,
                                               const MidcourseGeoPoint& ned_reference, double update_dt_s,
                                               std::string& error) {
    ++impl_->call_sequence;
    std::vector<plane::algorithm::TargetMeasurement> converted;
    converted.reserve(history.size());
    for (const auto& item : history) converted.push_back(to_algorithm_measurement(item));
    const auto converted_reference = to_algorithm_geo(ned_reference);
    SPDLOG_INFO(
        "TargetEstimator::init 输入: history_size={} "
        "ned_ref={{latitude={:.6f},longitude={:.6f},height={:.2f},altitude={:.2f}}} update_dt_s={:.3f}",
        converted.size(), converted_reference.latitude, converted_reference.longitude, converted_reference.height,
        converted_reference.altitude, update_dt_s);
    for (std::size_t index = 0; index < converted.size(); ++index) {
        const auto& measurement = converted[index];
        SPDLOG_INFO(
            "TargetEstimator::init 输入: history_index={} "
            "measurement={{geo={{latitude={:.6f},longitude={:.6f},height={:.2f},altitude={:.2f}}},timestamp_ms={}}}",
            index, measurement.geo.latitude, measurement.geo.longitude, measurement.geo.height,
            measurement.geo.altitude, measurement.timestamp_ms);
    }
    impl_->estimate = plane::algorithm::TargetEstimate{};
    const bool ok = impl_->estimator.init(converted, converted_reference, update_dt_s, error);
    SPDLOG_INFO("TargetEstimator::init 输出: success={} error='{}'", bool_name(ok), error);
    return ok;
}

bool MidcourseAlgorithmAdapter::set_config(const MidcourseAlgorithmConfig& config, std::string& error) {
    ++impl_->call_sequence;
    plane::algorithm::MidCourseConfig converted;
    converted.max_horizontal_speed_m_s = config.max_horizontal_speed_m_s;
    converted.max_climb_speed_m_s = config.max_climb_speed_m_s;
    converted.max_descent_speed_m_s = config.max_descent_speed_m_s;
    converted.cmd_accel_max_m_s2 = config.max_acceleration_m_s2;
    converted.update_dt_s = config.update_dt_s;
    converted.target_invalid_grace_s = config.target_invalid_grace_s;
    converted.r_min_m = config.r_min_m;
    converted.control_output_mode = plane::algorithm::MidcourseControlMode::Attitude;
    converted.enable_internal_terminal_confirmation = false;
    SPDLOG_INFO(
        "MidcoursePlanner::setConfig 输入: "
        "config={{max_horizontal_speed_m_s={:.2f},max_climb_speed_m_s={:.2f},"
        "max_descent_speed_m_s={:.2f},v_desired_m_s={:.2f},tau_ff_s={:.3f},r_min_m={:.2f},"
        "cmd_accel_max_m_s2={:.2f},update_dt_s={:.3f},target_invalid_grace_s={:.3f},"
        "control_output_mode={},enable_internal_terminal_confirmation={}}}",
        converted.max_horizontal_speed_m_s, converted.max_climb_speed_m_s, converted.max_descent_speed_m_s,
        converted.v_desired_m_s, converted.tau_ff_s, converted.r_min_m, converted.cmd_accel_max_m_s2,
        converted.update_dt_s, converted.target_invalid_grace_s, control_mode_name(converted.control_output_mode),
        bool_name(converted.enable_internal_terminal_confirmation));
    const bool ok = impl_->planner.setConfig(converted, error);
    SPDLOG_INFO("MidcoursePlanner::setConfig 输出: success={} error='{}'", bool_name(ok), error);
    return ok;
}

bool MidcourseAlgorithmAdapter::update_target(const MidcourseMeasurement& measurement, std::string& error) {
    ++impl_->call_sequence;
    ++impl_->estimator_update_sequence;
    const auto converted_measurement = to_algorithm_measurement(measurement);
    SPDLOG_INFO(
        "TargetEstimator::update 输入: "
        "measurement={{geo={{latitude={:.6f},longitude={:.6f},height={:.2f},altitude={:.2f}}},timestamp_ms={}}}",
        converted_measurement.geo.latitude, converted_measurement.geo.longitude, converted_measurement.geo.height,
        converted_measurement.geo.altitude, converted_measurement.timestamp_ms);
    plane::algorithm::TargetEstimate updated;
    const bool ok = impl_->estimator.update(converted_measurement, updated, error);
    SPDLOG_INFO(
        "TargetEstimator::update 输出: success={} "
        "output={{geo={{latitude={:.6f},longitude={:.6f},height={:.2f},altitude={:.2f}}},"
        "vn_m_s={:.2f},ve_m_s={:.2f},vd_m_s={:.2f},valid={},timestamp_ms={},age_ms={}}} error='{}'",
        bool_name(ok), updated.geo.latitude, updated.geo.longitude, updated.geo.height, updated.geo.altitude,
        updated.vn_m_s, updated.ve_m_s, updated.vd_m_s, bool_name(updated.valid), updated.timestamp_ms, updated.age_ms,
        error);
    if (!ok) return false;
    impl_->estimate = updated;
    return true;
}

bool MidcourseAlgorithmAdapter::update(const MidcourseVehicleState& vehicle, MidcourseAttitudeOutput& output,
                                       std::string& error) {
    ++impl_->call_sequence;
    ++impl_->planner_update_sequence;
    plane::model::VehicleState converted_vehicle;
    converted_vehicle.geoPos = to_algorithm_geo(vehicle.geo);
    converted_vehicle.startPos = to_algorithm_geo(vehicle.start);
    converted_vehicle.vn_m_s = static_cast<float>(vehicle.vn_m_s);
    converted_vehicle.ve_m_s = static_cast<float>(vehicle.ve_m_s);
    converted_vehicle.vd_m_s = static_cast<float>(vehicle.vd_m_s);

    SPDLOG_INFO(
        "MidcoursePlanner::update 输入: "
        "vehicle={{geoPos={{latitude={:.6f},longitude={:.6f},height={:.2f},altitude={:.2f}}},"
        "startPos={{latitude={:.6f},longitude={:.6f},height={:.2f},altitude={:.2f}}},"
        "vn_m_s={:.2f},ve_m_s={:.2f},vd_m_s={:.2f}}} "
        "target={{geo={{latitude={:.6f},longitude={:.6f},height={:.2f},altitude={:.2f}}},"
        "vn_m_s={:.2f},ve_m_s={:.2f},vd_m_s={:.2f},valid={},timestamp_ms={},age_ms={}}}",
        converted_vehicle.geoPos.latitude, converted_vehicle.geoPos.longitude, converted_vehicle.geoPos.height,
        converted_vehicle.geoPos.altitude, converted_vehicle.startPos.latitude, converted_vehicle.startPos.longitude,
        converted_vehicle.startPos.height, converted_vehicle.startPos.altitude, converted_vehicle.vn_m_s,
        converted_vehicle.ve_m_s, converted_vehicle.vd_m_s, impl_->estimate.geo.latitude, impl_->estimate.geo.longitude,
        impl_->estimate.geo.height, impl_->estimate.geo.altitude, impl_->estimate.vn_m_s, impl_->estimate.ve_m_s,
        impl_->estimate.vd_m_s, bool_name(impl_->estimate.valid), impl_->estimate.timestamp_ms, impl_->estimate.age_ms);

    plane::algorithm::MidCourseOutput converted_output;
    const bool ok = impl_->planner.update(converted_vehicle, impl_->estimate, converted_output, error);
    if (const auto* attitude = std::get_if<plane::algorithm::AttitudeCommand>(&converted_output.cmd.value)) {
        SPDLOG_INFO(
            "MidcoursePlanner::update 输出: success={} "
            "output={{cmd={{mode={},value={{quat_body_to_ned=[{:.6f},{:.6f},{:.6f},{:.6f}],"
            "thrust_body=[{:.3f},{:.3f},{:.3f}]}}}},eta_seconds={:.2f},following={},terminal_confirmed={}}} "
            "error='{}'",
            bool_name(ok), control_mode_name(converted_output.cmd.mode), attitude->quat_body_to_ned[0],
            attitude->quat_body_to_ned[1], attitude->quat_body_to_ned[2], attitude->quat_body_to_ned[3],
            attitude->thrust_body[0], attitude->thrust_body[1], attitude->thrust_body[2], converted_output.eta_seconds,
            bool_name(converted_output.following), bool_name(converted_output.terminal_confirmed), error);
    } else if (const auto* velocity =
                   std::get_if<plane::algorithm::VelocityAccelerationCommand>(&converted_output.cmd.value)) {
        const std::string yaw = velocity->yaw_rad.has_value() ? std::to_string(*velocity->yaw_rad) : "null";
        SPDLOG_INFO(
            "MidcoursePlanner::update 输出: success={} "
            "output={{cmd={{mode={},value={{vn_m_s={:.2f},ve_m_s={:.2f},vd_m_s={:.2f},"
            "an_m_s2={:.2f},ae_m_s2={:.2f},ad_m_s2={:.2f},yaw_rad={}}}}},eta_seconds={:.2f},"
            "following={},terminal_confirmed={}}} error='{}'",
            bool_name(ok), control_mode_name(converted_output.cmd.mode), velocity->vn_m_s, velocity->ve_m_s,
            velocity->vd_m_s, velocity->an_m_s2, velocity->ae_m_s2, velocity->ad_m_s2, yaw,
            converted_output.eta_seconds, bool_name(converted_output.following),
            bool_name(converted_output.terminal_confirmed), error);
    }
    if (!ok) return false;
    if (converted_output.cmd.mode != plane::algorithm::MidcourseControlMode::Attitude) {
        error = "中制导算法未返回 Attitude 模式";
        SPDLOG_ERROR("MidcoursePlanner::update 校验失败: error='{}'", error);
        return false;
    }
    const auto* attitude = std::get_if<plane::algorithm::AttitudeCommand>(&converted_output.cmd.value);
    if (attitude == nullptr) {
        error = "中制导算法 Attitude 模式与指令类型不一致";
        SPDLOG_ERROR("MidcoursePlanner::update 校验失败: error='{}'", error);
        return false;
    }
    output.quat_body_to_ned = attitude->quat_body_to_ned;
    output.thrust_body = attitude->thrust_body;
    output.eta_seconds = converted_output.eta_seconds;
    output.following = converted_output.following;
    return true;
}

}  // namespace plane::task
