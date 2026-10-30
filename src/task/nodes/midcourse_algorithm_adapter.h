#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace plane::task {

/// 与发布包 plane::model 类型隔离的中制导输入点。
struct MidcourseGeoPoint {
    double latitude = 0.0;
    double longitude = 0.0;
    float height = 0.0F;
    float altitude = 0.0F;
};

struct MidcourseMeasurement {
    MidcourseGeoPoint geo;
    std::int64_t timestamp_ms = 0;
};

struct MidcourseVehicleState {
    MidcourseGeoPoint geo;
    MidcourseGeoPoint start;
    double vn_m_s = 0.0;
    double ve_m_s = 0.0;
    double vd_m_s = 0.0;
};

struct MidcourseAlgorithmConfig {
    double max_horizontal_speed_m_s = 0.0;
    double max_climb_speed_m_s = 0.0;
    double max_descent_speed_m_s = 0.0;
    double max_acceleration_m_s2 = 0.0;
    double update_dt_s = 0.05;
    double target_invalid_grace_s = 3.0;
    double r_min_m = 50.0;
};

struct MidcourseAttitudeOutput {
    std::array<double, 4> quat_body_to_ned = {1.0, 0.0, 0.0, 0.0};
    std::array<double, 3> thrust_body = {0.0, 0.0, 0.0};
    double eta_seconds = -1.0;
    bool following = false;
};

/// 通过 common 公共接口直接构造 TargetEstimator/MidcoursePlanner；不使用 Pluma。
class MidcourseAlgorithmAdapter {
   public:
    MidcourseAlgorithmAdapter();
    ~MidcourseAlgorithmAdapter();

    MidcourseAlgorithmAdapter(const MidcourseAlgorithmAdapter&) = delete;
    MidcourseAlgorithmAdapter& operator=(const MidcourseAlgorithmAdapter&) = delete;

    bool init_estimator(const std::vector<MidcourseMeasurement>& history, const MidcourseGeoPoint& ned_reference,
                        double update_dt_s, std::string& error);
    bool set_config(const MidcourseAlgorithmConfig& config, std::string& error);
    bool update_target(const MidcourseMeasurement& measurement, std::string& error);
    bool update(const MidcourseVehicleState& vehicle, MidcourseAttitudeOutput& output, std::string& error);

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace plane::task
