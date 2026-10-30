#include "flight_control/attitude_conversion.h"

#include <algorithm>
#include <cmath>

namespace plane::flight_control {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHorizontalThrustTolerance = 1e-6;

}  // namespace

bool convert_attitude_setpoint(const AttitudeSetpoint& input, EulerAttitudeSetpoint& output, std::string& error) {
    for (double value : input.quat_body_to_ned) {
        if (!std::isfinite(value)) {
            error = "Attitude 四元数包含非有限值";
            return false;
        }
    }
    for (double value : input.thrust_body) {
        if (!std::isfinite(value)) {
            error = "Attitude 推力包含非有限值";
            return false;
        }
    }
    if (std::fabs(input.thrust_body[0]) > kHorizontalThrustTolerance ||
        std::fabs(input.thrust_body[1]) > kHorizontalThrustTolerance) {
        error = "Attitude 横向机体系推力必须为零";
        return false;
    }
    if (input.thrust_body[2] < 0.0 || input.thrust_body[2] > 1.0) {
        error = "Attitude 推力必须位于 [0, 1]";
        return false;
    }

    double w = input.quat_body_to_ned[0];
    double x = input.quat_body_to_ned[1];
    double y = input.quat_body_to_ned[2];
    double z = input.quat_body_to_ned[3];
    const double norm = std::sqrt(w * w + x * x + y * y + z * z);
    if (!std::isfinite(norm) || norm < 1e-9) {
        error = "Attitude 四元数模长非法";
        return false;
    }
    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;

    const double roll_rad = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double pitch_rad = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
    const double yaw_rad = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    output.roll_deg = static_cast<float>(roll_rad * 180.0 / kPi);
    output.pitch_deg = static_cast<float>(pitch_rad * 180.0 / kPi);
    output.yaw_deg = static_cast<float>(yaw_rad * 180.0 / kPi);
    output.thrust = static_cast<float>(input.thrust_body[2]);
    return true;
}

}  // namespace plane::flight_control
