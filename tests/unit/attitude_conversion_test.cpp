#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>
#include <string>

#include "flight_control/attitude_conversion.h"

namespace {

bool near(float first, float second, float tolerance = 1e-3F) {
    return std::fabs(first - second) <= tolerance;
}

}  // namespace

int main() {
    using plane::flight_control::AttitudeSetpoint;
    using plane::flight_control::EulerAttitudeSetpoint;
    using plane::flight_control::convert_attitude_setpoint;

    std::string error;
    EulerAttitudeSetpoint output;

    AttitudeSetpoint identity;
    identity.thrust_body = {0.0, 0.0, 0.42};
    assert(convert_attitude_setpoint(identity, output, error));
    assert(near(output.roll_deg, 0.0F));
    assert(near(output.pitch_deg, 0.0F));
    assert(near(output.yaw_deg, 0.0F));
    assert(near(output.thrust, 0.42F));

    constexpr double kSqrtHalf = 0.7071067811865476;
    AttitudeSetpoint yaw_90;
    yaw_90.quat_body_to_ned = {kSqrtHalf * 2.0, 0.0, 0.0, kSqrtHalf * 2.0};
    yaw_90.thrust_body = {0.0, 0.0, 0.5};
    assert(convert_attitude_setpoint(yaw_90, output, error));
    assert(near(output.roll_deg, 0.0F));
    assert(near(output.pitch_deg, 0.0F));
    assert(near(output.yaw_deg, 90.0F));

    AttitudeSetpoint invalid = identity;
    invalid.quat_body_to_ned = {0.0, 0.0, 0.0, 0.0};
    assert(!convert_attitude_setpoint(invalid, output, error));
    invalid = identity;
    invalid.quat_body_to_ned[0] = std::numeric_limits<double>::quiet_NaN();
    assert(!convert_attitude_setpoint(invalid, output, error));
    invalid = identity;
    invalid.thrust_body[0] = 0.01;
    assert(!convert_attitude_setpoint(invalid, output, error));
    invalid = identity;
    invalid.thrust_body[2] = 1.01;
    assert(!convert_attitude_setpoint(invalid, output, error));
    return 0;
}
