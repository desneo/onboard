#pragma once

#include <string>

#include "flight_control/flight_control_api.h"

namespace plane::flight_control {

struct EulerAttitudeSetpoint {
    float roll_deg = 0.0F;
    float pitch_deg = 0.0F;
    float yaw_deg = 0.0F;
    float thrust = 0.0F;
};

bool convert_attitude_setpoint(const AttitudeSetpoint& input, EulerAttitudeSetpoint& output, std::string& error);

}  // namespace plane::flight_control
