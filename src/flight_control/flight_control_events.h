#pragma once

#include <string>
#include <string_view>

namespace plane::flight_control {

/// 飞控错误事件载荷。
struct FlightControlError {
    std::string detail;
};

namespace events {

inline constexpr std::string_view kError = "flight_control.error";

}  // namespace events

}  // namespace plane::flight_control
