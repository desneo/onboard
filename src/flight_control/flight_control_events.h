#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace plane::flight_control {

/// 飞控错误事件载荷。
struct FlightControlError {
    std::string detail;
};

struct ManualTakeoverEvent {
    std::uint64_t execution_id = 0;
    std::string flight_mode;
};

namespace events {

inline constexpr std::string_view kError = "flight_control.error";
inline constexpr std::string_view kManualTakeover = "flight_control.manual_takeover";

}  // namespace events

}  // namespace plane::flight_control
