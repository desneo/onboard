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
    std::string previous_flight_mode;
};

namespace events {

inline constexpr std::string_view kError = "flight_control.error";
// 载荷为 plane::model::Alert，通信模块只入队并由通信工作线程发送。
inline constexpr std::string_view kDiagnostic = "flight_control.diagnostic";
inline constexpr std::string_view kManualTakeover = "flight_control.manual_takeover";

}  // namespace events

}  // namespace plane::flight_control
