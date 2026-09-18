#pragma once

#include <string_view>

namespace plane::perception {
namespace events {

inline constexpr std::string_view kDetection = "perception.detection";
inline constexpr std::string_view kTracking = "perception.tracking";
inline constexpr std::string_view kGuidance = "perception.guidance";
inline constexpr std::string_view kError = "perception.error";

}  // namespace events
}  // namespace plane::perception
