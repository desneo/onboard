#pragma once

#include <string_view>

namespace plane::perception {
namespace events {

inline constexpr std::string_view kDetection = "perception.detection";
inline constexpr std::string_view kError = "perception.error";

}  // namespace events
}  // namespace plane::perception
