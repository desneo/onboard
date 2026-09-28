#pragma once

#include <cstdint>

namespace plane::core {

/// 单调时钟当前时间（毫秒，steady_clock）；供各模块统一取时间基准。
std::int64_t now_ms();

}  // namespace plane::core
