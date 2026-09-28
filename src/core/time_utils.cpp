#include "core/time_utils.h"

#include <chrono>

namespace plane::core {

std::int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace plane::core
