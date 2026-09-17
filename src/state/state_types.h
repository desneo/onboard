#pragma once

#include <cstdint>

namespace plane::state {

/// 地面站通信链路状态（SPEC §4.3）。
struct GcsLinkState {
    bool connected = false;
    std::int64_t last_seen_ms = 0;
};

}  // namespace plane::state
