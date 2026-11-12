#pragma once

#include <spdlog/spdlog.h>

namespace plane::core {

/// 为当前全局日志器设置统一格式；级别保留 spdlog 的英文标识。
inline void set_project_log_pattern() {
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%t] %s:%# %v");
}

}  // namespace plane::core
