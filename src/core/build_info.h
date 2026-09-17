#pragma once

#include <string>

namespace plane::core {

/// 构建版本信息，启动时输出（ARCHITECTURE §28.3）。
struct BuildInfo {
    std::string version;
    std::string git_hash;
    std::string build_type;
};

/// 返回当前程序的构建信息，取值来自 CMake 注入的宏。
BuildInfo current_build_info();

}  // namespace plane::core
