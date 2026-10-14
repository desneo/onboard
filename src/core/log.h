#pragma once

#include <cstddef>
#include <string>

namespace plane::core {

/// 日志配置；默认单文件 50 MB、保留 5 个历史文件（ARCHITECTURE §28.3）。
struct LogOptions {
    std::string app_name = "plane";
    std::string log_dir = "logs";
    std::size_t max_file_size_bytes = 50 * 1024 * 1024;
    std::size_t max_files = 5;
};

/// 初始化全局日志；项目代码统一通过 spdlog 输出。
void init_logging(const LogOptions& options);

/// 刷盘并释放全局日志。
void shutdown_logging();

}  // namespace plane::core
