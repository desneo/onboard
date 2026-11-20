#include "core/log.h"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <vector>

#include "core/log_pattern.h"

namespace plane::core {
void init_logging(const LogOptions& options) {
    if (options.flush_interval_ms <= 0) {
        throw std::invalid_argument("flush_interval_ms 必须为正整数");
    }
    // 先回收旧刷新线程并保留未写出的日志，再替换默认 logger。
    spdlog::flush_every(std::chrono::milliseconds::zero());
    if (auto logger = spdlog::default_logger()) {
        logger->flush();
    }
    std::filesystem::create_directories(options.log_dir);

    std::vector<spdlog::sink_ptr> sinks;
    if (options.console) {
        sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    }
    sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        options.log_dir + "/" + options.app_name + ".log", options.max_file_size_bytes, options.max_files));

    auto logger = std::make_shared<spdlog::logger>(options.app_name, sinks.begin(), sinks.end());
    logger->set_level(spdlog::level::info);
    // 警告及错误立即刷新，普通日志由单个周期线程批量刷新。
    logger->flush_on(spdlog::level::warn);
    spdlog::set_default_logger(logger);
    set_project_log_pattern();
    spdlog::flush_every(std::chrono::milliseconds(options.flush_interval_ms));
}

void shutdown_logging() {
    if (auto logger = spdlog::default_logger()) {
        logger->flush();
    }
    spdlog::shutdown();
}

}  // namespace plane::core
