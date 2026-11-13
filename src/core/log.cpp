#include "core/log.h"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <memory>
#include <vector>

#include "core/log_pattern.h"

namespace plane::core {
void init_logging(const LogOptions& options) {
    std::filesystem::create_directories(options.log_dir);

    std::vector<spdlog::sink_ptr> sinks;
    if (options.console) {
        sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    }
    sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        options.log_dir + "/" + options.app_name + ".log", options.max_file_size_bytes, options.max_files));

    auto logger = std::make_shared<spdlog::logger>(options.app_name, sinks.begin(), sinks.end());
    logger->set_level(spdlog::level::info);
    // 服务模式通过日志文件排查，及时写出启动和连接状态。
    logger->flush_on(options.console ? spdlog::level::warn : spdlog::level::info);
    spdlog::set_default_logger(logger);
    set_project_log_pattern();
}

void shutdown_logging() { spdlog::shutdown(); }

}  // namespace plane::core
