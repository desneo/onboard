#include "core/log.h"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <memory>
#include <vector>

namespace plane::core {
void set_project_log_pattern() { spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%t] %s:%# %v"); }

void init_logging(const LogOptions& options) {
    std::filesystem::create_directories(options.log_dir);

    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        options.log_dir + "/" + options.app_name + ".log", options.max_file_size_bytes, options.max_files));

    auto logger = std::make_shared<spdlog::logger>(options.app_name, sinks.begin(), sinks.end());
    logger->set_level(spdlog::level::info);
    logger->flush_on(spdlog::level::warn);
    spdlog::set_default_logger(logger);
    set_project_log_pattern();
}

void shutdown_logging() { spdlog::shutdown(); }

}  // namespace plane::core
