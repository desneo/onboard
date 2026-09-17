#include <spdlog/spdlog.h>

#include <string>
#include <utility>

#include "core/build_info.h"
#include "core/config.h"
#include "core/deps_check.h"
#include "core/log.h"
#include "core/placeholder_modules.h"
#include "core/runtime.h"

int main(int argc, char** argv) {
    std::string config_path = "config/onboard.conf";
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--deps-check") {
            return plane::core::run_deps_check();
        }
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        }
    }

    plane::core::AppConfig config;
    plane::core::load_config(config_path, config);

    plane::core::LogOptions log_options;
    log_options.log_dir = config.log_dir;
    plane::core::init_logging(log_options);

    const plane::core::BuildInfo build_info = plane::core::current_build_info();
    spdlog::info("onboard 启动: version={} git={} build={}", build_info.version, build_info.git_hash,
                 build_info.build_type);
    spdlog::info("配置: safe_height_m={} state_report_period_ms={}", config.system_param.safeHeight_m,
                 config.state_report_period_ms);

    plane::core::Runtime runtime;
    for (auto& module : plane::core::make_placeholder_modules()) {
        runtime.add_module(std::move(module));
    }
    if (!runtime.run()) {
        plane::core::shutdown_logging();
        return 1;
    }
    runtime.shutdown();

    plane::core::shutdown_logging();
    return 0;
}
