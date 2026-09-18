#include <spdlog/spdlog.h>

#include <chrono>
#include <csignal>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "communication/communication_events.h"
#include "communication/communication_module.h"
#include "core/build_info.h"
#include "core/config.h"
#include "core/deps_check.h"
#include "core/event_bus.h"
#include "core/log.h"
#include "core/placeholder_modules.h"
#include "core/runtime.h"
#include "flight_control/flight_control_events.h"
#include "flight_control/flight_control_module.h"
#include "state/state_store.h"
#include "task/task_scheduler.h"

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

void handle_stop_signal(int) { g_stop_requested = 1; }

}  // namespace

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

    plane::core::EventBus event_bus;
    event_bus.subscribe<plane::communication::CommunicationError>(
        plane::communication::events::kError,
        [](const plane::communication::CommunicationError& error) { spdlog::error("通信错误事件: {}", error.detail); });
    event_bus.subscribe<plane::flight_control::FlightControlError>(
        plane::flight_control::events::kError, [](const plane::flight_control::FlightControlError& error) {
            spdlog::error("飞控错误事件: {}", error.detail);
        });

    auto state_store = std::make_unique<plane::state::StateStore>(event_bus, config);
    auto flight_control = std::make_unique<plane::flight_control::FlightControlModule>(event_bus, config, *state_store);
    auto task_scheduler = std::make_unique<plane::task::TaskScheduler>(event_bus, config, *flight_control);
    auto communication =
        std::make_unique<plane::communication::CommunicationModule>(event_bus, config, *state_store, *task_scheduler);
    task_scheduler->set_communication(*communication);

    plane::core::Runtime runtime;
    runtime.add_module(std::move(state_store));
    runtime.add_module(std::move(flight_control));
    runtime.add_module(std::move(task_scheduler));
    runtime.add_module(std::move(communication));
    for (auto& module : plane::core::make_placeholder_modules(
             {"Communication", "UAV State Management", "Flight Control", "Task Scheduler"})) {
        runtime.add_module(std::move(module));
    }
    if (!runtime.run()) {
        plane::core::shutdown_logging();
        return 1;
    }

    std::signal(SIGINT, handle_stop_signal);
    std::signal(SIGTERM, handle_stop_signal);
    spdlog::info("系统运行中，Ctrl+C 退出");
    while (g_stop_requested == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    spdlog::info("收到退出信号，开始关闭");

    runtime.shutdown();

    plane::core::shutdown_logging();
    return 0;
}
