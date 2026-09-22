#include <spdlog/spdlog.h>

#include <chrono>
#include <csignal>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "camera/camera_control_module.h"
#include "communication/communication_events.h"
#include "communication/communication_module.h"
#include "core/build_info.h"
#include "core/config.h"
#include "core/deps_check.h"
#include "core/event_bus.h"
#include "core/log.h"
#include "core/placeholder_modules.h"
#include "core/runtime.h"
#include "core/system_param_store.h"
#include "flight_control/flight_control_events.h"
#include "flight_control/flight_control_module.h"
#include "perception/perception.h"
#include "perception/plugin_management.h"
#include "state/state_monitor.h"
#include "state/state_store.h"
#include "target/target_store.h"
#include "task/task_execution_engine.h"
#include "task/task_scheduler.h"
#include "video/video_streaming.h"

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
    plane::core::LogOptions log_options;
    plane::core::init_logging(log_options);

    plane::core::load_config(config_path, config);
    if (!config.log_dir.empty() && config.log_dir != log_options.log_dir) {
        log_options.log_dir = config.log_dir;
        plane::core::shutdown_logging();
        plane::core::init_logging(log_options);
    }

    const plane::core::BuildInfo build_info = plane::core::current_build_info();
    SPDLOG_INFO("onboard 启动: version={} git={} build={}", build_info.version, build_info.git_hash,
                build_info.build_type);
    SPDLOG_INFO("配置: safe_height_m={} state_report_period_ms={} is_sim={}", config.system_param.safeHeight_m,
                config.state_report_period_ms, config.is_sim);

    plane::core::EventBus event_bus;
    event_bus.subscribe<plane::communication::CommunicationError>(
        plane::communication::events::kError,
        [](const plane::communication::CommunicationError& error) { SPDLOG_ERROR("通信错误事件: {}", error.detail); });
    event_bus.subscribe<plane::flight_control::FlightControlError>(
        plane::flight_control::events::kError,
        [](const plane::flight_control::FlightControlError& error) { SPDLOG_ERROR("飞控错误事件: {}", error.detail); });

    auto state_store = std::make_unique<plane::state::StateStore>(event_bus, config);
    auto flight_control = std::make_unique<plane::flight_control::FlightControlModule>(event_bus, config, *state_store);
    auto system_param_store = std::make_unique<plane::core::SystemParamStore>(event_bus, config);
    auto plugin_management = std::make_unique<plane::perception::PluginManagement>(config);
    auto camera_control = std::make_unique<plane::camera::CameraControlModule>(event_bus);
    auto video_streaming = std::make_unique<plane::video::VideoStreaming>(config);
    auto perception = std::make_unique<plane::perception::Perception>(event_bus, config, *plugin_management,
                                                                      *state_store, *flight_control, *video_streaming);
    auto target_store = std::make_unique<plane::target::TargetStore>(event_bus);
    auto task_engine = std::make_unique<plane::task::TaskExecutionEngine>(
        event_bus, config, *flight_control, *state_store, *system_param_store, *perception, *target_store);
    auto task_scheduler = std::make_unique<plane::task::TaskScheduler>(event_bus, config, *flight_control,
                                                                       *system_param_store, task_engine.get());
    auto communication =
        std::make_unique<plane::communication::CommunicationModule>(event_bus, config, *state_store, *task_scheduler);
    task_scheduler->set_communication(*communication);
    auto state_monitor = std::make_unique<plane::state::StateMonitor>(event_bus, config, *communication);

    plane::core::Runtime runtime;
    runtime.add_module(std::move(state_store));
    runtime.add_module(std::move(flight_control));
    runtime.add_module(std::move(system_param_store));
    runtime.add_module(std::move(plugin_management));
    runtime.add_module(std::move(camera_control));
    runtime.add_module(std::move(video_streaming));
    runtime.add_module(std::move(perception));
    runtime.add_module(std::move(target_store));
    runtime.add_module(std::move(task_engine));
    runtime.add_module(std::move(task_scheduler));
    runtime.add_module(std::move(communication));
    runtime.add_module(std::move(state_monitor));
    for (auto& module : plane::core::make_placeholder_modules(
             {"Communication", "UAV State Management", "Flight Control", "Task Scheduler", "Task Execution Engine",
              "Plugin Management", "Camera Control", "Video Streaming", "Perception", "Target Management",
              "StateMonitor"})) {
        runtime.add_module(std::move(module));
    }
    if (!runtime.run()) {
        plane::core::shutdown_logging();
        return 1;
    }

    std::signal(SIGINT, handle_stop_signal);
    std::signal(SIGTERM, handle_stop_signal);
    SPDLOG_INFO("系统运行中，Ctrl+C 退出");
    while (g_stop_requested == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    SPDLOG_INFO("收到退出信号，开始关闭");

    runtime.shutdown();

    plane::core::shutdown_logging();
    return 0;
}
