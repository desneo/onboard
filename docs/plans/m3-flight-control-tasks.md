# M3 任务级计划（飞控接入：MAVSDK + PX4）

> 设计依据：`m3-flight-control-design.md`（已逐项确认）。
> 执行位置：工程根 `onboard/`。
> 验证原则：每个 Task 结束后 x64 Release 构建通过；不写测试代码（SPEC §11）。
> 本里程碑 SITL 不可用：Task 5 为部分验收，飞控动作链路列清单待用户 SITL 验证。

---

## Task 1：飞控接口与错误事件

**Files**

- Create: `src/flight_control/flight_control_api.h`
- Create: `src/flight_control/flight_control_events.h`

**Interfaces**

- Produces: `plane::flight_control::FlightControlApi`、`FlightControlError`、`events::kError`

- [ ] Step 1: 写 `src/flight_control/flight_control_api.h`

```cpp
#pragma once

#include <string>
#include <vector>

#include "message.h"

namespace plane::flight_control {

/// Flight Control 对外稳定能力接口（ARCHITECTURE §13）。
class FlightControlApi {
   public:
    virtual ~FlightControlApi() = default;

    virtual bool arm(std::string& error) = 0;
    virtual bool disarm(std::string& error) = 0;
    virtual bool takeoff(float height_m, std::string& error) = 0;
    virtual bool land(std::string& error) = 0;
    virtual bool return_to_launch(std::string& error) = 0;
    virtual bool hold(std::string& error) = 0;
    virtual bool stop_motor(std::string& error) = 0;
    virtual bool goto_waypoint(const plane::model::GeoPoint& point, std::string& error) = 0;
    virtual bool execute_route(const std::vector<plane::model::GeoPoint>& points, std::string& error) = 0;
    virtual bool set_ned_velocity(const plane::model::NedVelocity& velocity, std::string& error) = 0;
    virtual void clear_ned_velocity() = 0;

    virtual std::string link_state() const = 0;
};

}  // namespace plane::flight_control
```

- [ ] Step 2: 写 `src/flight_control/flight_control_events.h`

```cpp
#pragma once

#include <string>
#include <string_view>

namespace plane::flight_control {

/// 飞控错误事件载荷。
struct FlightControlError {
    std::string detail;
};

namespace events {

inline constexpr std::string_view kError = "flight_control.error";

}  // namespace events

}  // namespace plane::flight_control
```

- [ ] Step 3: 构建与提交

```bash
cmake --build build -j
clang-format --dry-run --Werror src/flight_control/flight_control_api.h src/flight_control/flight_control_events.h
git add src/flight_control/flight_control_api.h src/flight_control/flight_control_events.h
git commit -m "feat(m3): 增加飞控接口与错误事件"
```

---

## Task 2：飞控配置项

**Files**

- Modify: `src/core/config.h`
- Modify: `src/core/config.cpp`
- Modify: `config/onboard.conf`

- [ ] Step 1: `src/core/config.h` 的 `AppConfig` 中，在 `gcs_link_timeout_ms` 之后追加

```cpp
    std::string fc_link_type = "udp";
    std::string fc_udp_url = "udp://:14540";
    std::string fc_serial_device = "/dev/ttyUSB0";
    int fc_serial_baud = 57600;
    int fc_heartbeat_timeout_ms = 2000;
    double fc_route_speed_m_s = 5.0;
```

- [ ] Step 2: `src/core/config.cpp` 解析分支中，在 `gcs_link_timeout_ms` 分支之后追加

```cpp
        } else if (key == "fc_link_type") {
            out.fc_link_type = value;
        } else if (key == "fc_udp_url") {
            out.fc_udp_url = value;
        } else if (key == "fc_serial_device") {
            out.fc_serial_device = value;
        } else if (key == "fc_serial_baud") {
            out.fc_serial_baud = std::stoi(value);
        } else if (key == "fc_heartbeat_timeout_ms") {
            out.fc_heartbeat_timeout_ms = std::stoi(value);
        } else if (key == "fc_route_speed_m_s") {
            out.fc_route_speed_m_s = std::stod(value);
```

- [ ] Step 3: `config/onboard.conf` 末尾追加

```text
# 飞控连接（udp 或 serial）
fc_link_type = udp
fc_udp_url = udp://:14540
fc_serial_device = /dev/ttyUSB0
fc_serial_baud = 57600
fc_heartbeat_timeout_ms = 2000
fc_route_speed_m_s = 5.0
```

- [ ] Step 4: 构建、运行与提交

```bash
cmake --build build -j
timeout --preserve-status 2 ./build/onboard; echo "exit=$?"
clang-format --dry-run --Werror src/core/config.h src/core/config.cpp
git add src/core/config.h src/core/config.cpp config/onboard.conf
git commit -m "feat(m3): 增加飞控连接配置"
```

Expected: 构建通过；运行无「未知配置项」警告，exit=0。

---

## Task 3：FlightControlModule 实现

**Files**

- Create: `src/flight_control/flight_control_module.h`
- Create: `src/flight_control/flight_control_module.cpp`
- Modify: `CMakeLists.txt`

**Interfaces**

- Consumes: `FlightControlApi`/事件（Task 1）、`AppConfig`（Task 2）、`StateStoreApi`（M2）
- Produces: `plane::flight_control::FlightControlModule`

- [ ] Step 1: 写 `src/flight_control/flight_control_module.h`

```cpp
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"
#include "flight_control/flight_control_api.h"
#include "state/state_store_api.h"

namespace plane::flight_control {

/// 飞控接入：MAVSDK 连接管理、遥测聚合与能力分发（ARCHITECTURE §13）。
class FlightControlModule final : public plane::core::IModule, public FlightControlApi {
   public:
    FlightControlModule(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                        plane::state::StateStoreApi& state_store);
    ~FlightControlModule() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    bool arm(std::string& error) override;
    bool disarm(std::string& error) override;
    bool takeoff(float height_m, std::string& error) override;
    bool land(std::string& error) override;
    bool return_to_launch(std::string& error) override;
    bool hold(std::string& error) override;
    bool stop_motor(std::string& error) override;
    bool goto_waypoint(const plane::model::GeoPoint& point, std::string& error) override;
    bool execute_route(const std::vector<plane::model::GeoPoint>& points, std::string& error) override;
    bool set_ned_velocity(const plane::model::NedVelocity& velocity, std::string& error) override;
    void clear_ned_velocity() override;

    std::string link_state() const override;

   private:
    struct TelemetryCache {
        double latitude_deg = 0.0;
        double longitude_deg = 0.0;
        float relative_altitude_m = 0.0F;
        float absolute_altitude_m = 0.0F;
        double home_latitude_deg = 0.0;
        double home_longitude_deg = 0.0;
        float home_relative_altitude_m = 0.0F;
        float home_absolute_altitude_m = 0.0F;
        float roll_deg = 0.0F;
        float pitch_deg = 0.0F;
        float yaw_deg = 0.0F;
        float vn_m_s = 0.0F;
        float ve_m_s = 0.0F;
        float vd_m_s = 0.0F;
        float airspeed_m_s = 0.0F;
        float voltage_v = 0.0F;
        std::string flight_mode;
    };

    std::string connection_url() const;
    void monitor_loop();
    void state_loop();
    void offboard_loop();
    void attach_system(const std::shared_ptr<mavsdk::System>& system);
    void detach_system();
    bool ensure_connected(std::string& error) const;
    bool upload_and_start_mission(const std::vector<plane::model::GeoPoint>& points, std::string& error);
    void report_error(const std::string& detail) const;
    std::int64_t now_ms() const;

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    plane::state::StateStoreApi& state_store_;

    std::unique_ptr<mavsdk::Mavsdk> mavsdk_;
    std::shared_ptr<mavsdk::System> system_;
    std::unique_ptr<mavsdk::Action> action_;
    std::unique_ptr<mavsdk::Telemetry> telemetry_;
    std::unique_ptr<mavsdk::Mission> mission_;
    std::unique_ptr<mavsdk::Offboard> offboard_;

    mutable std::mutex cache_mutex_;
    TelemetryCache cache_;
    std::atomic<bool> connected_{false};

    std::mutex offboard_mutex_;
    mavsdk::Offboard::VelocityNedYaw setpoint_;
    bool offboard_active_ = false;

    std::thread monitor_thread_;
    std::thread state_thread_;
    std::thread offboard_thread_;
    std::atomic<bool> running_{false};
    std::int64_t last_wait_log_ms_ = 0;
};

}  // namespace plane::flight_control
```

- [ ] Step 2: 写 `src/flight_control/flight_control_module.cpp`

```cpp
#include "flight_control/flight_control_module.h"

#include <chrono>

#include <magic_enum/magic_enum.hpp>
#include <spdlog/spdlog.h>

#include "flight_control/flight_control_events.h"

namespace plane::flight_control {
namespace {

constexpr int kMonitorIntervalMs = 200;
constexpr int kStateIntervalMs = 100;
constexpr int kOffboardIntervalMs = 50;
constexpr int kWaitLogIntervalMs = 5000;

template <typename Result>
std::string result_text(Result result) {
    return std::string(magic_enum::enum_name(result));
}

}  // namespace

FlightControlModule::FlightControlModule(plane::core::EventBus& event_bus,
                                         const plane::core::AppConfig& config,
                                         plane::state::StateStoreApi& state_store)
    : event_bus_(event_bus), config_(config), state_store_(state_store) {}

FlightControlModule::~FlightControlModule() {
    stop();
}

std::string FlightControlModule::name() const {
    return "Flight Control";
}

bool FlightControlModule::init() {
    return true;
}

std::string FlightControlModule::connection_url() const {
    if (config_.fc_link_type == "serial") {
        return "serial://" + config_.fc_serial_device + ":" + std::to_string(config_.fc_serial_baud);
    }
    return config_.fc_udp_url;
}

bool FlightControlModule::start() {
    mavsdk_ = std::make_unique<mavsdk::Mavsdk>(
        mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::CompanionComputer});
    mavsdk_->set_heartbeat_timeout_s(static_cast<double>(config_.fc_heartbeat_timeout_ms) / 1000.0);

    const std::string url = connection_url();
    const mavsdk::ConnectionResult result = mavsdk_->add_any_connection(url);
    if (result != mavsdk::ConnectionResult::Success) {
        report_error("飞控连接初始化失败: " + url + " (" + result_text(result) + ")");
        return false;
    }
    spdlog::info("飞控连接参数: {} (心跳超时 {} ms)", url, config_.fc_heartbeat_timeout_ms);

    running_ = true;
    monitor_thread_ = std::thread([this]() { monitor_loop(); });
    state_thread_ = std::thread([this]() { state_loop(); });
    offboard_thread_ = std::thread([this]() { offboard_loop(); });
    return true;
}

void FlightControlModule::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    clear_ned_velocity();
    if (offboard_thread_.joinable()) {
        offboard_thread_.join();
    }
    if (state_thread_.joinable()) {
        state_thread_.join();
    }
    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }
    detach_system();
    mavsdk_.reset();
}

void FlightControlModule::release() {}

void FlightControlModule::monitor_loop() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kMonitorIntervalMs));
        const std::vector<std::shared_ptr<mavsdk::System>> systems = mavsdk_->systems();
        if (!system_) {
            for (const auto& candidate : systems) {
                if (candidate->has_autopilot()) {
                    attach_system(candidate);
                    break;
                }
            }
            if (!system_) {
                const std::int64_t now = now_ms();
                if (now - last_wait_log_ms_ >= kWaitLogIntervalMs) {
                    last_wait_log_ms_ = now;
                    spdlog::info("等待飞控连接: {}", connection_url());
                }
            }
            continue;
        }
        bool found = false;
        for (const auto& candidate : systems) {
            if (candidate.get() == system_.get()) {
                found = true;
                break;
            }
        }
        if (!found) {
            detach_system();
        }
    }
}

void FlightControlModule::attach_system(const std::shared_ptr<mavsdk::System>& system) {
    system_ = system;
    action_ = std::make_unique<mavsdk::Action>(system_);
    telemetry_ = std::make_unique<mavsdk::Telemetry>(system_);
    mission_ = std::make_unique<mavsdk::Mission>(system_);
    offboard_ = std::make_unique<mavsdk::Offboard>(system_);

    telemetry_->subscribe_position([this](mavsdk::Telemetry::Position value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.latitude_deg = value.latitude_deg;
        cache_.longitude_deg = value.longitude_deg;
        cache_.relative_altitude_m = value.relative_altitude_m;
        cache_.absolute_altitude_m = value.absolute_altitude_m;
    });
    telemetry_->subscribe_home([this](mavsdk::Telemetry::Position value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.home_latitude_deg = value.latitude_deg;
        cache_.home_longitude_deg = value.longitude_deg;
        cache_.home_relative_altitude_m = value.relative_altitude_m;
        cache_.home_absolute_altitude_m = value.absolute_altitude_m;
    });
    telemetry_->subscribe_attitude_euler([this](mavsdk::Telemetry::EulerAngle value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.roll_deg = value.roll_deg;
        cache_.pitch_deg = value.pitch_deg;
        cache_.yaw_deg = value.yaw_deg;
    });
    telemetry_->subscribe_velocity_ned([this](mavsdk::Telemetry::VelocityNed value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.vn_m_s = value.north_m_s;
        cache_.ve_m_s = value.east_m_s;
        cache_.vd_m_s = value.down_m_s;
    });
    telemetry_->subscribe_fixedwing_metrics([this](mavsdk::Telemetry::FixedwingMetrics value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.airspeed_m_s = value.airspeed_m_s;
    });
    telemetry_->subscribe_battery([this](mavsdk::Telemetry::Battery value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.voltage_v = value.voltage_v;
    });
    telemetry_->subscribe_flight_mode([this](mavsdk::Telemetry::FlightMode value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.flight_mode = std::string(magic_enum::enum_name(value));
    });
    system_->subscribe_is_connected([this](bool connected) {
        connected_ = connected;
        spdlog::info("飞控连接{}", connected ? "建立" : "异常");
    });

    spdlog::info("飞控系统已接入: system_id={}", system_->get_system_id());
}

void FlightControlModule::detach_system() {
    {
        std::lock_guard<std::mutex> lock(offboard_mutex_);
        offboard_active_ = false;
        offboard_.reset();
    }
    connected_ = false;
    mission_.reset();
    telemetry_.reset();
    action_.reset();
    if (system_) {
        spdlog::info("飞控系统已移除");
        system_.reset();
    }
}

void FlightControlModule::state_loop() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kStateIntervalMs));
        plane::model::VehicleState state;
        state.planeId = config_.plane_id;
        {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            state.geoPos.latitude = cache_.latitude_deg;
            state.geoPos.longitude = cache_.longitude_deg;
            state.geoPos.height = cache_.relative_altitude_m;
            state.geoPos.altitude = cache_.absolute_altitude_m;
            state.startPos.latitude = cache_.home_latitude_deg;
            state.startPos.longitude = cache_.home_longitude_deg;
            state.startPos.height = cache_.home_relative_altitude_m;
            state.startPos.altitude = cache_.home_absolute_altitude_m;
            state.roll_deg = cache_.roll_deg;
            state.pitch_deg = cache_.pitch_deg;
            state.yaw_deg = cache_.yaw_deg;
            state.vn_m_s = cache_.vn_m_s;
            state.ve_m_s = cache_.ve_m_s;
            state.vd_m_s = cache_.vd_m_s;
            state.airspeed_m_s = cache_.airspeed_m_s;
            state.voltage_v = cache_.voltage_v;
            state.flightMode = cache_.flight_mode;
            state.fcLinkState = connected_ ? "connected" : "disconnected";
        }
        state_store_.update_own_state(state);
    }
}

void FlightControlModule::offboard_loop() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kOffboardIntervalMs));
        std::lock_guard<std::mutex> lock(offboard_mutex_);
        if (offboard_active_ && offboard_) {
            offboard_->set_velocity_ned(setpoint_);
        }
    }
}

bool FlightControlModule::ensure_connected(std::string& error) const {
    if (!system_ || !action_ || !connected_) {
        error = "飞控未连接";
        return false;
    }
    return true;
}

bool FlightControlModule::arm(std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    const mavsdk::Action::Result result = action_->arm();
    if (result != mavsdk::Action::Result::Success) {
        error = "Arm 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    return true;
}

bool FlightControlModule::disarm(std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    const mavsdk::Action::Result result = action_->disarm();
    if (result != mavsdk::Action::Result::Success) {
        error = "Disarm 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    return true;
}

bool FlightControlModule::takeoff(float height_m, std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    mavsdk::Action::Result result = action_->set_takeoff_altitude(height_m);
    if (result != mavsdk::Action::Result::Success) {
        error = "设置起飞高度失败: " + result_text(result);
        report_error(error);
        return false;
    }
    result = action_->takeoff();
    if (result != mavsdk::Action::Result::Success) {
        error = "Takeoff 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    return true;
}

bool FlightControlModule::land(std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    const mavsdk::Action::Result result = action_->land();
    if (result != mavsdk::Action::Result::Success) {
        error = "Land 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    return true;
}

bool FlightControlModule::return_to_launch(std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    const mavsdk::Action::Result result = action_->return_to_launch();
    if (result != mavsdk::Action::Result::Success) {
        error = "RTL 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    return true;
}

bool FlightControlModule::hold(std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    const mavsdk::Action::Result result = action_->hold();
    if (result != mavsdk::Action::Result::Success) {
        error = "Hold 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    return true;
}

bool FlightControlModule::stop_motor(std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    const mavsdk::Action::Result result = action_->kill();
    if (result != mavsdk::Action::Result::Success) {
        error = "StopMotor 失败: " + result_text(result);
        report_error(error);
        return false;
    }
    return true;
}

bool FlightControlModule::goto_waypoint(const plane::model::GeoPoint& point, std::string& error) {
    return upload_and_start_mission({point}, error);
}

bool FlightControlModule::execute_route(const std::vector<plane::model::GeoPoint>& points,
                                        std::string& error) {
    return upload_and_start_mission(points, error);
}

bool FlightControlModule::upload_and_start_mission(const std::vector<plane::model::GeoPoint>& points,
                                                   std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    if (points.empty()) {
        error = "航点列表为空";
        return false;
    }
    mavsdk::Mission::MissionPlan plan;
    for (const auto& point : points) {
        mavsdk::Mission::MissionItem item;
        item.latitude_deg = point.latitude;
        item.longitude_deg = point.longitude;
        item.relative_altitude_m = point.height;
        item.speed_m_s = static_cast<float>(config_.fc_route_speed_m_s);
        item.is_fly_through = true;
        plan.mission_items.push_back(item);
    }
    mavsdk::Mission::Result result = mission_->upload_mission(plan);
    if (result != mavsdk::Mission::Result::Success) {
        error = "上传航线失败: " + result_text(result);
        report_error(error);
        return false;
    }
    result = mission_->start_mission();
    if (result != mavsdk::Mission::Result::Success) {
        error = "启动航线失败: " + result_text(result);
        report_error(error);
        return false;
    }
    return true;
}

bool FlightControlModule::set_ned_velocity(const plane::model::NedVelocity& velocity,
                                           std::string& error) {
    if (!ensure_connected(error)) {
        return false;
    }
    mavsdk::Offboard::VelocityNedYaw setpoint;
    setpoint.north_m_s = static_cast<float>(velocity.north_m_s);
    setpoint.east_m_s = static_cast<float>(velocity.east_m_s);
    setpoint.down_m_s = static_cast<float>(velocity.down_m_s);
    setpoint.yaw_deg = static_cast<float>(velocity.yaw_deg);

    std::lock_guard<std::mutex> lock(offboard_mutex_);
    setpoint_ = setpoint;
    if (offboard_active_) {
        return true;
    }
    mavsdk::Offboard::Result result = offboard_->set_velocity_ned(setpoint_);
    if (result != mavsdk::Offboard::Result::Success) {
        error = "Offboard 设定值下发失败: " + result_text(result);
        report_error(error);
        return false;
    }
    result = offboard_->start();
    if (result != mavsdk::Offboard::Result::Success) {
        error = "Offboard 启动失败: " + result_text(result);
        report_error(error);
        return false;
    }
    offboard_active_ = true;
    spdlog::info("Offboard 已启动");
    return true;
}

void FlightControlModule::clear_ned_velocity() {
    std::lock_guard<std::mutex> lock(offboard_mutex_);
    if (!offboard_active_) {
        return;
    }
    offboard_active_ = false;
    if (offboard_) {
        offboard_->stop();
        spdlog::info("Offboard 已停止");
    }
}

std::string FlightControlModule::link_state() const {
    return connected_ ? "connected" : "disconnected";
}

void FlightControlModule::report_error(const std::string& detail) const {
    FlightControlError event;
    event.detail = detail;
    event_bus_.publish(events::kError, event);
}

std::int64_t FlightControlModule::now_ms() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace plane::flight_control
```

- [ ] Step 3: `CMakeLists.txt` 的 `add_executable(onboard ...)` 文件列表中追加

```cmake
  src/flight_control/flight_control_module.cpp
```

- [ ] Step 4: 构建与格式

```bash
cmake --build build -j
clang-format --dry-run --Werror src/flight_control/flight_control_module.h src/flight_control/flight_control_module.cpp
```

Expected: 构建通过且无 warning；格式通过（有差异先 `clang-format -i` 再构建）。

- [ ] Step 5: 提交

```bash
git add src/flight_control/flight_control_module.h src/flight_control/flight_control_module.cpp CMakeLists.txt
git commit -m "feat(m3): 增加 FlightControlModule"
```

---

## Task 4：main 接线与无 SITL 运行

**Files**

- Modify: `src/main.cpp`

**Interfaces**

- Consumes: `FlightControlModule`（Task 3）

- [ ] Step 1: `src/main.cpp` include 区追加

```cpp
#include "flight_control/flight_control_events.h"
#include "flight_control/flight_control_module.h"
```

- [ ] Step 2: 错误事件订阅追加（与 CommunicationError 订阅并列）

```cpp
    event_bus.subscribe<plane::flight_control::FlightControlError>(
        plane::flight_control::events::kError,
        [](const plane::flight_control::FlightControlError& error) {
            spdlog::error("飞控错误事件: {}", error.detail);
        });
```

- [ ] Step 3: 模块注册块改为

```cpp
    plane::task::NoTaskStateProvider task_state_provider;
    auto state_store = std::make_unique<plane::state::StateStore>(event_bus, config);
    auto flight_control =
        std::make_unique<plane::flight_control::FlightControlModule>(event_bus, config, *state_store);
    auto communication = std::make_unique<plane::communication::CommunicationModule>(
        event_bus, config, *state_store, task_state_provider);

    plane::core::Runtime runtime;
    runtime.add_module(std::move(state_store));
    runtime.add_module(std::move(flight_control));
    runtime.add_module(std::move(communication));
    for (auto& module :
         plane::core::make_placeholder_modules({"Communication", "UAV State Management", "Flight Control"})) {
        runtime.add_module(std::move(module));
    }
```

- [ ] Step 4: 构建、运行与格式

```bash
cmake --build build -j
timeout --preserve-status 6 ./build/onboard; echo "exit=$?"
clang-format --dry-run --Werror src/main.cpp
```

Expected: 构建通过；运行日志包含 `飞控连接参数: udp://:14540 (心跳超时 2000 ms)`、`Communication 已启动`；首个 `等待飞控连接: udp://:14540` 在启动后约 0.2 秒出现，之后每 5 秒一次；Ctrl+C/超时退出 exit=0。

- [ ] Step 5: 提交

```bash
git add src/main.cpp
git commit -m "feat(m3): 接线 FlightControl 模块"
```

---

## Task 5：M3 部分验收

**Files**

- Modify: `tools/gcs_sim/main.cpp`
- Modify: `docs/plans/m3-flight-control-tasks.md`
- Modify: `docs/DEV_PLAN.md`

- [ ] Step 1: gcs_sim 的 VehicleState 日志补充 fcLinkState/flightMode

将 `on_sample` 中 `VehicleState` 的打印改为包含下列字段：

```cpp
spdlog::info(
    "收到 VehicleState: planeId={}, fcLinkState={}, flightMode={}, voltage={}, lat={}, lon={}, height={}",
    payload.value("planeId", std::string{}), payload.value("fcLinkState", std::string{}),
    payload.value("flightMode", std::string{}), payload.value("voltage_v", 0.0F),
    geo.value("latitude", 0.0), geo.value("longitude", 0.0), geo.value("height", 0.0F));
```

构建、格式与提交：

```bash
cmake --build build -j
clang-format --dry-run --Werror tools/gcs_sim/main.cpp
git add tools/gcs_sim/main.cpp
git commit -m "feat(m3): gcs_sim 输出飞控连接与飞行模式"
```

- [ ] Step 2: 干净构建

```bash
rm -rf build logs
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

- [ ] Step 3: 无 SITL 连接状态

终端 A：

```bash
./build/onboard
```

Expected：日志 `飞控连接参数: udp://:14540`，随后每 5 秒 `等待飞控连接: udp://:14540`，无崩溃。

- [ ] Step 4: 状态上报联动（终端 B）

```bash
./build/gcs_sim --plane-id uav1 --listen-seconds 3
```

Expected：gcs_sim 收到 `VehicleState`，其中 `fcLinkState=disconnected`、`flightMode=`（为空，无 SITL）（遥测聚合与上报链路可用）；TaskInfo/Command 的 Ack 正常。

- [ ] Step 5: 规范检查

```bash
clang-format --dry-run --Werror $(git ls-files src tools)
grep -rn "std::cout\|printf(" src/ tools/ || echo "no cout/printf"
```

- [ ] Step 6: 文档与收尾提交

- 勾选本文档，并在文末记录执行偏差与验收证据；
- 更新 `docs/DEV_PLAN.md` §11 索引：M3 状态 → 已完成（部分验收，SITL 待验）；
- 将「SITL 待验清单」保留在本文档，供用户后续运行定位。

```bash
git add docs
git commit -m "docs(m3): M3 部分验收与文档更新"
```

---

## 附：SITL 待用户验证清单（M3 交付）

用户环境具备 SITL 后，按如下顺序验证并反馈日志：

1. UDP 连接建立（`udp://:14540`）与串口连接建立（`serial:///dev/ttyUSB0:57600`）；
2. 断开数据 2 秒判连接异常、恢复后自动重连（观察 `飞控连接异常/建立` 日志）；
3. Arm → Takeoff(安全高度) → Hold → Land；Disarm；RTL；
4. Waypoint（单点）与 Route（多点，`fc_route_speed_m_s`）；
5. StopMotor（仅 GCS 人工触发，注意安全，SITL 中验证 kill 行为）；
6. NED Velocity：设置后 PX4 进入 Offboard，`clear_ned_velocity` 后退出；
7. `VehicleState` 各字段（经纬高、姿态、NED 速度、空速、电压、飞行模式、fcLinkState）与 SITL 状态一致。
