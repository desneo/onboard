# M6 任务级计划（相机、图像、插件框架与感知）

> 设计依据：`m6-perception-plugins-design.md`（逐项确认）。
> 执行位置：工程根 `onboard/`。
> 验证原则：每个 Task 结束后 x64 Release 构建通过；不写测试代码（SPEC §11）。
> 状态：已完成并部分验收（2026-09-18）；RTSP/MIPI 实流取图待用户提供源。

---

## Task 1：M6 配置项

**Files**

- Modify: `src/core/config.h`
- Modify: `src/core/config.cpp`
- Modify: `config/onboard.conf`

- [x] Step 1: `src/core/config.h` 的 `AppConfig` 中，在 `bt_takeoff_tolerance_m` 之后追加

```cpp
    std::string plugins_dir = "build/plugins";
    std::string image_source;
    int image_reconnect_interval_ms = 2000;
    int perception_process_interval_ms = 100;
    bool perception_selftest_on_start = false;
```

- [x] Step 2: `src/core/config.cpp` 解析分支中，在 `bt_takeoff_tolerance_m` 分支之后追加

```cpp
        } else if (key == "plugins_dir") {
            out.plugins_dir = value;
        } else if (key == "image_source") {
            out.image_source = value;
        } else if (key == "image_reconnect_interval_ms") {
            out.image_reconnect_interval_ms = std::stoi(value);
        } else if (key == "perception_process_interval_ms") {
            out.perception_process_interval_ms = std::stoi(value);
        } else if (key == "perception_selftest_on_start") {
            out.perception_selftest_on_start = (value == "true" || value == "1");
```

- [x] Step 3: `config/onboard.conf` 末尾追加

```text
# 感知与插件
plugins_dir = build/plugins
image_source =
image_reconnect_interval_ms = 2000
perception_process_interval_ms = 100
perception_selftest_on_start = false
```

- [x] Step 4: 构建、运行与提交

```bash
cmake --build build -j
timeout --preserve-status 2 ./build/onboard; echo "exit=$?"
clang-format --dry-run --Werror src/core/config.h src/core/config.cpp
git add src/core/config.h src/core/config.cpp config/onboard.conf
git commit -m "feat(m6): 增加感知与插件配置"
```

Expected: 构建通过；运行无「未知配置项」，exit=0。

---

## Task 2：Camera Control（接口 + Noop 模块）

**Files**

- Create: `src/camera/camera_control_api.h`
- Create: `src/camera/camera_control_events.h`
- Create: `src/camera/camera_control_module.h`
- Create: `src/camera/camera_control_module.cpp`
- Modify: `CMakeLists.txt`

- [x] Step 1: 写 `src/camera/camera_control_api.h`

```cpp
#pragma once

#include <string>

namespace plane::camera {

/// 相机链路状态。
struct CameraStatus {
    bool connected = false;
    std::string detail;
};

/// 相机链路与相机能力接口（SPEC §8.1）；控制指令集待 SDK 到位后细化。
class CameraControlApi {
   public:
    virtual ~CameraControlApi() = default;

    virtual bool connect(std::string& error) = 0;
    virtual void disconnect() = 0;
    virtual CameraStatus status() const = 0;
    virtual bool send_command(const std::string& command, std::string& error) = 0;
};

}  // namespace plane::camera
```

- [x] Step 2: 写 `src/camera/camera_control_events.h`

```cpp
#pragma once

#include <string_view>

namespace plane::camera {
namespace events {

inline constexpr std::string_view kStatusChanged = "camera.status_changed";

}  // namespace events
}  // namespace plane::camera
```

- [x] Step 3: 写 `src/camera/camera_control_module.h`

```cpp
#pragma once

#include <mutex>
#include <string>

#include "camera/camera_control_api.h"
#include "core/event_bus.h"
#include "core/module.h"

namespace plane::camera {

/// 第一版 Noop 实现：仅维护链路状态与日志；相机 SDK 接入时替换内部实现。
class CameraControlModule final : public plane::core::IModule, public CameraControlApi {
   public:
    explicit CameraControlModule(plane::core::EventBus& event_bus);
    ~CameraControlModule() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    bool connect(std::string& error) override;
    void disconnect() override;
    CameraStatus status() const override;
    bool send_command(const std::string& command, std::string& error) override;

   private:
    plane::core::EventBus& event_bus_;
    mutable std::mutex mutex_;
    CameraStatus status_;
};

}  // namespace plane::camera
```

- [x] Step 4: 写 `src/camera/camera_control_module.cpp`

```cpp
#include "camera/camera_control_module.h"

#include <spdlog/spdlog.h>

#include "camera/camera_control_events.h"

namespace plane::camera {

CameraControlModule::CameraControlModule(plane::core::EventBus& event_bus) : event_bus_(event_bus) {}

CameraControlModule::~CameraControlModule() {
    stop();
}

std::string CameraControlModule::name() const {
    return "Camera Control";
}

bool CameraControlModule::init() {
    return true;
}

bool CameraControlModule::start() {
    std::string error;
    if (!connect(error)) {
        spdlog::error("相机链路自动建联失败: {}", error);
        return false;
    }
    return true;
}

void CameraControlModule::stop() {
    disconnect();
}

void CameraControlModule::release() {}

bool CameraControlModule::connect(std::string&) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.connected = true;
        status_.detail = "noop";
    }
    spdlog::info("相机链路已建立 (Noop)");
    event_bus_.publish(events::kStatusChanged, status());
    return true;
}

void CameraControlModule::disconnect() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.connected = false;
        status_.detail = "noop";
    }
    spdlog::info("相机链路已断开 (Noop)");
    event_bus_.publish(events::kStatusChanged, status());
}

CameraStatus CameraControlModule::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

bool CameraControlModule::send_command(const std::string& command, std::string&) {
    spdlog::info("相机指令 (Noop): {}", command);
    return true;
}

}  // namespace plane::camera
```

- [x] Step 5: `CMakeLists.txt` 的 `add_executable(onboard ...)` 文件列表追加

```cmake
  src/camera/camera_control_module.cpp
```

- [x] Step 6: 构建与格式

```bash
cmake --build build -j
clang-format --dry-run --Werror src/camera/*.h src/camera/*.cpp
```

Expected: 构建通过；格式通过（差异先 `-i` 再构建）。

- [x] Step 7: 提交

```bash
git add src/camera CMakeLists.txt
git commit -m "feat(m6): 增加 Camera Control 接口与 Noop 模块"
```

---

## Task 3：Plugin Management（Pluma 宿主侧）

**Files**

- Create: `src/perception/plugin_manager_api.h`
- Create: `src/perception/plugin_management.h`
- Create: `src/perception/plugin_management.cpp`
- Create: `src/perception/plugin_providers.cpp`
- Modify: `CMakeLists.txt`

- [x] Step 1: 写 `src/perception/plugin_manager_api.h`

```cpp
#pragma once

#include <memory>

#include "functional/detection_api.h"
#include "functional/guidance_api.h"
#include "functional/tracking_api.h"

namespace plane::perception {

/// 插件按需实例化接口（ARCHITECTURE §19）。
class PluginManagerApi {
   public:
    virtual ~PluginManagerApi() = default;

    virtual std::shared_ptr<plane::algorithm::DetectionPlugin> create_detection() = 0;
    virtual std::shared_ptr<plane::algorithm::TrackingPlugin> create_tracking() = 0;
    virtual std::shared_ptr<plane::algorithm::GuidancePlugin> create_guidance() = 0;
};

}  // namespace plane::perception
```

- [x] Step 2: 写 `src/perception/plugin_management.h`

```cpp
#pragma once

#include <memory>
#include <string>

#include <Pluma/Pluma.hpp>

#include "core/config.h"
#include "core/module.h"
#include "perception/plugin_manager_api.h"

namespace plane::perception {

/// Plugin Management：插件库加载与对象按需实例化（ARCHITECTURE §19）。
class PluginManagement final : public plane::core::IModule, public PluginManagerApi {
   public:
    explicit PluginManagement(const plane::core::AppConfig& config);
    ~PluginManagement() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    std::shared_ptr<plane::algorithm::DetectionPlugin> create_detection() override;
    std::shared_ptr<plane::algorithm::TrackingPlugin> create_tracking() override;
    std::shared_ptr<plane::algorithm::GuidancePlugin> create_guidance() override;

   private:
    const plane::core::AppConfig& config_;
    pluma::Pluma pluma_;
    bool loaded_ = false;
};

}  // namespace plane::perception
```

- [x] Step 3: 写 `src/perception/plugin_management.cpp`

```cpp
#include "perception/plugin_management.h"

#include <vector>

#include <spdlog/spdlog.h>

namespace plane::perception {
namespace {

template <typename PluginType, typename ProviderType>
std::shared_ptr<PluginType> create_first(pluma::Pluma& pluma, const char* type_name) {
    std::vector<ProviderType*> providers;
    pluma.getProviders(providers);
    if (providers.empty()) {
        spdlog::warn("未找到插件 provider: {}", type_name);
        return nullptr;
    }
    return std::shared_ptr<PluginType>(providers.front()->create());
}

template <typename ProviderType>
std::size_t provider_count(pluma::Pluma& pluma) {
    std::vector<ProviderType*> providers;
    pluma.getProviders(providers);
    return providers.size();
}

}  // namespace

PluginManagement::PluginManagement(const plane::core::AppConfig& config) : config_(config) {}

PluginManagement::~PluginManagement() {
    stop();
}

std::string PluginManagement::name() const {
    return "Plugin Management";
}

bool PluginManagement::init() {
    return true;
}

bool PluginManagement::start() {
    pluma_.acceptProviderType<plane::algorithm::DetectionPluginProvider>();
    pluma_.acceptProviderType<plane::algorithm::TrackingPluginProvider>();
    pluma_.acceptProviderType<plane::algorithm::GuidancePluginProvider>();

    const int loaded = pluma_.loadFromFolder(config_.plugins_dir);
    loaded_ = true;
    spdlog::info("插件目录加载: {} (成功加载 {} 个库)", config_.plugins_dir, loaded);
    spdlog::info("插件 provider: detection={}, tracking={}, guidance={}",
                 provider_count<plane::algorithm::DetectionPluginProvider>(pluma_),
                 provider_count<plane::algorithm::TrackingPluginProvider>(pluma_),
                 provider_count<plane::algorithm::GuidancePluginProvider>(pluma_));
    return true;
}

void PluginManagement::stop() {
    if (!loaded_) {
        return;
    }
    pluma_.unloadAll();
    loaded_ = false;
}

void PluginManagement::release() {}

std::shared_ptr<plane::algorithm::DetectionPlugin> PluginManagement::create_detection() {
    return create_first<plane::algorithm::DetectionPlugin, plane::algorithm::DetectionPluginProvider>(
        pluma_, "DetectionPlugin");
}

std::shared_ptr<plane::algorithm::TrackingPlugin> PluginManagement::create_tracking() {
    return create_first<plane::algorithm::TrackingPlugin, plane::algorithm::TrackingPluginProvider>(
        pluma_, "TrackingPlugin");
}

std::shared_ptr<plane::algorithm::GuidancePlugin> PluginManagement::create_guidance() {
    return create_first<plane::algorithm::GuidancePlugin, plane::algorithm::GuidancePluginProvider>(
        pluma_, "GuidancePlugin");
}

}  // namespace plane::perception
```

- [x] Step 4: 写 `src/perception/plugin_providers.cpp`（宿主侧 Provider 静态成员定义）

```cpp
#include "functional/detection_api.h"
#include "functional/guidance_api.h"
#include "functional/tracking_api.h"

namespace plane::algorithm {

PLUMA_PROVIDER_SOURCE(DetectionPlugin, 1, 1);
PLUMA_PROVIDER_SOURCE(TrackingPlugin, 1, 1);
PLUMA_PROVIDER_SOURCE(GuidancePlugin, 1, 1);

}  // namespace plane::algorithm
```

- [x] Step 5: `CMakeLists.txt` 的 `add_executable(onboard ...)` 文件列表追加

```cmake
  src/perception/plugin_management.cpp
  src/perception/plugin_providers.cpp
```

- [x] Step 6: 构建与格式

```bash
cmake --build build -j
clang-format --dry-run --Werror $(git ls-files src/perception)
```

Expected: 构建通过；格式通过（差异先 `-i` 再构建）。

- [x] Step 7: 提交

```bash
git add src/perception CMakeLists.txt
git commit -m "feat(m6): 增加 Plugin Management 与宿主 Provider 定义"
```

---

## Task 4：三个示例插件

**Files**

- Create: `plugins/detection_stub/detection_stub.cpp`
- Create: `plugins/tracking_stub/tracking_stub.cpp`
- Create: `plugins/guidance_stub/guidance_stub.cpp`
- Modify: `CMakeLists.txt`

- [x] Step 1: 写 `plugins/detection_stub/detection_stub.cpp`

```cpp
#include <memory>
#include <string>
#include <vector>

#include <Pluma/Connector.hpp>
#include <Pluma/Pluma.hpp>
#include <opencv2/core.hpp>
#include <spdlog/spdlog.h>

#include "functional/detection_api.h"

namespace plane::algorithm {
namespace {

class DetectionStub final : public DetectionPlugin {
   public:
    bool init(const DetectConfig& config) override {
        config_ = config;
        spdlog::info("DetectionStub init: model_path={}", config_.model_path);
        return true;
    }

    bool detect(const cv::Mat&, std::vector<DetectTarget>& out, std::string&) override {
        out.clear();
        return true;
    }

   private:
    DetectConfig config_;
};

}  // namespace

PLUMA_INHERIT_PROVIDER(DetectionStub, DetectionPlugin);
PLUMA_PROVIDER_SOURCE(DetectionPlugin, 1, 1);

PLUMA_CONNECTOR
bool connectToPlumaHost(pluma::Host& host) {
    host.add(new DetectionStubProvider());
    return true;
}

}  // namespace plane::algorithm
```

- [x] Step 2: 写 `plugins/tracking_stub/tracking_stub.cpp`

```cpp
#include <memory>
#include <string>

#include <Pluma/Connector.hpp>
#include <Pluma/Pluma.hpp>
#include <opencv2/core.hpp>
#include <spdlog/spdlog.h>

#include "functional/tracking_api.h"

namespace plane::algorithm {
namespace {

class TrackingStub final : public TrackingPlugin {
   public:
    bool init(const TrackConfig& config) override {
        config_ = config;
        spdlog::info("TrackingStub init: model_path={}", config_.model_path);
        return true;
    }

    bool start(const cv::Mat&, const DetectTarget& initTarget, std::string&) override {
        target_ = initTarget;
        return true;
    }

    bool track(const cv::Mat&, DetectTarget& out, std::string&) override {
        out = target_;
        return true;
    }

   private:
    TrackConfig config_;
    DetectTarget target_;
};

}  // namespace

PLUMA_INHERIT_PROVIDER(TrackingStub, TrackingPlugin);
PLUMA_PROVIDER_SOURCE(TrackingPlugin, 1, 1);

PLUMA_CONNECTOR
bool connectToPlumaHost(pluma::Host& host) {
    host.add(new TrackingStubProvider());
    return true;
}

}  // namespace plane::algorithm
```

- [x] Step 3: 写 `plugins/guidance_stub/guidance_stub.cpp`

```cpp
#include <memory>
#include <string>

#include <Pluma/Connector.hpp>
#include <Pluma/Pluma.hpp>
#include <spdlog/spdlog.h>

#include "functional/guidance_api.h"

namespace plane::algorithm {
namespace {

class GuidanceStub final : public GuidancePlugin {
   public:
    bool init(const GuidanceConfig& config) override {
        config_ = config;
        spdlog::info("GuidanceStub init: desired_distance_m={}", config_.desired_distance_m);
        return true;
    }

    bool guide(const DetectTarget&, const VehicleState&, NedVelocity& out, std::string&) override {
        out = NedVelocity{};
        return true;
    }

   private:
    GuidanceConfig config_;
};

}  // namespace

PLUMA_INHERIT_PROVIDER(GuidanceStub, GuidancePlugin);
PLUMA_PROVIDER_SOURCE(GuidancePlugin, 1, 1);

PLUMA_CONNECTOR
bool connectToPlumaHost(pluma::Host& host) {
    host.add(new GuidanceStubProvider());
    return true;
}

}  // namespace plane::algorithm
```

- [x] Step 4: `CMakeLists.txt` 末尾追加三个插件目标

```cmake
add_library(detection_stub SHARED plugins/detection_stub/detection_stub.cpp)
target_link_libraries(detection_stub PRIVATE plane::pluma plane::common_include opencv_core spdlog::spdlog)
set_target_properties(detection_stub PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/plugins")

add_library(tracking_stub SHARED plugins/tracking_stub/tracking_stub.cpp)
target_link_libraries(tracking_stub PRIVATE plane::pluma plane::common_include opencv_core spdlog::spdlog)
set_target_properties(tracking_stub PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/plugins")

add_library(guidance_stub SHARED plugins/guidance_stub/guidance_stub.cpp)
target_link_libraries(guidance_stub PRIVATE plane::pluma plane::common_include spdlog::spdlog)
set_target_properties(guidance_stub PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/plugins")
```

- [x] Step 5: 构建与检查产物

```bash
cmake --build build -j
ls build/plugins/
```

Expected: 构建通过；`build/plugins/` 下存在 `libdetection_stub.so`、`libtracking_stub.so`、`libguidance_stub.so`。

- [x] Step 6: 格式与提交

```bash
clang-format --dry-run --Werror $(git ls-files plugins)
git add plugins CMakeLists.txt
git commit -m "feat(m6): 增加检测/跟踪/导引示例插件"
```

---

## Task 5：Perception（图像输入与算法调度）

**Files**

- Create: `src/perception/perception_api.h`
- Create: `src/perception/perception_events.h`
- Create: `src/perception/perception.h`
- Create: `src/perception/perception.cpp`
- Modify: `CMakeLists.txt`

**Interfaces**

- Consumes: `PluginManagerApi`（Task 3）、`StateStoreApi`（M2）、`AppConfig`（Task 1）
- Produces: `plane::perception::Perception`（`IModule` + `PerceptionApi`）；4 个结果事件

- [x] Step 1: 写 `src/perception/perception_api.h`

```cpp
#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "message.h"

namespace plane::perception {

struct DetectionResult {
    std::vector<plane::model::DetectTarget> targets;
};

struct TrackingResult {
    plane::model::DetectTarget target;
    bool lost = false;
};

struct GuidanceResult {
    plane::model::NedVelocity velocity;
    bool valid = false;
};

struct PerceptionError {
    std::string detail;
};

/// 感知业务级控制接口（ARCHITECTURE §15.3）。
class PerceptionApi {
   public:
    virtual ~PerceptionApi() = default;

    virtual bool start_detection(std::string& error) = 0;
    virtual void stop_detection() = 0;
    virtual bool select_target(const plane::model::DetectTarget& target, std::string& error) = 0;
    virtual void stop_tracking() = 0;
    virtual bool start_guidance(std::string& error) = 0;
    virtual void stop_guidance() = 0;
    virtual bool latest_frame(cv::Mat& out) const = 0;
};

}  // namespace plane::perception
```

- [x] Step 2: 写 `src/perception/perception_events.h`

```cpp
#pragma once

#include <string_view>

namespace plane::perception {
namespace events {

inline constexpr std::string_view kDetection = "perception.detection";
inline constexpr std::string_view kTracking = "perception.tracking";
inline constexpr std::string_view kGuidance = "perception.guidance";
inline constexpr std::string_view kError = "perception.error";

}  // namespace events
}  // namespace plane::perception
```

- [x] Step 3: 写 `src/perception/perception.h`

```cpp
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core.hpp>

#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"
#include "functional/detection_api.h"
#include "functional/guidance_api.h"
#include "functional/tracking_api.h"
#include "perception/perception_api.h"
#include "perception/plugin_manager_api.h"
#include "state/state_store_api.h"

namespace cv {
class VideoCapture;
}

namespace plane::perception {

/// MIPI/RTSP 图像输入、算法插件调度与结果发布（ARCHITECTURE §15）。
class Perception final : public plane::core::IModule, public PerceptionApi {
   public:
    Perception(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
               PluginManagerApi& plugin_manager, plane::state::StateStoreApi& state_store);
    ~Perception() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    bool start_detection(std::string& error) override;
    void stop_detection() override;
    bool select_target(const plane::model::DetectTarget& target, std::string& error) override;
    void stop_tracking() override;
    bool start_guidance(std::string& error) override;
    void stop_guidance() override;
    bool latest_frame(cv::Mat& out) const override;

   private:
    void capture_loop();
    void process_loop();
    bool open_source();
    bool ensure_detection(std::string& error);
    bool ensure_tracking(std::string& error);
    bool ensure_guidance(std::string& error);
    void run_selftest(const cv::Mat& frame);
    void report_error(const std::string& detail) const;
    bool latest_frame_locked(cv::Mat& out) const;

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    PluginManagerApi& plugin_manager_;
    plane::state::StateStoreApi& state_store_;

    std::unique_ptr<cv::VideoCapture> capture_;
    mutable std::mutex frame_mutex_;
    cv::Mat latest_frame_;

    std::mutex plugin_mutex_;
    std::shared_ptr<plane::algorithm::DetectionPlugin> detection_;
    std::shared_ptr<plane::algorithm::TrackingPlugin> tracking_;
    std::shared_ptr<plane::algorithm::GuidancePlugin> guidance_;

    std::atomic<bool> detection_enabled_{false};
    std::atomic<bool> tracking_enabled_{false};
    std::atomic<bool> guidance_enabled_{false};
    std::mutex target_mutex_;
    plane::model::DetectTarget tracking_target_;

    std::thread capture_thread_;
    std::thread process_thread_;
    std::atomic<bool> running_{false};
    bool selftest_done_ = false;
};

}  // namespace plane::perception
```

- [x] Step 4: 写 `src/perception/perception.cpp`

```cpp
#include "perception/perception.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/videoio.hpp>
#include <spdlog/spdlog.h>

#include "perception/perception_events.h"

namespace plane::perception {
namespace {

bool is_numeric(const std::string& text) {
    return !text.empty() && std::all_of(text.begin(), text.end(),
                                        [](unsigned char c) { return std::isdigit(c) != 0; });
}

}  // namespace

Perception::Perception(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                       PluginManagerApi& plugin_manager, plane::state::StateStoreApi& state_store)
    : event_bus_(event_bus),
      config_(config),
      plugin_manager_(plugin_manager),
      state_store_(state_store) {}

Perception::~Perception() {
    stop();
}

std::string Perception::name() const {
    return "Perception";
}

bool Perception::init() {
    return true;
}

bool Perception::start() {
    running_ = true;
    capture_thread_ = std::thread([this]() { capture_loop(); });
    process_thread_ = std::thread([this]() { process_loop(); });
    return true;
}

void Perception::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    if (process_thread_.joinable()) {
        process_thread_.join();
    }
    if (capture_) {
        capture_->release();
        capture_.reset();
    }
    {
        std::lock_guard<std::mutex> lock(plugin_mutex_);
        detection_.reset();
        tracking_.reset();
        guidance_.reset();
    }
}

void Perception::release() {}

bool Perception::start_detection(std::string& error) {
    if (!ensure_detection(error)) {
        report_error("启动检测失败: " + error);
        return false;
    }
    detection_enabled_ = true;
    return true;
}

void Perception::stop_detection() {
    detection_enabled_ = false;
}

bool Perception::select_target(const plane::model::DetectTarget& target, std::string& error) {
    cv::Mat frame;
    if (!latest_frame_locked(frame)) {
        error = "当前无图像帧";
        return false;
    }
    if (!ensure_tracking(error)) {
        report_error("启动跟踪失败: " + error);
        return false;
    }
    if (!tracking_->start(frame, target, error)) {
        report_error("建立跟踪失败: " + error);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        tracking_target_ = target;
    }
    tracking_enabled_ = true;
    return true;
}

void Perception::stop_tracking() {
    tracking_enabled_ = false;
}

bool Perception::start_guidance(std::string& error) {
    if (!ensure_guidance(error)) {
        report_error("启动导引失败: " + error);
        return false;
    }
    guidance_enabled_ = true;
    return true;
}

void Perception::stop_guidance() {
    guidance_enabled_ = false;
}

bool Perception::latest_frame(cv::Mat& out) const {
    return latest_frame_locked(out);
}

bool Perception::latest_frame_locked(cv::Mat& out) const {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (latest_frame_.empty()) {
        return false;
    }
    out = latest_frame_;
    return true;
}

bool Perception::open_source() {
    capture_ = std::make_unique<cv::VideoCapture>();
    const std::string& source = config_.image_source;
    bool opened = false;
    if (is_numeric(source)) {
        opened = capture_->open(std::stoi(source));
    } else {
        opened = capture_->open(source, cv::CAP_ANY);
    }
    return opened && capture_->isOpened();
}

void Perception::capture_loop() {
    while (running_) {
        if (config_.image_source.empty()) {
            spdlog::warn("图像源未配置 (image_source 为空)");
            std::this_thread::sleep_for(
                std::chrono::milliseconds(config_.image_reconnect_interval_ms));
            continue;
        }
        if (!open_source()) {
            report_error("图像源打开失败: " + config_.image_source);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(config_.image_reconnect_interval_ms));
            continue;
        }
        spdlog::info("图像源已打开: {}", config_.image_source);
        bool first_frame = true;
        while (running_) {
            cv::Mat frame;
            if (!capture_->read(frame) || frame.empty()) {
                spdlog::warn("图像读取失败，尝试重连: {}", config_.image_source);
                break;
            }
            if (first_frame) {
                first_frame = false;
                spdlog::info("首帧: {}x{}", frame.cols, frame.rows);
            }
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = frame;
        }
        capture_->release();
    }
}

void Perception::process_loop() {
    while (running_) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(config_.perception_process_interval_ms));
        cv::Mat frame;
        const bool has_frame = latest_frame_locked(frame);
        if (config_.perception_selftest_on_start && !selftest_done_) {
            selftest_done_ = true;
            const cv::Mat selftest_frame =
                has_frame ? frame : cv::Mat::zeros(480, 640, CV_8UC3);
            run_selftest(selftest_frame);
        }
        if (!has_frame) {
            continue;
        }
        if (detection_enabled_) {
            std::string error;
            if (ensure_detection(error)) {
                std::vector<plane::model::DetectTarget> targets;
                if (detection_->detect(frame, targets, error)) {
                    DetectionResult result;
                    result.targets = std::move(targets);
                    event_bus_.publish(events::kDetection, result);
                } else {
                    report_error("检测失败: " + error);
                }
            }
        }
        if (tracking_enabled_) {
            std::string error;
            if (ensure_tracking(error)) {
                plane::model::DetectTarget target;
                if (tracking_->track(frame, target, error)) {
                    TrackingResult result;
                    result.target = target;
                    result.lost = false;
                    {
                        std::lock_guard<std::mutex> lock(target_mutex_);
                        tracking_target_ = target;
                    }
                    event_bus_.publish(events::kTracking, result);
                } else {
                    TrackingResult result;
                    {
                        std::lock_guard<std::mutex> lock(target_mutex_);
                        result.target = tracking_target_;
                    }
                    result.lost = true;
                    event_bus_.publish(events::kTracking, result);
                }
            }
        }
        if (guidance_enabled_ && tracking_enabled_) {
            std::string error;
            if (ensure_guidance(error)) {
                plane::model::DetectTarget target;
                {
                    std::lock_guard<std::mutex> lock(target_mutex_);
                    target = tracking_target_;
                }
                plane::model::NedVelocity velocity;
                if (guidance_->guide(target, state_store_.own_state(), velocity, error)) {
                    GuidanceResult result;
                    result.velocity = velocity;
                    result.valid = true;
                    event_bus_.publish(events::kGuidance, result);
                } else {
                    GuidanceResult result;
                    result.valid = false;
                    event_bus_.publish(events::kGuidance, result);
                }
            }
        }
    }
}

void Perception::run_selftest(const cv::Mat& frame) {
    spdlog::info("感知自检开始 (frame {}x{})", frame.cols, frame.rows);
    std::string error;
    if (!ensure_detection(error)) {
        report_error("自检检测插件不可用: " + error);
        return;
    }
    std::vector<plane::model::DetectTarget> targets;
    if (detection_->detect(frame, targets, error)) {
        spdlog::info("自检 detect 成功: targets={}", targets.size());
    } else {
        report_error("自检 detect 失败: " + error);
        return;
    }
    if (!ensure_tracking(error)) {
        report_error("自检跟踪插件不可用: " + error);
        return;
    }
    plane::model::DetectTarget init;
    if (targets.empty()) {
        init.bbox.x = frame.cols / 2 - 50;
        init.bbox.y = frame.rows / 2 - 50;
        init.bbox.width = 100;
        init.bbox.height = 100;
        init.confidence = 0.9F;
        init.type = "stub";
    } else {
        init = targets.front();
    }
    if (!tracking_->start(frame, init, error)) {
        report_error("自检 track start 失败: " + error);
        return;
    }
    plane::model::DetectTarget tracked;
    if (tracking_->track(frame, tracked, error)) {
        spdlog::info("自检 track 成功: type={}", tracked.type);
    } else {
        report_error("自检 track 失败: " + error);
        return;
    }
    if (!ensure_guidance(error)) {
        report_error("自检导引插件不可用: " + error);
        return;
    }
    plane::model::NedVelocity velocity;
    if (guidance_->guide(tracked, state_store_.own_state(), velocity, error)) {
        spdlog::info("自检 guide 成功: vn={} ve={} vd={}", velocity.north_m_s, velocity.east_m_s,
                     velocity.down_m_s);
    } else {
        report_error("自检 guide 失败: " + error);
    }
}

bool Perception::ensure_detection(std::string& error) {
    std::lock_guard<std::mutex> lock(plugin_mutex_);
    if (detection_) {
        return true;
    }
    detection_ = plugin_manager_.create_detection();
    if (!detection_) {
        error = "无可用检测插件";
        return false;
    }
    plane::algorithm::DetectConfig detect_config;
    if (!detection_->init(detect_config)) {
        detection_.reset();
        error = "检测插件初始化失败";
        return false;
    }
    spdlog::info("检测插件已实例化");
    return true;
}

bool Perception::ensure_tracking(std::string& error) {
    std::lock_guard<std::mutex> lock(plugin_mutex_);
    if (tracking_) {
        return true;
    }
    tracking_ = plugin_manager_.create_tracking();
    if (!tracking_) {
        error = "无可用跟踪插件";
        return false;
    }
    plane::algorithm::TrackConfig track_config;
    if (!tracking_->init(track_config)) {
        tracking_.reset();
        error = "跟踪插件初始化失败";
        return false;
    }
    spdlog::info("跟踪插件已实例化");
    return true;
}

bool Perception::ensure_guidance(std::string& error) {
    std::lock_guard<std::mutex> lock(plugin_mutex_);
    if (guidance_) {
        return true;
    }
    guidance_ = plugin_manager_.create_guidance();
    if (!guidance_) {
        error = "无可用导引插件";
        return false;
    }
    plane::algorithm::GuidanceConfig guidance_config;
    if (!guidance_->init(guidance_config)) {
        guidance_.reset();
        error = "导引插件初始化失败";
        return false;
    }
    spdlog::info("导引插件已实例化");
    return true;
}

void Perception::report_error(const std::string& detail) const {
    spdlog::error("Perception: {}", detail);
    PerceptionError event;
    event.detail = detail;
    event_bus_.publish(events::kError, event);
}

}  // namespace plane::perception
```

- [x] Step 5: `CMakeLists.txt` 修改

`add_executable(onboard ...)` 文件列表追加：

```cmake
  src/perception/perception.cpp
```

`target_link_libraries(onboard ...)` 追加 `opencv_videoio`（系统 OpenCV 提供，`plane_deps` 仅含 `opencv_core`）。

- [x] Step 6: 构建与格式

```bash
cmake --build build -j
clang-format --dry-run --Werror $(git ls-files src/perception)
```

Expected: 构建通过且无 warning；格式通过（差异先 `-i` 再构建）。

- [x] Step 7: 提交

```bash
git add src/perception/perception.h src/perception/perception.cpp src/perception/perception_api.h src/perception/perception_events.h CMakeLists.txt
git commit -m "feat(m6): 增加 Perception 图像输入与算法调度"
```

---

## Task 6：main 接线

**Files**

- Modify: `src/main.cpp`

- [x] Step 1: include 追加

```cpp
#include "camera/camera_control_module.h"
#include "perception/perception.h"
#include "perception/plugin_management.h"
```

- [x] Step 2: 模块注册块改为（在 state_store/flight_control 创建之后追加并注册）

```cpp
    auto plugin_management = std::make_unique<plane::perception::PluginManagement>(config);
    auto camera_control = std::make_unique<plane::camera::CameraControlModule>(event_bus);
    auto perception = std::make_unique<plane::perception::Perception>(
        event_bus, config, *plugin_management, *state_store);
```

注册顺序：

```cpp
    runtime.add_module(std::move(state_store));
    runtime.add_module(std::move(flight_control));
    runtime.add_module(std::move(plugin_management));
    runtime.add_module(std::move(camera_control));
    runtime.add_module(std::move(perception));
    runtime.add_module(std::move(task_engine));
    runtime.add_module(std::move(task_scheduler));
    runtime.add_module(std::move(communication));
    for (auto& module : plane::core::make_placeholder_modules(
             {"Communication", "UAV State Management", "Flight Control", "Task Scheduler",
              "Task Execution Engine", "Plugin Management", "Camera Control", "Perception"})) {
        runtime.add_module(std::move(module));
    }
```

- [x] Step 3: 构建、运行与格式

```bash
cmake --build build -j
timeout --preserve-status 3 ./build/onboard; echo "exit=$?"
clang-format --dry-run --Werror src/main.cpp
```

Expected: 构建通过；日志含 `插件目录加载`、`相机链路已建立 (Noop)`、`图像源未配置`，exit=0。

- [x] Step 4: 提交

```bash
git add src/main.cpp
git commit -m "feat(m6): 接线相机、插件管理与感知模块"
```

---

## Task 7：M6 部分验收

**Files**

- Modify: `docs/plans/m6-perception-plugins-tasks.md`
- Modify: `docs/DEV_PLAN.md`

- [x] Step 1: 干净构建与产物检查

```bash
rm -rf build logs
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ls build/plugins/
```

Expected: 构建无 warning；三个 stub `.so` 存在。

- [x] Step 2: 默认运行（加载与相机占位）

```bash
timeout --preserve-status 3 ./build/onboard 2>&1 | grep -E "插件目录加载|插件 provider|相机链路"
```

Expected：`插件目录加载: build/plugins (成功加载 3 个库)`、`插件 provider: detection=1, tracking=1, guidance=1`、`相机链路已建立 (Noop)`。

- [x] Step 3: 启动自检（三接口）

```bash
printf 'plugins_dir = build/plugins\nperception_selftest_on_start = true\nperception_process_interval_ms = 100\n' > /tmp/opencode/m6_selftest.conf
timeout --preserve-status 3 ./build/onboard --config /tmp/opencode/m6_selftest.conf 2>&1 | grep -E "自检|插件已实例化"
```

Expected：`检测插件已实例化`、`自检 detect 成功: targets=0`、`自检 track 成功`、`自检 guide 成功: vn=0 ve=0 vd=0`。
（自检使用合成灰帧，无图像源也可运行。）

- [x] Step 4: 图像源与自检（有源时）

```bash
printf 'plugins_dir = build/plugins\nimage_source = <rtsp或视频路径>\nperception_selftest_on_start = true\n' > /tmp/opencode/m6_source.conf
timeout --preserve-status 5 ./build/onboard --config /tmp/opencode/m6_source.conf 2>&1 | grep -E "图像源已打开|首帧|自检"
```

Expected：`图像源已打开`、`首帧: WxH`、三接口自检成功。

- [x] Step 5: 规范与回归

```bash
clang-format --dry-run --Werror $(git ls-files src tools plugins)
grep -rn "std::cout\|printf(" src/ tools/ plugins/ || echo "no cout/printf"
```

- [x] Step 6: 文档与收尾提交

- 勾选本文档并记录验收证据与偏差；
- 更新 `docs/DEV_PLAN.md` §11 索引：M6 → 已完成（部分验收，RTSP 实流待验）。

```bash
git add docs
git commit -m "docs(m6): M6 部分验收与文档更新"
```

---

## 附：待用户补充验证（M6 交付）

1. 提供 RTSP 源（或 MIPI 设备）后：`image_source` 取图、首帧尺寸、持续帧率；
2. 用 `PerceptionApi`（start_detection/select_target/start_guidance）触发周期检测/跟踪/导引，观察 `perception.*` 事件与日志；
3. 替换真实算法插件后：插件加载、实例化与结果输出。

---

## 执行记录与偏差

1. **Pluma Provider 类型名**：`PLUMA_PROVIDER_TYPE` 是 Provider 头中的 private 静态成员，`create_first` 改为显式传入类型名字面量（已同步回计划 Task 3 代码）。
2. **OpenCV VideoIO**：`cv::VideoCapture` 需要 `opencv_videoio`，在 onboard 链接库中追加（`plane_deps` 仅含 `opencv_core`；已同步回计划 Task 5）。
3. **相机停止幂等**：`CameraControlModule::stop()` 被 Runtime 与析构各调一次导致重复断开日志，增加 `stopped_` 幂等标志（提交 `本文件所在提交`）。
4. **验收证据（2026-09-18，无图像源）**
   - 构建无 warning；`build/plugins/` 生成三个 stub `.so`；
   - 默认运行：`插件目录加载: build/plugins (成功加载 3 个库)`、`插件 provider: detection=1, tracking=1, guidance=1`、`相机链路已建立 (Noop)`、`图像源未配置 (image_source 为空)` 周期 warn；
   - 自检（`perception_selftest_on_start=true`，合成 640x480 灰帧）：`检测插件已实例化`、`自检 detect 成功: targets=0`、`跟踪插件已实例化`、`自检 track 成功: type=stub`、`导引插件已实例化`、`自检 guide 成功: vn=0 ve=0 vd=0`；
   - 回归：M1–M5 行为正常；`clang-format` 通过；无 `std::cout/printf`。
5. **待用户验证**：RTSP/MIPI 实流取图、`PerceptionApi` 周期检测/跟踪/导引、真实算法插件替换。
