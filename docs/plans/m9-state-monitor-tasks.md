# M9 任务级计划（安全告警与系统参数动态下发）

> 设计依据：`m9-state-monitor-design.md`（逐项确认；仅低电压告警）。
> 执行位置：工程根 `onboard/`。
> 验证原则：每个 Task 结束后 x64 Release 构建通过；不写测试代码（SPEC §11）。
> 无 SITL：Task 8 为部分验收，低电压告警链路待 SITL。

---

## Task 1：告警配置项

**Files**

- Modify: `src/core/config.h`
- Modify: `src/core/config.cpp`
- Modify: `config/onboard.conf`

- [ ] Step 1: `src/core/config.h` 的 `AppConfig` 中，在 `video_overlay_enabled` 之后追加

```cpp
    double alert_voltage_threshold_v = 21.0;
```

- [ ] Step 2: `src/core/config.cpp` 解析分支中，在 `video_overlay_enabled` 分支之后追加

```cpp
        } else if (key == "alert_voltage_threshold_v") {
            out.alert_voltage_threshold_v = std::stod(value);
```

- [ ] Step 3: `config/onboard.conf` 末尾追加

```text
# 安全告警（低电压阈值，按实机电池配置）
alert_voltage_threshold_v = 21.0
```

- [ ] Step 4: 构建、运行与提交

```bash
cmake --build build -j
timeout --preserve-status 2 ./build/onboard; echo "exit=$?"
clang-format --dry-run --Werror src/core/config.h src/core/config.cpp
git add src/core/config.h src/core/config.cpp config/onboard.conf
git commit -m "feat(m9): 增加低电压告警阈值配置"
```

Expected: 构建通过；运行无「未知配置项」，exit=0。

---

## Task 2：SystemParamStore

**Files**

- Create: `src/core/system_param_api.h`
- Create: `src/core/system_param_events.h`
- Create: `src/core/system_param_store.h`
- Create: `src/core/system_param_store.cpp`
- Modify: `CMakeLists.txt`

- [ ] Step 1: 写 `src/core/system_param_api.h`

```cpp
#pragma once

#include "message.h"

namespace plane::core {

/// 运行时系统参数查询接口（ARCHITECTURE §7）。
class SystemParamApi {
   public:
    virtual ~SystemParamApi() = default;

    virtual plane::model::SystemParam current() const = 0;
};

}  // namespace plane::core
```

- [ ] Step 2: 写 `src/core/system_param_events.h`

```cpp
#pragma once

#include <string_view>

namespace plane::core {
namespace events {

inline constexpr std::string_view kSystemParamChanged = "system_param.changed";

}  // namespace events
}  // namespace plane::core
```

- [ ] Step 3: 写 `src/core/system_param_store.h`

```cpp
#pragma once

#include <mutex>
#include <string>

#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"
#include "core/system_param_api.h"

namespace plane::core {

/// 系统参数运行时存储：启动默认值 + GCS 动态更新（ARCHITECTURE §7）。
class SystemParamStore final : public IModule, public SystemParamApi {
   public:
    SystemParamStore(EventBus& event_bus, const AppConfig& config);
    ~SystemParamStore() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    plane::model::SystemParam current() const override;

   private:
    void on_system_param(const plane::model::SystemParam& param);

    EventBus& event_bus_;
    mutable std::mutex mutex_;
    plane::model::SystemParam param_;
};

}  // namespace plane::core
```

- [ ] Step 4: 写 `src/core/system_param_store.cpp`

```cpp
#include "core/system_param_store.h"

#include <spdlog/spdlog.h>

#include "communication/communication_events.h"
#include "core/system_param_events.h"

namespace plane::core {

SystemParamStore::SystemParamStore(EventBus& event_bus, const AppConfig& config)
    : event_bus_(event_bus), param_(config.system_param) {}

SystemParamStore::~SystemParamStore() {
    stop();
}

std::string SystemParamStore::name() const {
    return "System Param";
}

bool SystemParamStore::init() {
    return true;
}

bool SystemParamStore::start() {
    event_bus_.subscribe<plane::model::SystemParam>(
        plane::communication::events::kSystemParamReceived,
        [this](const plane::model::SystemParam& param) { on_system_param(param); });
    spdlog::info("系统参数就绪: safe_height_m={}", current().safeHeight_m);
    return true;
}

void SystemParamStore::stop() {}

void SystemParamStore::release() {}

plane::model::SystemParam SystemParamStore::current() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return param_;
}

void SystemParamStore::on_system_param(const plane::model::SystemParam& param) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        param_ = param;
    }
    spdlog::info("系统参数更新: safe_height_m={}", param.safeHeight_m);
    event_bus_.publish(events::kSystemParamChanged, param);
}

}  // namespace plane::core
```

- [ ] Step 5: `CMakeLists.txt` 的 `add_executable(onboard ...)` 文件列表追加

```cmake
  src/core/system_param_store.cpp
```

- [ ] Step 6: 构建与格式

```bash
cmake --build build -j
clang-format --dry-run --Werror src/core/system_param_api.h src/core/system_param_events.h src/core/system_param_store.h src/core/system_param_store.cpp
```

Expected: 构建通过且无 warning；格式通过（差异先 `-i` 再构建）。

- [ ] Step 7: 提交

```bash
git add src/core/system_param_api.h src/core/system_param_events.h src/core/system_param_store.h src/core/system_param_store.cpp CMakeLists.txt
git commit -m "feat(m9): 增加 SystemParamStore 运行时参数"
```

---

## Task 3：TaskScheduler 改读动态安全高度

**Files**

- Modify: `src/task/task_scheduler.h`
- Modify: `src/task/task_scheduler.cpp`

- [ ] Step 1: `task_scheduler.h` 修改

1) include 追加：

```cpp
#include "core/system_param_api.h"
```

2) 构造函数声明改为：

```cpp
    TaskScheduler(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                  plane::flight_control::FlightControlApi& flight_control,
                  plane::core::SystemParamApi& system_param,
                  TaskExecutionEngineApi* engine = nullptr);
```

3) 成员区追加（放在 `flight_control_` 之后、`communication_` 之前，保证与构造初始化顺序一致）：

```cpp
    plane::core::SystemParamApi& system_param_;
```

- [ ] Step 2: `task_scheduler.cpp` 修改

1) 构造函数定义改为：

```cpp
TaskScheduler::TaskScheduler(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                             plane::flight_control::FlightControlApi& flight_control,
                             plane::core::SystemParamApi& system_param,
                             TaskExecutionEngineApi* engine)
    : event_bus_(event_bus),
      config_(config),
      flight_control_(flight_control),
      system_param_(system_param),
      engine_(engine) {}
```

2) `dispatch_command()` 的 Takeoff 分支中：

将：

```cpp
            if (height <= 0.0F) {
                height = static_cast<float>(config_.system_param.safeHeight_m);
                spdlog::info("Takeoff 未指定高度，使用安全高度: {} m", height);
            }
```

改为：

```cpp
            if (height <= 0.0F) {
                height = static_cast<float>(system_param_.current().safeHeight_m);
                spdlog::info("Takeoff 未指定高度，使用安全高度: {} m", height);
            }
```

- [ ] Step 3: 语法检查（main 暂断属预期，Task 7 修复）

```bash
cmake --build build -j 2>&1 | grep -E "task_scheduler|main" | head
```

Expected: task_scheduler 无错误；main.cpp 因构造参数不匹配报错（预期）。

- [ ] Step 4: 格式与提交

```bash
clang-format --dry-run --Werror src/task/task_scheduler.h src/task/task_scheduler.cpp
git add src/task/task_scheduler.h src/task/task_scheduler.cpp
git commit -m "feat(m9): TaskScheduler 使用动态安全高度"
```

---

## Task 4：TakeoffNode 与 TaskExecutionEngine 改造

**Files**

- Modify: `src/task/nodes/takeoff_node.h`
- Modify: `src/task/nodes/takeoff_node.cpp`
- Modify: `src/task/task_execution_engine.h`
- Modify: `src/task/task_execution_engine.cpp`

- [ ] Step 1: `takeoff_node.h` 改为

```cpp
#pragma once

#include "core/system_param_api.h"
#include "task/nodes/flight_action_node.h"

namespace plane::task {

class TakeoffNode final : public FlightActionNode {
   public:
    TakeoffNode(const std::string& name, const BT::NodeConfig& config,
                plane::flight_control::FlightControlApi* flight_control,
                plane::state::StateStoreApi* state_store,
                const plane::core::AppConfig* app_config,
                const plane::core::SystemParamApi* system_param)
        : FlightActionNode(name, config, flight_control, state_store, app_config),
          system_param_(system_param) {}

    static BT::PortsList providedPorts() { return {}; }

   protected:
    bool dispatch(std::string& error) override;
    BT::NodeStatus check() override;

   private:
    const plane::core::SystemParamApi* system_param_;
};

}  // namespace plane::task
```

- [ ] Step 2: `takeoff_node.cpp` 改为

```cpp
#include "task/nodes/takeoff_node.h"

namespace plane::task {

bool TakeoffNode::dispatch(std::string& error) {
    const auto height = static_cast<float>(system_param_->current().safeHeight_m);
    return flight_control_->takeoff(height, error);
}

BT::NodeStatus TakeoffNode::check() {
    const auto status = flight_control_->flight_status();
    if (!status.in_air) {
        return BT::NodeStatus::RUNNING;
    }
    const auto target = static_cast<float>(system_param_->current().safeHeight_m);
    const auto tolerance = static_cast<float>(app_config_->bt_takeoff_tolerance_m);
    return state_store_->own_state().geoPos.height >= target - tolerance ? BT::NodeStatus::SUCCESS
                                                                        : BT::NodeStatus::RUNNING;
}

}  // namespace plane::task
```

- [ ] Step 3: `task_execution_engine.h` 修改

1) include 追加：

```cpp
#include "core/system_param_api.h"
```

2) 构造函数声明改为：

```cpp
    TaskExecutionEngine(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                        plane::flight_control::FlightControlApi& flight_control,
                        plane::state::StateStoreApi& state_store,
                        plane::core::SystemParamApi& system_param);
```

3) 成员区追加（放在 `state_store_` 之后、`mutex_` 之前，保证与构造初始化顺序一致）：

```cpp
    plane::core::SystemParamApi& system_param_;
```

- [ ] Step 4: `task_execution_engine.cpp` 修改

1) 构造函数定义改为：

```cpp
TaskExecutionEngine::TaskExecutionEngine(plane::core::EventBus& event_bus,
                                         const plane::core::AppConfig& config,
                                         plane::flight_control::FlightControlApi& flight_control,
                                         plane::state::StateStoreApi& state_store,
                                         plane::core::SystemParamApi& system_param)
    : event_bus_(event_bus),
      config_(config),
      flight_control_(flight_control),
      state_store_(state_store),
      system_param_(system_param) {}
```

2) `build_basic_flight()` 中 Takeoff 注册改为：

```cpp
    factory.registerNodeType<TakeoffNode>("Takeoff", &flight_control_, &state_store_, &config_,
                                          &system_param_);
```

- [ ] Step 5: 语法检查与格式

```bash
cmake --build build -j 2>&1 | grep -E "takeoff_node|task_execution_engine|main" | head
clang-format --dry-run --Werror src/task/nodes/takeoff_node.h src/task/nodes/takeoff_node.cpp src/task/task_execution_engine.h src/task/task_execution_engine.cpp
```

Expected: 节点/引擎无编译错误（main 仍报构造参数错，Task 7 修复）；格式通过。

- [ ] Step 6: 提交

```bash
git add src/task/nodes/takeoff_node.h src/task/nodes/takeoff_node.cpp src/task/task_execution_engine.h src/task/task_execution_engine.cpp
git commit -m "feat(m9): TakeoffNode 使用动态安全高度"
```

---

## Task 5：StateMonitor

**Files**

- Create: `src/state/state_monitor.h`
- Create: `src/state/state_monitor.cpp`
- Modify: `CMakeLists.txt`

- [ ] Step 1: 写 `src/state/state_monitor.h`

```cpp
#pragma once

#include <mutex>
#include <string>

#include "communication/communication_api.h"
#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"

namespace plane::state {

/// 安全监控：第一版仅低电压告警，边沿触发，仅飞控连接时评估（SPEC §10）。
class StateMonitor final : public plane::core::IModule {
   public:
    StateMonitor(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                 plane::communication::CommunicationApi& communication);
    ~StateMonitor() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

   private:
    void on_own_state(const plane::model::VehicleState& state);
    void send_alert(const std::string& detail);

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    plane::communication::CommunicationApi& communication_;
    std::mutex mutex_;
    bool low_voltage_latched_ = false;
};

}  // namespace plane::state
```

- [ ] Step 2: 写 `src/state/state_monitor.cpp`

```cpp
#include "state/state_monitor.h"

#include <spdlog/spdlog.h>

#include "state/state_events.h"

namespace plane::state {

StateMonitor::StateMonitor(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                           plane::communication::CommunicationApi& communication)
    : event_bus_(event_bus), config_(config), communication_(communication) {}

StateMonitor::~StateMonitor() {
    stop();
}

std::string StateMonitor::name() const {
    return "StateMonitor";
}

bool StateMonitor::init() {
    return true;
}

bool StateMonitor::start() {
    event_bus_.subscribe<plane::model::VehicleState>(
        events::kOwnStateChanged,
        [this](const plane::model::VehicleState& state) { on_own_state(state); });
    return true;
}

void StateMonitor::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    low_voltage_latched_ = false;
}

void StateMonitor::release() {}

void StateMonitor::on_own_state(const plane::model::VehicleState& state) {
    if (state.fcLinkState != "connected") {
        std::lock_guard<std::mutex> lock(mutex_);
        low_voltage_latched_ = false;
        return;
    }
    const double threshold = config_.alert_voltage_threshold_v;
    const double voltage = static_cast<double>(state.voltage_v);
    bool trigger = false;
    bool recovered = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (voltage < threshold && !low_voltage_latched_) {
            low_voltage_latched_ = true;
            trigger = true;
        } else if (voltage >= threshold && low_voltage_latched_) {
            low_voltage_latched_ = false;
            recovered = true;
        }
    }
    if (trigger) {
        const std::string detail =
            "voltage=" + std::to_string(voltage) + "V < threshold=" + std::to_string(threshold) + "V";
        spdlog::warn("低电压告警: {}", detail);
        send_alert(detail);
    } else if (recovered) {
        spdlog::info("电压恢复正常: {} V", voltage);
    }
}

void StateMonitor::send_alert(const std::string& detail) {
    plane::model::Alert alert;
    alert.type = "low_voltage";
    alert.detail = detail;
    if (!communication_.sendToStation(plane::communication::OutgoingMessage{alert})) {
        spdlog::error("低电压告警发送失败: {}", detail);
    }
}

}  // namespace plane::state
```

- [ ] Step 3: `CMakeLists.txt` 的 `add_executable(onboard ...)` 文件列表追加

```cmake
  src/state/state_monitor.cpp
```

- [ ] Step 4: 构建与格式

```bash
cmake --build build -j 2>&1 | grep -E "state_monitor|main" | head
clang-format --dry-run --Werror src/state/state_monitor.h src/state/state_monitor.cpp
```

Expected: state_monitor 无错误；main 仍报构造参数错（Task 7 修复）；格式通过。

- [ ] Step 5: 提交

```bash
git add src/state/state_monitor.h src/state/state_monitor.cpp CMakeLists.txt
git commit -m "feat(m9): 增加 StateMonitor 低电压告警"
```

---

## Task 6：gcs_sim 系统参数下发

**Files**

- Modify: `tools/gcs_sim/main.cpp`

- [ ] Step 1: `Options` 增加

```cpp
    std::vector<std::pair<std::string, std::string>> send_params;
```

（若文件未包含 `<utility>`/`<vector>`，补充 include。）

- [ ] Step 2: 解析追加

```cpp
        } else if (arg == "--send-param" && i + 1 < argc) {
            const std::string pair = argv[++i];
            const auto pos = pair.find('=');
            if (pos != std::string::npos) {
                options.send_params.emplace_back(pair.substr(0, pos), pair.substr(pos + 1));
            }
```

- [ ] Step 3: 发送逻辑（放在 GcsTarget 发送之后、友机状态/listen 之前）

```cpp
    for (const auto& [key, value] : options.send_params) {
        nlohmann::json payload = nlohmann::json::object();
        payload[key] = std::stod(value);
        const std::string message_id = options.sender_id + "-param-" + std::to_string(++sequence);
        session.put(zenoh::KeyExpr("sendToPlane/" + options.plane_id),
                    zenoh::Bytes(build_wire(options.sender_id, message_id, "SystemParam", payload.dump())));
    }
    if (!options.send_params.empty()) {
        spdlog::info("已发送 SystemParam: {} 条", options.send_params.size());
    }
```

- [ ] Step 4: 构建、格式与提交

```bash
cmake --build build -j
clang-format --dry-run --Werror tools/gcs_sim/main.cpp
git add tools/gcs_sim/main.cpp
git commit -m "feat(m9): gcs_sim 支持系统参数下发"
```

---

## Task 7：main 接线

**Files**

- Modify: `src/main.cpp`

- [ ] Step 1: include 追加

```cpp
#include "core/system_param_store.h"
#include "state/state_monitor.h"
```

- [ ] Step 2: 创建与接线

在 `flight_control` 之后创建：

```cpp
    auto system_param_store = std::make_unique<plane::core::SystemParamStore>(event_bus, config);
```

`task_engine` 构造改为：

```cpp
    auto task_engine = std::make_unique<plane::task::TaskExecutionEngine>(
        event_bus, config, *flight_control, *state_store, *system_param_store);
```

`task_scheduler` 构造改为：

```cpp
    auto task_scheduler = std::make_unique<plane::task::TaskScheduler>(
        event_bus, config, *flight_control, *system_param_store, task_engine.get());
```

`communication` 之后创建：

```cpp
    auto state_monitor = std::make_unique<plane::state::StateMonitor>(event_bus, config, *communication);
```

注册顺序：

```cpp
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
```

占位 skip 列表增加 `"StateMonitor"`。

- [ ] Step 3: 构建、运行与格式

```bash
cmake --build build -j
timeout --preserve-status 2 ./build/onboard; echo "exit=$?"
clang-format --dry-run --Werror src/main.cpp
```

Expected: 构建通过；日志含 `系统参数就绪: safe_height_m=30`；`StateMonitor` 为真实模块；exit=0。

- [ ] Step 4: 提交

```bash
git add src/main.cpp
git commit -m "feat(m9): 接线 SystemParamStore 与 StateMonitor"
```

---

## Task 8：M9 部分验收

**Files**

- Modify: `docs/plans/m9-state-monitor-tasks.md`
- Modify: `docs/DEV_PLAN.md`

- [ ] Step 1: 干净构建与启动

```bash
rm -rf build logs
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
timeout --preserve-status 2 ./build/onboard 2>&1 | grep -E "系统参数就绪|模块 start: StateMonitor|低电压告警"
```

Expected：`系统参数就绪: safe_height_m=30`；`StateMonitor` 启动；无低电压告警（飞控未连接不评估）。

- [ ] Step 2: 动态参数下发（终端 A/B）

终端 A：`./build/onboard`

终端 B：

```bash
./build/gcs_sim --no-task --no-command --send-param safe_height_m=50 --listen-seconds 1
./build/gcs_sim --no-task --command-type Takeoff --takeoff-height 0 --listen-seconds 2
```

Expected：

- onboard `系统参数更新: safe_height_m=50`；
- 第二条运行后 onboard `Takeoff 未指定高度，使用安全高度: 50 m`，随后派发失败告警（飞控未连接，预期）。

- [ ] Step 3: 规范与回归

```bash
clang-format --dry-run --Werror $(git ls-files src tools plugins)
grep -rn "std::cout\|printf(" src/ tools/ plugins/ || echo "no cout/printf"
```

- [ ] Step 4: 文档与收尾提交

- 勾选本文档并记录验收证据与偏差；
- 更新 `docs/DEV_PLAN.md` §11 索引：M9 → 已完成（部分验收，告警待 SITL）。

```bash
git add docs
git commit -m "docs(m9): M9 部分验收与文档更新"
```

---

## 附：待 SITL 验证（M9 交付）

1. 低电压告警：SITL 电压低于 `alert_voltage_threshold_v` → GCS 收到 `Alert{low_voltage}`，恢复后再次触发有效；系统无自动处置；
2. `basic_flight` 的 TakeoffNode 使用动态安全高度（先 `--send-param safe_height_m=50` 再执行任务）。
