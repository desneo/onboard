# M5 任务级计划（BehaviorTree 任务执行引擎）

> 设计依据：`m5-behaviortree-engine-design.md`（逐项确认；FlightControl 内部用 MAVSDK 枚举、上报转字符串；BT 三项参数配置化）。
> 执行位置：工程根 `onboard/`。
> 验证原则：每个 Task 结束后 x64 Release 构建通过；不写测试代码（SPEC §11）。
> SITL 不可用：Task 6 为部分验收，basic_flight 完整执行与替换/中止待 SITL。

---

## Task 1：行为树配置项

**Files**

- Modify: `src/core/config.h`
- Modify: `src/core/config.cpp`
- Modify: `config/onboard.conf`

- [ ] Step 1: `src/core/config.h` 的 `AppConfig` 中，在 `fc_route_speed_m_s` 之后追加

```cpp
    int bt_tick_interval_ms = 50;
    int bt_node_timeout_ms = 30000;
    double bt_takeoff_tolerance_m = 1.0;
```

- [ ] Step 2: `src/core/config.cpp` 解析分支中，在 `fc_route_speed_m_s` 分支之后追加

```cpp
        } else if (key == "bt_tick_interval_ms") {
            out.bt_tick_interval_ms = std::stoi(value);
        } else if (key == "bt_node_timeout_ms") {
            out.bt_node_timeout_ms = std::stoi(value);
        } else if (key == "bt_takeoff_tolerance_m") {
            out.bt_takeoff_tolerance_m = std::stod(value);
```

- [ ] Step 3: `config/onboard.conf` 末尾追加

```text
# 行为树执行参数
bt_tick_interval_ms = 50
bt_node_timeout_ms = 30000
bt_takeoff_tolerance_m = 1.0
```

- [ ] Step 4: 构建、运行与提交

```bash
cmake --build build -j
timeout --preserve-status 2 ./build/onboard; echo "exit=$?"
clang-format --dry-run --Werror src/core/config.h src/core/config.cpp
git add src/core/config.h src/core/config.cpp config/onboard.conf
git commit -m "feat(m5): 增加行为树执行配置"
```

Expected: 构建通过；运行无「未知配置项」警告，exit=0。

---

## Task 2：FlightControl FlightStatus 扩展

**Files**

- Modify: `src/flight_control/flight_control_api.h`
- Modify: `src/flight_control/flight_control_module.h`
- Modify: `src/flight_control/flight_control_module.cpp`

**Interfaces**

- Produces: `FlightStatus{armed, in_air, mission_current, mission_total, takeoff_altitude_m}` 与 `flight_status()`

- [ ] Step 1: `flight_control_api.h` 在 `FlightControlApi` 之前追加

```cpp
/// 飞控运行状态快照；模块内部使用 MAVSDK 枚举，对外只暴露基础字段。
struct FlightStatus {
    bool armed = false;
    bool in_air = false;
    int mission_current = 0;
    int mission_total = 0;
    float takeoff_altitude_m = 0.0F;  ///< 最近一次 Takeoff 指令高度
};
```

并在 `FlightControlApi` 末尾（`link_state()` 之后）追加：

```cpp
    virtual plane::flight_control::FlightStatus flight_status() const = 0;
```

- [ ] Step 2: `flight_control_module.h` 修改

1) `TelemetryCache` 中把 `std::string flight_mode;` 改为：

```cpp
        mavsdk::Telemetry::FlightMode flight_mode = mavsdk::Telemetry::FlightMode::Unknown;
```

并追加：

```cpp
        bool armed = false;
        bool in_air = false;
        int mission_current = 0;
        int mission_total = 0;
        float takeoff_altitude_m = 0.0F;
```

2) 公开方法区追加：

```cpp
    plane::flight_control::FlightStatus flight_status() const override;
```

- [ ] Step 3: `flight_control_module.cpp` 修改

1) `subscribe_flight_mode` 回调改为直接存枚举：

```cpp
    telemetry_->subscribe_flight_mode([this](mavsdk::Telemetry::FlightMode value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.flight_mode = value;
    });
```

2) 在 `subscribe_flight_mode` 之后追加：

```cpp
    telemetry_->subscribe_armed([this](bool value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.armed = value;
    });
    telemetry_->subscribe_in_air([this](bool value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.in_air = value;
    });
    mission_->subscribe_mission_progress([this](mavsdk::Mission::MissionProgress value) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.mission_current = value.current;
        cache_.mission_total = value.total;
    });
```

3) `state_loop()` 中 `state.flightMode` 赋值改为：

```cpp
            state.flightMode = std::string(magic_enum::enum_name(cache_.flight_mode));
```

4) `takeoff()` 中 `action_->takeoff()` 成功后追加：

```cpp
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.takeoff_altitude_m = height_m;
    }
```

5) `detach_system()` 中 `connected_ = false;` 之后追加缓存复位：

```cpp
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cache_.armed = false;
        cache_.in_air = false;
        cache_.mission_current = 0;
        cache_.mission_total = 0;
    }
```

6) 文件末尾（`now_ms()` 之前或之后）追加：

```cpp
plane::flight_control::FlightStatus FlightControlModule::flight_status() const {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    plane::flight_control::FlightStatus status;
    status.armed = cache_.armed;
    status.in_air = cache_.in_air;
    status.mission_current = cache_.mission_current;
    status.mission_total = cache_.mission_total;
    status.takeoff_altitude_m = cache_.takeoff_altitude_m;
    return status;
}
```

- [ ] Step 4: 构建、格式与提交

```bash
cmake --build build -j
timeout --preserve-status 2 ./build/onboard; echo "exit=$?"
clang-format --dry-run --Werror src/flight_control/flight_control_api.h src/flight_control/flight_control_module.h src/flight_control/flight_control_module.cpp
git add src/flight_control/flight_control_api.h src/flight_control/flight_control_module.h src/flight_control/flight_control_module.cpp
git commit -m "feat(m5): FlightControl 增加飞行状态快照"
```

Expected: 构建通过，运行正常退出 exit=0（飞控仍不可用，不影响）。

---

## Task 3：动作节点库

**Files**

- Create: `src/task/nodes/flight_action_node.h`
- Create: `src/task/nodes/arm_node.h/.cpp`
- Create: `src/task/nodes/takeoff_node.h/.cpp`
- Create: `src/task/nodes/land_node.h/.cpp`
- Create: `src/task/nodes/rtl_node.h/.cpp`
- Create: `src/task/nodes/hover_node.h/.cpp`
- Create: `src/task/nodes/stop_motor_node.h/.cpp`
- Modify: `CMakeLists.txt`

- [ ] Step 1: 写 `src/task/nodes/flight_action_node.h`

```cpp
#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <spdlog/spdlog.h>

#include "core/config.h"
#include "flight_control/flight_control_api.h"
#include "state/state_store_api.h"

namespace plane::task {

/// 飞控动作节点基类：分发一次、轮询完成、统一超时（SPEC §5.8 以实际状态判定）。
class FlightActionNode : public BT::StatefulActionNode {
   public:
    FlightActionNode(const std::string& name, const BT::NodeConfig& config,
                     plane::flight_control::FlightControlApi* flight_control,
                     plane::state::StateStoreApi* state_store,
                     const plane::core::AppConfig* app_config)
        : BT::StatefulActionNode(name, config),
          flight_control_(flight_control),
          state_store_(state_store),
          app_config_(app_config) {}

   protected:
    BT::NodeStatus onStart() override {
        start_ms_ = now_ms();
        if (config().blackboard) {
            config().blackboard->set("current_node", std::string(name()));
        }
        std::string error;
        if (!dispatch(error)) {
            spdlog::error("节点 {} 分发失败: {}", name(), error);
            return BT::NodeStatus::FAILURE;
        }
        return check();
    }

    BT::NodeStatus onRunning() override {
        if (now_ms() - start_ms_ > app_config_->bt_node_timeout_ms) {
            spdlog::error("节点 {} 超时 ({} ms)", name(), app_config_->bt_node_timeout_ms);
            return BT::NodeStatus::FAILURE;
        }
        return check();
    }

    void onHalted() override { spdlog::info("节点 {} 已中止", name()); }

    virtual bool dispatch(std::string& error) = 0;
    virtual BT::NodeStatus check() = 0;

    plane::flight_control::FlightControlApi* flight_control_;
    plane::state::StateStoreApi* state_store_;
    const plane::core::AppConfig* app_config_;

   private:
    std::int64_t now_ms() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }
    std::int64_t start_ms_ = 0;
};

}  // namespace plane::task
```

- [ ] Step 2: 写六个节点（`.h` 结构一致，仅类名不同）

`arm_node.h`：

```cpp
#pragma once

#include "task/nodes/flight_action_node.h"

namespace plane::task {

class ArmNode final : public FlightActionNode {
   public:
    using FlightActionNode::FlightActionNode;
    static BT::PortsList providedPorts() { return {}; }

   protected:
    bool dispatch(std::string& error) override;
    BT::NodeStatus check() override;
};

}  // namespace plane::task
```

其余节点头文件按同样结构替换类名：`TakeoffNode`、`LandNode`、`RtlNode`、`HoverNode`、`StopMotorNode`。

`arm_node.cpp`：

```cpp
#include "task/nodes/arm_node.h"

namespace plane::task {

bool ArmNode::dispatch(std::string& error) {
    return flight_control_->arm(error);
}

BT::NodeStatus ArmNode::check() {
    return flight_control_->flight_status().armed ? BT::NodeStatus::SUCCESS
                                                  : BT::NodeStatus::RUNNING;
}

}  // namespace plane::task
```

`takeoff_node.cpp`：

```cpp
#include "task/nodes/takeoff_node.h"

namespace plane::task {

bool TakeoffNode::dispatch(std::string& error) {
    const auto height = static_cast<float>(app_config_->system_param.safeHeight_m);
    return flight_control_->takeoff(height, error);
}

BT::NodeStatus TakeoffNode::check() {
    const auto status = flight_control_->flight_status();
    if (!status.in_air) {
        return BT::NodeStatus::RUNNING;
    }
    const auto target = static_cast<float>(app_config_->system_param.safeHeight_m);
    const auto tolerance = static_cast<float>(app_config_->bt_takeoff_tolerance_m);
    return state_store_->own_state().geoPos.height >= target - tolerance ? BT::NodeStatus::SUCCESS
                                                                        : BT::NodeStatus::RUNNING;
}

}  // namespace plane::task
```

`land_node.cpp`：

```cpp
#include "task/nodes/land_node.h"

namespace plane::task {

bool LandNode::dispatch(std::string& error) {
    return flight_control_->land(error);
}

BT::NodeStatus LandNode::check() {
    return flight_control_->flight_status().in_air ? BT::NodeStatus::RUNNING
                                                   : BT::NodeStatus::SUCCESS;
}

}  // namespace plane::task
```

`rtl_node.cpp`：

```cpp
#include "task/nodes/rtl_node.h"

namespace plane::task {

bool RtlNode::dispatch(std::string& error) {
    return flight_control_->return_to_launch(error);
}

BT::NodeStatus RtlNode::check() {
    return flight_control_->flight_status().in_air ? BT::NodeStatus::RUNNING
                                                   : BT::NodeStatus::SUCCESS;
}

}  // namespace plane::task
```

`hover_node.cpp`：

```cpp
#include "task/nodes/hover_node.h"

namespace plane::task {

bool HoverNode::dispatch(std::string& error) {
    return flight_control_->hold(error);
}

BT::NodeStatus HoverNode::check() {
    return BT::NodeStatus::SUCCESS;
}

}  // namespace plane::task
```

`stop_motor_node.cpp`：

```cpp
#include "task/nodes/stop_motor_node.h"

namespace plane::task {

bool StopMotorNode::dispatch(std::string& error) {
    return flight_control_->stop_motor(error);
}

BT::NodeStatus StopMotorNode::check() {
    return BT::NodeStatus::SUCCESS;
}

}  // namespace plane::task
```

- [ ] Step 3: `CMakeLists.txt` 的 `add_executable(onboard ...)` 文件列表追加

```cmake
  src/task/nodes/arm_node.cpp
  src/task/nodes/takeoff_node.cpp
  src/task/nodes/land_node.cpp
  src/task/nodes/rtl_node.cpp
  src/task/nodes/hover_node.cpp
  src/task/nodes/stop_motor_node.cpp
```

- [ ] Step 4: 构建与格式

```bash
cmake --build build -j
clang-format --dry-run --Werror $(git ls-files src/task/nodes)
```

Expected: 构建通过且无 warning；格式通过（差异先 `clang-format -i` 再构建）。

- [ ] Step 5: 提交

```bash
git add src/task/nodes CMakeLists.txt
git commit -m "feat(m5): 增加基础指令动作节点库"
```

---

## Task 4：TaskExecutionEngine

**Files**

- Create: `src/task/task_execution_engine.h`
- Create: `src/task/task_execution_engine.cpp`
- Modify: `CMakeLists.txt`

**Interfaces**

- Consumes: 节点库（Task 3）、`FlightStatus`（Task 2）、配置（Task 1）、M4 的 `TaskExecutionEngineApi`/事件
- Produces: `plane::task::TaskExecutionEngine`（`IModule` + `TaskExecutionEngineApi`）

- [ ] Step 1: 写 `src/task/task_execution_engine.h`

```cpp
#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/control_node.h>

#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"
#include "flight_control/flight_control_api.h"
#include "state/state_store_api.h"
#include "task/task_execution_engine_api.h"

namespace plane::task {

/// 任务执行引擎：taskCode → 内置 BehaviorTree，tick 执行并上报进度/结果（ARCHITECTURE §9）。
class TaskExecutionEngine final : public plane::core::IModule, public TaskExecutionEngineApi {
   public:
    TaskExecutionEngine(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                        plane::flight_control::FlightControlApi& flight_control,
                        plane::state::StateStoreApi& state_store);
    ~TaskExecutionEngine() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    bool start(const plane::model::TaskInfo& task, std::string& error) override;
    void abort() override;

   private:
    BT::Tree build_basic_flight();
    void tick_loop();
    void publish_progress(const std::string& node);
    void finish(plane::model::TaskStatus status, const std::string& detail);

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    plane::flight_control::FlightControlApi& flight_control_;
    plane::state::StateStoreApi& state_store_;

    std::mutex mutex_;
    BT::Tree tree_;
    bool tree_ready_ = false;
    std::string current_node_;
    std::thread tick_thread_;
    std::atomic<bool> ticking_{false};
};

}  // namespace plane::task
```

- [ ] Step 2: 写 `src/task/task_execution_engine.cpp`

```cpp
#include "task/task_execution_engine.h"

#include <chrono>
#include <exception>

#include <magic_enum/magic_enum.hpp>
#include <spdlog/spdlog.h>

#include "task/nodes/arm_node.h"
#include "task/nodes/hover_node.h"
#include "task/nodes/land_node.h"
#include "task/nodes/rtl_node.h"
#include "task/nodes/stop_motor_node.h"
#include "task/nodes/takeoff_node.h"
#include "task/task_events.h"

namespace plane::task {

TaskExecutionEngine::TaskExecutionEngine(plane::core::EventBus& event_bus,
                                         const plane::core::AppConfig& config,
                                         plane::flight_control::FlightControlApi& flight_control,
                                         plane::state::StateStoreApi& state_store)
    : event_bus_(event_bus),
      config_(config),
      flight_control_(flight_control),
      state_store_(state_store) {}

TaskExecutionEngine::~TaskExecutionEngine() {
    stop();
}

std::string TaskExecutionEngine::name() const {
    return "Task Execution Engine";
}

bool TaskExecutionEngine::init() {
    return true;
}

bool TaskExecutionEngine::start() {
    return true;
}

void TaskExecutionEngine::stop() {
    abort();
}

void TaskExecutionEngine::release() {}

BT::Tree TaskExecutionEngine::build_basic_flight() {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ArmNode>("Arm", &flight_control_, &state_store_, &config_);
    factory.registerNodeType<TakeoffNode>("Takeoff", &flight_control_, &state_store_, &config_);
    factory.registerNodeType<LandNode>("Land", &flight_control_, &state_store_, &config_);

    auto subtree = std::make_shared<BT::Tree::Subtree>();
    subtree->blackboard = BT::Blackboard::create();
    subtree->tree_ID = "basic_flight";
    subtree->instance_name = "basic_flight";

    BT::NodeConfig node_config;
    node_config.blackboard = subtree->blackboard;

    auto root = factory.instantiateTreeNode("Sequence", "Sequence", node_config);
    auto arm = factory.instantiateTreeNode("Arm", "Arm", node_config);
    auto takeoff = factory.instantiateTreeNode("Takeoff", "Takeoff", node_config);
    auto land = factory.instantiateTreeNode("Land", "Land", node_config);

    auto* control = dynamic_cast<BT::ControlNode*>(root.get());
    control->addChild(arm.get());
    control->addChild(takeoff.get());
    control->addChild(land.get());

    subtree->nodes.push_back(std::move(root));
    subtree->nodes.push_back(std::move(arm));
    subtree->nodes.push_back(std::move(takeoff));
    subtree->nodes.push_back(std::move(land));

    BT::Tree tree;
    tree.subtrees.push_back(subtree);
    tree.initialize();
    return tree;
}

bool TaskExecutionEngine::start(const plane::model::TaskInfo& task, std::string& error) {
    if (task.taskCode != "basic_flight") {
        error = "未知任务类型: " + task.taskCode;
        return false;
    }
    abort();
    std::lock_guard<std::mutex> lock(mutex_);
    try {
        tree_ = build_basic_flight();
    } catch (const std::exception& e) {
        error = std::string("行为树创建失败: ") + e.what();
        return false;
    }
    tree_ready_ = true;
    current_node_.clear();
    ticking_ = true;
    tick_thread_ = std::thread([this]() { tick_loop(); });
    spdlog::info("行为树已创建: basic_flight, taskId={}", task.taskId);
    return true;
}

void TaskExecutionEngine::abort() {
    ticking_ = false;
    if (tick_thread_.joinable()) {
        tick_thread_.join();
    }
    bool had_tree = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        had_tree = tree_ready_;
        if (tree_ready_) {
            tree_.haltTree();
        }
        tree_ready_ = false;
        current_node_.clear();
    }
    if (had_tree) {
        spdlog::info("任务中止，行为树已停止");
    }
}

void TaskExecutionEngine::tick_loop() {
    while (ticking_) {
        BT::NodeStatus status = BT::NodeStatus::IDLE;
        bool changed = false;
        bool exception_thrown = false;
        std::string exception_text;
        std::string node;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!tree_ready_) {
                break;
            }
            try {
                status = tree_.tickOnce();
            } catch (const std::exception& e) {
                exception_thrown = true;
                exception_text = e.what();
            }
            if (!exception_thrown && !tree_.subtrees.empty() && tree_.subtrees.front()->blackboard) {
                tree_.subtrees.front()->blackboard->get("current_node", node);
                if (!node.empty() && node != current_node_) {
                    current_node_ = node;
                    changed = true;
                }
            }
        }
        if (exception_thrown) {
            finish(plane::model::TaskStatus::Failed, "行为树异常: " + exception_text);
            return;
        }
        if (changed) {
            publish_progress(node);
        }
        if (status == BT::NodeStatus::SUCCESS) {
            finish(plane::model::TaskStatus::Completed, "");
            return;
        }
        if (status == BT::NodeStatus::FAILURE) {
            finish(plane::model::TaskStatus::Failed, "节点执行失败: " + node);
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(config_.bt_tick_interval_ms));
    }
}

void TaskExecutionEngine::publish_progress(const std::string& node) {
    TaskProgress progress;
    progress.currentBehaviorTree = "basic_flight";
    progress.currentNode = node;
    event_bus_.publish(events::kExecutionProgress, progress);
    spdlog::info("任务执行进度: basic_flight / {}", node);
}

void TaskExecutionEngine::finish(plane::model::TaskStatus status, const std::string& detail) {
    ticking_ = false;
    TaskEngineResult result;
    result.status = status;
    result.detail = detail;
    event_bus_.publish(events::kEngineResult, result);
    spdlog::info("行为树结束: status={}, detail={}", std::string(magic_enum::enum_name(status)),
                 detail);
}

}  // namespace plane::task
```

- [ ] Step 3: `CMakeLists.txt` 的 `add_executable(onboard ...)` 文件列表追加

```cmake
  src/task/task_execution_engine.cpp
```

- [ ] Step 4: 构建与格式

```bash
cmake --build build -j
clang-format --dry-run --Werror src/task/task_execution_engine.h src/task/task_execution_engine.cpp
```

Expected: 构建通过且无 warning；格式通过（差异先 `clang-format -i` 再构建）。
若 `registerNodeType` 的 ExtraArgs 形式编译失败，回退为显式 PortsList：`factory.registerNodeType<ArmNode>("Arm", ArmNode::providedPorts(), &flight_control_, &state_store_, &config_);`（其余节点同理）；若指针参数仍不兼容，改用 `std::ref` 包装。修正后在报告说明。

- [ ] Step 5: 提交

```bash
git add src/task/task_execution_engine.h src/task/task_execution_engine.cpp CMakeLists.txt
git commit -m "feat(m5): 增加 BehaviorTree 任务执行引擎"
```

---

## Task 5：gcs_sim 任务类型选项与 main 接线

**Files**

- Modify: `tools/gcs_sim/main.cpp`
- Modify: `src/main.cpp`

- [ ] Step 1: gcs_sim 增加 `--task-code`

`Options` 增加：

```cpp
    std::string task_code = "suicide";
```

解析追加：

```cpp
        } else if (arg == "--task-code" && i + 1 < argc) {
            options.task_code = argv[++i];
```

TaskInfo payload 中的 `taskCode` 改为 `options.task_code`。

```bash
cmake --build build -j
clang-format --dry-run --Werror tools/gcs_sim/main.cpp
git add tools/gcs_sim/main.cpp
git commit -m "feat(m5): gcs_sim 支持指定任务类型"
```

- [ ] Step 2: `src/main.cpp` 接线

include 追加：

```cpp
#include "task/task_execution_engine.h"
```

模块注册块改为：

```cpp
    auto state_store = std::make_unique<plane::state::StateStore>(event_bus, config);
    auto flight_control =
        std::make_unique<plane::flight_control::FlightControlModule>(event_bus, config, *state_store);
    auto task_engine = std::make_unique<plane::task::TaskExecutionEngine>(
        event_bus, config, *flight_control, *state_store);
    auto task_scheduler = std::make_unique<plane::task::TaskScheduler>(event_bus, config,
                                                                       *flight_control,
                                                                       task_engine.get());
    auto communication = std::make_unique<plane::communication::CommunicationModule>(
        event_bus, config, *state_store, *task_scheduler);
    task_scheduler->set_communication(*communication);

    plane::core::Runtime runtime;
    runtime.add_module(std::move(state_store));
    runtime.add_module(std::move(flight_control));
    runtime.add_module(std::move(task_engine));
    runtime.add_module(std::move(task_scheduler));
    runtime.add_module(std::move(communication));
    for (auto& module : plane::core::make_placeholder_modules(
             {"Communication", "UAV State Management", "Flight Control", "Task Scheduler",
              "Task Execution Engine"})) {
        runtime.add_module(std::move(module));
    }
```

- [ ] Step 3: 构建、运行与格式

```bash
cmake --build build -j
timeout --preserve-status 2 ./build/onboard; echo "exit=$?"
clang-format --dry-run --Werror src/main.cpp
```

Expected: 构建通过；Task Execution Engine 为真实模块，exit=0。

- [ ] Step 4: 提交

```bash
git add src/main.cpp
git commit -m "feat(m5): 接线 TaskExecutionEngine"
```

---

## Task 6：M5 部分验收

**Files**

- Modify: `docs/plans/m5-behaviortree-engine-tasks.md`
- Modify: `docs/DEV_PLAN.md`

- [ ] Step 1: 干净构建

```bash
rm -rf build logs
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

- [ ] Step 2: basic_flight 失败链路

终端 A：`./build/onboard`

终端 B：

```bash
./build/gcs_sim --no-command --task-code basic_flight --listen-seconds 3
```

Expected（onboard 日志）：

- `行为树已创建: basic_flight, taskId=task-1`；
- `任务执行进度: basic_flight / Arm`；
- `节点 Arm 分发失败: 飞控未连接`；
- `行为树结束: status=Failed, detail=节点执行失败: Arm`；
- `任务结束: taskId=task-1, status=Failed`（Scheduler 日志）；
- gcs_sim `TaskState`：短暂 InProgress 或直接 None（立即回无任务）。

- [ ] Step 3: 未知任务类型

终端 B：

```bash
./build/gcs_sim --no-command --listen-seconds 2
```

Expected：onboard 日志 `任务启动失败: 未知任务类型: suicide`；gcs_sim `TaskState status=None`。

- [ ] Step 4: 规范与回归

```bash
clang-format --dry-run --Werror $(git ls-files src tools)
grep -rn "std::cout\|printf(" src/ tools/ || echo "no cout/printf"
```

Expected：格式通过、无 cout/printf；M1–M4 行为正常。

- [ ] Step 5: 文档与收尾提交

- 勾选本文档并记录验收证据与偏差；
- 更新 `docs/DEV_PLAN.md` §11 索引：M5 → 已完成（部分验收，SITL 待验）。

```bash
git add docs
git commit -m "docs(m5): M5 部分验收与文档更新"
```

---

## 附：SITL 待用户验证清单（M5 交付）

1. `basic_flight` 完整执行：Arm → Takeoff 到安全高度 → Land，每步以实际状态判定完成；
2. 执行中下发新任务/新指令：`haltTree` 后旧任务终止、新内容执行，`TaskState` 期间上报 InProgress+树/节点；
3. 节点超时（`bt_node_timeout_ms`）触发任务 Failed；
4. `bt_tick_interval_ms` 调整后的 tick 频率影响。
