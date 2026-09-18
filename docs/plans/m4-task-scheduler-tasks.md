# M4 任务级计划（Task Scheduler 与控制仲裁）

> 设计依据：`m4-task-scheduler-design.md`（逐项确认；任务终态立即回到无任务）。
> 执行位置：工程根 `onboard/`。
> 验证原则：每个 Task 结束后 x64 Release 构建通过；不写测试代码（SPEC §11）。
> 状态：已完成并部分验收（2026-09-18）；实际动作待用户 SITL 验证（清单见文末）。

---

## Task 1：引擎接口与调度事件

**Files**

- Create: `src/task/task_execution_engine_api.h`
- Create: `src/task/task_events.h`

**Interfaces**

- Produces: `TaskExecutionEngineApi`、`TaskEngineResult`、`TaskProgress`、`TaskSchedulerError` 与事件 ID

- [x] Step 1: 写 `src/task/task_execution_engine_api.h`

```cpp
#pragma once

#include <string>

#include "message.h"

namespace plane::task {

/// 任务执行引擎接口；M5 实现（ARCHITECTURE §8/§9）。
class TaskExecutionEngineApi {
   public:
    virtual ~TaskExecutionEngineApi() = default;

    virtual bool start(const plane::model::TaskInfo& task, std::string& error) = 0;
    virtual void abort() = 0;
};

}  // namespace plane::task
```

- [x] Step 2: 写 `src/task/task_events.h`

```cpp
#pragma once

#include <string>
#include <string_view>

#include "message.h"

namespace plane::task {

/// 引擎执行结果（M5 发布，Scheduler 消费）。
struct TaskEngineResult {
    plane::model::TaskStatus status = plane::model::TaskStatus::None;
    std::string detail;
};

/// 引擎执行进度（M5 发布，Scheduler 消费）。
struct TaskProgress {
    std::string currentBehaviorTree;
    std::string currentNode;
};

/// Task Scheduler 错误事件载荷。
struct TaskSchedulerError {
    std::string detail;
};

namespace events {

inline constexpr std::string_view kEngineResult = "task.engine_result";
inline constexpr std::string_view kExecutionProgress = "task.execution_progress";
inline constexpr std::string_view kError = "task_scheduler.error";

}  // namespace events

}  // namespace plane::task
```

- [x] Step 3: 构建与提交

```bash
cmake --build build -j
clang-format --dry-run --Werror src/task/task_execution_engine_api.h src/task/task_events.h
git add src/task/task_execution_engine_api.h src/task/task_events.h
git commit -m "feat(m4): 增加任务引擎接口与调度事件"
```

---

## Task 2：TaskScheduler 实现

**Files**

- Create: `src/task/task_scheduler.h`
- Create: `src/task/task_scheduler.cpp`
- Modify: `CMakeLists.txt`

**Interfaces**

- Consumes: `TaskExecutionEngineApi`/事件（Task 1）、`FlightControlApi`（M3）、`CommunicationApi`（M1）、`TaskStateProvider`（M2）
- Produces: `plane::task::TaskScheduler`（`IModule` + `TaskStateProvider`，实现 `set_communication`）

说明：Scheduler 需要 `CommunicationApi` 发送派发失败告警，而 Communication 需要 `TaskStateProvider`（Scheduler 实现）——存在构造循环。采用两阶段接线：Scheduler 先以 `nullptr` 构造，Communication 用 Scheduler 作 provider 构造，再 `scheduler.set_communication(*communication)`（Task 5 执行）。

- [x] Step 1: 写 `src/task/task_scheduler.h`

```cpp
#pragma once

#include <mutex>
#include <string>

#include "communication/communication_api.h"
#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"
#include "flight_control/flight_control_api.h"
#include "task/task_events.h"
#include "task/task_execution_engine_api.h"
#include "task/task_state_provider.h"

namespace plane::task {

/// Task Scheduler：Task 与独立 Command 的唯一控制仲裁者（ARCHITECTURE §8）。
class TaskScheduler final : public plane::core::IModule, public TaskStateProvider {
   public:
    TaskScheduler(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                  plane::flight_control::FlightControlApi& flight_control,
                  TaskExecutionEngineApi* engine = nullptr);
    ~TaskScheduler() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    plane::model::TaskState current_task_state() const override;

    /// 两阶段接线：Communication 构造完成后注入，用于告警发送。
    void set_communication(plane::communication::CommunicationApi& communication);

   private:
    enum class ControlKind { None, Command, Task };

    void on_task_info(const plane::model::TaskInfo& task);
    void on_command(const plane::model::Command& command);
    void on_engine_result(const TaskEngineResult& result);
    void on_execution_progress(const TaskProgress& progress);
    bool dispatch_command(const plane::model::Command& command);
    void send_alert(const std::string& detail);
    void report_error(const std::string& detail) const;

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    plane::flight_control::FlightControlApi& flight_control_;
    plane::communication::CommunicationApi* communication_ = nullptr;
    TaskExecutionEngineApi* engine_;

    mutable std::mutex mutex_;
    ControlKind control_ = ControlKind::None;
    plane::model::TaskState task_state_;
};

}  // namespace plane::task
```

- [x] Step 2: 写 `src/task/task_scheduler.cpp`

```cpp
#include "task/task_scheduler.h"

#include <magic_enum/magic_enum.hpp>
#include <spdlog/spdlog.h>

#include "communication/communication_events.h"

namespace plane::task {

TaskScheduler::TaskScheduler(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                             plane::flight_control::FlightControlApi& flight_control,
                             TaskExecutionEngineApi* engine)
    : event_bus_(event_bus), config_(config), flight_control_(flight_control), engine_(engine) {}

TaskScheduler::~TaskScheduler() {
    stop();
}

std::string TaskScheduler::name() const {
    return "Task Scheduler";
}

bool TaskScheduler::init() {
    return true;
}

bool TaskScheduler::start() {
    event_bus_.subscribe<plane::model::TaskInfo>(
        plane::communication::events::kTaskInfoReceived,
        [this](const plane::model::TaskInfo& task) { on_task_info(task); });
    event_bus_.subscribe<plane::model::Command>(plane::communication::events::kCommandReceived,
                                                [this](const plane::model::Command& command) {
                                                    on_command(command);
                                                });
    event_bus_.subscribe<TaskEngineResult>(events::kEngineResult,
                                           [this](const TaskEngineResult& result) {
                                               on_engine_result(result);
                                           });
    event_bus_.subscribe<TaskProgress>(events::kExecutionProgress,
                                       [this](const TaskProgress& progress) {
                                           on_execution_progress(progress);
                                       });
    return true;
}

void TaskScheduler::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    control_ = ControlKind::None;
    task_state_ = plane::model::TaskState{};
}

void TaskScheduler::release() {}

void TaskScheduler::set_communication(plane::communication::CommunicationApi& communication) {
    communication_ = &communication;
}

plane::model::TaskState TaskScheduler::current_task_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return task_state_;
}

void TaskScheduler::on_task_info(const plane::model::TaskInfo& task) {
    if (task.taskAction == plane::model::TaskAction::Stop) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (control_ != ControlKind::Task || task_state_.taskId != task.taskId) {
                spdlog::warn("忽略不匹配的任务停止: taskId={}", task.taskId);
                return;
            }
            spdlog::info("任务中止: taskId={}", task_state_.taskId);
            control_ = ControlKind::None;
            task_state_ = plane::model::TaskState{};
        }
        if (engine_ != nullptr) {
            engine_->abort();
        }
        return;
    }

    bool replaced_task = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        replaced_task = control_ == ControlKind::Task;
        if (replaced_task) {
            spdlog::info("任务被新任务替换: taskId={}", task_state_.taskId);
        }
        control_ = ControlKind::Task;
        task_state_ = plane::model::TaskState{};
        task_state_.taskId = task.taskId;
        task_state_.status = plane::model::TaskStatus::Pending;
    }
    if (replaced_task && engine_ != nullptr) {
        engine_->abort();
    }
    if (engine_ == nullptr) {
        spdlog::warn("执行引擎未接入，任务保持 Pending: taskId={}", task.taskId);
        return;
    }
    std::string error;
    if (engine_->start(task, error)) {
        std::lock_guard<std::mutex> lock(mutex_);
        task_state_.status = plane::model::TaskStatus::InProgress;
        spdlog::info("任务接收并启动: taskId={}", task.taskId);
    } else {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            control_ = ControlKind::None;
            task_state_ = plane::model::TaskState{};
        }
        report_error("任务启动失败: " + error);
    }
}

void TaskScheduler::on_command(const plane::model::Command& command) {
    bool replaced_task = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        replaced_task = control_ == ControlKind::Task;
        if (replaced_task) {
            spdlog::info("任务被独立指令替换: taskId={}", task_state_.taskId);
        }
        control_ = ControlKind::Command;
        task_state_ = plane::model::TaskState{};
    }
    if (replaced_task && engine_ != nullptr) {
        engine_->abort();
    }
    dispatch_command(command);
}

void TaskScheduler::on_engine_result(const TaskEngineResult& result) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (control_ != ControlKind::Task) {
        spdlog::warn("忽略非当前任务的引擎结果");
        return;
    }
    spdlog::info("任务结束: taskId={}, status={}, detail={}", task_state_.taskId,
                 std::string(magic_enum::enum_name(result.status)), result.detail);
    control_ = ControlKind::None;
    task_state_ = plane::model::TaskState{};
}

void TaskScheduler::on_execution_progress(const TaskProgress& progress) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (control_ != ControlKind::Task) {
        return;
    }
    task_state_.currentBehaviorTree = progress.currentBehaviorTree;
    task_state_.currentNode = progress.currentNode;
}

bool TaskScheduler::dispatch_command(const plane::model::Command& command) {
    std::string error;
    bool ok = false;
    switch (command.type) {
        case plane::model::CommandType::Arm:
            ok = flight_control_.arm(error);
            break;
        case plane::model::CommandType::Disarm:
            ok = flight_control_.disarm(error);
            break;
        case plane::model::CommandType::Takeoff: {
            float height = command.takeoffHeight_m;
            if (height <= 0.0F) {
                height = static_cast<float>(config_.system_param.safeHeight_m);
                spdlog::info("Takeoff 未指定高度，使用安全高度: {} m", height);
            }
            ok = flight_control_.takeoff(height, error);
            break;
        }
        case plane::model::CommandType::Land:
            ok = flight_control_.land(error);
            break;
        case plane::model::CommandType::RTL:
            ok = flight_control_.return_to_launch(error);
            break;
        case plane::model::CommandType::Hover:
            ok = flight_control_.hold(error);
            break;
        case plane::model::CommandType::Waypoint:
            if (command.waypoints.empty()) {
                error = "航点为空";
                break;
            }
            ok = flight_control_.goto_waypoint(command.waypoints.front(), error);
            break;
        case plane::model::CommandType::Route:
            if (command.waypoints.empty()) {
                error = "航点为空";
                break;
            }
            ok = flight_control_.execute_route(command.waypoints, error);
            break;
        case plane::model::CommandType::StopMotor:
            ok = flight_control_.stop_motor(error);
            break;
    }
    if (ok) {
        spdlog::info("指令已派发: {}", command.commandId);
        return true;
    }
    const std::string detail = "指令派发失败: " + command.commandId + " (" + error + ")";
    report_error(detail);
    send_alert(detail);
    return false;
}

void TaskScheduler::send_alert(const std::string& detail) {
    if (communication_ == nullptr) {
        spdlog::error("通信模块未接入，告警未发送: {}", detail);
        return;
    }
    plane::model::Alert alert;
    alert.type = "command_dispatch_failed";
    alert.detail = detail;
    if (!communication_->sendToStation(plane::communication::OutgoingMessage{alert})) {
        spdlog::error("告警发送失败: {}", detail);
    }
}

void TaskScheduler::report_error(const std::string& detail) const {
    spdlog::error("TaskScheduler: {}", detail);
    TaskSchedulerError event;
    event.detail = detail;
    event_bus_.publish(events::kError, event);
}

}  // namespace plane::task
```

- [x] Step 3: `CMakeLists.txt` 的 `add_executable(onboard ...)` 文件列表追加

```cmake
  src/task/task_scheduler.cpp
```

- [x] Step 4: 构建与格式

```bash
cmake --build build -j
clang-format --dry-run --Werror src/task/task_scheduler.h src/task/task_scheduler.cpp
```

Expected: 构建通过且无 warning；格式通过（差异先 `clang-format -i` 再构建）。

- [x] Step 5: 提交

```bash
git add src/task/task_scheduler.h src/task/task_scheduler.cpp CMakeLists.txt
git commit -m "feat(m4): 增加 TaskScheduler 控制仲裁"
```

---

## Task 3：通信边界拒绝非地面站控制消息

**Files**

- Modify: `src/communication/communication_module.cpp`

**Interfaces**

- 保证 `communication.task_info` / `communication.command` 仅由 `senderId == station_id` 产生

- [x] Step 1: `handle_envelope()` 开头、station seen 判断之后插入

```cpp
    if ((envelope.type == plane::model::MessageType::TaskInfo ||
         envelope.type == plane::model::MessageType::Command) &&
        envelope.senderId != config_.station_id) {
        spdlog::warn("拒绝非地面站控制消息: senderId={}", envelope.senderId);
        return;
    }
```

- [x] Step 2: 构建、格式与提交

```bash
cmake --build build -j
clang-format --dry-run --Werror src/communication/communication_module.cpp
git add src/communication/communication_module.cpp
git commit -m "feat(m4): 通信边界拒绝非地面站控制消息"
```

---

## Task 4：gcs_sim 调度联调选项

**Files**

- Modify: `tools/gcs_sim/main.cpp`

**Interfaces**

- Produces: `--no-command`、`--no-task`、`--task-action`、`--command-type`、`--takeoff-height`

- [x] Step 1: `Options` 增加字段

```cpp
    bool send_task = true;
    bool send_command = true;
    std::string task_action = "Start";
    std::string command_type = "Arm";
    float takeoff_height_m = 0.0F;
```

- [x] Step 2: 参数解析追加

```cpp
        } else if (arg == "--no-command") {
            options.send_command = false;
        } else if (arg == "--no-task") {
            options.send_task = false;
        } else if (arg == "--task-action" && i + 1 < argc) {
            options.task_action = argv[++i];
        } else if (arg == "--command-type" && i + 1 < argc) {
            options.command_type = argv[++i];
        } else if (arg == "--takeoff-height" && i + 1 < argc) {
            options.takeoff_height_m = std::stof(argv[++i]);
```

- [x] Step 3: 发送与等待逻辑改为可选

将现有「构造 task_payload/TaskInfo、command_payload/Command、顺序等待两个 Ack」的代码块替换为：

```cpp
    bool task_acked = true;
    if (options.send_task) {
        const std::string task_payload =
            nlohmann::json{{"taskId", "task-1"},
                           {"taskCode", "suicide"},
                           {"targetTrackId", ""},
                           {"taskAction", options.task_action}}
                .dump();
        const std::string task_id = send_typed(session, options, "TaskInfo", task_payload, ++sequence);
        std::unique_lock<std::mutex> lock(mutex);
        task_acked = cv.wait_for(lock, std::chrono::milliseconds(options.ack_timeout_ms),
                                 [&]() { return acked_ids.count(task_id) > 0; });
    }
    bool command_acked = true;
    if (options.send_command) {
        const std::string command_payload =
            nlohmann::json{{"commandId", "cmd-1"},
                           {"type", options.command_type},
                           {"takeoffHeight_m", options.takeoff_height_m},
                           {"waypoints", nlohmann::json::array()}}
                .dump();
        const std::string command_id =
            send_typed(session, options, "Command", command_payload, ++sequence);
        std::unique_lock<std::mutex> lock(mutex);
        command_acked = cv.wait_for(lock, std::chrono::milliseconds(options.ack_timeout_ms),
                                    [&]() { return acked_ids.count(command_id) > 0; });
    }
    spdlog::info("Ack 结果: TaskInfo={}, Command={}", task_acked, command_acked);
```

（删除原有 task_payload/task_id/command_payload/command_id 与合并等待块。）

- [x] Step 4: 构建、格式与提交

```bash
cmake --build build -j
clang-format --dry-run --Werror tools/gcs_sim/main.cpp
git add tools/gcs_sim/main.cpp
git commit -m "feat(m4): gcs_sim 支持调度联调选项"
```

---

## Task 5：main 接线与删除 NoTaskStateProvider

**Files**

- Modify: `src/main.cpp`
- Modify: `src/task/task_state_provider.h`
- Modify: `src/task/task_scheduler.h`（若 Task 2 已含 set_communication，无需再改）

**Interfaces**

- 两阶段接线：Scheduler 先构造（无 Communication），Communication 以 Scheduler 为 provider，再 `set_communication`

- [x] Step 1: 删除 `src/task/task_state_provider.h` 中的 `NoTaskStateProvider` 类

文件最终保留：

```cpp
#pragma once

#include "message.h"

namespace plane::task {

/// 当前任务状态数据源（ARCHITECTURE §18）。
class TaskStateProvider {
   public:
    virtual ~TaskStateProvider() = default;

    virtual plane::model::TaskState current_task_state() const = 0;
};

}  // namespace plane::task
```

- [x] Step 2: `src/main.cpp` include 追加

```cpp
#include "task/task_scheduler.h"
```

并移除 `NoTaskStateProvider` 的 include/使用。

- [x] Step 3: 模块注册块改为

```cpp
    auto state_store = std::make_unique<plane::state::StateStore>(event_bus, config);
    auto flight_control =
        std::make_unique<plane::flight_control::FlightControlModule>(event_bus, config, *state_store);
    auto task_scheduler =
        std::make_unique<plane::task::TaskScheduler>(event_bus, config, *flight_control);
    auto communication = std::make_unique<plane::communication::CommunicationModule>(
        event_bus, config, *state_store, *task_scheduler);
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
```

- [x] Step 4: 构建、运行与格式

```bash
cmake --build build -j
timeout --preserve-status 2 ./build/onboard; echo "exit=$?"
clang-format --dry-run --Werror src/main.cpp
```

Expected: 构建通过；日志中 `Task Scheduler` 为真实模块（非占位）；exit=0。

- [x] Step 5: 提交

```bash
git add src/main.cpp src/task/task_state_provider.h
git commit -m "feat(m4): 接线 TaskScheduler 并移除 NoTaskStateProvider"
```

---

## Task 6：M4 部分验收

**Files**

- Modify: `docs/plans/m4-task-scheduler-tasks.md`
- Modify: `docs/DEV_PLAN.md`

- [x] Step 1: 干净构建

```bash
rm -rf build logs
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

- [x] Step 2: 任务接收与 Pending 上报

终端 A：`./build/onboard`

终端 B：`./build/gcs_sim --no-command --listen-seconds 3`

Expected：

- onboard 日志 `执行引擎未接入，任务保持 Pending: taskId=task-1`；
- gcs_sim 周期 `TaskState` 显示 `status=Pending`（行为树/节点为空）。

- [x] Step 3: 指令替换任务 + 派发失败告警

终端 B：`./build/gcs_sim --listen-seconds 3`

Expected：

- onboard 日志 `任务被独立指令替换: taskId=task-1`、`指令派发失败: cmd-1 (飞控未连接)`；
- gcs_sim 收到 `Alert`（payload `type=command_dispatch_failed`）并回 Ack；
- gcs_sim 周期 `TaskState` 为 `status=None`（独立指令不产生任务状态）。

- [x] Step 4: 补安全高度

终端 B：`./build/gcs_sim --no-task --command-type Takeoff --takeoff-height 0 --listen-seconds 2`

Expected：onboard 日志 `Takeoff 未指定高度，使用安全高度: 30 m`，随后派发失败告警。

- [x] Step 5: 任务中止

终端 B（两次运行间保持 onboard 运行）：

```bash
./build/gcs_sim --no-command --listen-seconds 1
./build/gcs_sim --no-command --task-action Stop --listen-seconds 2
```

Expected：onboard 日志 `任务中止: taskId=task-1`；gcs_sim `TaskState` 回到 `status=None`。

- [x] Step 6: 边界拒绝

终端 B（sender-id 非 gcs）：

```bash
./build/gcs_sim --sender-id uav2 --listen-seconds 1
```

Expected：onboard 日志 `拒绝非地面站控制消息: senderId=uav2`，无仲裁行为与告警。

- [x] Step 7: 规范与回归

```bash
clang-format --dry-run --Werror $(git ls-files src tools)
grep -rn "std::cout\|printf(" src/ tools/ || echo "no cout/printf"
```

Expected：格式通过、无 cout/printf；M1/M2/M3 行为正常（Ack、状态上报、飞控连接日志）。

- [x] Step 8: 文档与收尾提交

- 勾选本文档并记录验收证据与偏差；
- 更新 `docs/DEV_PLAN.md` §11 索引：M4 状态 → 已完成（部分验收，SITL 待验）。

```bash
git add docs
git commit -m "docs(m4): M4 部分验收与文档更新"
```

---

## 附：SITL 待用户验证清单（M4 交付）

1. Arm/Takeoff/Land/RTL/Hold/Waypoint/Route/StopMotor 经 Task Scheduler 派发后实际生效；
2. 执行中下发新指令/新任务：旧内容被终止、新内容立即执行；
3. Takeoff 未指定高度时使用安全高度；
4. NED Velocity 不在 M4（M10 由任务逻辑产生）。

---

## 执行记录与偏差

1. **构造循环依赖处理**：Scheduler 需要 CommunicationApi（告警），Communication 需要 TaskStateProvider（Scheduler 实现）。采用两阶段接线：Scheduler 先构造（`engine=nullptr`），Communication 以其为 provider，再 `set_communication`。与设计的接口语义一致，仅为构造顺序调整。
2. **验收证据（2026-09-18，无 SITL）**
   - 任务 Pending：`--no-command` 时 gcs_sim 周期 `TaskState status=Pending`，onboard 日志 `执行引擎未接入，任务保持 Pending: taskId=task-1`；
   - 指令替换 + 告警：`任务被独立指令替换: taskId=task-1`、`指令派发失败: cmd-1 (飞控未连接)`，gcs_sim 收到 `Alert`，随后 `TaskState status=None`（独立指令不产生任务状态）；
   - 安全高度：`--no-task --command-type Takeoff --takeoff-height 0` → `Takeoff 未指定高度，使用安全高度: 30 m`；
   - 任务中止：Stop 后 onboard `任务中止: taskId=task-1`，gcs_sim `TaskState status=None`；
   - 边界拒绝：`--sender-id uav2` → onboard `拒绝非地面站控制消息: senderId=uav2`，gcs_sim 未收到 Ack（`TaskInfo=false, Command=false`）；
   - 回归：M1 Ack、M2 状态上报、M3 飞控连接日志均正常。
3. `clang-format --dry-run --Werror $(git ls-files src tools)` 通过；无 `std::cout/printf`。
4. 任务终态按已确认口径立即回到无任务，终态仅日志/事件可见。
