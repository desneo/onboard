# M4 Change Design（Task Scheduler 与控制仲裁）

> 状态：待用户确认（逐项）。任务级计划待确认后生成 `m4-task-scheduler-tasks.md`。
> 来源：`DEV_PLAN.md` 路线图 M4。
> 输入：SPEC §5/§7、ARCHITECTURE §8/§10/§22/§23/ADR-014、M1–M3 产物。

---

## 1. 目标与范围

M4 交付 Task Scheduler：

- 统一仲裁 Task 与独立 Command：任意时刻最多一个控制内容，不排队，新内容立即替换；
- GCS 独立 Command 经仲裁后派发 Flight Control；
- 未指定高度的 Takeoff 补入默认安全高度；
- Task 生命周期与状态（None/Pending/InProgress/Completed/Aborted/Failed）；
- 实现 `TaskStateProvider`，对外提供当前任务状态（无任务时为 None）；
- 派发失败的独立 Command 向 GCS 告警；
- 通信边界拒绝非地面站下发的 Task/Command。

不在 M4：BehaviorTree 与任务实际执行（M5）、安全高度动态下发业务（M9）、指令完成状态跟踪（ARCH §10 明确不跟踪）。

## 2. 已确认决策（用户 2026-09-17）

| # | 决策 |
|---|---|
| 1 | M4 定义 `TaskExecutionEngineApi`；引擎未接入（M5 前）时任务保持 Pending 并 warn，不阻塞接收/替换/中止 |
| 2 | 独立 Command 派发失败：**不重试**，记 `task_scheduler.error` 事件并向 GCS 发送告警 |
| 3 | 删除 M2 的 `NoTaskStateProvider`，由 Task Scheduler 实现 `TaskStateProvider` |
| 4 | 验收为部分验收（无 SITL）：观察仲裁/替换/补高度/告警/状态上报；实际动作待 SITL |

## 3. 模块与文件

### 3.1 `src/task/`（新增/修改）

| 文件 | 职责 |
|---|---|
| `task_execution_engine_api.h` | 新增：M5 引擎接口 |
| `task_events.h` | 新增：引擎结果与执行进度事件 |
| `task_scheduler.h/.cpp` | 新增：`core::IModule` + `TaskStateProvider` 实现 |
| `task_state_provider.h` | 修改：删除 `NoTaskStateProvider` |

接口与事件：

```cpp
/// M5 实现（Task Execution Engine）。
class TaskExecutionEngineApi {
   public:
    virtual ~TaskExecutionEngineApi() = default;
    virtual bool start(const plane::model::TaskInfo& task, std::string& error) = 0;
    virtual void abort() = 0;
};

struct TaskEngineResult {
    plane::model::TaskStatus status = plane::model::TaskStatus::None;  // Completed/Aborted/Failed
    std::string detail;
};

struct TaskProgress {
    std::string currentBehaviorTree;
    std::string currentNode;
};
```

| Event ID | 载荷 | 发布方 | 消费方 |
|---|---|---|---|
| `task.engine_result` | `TaskEngineResult` | M5 引擎 | Task Scheduler |
| `task.execution_progress` | `TaskProgress` | M5 引擎 | Task Scheduler |
| `task_scheduler.error` | `TaskSchedulerError{detail}` | Task Scheduler | 日志/后续 |

（事件在 M4 定义并在 M5 使用；Scheduler 在 M4 即订阅，M5 接入后自动生效。）

### 3.2 Task Scheduler 行为

状态：`enum class ControlKind { None, Command, Task }`；Task 上下文含 `taskId/taskCode/status/currentBehaviorTree/currentNode`。

入站（订阅 `communication.task_info`、`communication.command`）：

1. `TaskInfo(Start)`：
   - 终止当前控制内容（若为 Task 且引擎已接入 → `engine_->abort()`；状态置 `Aborted`）；
   - 记录新 Task：`status = Pending`，清空行为树/节点字段；
   - 引擎已接入 → `engine_->start(task, error)`，成功置 `InProgress`；失败置 `Failed` 并报错；
   - 引擎未接入 → warn「执行引擎未接入，任务保持 Pending」（M5 接入后消除）。
2. `TaskInfo(Stop)`：仅当 `taskId` 与当前任务一致才处理 → 中止（引擎已接入则 `abort()`），状态 `Aborted`；否则忽略并 warn。
3. `Command`：终止当前控制内容（Task 则按上），置 `ControlKind::Command`；
   - Takeoff 且 `takeoffHeight_m <= 0` → 使用 `config.system_param.safeHeight_m` 并记 info 日志；
   - 通过 `FlightControlApi` 派发（Waypoint/Route 使用 `Command.waypoints`）；
   - 派发失败：`task_scheduler.error` 事件 + **向 GCS 发送 `Alert{type="command_dispatch_failed", detail=...}`**（经 `CommunicationApi::sendToStation`，Alert 属可靠发送）；
   - 独立 Command 不产生任务状态（SPEC §4.4）。

`TaskStateProvider::current_task_state()`：

- 无 Task → `TaskStatus::None`；
- 有 Task → 返回缓存（taskId/status/currentBehaviorTree/currentNode）；
- 独立 Command 执行中仍返回 `None`。

### 3.3 通信边界策略（Communication 修改）

- 入站 `TaskInfo` / `Command`：仅当 `senderId == station_id` 才发布事件；否则 warn「拒绝非地面站控制消息: senderId=...」并丢弃（SPEC §5.6/§9.2：其他无人机不得下发任务；指令来源仅地面站或机载内部）。

### 3.4 main 接线

- 创建 `TaskScheduler`（注入 `EventBus`、`AppConfig`、`FlightControlApi`、`CommunicationApi`），engine 传 `nullptr`（M5 注入）；
- Communication 的 `TaskStateProvider` 改传 TaskScheduler；
- 顺序：StateStore → FlightControl → Communication → TaskScheduler → 其余占位；skip 增加 `"Task Scheduler"`；
- 删除 `NoTaskStateProvider` 引用。

### 3.5 gcs_sim 扩展（联调手段）

新增选项（默认行为保持 M1–M3 场景）：

| 选项 | 说明 |
|---|---|
| `--no-command` | 只发 TaskInfo(Start) |
| `--no-task` | 只发 Command |
| `--task-action <Start\|Stop>` | 任务动作 |
| `--command-type <Arm\|Disarm\|Takeoff\|Land\|RTL\|Waypoint\|Route\|Hover\|StopMotor>` | 指令类型 |
| `--takeoff-height <m>` | Takeoff 高度（0 表示未指定，验证补高度） |

## 4. 安全高度

- M4 使用启动配置 `safe_height_m`（`AppConfig.system_param`）；
- 运行期动态下发（`communication.system_param` 事件 → 更新运行时参数）属 M9，M4 不实现；
- 仅对未指定高度的 Takeoff（独立指令；任务内起飞步骤由 M5 处理）。

## 5. 线程与生命周期

- Task Scheduler 不引入独立线程：事件回调在 eventcpp 发布线程执行，采用互斥锁保护状态；
- 命令派发为同步调用（可能阻塞，如 Mission 上传；第一版接受，M5 后视情况调整）；
- `init/start` 订阅事件并初始化状态；`stop` 置 `None` 状态；事件订阅在 `stop` 时保留（对象生命周期内有效）。

## 6. M4 验收（部分验收，无 SITL）

1. 构建与格式通过；
2. 任务接收与状态：`gcs_sim --no-command` → gcs_sim 周期 `TaskState` 显示 `status=Pending`（行为树/节点为空）；onboard 无引擎 warn 日志；
3. 指令替换任务：`gcs_sim`（默认 TaskInfo + Command Arm）→ onboard 日志显示任务被中止、命令派发；gcs_sim 收到 `Alert`（`command_dispatch_failed`，因飞控未连接）并回 Ack；
4. 补安全高度：`gcs_sim --no-task --command-type Takeoff --takeoff-height 0` → onboard 日志 `Takeoff 未指定高度，使用安全高度: 30 m`；
5. 任务中止：`gcs_sim --task-action Stop --no-command`（在任务 Pending 时）→ 状态变 Aborted；
6. 边界拒绝：`gcs_sim --sender-id uav2` 发送控制消息 → onboard 日志 `拒绝非地面站控制消息: senderId=uav2`，不产生仲裁行为；
7. 回归：M1 Ack、M2 状态上报、M3 飞控连接日志正常。

待 SITL：Arm/Takeoff/Waypoint 等实际动作与替换时的真实中止行为。

## 7. M4 不做

- BehaviorTree 与任务实际执行（M5）；
- 指令完成状态跟踪、指令队列（SPEC §13.2）；
- 任务优先级、暂停/恢复（SPEC §13.2）；
- 安全高度动态下发（M9）；
- 内部模块主动产生独立指令的专用 API（当前无消费者，M5 任务内指令直达 Flight Control）。

## 8. 风险与说明

| # | 事项 | 处理 |
|---|---|---|
| 1 | 命令派发同步调用可能阻塞（Mission 上传） | 第一版接受；M5 联调后评估是否需要异步化 |
| 2 | 引擎未接入期间任务停留 Pending | 已确认的过渡行为；M5 接入后自动进入执行 |
| 3 | 替换语义中「中止」与「失败」的区分 | 替换/外部停止 → Aborted；引擎异常 → Failed；符合 SPEC §5.7 |
| 4 | Alert 告警类型为字符串 | 与 ADR-017 一致（第一版字符串） |
| 5 | Task/Command 边界拒绝在 Communication 实现 | senderId 仅在通信边界可用；策略集中、避免事件泄漏来源信息 |
