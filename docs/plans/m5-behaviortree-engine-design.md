# M5 Change Design（BehaviorTree 任务执行引擎）

> 状态：待用户确认（逐项）。任务级计划待确认后生成 `m5-behaviortree-engine-tasks.md`。
> 来源：`DEV_PLAN.md` 路线图 M5。
> 输入：SPEC §5.6–§5.9、ARCHITECTURE §9/§20/ADR-005/ADR-010、M4 的 `TaskExecutionEngineApi` 与事件、BT.CPP 4.10 本地头文件。

---

## 1. 目标与范围

M5 交付 Task Execution Engine：

- 实现 M4 定义的 `TaskExecutionEngineApi`（`start/abort`）；
- taskCode → 内置 BehaviorTree 策略映射（GCS 不定义树）；
- 内置任务类型 `basic_flight`（Arm → Takeoff(安全高度) → Land），用于打通引擎；
- 基础指令节点库：Arm/Takeoff/Land/RTL/Hover/StopMotor，含统一节点超时；
- 完成判定基于 UAV 实际状态（由 FlightControl 新增 `FlightStatus` 提供）；
- 执行进度与结果通过事件上报：`task.execution_progress`、`task.engine_result`；
- 中止/替换时 `haltTree()` 并停止 tick 线程。

不在 M5：自杀式反无任务树与感知/目标节点（M10）、Waypoint/Route 节点（见 §7）、并行/异步复杂流程、任务参数体系扩展。

## 2. 已确认决策（用户 2026-09-18）

| # | 决策 |
|---|---|
| 1 | taskCode→内置树映射；M5 注册 `basic_flight`，M10 注册 `suicide` |
| 2 | 行为树用 C++ 代码构建（不用 XML） |
| 3 | 完成判定数据源：FlightControl 新增 `FlightStatus`（armed/in_air/mission 进度）与同步查询；不改 `VehicleState`/common_include |
| 4 | M5 节点范围：Arm/Takeoff/Land/RTL/Hover/StopMotor + 统一超时；Waypoint/Route 推迟到 M10 |
| 5 | 无 SITL 部分验收；替换/中止与真实飞行动作待 SITL |

## 3. FlightControl 扩展（M3 增量）

`flight_control_api.h` 追加：

```cpp
struct FlightStatus {
    bool armed = false;
    bool in_air = false;
    int mission_current = 0;
    int mission_total = 0;
    float takeoff_altitude_m = 0.0F;  ///< 最近一次 Takeoff 指令高度
};

// FlightControlApi 内追加：
virtual plane::flight_control::FlightStatus flight_status() const = 0;
```

`FlightControlModule`：

- 遥测新增订阅：`subscribe_armed`、`subscribe_in_air`、`subscribe_mission_progress`，写入缓存；
- `takeoff()` 成功时记录 `takeoff_altitude_m`；
- `flight_status()` 返回锁保护快照；
- 系统移除时状态复位（armed/in_air=false，mission 归零）。

## 4. 执行引擎设计

### 4.1 文件

| 文件 | 职责 |
|---|---|
| `src/task/task_execution_engine.h/.cpp` | `IModule` + `TaskExecutionEngineApi`；策略映射、tick 线程、事件发布 |
| `src/task/nodes/flight_action_node.h` | 动作节点基类：超时、分发一次、轮询完成 |
| `src/task/nodes/arm_node.h/.cpp` | Arm：分发成功后以 `FlightStatus.armed` 判定 |
| `src/task/nodes/takeoff_node.h/.cpp` | Takeoff：输入高度；`in_air && height >= 目标 - 容差` |
| `src/task/nodes/land_node.h/.cpp` | Land：`!in_air` |
| `src/task/nodes/rtl_node.h/.cpp` | RTL：`!in_air` |
| `src/task/nodes/hover_node.h/.cpp` | Hover：分发成功即 SUCCESS |
| `src/task/nodes/stop_motor_node.h/.cpp` | StopMotor：分发成功即 SUCCESS |

### 4.2 节点契约

- 均继承 `BT::StatefulActionNode`，构造参数 `(name, config, FlightControlApi&, StateStoreApi&, const AppConfig&)`；
- `onStart()`：记录起始时间、调用一次分发（Arm/Takeoff/Land/RTL/Hover/StopMotor），分发失败 → `FAILURE`；同时把节点名写入黑板 `current_node`；
- `onRunning()`：超时（`bt_node_timeout_ms`）→ `FAILURE`；否则调用 `check()` 返回 `SUCCESS/RUNNING`；
- `onHalted()`：仅日志（动作不撤销；后续如需发 Hold 再评估）；
- 完成判定只读 `FlightStatus` / `VehicleState`，不以「指令已发送成功」为准（SPEC §5.8）。

### 4.3 内置任务 `basic_flight`

```text
Sequence
 ├── Arm
 ├── Takeoff(height = safe_height_m)
 └── Land
```

- 树名 `basic_flight`；Takeoff 高度来自 `AppConfig.system_param.safeHeight_m`；
- 映射：`if (task.taskCode == "basic_flight") build_basic_flight(factory)`；未知 taskCode → 启动失败（`Failed` + 错误事件）。

### 4.4 tick 与进度

- 引擎持有 1 个 tick 线程，周期 `bt_tick_interval_ms`（默认 50 ms）；
- 每次 tick 后读取黑板 `current_node`，变化时发布 `task.execution_progress{tree, node}` 并记 info 日志；
- `tickOnce()` 返回终态：SUCCESS → `TaskEngineResult{Completed}`；FAILURE → `{Failed, 节点名}`；异常捕获 → `{Failed, what()}`；
- 终态发布后停止 tick 线程并等待下一次 `start()`。

### 4.5 中止

- `abort()`：置停止标志 → `haltTree()` → join；不发布引擎结果（由 Scheduler 处理状态），仅 info 日志「任务中止，行为树已停止」。

## 5. 配置

| key | 默认 | 说明 |
|---|---|---|
| `bt_tick_interval_ms` | 50 | 行为树 tick 周期 |
| `bt_node_timeout_ms` | 30000 | 单节点超时（含分发与等待完成） |
| `bt_takeoff_tolerance_m` | 1.0 | 起飞到达判定容差 |

## 6. main 接线

```text
StateStore → FlightControl → TaskExecutionEngine → TaskScheduler(engine) → Communication → 占位
```

- `TaskScheduler` 构造时传入引擎指针（M4 已预留 `engine` 参数）；
- 占位 skip 增加 `"Task Execution Engine"`；
- 无 SITL 时 `basic_flight` 首节点 Arm 分发失败 → 任务 Failed（可观测）。

## 7. M5 不做（含路线图调整）

- `suicide` 任务树、检测/跟踪/导引节点、目标交互（M10）；
- Waypoint/Route 节点：`TaskInfo` 无航点数据，基础指令序列任务中的航点应由任务类型内置；推迟到 M10 与具体任务定义一并实现；
- 节点级重试、并行节点、行为树热更新；
- 特殊任务参数（检测启动条件等，M10）。

## 8. M5 验收（部分验收，无 SITL）

1. 构建与格式通过；
2. `gcs_sim --no-command --listen-seconds 3`：
   - onboard 日志：任务接收 → `行为树已创建: basic_flight` → 节点 `Arm` 分发失败（飞控未连接）→ `task.engine_result: Failed` → Scheduler `任务结束: status=Failed`；
   - gcs_sim `TaskState`：任务开始后短暂 `InProgress`（行为树/节点字段可见，若轮询间隔错过则为 None）；
3. 未知 taskCode（gcs_sim 目前固定 `suicide`，用 `--task-action Start` 缺省即可观察）：当前 `gcs_sim` 发送 `taskCode=suicide` → 引擎应返回启动失败 `未知任务类型: suicide`（M10 接入前），onboard 日志 `任务启动失败`；
4. 中止/替换路径：无 SITL 下任务立即失败，替换窗口过短，**待 SITL 验证**（SITL 下 basic_flight 将持续 RUNNING，可观察）；
5. 规范与回归（M1–M4 行为不变）。

SITL 待验清单：

- `basic_flight` 完整执行：Arm → Takeoff 到安全高度 → Land，且每步以实际状态判定完成；
- 执行中下发新任务/新指令：`haltTree` 后旧任务终止、新内容执行；
- 节点超时（人为使某步不可达）→ 任务 Failed；
- `TaskState` 在执行期间上报 `InProgress` + 行为树/节点，终态后回到 None。

## 9. 风险与说明

| # | 事项 | 处理 |
|---|---|---|
| 1 | `basic_flight` 是机载内置验证任务类型，SPEC 未点名 | 设计已确认；M10 补充 `suicide` |
| 2 | 无 SITL 时替换/中止路径不可端到端验证 | 列入 SITL 待验清单 |
| 3 | 节点在 tick 线程内调用 FlightControl 同步分发（Arm 等阻塞时间短） | 第一版接受；如 Mission 类节点阻塞明显，M10 再评估异步化 |
| 4 | `FlightStatus` 为 FlightControl 接口类型，不进入 common_include | 仅机载内部消费，符合模块边界 |
| 5 | 节点分发失败即 FAILURE，符合 SPEC §5.8 一步失败则任务失败 | 无重试 |
