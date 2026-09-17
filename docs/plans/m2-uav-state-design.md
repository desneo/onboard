# M2 Change Design（UAV 状态管理与周期上报）

> 状态：待用户确认。任务级计划待确认后生成 `m2-uav-state-tasks.md`。
> 来源：`DEV_PLAN.md` 路线图 M2。
> 输入：SPEC §4（已按 2026-09-17 决策修订）、ARCHITECTURE §11/§18、`common_include/message.h`、M1 产物。

---

## 1. 目标与范围

M2 交付状态管理与周期上报：

- UAV State Management：本机状态、其他无人机状态、地面站链路状态的统一存储与同步查询；
- 状态变化事件（本机/他机/GCS 链路）；
- 周期上报：本机状态经 `sendToAll` 上报；当前任务状态经 `sendToStation` 上报 GCS；
- 本机状态写入路径（M3 Flight Control 接入后填充真实值）；
- `TaskStateProvider` 接口与「无任务」默认实现（M5 由 Task Scheduler 实现）；
- gcs_sim 扩展：订阅 `sendToAll`、打印状态报文、模拟他机状态。

不在 M2：飞控数据采集（M3）、任务/指令业务（M4/M5）、安全告警（M9）、他机状态超时失效清理、轨迹平滑与历史。

## 2. 已确认决策（用户 2026-09-17）

| # | 决策 |
|---|---|
| 1 | 地面站只维护通信链路状态：依据最近收到 GCS 报文时间判定（阈值可配）；不维护其位置/空速/电压（SPEC §4.3 已修订） |
| 2 | 任务状态数据源用 `TaskStateProvider` 接口，M2 提供 `NoTaskStateProvider` 默认实现 |
| 3 | 周期上报本机状态**只发 `sendToAll`**（语义为向除本机外所有节点投递，不依赖网络广播；SPEC §9.2 已修订） |
| 4 | 本机状态在 M2 只留写入路径，真实数据由 M3 接入；M2 以默认值验证上报链路 |
| 5 | 当前任务状态经 `sendToStation` 上报 GCS；无任务时 `TaskStatus::None`，即「无任务」 |

## 3. 上层文档同步（已完成）

- SPEC §4.3：地面站仅维护通信链路状态，删除空速/电压/经纬度；§12 验收项同步；
- SPEC §9.2：`sendToAll` 语义为向除本机外所有节点投递，不依赖网络广播；
- ARCHITECTURE §11：去掉地面站业务字段；§18：上报通道与 `sendToAll` 说明；
- requirement §4：同步状态管理范围与 `sendToAll` 语义。

## 4. 模块与文件

### 4.1 `src/state/`（新增）

| 文件 | 职责 |
|---|---|
| `state_types.h` | `GcsLinkState{connected, last_seen_ms}` |
| `state_store_api.h` | 对外稳定接口：查询 + 写入（数据来源模块调用） |
| `state_events.h` | 状态变化事件 ID |
| `state_store.h/.cpp` | `core::IModule` 实现；存储、查询、事件发布、GCS 链路超时检查 |

接口：

```cpp
struct GcsLinkState {
    bool connected = false;
    std::int64_t last_seen_ms = 0;
};

class StateStoreApi {
   public:
    virtual ~StateStoreApi() = default;

    virtual plane::model::VehicleState own_state() const = 0;
    virtual std::vector<plane::model::VehicleState> peer_states() const = 0;
    virtual bool peer_state(const std::string& plane_id, plane::model::VehicleState& out) const = 0;
    virtual GcsLinkState gcs_link() const = 0;

    virtual void update_own_state(const plane::model::VehicleState& state) = 0;
    virtual void update_peer_state(const plane::model::VehicleState& state) = 0;
    virtual void notify_station_seen() = 0;
};

class StateStore final : public plane::core::IModule, public StateStoreApi {
    // name() = "UAV State Management"
};
```

状态变化事件：

| Event ID | 载荷 | 触发 |
|---|---|---|
| `state.own_changed` | `VehicleState` | 每次 `update_own_state` |
| `state.peer_changed` | `VehicleState` | 每次 `update_peer_state` |
| `state.gcs_link_changed` | `GcsLinkState` | 连接状态翻转时 |

### 4.2 `src/task/task_state_provider.h`（新增，仅接口）

```cpp
class TaskStateProvider {
   public:
    virtual ~TaskStateProvider() = default;
    virtual plane::model::TaskState current_task_state() const = 0;
};

/// M2 默认实现：始终「无任务」；M5 由 Task Scheduler 替换。
class NoTaskStateProvider final : public TaskStateProvider {
   public:
    plane::model::TaskState current_task_state() const override;
};
```

### 4.3 Communication 扩展

- 构造依赖新增：`StateStoreApi&`、`TaskStateProvider&`；
- 入站报文处理：`senderId == station_id` 时调用 `state_store_.notify_station_seen()`；`VehicleState` 且非 GCS 来源时调用 `state_store_.update_peer_state(...)`；
- `serialization` 增加 `from_payload(VehicleState)`（`to_payload` M1 已有）；
- 周期上报（在通信工作线程内计时，周期取 `state_report_period_ms`）：
  - 每周期：`sendToAll(state_store_.own_state())`；`sendToStation(task_state_provider_.current_task_state())`；
  - 上报失败按 M1 错误路径发布 `communication.error`（状态类为 best-effort，不进 pending）。

不新增通信事件（避免 state ↔ communication 头文件循环依赖）：状态数据由 Communication 直接写入 StateStore 接口。

### 4.4 配置

| key | 默认 | 说明 |
|---|---|---|
| `gcs_link_timeout_ms` | 5000 | 超过该时长未收到 GCS 报文判定链路断开 |
| `state_report_period_ms` | 1000 | 复用 M1 配置项，本机/任务状态上报周期 |

### 4.5 main 接线

- 创建真实 `StateStore` 模块与 `NoTaskStateProvider`；
- `CommunicationModule` 注入 `EventBus`、`AppConfig`、`StateStore`、`NoTaskStateProvider`；
- 占位模块 skip 列表增加 `"UAV State Management"`。

### 4.6 gcs_sim 扩展（`tools/gcs_sim/main.cpp`）

- 订阅 `sendToStation` 与 `sendToAll`；
- 收到 `VehicleState`/`TaskState` 时打印关键字段（planeId、经纬高、电压、任务状态）并计数；
- 新增 `--send-peer-state <plane-id>`：以该机身份向 `sendToPlane/<目标机>` 发送一条 `VehicleState`（模拟友机状态上报）。

## 5. 线程与生命周期

- StateStore 模块：`start` 时启动 200 ms tick 线程，用于 GCS 链路超时判定与事件发布；`stop` 停止并 join；查询加互斥锁；
- Communication 工作线程：继续承担 RX 队列与 pending，新增周期上报计时（50 ms tick 精度内）；
- 状态数据为「最新有效值」，peer 表按 planeId 覆盖更新，不做平滑与历史（SPEC §4.4/§8.4）。

## 6. M2 验收

1. 构建：x64 Release 通过；`clang-format` 通过；
2. 终端 A `./build/onboard`，终端 B `./build/gcs_sim --plane-id uav1 --listen-seconds 4`：
   - gcs_sim 每秒收到 1 条 `VehicleState`（planeId=uav1）与 1 条 `TaskState`（status=None）；
3. 周期可配：用临时配置（`state_report_period_ms=300`）启动 onboard，gcs_sim 观察到约 3 条/秒；
4. 他机状态：`gcs_sim --plane-id uav1 --send-peer-state uav2` → onboard 日志出现 `他机状态更新: uav2`；
5. GCS 链路：gcs_sim 发送任一报文后 onboard 日志 `地面站链路: connected=true`；gcs_sim 退出且超过 `gcs_link_timeout_ms` 后 onboard 日志 `地面站链路: connected=false`（验证时可用临时配置把阈值调小至 1000 ms）；
6. 回归：M1 的 TaskInfo/Command Ack 行为不受影响。

## 7. M2 不做

- 本机状态真实数据（M3）；
- 其他无人机状态超时失效与清理；
- GCS 业务状态（位置/速度/电压，已从 SPEC 移除）；
- 状态历史、轨迹平滑、雷达目标；
- 事件驱动的状态广播（消费者按需同步查询）。

## 8. 风险与说明

| # | 事项 | 处理 |
|---|---|---|
| 1 | M2 阶段本机状态为默认值（planeId 已填），上报内容无实际飞控数据 | 预期行为；M3 接入 `update_own_state` 后自动生效 |
| 2 | `sendToAll` 经 Zenoh 路由投递，无线环境下可能受传输方式影响 | 语义与配置已在 SPEC/ARCH 明确；如部署发现多播问题，用 `zenoh_connect/listen` 或后续评估逐节点单播 |
| 3 | 他机状态长期保留，不清理 | 第一版规模 ≤8 架，内存可控；后续按需加超时失效 |
| 4 | Communication 直接写 StateStore（同步调用） | 避免 state↔communication 头文件循环依赖；符合 ARCH「调用明确能力」 |
