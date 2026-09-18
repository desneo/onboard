# M9 Change Design（安全告警与系统参数动态下发）

> 状态：已确认（2026-09-18 逐项；仅低电压告警）。任务级计划见 `m9-state-monitor-tasks.md`。
> 来源：`DEV_PLAN.md` 路线图 M9。
> 输入：SPEC §7/§10（已修订：仅低电压）、ARCHITECTURE §7/§12、M2 的 `state.own_changed` 事件、M4 的 Alert 发送先例、M5 的 TakeoffNode。

---

## 1. 目标与范围

M9 交付：

- `SystemParamStore`（core）：启动加载配置默认值，运行中接受 GCS `SystemParam` 动态更新，对外提供 `SystemParamApi` 与 `system_param.changed` 事件；
- TaskScheduler 与 TakeoffNode 改读运行时安全高度；
- StateMonitor：仅低电压告警（阈值可配）、仅在飞控连接正常时评估、边沿触发、经 Communication 上报 GCS；不自动处置；
- gcs_sim `--send-param <k=v>` 动态下发验证。

不在 M9：姿态异常告警（已从 SPEC 移除）、自动返航/降落/中止/接管、链路质量监控、电子围栏。

## 2. 已确认决策（用户 2026-09-18）

| # | 决策 |
|---|---|
| 1 | 安全高度用 `SystemParamStore` 存与分发（非可变 AppConfig） |
| 2 | 告警边沿触发（正常→异常一次；恢复后可再次触发） |
| 3 | 仅 `fcLinkState == "connected"` 时评估阈值 |
| 4 | 第一版**仅低电压告警**，阈值配置项 `alert_voltage_threshold_v` 默认 21.0（6S 参考值）；已授权同步修订 SPEC/ARCH/requirement |
| 5 | 无 SITL：告警链路等 SITL 验证；动态参数下发可在当前环境验证 |
| 6 | gcs_sim 增加 `--send-param <k=v>`（可多次） |

## 3. SystemParamStore（core）

| 文件 | 职责 |
|---|---|
| `src/core/system_param_api.h` | 查询接口 |
| `src/core/system_param_store.h/.cpp` | `core::IModule` + `SystemParamApi`；订阅并发布事件 |

```cpp
class SystemParamApi {
   public:
    virtual ~SystemParamApi() = default;
    virtual plane::model::SystemParam current() const = 0;
};
```

- 构造：`SystemParamStore(const AppConfig&)`，以 `config.system_param` 为启动默认值；
- `start()`：订阅 `communication.system_param` → 覆盖当前值 → 发布 `system_param.changed`（载荷 `SystemParam`）→ info 日志 `系统参数更新: safe_height_m=...`；
- `stop()`：不做特殊处理（保留当前值；进程重启后回到配置默认值）；

### 3.1 消费方改造（M5 增量）

- `TaskScheduler` 构造增加 `SystemParamApi&`；Takeoff 补高度改读 `system_param_.current().safeHeight_m`；
- `TaskExecutionEngine` 构造增加 `SystemParamApi&`；`TakeoffNode` 增加 `SystemParamApi*` 额外参数，高度取 `current().safeHeight_m`（容差/超时仍用 AppConfig）。

## 4. StateMonitor

### 4.1 文件（`src/state/`）

| 文件 | 职责 |
|---|---|
| `state_monitor.h/.cpp` | `core::IModule`；订阅 `state.own_changed`，低电压边沿告警 |

### 4.2 行为

- 订阅 `state.own_changed`（载荷 `VehicleState`）；
- `fcLinkState != "connected"`：不评估，并清除告警闩锁（重连后重新评估）；
- `voltage_v < alert_voltage_threshold_v` 且未闩锁 → 发送 `Alert{type="low_voltage", detail="voltage=..V < threshold=..V"}`（经 `CommunicationApi::sendToStation`，Alert 属可靠发送）+ warn 日志 + 置闩锁；
- 电压恢复到阈值以上 → 清除闩锁 + info 日志 `电压恢复正常`；
- 不做任何自动处置（SPEC §10）。

## 5. 配置

| key | 默认 | 说明 |
|---|---|---|
| `alert_voltage_threshold_v` | 21.0 | 低电压告警阈值（6S 参考值，按实机电池配置） |

## 6. gcs_sim 扩展

| 选项 | 说明 |
|---|---|
| `--send-param <key=value>` | 发送 SystemParam（可多次；当前仅 `safe_height_m`），payload JSON 数值 |

- 发送到 `sendToPlane/<plane-id>`，无需 Ack（SystemParam 非 Task/Command）。

## 7. main 接线

- 创建 `SystemParamStore`（先于 TaskScheduler/TaskExecutionEngine）并注入两者；
- 创建 `StateMonitor`（在 Communication 之后，注入 `CommunicationApi&`）；
- 注册顺序：`... TaskScheduler → Communication → StateMonitor → 占位`；`SystemParamStore` 注册在 TaskExecutionEngine 之前；
- 占位 skip 增加 `"StateMonitor"`。

## 8. M9 验收（部分验收）

1. 构建与格式通过；
2. 启动：`SystemParamStore` 初始化日志（safe_height_m=配置值）；`StateMonitor` 为真实模块；飞控未连接时无告警；
3. 动态参数下发：`gcs_sim --send-param safe_height_m=50` → onboard `系统参数更新: safe_height_m=50`；随后 `gcs_sim --no-task --command-type Takeoff --takeoff-height 0` → onboard `Takeoff 未指定高度，使用安全高度: 50 m`；
4. 回归与规范（M1–M8 行为不变；clang-format；无 cout/printf）。

待 SITL 验证：

- 低电压告警：SITL 设置低电压 → GCS 收到 `Alert{low_voltage}`，且系统无自动处置；恢复电压后再次触发有效；
- `basic_flight` 的 TakeoffNode 使用动态安全高度。

## 9. M9 不做

- 姿态异常告警（SPEC 已移除）；
- 告警自动处置、告警去重/抑制窗口（边沿触发已足够）、告警历史；
- 其他系统参数的动态更新（第一版仅安全高度）；
- 参数持久化（重启回默认）。

## 10. 风险与说明

| # | 事项 | 处理 |
|---|---|---|
| 1 | 阈值 21.0 为 6S 参考值 | 配置项可调；上机前按实际电池确认 |
| 2 | 无 SITL 无法验证告警链路 | 等 SITL；当前仅验证断连不误报与参数下发 |
| 3 | `system_param.changed` 暂无消费者 | 为后续（告警/任务）预留；不增加额外逻辑 |
| 4 | M5 代码需小幅改造（构造参数与 TakeoffNode） | 属设计增量，任务计划中单列并回归 M5 |
