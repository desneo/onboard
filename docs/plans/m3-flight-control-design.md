# M3 Change Design（飞控接入：MAVSDK + PX4）

> 状态：已确认（2026-09-17 逐项确认）。任务级计划见 `m3-flight-control-tasks.md`。
> 来源：`DEV_PLAN.md` 路线图 M3。
> 输入：SPEC §3/§5.2/§5.3/§5.4、ARCHITECTURE §13/§26、`common_include/message.h`、MAVSDK 3.17.4 本地头文件、M2 产物。

---

## 1. 目标与范围

M3 交付 Flight Control 模块：

- MAVSDK 集成与串口/UDP 连接 PX4；
- 连接生命周期：连接建立、正常通信、连续 2 秒无有效数据判异常、恢复自动重连；
- 飞控能力：Arm、Disarm、Takeoff、Land、RTL、Hover、Waypoint、Route、StopMotor、NED Velocity；
- 飞控遥测 → 本机 `VehicleState`，写入 StateStore 供状态管理与上报使用；
- 飞控连接状态进入 `VehicleState.fcLinkState`。

不在 M3：任务/指令调度（M4/M5）、相机链路、感知、告警业务、Geographic Tool（见 §9）、参数读写与 Health 详情。

## 2. 已确认决策（用户 2026-09-17）

| # | 决策 |
|---|---|
| 1 | 2 秒判定用 MAVSDK `Mavsdk::set_heartbeat_timeout_s(2.0)`（默认 3s），订阅 `System::subscribe_is_connected` 记录连接建立/异常/恢复 |
| 2 | Hover 用 `Action::hold()` |
| 3 | StopMotor 用 `Action::kill()`（仅 GCS 人工触发，由调用方保证） |
| 4 | Waypoint/Route 用 Mission（`upload_mission` + `start_mission`）；NED Velocity 用 Offboard，模块内部 20 Hz 持续发送最新设定值 |
| 5 | SITL 暂不可用：M3 部分验收（构建 + 无 SITL 连接状态日志），飞控动作链路待用户 SITL 运行验证 |

## 3. 模块与文件

### 3.1 `src/flight_control/`（新增）

| 文件 | 职责 |
|---|---|
| `flight_control_api.h` | 对外稳定能力接口（M4 派发指令、M5 BT 节点调用） |
| `flight_control_events.h` | 飞控错误事件 ID 与载荷 |
| `flight_control_module.h/.cpp` | `core::IModule` 实现：连接管理、遥测聚合、能力分发 |

接口（返回 `bool` + 尾部 `std::string& error`，与插件接口约定一致）：

```cpp
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
```

- `goto_waypoint`：上传 1 个航点的 Mission 并启动；
- `execute_route`：上传 N 个航点的 Mission 并启动；
- `set_ned_velocity`：更新最新的 NED 设定值并确保 Offboard 已启动；`clear_ned_velocity` 停止 Offboard 并清零设定值。
- 命令完成判定不属 Flight Control（SPEC §5.8），由调用方（BT 节点）观察 `VehicleState`。

事件：

| Event ID | 载荷 | 说明 |
|---|---|---|
| `flight_control.error` | `FlightControlError{detail}` | MAVSDK 调用失败、连接异常等统一错误（ARCHITECTURE §26） |

### 3.2 遥测 → VehicleState 映射

| 遥测源（MAVSDK） | VehicleState 字段 |
|---|---|
| `subscribe_position` | `geoPos.latitude/longitude/height(relative_altitude_m)/altitude(absolute_altitude_m)` |
| `subscribe_home` | `startPos` |
| `subscribe_attitude_euler` | `roll_deg/pitch_deg/yaw_deg` |
| `subscribe_velocity_ned` | `vn_m_s/ve_m_s/vd_m_s` |
| `subscribe_fixedwing_metrics` | `airspeed_m_s` |
| `subscribe_battery` | `voltage_v` |
| `subscribe_flight_mode` | `flightMode`（magic_enum 转字符串） |
| `System::is_connected` | `fcLinkState`（`connected` / `disconnected`） |
| — | `planeId` = `config.plane_id`；相机角度与 `currentBehaviorTree/currentNode` 不在 M3 填充 |

聚合：遥测回调只更新受锁保护的缓存；独立 10 Hz 线程组装 `VehicleState` 并调用 `StateStoreApi::update_own_state`（避免高频事件风暴）。

### 3.3 连接管理

- 构造：`mavsdk::Mavsdk`（`ComponentType::CompanionComputer`）+ `set_heartbeat_timeout_s(fc_heartbeat_timeout_ms / 1000.0)`；
- `start()`：`add_any_connection(fc_udp_url | serial://<device>:<baud>)`；
- 监控线程（200 ms）：
  - 系统出现（`first_autopilot()`）→ 创建 Action/Telemetry/Mission/Offboard 插件、建立遥测缓存与 `subscribe_is_connected`，日志「飞控连接建立」；
  - 系统消失/心跳超时（MAVSDK 按 2 s 判定）→ 释放插件与订阅、日志「飞控连接异常」，MAVSDK 自动重连；
  - 恢复 → 日志「飞控连接恢复」；
- 连接状态通过 `vehicle_state.fcLinkState` 对外体现；`link_state()` 返回当前字符串。
- 所有 `add_any_connection`/命令调用捕获 `mavsdk::Mavsdk::ConnectionResult` / `Action::Result` 等，失败转 `flight_control.error` + 日志。

### 3.4 Offboard 约束

- PX4 要求在切入 Offboard 前持续收到设定值：`set_ned_velocity` 首次调用时先发送一次设定值再 `offboard.start()`，随后由 20 Hz 定时器持续发送最新设定值；
- `clear_ned_velocity` 停止定时器并 `offboard.stop()`；
- 未连接时调用返回失败并给出 error，不缓存。

## 4. 配置

| key | 默认 | 说明 |
|---|---|---|
| `fc_link_type` | `udp` | `udp` 或 `serial` |
| `fc_udp_url` | `udp://:14540` | UDP 连接 URL |
| `fc_serial_device` | `/dev/ttyUSB0` | 串口设备 |
| `fc_serial_baud` | 57600 | 串口波特率 |
| `fc_heartbeat_timeout_ms` | 2000 | 心跳超时（SPEC §3 的 2 秒） |
| `fc_route_speed_m_s` | 5.0 | Mission 航点默认速度 |

## 5. main 接线

- 创建真实 `FlightControlModule`（注入 `EventBus`、`AppConfig`、`StateStoreApi`）；
- 顺序：`StateStore` → `FlightControl` → `Communication` → 其余占位模块；
- 占位 skip 增加 `"Flight Control"`。

## 6. M3 验收（部分验收，SITL 项待用户执行）

可在当前环境验证：

1. 构建：x64 Release 通过；`clang-format` 通过；
2. 无 SITL 运行：默认 `udp://:14540` 无飞控 → 日志出现飞控连接监控与「未连接」状态，进程稳定，Ctrl+C 正常退出；
3. 状态上报联动：`gcs_sim` 收到周期 `VehicleState`，且 `fcLinkState=disconnected`、`flightMode` 为空（证明遥测聚合路径与上报链路打通）；
4. 命令接口在未连接时返回 false 并产生 `flight_control.error`（通过 M4 接入前的直接调用无法覆盖，M4 验证）。

待用户 SITL 验证（M3 交付时提供清单）：

- 串口与 UDP 连接建立；连续 2 秒无数据判异常；恢复自动重连；
- Arm/Disarm/Takeoff/Land/RTL/Hold/Waypoint/Route/StopMotor；
- NED Velocity 经 Offboard 生效与 `clear_ned_velocity` 退出；
- `VehicleState` 各字段与 PX4 实际状态一致。

## 7. 线程与生命周期

- 连接监控线程（200 ms）、遥测聚合线程（10 Hz）、Offboard 保活线程（20 Hz，仅激活时运行）；
- 遥测订阅与插件随系统出现/消失创建与销毁，受互斥锁保护；
- `stop()` 顺序：停 Offboard/保活 → 退订 → 销毁插件 → `mavsdk` 关闭；`release()` 只做资源清理。

## 8. 风险与说明

| # | 事项 | 处理 |
|---|---|---|
| 1 | SITL 暂不可用，飞控动作链路未端到端验证 | 部分验收 + 用户 SITL 清单；日志定位问题 |
| 2 | MAVSDK 运行期需要 `libmavsdk.so` 等动态库 | 运行时 RPATH/`LD_LIBRARY_PATH`；部署在 M10 统一处理 |
| 3 | Offboard 需要持续设定值 | 20 Hz 保活；未激活时不运行 |
| 4 | Mission 航点高度取 `GeoPoint.height`（相对高度），速度取配置 | 接口设计阶段固定；如需按任务指定再扩展 |
| 5 | MAVSDK 心跳超时从 3 s 调为 2 s，可能在高延迟链路上误判 | 按 SPEC 要求固定 2 s；如现场误判再评估链路质量与阈值配置化 |
| 6 | `VehicleState.currentBehaviorTree/currentNode` 在 M3 为空 | 属任务状态字段，M5 设计时决定是否填充 |

## 9. 路线图调整建议

- `Geographic Tool` 在 M3 无消费者（遥测字段直接映射，无需地理计算），按 AGENTS「避免猜测性功能」推迟到首个消费者（中段制导 M5/M10）时创建；`DEV_PLAN.md` §4 目录树中 `geo/` 的里程碑标注将从 M3 调整为 M5/M10。
- 其余路线图不变。

## 10. M3 不做

- 任务/指令仲裁与派发（M4）、BT 节点（M5）；
- 飞控参数读写、Health 详情、日志回放（SPEC §13.2）；
- Body 速度控制、地面站直发 NED（SPEC §13.2）；
- 多飞控/多机系统管理（第一版单机）。
