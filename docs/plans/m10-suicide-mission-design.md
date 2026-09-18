# M10 Change Design（自杀式反无任务全链路）

> 状态：已确认（2026-09-18）。任务级计划见 `m10-suicide-mission-tasks.md`。
> 来源：`DEV_PLAN.md` 路线图 M10。
> 输入：SPEC §6/§7/§12、ARCHITECTURE §9/§13/§15/§16/§21/§22/§23/§28.6、M3 `FlightControlApi`、M5 引擎与节点、M6 `PerceptionApi`/`perception.*` 事件、M7 `TargetStoreApi`、M9 `SystemParamApi`/运行配置、`../common/common_include`（message.h、functional/*.h）。
> 用户 2026-09-18 最新要求：**“perception 要直连飞控，因为它要知道下发的这条指令是下发给飞控成功还是失败；下发飞控指令的都改成直连都有这个要求”**。该要求与 ARCHITECTURE §15/§23 冲突，本设计按“先改架构文档 + ADR”处理，见 §3。

---

## 1. 目标与范围

M10 交付自杀式反无任务全链路：

- 内置任务 `suicide`（BehaviorTree）：Arm → Takeoff（安全高度）→ 中段制导 → 目标检测 → 单目标跟踪 → 导引撞击；
- 阶段切换条件与失败判定全部配置化（SPEC §6“配置/任务参数”）；
- 新增 `src/geo/` Geographic Tool（GeographicLib 封装），供中段制导与撞击判定计算距离/方位（ARCHITECTURE §21、ADR-012）；
- **Perception 直连 FlightControl** 下发导引速度并获取成败（用户最新要求，见 §3）；
- `TargetStoreApi` 暴露跟踪状态（有效/丢失），供跟踪与导引节点判定（M7 遗留：M7 设计已声明“M10 依据 lost 与任务参数判定失败”）；
- `PerceptionApi` 与 stub 插件支持无真实算法/无源时的链路演练（合成输出，仅联调用途）；
- `gcs_sim` 支持 `targetTrackId` 与 GCS 目标联调；
- 端到端演练、SPEC §12 验收清单走查、部署打包。

不在 M10：

- 真实 AI 算法、算法模型与推理框架（SPEC §13.1）；
- 视觉目标转 WGS84、轨迹平滑、雷达目标（SPEC §13.2）；
- 路径规划、盘旋半径、电子围栏（SPEC §13.2）；
- 告警自动处置（SPEC §13.2）；
- 任务参数体系扩展（沿用配置文件参数 + `TaskInfo.targetTrackId`）。

---

## 2. 已确认决策（用户 2026-09-18）

| # | 决策 |
|---|---|
| 1 | 中段制导采用 **Offboard NED 速度逼近**：Geographic Tool 算目标方位/距离，持续向最新 GCS 目标位置逼近 |
| 2 | 检测启动条件：**水平距离 ≤ 配置阈值即 `start_detection`，不悬停**（检测/跟踪阶段继续向目标逼近） |
| 3 | 导引输出通路：**Perception 直连 FlightControl 下发导引速度并获取成败**；统一原则“下发飞控指令必须同步直连、拿到结果” |
| 4 | 导引启动条件：**跟踪稳定**（连续无丢失达到配置时长）后启动 |
| 5 | 撞击完成判定：**遥测异常判定**（具体条件见 §6.5，待逐项确认） |
| 6 | 无 SITL/无实流：阶段控制与撞击无法端到端验证，按“部分验收 + SITL/实流待验清单”处理（同 M3–M9 口径） |

---

## 3. 架构变更（需授权）

用户要求通过 C++ 同步调用让 Perception 下发飞控并获取成败，属于 ARCHITECTURE 边界变化。按 AGENTS.MD“涉及架构变化先改文档并确认”，本 Change 需同步：

| 文档 | 变更 |
|---|---|
| `ARCHITECTURE.md` §15/§15.3 | Perception 职责增加：导引阶段直连 FlightControl 下发导引速度（仅在 `guidance_enabled` 时）；控制权归任务执行层（`start_guidance/stop_guidance`），Perception 只做算法输出→下发与结果反馈 |
| `ARCHITECTURE.md` §22/§23 | 依赖与通信模型增加“Perception → FlightControl（导引速度，仅此一项）”，并写入统一原则：**下发飞控指令的模块必须同步直连并获取结果** |
| `ARCHITECTURE.md` §30 | 新增 **ADR-018：业务模块下发飞控指令采用同步直连**（Context/Decision/Alternatives/Rationale/Consequences） |
| `SPEC.md` | 不改（SPEC 未限定下发通路；“完成按实际结果判定”不受影响） |

飞控指令下发通路现状核查（本次一并确认）：

| 下发方 | 现状 | 结论 |
|---|---|---|
| Task Scheduler（独立指令） | 同步直连 FlightControl，已获取 bool+error | 符合 |
| Task Execution Engine（任务步骤） | 节点同步直连 FlightControl | 符合 |
| Perception（导引速度） | 仅发布 `perception.guidance` 事件，不直连 | **本次改为直连** |

安全边界（写入设计与 ADR）：

- Perception 仅在 `guidance_enabled_` 为真且跟踪有效时下发；`stop_guidance()`、`stop()`、任务中止/结束时调用 `clear_ned_velocity()`；
- 下发失败（`set_ned_velocity` 返回 false）记 `perception.error` 事件与 error 日志，并写入 `GuidanceResult.applied=false`；
- “下发成功”仅表示飞控接受设定值；实际撞击完成由任务节点按遥测判定（SPEC §5.8 口径不变）。

---

## 4. Geographic Tool（`src/geo/`）

| 文件 | 职责 |
|---|---|
| `src/geo/geo_tool.h/.cpp` | 无状态地理计算工具（GeographicLib 封装），不依赖任何业务模块 |

```cpp
namespace plane::geo {
class GeoCalculator {
   public:
    static double distance_m(const plane::model::GeoPoint& from, const plane::model::GeoPoint& to);
    static double bearing_deg(const plane::model::GeoPoint& from, const plane::model::GeoPoint& to);
};
}  // namespace plane::geo
```

- `distance_m`：Geodesic::Inverse 的大地线距离；`bearing_deg`：起点方位角（0–360，北 0，顺时针）；
- GeographicLib 类型不出 `geo/`；业务模块不得直接 include GeographicLib（ARCHITECTURE §21/§29-11）；
- CMake：`CMakeLists.txt` 增加 `src/geo/geo_tool.cpp`；`GeographicLib::GeographicLib` 已由 `plane::deps` 提供；
- 首个消费者：`MidcourseNode`（因此模块目录在本 Change 创建，符合 DEV_PLAN §4）。

---

## 5. 执行引擎与行为树

### 5.1 文件

| 文件 | 职责 |
|---|---|
| `src/task/task_execution_engine.h/.cpp` | 增加 `build_suicide(task)`；taskCode 映射 `suicide`；终结/中止统一清理 |
| `src/task/nodes/midcourse_node.h/.cpp` | 中段制导：GCS 目标逼近，距离达标 → SUCCESS |
| `src/task/nodes/detect_node.h/.cpp` | 启动检测，按规则选定目标并建立跟踪 |
| `src/task/nodes/track_node.h/.cpp` | 跟踪监视：丢失超时 → FAILURE，稳定达标 → SUCCESS |
| `src/task/nodes/guide_node.h/.cpp` | 启动导引；撞击判定 → SUCCESS，阶段超时/持续丢失 → FAILURE |
| `src/task/nodes/approach_controller.h/.cpp` | 中段/检测/跟踪共享的逼近速度计算（纯逻辑，不依赖 BT） |

### 5.2 树结构（C++ 手工建树，沿用 M5 方式）

```text
Sequence (suicide)
 ├── Arm
 ├── Takeoff（SystemParamStore 安全高度）
 ├── Midcourse
 ├── Detect
 ├── Track
 └── Guide
```

- 节点构造注入：`FlightControlApi*`、`StateStoreApi*`、`PerceptionApi*`、`TargetStoreApi*`、`SystemParamApi*`、`const AppConfig*`、`SuicideTaskContext*`；
- `SuicideTaskContext`（引擎成员，树生命周期内有效）：`{ std::string targetTrackId; }`，由 `start(task)` 写入；
- 行为树名固定 `suicide`，进度上报 `currentBehaviorTree=suicide`。

### 5.3 GCS 目标选择

- 中段开始时解析目标：`TaskInfo.targetTrackId` 非空则按 id 查 `TargetStoreApi::gcs_target`；为空则取当前唯一 GCS 目标（多于一个视为无法确定 → FAILURE）；
- 目标不存在 → 节点 FAILURE（任务失败），日志与 `task.engine_result` 可观测；
- 目标位置持续更新：每个 tick 重新查询最新值（不缓存），适配移动目标。

### 5.4 节点行为

| 节点 | onStart | onRunning | onHalted |
|---|---|---|---|
| Midcourse | 解析 GCS 目标；失败 → FAILURE | 距离 ≤ `suicide_detection_start_distance_m` → SUCCESS；否则通过 `ApproachController` 下发 NED 速度；下发失败 → FAILURE | `clear_ned_velocity()` |
| Detect | `PerceptionApi::start_detection`；失败 → FAILURE | 继续逼近；轮询 `vision_targets()` 按选择规则选目标 → `select_target()` 成功 → SUCCESS；`select_target` 失败 → FAILURE | `stop_detection()` |
| Track | 记录开始时间与稳定起点 | `tracking_state()`：有效且连续稳定 ≥ `suicide_guidance_start_stable_ms` → SUCCESS；连续丢失 ≥ `suicide_track_lost_timeout_ms` → FAILURE；期间继续逼近（`suicide_track_approach_speed_m_s`） | 不清理（由下一阶段/引擎统一） |
| Guide | `PerceptionApi::start_guidance`；失败 → FAILURE | 撞击判定成立 → SUCCESS；持续丢失超阈值 → FAILURE；阶段超时 → FAILURE | `stop_guidance()` + `clear_ned_velocity()` |

- Guide 阶段的实际速度下发由 Perception 完成（§3/§6），节点只控制启停与判定；
- 阶段成功/失败沿用 M5“分发失败即 FAILURE、无重试”口径。

### 5.5 ApproachController（逼近控制）

- 输入：`own_state().geoPos`、最新 GCS 目标 `geoPos`、期望速度、配置；
- 输出：`NedVelocity`（水平朝目标方位速度 + 高度保持/接近）与水平距离；
- 高度策略：目标 `height > 0` 时按高度差做垂直速度（限幅 ±`suicide_climb_speed_m_s`），否则 `down_m_s=0` 保持当前高度；
- 计算使用 `GeoCalculator`；不引入 PID/复杂控制（GCS 目标位置由 PX4 Offboard 速度环跟踪）。

### 5.6 超时

- 新增 `suicide_phase_timeout_ms`（默认 300000，即 5 分钟）用于 Midcourse/Detect/Track/Guide；
- `FlightActionNode` 增加可覆写 `node_timeout_ms()`（默认仍取 `bt_node_timeout_ms`），suicide 阶段节点覆写为 `suicide_phase_timeout_ms`；
- 超时 → FAILURE（任务失败），日志打印阶段名与超时值。

### 5.7 终结与中止清理（安全）

引擎 `finish()` 与 `abort()` 统一执行：`clear_ned_velocity()` + `stop_detection()` + `stop_tracking()` + `stop_guidance()`（各接口幂等），确保任何终态（完成/失败/中止/替换）都不遗留飞控速度与算法运行。

---

## 6. 感知与目标改造

### 6.1 Perception 直连 FlightControl

- `Perception` 构造增加 `plane::flight_control::FlightControlApi&`（main 中 FlightControl 先于 Perception 构造，无循环依赖）；
- `process_loop()` 导引分支：`guide()` 成功后调用 `set_ned_velocity(velocity, error)`：
  - 成功：`GuidanceResult{velocity, valid=true, applied=true}`；
  - 失败：`GuidanceResult{valid=true, applied=false}` + `perception.error` 事件 + error 日志；
- `stop_guidance()`：置 `guidance_enabled_=false` 后 `clear_ned_velocity()`；
- `stop()`/析构：确保 `clear_ned_velocity()`（幂等）；
- `GuidanceResult` 增加 `bool applied = false;`（内部结构，非 common_include）。

### 6.2 TargetStoreApi 扩展

```cpp
struct TrackingState {
    plane::model::DetectTarget target;  ///< 最后已知目标
    bool valid = false;                 ///< 是否收到过跟踪结果
    bool lost = false;                  ///< 最新一次跟踪结果是否丢失
};

virtual TrackingState tracking_state() const = 0;
```

- `TargetStore::on_tracking`：记录 `valid_=true`、`lost_=result.lost`（保留最后位置）；
- 保留现有 `tracked_target()`（兼容）；M10 新节点使用 `tracking_state()`；
- 选择规则在任务层实现（不同规则组合 `vision_targets()`），不在 TargetStore 内。

### 6.3 目标选择规则（`suicide_target_select_rule`）

| 取值 | 规则 |
|---|---|
| `nearest_center`（默认） | 目标框中心距画面中心最近 |
| `highest_confidence` | `confidence` 最大（并列取 nearest_center） |
| `first` | 检测列表第一个 |

画面尺寸取 `PerceptionApi::latest_frame()`。

---

## 7. 配置项（`AppConfig` + `config/onboard.conf`）

| key | 默认 | 说明 |
|---|---|---|
| `suicide_detection_start_distance_m` | 200.0 | 中段→检测的水平距离阈值 |
| `suicide_approach_speed_m_s` | 10.0 | 中段水平逼近速度 |
| `suicide_track_approach_speed_m_s` | 5.0 | 跟踪阶段水平逼近速度（降低便于保持目标可见） |
| `suicide_climb_speed_m_s` | 2.0 | 垂直速度限幅（高度差控制） |
| `suicide_target_select_rule` | nearest_center | 目标选择规则 |
| `suicide_guidance_start_stable_ms` | 500 | 跟踪稳定多久后进入导引 |
| `suicide_track_lost_timeout_ms` | 3000 | 连续丢失多久判定任务失败 |
| `suicide_phase_timeout_ms` | 300000 | suicide 阶段节点超时 |
| `suicide_impact_height_m` | 2.0 | 撞击判定：相对高度 ≤ 该值 |
| `suicide_impact_vd_m_s` | 3.0 | 撞击判定：下坠速度 ≥ 该值 |

---

## 8. 联调辅助（仅验证用途）

无真实算法/无图像源时，为打通 SPEC §12“调用检测、跟踪、导引并产生飞控控制”的链路验证：

- `detection_stub`：环境变量 `PLANE_STUB_DETECT=center` 时返回画面中心一个合成目标（100x100、confidence 0.9、type=stub）；未设置时保持 M6 行为（空列表）；
- `guidance_stub`：环境变量 `PLANE_STUB_GUIDE=forward` 时输出 `north_m_s=1.0`（其余为 0）；未设置时保持零速度；
- `tracking_stub`：保持回传入参目标；可选 `PLANE_STUB_TRACK_LOST_AFTER=<n>` 第 n 次后 `track` 返回失败（验证丢失失败路径）；
- 环境变量仅影响 stub 插件，真实算法插件不感知；文档在 `config/onboard.conf` 注释说明。

图像源：用本地视频文件路径（OpenCV 可读）或 RTSP 地址配置 `image_source` 即可驱动 Perception。

---

## 9. gcs_sim 扩展

| 选项 | 说明 |
|---|---|
| `--target-track-id <id>` | 写入 `TaskInfo.targetTrackId`（默认空） |

联调示例：

```bash
# suicide 任务 + 指定 GCS 目标持续更新
./build/gcs_sim --task-code suicide --target-track-id t1 \
  --send-gcs-target t1 --gcs-target-repeat 100 --listen-seconds 5
```

- 既有 `--send-gcs-target`/`--gcs-target-repeat` 复用；GcsTarget 为状态类消息，无需 Ack（M7 口径）。

---

## 10. main 接线

```text
StateStore → FlightControl → SystemParamStore → PluginManagement → CameraControl
→ VideoStreaming → Perception(FlightControl) → TargetStore → TaskExecutionEngine(*)
→ TaskScheduler → Communication → StateMonitor
```

- `TaskExecutionEngine` 构造增加 `PerceptionApi&`、`TargetStoreApi&`（`FlightControl`/`StateStore`/`SystemParam` 已注入）；
- `Perception` 构造增加 `FlightControlApi&`；
- 占位模块列表不变（无新增模块；`geo` 为纯工具）。

---

## 11. 验收方式（部分验收）

当前环境无 SITL、无实流图像源，M10 完成后：

1. 构建无 warning；`clang-format` 通过；无 `std::cout/printf`；
2. 任务启动与失败路径（无 SITL）：下发 `suicide` → Midcourse 分发失败（飞控未连接）→ 任务 Failed，`TaskState` 终态回 None；
3. 阶段条件单测式验证（不依赖飞控）：
   - GCS 目标解析与距离/方位计算日志；
   - 目标选择规则（用自检检测结果或 stub 合成目标）；
   - 跟踪丢失超时 → FAILURE（`PLANE_STUB_TRACK_LOST_AFTER`）；
4. Perception 直连路径：导引自检或 stub 模式下 `set_ned_velocity` 调用日志（飞控未连接时 `applied=false` + error 事件，验证反馈通路）；
5. SPEC §12 清单逐项走查记录（沿用 M1–M9 证据，M10 项标注待 SITL/实流）；
6. 部署打包（见 §12）。

SITL/实流待验清单（交付给用户）：

- 中段制导实际逼近与阈值切换；
- 检测→选定→跟踪→导引完整阶段链路；
- 跟踪丢失恢复/失败；
- 导引阶段产生飞控控制（Offboard 速度）；
- 撞击完成判定；
- 执行中替换/中止时 Offboard 与算法全部退出。

---

## 12. 部署打包（待确认）

DEV_PLAN M10 要求“部署打包（可执行 + 动态库 + 配置）”。候选方案：

| 方案 | 说明 |
|---|---|
| A. CMake install（推荐） | 增加 `install(TARGETS onboard ...)`、插件 `.so`、`config/`；`cmake --install build --prefix dist` 产出 `dist/{bin/onboard, lib/plugins/*.so, config/onboard.conf}`；三方 `.so` 以文档列出，不复制 |
| B. 脚本打包 | `tools/package.sh` 组装 `dist/`（含三方运行库拷贝），可重复执行 |
| C. 暂不做 | 仅交付 build 目录，归档在 M10 验收时人工拷贝 |

---

## 13. 风险与说明

| # | 事项 | 处理 |
|---|---|---|
| 1 | Perception 直连飞控改变架构边界 | 已按用户要求；同步 ARCHITECTURE §15/§22/§23 + ADR-018，需确认后才能改文档 |
| 2 | Offboard 速度控制与 PX4 模式切换 | 中段开始即进入 Offboard（`set_ned_velocity` 内部自动 start）；起飞用 Action，切换时机由首个速度下发触发 |
| 3 | 撞击遥测判定在 SITL 可模拟性 | 条件含 `!in_air`/高度/下坠速度；SITL 下可人工触发，列为待验 |
| 4 | 阶段超时沿用统一值可能不匹配真实任务 | 全部配置化，联调时调整 |
| 5 | stub 合成输出可能被误用于产品 | 仅环境变量显式开启；默认行为不变；文档标注“联调用途” |
| 6 | `suicide` 阶段树对 GCS 目标强依赖 | `targetTrackId` 为空时取唯一 GCS 目标；无法确定则任务失败并告警，行为可观测 |
| 7 | 跟踪阶段继续逼近可能提前接近目标 | 跟踪速度单独可配，默认降低；后续按实飞调参 |

---

## 14. M10 不做

- 真实检测/跟踪/导引算法与模型；
- 视觉目标转 WGS84、轨迹平滑/评价、雷达；
- 路径规划、避障、盘旋半径、电子围栏；
- 任务参数在线下发体系（阈值仅配置文件）；
- 姿态异常告警与自动处置；
- 目标历史任务查询/上报。
