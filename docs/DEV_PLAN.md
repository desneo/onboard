# DEV_PLAN.md — 第一版开发计划（总体路线图）

> 本文是机载程序第一版的总体开发路线图（M0–M10），用于确定里程碑顺序、依赖关系、交付物与验证标准。
> 每个里程碑在开工前再细化为可执行的任务清单（按 AGENTS.md：先 Change Design，确认后再生成 tasks）。
> 本文不替代 `SPEC.md` 与 `ARCHITECTURE.md`，冲突时按 AGENTS.md 的信息来源优先级处理。

- 状态：待确认
- 输入：`docs/requirement.md`、`docs/SPEC.md`、`docs/ARCHITECTURE.md`
- 外部接口基线：`../common/common_include`（geo_types.h、message.h、functional/*.h）
- 依赖基线：`../../../3rdparty/common`（相对工程根 `onboard/`）

---

## 1. 目标与范围

交付一个基于 C++17 的单进程机载程序，运行于 Ubuntu 22.04，通过串口或 UDP 连接 PX4，实现：

- 指令与任务接收、仲裁与执行（BehaviorTree 编排）；
- 自杀式反无任务全链路（Arm → Takeoff → 中段制导 → 检测 → 跟踪 → 导引撞击）；
- UAV 状态管理、周期上报、飞控连接管理；
- 相机控制、图像获取、AI 插件调度、目标管理、RTSP 推流；
- GCS / UAV 间 Zenoh 通信；
- 安全告警。

第一版本验证标准：**工程编译构建成功 + 各里程碑联调验收点通过**。第一版本不编写测试代码（SPEC §11、ARCHITECTURE §28.7）。

---

## 2. 全局约束（所有里程碑隐含遵守）

| 项 | 约束 | 来源 |
|---|---|---|
| 语言 | C++17 | SPEC §2、ARCHITECTURE §28.1 |
| 命名空间 | `namespace plane`（强制执行） | ARCHITECTURE §28.4 |
| 构建 | CMake，支持 x86；交叉编译能力保留 | ARCHITECTURE §28.2 |
| 本计划构建目标 | 仅 x64 Release（arm64 暂缓，见 §10） | 用户确认 |
| 平台 | Ubuntu 22.04 | SPEC §2 |
| 依赖接入 | 优先 `find_package`，使用本地已编译库；缺失时提示补充，不自行替换来源 | requirement §3、ARCHITECTURE §28.2 |
| 单进程 | 单进程 + 少量长期线程 + eventcpp 事件驱动；不一个模块一个线程 | ADR-001、ARCHITECTURE §24 |
| 进程内事件 | eventcpp；异步联动注册唯一 Event ID | ARCHITECTURE §3.2/§22 |
| 网络通信 | 仅 Communication 模块使用 Zenoh，业务模块不得直接依赖 | ADR-002、ARCHITECTURE §3.4 |
| 序列化 | nlohmann_json 只在 Communication 模块内使用，JSON 不出通信模块 | ARCHITECTURE §18 |
| 日志 | 统一 spdlog 封装；禁止 `std::cout`/`printf`；单文件 50 MB、保留 5 个 | ARCHITECTURE §28.3 |
| 格式化 | clang-format：Google Style、缩进 4、Column Limit 120、Pointer Alignment Left | ARCHITECTURE §28.5 |
| 公共头文件 | 跨边界结构体统一引用 `../common/common_include`，不重复定义 | ARCHITECTURE §28.6 |
| 测试 | 不写测试代码，编译通过 + 联调验证 | SPEC §11 |
| 文档 | 需求变化改 SPEC，架构变化改 ARCHITECTURE，均需先确认 | AGENTS.md |

### 2.1 构建命令基线

```bash
BASE=/home/zsh/gitlab_0917/3rdparty/common
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$BASE/fmt/12.2.0/x64/ubuntu22.04;\
$BASE/spdlog/v1.17.0/x64/ubuntu22.04;\
$BASE/geographiclib/v2.7/x64/ubuntu22.04;\
$BASE/mavsdk/v3.17.4/x64/ubuntu22.04;\
$BASE/zenohc/1.10.0/x64/ubuntu22.04;\
$BASE/zenohcpp/1.10.0/x64/ubuntu22.04"
cmake --build build -j
```

M0 的 `cmake/Dependencies.cmake` 会内置上述路径（相对工程根 `../../../3rdparty/common`），M0 完成后普通命令即可：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

建议在 `cmake/Dependencies.cmake` 中定义 `PLANE_3RDPARTY_DIR`，默认值：

```cmake
set(PLANE_3RDPARTY_DIR "${CMAKE_SOURCE_DIR}/../../../3rdparty/common" CACHE PATH "本地三方库根目录")
set(PLANE_COMMON_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../common/common_include" CACHE PATH "公共接口头文件目录")
```

---

## 3. 依赖接入基线（2026-09-17 盘点）

`../../../3rdparty/common` 实际内容：

| 库 | 版本 | 接入方式 | 备注 |
|---|---|---|---|
| fmt | 12.2.0 | `find_package(fmt)` | 有 config |
| spdlog | 1.17.0 | `find_package(spdlog)` | 依赖 fmt，须同时提供 fmt 前缀 |
| GeographicLib | 2.7 | `find_package(GeographicLib)` | 有 config |
| MAVSDK | 3.17.4 | `find_package(MAVSDK)` | 还需系统依赖 CURL/jsoncpp/picosha2/tinyxml2/mav/LibEvents/LibLZMA |
| zenohc | 1.10.0 | `find_package(zenohc)` | 有 config（.so + .a） |
| zenohcpp | 1.10.0 | `find_package(zenohcxx)` | 接口库；须先找到 `zenohc::lib` |
| BehaviorTree.CPP | 4.10.0 | 待定（无 config） | 需自写 Find 模块或 imported target |
| Pluma | 未标注 | 待定（无 config） | 仅 `libpluma.so`；x64/arm64 均有 |
| ZLMediaKit | 未标注（git 4b07053） | 待定（无 config） | 仅 `libmk_api.so` + MediaServer |
| eventcpp | v0.1.3 | header-only | 实际头文件为 `eventpp/` |
| magic_enum | 0.9.8 | header-only | |
| nlohmann_json | v3.12.0 | header-only | 仅 Communication 使用 |
| OpenCV | 系统自带 | `find_package(OpenCV)` | 使用系统包（requirement §3） |

结论：

1. M0 需要为 BehaviorTree.CPP、Pluma、ZLMediaKit、eventcpp、magic_enum、nlohmann_json 提供统一的 Find 模块/interface target 封装，业务模块只链 `plane::xxx` 目标。
2. M0 需验证 MAVSDK 的系统依赖是否齐备；缺失时按 requirement 规则提示补充，不替换来源。
3. `3rdparty/common` 中带二进制的库基本只有 x64；arm64 按用户决定暂缓。

---

## 4. 目标工程结构（随里程碑逐步建立，不对已建结构做重构）

```text
onboard/
├── AGENTS.MD
├── CMakeLists.txt
├── cmake/                       （M0）三方库路径与 Find 封装
│   ├── Dependencies.cmake
│   └── Find*.cmake
├── config/                      （M0）运行配置（默认安全高度等）
├── docs/                        requirement / SPEC / ARCHITECTURE / DEV_PLAN
├── plugins/                     （M6）算法插件实现（编译为动态库，Pluma provider）
│   ├── detection_stub/
│   ├── tracking_stub/
│   └── guidance_stub/
└── src/
    ├── main.cpp                 （M0）
    ├── core/                    （M0）Core Runtime：生命周期、配置、日志、event bus
    ├── communication/           （M1）Zenoh 适配、序列化、上报调度
    ├── state/                   （M2/M9）UAV State Management + StateMonitor
    ├── task/                    （M4/M5）Task Scheduler + Task Execution Engine + BT 节点
    ├── flight_control/          （M3）MAVSDK/PX4 适配
    ├── camera/                  （M6）Camera Control
    ├── perception/              （M6）图像输入 + 插件调度 + Plugin Management
    ├── target/                  （M7）Target Management
    ├── video/                   （M8）Video Streaming / ZLMediaKit
    └── geo/                     （M5/M10）Geographic Tool（推迟，首个消费者时创建）
```

M0 只创建 `cmake/`、`config/`、`src/main.cpp`、`src/core/`；其余模块目录在对应里程碑创建，M0 用占位模块注册表打通生命周期。

业务模块只在各自目录内实现，依赖通过稳定能力接口或 eventcpp；同级模块不得交叉引入实现头文件（ARCHITECTURE §22）。

---

## 5. 里程碑总览

| 里程碑 | 目标 | 主要模块 | 前置 | 验证方式 |
|---|---|---|---|---|
| M0 | 工程骨架 + 依赖接入 + 日志/配置/事件总线 | Core Runtime | — | x64 Release 构建通过；程序启动输出 Build Info 后正常退出 |
| M1 | 网络通信基础 + 序列化 + ACK | Communication | M0 | Zenoh 与模拟 GCS 端收发报文；Message↔class 转换正确 |
| M2 | 状态管理与周期上报 | UAV State Management、Communication | M1 | 本机/友机/GCS 状态更新与查询；周期上报可见 |
| M3 | 飞控接入与连接管理 | Flight Control | M0 | PX4 SITL：串口/UDP 连接、2 秒超时识别、自动重连、基础动作 |
| M4 | 单一控制仲裁：指令与任务调度 | Task Scheduler | M2、M3 | SITL：指令派发、新控制内容立即替换、默认安全高度补入 |
| M5 | BehaviorTree 执行引擎 | Task Execution Engine | M4 | SITL：任务按步骤执行、状态与当前节点上报、可中止 |
| M6 | 相机、图像、插件框架与感知 | Camera Control、Perception、Plugin Management | M0、M2 | 加载示例算法插件；MIPI/RTSP 取图并调用插件 |
| M7 | 目标管理 | Target Management | M1、M6 | 视觉目标与 GCS 目标统一维护；GCS 目标 20 条历史 |
| M8 | RTSP 推流 | Video Streaming | M6 | 独立于 Perception 推流；`ffprobe` 可拉流 |
| M9 | 安全告警与系统参数下发 | StateMonitor、Communication | M2 | 模拟低电压/姿态异常产生告警并上报；安全高度可动态下发 |
| M10 | 自杀式反无任务全链路 + 第一版验收 | Task Execution Engine、Perception、Target、Flight Control | 全部 | SITL + 插件全链路演练；走完 SPEC §12 验收清单 |

依赖关系：

```text
M0 ──► M1 ──► M2 ──► M4 ──► M5 ──┐
 │              ▲      ▲          │
 ├────► M3 ─────┘      │          ▼
 │                     │        M10
 ├────► M6 ──► M7 ─────┘
 │       │
 │       └────► M8
 └────► M9（依赖 M2）
```

M3、M6、M9 与主线可并行；M10 必须最后。

---

## 6. 里程碑明细

### M0 工程骨架与依赖接入

**范围**

- CMake 工程：C++17、`-Wall -Wextra`、Release/Debug、`namespace plane`；`.clang-format` 落地到工程根。
- `cmake/Dependencies.cmake`：`PLANE_3RDPARTY_DIR`、`PLANE_COMMON_INCLUDE_DIR`；有 config 的库走 `find_package`，其余封装为 imported target。
- Core Runtime：初始化顺序（日志 → 配置 → 模块生命周期）、模块生命周期接口（init/start/stop/release）、构建版本信息输出。
- 日志封装（spdlog，滚动 50 MB × 5）、配置加载（`config/` 默认值，含安全高度）。
- `src/` 下建立 core 目录与占位模块注册表，打通生命周期；具体业务模块目录在对应里程碑创建。
- eventcpp 初始化推迟到 M1（首个消费者 Communication 出现时引入，避免无消费者的猜测性设计）。

**交付物**：可构建可运行的 `onboard` 可执行文件；`config/` 默认配置。

**验证**：x64 Release 构建通过；启动打印 Build Info + 模块初始化日志，正常退出；日志文件按配置滚动。

**风险**：MAVSDK 系统依赖缺失（M0 只验证 `find_package(MAVSDK)` 能否通过，不阻塞 M0）。

---

### M1 通信基础

**范围**

- Zenoh session 生命周期、配置（联通方式、路由参数）。
- 收发 API：`sendToStation`、`sendToPlane`、`sendToAll`（topic 常量取自 `message.h`）；优先单播、减少广播。
- 反序列化：`std::string` → `Message{messageId, type, payload}` → 按 `MessageType` 转 `nlohmann::json` → 内部 class → eventcpp 分发；JSON 不出 Communication。
- 可靠消息：ACK、超时、重试；实时状态不要求逐条确认。

**交付物**：Communication 模块；可独立运行的模拟 GCS 联调手段（脚本或最小可执行程序）。

**验证**：本机两个进程（机载与模拟 GCS）互发各 `MessageType`；Task/Command 消息具备 ACK；断线重连不崩溃。

**待定**：Zenoh 多播发现在目标网络不可用时需切换为单播/固定地址，M1 设计阶段决定。

---

### M2 状态管理与周期上报

**范围**

- UAV State Management：本机/友机/GCS 状态上下文（字段以 SPEC §4 与 `VehicleState` 为准）；同步查询 + 状态变化事件；最新有效状态权威。
- 周期上报：本机状态发往 GCS 与其他飞机；当前任务状态发往 GCS（无任务时上报「无任务」）；周期可配。
- 状态来源接入：M3 完成后接 Flight Control，M1 接通信。

**验证**：查询接口返回最新值；周期上报在模拟 GCS 端可见且周期可配；旧值不被使用。

---

### M3 飞控接入

**范围**

- Flight Control 适配层：MAVSDK 集成、串口与 UDP 连接参数、PX4 第一版。
- 连接生命周期：连接建立/正常/异常/恢复；连续 2 秒无有效数据判异常并自动重连；状态发布至 eventcpp 并进入状态管理。
- 飞控能力：Arm、Disarm、Takeoff、Land、RTL、Waypoint、Route、Hover、StopMotor（仅 GCS 人工触发）、NED Velocity（仅机载内部）。
- 飞控错误统一转换（ARCHITECTURE §26）。

**验证**：PX4 SITL 下完成串口与 UDP 连接；人为断流验证 2 秒判定与重连；起飞/降落/返航/航点动作成功。

**风险**：MAVSDK 系统依赖未齐备；SITL 环境需确认。

---

### M4 单一控制仲裁

**范围**

- Task Scheduler 统一管理 Task 与独立 Command：任意时刻最多一个控制内容，不排队，新内容立即替换。
- GCS 独立 Command 经仲裁后派发 Flight Control；Task 内部步骤产生的 Command 直达 Flight Control。
- Task 生命周期与状态（None/Pending/InProgress/Completed/Aborted/Failed）、对外提供当前任务状态。
- 未指定高度的 Takeoff 补入默认安全高度。

**验证**：SITL 下发 Arm/Takeoff/Waypoint；执行中下发新指令，旧内容立即终止；上报状态正确。

---

### M5 BehaviorTree 执行引擎

**范围**

- Task Execution Engine：Task 类型 → 执行策略映射；创建/执行 BehaviorTree；节点调用 Flight Control / State / Perception / Target。
- 基础指令节点库（Arm、Takeoff、Land、RTL、Waypoint、Route、Hover、StopMotor）与完成判定（以实际 UAV 状态判定，不以发送成功为准）。
- 行为树与节点状态上报（`TaskState.currentBehaviorTree` / `currentNode`）；任务中止与替换。

**验证**：SITL 执行由基础指令组成的任务；前一步完成才进入下一步；失败进入 Failed；任务状态上报含行为树与节点。

---

### M6 相机、图像与插件框架

**范围**

- Camera Control：链路建联、状态获取、控制指令下发。
- Perception：MIPI/RTSP 输入、最新帧模型（无历史缓存）、按任务阶段调度 Detection/Tracking/Guidance、结果经 eventcpp 发布。
- Plugin Management（Pluma）：插件发现、库加载、对象按需实例化、释放。
- `plugins/` 下按 `functional/*.h` 接口实现可加载的示例/占位插件，用于打通加载链路（算法本体不在本项目）。

**验证**：插件库可被加载并实例化；RTSP 输入源取图成功；调用检测/跟踪/导引接口返回结果。

---

### M7 目标管理

**范围**

- 两类目标统一维护：机载感知目标（仅最新）与 GCS 目标（最新 + 每目标 20 条历史，ADR-015）。
- 查询能力供 Task Execution Engine 使用；视觉目标只维护图像位置，不转 WGS84，不做轨迹平滑；不接雷达。

**验证**：检测结果与 GCS 目标均可查询与更新；GCS 目标历史条数上限正确。

---

### M8 RTSP 推流

**范围**

- Video Streaming：接收 Perception 的图像帧与叠加信息、编码、封装、ZLMediaKit 集成、RTSP 输出；与 RTSP 输入链路独立。
- `libmk_api.so` 链接与运行期部署（RPATH/`LD_LIBRARY_PATH`）。
- 抑制或接管 ZLMediaKit 自身的默认日志输出（M0 观察：其静态初始化会向 stdout 打印 codec 注册日志），避免污染统一日志。

**验证**：`ffprobe` 拉流成功；停止推流不影响 Perception。

---

### M9 安全告警与系统参数

**范围**

- StateMonitor：低电压、姿态异常监控（阈值可配）；仅告警，不自动中止/返航/降落/接管。
- Alert 经 Communication 上报 GCS。
- 系统参数：安全高度启动时加载默认值，运行中接受 GCS 下发更新。

**验证**：注入异常值产生告警并在模拟 GCS 可见；动态下发安全高度后，下一次未指定高度的起飞使用新值。

---

### M10 自杀式反无任务与第一版验收

**范围**

- 反无行为树：Arm → Takeoff → 中段制导（按 GCS 目标 WGS84 接近）→ 检测（选定目标）→ 单目标跟踪（丢失恢复/失败条件）→ 导引撞击（Guidance 输出 → NED Velocity 控制）。
- 阶段切换条件与失败判定按任务参数/配置；跟踪丢失无法恢复则任务失败。
- 端到端联调：GCS 模拟端 + SITL + 示例插件。
- 走查 SPEC §12 验收清单，逐项记录结果；部署打包（可执行 + 动态库 + 配置）。

**验证**：全链路演练通过；执行中收到新控制内容立即中止并按最新内容执行。

---

## 7. Change 拆分建议

每个里程碑作为独立 change 推进；change-id 建议：

| change-id | 对应里程碑 |
|---|---|
| `m0-project-skeleton` | M0 |
| `m1-communication` | M1 |
| `m2-uav-state` | M2 |
| `m3-flight-control` | M3 |
| `m4-task-scheduler` | M4 |
| `m5-behaviortree-engine` | M5 |
| `m6-perception-plugins` | M6 |
| `m7-target-management` | M7 |
| `m8-video-streaming` | M8 |
| `m9-state-monitor` | M9 |
| `m10-suicide-mission` | M10 |

工作流（AGENTS.md）：每个 change 先出 `design.md` 并确认，再出 `tasks.md`，任务级计划细化到文件与步骤；不跳过、不重定义任务。当前选择单文档路线图，是否新建 OpenSpec 目录结构由用户后续决定。

---

## 8. SPEC 覆盖矩阵

| SPEC 章节 | 覆盖里程碑 |
|---|---|
| §1 定位与边界 | M0/M10（边界由架构承接） |
| §2 运行环境 | M0 |
| §3 飞控 | M3 |
| §4 状态与上报 | M2、M5（任务状态上报） |
| §5 指令与任务 | M4、M5 |
| §6 自杀式反无 | M10（依赖 M5–M9） |
| §7 安全高度 | M4、M9 |
| §8 相机/图像/AI/目标 | M6、M7、M8 |
| §9 通信 | M1、M2、M9 |
| §10 安全告警 | M9 |
| §11 非功能要求 | 全局约束 |
| §12 验收清单 | M10 |
| §13 非目标范围 | 不在任何里程碑引入 |
| §14 业务模型 | 由 M4/M5/M10 落地 |

---

## 9. 风险与待确认

| # | 事项 | 影响 | 处理 |
|---|---|---|---|
| 1 | `3rdparty/common` 缺 arm64 库（仅 pluma 有） | arm64/RK3588 构建不可用 | 用户确认暂不处理；后续需补齐清单后再接 |
| 2 | MAVSDK 系统依赖（CURL、jsoncpp、picosha2、tinyxml2、mav、LibEvents、LibLZMA）不在 3rdparty | M3 无法链接 | M0 验证 `find_package(MAVSDK)`；缺失时按规则提示用户补充 |
| 3 | BehaviorTree.CPP、Pluma、ZLMediaKit、eventcpp 无 CMake config | M0 依赖封装 | 自写 Find 模块/imported target，统一别名 `plane::xxx` |
| 4 | Pluma 无版本号；ZLMediaKit 版本为 git 短哈希 | 兼容性不可追溯 | 记录实际使用目录/哈希；升级时回归 M6/M8 |
| 5 | eventcpp 目录实际头文件为 `eventpp`（v0.1.3） | 命名差异 | 代码统一 `#include <eventpp/...>`；文档措辞后续统一时再确认 |
| 6 | Zenoh 发现方式（多播）在机载网络可能不可用 | M1/M2 通信不稳定 | M1 设计阶段确定单播/固定路由配置 |
| 7 | PX4 SITL 与 MAVSDK 联调环境未验证 | M3–M5、M10 无法验证 | 开工 M3 前确认 SITL 可运行（x64） |
| 8 | 不写测试代码，回归依赖手工联调 | 后续里程碑回归成本高 | 每个里程碑保留可重复的联调脚本/模拟端（复用，不视为测试代码） |
| 9 | OpenCV 使用系统包，插件 ABI 需与算法插件提供方一致 | 插件加载失败风险 | M6 前确认系统 OpenCV 版本并通知插件方 |

---

## 10. 里程碑完成定义（DoD）

每个里程碑必须同时满足：

1. x64 Release 构建通过，无新增警告级别错误；
2. §6 中该里程碑的验证点通过（可重复执行）；
3. 未引入 SPEC §13 非目标范围的内容；
4. 未偏离 ARCHITECTURE 的模块边界与依赖方向；如偏离先改文档并确认；
5. 若产生文档变化，按 AGENTS.md 提醒用户确认后更新。

---

## 11. 计划文档索引

| 文档 | 内容 | 状态 |
|---|---|---|
| `plans/m0-project-skeleton.md` | M0 任务级计划与执行记录 | 已完成（d48569f..94bdad3） |
| `plans/m1-communication-design.md` | M1 Change Design（通信基础） | 已确认并实施 |
| `plans/m1-communication-tasks.md` | M1 任务级计划与执行记录 | 已完成并通过验收 |
| `plans/m2-uav-state-design.md` | M2 Change Design（UAV 状态管理与周期上报） | 已确认并实施 |
| `plans/m2-uav-state-tasks.md` | M2 任务级计划与执行记录 | 已完成并通过验收 |
| `plans/m3-flight-control-design.md` | M3 Change Design（飞控接入） | 已确认（逐项） |
| `plans/m3-flight-control-tasks.md` | M3 任务级计划 | 待执行 |

约定：

1. 每个里程碑先生成 design 文档并确认，再生成 tasks 文档执行；
2. 任务执行的偏差与裁决记录在对应里程碑文档中；
3. 本文件只保留路线图、全局约束、依赖基线、风险与 DoD，不再追加任务细节。

---

## 12. 后续

M1 任务见 `plans/m1-communication-tasks.md`；M0 完成后按 §11 索引继续 M2 及后续。
