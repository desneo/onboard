# plane

> 本文面向项目使用者，说明项目背景、当前实现、构建运行方法和配置。目标需求见
> [`docs/SPEC.md`](docs/SPEC.md)，实现约束见
> [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)，测试方法见
> [`docs/TESTING.md`](docs/TESTING.md)。

## 项目背景

`plane` 是运行在无人机机载计算平台上的 C++17 应用，面向基于 PX4 的反无人机任务。程序连接飞控、地面站、其他无人机和相机控制接口，在单进程内完成状态管理、任务仲裁和行为树执行。本版本不启动图像处理、视觉插件和视频推流。

支持 Ubuntu 22.04 的 x86_64 与 ARM64，首要硬件平台为 RK3588。

## 核心功能场景

以下场景描述项目的目标用途；实际完成情况以“当前能力”和“当前已知限制”为准。

### 任务范围

本版本完全移除本地视觉处理。`AntiUav` 任务恢复中制导阶段，但不恢复视觉／末制导；任务持续运行，
直到收到 Task Stop、新控制内容或进程退出。

### 单机飞行控制

地面站可以下发解锁、上锁、起飞、降落、返航、航点飞行、航线飞行、悬停和停桨等独立指令。航点和航线既可作为独立指令，也可作为复杂任务中的执行步骤。

### 最新控制内容优先

系统不维护任务或指令等待队列，任意时刻只执行一个 Task 或 Command。收到新的控制内容时，中止当前执行并立即切换到最新内容。

### 相机／吊舱控制

保留相机控制接口，用于发送吊舱指令及获取状态，不进行图像采集、处理或视频转发。当前实现仍为 Noop，未接入具体吊舱协议。

### 地面站与机间通信

系统通过 Zenoh 与地面站及其他无人机交换业务消息，支持向地面站、指定无人机和其他节点发送消息，并周期上报本机状态与当前任务状态。

## 当前能力

- 通过 UDP 或串口连接 PX4，采集位置、姿态、速度、电压和飞行模式。
- 执行解锁、上锁、起飞、降落、返航、悬停、停桨、航点和航线指令。
- 基于 Zenoh 接收地面站任务、指令、系统参数和目标信息。
- 周期广播本机状态，接收并缓存其他无人机的最新状态；位置更新不逐条打印，通过 Zenoh liveliness 仅记录上下线。
- 通过 BehaviorTree 仲裁和执行单个 Task 或 Command，新控制内容替换当前内容。
- 通过 common 公共接口直接链接预编译 `libmidcourse_alg_3588.so` 执行反无人机中制导，不使用 Pluma 动态插件加载。
- 飞控连接建立后预读并缓存四项 PX4 飞控系统参数；中制导会话只读取通用飞控参数缓存，不在任务路径查询 PX4 参数。
- 配套 `gcs_sim` 监听 UDP 7790，将雷达 confirmed 航迹与 Zenoh 友机状态匹配，过滤友机后发布靶机。
- 中制导输出经过校验和四元数转换后使用 MAVSDK Attitude Offboard，并按 50 ms 重发设定值。
- 本版本已停用图像采集、检测/MOT 和视频推流。
- 监测实机低电压并向地面站告警。
- 已删除检测节点及原有视觉任务流程，相关任务启动时返回不可用。

当前已知限制：

- 反无人机任务只实现中制导，不包含末端交接确认、视觉跟踪或末段导引撞击。
- 相机控制模块目前是 Noop，不控制真实相机或云台。
- 友机状态参与 GCS 雷达航迹过滤，但尚未参与编队、避碰或其他协同决策。

## 目录概览

```text
src/                  机载程序源码
config/plane.conf     默认运行配置
tests/                契约测试、集成测试及测试配置
scripts/              源码树中的程序启停脚本
tools/                打包和辅助程序
cmake/                CMake 依赖配置
docs/                 需求、架构和测试说明
```

完整目录职责见 [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)。

## 外部依赖

- 公共数据模型与插件接口：`../common/common_include`
- 中制导发布包：`../algorithm/midcourse_alg/midcourse_alg-0.1.0-aarch64`
- 预编译第三方库：`../../../3rdparty/common`
- 历史视觉插件和模型本版本不参与打包或运行。

中制导以 imported target 直接链接发布包中的 `libmidcourse_alg_3588.so`；该库不设置 SONAME，
运行包按原文件名分发。编译仅引用 `common_include/algorithm/midcourse_alg.h`，最终程序不得依赖
`libpluma`。主要依赖还包括 MAVSDK、Zenoh、
BehaviorTree.CPP、eventpp、GeographicLib、spdlog、fmt、nlohmann-json 和 magic_enum。

依赖缺失时应补充对应本地库，项目不会在线下载依赖。

## 构建与打包

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

默认构建会生成可独立运行的 `build/plane/`，包含主程序、中制导库及运行库；本版本不暂存图像插件、
模型、检测配置、Pluma 或 RKNN 库。

已有构建目录也可以手动重新打包：

```bash
./tools/package.sh build
```

打包脚本会收集 `/lib`、`/usr/lib` 之外的项目/三方动态库。系统库仍需由目标机提供。

## 运行与停止

### 部署独立运行目录

`build/plane/` 包含本说明、主程序、配置、启停脚本和项目动态库。本版本启动不需要模型或图像插件。在目录根部执行：

```bash
cd /path/to/plane

# 确认目标机架构正确、关键文件齐全，并检查随包动态库解析。
uname -m
test -x plane && test -f config/plane.conf && test -d lib
! ldd ./plane | grep 'not found'
./plane --deps-check
```

部署前按现场环境修改 `config/plane.conf`，至少确认 `[flight_control]` 中的连接类型及对应 UDP 或串口参数和 Zenoh 端点。图像相关配置已删除。

本版本启动脚本不检查 RKNN 模型，也不修改图像尺寸配置。交互式启动：

```bash
./runplane.sh
```

查看启动用法可执行 `./runplane.sh --help`。启动脚本会自动为 `plane` 和已有的 `stopplane.sh` 补齐执行权限。
若上传工具丢失了启动脚本自身的执行权限，可使用 `bash ./runplane.sh` 启动。

无人值守后台启动及状态检查：

```bash
mkdir -p logs
nohup ./runplane.sh >logs/plane.stdout.log 2>&1 &
pgrep -a -x plane
tail -f logs/plane.log
```

主日志固定写入 `logs/plane.log`，重启后追加；单文件达到 50 MB 后按
`plane.1.log`、`plane.2.log` 至 `plane.5.log` 轮转（序号越小越新，超过 5 份淘汰最旧文件）。
项目人工编写的运行日志和业务描述均使用中文；日志级别保留 spdlog 的 `[info]`、`[warn]`、
`[error]` 等原始英文标识，线程 ID 直接显示为数字方括号（如 `[701742]`）。API、协议、类型、枚举、
配置键、函数参数名、结构体及其字段名和第三方库
原始错误等代码标识保留原文。中制导日志使用 `接口名 输入/输出` 的简洁前缀，结构体字段保持公共接口
原名；经纬度保留 6 位小数，速度保留 2 位小数。接口输入输出不重复打印会话号、调用号和更新号；
PX4 参数使用 `MPC_XY_VEL_MAX`、`MPC_Z_VEL_MAX_UP`、`MPC_Z_VEL_MAX_DN`、`MPC_ACC_HOR_MAX` 原名；
日志保留会话生命周期统计、20 帧初始化历史、完整配置、目标估计及每个 50 ms 规划周期的原始控制输出，
可按日志时间顺序还原调用流程。
多机部署同样使用此目录，不再为每次启动创建日期目录。
`plane.stdout.log` 单独记录标准输出和错误输出，包含重复的程序日志以及第三方库、插件和启动脚本输出；
它不参与主日志轮转，后台启动时覆盖。

停止程序（默认等待 10 秒优雅退出）：

```bash
./stopplane.sh
```

若启动失败，先查看 `logs/plane.stdout.log` 和 `logs/plane.log`，并再次运行 `ldd ./plane` 检查运行库是否缺失。

### 从源码树运行

从源码树启动：

```bash
./scripts/runplane.sh
```

或指定独立运行目录：

```bash
./scripts/runplane.sh /path/to/plane_dir
```

启动脚本会补齐执行权限后运行 `plane`，不再检查图像模型或调用 `rknn-inspect`。

停止程序：

```bash
./scripts/stopplane.sh
```

打包后两个脚本仍位于独立运行目录根部，可在 `build/plane/` 中使用 `./runplane.sh` 和 `./stopplane.sh`。

直接运行及依赖检查：

```bash
cd build/plane
./plane
./plane --deps-check
```

程序从当前工作目录读取 `config/plane.conf`，因此直接运行时应位于独立运行目录。

## 配置说明

默认配置为 [`config/plane.conf`](config/plane.conf)，采用带 section 的 INI 格式。配置项必须按模块放在 `[section]` 下，使用 `key = value`。

### 系统与通信

| 配置 | 说明 |
| --- | --- |
| `system.safe_height_m` | 未指定起飞高度时使用的安全高度 |
| `system.log_dir` | 日志目录 |
| `system.board_ip` | VehicleInfo 中的任务板 IP；fleet 部署在未配置时填入板卡 IP |
| `camera.pod_video_url` | VehicleInfo 中的吊舱视频流地址，未配置时为空，仅传递地址 |
| `system.state_report_period_ms` | 状态上报周期 |
| `communication.plane_id` / `station_id` | 本机和地面站标识 |
| `communication.zenoh_config_file` | 可选 Zenoh 配置文件 |
| `communication.zenoh_connect` / `zenoh_listen` | Zenoh 连接或监听端点 |
| `communication.ack_timeout_ms` / `ack_max_retries` | 可靠消息确认超时和重试次数 |
| `communication.rx_queue_max` | 通信接收队列容量，满时丢弃最旧消息 |
| `communication.gcs_link_timeout_ms` | 地面站链路超时阈值 |
| `communication.debug_log` | 是否记录 Zenoh 消息目标端点 |

### 飞控与任务

| 配置 | 说明 |
| --- | --- |
| `flight_control.link_type` | `udp` 或 `serial` |
| `flight_control.udp_url` | MAVSDK UDP 地址 |
| `flight_control.serial_device` / `serial_baud` | 串口设备和波特率 |
| `flight_control.heartbeat_timeout_ms` | MAVSDK 心跳超时 |
| `flight_control.route_speed_m_s` | 航点、航线以及航线开始前临时高度航点的速度 |
| `behavior_tree.tick_interval_ms` | 行为树 tick 周期 |
| `behavior_tree.node_timeout_ms` | 普通行为树节点超时；Route 航线飞行不受该时限限制，持续等待飞控任务进度完成 |
| `behavior_tree.takeoff_tolerance_m` | 起飞高度判定容差 |
| `behavior_tree.height_adjust_timeout_ms` | 航线开始前调整相对高度的超时 |
| `behavior_tree.height_adjust_tolerance_m` | 航线开始前调整相对高度的到达容差 |
| `midcourse.init_timeout_ms` | 等待严格递增的 20 帧雷达历史的最长时间，默认 30000 ms |
| `midcourse.target_invalid_grace_s` | 算法内部目标预测宽限期，默认 3 秒 |
| `midcourse.r_min_m` | 中制导距离保护，默认 50 米；与 GCS 友机匹配门限相互独立 |

UDP 飞控端口位于 `14580`～`14589`（`1458*`） 时，程序将本机视为仿真飞机，并禁用低电压告警。

### 安全

| 配置 | 说明 |
| --- | --- |
| `safety.alert_voltage_threshold_v` | 实机低电压告警阈值 |

配置文件修改后重启程序生效；`SystemParam` 消息可在运行中更新安全高度。

## 机载信息上报与查询

共享消息数据和通信地址模板定义位于 `../common/common_include/message.h`：
`topic_queryToPlane = "queryToPlane/{planeId}/{MessageType}"`，
`topic_stationLiveliness = "station/liveliness/{stationId}"`。
查询接口及实现位于本工程 `src/communication`，其他工程可以依据公共地址模板和消息定义接入。
`VehicleInfo` 的 JSON 载荷为 `{"planeId":"uav1","boardIp":"192.168.1.100","podVideoUrl":"","isSim":true}`，
仍放在标准 Message 信封的 payload 字符串内，type 为 VehicleInfo。
机载启动后发送一次，地面站上线／重连时再次发送，不周期广播。
`isSim` 为 JSON 布尔值，直接使用现有仿真判定：UDP 端口 `14580`～`14589` 时为 true，其他情况为 false。
该字段仅加入 VehicleInfo；读取缺少该字段的旧 VehicleInfo 时按 false 处理。

地面站先订阅 `sendToStation`，再声明 Zenoh 存活标记 `station/liveliness/<station_id>`。
主动查询使用原生 `get("queryToPlane/<planeId>/VehicleInfo", "", ...)`，返回同样的 Message 信封；
机载 CommunicationApi 直接暴露 queryPlane 接口，目前只支持 VehicleInfo。
旧地面站要接收新信息，需支持新消息类型和地面站存活标记。

使用配套工具查询（工具先订阅再声明地面站在线）：

```bash
./build/gcs_sim --plane-id uav1 --sender-id GCS --query VehicleInfo
```

可用 `--connect tcp/<host>:<port>` 指定 Zenoh 端点，`--query-timeout-ms 2000` 设置超时；
只查询、不声明地面站上线时追加 `--no-presence`。
查询失败、类型不支持或没有响应时，工具返回非零退出码。

## 雷达入口与中制导联调

`gcs_sim` 默认监听 `0.0.0.0:7790`，只接收 `type=radar_tracks` 的 UDP JSON；`target_truth`
及其他类型被忽略。防守飞机坐标不由 7790 提供，而是从 Zenoh `VehicleState` 获取。GCS 使用
扫描时间外推友机位置，对 50 米内的 confirmed 雷达航迹做全局一对一匹配并过滤；友机没有出现在
某次雷达扫描中不会阻塞靶机，但友机状态缺失或过期时整帧暂停转发。

手动启动 `192.168.1.56:14580` 的 PX4 SITL 后，可执行：

```bash
python3 tools/fleetctl.py test-midcourse --count 1 --target-track-id <track_id>
```

工具部署到 `uav1`，监听 `192.168.1.100:7790`。机载程序从启动起持续缓存每个航迹最近 20 帧，
中制导会话直接使用已有历史，不从任务启动时重新计数；测试继续发送第 21 帧并验证增量更新、
Attitude Offboard、持续运行和 Task Stop 清理。目标 ID 留空时，机载只在当前候选唯一时自动锁定。

## 测试

构建检查、自动化脚本、硬件环境和人工验收步骤见 [`docs/TESTING.md`](docs/TESTING.md)。
