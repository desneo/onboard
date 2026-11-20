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
机载支持 `SelfCheck` 起飞前自检，确认实际解锁后恢复上锁，并逐机回报执行结果。可用 `gcs_sim` 发令和查看结果。

### 最新控制内容优先

系统不维护任务或指令等待队列，任意时刻只执行一个 Task 或 Command。收到新的控制内容时，中止当前执行并立即切换到最新内容。
自检仅在空闲时启动，自检期间人工接管、显式上锁和停桨指令可以中止它。

人工接管优先于所有自动任务和指令。执行中，PX4 切入 `Position`、`Stabilized` 或非预期的 `Loiter` 时，机载立即停止旧执行的控制下发并通知地面站。飞手随后拨回原档位，旧执行也不会恢复。

### 相机／吊舱控制

机载通过 UDP 接入艾晟 ASD-2、中科 D80 和捷联 A2416A 仿真吊舱。艾晟和中科支持双轴连续转动、单轴归零和指定机械角，固定光轴的捷联支持状态上报。机载向 `sendToStation` 发布 `PodState`，默认 5 Hz，`fields` 提供中文展示键和值。视频流地址随 `AgentInfo` 发布。

### 地面站与机间通信

系统通过 Zenoh 与地面站及其他无人机交换业务消息，支持向地面站、指定无人机和其他节点发送消息，并周期上报本机状态与当前任务状态。

## 当前能力

- 通过 UDP 或串口连接 PX4，采集位置、姿态、速度、电压和飞行模式。
- 执行解锁、上锁、起飞、降落、返航、悬停、停桨、航点和航线指令。
- 基于 Zenoh 接收地面站任务、指令、系统参数和目标信息。
- 周期广播本机状态，接收并缓存其他无人机的最新状态；位置更新不逐条打印，通过 Zenoh liveliness 仅记录上下线。
- 调度器按控制优先级仲裁单个 Task 或 Command，执行引擎通过 BehaviorTree 执行。
- 中制导控制器使用 common 公共类型直接调用 `libmidcourse_alg.so` 中的目标估计器和规划器。
- 中制导规划限值默认来自 `[midcourse_algorithm]`；设置 `use_px4_limits = true` 后，会话使用飞控在连接阶段读取的四项 PX4 缓存值。
- 配套 `gcs_sim` 监听 UDP 7790，将雷达 confirmed 航迹与 Zenoh 友机状态匹配，过滤友机后发布靶机。
- 中制导默认下发 NED 速度、加速度前馈和航向，也支持 Attitude Offboard 的姿态与推力设定值。
- 本版本已停用图像采集、检测/MOT 和视频推流。
- 监测实机低电压并向地面站告警。
- 已删除检测节点及原有视觉任务流程，相关任务启动时返回不可用。

当前已知限制：

- 反无人机任务只实现中制导，不包含末端交接确认、视觉跟踪或末段导引撞击。
- 吊舱控制链路支持仿真 UDP。实体吊舱使用的 RK3588 串口接入由 `PodTransport` 扩展。
- 友机状态参与 GCS 雷达航迹过滤，但尚未参与编队、避碰或其他协同决策。

## 目录概览

```text
src/                  机载程序源码
config/plane.ini      默认运行配置
config/profiles/      机群部署预置
tests/                测试代码及环境清单
scripts/              机载启停和自启部署脚本
tools/                打包和辅助程序
cmake/                CMake 依赖配置
docs/                 需求、架构和测试说明
```

完整目录职责见 [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)。

## 外部依赖

- 公共数据模型与插件接口：`../common/common_include`
- 中制导 AArch64 动态库：`../algorithm/midcourse_alg/build/libmidcourse_alg.so`
- 预编译第三方库：`../../../3rdparty/common`
- 历史视觉插件和模型本版本不参与打包或运行。

中制导以 imported target 直接链接 `libmidcourse_alg.so`，运行包按其 SONAME 分发。
算法库与机载需使用同一版本的 `common_include/algorithm/midcourse_alg.h`；公共结构体变更后需重建算法库。
如通过 `PLANE_MIDCOURSE_ALG_LIBRARY` 指定库路径，运行包使用该路径下的库。最终程序不得依赖
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

`build/plane/` 包含本说明、主程序、配置、启停与自启部署脚本和项目动态库。本版本启动不需要模型或图像插件。在目录根部执行：

```bash
cd /path/to/plane

# 确认目标机架构正确、关键文件齐全，并检查随包动态库解析。
uname -m
test -x plane && test -f config/plane.ini && test -d lib
! ldd ./plane | grep 'not found'
./plane --deps-check
```

部署前在 `config/plane.ini` 选择现场预置并确认 Zenoh 端点。将同一份运行目录复制到机群的每块板卡。
升级已有单机部署时，先备份 `config/plane.conf`，再将现场参数迁入 `config/plane.ini` 并清空 `[deployment]` 的 `profile`。

### 安装开机自启服务

在 Ubuntu 22.04 板卡上，从独立运行目录执行以下命令。先用 `stopplane.sh` 停止已有的手动进程，再用部署账号安装服务。

```bash
cd /home/firefly/plane
./stopplane.sh
sudo ./install-plane-service.sh install
systemctl status plane.service
tail -F logs/plane.log
```

安装启动 `plane.service` 并启用开机自启。服务以调用 `sudo` 的部署账号运行，直接以 root 执行安装脚本时使用运行目录的属主账号。服务最早在系统开机满 10 秒时启动。后台运行时应用日志写入 `logs/plane.log`，启动脚本和第三方库的标准输出、错误可用 `sudo journalctl -u plane.service -f` 查看。

使用 `./runplane.sh --service [plane_dir]` 启动后台服务，使用 `./stopplane.sh` 停止。主动停止会取消已排队的自动重启，保留下次开机自启。服务模式的优雅停机超时为 10 秒，手动模式可通过 `./stopplane.sh [超时秒数]` 指定。

异常退出后等待 5 秒重启。60 秒内最多启动 5 次，超过限额进入 `failed`。排除故障后执行 `sudo systemctl reset-failed plane.service`，再执行 `./runplane.sh --service`。

安装脚本通过 `sysctl.d` 设置板卡的全局 `kernel.core_pattern`，内核直接写入运行目录下的 `logs/core/core.<进程名>.<PID>.<Unix 时间戳>`。core 全部保留，由部署人员清理。板卡上其他可生成 core 的进程也使用该路由，写入需要目标目录权限。遇到外部 core 处理器或会覆盖该规则的配置时，安装脚本会报告冲突。

更新服务时，先停止现有服务，部署运行包后重新执行安装命令。执行 `sudo ./install-plane-service.sh uninstall` 可停止并卸载服务，移除它管理的全局 core 规则。当前路由仍由该规则管理时，卸载恢复安装前的路由。卸载保留配置、日志和历史 core，包括已有的 `logs/cores/` 文件。

### 手动运行

服务正在运行、启动或等待重启时，先执行 `./stopplane.sh`，再在终端启动前台进程。

```bash
./runplane.sh
```

前台运行时，应用日志同时输出到终端和日志文件。按 `Ctrl+C` 退出，再用 `./runplane.sh --service` 恢复后台服务。
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
原始错误等代码标识保留原文。PX4 参数使用 `MPC_XY_VEL_MAX`、`MPC_Z_VEL_MAX_UP`、
`MPC_Z_VEL_MAX_DN`、`MPC_ACC_HOR_MAX` 原名。中制导在调用 `setConfig` 前记录限值来源和 `MidCourseConfig` 全部字段的实际值，
并记录初始化历史帧数和周期、首次目标增量更新以及 Offboard 启动。
中制导算法的 `MidcoursePlanner::setConfig`、`TargetEstimator::init/update` 和 `MidcoursePlanner::update`
每次调用均以 `[info]` 输出完整入参、返回值、有效出参和原始 `error`，包括初始化全部历史量测、
本机与目标位置速度、速度加速度指令或姿态四元数与推力，以及 `eta_seconds`、`following` 和 `terminal_confirmed`。
`yaw_rad` 为空时显示 `nullopt`；调用失败时显示无有效输出。位置和算法字段保持公共接口原名。
多机部署同样使用此目录，不再为每次启动创建日期目录。
飞控 `STATUSTEXT` 按 `[info]` 输出，格式为 `飞控 STATUSTEXT：type=<类型> text=<飞控原文>`，保留 MAVSDK 的 `type` 和 `text`。
`plane.stdout.log` 单独记录标准输出和错误输出，包含重复的程序日志以及第三方库、插件和启动脚本输出；
它不参与主日志轮转，后台启动时覆盖。

停止程序（默认等待 10 秒优雅退出）：

```bash
./stopplane.sh
```

若启动失败，查看终端输出、`logs/plane.log` 或手动后台运行时的 `logs/plane.stdout.log`，并运行 `ldd ./plane` 检查运行库。

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

打包后启停与自启部署脚本均位于独立运行目录根部。

直接运行及依赖检查：

```bash
cd build/plane
./plane
./plane --deps-check
```

程序从当前工作目录读取 `config/plane.ini`，因此直接运行时应位于独立运行目录。

## 配置说明

配置项按模块放在 `[section]` 下，采用 `key = value` 格式。默认配置见 [`config/plane.ini`](config/plane.ini)。

### 机群部署预置

在 `config/plane.ini` 中设置 `[deployment]` 的 `profile`，相对路径以运行目录为基准。默认使用捷联 A2416A 仿真预置。

```ini
[deployment]
profile = config/profiles/sitl_jielian_a2416a.ini
```

预置使用飞机 ID 作为飞机节名，`board_ip` 和 `fc_url` 必填。UDP 飞控可填写 `udpout://192.168.1.56:14280`，串口可填写 `serial:///dev/ttyS0:57600`。以下为默认预置中的地面站和一号机。

```ini
[groundstation]
ip = 192.168.1.41

[uav1]
board_ip = 192.168.1.100
fc_url = udpout://192.168.1.56:14280
pod_model = jielian_a2416a
pod_video_url = rtsp://192.168.1.56:8554/jielian_a2416a
pod_control_url = udp://192.168.1.56:8001
```

启动时，程序用已启用的非回环网卡 IPv4 地址匹配 `board_ip`。预置须有唯一匹配，飞机 ID 和板卡 IP 在文件内也须唯一。配置无法读取、必填项无效或匹配失败时，程序退出并记录原因。
命中的节提供本机身份、飞控连接和吊舱配置，`pod_video_url` 随 `AgentInfo` 上报。
`pod_model`、`pod_video_url` 和 `pod_control_url` 均可留空。

程序在本机 `board_ip` 的 TCP `7447` 端口监听，主动连接 `groundstation.ip` 和按 IPv4 数值升序排列的所有其他飞机。
这些端点覆盖主配置中的 `zenoh_listen` 和 `zenoh_connect`。`groundstation.ip` 可省略或留空，填写时须为单播 IPv4，且与所有板卡 IP 不同。
同一机群的 `board_ip` 应是节点之间可达的地址。地面站使用现有的 `tcp/0.0.0.0:7447` 监听。

随包预置文件按 `sitl_<pod_model>.ini` 命名，可选 `jielian_a2416a`、`two_axis`、`aisheng_asd2` 和 `zhongke_d80`。
仿真预置的地面站为 `192.168.1.41`，`real_test.ini` 的地面站为 `192.168.1.200`。
部署客户机群时，将对应 INI 放入运行目录的 `config/profiles/` 并修改 `profile`。单机直配时清空 `profile`，在主配置中填写板卡 IP、飞机 ID、飞控连接和吊舱配置。

### 系统与通信

| 配置 | 说明 |
| --- | --- |
| `system.safe_height_m` | 未指定起飞高度时使用的安全高度 |
| `system.log_dir` | 日志目录 |
| `system.board_ip` | `AgentInfo.boardIp`，fleet 部署在未配置时填入板卡 IP |
| `camera.pod_model` / `pod_control_url` | 单机直配的吊舱型号和控制地址 |
| `camera.pod_state_report_hz` | `PodState` 上报频率，默认 5 Hz，单机直配和机群预置共用 |
| `camera.pod_video_url` | `AgentInfo.videoUrl`，未配置时为空 |
| `system.state_report_period_ms` | 状态上报周期 |
| `communication.plane_id` | 本机标识 |
| `communication.zenoh_config_file` | Zenoh 原生配置文件，默认 `config/zenoh.json5` |
| `communication.zenoh_connect` / `zenoh_listen` | 单机直配的 Zenoh 连接或监听端点，留空沿用 Zenoh 原生配置 |
| `communication.ack_timeout_ms` / `ack_max_retries` | 可靠消息确认超时和重试次数 |
| `communication.rx_queue_max` | 通信接收队列容量，满时丢弃最旧消息 |
| `communication.gcs_link_timeout_ms` | 地面站链路超时阈值 |
| `communication.debug_log` | 是否记录 Zenoh 消息目标端点 |

Zenoh 使用 peer 模式。机群预置通过 TCP 单播建链，组播发现和 gossip 由 Zenoh 原生配置控制，默认开启。
`config/zenoh.json5` 将 `transport/unicast/open_timeout` 设为 `2000` 毫秒，限制每次主动建链的等待时间；
连接失败后仍按默认策略重试。该值不修改已建立连接的在线判定或业务消息确认超时。
从运行目录根部启动时加载此文件；清空 `zenoh_config_file` 则使用 Zenoh 库的默认配置。
SITL 测试统一使用 `tests/fleet.ini` 的 `simulator.host`（当前为 `192.168.1.56`）。
测试部署会校正板端旧的 SITL IP，保留现有 `14580`～`14589` 的 UDP 端口，并验证实际飞控连接和遥测。

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
| `midcourse_algorithm.use_px4_limits` | 默认 `false`，使用 INI 的四项规划限值；`true` 使用 PX4 缓存 |
| `midcourse_algorithm.max_horizontal_speed_m_s` | 水平速度限值，默认 60 m/s |
| `midcourse_algorithm.max_climb_speed_m_s` / `max_descent_speed_m_s` | 爬升、下降速度限值，默认均为 20 m/s |
| `midcourse_algorithm.cmd_accel_max_m_s2` | 指令加速度限值，默认 20 m/s² |
| `midcourse_algorithm.v_cruise_m_s` | 逼近段巡航速度，默认 55 m/s |
| `midcourse_algorithm.approach_brake_k` | 逼近段到点减速增益，默认 0.5 s⁻¹ |
| `midcourse_algorithm.tau_ff_s` / `r_min_m` | 视线角速率前馈增益和距离保护，默认 1 秒和 50 米 |
| `midcourse_algorithm.update_dt_s` | 规划器调用周期，默认 0.05 秒，须与 `behavior_tree.tick_interval_ms` 对应 |
| `midcourse_algorithm.target_invalid_grace_s` | 目标预测宽限期，默认 3 秒 |
| `midcourse_algorithm.control_output_mode` | 默认 `VelocityAcceleration`，可选 `Attitude` |
| `midcourse_algorithm.hover_brake_accel_m_s2` | 速度档触发悬停时的减速度，默认 5 m/s² |
| `midcourse_algorithm.yaw_mode` / `yaw_rate_max_deg_s` | 默认 `1` 按速度方向调整期望航向，变化率上限 45 deg/s；`0` 保持当前航向 |
| `midcourse_algorithm.track_gain_d` | 轨迹跟踪速度误差反馈增益，默认 `0.0`，仅使用位置反馈 |
| `midcourse_algorithm.enable_internal_terminal_confirmation` | 允许算法内部确认，默认 `false`。开启后达到确认条件时算法输出悬停设定值，任务继续运行 |
| `midcourse_algorithm.confirm_boresight_deg` / `confirm_rate_deg_s` / `confirm_hold_s` | 内部确认的离轴角上限、弹轴角速率上限和持续时长，默认 5 度、5 度/秒和 2 秒 |

省略 `update_dt_s` 时，规划器调用周期取自 `behavior_tree.tick_interval_ms`。
目标估计器周期取自雷达历史时间戳间隔的中位数。

UDP 飞控端口位于场景 02 的 PX4 远端端口 `14030`～`14039`、本地端口
`14280`～`14289`，或旧 SITL 的 `14580`～`14589`（`1458*`）时，程序将本机视为仿真飞机，
并禁用低电压告警。仿真连接以 MAVSDK `GroundStation` 身份持续发送 GCS 心跳，满足 PX4 的
GCS 预检；串口和其他 UDP 连接仍以 `CompanionComputer` 身份接入真实飞控。

### 安全

| 配置 | 说明 |
| --- | --- |
| `safety.alert_voltage_threshold_v` | 实机低电压告警阈值 |

配置文件修改后重启程序生效；`SystemParam` 消息可在运行中更新安全高度。

## AgentInfo 上报与查询

机载和地面站通过 `sendToAll` 发布 `AgentInfo`。节点启动、发现其他节点上线或重连时上报，
重复上线通知按存活标记去重。各节点在同一 Zenoh 网络中使用唯一的 `agentId`。
字段、枚举和通信地址见 [`message.h`](../common/common_include/message.h) 与[跨端流程](../common/docs/groundstation-onboard-flow.md)。

机载的 `agentType = "Vehicle"`，`isSim` 沿用飞控仿真判定。`gcs_sim` 上报 `agentType = "GroundStation"` 和
`isSim = true`，默认 `agentId = "gcs-sim"`。多实例联调时分别用 `--sender-id` 设置唯一 ID，通信 IP 由 `--board-ip` 指定。
机载 `CommunicationApi::queryAgent` 通过 `queryToAgent/{agentId}/AgentInfo` 查询，两种角色使用相同地址格式。

使用配套工具查询机载或地面站。

```bash
./build/gcs_sim --query AgentInfo --agent-id uav1 --no-radar-udp --listen-seconds 1
```

`--agent-id` 默认取 `--plane-id`。可用 `--connect tcp/<host>:<port>` 指定 Zenoh 端点，
`--query-timeout-ms 2000` 设置超时。独立查询追加 `--no-presence`，跳过本工具的上线标记、上报和查询响应。
查询失败、类型不支持或没有响应时，工具返回非零退出码。

自动化控制场景可追加 `--control-settle-ms <ms>`：工具在建立 Zenoh 上行订阅与地面站
存活标记后，先等待指定时长使路由匹配完成，再发送 Task 或 Command。默认不等待；该参数
只影响控制报文的发送时机，不增加重发或改变 Ack 语义。

## 雷达入口与中制导联调

`gcs_sim` 默认监听 `0.0.0.0:7790`。雷达对外只有 `type=radar_tracks`、`schema_version=1` 一种
UDP JSON 报文，每个扫描周期对应一个完整数据报；`tracks` 可以为空，也可以同时包含 `confirmed`
和 `coasting`。空扫描和保持航迹会被正常接收，但只有 `confirmed` 航迹参与友机过滤并发布为
`GcsTarget`，载荷包含 `trackId`、`geoPos` 和 `timestamp_ms`。防守飞机坐标从 Zenoh `VehicleState`
获取。GCS 使用扫描时间外推友机位置，对 50 米内的 confirmed 雷达航迹做全局一对一匹配并过滤；友机没有出现在某次
雷达扫描中不会阻塞靶机，但友机状态缺失或过期时整帧暂停转发。

运行时每 5 秒至多输出一次雷达接收统计，分别记录 UDP 数据报、有效 `radar_tracks` 扫描、
无靶机输出扫描和已发布靶机数量，用于区分网络未到达、空/保持扫描和 confirmed 靶机发布。

手动启动 `192.168.1.56:14580` 的 PX4 SITL 后，可执行：

```bash
python3 tools/fleetctl.py test-midcourse --count 1 --target-track-id <track_id>
```

工具部署到 `uav1`，监听 `192.168.1.100:7790`。机载程序从启动起持续缓存每个航迹最近 20 帧，
中制导会话直接使用已有历史，不从任务启动时重新计数；测试继续发送第 21 帧并验证增量更新、
当前配置的 Offboard 输出、持续运行和 Task Stop 清理。目标 ID 留空时，机载只在当前候选唯一时自动锁定。

## 测试

构建检查、自动化脚本、硬件环境和人工验收步骤见 [`docs/TESTING.md`](docs/TESTING.md)。
