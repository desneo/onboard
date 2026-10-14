# plane

> 本文面向项目使用者，说明项目背景、当前实现、构建运行方法和配置。目标需求见
> [`docs/SPEC.md`](docs/SPEC.md)，实现约束见
> [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)，测试方法见
> [`docs/TESTING.md`](docs/TESTING.md)。

## 项目背景

`plane` 是运行在无人机机载计算平台上的 C++17 应用，面向基于 PX4 的反无人机任务。程序连接飞控、地面站、其他无人机和视觉插件，在单进程内完成状态管理、任务仲裁、行为树执行、感知和视频推流。

支持 Ubuntu 22.04 的 x86_64 与 ARM64，首要硬件平台为 RK3588。

## 核心功能场景

以下场景描述项目的目标用途；实际完成情况以“当前能力”和“当前已知限制”为准。

### 自主反无人机任务

地面站下发目标和任务后，机载程序按预定流程控制基于 PX4 的火箭穿越机执行自杀式反制：

```text
解锁 → 起飞 → 中段制导 → 目标检测 → 单目标跟踪 → 导引撞击
```

系统负责组织任务阶段、调用外部视觉算法并向飞控下发控制；飞控固件和检测、跟踪、导引算法本身由外部能力提供。

### 单机飞行控制

地面站可以下发解锁、上锁、起飞、降落、返航、航点飞行、航线飞行、悬停和停桨等独立指令。航点和航线既可作为独立指令，也可作为复杂任务中的执行步骤。

### 最新控制内容优先

系统不维护任务或指令等待队列，任意时刻只执行一个 Task 或 Command。收到新的控制内容时，中止当前执行并立即切换到最新内容。

### 感知与视频链路

系统从 MIPI 或 RTSP 获取图像，按任务阶段调用检测、单目标跟踪和图像导引插件，管理算法输出，并通过视频推流模块向外提供实时画面。

### 地面站与机间通信

系统通过 Zenoh 与地面站及其他无人机交换业务消息，支持向地面站、指定无人机和其他节点发送消息，并周期上报本机状态与当前任务状态。

## 当前能力

- 通过 UDP 或串口连接 PX4，采集位置、姿态、速度、电压和飞行模式。
- 执行解锁、上锁、起飞、降落、返航、悬停、停桨、航点和航线指令。
- 基于 Zenoh 接收地面站任务、指令、系统参数和目标信息。
- 周期广播本机状态，接收并缓存其他无人机的最新状态。
- 通过 BehaviorTree 仲裁和执行单个 Task 或 Command，新控制内容替换当前内容。
- 从 MIPI 或 RTSP 取图，调用检测/MOT 插件并输出视觉目标。
- 通过插件输出 RTSP 或 WebRTC 视频流，并叠加检测结果。
- 监测实机低电压并向地面站告警。
- 执行反无人机任务的解锁、起飞、中段制导和目标检测阶段。

当前已知限制：

- 反无人机任务尚未实现单目标跟踪和末段导引撞击。
- 相机控制模块目前是 Noop，不控制真实相机或云台。
- 友机状态已交换和缓存，但尚未参与编队、避碰或协同决策。
- 视觉目标仅在本机保存和叠加显示，尚未通过通信模块上报。

## 目录概览

```text
src/                  机载程序源码
config/plane.conf     默认运行配置
tests/                契约测试、集成测试及测试配置
scripts/              源码树中的程序启停脚本
tools/                插件暂存、打包和辅助程序
cmake/                CMake 依赖配置
docs/                 需求、架构和测试说明
```

完整目录职责见 [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)。

## 外部依赖

- 公共数据模型与插件接口：`../common/common_include`
- 预编译第三方库：`../../../3rdparty/common`
- 默认插件来源：相邻的 `functional/` 和 `algorithm/` 工程
- 默认模型来源：`../algorithm/perception/models`

主要依赖包括 MAVSDK、Zenoh、BehaviorTree.CPP、eventpp、Pluma、OpenCV、GeographicLib、spdlog、fmt、nlohmann-json、magic_enum，以及 ARM64 上用于缩放的 RGA。

依赖缺失时应补充对应本地库或通过下述环境变量指定插件，项目不会在线下载依赖。

## 构建与打包

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

默认构建会生成可独立运行的 `build/plane/`，同时暂存插件和运行库。插件不在默认位置时，可在构建前设置：

```bash
export PLANE_IMAGE_INPUT_SO=/path/to/libimage_input.so
export PLANE_VIDEO_OUTPUT_SO=/path/to/libvideostream_output.so
export PLANE_DET_MOT_SO=/path/to/libdet_mot.so
export PLANE_MODELS_DIR=/path/to/models
```

已有构建目录也可以手动重新打包：

```bash
./tools/package.sh build
```

系统级 OpenCV、FFmpeg、GStreamer 等运行库仍由部署环境提供。

## 运行与停止

从源码树启动：

```bash
./scripts/runplane.sh
```

或指定独立运行目录：

```bash
./scripts/runplane.sh /path/to/plane_dir
```

启动脚本要求存在 `rknn-inspect`，会读取 RKNN 模型输入尺寸并同步更新取图、检测和推流分辨率，然后运行 `plane`。

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

默认配置为 [`config/plane.conf`](config/plane.conf)，格式为 `key = value`。

### 系统与通信

| 配置 | 说明 |
| --- | --- |
| `safe_height_m` | 未指定起飞高度时使用的安全高度 |
| `log_dir` | 日志目录 |
| `state_report_period_ms` | 状态上报周期 |
| `plane_id` / `station_id` | 本机和地面站标识 |
| `zenoh_config_file` | 可选 Zenoh 配置文件 |
| `zenoh_connect` / `zenoh_listen` | Zenoh 连接或监听端点 |
| `comm_ack_timeout_ms` / `comm_ack_max_retries` | 可靠消息确认超时和重试次数 |
| `comm_rx_queue_max` | 通信接收队列容量，满时丢弃最旧消息 |
| `gcs_link_timeout_ms` | 地面站链路超时阈值 |
| `debug_zenoh_log` | 是否记录 Zenoh 消息目标端点 |

### 飞控与任务

| 配置 | 说明 |
| --- | --- |
| `fc_link_type` | `udp` 或 `serial` |
| `fc_udp_url` | MAVSDK UDP 地址 |
| `fc_serial_device` / `fc_serial_baud` | 串口设备和波特率 |
| `fc_heartbeat_timeout_ms` | MAVSDK 心跳超时 |
| `fc_route_speed_m_s` | 航点和航线速度 |
| `bt_tick_interval_ms` | 行为树 tick 周期 |
| `bt_node_timeout_ms` | 普通行为树节点超时 |
| `bt_takeoff_tolerance_m` | 起飞高度判定容差 |

UDP 飞控端口位于 `14200`～`14299` 时，程序将本机视为仿真飞机，并禁用低电压告警。

### 图像与检测

| 配置 | 说明 |
| --- | --- |
| `plugins_dir` | 动态插件目录 |
| `image_source_type` | `Mipi` 或 `Rtsp` |
| `image_mipi_device` | MIPI 设备；空值表示自动发现 |
| `image_rtsp_url` | RTSP 输入地址 |
| `image_reconnect_interval_s` | 图像源断流重试间隔 |
| `image_width` / `image_height` | 图像输入尺寸 |
| `det_model_path` | 检测模型文件或目录 |
| `det_config_dir` | 检测插件配置目录 |
| `det_confidence_threshold` / `det_nms_threshold` | 检测阈值 |
| `det_track_thresh` / `det_track_buffer` | MOT 参数 |
| `det_input_width` / `det_input_height` | 检测输入尺寸 |

### 视频输出

| 配置 | 说明 |
| --- | --- |
| `stream_type` | `Rtsp` 或 `WebRtc` |
| `stream_url` | 可选的显式推流地址 |
| `stream_app` / `stream_name` | 流应用名和流名称 |
| `stream_rtsp_port` / `stream_http_port` / `stream_rtc_port` | 服务端口 |
| `stream_width` / `stream_height` | 输出分辨率 |
| `stream_max_fps` | 最大输出帧率 |
| `stream_bitrate_kbps` / `stream_gop` | 编码码率和 GOP |

### 安全与反无人机任务

| 配置 | 说明 |
| --- | --- |
| `alert_voltage_threshold_v` | 实机低电压告警阈值 |
| `suicide_detection_start_distance_m` | 切换到检测阶段的水平距离 |
| `suicide_approach_speed_m_s` | 中段逼近速度 |
| `suicide_track_approach_speed_m_s` | 跟踪阶段逼近速度，当前尚未使用 |
| `suicide_climb_speed_m_s` | 中段垂直速度限制 |
| `suicide_target_select_rule` | 视觉目标选择规则，当前尚未使用 |
| `suicide_guidance_start_stable_ms` | 导引启动稳定时间，当前尚未使用 |
| `suicide_track_lost_timeout_ms` | 跟踪丢失超时，当前尚未使用 |
| `suicide_phase_timeout_ms` | 中段和检测阶段超时 |
| `suicide_impact_height_m` / `suicide_impact_vd_m_s` | 撞击判定参数，当前尚未使用 |
| `debug_target_log` | 是否输出逐帧目标调试日志 |

配置文件修改后重启程序生效；`SystemParam` 消息可在运行中更新安全高度。

## 测试

构建检查、自动化脚本、硬件环境和人工验收步骤见 [`docs/TESTING.md`](docs/TESTING.md)。
