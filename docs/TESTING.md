# 测试说明

> 本文定义项目需要满足的测试环境、场景、执行方式和通过标准。产品验收目标见
> [`SPEC.md`](SPEC.md)，构建运行方法见 [`../README.md`](../README.md)。新增或修改功能时，应同步补充对应场景。

## 1. 测试原则

- 测试结果只允许标记为：通过、失败、跳过、未执行。
- 只有实际执行且满足全部断言的场景可以标记为通过。
- 因硬件、模型、插件或网络环境缺失而未运行时，必须说明原因和影响。
- 不得通过删除断言、放宽标准或静默跳过掩盖失败。
- 自动化脚本覆盖不完整时，仍需按本文执行人工或联调验收。
- 测试产生的临时文件应放在测试临时目录，不污染工程和正式配置。
- 相关场景不验证低电压告警。

## 2. 基准环境

### 2.1 平台与设备
#### px4 sitl环境
1）192.168.1.40 zsh/1，登录后到 `/home/zsh/code/PX4-Autopilot/scenarios/02_rocket_two_axis_pod` 执行
    先停止，再启动全新的环境
    ./start.sh 脚本启动测试仿真环境，默认3架， -n 参数用于指定启动的飞机架次；
    ./stop.sh停止仿真环境
2）start脚本执行后会输出几架飞机使用的飞机url、吊舱视频流和吊舱控制端口，从中解析出信息来修改飞机中对应的配置文件

#### 测试环境
1）每架机载对应一块板子
2）4架飞机分别是 `192.168.1.100`、`192.168.1.101`、`192.168.1.102`、`192.168.1.103`，均使用 `firefly/firefly`
3）机器可读清单以 `config/fleet.ini` 为准。请求飞机数超过其中声明的板卡数时，必须在启动任何程序前报告环境不足。

#### 多机自动化

执行约束：

- 用户要求启动或验证多架飞机时，必须使用 `python3 tools/fleetctl.py`，不得临时拼接远端命令绕过预检。
- 环境清单以 `config/fleet.ini` 为准，不得扫描或猜测未声明的板卡。
- 请求数量超过清单声明数量时，只报告环境不足，不启动部分 SITL 或机载程序。
- 板端架构或 Ubuntu 版本与 `fleet.ini` 不一致时，必须在修改远端环境前终止，不得跨发行版混装 APT 依赖。
- 飞控端口使用 `fleet.ini` 声明的 API/Offboard 端口基准，并在每次启动后验证对应 UDP 端口均已实际监听。所有端点都必须通过数量和唯一性校验，不得使用未经验证的猜测值。

只做连接和环境预检：

```bash
python3 tools/fleetctl.py preflight --count 2
```

启动两架飞机并验证双方通过 Zenoh liveliness 感知到友机在线：

```bash
python3 tools/fleetctl.py test --count 2
```

只部署双机并验证上下线及日志降频，不重启 SITL、不修改板端现有连接参数：

```bash
python3 tools/fleetctl.py test-presence --count 2
```

用户手动启动 `192.168.1.40:14580` 的 PX4 SITL 后，部署一号机并验证中制导：

```bash
python3 tools/fleetctl.py test-midcourse --count 1 --target-track-id <track_id>
```

该命令不会启动或停止 SITL；完成板端自检后会提示持续向 `192.168.1.100:7790` 发送至少
21 个时间戳严格递增的 `radar_tracks` 扫描。7790 只接收雷达数据，防守机状态由 GCS 从
Zenoh 获取，雷达帧无需包含防守机航迹。

该命令只预检和操作清单中的前两块板卡，备份旧程序及配置，验证双向正常退出、恢复上线，
再终止二号机进程以验证异常退出及恢复。初始和结束各观察 10 秒，检查重复上下线和位置更新日志。
报告及各阶段原始日志保存在 `logs/fleet/<run_id>/`，成功后双机保持运行。

如果部署已启动但监控会话中断，可复核并重写该次报告，不重启现场：

```bash
python3 tools/fleetctl.py verify --count 2 --run-id <run_id>
```

停止本次环境：

```bash
python3 tools/fleetctl.py down --count 2
```

`test` 使用完整的 `build/plane`；包不完整时才调用 CMake 编译。脚本先停止旧机载进程，随后按以下顺序准备板端环境：

1. 校验板端架构与 Ubuntu 版本，再把 `config/fleet.ini` 中的代理写入板端 `/etc/apt/apt.conf.d/99zz-fleet-proxy`；
2. 按清单补装核心运行依赖，不安装图像处理或媒体库；
3. 重启 SITL，解析飞控地址；本版本不安装 RKNN 库或启动图像链路；
4. 同步到 `/home/firefly/plane`，生成逐机配置，启动并持续分析日志。

Zenoh 使用默认发现，不依赖 GCS 地址。测试证据写入 `logs/fleet/<run_id>/`；验证通过后环境保持运行，以便继续检查，失败时也保留现场并提示使用 `down` 停止。

板端运行日志固定写入 `logs/plane.log`，不创建日期目录，单文件达到 50 MB 后轮转为
`plane.1.log` 至 `plane.5.log`。`logs/plane.stdout.log` 保存本次启动的标准输出和错误输出。
`verify --run-id` 将当前板端日志采集到指定的本地证据目录，不用于读取板端历史运行。

### 2.2 软件出包

运行包只要求主程序、配置、启停脚本和核心动态库；不包含模型、视觉插件或媒体服务。
相机控制模块保留，但接口不传递图像帧。

## 3. 基础验证

### 3.1 构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

通过标准：

- CMake 配置和编译成功；
- 生成 `build/plane/plane`；
- 打包阶段成功生成独立运行目录；
- 不新增编译警告；已有警告应在交付说明中列出。

### 3.2 依赖自检

```bash
cd build/plane
./plane --deps-check
```

通过标准：程序退出码为 0，日志包含依赖版本和检查成功信息。

## 4. 自动化测试

### 4.1 无图像依赖检查

- 源码不得引用 Perception、DetectionResult、VisionTarget 或 OpenCV 图像帧类型。
- 编译参数、链接命令及主程序的动态依赖不得包含 OpenCV、RGA、Pluma 或 RKNN。
- 启动脚本不要求模型或图像插件；保留自动修复执行权限。
- 相机控制仍可通过现有接口发送指令和查询状态；具体吊舱联调另行验证。

### 4.2 核心回归

`zenoh_liveliness` 在本机 TCP 回环网络上验证已有存活标记发现、退出下线和重连上线，不连接飞机。
`state_store_quiet` 验证连续状态更新仍发布内部事件并更新缓存，但不产生日志。
部署工具按最新运行中的上下线事件验证在线状态，不再把日志条数当作收到的位置样本数。

运行日志检查须确认项目人工编写的消息和业务描述均使用中文，不出现英文句子；API 名、协议名、
类型名、枚举值、配置键、函数参数名、结构体及其字段名和第三方库原始错误等代码标识保留原文。
检查 PX4 飞控系统参数缓存日志时，四项键必须为 `MPC_XY_VEL_MAX`、`MPC_Z_VEL_MAX_UP`、
`MPC_Z_VEL_MAX_DN`、`MPC_ACC_HOR_MAX`，不得使用中文意译键名。
日志级别继续使用 spdlog 的 `[info]`、`[warn]`、`[error]` 等原始英文标识。主程序和 `gcs_sim`
应使用相同格式，线程 ID 只显示为 `[%t]`，不得带“线程”前缀，且格式中不自动输出英文函数名。


`config_simulation` 验证 UDP 端口 `14580`～`14589` 为仿真，范围外端口和串口不判定为仿真。

`radar_target_filter` 验证 JSON 容错、confirmed 过滤、严格时间戳输入、多友机全局匹配、
50 米门限、NED 状态外推、过期状态失败关闭，以及“友机未出现在雷达帧时仍转发靶机”。
`midcourse_algorithm_adapter` 使用真实 `libmidcourse_alg_3588.so` 验证 20 帧初始化、增量更新和逐 tick 调用；
`midcourse_controller` 验证任务启动前已形成的 20 帧持续缓存、参数缓存读取、会话重置和错误路径；
`attitude_conversion` 验证四元数归一化与姿态转换。
中制导接口日志必须使用 `接口名 输入/输出` 的简洁前缀，结构体字段保持公共接口原名，经纬度固定
保留 6 位小数，速度固定保留 2 位小数，不得恢复“中制导接口 方向=……”前缀或中文结构体字段。
接口输入输出不得包含会话号、调用号或更新号；会话销毁日志保留调用次数和两类更新次数的累计统计。
排查真实算法调用时，可按 `MidcoursePlanner::` 和 `TargetEstimator::` 过滤板端 `logs/plane.log`；
同一会话内按日志时间排序后，必须能看到 `MidcoursePlanner::setConfig` → `TargetEstimator::init` →
持续 `MidcoursePlanner::update` →（存在新帧时）`TargetEstimator::update` 的输入输出，接口失败时
保留原始错误字符串。

```bash
ctest --test-dir build --output-on-failure
python3 tests/unit/test_fleetctl.py
```

### 4.3 航线执行超时回归

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build --target mission_node_timeout_test -j
ctest --test-dir build -R '^mission_node_timeout$' --output-on-failure
```

使用模拟飞控与测试时间源验证，无需仿真飞控或真实等待：

- Route 在等待首次进度、飞行时间超过普通节点超时和长时间运行时均保持 RUNNING，直到任务进度完成。
- Waypoint 仍遵守普通节点超时。
- 长航线仍支持中止和重新启动，飞控分发失败仍返回 FAILURE。

### 4.4 航线起飞与高度调整

```bash
cmake --build build --target route_preflight_test -j
ctest --test-dir build -R '^route_preflight$' --output-on-failure
```

使用模拟飞控验证：

- 未解锁且未起飞、已解锁但未起飞、已经起飞三种状态生成正确的航线前置节点；
- 高度调整只使用首航点和当前状态的相对高度，不混用海拔高度；
- 高度调整向飞控下发当前位置、目标相对高度的临时非穿越航点，不使用 Offboard；
- 临时航点任务完成、达到高度容差并退出 Mission 后才开始正式航线；
- 中止和超时时切换 Hold，停止临时航点任务；
- 起飞节点使用本次实际下发的目标高度判定完成。

## 5. 相机／吊舱控制

保留指令发送、状态查询和链路状态接口，不做图像处理。当前为 Noop 实现，不能将其单元验证等同于真实吊舱接入成功。

## 6. 飞控与指令验收

以下场景需要仿真飞控或实机环境，目前没有完整自动化脚本：

| 场景 | 通过标准 |
| --- | --- |
| UDP 连接 | 能发现 PX4，状态变为 connected，断开和恢复可识别 |
| 串口连接 | 使用配置的设备和波特率连接 PX4 |
| Arm/Disarm | 指令成功且以真实 armed 状态判定完成 |
| Takeoff | 使用指定高度；未指定时使用安全高度；达到容差后完成 |
| Land/RTL | 以实际离地状态判定完成 |
| Hover | PX4 接受 Hold 指令 |
| Waypoint | 上传单航点并以 Mission Progress 判定完成 |
| Route | 未起飞时自动解锁并起飞；开始前调整到首航点相对高度；随后按顺序执行多航点并以 Mission Progress 判定完成；飞行超过普通节点超时仍继续等待完成，新指令仍可替换航线 |
| StopMotor | 仅接受地面站人工指令，不由任务自主触发 |
| Attitude Offboard | 先发送合法姿态／推力设定值再启动 Offboard，按 50 ms 重发；起飞完成切换覆盖 Offboard 时能够限频恢复；停止任务时停止 Offboard 并清空缓存 |

实机执行 StopMotor 和撞击相关测试前必须确认安全区域和测试授权。

## 7. 任务与通信验收

| 场景 | 通过标准 |
| --- | --- |
| 单一控制内容 | 任意时刻只有一个 Task 或 Command 执行 |
| 最新内容替换 | 新 Task/Command 中止当前执行并立即启动最新内容 |
| 任务中止 | 只有匹配 taskId 的 Stop 生效，后续节点不再执行 |
| 非 GCS 控制 | 拒绝其他发送方下发的 Task 和 Command |
| 任务确认 | 收到合法 Task 后返回对应 messageId 的 Ack |
| 状态广播 | 按配置周期广播本机最新 VehicleState |
| 友机状态 | 按 planeId 更新和查询其他无人机最新状态，位置更新不打印日志 |
| 友机上下线 | Zenoh 存活标记出现时打印一次上线，撤销或失联时打印一次下线，恢复后重新打印上线 |
| 任务状态 | 地面站能获得状态、行为树和当前节点；无任务时为 None |
| GCS 链路 | 收到 GCS 报文变为 connected，超过阈值变为 disconnected |
| 队列上限 | 接收队列满时丢弃最旧消息，进程保持可用 |
| 确认重试 | 可靠消息超时后按配置重试，超过次数产生明确错误 |

## 8. 中制导验收

自动化和板端联调必须共同覆盖：

1. `ldd build/plane/plane` 包含 `libmidcourse_alg_3588.so`，不包含 Pluma；无 SONAME 的算法库可由 `$ORIGIN/lib` 解析。
2. `AntiUav` 根据飞行状态选择直接 Midcourse、Takeoff → Midcourse 或 Arm → Takeoff → Midcourse。
3. 指定 ID 时锁定该目标；未指定时只允许唯一候选，不得在多个目标间自动跳换。
4. 机载从进程启动起持续缓存目标；任务启动前已经收到 20 帧时直接完成初始化，第 21 帧产生首次增量更新；规划器每个 50 ms tick 先于本 tick 的目标增量更新调用。
5. 飞控连接后成功读取并缓存 `MPC_XY_VEL_MAX`、`MPC_Z_VEL_MAX_UP`、`MPC_Z_VEL_MAX_DN`、`MPC_ACC_HOR_MAX`；中制导任务路径只读取缓存，断连时缓存失效，重连后重新获取，缓存未就绪时任务失败。
6. 算法输出的所有数值、四元数和推力经过校验，横向推力接近零，使用 body→NED 四元数转换后的 Attitude Offboard。
7. 雷达中断超过 3 秒后任务仍运行；Task Stop、新控制内容和异常退出均停止 Offboard 并重置会话。
8. GCS 从 Zenoh 获取防守机状态，只过滤 50 米内实际匹配到的雷达航迹；雷达帧没有友机航迹时仍转发靶机，状态缺失或过期时暂停整帧。

板端验收只有在用户确认 SITL 已启动后才允许发送控制；网络、权限或仿真环境缺失时必须报告阻塞。
报告保存在 `logs/fleet/<run_id>/midcourse-report.json`，并保留各阶段 GCS 和机载原始日志。

## 9. 已移除任务流程

原本依赖本地视觉检测的任务启动时必须明确返回不可用，不启动截短的任务树，也不误报完成。

## 10. 目标和安全验收

| 场景 | 通过标准 |
| --- | --- |
| GCS 目标 | 按 trackId 保存最新值，每个目标最多保留 20 条历史 |
| 动态安全高度 | SystemParam 更新后，后续默认起飞使用新值 |
| 实机低电压 | 飞控连接正常且首次低于阈值时发送一次告警 |
| 电压恢复 | 电压恢复后解除锁存，再次低电压可重新告警 |
| 仿真低电压 | 仿真飞机不发送低电压告警 |

## 11. 测试报告

交付时至少记录：

```text
构建：通过/失败/未执行，命令与结果
自动化测试：逐项状态
硬件与联调测试：逐项状态、环境和设备
未执行项：原因和风险
失败项：日志位置、现象和初步定位
```

## VehicleInfo 信息接口验证

`vehicle_info` CTest 使用关闭组播和 gossip 的 TCP 回环会话，验证配置加载、无地面站时的启动上报、
指定地面站上线／重复通知／重连、已有 token 的 history 发现、无周期重复上报，以及原生查询的成功、
当前配置读取、isSim 的 true/false 布尔值、非法参数、不支持类型、错误发送方和无响应。测试不发送飞行或任务指令。

部署并在一号机验证：

```bash
python3 tools/fleetctl.py test-vehicle-info --count 1
```

该命令只预检一号机，备份程序和配置，部署完整运行包和 gcs_sim，保留现有连接参数及视频地址；
任务板 IP 未配置时写入 fleet.ini 中的该板 IP。gcs_sim 在板端独立进程中运行，验证：

1. 地面站已订阅时启动机载，收到一次 VehicleInfo，观察期间没有周期重复；
2. 地面站再次上线及恢复上线时，各收到一次当前配置的信息；
3. 原生查询指定飞机，结果与板端配置完全一致；isSim 与现有仿真判定一致；仅查询时没有额外主动上报；
4. 不存在的飞机明确返回无响应或超时，不支持的查询类型明确失败；
5. 板端二进制校验一致、依赖自检成功、机载最终保持运行。

不重启 SITL、不发送控制报文、不连接或处理视频流。证据保存到
`logs/fleet/<run_id>/vehicle-info-report.json` 及对应原始日志。
