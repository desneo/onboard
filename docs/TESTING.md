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

#### PX4 SITL 环境

SITL 测试使用 `tests/fleet.ini` 的 `simulator.host`（当前为 `192.168.1.56`）和 `simulator.scenario_dir`。
场景的 `start.sh` 以 `-n` 指定飞机数量，并输出逐机飞控、吊舱视频流和控制端口。对现有
`14580`～`14589` 的 SITL UDP 连接保留端口并校正主机，启动后验证实际飞控连接和遥测。

#### 测试环境
1）每架机载对应一块板子
2）4架飞机分别是 `192.168.1.100`、`192.168.1.101`、`192.168.1.102`、`192.168.1.103`，均使用 `firefly/firefly`
3）机器可读清单以 `tests/fleet.ini` 为准。请求飞机数超过其中声明的板卡数时，必须在启动任何程序前报告环境不足。

#### 多机自动化

执行约束：

- 用户要求启动或验证多架飞机时，必须使用 `python3 tools/fleetctl.py`，不得临时拼接远端命令绕过预检。
- `fleetctl.py` 默认读取 `tests/fleet.ini`，可通过全局选项 `--config <路径>` 指定清单。只使用清单声明的板卡。
- 对已安装自启服务的板卡，执行 `sudo systemctl disable --now plane.service` 后再运行 `fleetctl` 的部署与启停测试。
- 请求数量超过清单声明数量时，只报告环境不足，不启动部分 SITL 或机载程序。
- 板端架构或 Ubuntu 版本与清单不一致时，必须在修改远端环境前终止，不得跨发行版混装 APT 依赖。
- 飞控端口使用清单声明的 API/Offboard 端口基准，并在每次启动后验证对应 UDP 端口均已实际监听。所有端点都必须通过数量和唯一性校验。

只做连接和环境预检：

```bash
python3 tools/fleetctl.py preflight --count 2
```

启动两架飞机并验证双方通过 Zenoh liveliness 感知到友机在线：

```bash
python3 tools/fleetctl.py test --count 2
```

只部署双机并验证上下线及日志降频，不重启 SITL；已有 SITL UDP 地址校正到清单中的仿真主机，保留端口及其他现场参数：

```bash
python3 tools/fleetctl.py test-presence --count 2
```

需要清空双机已有固定连接地址并使用随包的 2 秒建链超时配置时，增加 `--auto-discovery`；
SITL UDP 端口仍保留板端现有值，主机统一使用 `192.168.1.56`。

该命令预检和操作清单中的前两块板卡，备份旧程序及配置，验证双向正常退出、恢复上线，
再终止二号机进程以验证异常退出及恢复。初始和结束各观察 10 秒，检查重复上下线和位置更新日志。
报告及各阶段原始日志保存在 `logs/fleet/<run_id>/`，成功后双机保持运行。

用户手动启动 `192.168.1.56:14580` 的 PX4 SITL 后，部署一号机并验证中制导：

```bash
python3 tools/fleetctl.py test-midcourse --count 1 --target-track-id <track_id>
```

该命令不会启动或停止 SITL；完成板端自检后会提示持续向 `192.168.1.100:7790` 发送至少
21 个时间戳严格递增的 `radar_tracks` 扫描。7790 只接收雷达数据，防守机状态由 GCS 从
Zenoh 获取，雷达帧无需包含防守机航迹。空扫描和 `coasting` 航迹会被合法接收但不发布为
`GcsTarget`；测试所需的 20+1 帧必须包含指定 `track_id` 的 `confirmed` 航迹。运行日志中的
雷达接收统计用于区分 UDP 未到达、合法但无靶机输出的扫描和已发布靶机。

如果部署已启动但监控会话中断，可复核并重写该次报告，不重启现场：

```bash
python3 tools/fleetctl.py verify --count 2 --run-id <run_id>
```

停止本次环境：

```bash
python3 tools/fleetctl.py down --count 2
```

`test` 使用完整的 `build/plane`；包不完整时才调用 CMake 编译。脚本先停止旧机载进程，随后按以下顺序准备板端环境：

1. 校验板端架构与 Ubuntu 版本，再把清单中的代理写入板端 `/etc/apt/apt.conf.d/99zz-fleet-proxy`。
2. 按清单补装核心运行依赖。
3. 重启 SITL，解析飞控地址。
4. 同步到 `/home/firefly/plane`，生成逐机配置，启动并持续分析日志。

工具读取板端 `config/plane.ini`，首次升级可读取旧 `config/plane.conf` 迁移现场参数。需要注入逐机端点的测试先读取现场有效配置，再将生成的 `plane.ini` 的 `profile` 留空。

Zenoh 使用默认发现，不依赖 GCS 地址。测试证据写入 `logs/fleet/<run_id>/`；验证通过后环境保持运行，以便继续检查，失败时也保留现场并提示使用 `down` 停止。

板端运行日志固定写入 `logs/plane.log`，不创建日期目录，单文件达到 50 MB 后轮转为
`plane.1.log` 至 `plane.5.log`。`logs/plane.stdout.log` 保存本次启动的标准输出和错误输出。
`verify --run-id` 将当前板端日志采集到指定的本地证据目录，不用于读取板端历史运行。

### 2.2 软件出包

运行包包含主程序、启停脚本、`install-plane-service.sh`、核心动态库、`config/plane.ini`、`config/zenoh.json5` 和 `config/profiles/` 下的四份仿真预置。

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

### 3.3 部署预置板端验收

1. 确认捷联 A2416A SITL 一号机在 `192.168.1.56:14280` 提供飞控连接。
2. 从 `tests/fleet.ini` 选取 `uav1`（`192.168.1.100`），备份板端现场配置。将运行包部署到 `/home/firefly/plane`，保留默认 `profile = config/profiles/sitl_jielian_a2416a.ini` 并执行 `./runplane.sh`。
3. 从本机同一 Zenoh 网络查询板端信息并接收遥测。

```bash
./build/gcs_sim --query AgentInfo --agent-id uav1 --no-radar-udp --listen-seconds 5
```

通过标准为启动日志显示匹配 `uav1`、飞控实际连接到 `14280`，收到 `fcLinkState=connected` 的有效位置遥测。
查询结果应包含 `agentId=uav1`、`boardIp=192.168.1.100`、`videoUrl=rtsp://192.168.1.56:8554/jielian_a2416a` 和 `isSim=true`。

### 3.4 开机自启和崩溃恢复验收

- 本地对部署与启停脚本执行 `bash -n`，对生成的 `plane.service` 执行 `systemd-analyze verify`，并完成构建和相关回归。在隔离的 systemd 测试环境中验证连续异常触发启动限额。
- 在 uav1（`192.168.1.100`）的 `/home/firefly/plane` 部署，使用 3.3 节的 SITL 环境。记录安装前的 `sysctl -n kernel.core_pattern`，备份并停止已有服务，再部署运行包并执行安装。确认新路由指向运行目录的 `logs/core/`，服务为 `enabled` 和 `active`。重复安装和重复执行 `./runplane.sh --service` 后服务 PID 相同，执行 `./runplane.sh` 时提示先停服务。
- 检查后台服务启动日志只写入 `logs/plane.log`，journal 可查询 systemd 服务启停和异常记录。记录 `MainPID`，确认 `/proc/<PID>/exe` 指向 `plane`，再执行 `sudo systemctl kill --kill-who=main --signal=SIGABRT plane.service`。用 `file` 确认 `logs/core/` 中的 `core.<进程名>.<PID>.<Unix 时间戳>` 为 ELF core，核对文件名中的 PID，确认旧 core 保留。用 `journalctl -u plane.service -o short-monotonic` 检查 5 秒重启间隔和新进程，按 3.3 节确认飞控连接和有效位置遥测恢复。
- 分别在服务正常运行时和崩溃后 `SubState=auto-restart` 的等待窗口内执行 `./stopplane.sh`。等待超过 5 秒，检查 `MainPID=0`、服务保持停止且仍为 `enabled`。在终端执行 `./runplane.sh`，确认应用日志同时显示在终端并写入日志文件。按 `Ctrl+C` 退出，再执行 `./runplane.sh --service`，确认后台服务恢复且遥测正常。
- 卸载后检查服务注册已移除，`kernel.core_pattern` 已恢复为安装前的值，历史 core 可读。重新安装并确认服务处于 `enabled` 和 `active`。在维护窗口实际重启板卡，验证系统开机满 10 秒后服务启动、全局 core 路由生效，并检查飞控连接和遥测。

## 4. 自动化测试

### 4.1 无图像依赖检查

- 源码不得引用 Perception、DetectionResult、VisionTarget 或 OpenCV 图像帧类型。
- 编译参数、链接命令及主程序的动态依赖不得包含 OpenCV、RGA、Pluma 或 RKNN。
- 启动脚本不要求模型或图像插件；保留自动修复执行权限。
- 吊舱控制包和遥测包通过 UDP 传输，验收见相机／吊舱控制章节。

### 4.2 核心回归

`zenoh_liveliness` 在本机 TCP 回环网络上验证已有存活标记发现、退出下线和重连上线，不连接飞机。
`state_store_quiet` 验证连续状态更新仍发布内部事件并更新缓存，但不产生日志。
部署工具按最新运行中的上下线事件验证在线状态，不再把日志条数当作收到的位置样本数。

默认运行包须包含 `config/zenoh.json5`，由 `plane.ini` 的 `zenoh_config_file` 加载，
`transport/unicast/open_timeout` 为 `2000` 毫秒；`zenoh_connect` 留空，保留组播和 gossip 自动发现。
修改该配置后，在一号机验证自动发现的 AgentInfo 上报与查询、正常停止及恢复启动，
记录实际停止耗时，并确认没有触发停止脚本的强制结束。

运行日志检查须确认项目人工编写的消息和业务描述均使用中文，不出现英文句子；API 名、协议名、
类型名、枚举值、配置键、函数参数名、结构体及其字段名和第三方库原始错误等代码标识保留原文。
检查 PX4 飞控系统参数缓存日志时，四项键必须为 `MPC_XY_VEL_MAX`、`MPC_Z_VEL_MAX_UP`、
`MPC_Z_VEL_MAX_DN`、`MPC_ACC_HOR_MAX`，不得使用中文意译键名。
日志级别继续使用 spdlog 的 `[info]`、`[warn]`、`[error]` 等原始英文标识。主程序和 `gcs_sim`
应使用相同格式，线程 ID 只显示为 `[%t]`，不得带“线程”前缀，且格式中不自动输出英文函数名。


`config_simulation` 验证场景 02 的 PX4 远端 UDP 端口 `14030`～`14039`、本地端口
`14280`～`14289` 与旧 SITL 端口 `14580`～`14589` 为仿真，范围外端口和串口不判定为仿真。
配置测试还需覆盖同一预置按不同本机 IP 选择不同飞机 ID 和 RTSP、UDP 与串口飞控、单机直配，以及配置读取失败、重复 ID/IP、必填项无效、零匹配和多匹配。
`fleetctl` 单测验证测试清单默认路径、`--config` 覆盖及测试部署配置迁移。

`radar_target_filter` 位于 `tools/gcs_sim/tests`，验证不带速度和方位角的雷达航迹、JSON 容错、
空扫描、coasting 忽略、confirmed 过滤、严格时间戳输入、多友机全局匹配、
50 米门限、NED 状态外推、过期状态失败关闭，以及“友机未出现在雷达帧时仍转发靶机”。
`midcourse_controller` 使用真实 `libmidcourse_alg.so`，验证任务启动前已形成的 20 帧历史、增量更新、
INI／PX4 限值来源、两种输出模式、会话重置和错误路径；
`attitude_conversion` 验证四元数归一化与姿态转换。
按板端 `logs/plane.log` 的时间顺序检查规划器配置、估计器初始化、Offboard 启动和首次增量更新。
规划器配置日志包含限值来源和实际传入的四项限值。接口失败时保留原始错误字符串。

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
- 常规中止和超时时切换 Hold，人工接管中止时保留飞手选定的模式；
- 起飞节点使用本次实际下发的目标高度判定完成。

## 5. 相机／吊舱控制

运行 `ctest --test-dir build --output-on-failure` 检查吊舱编解码、断流停转、北向报文、默认上报频率和配置回退。

板端验收在 `192.168.1.56` 的场景 02 按 `--aircraft-model` 分别选择 `rocket_jielian_a2416a`、`rocket_aisheng_seeker` 和 `rocket_zhongke_d80`。按 `interfaces.tsv` 核对飞控和吊舱 UDP 端口，部署到 `uav1` 并确认收到 `fcLinkState=connected` 的实际位置遥测。

```bash
./build/gcs_sim --plane-id uav1 --no-radar-udp --control-settle-ms 700 --listen-seconds 4 --pod-angles 10,15
```

再分别使用 `--pod-zero-pitch`、`--pod-zero-yaw`、`--pod-rate 8,-5,1500` 和 `--pod-rate-timeout -8,5,1500`。
验收以 `PodState` 的实测角度和速度确认到位、另一轴保持及停止。使用 `gcs_sim` 日志时间核对默认约 5 Hz 的发布频率。捷联反馈应显示 `canMove=false`，并省略机械框架角。

## 6. 飞控与指令验收

以下场景需要仿真飞控或实机环境，目前没有完整自动化脚本：

| 场景 | 通过标准 |
| --- | --- |
| UDP 连接 | 能发现 PX4，状态变为 connected，断开和恢复可识别 |
| 仿真 GCS 预检 | 场景 02 或旧 SITL 中启动 plane 后，PX4 不再报告 `Preflight Fail: No connection to the GCS`；日志包含 `GroundStation` 身份，随后 Arm 可被接受 |
| 串口连接 | 使用配置的设备和波特率连接 PX4 |
| Arm/Disarm | 指令成功且以真实 armed 状态判定完成 |
| SelfCheck | 实际解锁后自动恢复上锁，飞机保持在地面，地面端收到可关联的成功或失败结果 |
| Takeoff | 使用指定高度；未指定时使用安全高度；达到容差后完成 |
| Land/RTL | 以实际离地状态判定完成 |
| Hover | PX4 接受 Hold 指令 |
| Waypoint | 上传单航点并以 Mission Progress 判定完成 |
| Route | 未起飞时自动解锁并起飞；开始前调整到首航点相对高度；随后按顺序执行多航点并以 Mission Progress 判定完成；飞行超过普通节点超时仍继续等待完成，新指令仍可替换航线 |
| StopMotor | 仅接受地面站人工指令，不由任务自主触发 |
| Attitude Offboard | 先发送合法姿态／推力设定值再启动 Offboard，按 50 ms 重发；起飞完成切换覆盖 Offboard 时能够限频恢复；停止任务时停止 Offboard 并清空缓存 |

实机执行 StopMotor 和撞击相关测试前必须确认安全区域和测试授权。

### 6.1 人工接管

运行 `ctest --test-dir build -R '^(manual_takeover|task_scheduler_takeover)$' --output-on-failure` 验证模式基线、快速切换、正常 `Hold`、完成遥测延迟、任务中止与一次性告警。

将机载程序部署到 `uav1` 的 `/home/firefly/plane`。
连接 `192.168.1.56` 的 PX4 SITL，使用 `mavlink status` 确认连到 `uav1` 的机载链路，再在该 PX4 实例的控制台调高机载链路 `HEARTBEAT` 频率。命令中的端口取 PX4 侧实例的本地 UDP 端口。

```console
mavlink stream -u <机载链路的PX4本地UDP端口> -s HEARTBEAT -r 20
```

抓取机载实际收到的 `HEARTBEAT`，核对其模式和接收间隔。用 SITL 模式控制模拟三档拨动，约 100 ms 的短时切换以实际收到的模式帧为准。实体遥控器的三档映射在台架上复验。

| 场景 | 通过标准 |
| --- | --- |
| 飞前 `Position`，随后由程序接手 | 正常起飞并进入预期自动模式。启动和重连时的首个模式不触发接管。 |
| 正常 `Hold` | 起飞完成、`Hover`、临时高度航点与正式航点正常结束均不告警。起飞尾段短暂覆盖 Offboard 后仍可正常交接。 |
| `Mission` 或 `Offboard` 中切入 `Loiter`、`Stabilized`、`Position` | 逐个独立验证三个入口，检查 PX4 的实际模式。航线未完成时进入 `Loiter`，在核对窗口结束前暂停下发，随后中止。接管后的出站抓包没有旧飞控指令、Offboard 设定值或切换 `Hold` 的清理报文。 |
| 快速拨动 `Loiter → Stabilized → Position` | 第一次符合接管条件即中止。同一次执行只产生一条 `manual_takeover` 告警，Task 上报 `Aborted`。旧报文重传只回 `Ack`，清理完成后新的 Task 或 Command 可以执行。 |
| 地面站界面 | 提示条显示飞机、实际模式和被中止的 Task 或 Command。重复收包或轮询时，界面只提示一次。 |

### 6.2 起飞前自检

`self_check_node` 覆盖解锁和上锁遥测、飞控拒绝、超时、重连与安全清理。`task_scheduler_takeover`
覆盖忙碌、优先中止和结果单次上报。`agent_info` 使用 Zenoh 回环验证单播、自检指令去重和结果关联。

部署到 `uav1` 的 `/home/firefly/plane` 后，先确认连接到 `192.168.1.56` 的 PX4 SITL
并收到实际遥测。将仿真飞机设置为地面上锁状态和 `Posctl` 模式。若 PX4 配置要求摇杆输入，
用模拟摇杆提供零油门输入，并确认 PX4 预检通过。从同一 Zenoh 网络运行以下命令。

```bash
./build/gcs_sim --plane-id uav1 --no-radar-udp --control-settle-ms 1000 --command-type SelfCheck
```

核对 `SelfCheckResult` 中的 `senderId`、`commandId`、`success` 和 `detail`。通过时核对
PX4 实际遥测的 `armed` 顺序为 `false → true → false`，自检全程保持在地面。用不支持的飞行模式
验证失败原因，确认机载未下发解锁。

## 7. 任务与通信验收

| 场景 | 通过标准 |
| --- | --- |
| 单一控制内容 | 任意时刻只有一个 Task 或 Command 执行 |
| 最新内容替换 | 新 Task/Command 按调度优先级中止并替换当前执行，自检只在空闲时启动 |
| 任务中止 | 只有匹配 taskId 的 Stop 生效，后续节点不再执行 |
| 控制报文 | 不同发送方的合法 TaskInfo 和 Command 均进入任务调度 |
| 接收确认 | 收到合法 TaskInfo 或 Command 后向 sendToStation 返回对应 messageId 的 Ack |
| 状态广播 | 按配置周期广播本机最新 VehicleState |
| 友机状态 | 按 planeId 更新和查询其他无人机最新状态，位置更新不打印日志 |
| 友机上下线 | Zenoh 存活标记出现时打印一次上线，撤销或失联时打印一次下线，恢复后重新打印上线 |
| 任务状态 | 地面站能获得状态、行为树和当前节点；无任务时为 None |
| GCS 链路 | 解码合法下行报文后变为 connected，超过阈值变为 disconnected |
| 队列上限 | 接收队列满时丢弃最旧消息，进程保持可用 |
| 确认重试 | 可靠消息超时后按配置重试，超过次数产生明确错误 |

## 8. 中制导验收

自动化和板端联调必须共同覆盖：

1. `ldd build/plane/plane` 包含 `libmidcourse_alg.so`，不包含 Pluma；算法库可由 `$ORIGIN/lib` 解析。
2. `AntiUav` 根据飞行状态选择直接 Midcourse、Takeoff → Midcourse 或 Arm → Takeoff → Midcourse。
3. 指定 ID 时锁定该目标；未指定时只允许唯一候选，不得在多个目标间自动跳换。
4. 机载从进程启动起持续缓存目标；任务启动前已经收到 20 帧时直接完成初始化，第 21 帧产生首次增量更新；规划器每个 50 ms tick 先于本 tick 的目标增量更新调用。
5. `[midcourse_algorithm]` 默认使用 INI 的四项限值，飞控缓存未就绪仍可配置规划器。
   `use_px4_limits = true` 时分别使用 `MPC_XY_VEL_MAX`、`MPC_Z_VEL_MAX_UP`、`MPC_Z_VEL_MAX_DN`、`MPC_ACC_HOR_MAX` 缓存值，
   缓存未就绪时任务失败，断连时缓存失效，重连后重新获取。
6. 使用 `VelocityAcceleration` 时，SITL 实际进入 Offboard。检查发往 SITL 的 `SET_POSITION_TARGET_LOCAL_NED`，
   三轴速度和加速度均有效，跟踪时至少一帧具有非零加速度，位置字段无效，航向使用弧度。使用 `Attitude` 时，
   校验 body→NED 四元数及推力，横向推力接近零，使用换算后的姿态和推力设定值。
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

## AgentInfo 信息接口验证

`agent_info` CTest 使用关闭组播和 gossip 的 TCP 回环会话，验证启动上报、多个地面站标记上线和重连、
重复标记去重、已有标记的 history 发现，以及 AgentInfo 查询。回环测试还验证不同发送方的控制报文分发
和 Ack，不连接飞控。

部署并在一号机验证：

```bash
python3 tools/fleetctl.py test-agent-info --count 1
```

该命令只预检一号机，备份程序和配置，部署完整运行包和 gcs_sim，保留现有连接参数及视频地址，
其中 SITL UDP 主机统一校正为清单中的 `192.168.1.56` 并验证实际飞控连接；
任务板 IP 未配置时写入清单中的该板 IP。gcs_sim 在板端独立进程中运行，验证：

1. 地面站已订阅时启动机载，收到一次 `AgentInfo`，观察期间没有周期重复。
2. 两套地面站身份在线时，均收到机载信息，机载也记录两个地面站的上报。地面站重连后再次收到上报。
3. 按 ID 查询机载与地面站，验证 `agentType`、地址及 `isSim`。独立查询期间机载上报计数为零。
4. 不存在的 Agent 返回无响应或超时，非法查询类型明确失败。
5. 板端二进制校验和依赖自检成功，机载保持运行，SITL 环境中验证飞控连接和遥测。

不重启 SITL、不发送控制报文、不连接或处理视频流。证据保存到
`logs/fleet/<run_id>/agent-info-report.json` 及对应原始日志。
