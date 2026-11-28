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
场景的 `start.sh` 以 `-n` 指定飞机数量，并输出逐机飞控、吊舱视频流和控制端口。
飞控 UDP 端口以本次运行包内预置为准，主机使用清单中的仿真主机；启动前检查对应端口实际监听，
启动后验证实际飞控连接和遥测。

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
- 飞控端口使用本次包内预置声明的值，并在每次启动后验证对应 UDP 端口均已实际监听。所有端点都必须通过数量和唯一性校验。

只做连接和环境预检：

```bash
python3 tools/fleetctl.py preflight --count 2
```

启动两架飞机并验证双方通过 Zenoh liveliness 感知到友机在线：

```bash
python3 tools/fleetctl.py test --count 2
```

全量部署双机并验证上下线及日志降频，不重启 SITL；飞控使用包内预置端口和清单中的仿真主机：

```bash
python3 tools/fleetctl.py test-presence --count 2
```

机间连接由部署预置生成，单次主动建链超时使用随包的 2 秒配置。
SITL UDP 端口取包内预置，主机统一使用 `192.168.1.56`。

该命令预检和操作清单中的前两块板卡，备份旧程序及配置，验证双向正常退出、恢复上线，
再终止二号机进程以验证异常退出及恢复。初始和结束各观察 10 秒，检查重复上下线和位置更新日志。
报告及各阶段原始日志保存在 `logs/fleet/<run_id>/`，成功后双机保持运行。

用户按包内预置手动启动 `192.168.1.56` 的 PX4 SITL（默认一号机端口为 `14280`）后，部署一号机并验证中制导：

```bash
python3 tools/fleetctl.py test-midcourse --count 1 --target-track-id <track_id>
```

该命令不会启动或停止 SITL；以下 N 表示包内 `midcourse.target_history_size` 的配置值。完成板端自检后会提示持续向
`192.168.1.100:7790` 发送至少 N+1 个时间戳严格递增的 `radar_tracks` 扫描。7790 只接收雷达数据，防守机状态由 GCS 从
Zenoh 获取，雷达帧无需包含防守机航迹。空扫描和 `coasting` 航迹会被合法接收但不发布为
`GcsTarget`；测试所需的 N+1 帧必须包含指定 `track_id` 的 `confirmed` 航迹。运行日志中的
雷达接收统计用于区分 UDP 未到达、合法但无靶机输出的扫描和已发布靶机。

用户通过地面站自行下发反无任务时，使用人工发令模式：

```bash
python3 tools/fleetctl.py test-midcourse --count 1 --manual-task
```

该模式全量部署 uav1 到 `/home/firefly/plane`，使用既有 SITL 并核对飞控连接、实际遥测、通信和日志采集。
观测 GCS 不接管雷达 UDP 输入，不发布目标，也不发送 Task、Command 或 Task Stop。
确认“环境已就绪，可以开始下发反无任务。”后，由用户保持目标雷达更新并下发任务，
按实际 `taskId` 和目标航迹验证初始化、Offboard 持续控制，并采集至少 5 秒稳定的 1 Hz 日志及持续目标更新。
正常阶段通过后提示用户暂停该目标更新，验证严格超过配置阈值后的失败日志、Offboard 停止、
会话清理和 `TaskState::Failed`；恢复目标数据后旧任务不得重启。报告及原始证据保存在
`logs/fleet/<run_id>/`。部署就绪只表示具备发令条件，任务验收结果以实际执行证据为准。

如果部署已启动但监控会话中断，可复核并重写该次报告，不重启现场：

```bash
python3 tools/fleetctl.py verify --count 2 --run-id <run_id>
```

停止本次环境：

```bash
python3 tools/fleetctl.py down --count 2
```

`test` 使用完整的 `build/plane`。停止现有机载进程前，先完成运行包及配置引用的校验；
包内 `profile` 必须非空、为包内相对路径且引用实际存在的预置。随后按以下顺序准备板端环境：

交叉编译运行包无法在代码机执行时，本地依赖自检延后到目标板；每次部署仍须在启动程序前于目标板完成
动态库解析检查和 `./plane --deps-check`，失败时不得启动。

1. 校验板端架构与 Ubuntu 版本，再把清单中的代理写入板端 `/etc/apt/apt.conf.d/99zz-fleet-proxy`。
2. 按清单补装核心运行依赖。
3. 重启 SITL，解析飞控地址。
4. 从本次包内配置生成部署目录，全量同步到 `/home/firefly/plane`，启动并持续分析日志。

程序、动态库、脚本、主配置、Zenoh 原生配置和全部预置均来自本次完整运行包。
工具不读取板端配置，不为旧配置格式或版本提供迁移、合并及继承路径。
全量同步覆盖全部预置，删除包外遗留部署文件，只保护 `logs/` 及其中的 `logs/core/`。
部署前保留整目录回退备份；备份不参与本次配置生成。

没有配置地面站时，Zenoh 仍可通过预置中的机间端点通信。测试证据写入 `logs/fleet/<run_id>/`；验证通过后环境保持运行，以便继续检查，失败时也保留现场并提示使用 `down` 停止。

板端运行日志固定写入 `logs/plane.log`，不创建日期目录，单文件达到 50 MB 后轮转为
`plane.1.log` 至 `plane.5.log`。`logs/plane.stdout.log` 保存本次启动的标准输出和错误输出。
`verify --run-id` 将当前板端日志采集到指定的本地证据目录，不用于读取板端历史运行。
普通 `[info]` 日志默认每 1000 ms 刷新，周期由 `[logging].flush_interval_ms` 配置；
`[warn]`、`[error]` 和任务、指令、自检结束摘要立即刷新。前台和 `--no-console` 模式规则相同。

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
2. 从 `tests/fleet.ini` 选取 `uav1`（`192.168.1.100`），在运行目录外备份现有完整运行目录。在本次包内主配置中选定 `sitl_jielian_a2416a.ini` 预置，全量部署整个运行包到 `/home/firefly/plane`，再执行 `./runplane.sh`。
3. 确认机载监听 `192.168.1.100:7447`，主动连接正在 TCP `7447` 监听的地面站 `192.168.1.41`，以及其他飞机。检查地面站收到 `uav1` 的 `AgentInfo` 和持续的 `VehicleState`。可从同一网络显式连接机载并查询。

```bash
./build/gcs_sim --connect tcp/192.168.1.100:7447 --query AgentInfo --agent-id uav1 --no-radar-udp --listen-seconds 5
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

`zenoh_liveliness` 在本机关闭组播和 gossip，使用两个互相拨号的机载 peer 和一个地面站 peer 验证 TCP 单播收发、`AgentInfo` 查询、退出下线和重连上线。
`state_store_quiet` 验证连续状态更新仍发布内部事件并更新缓存，但不产生日志。
`target_store_log` 使用可控时间验证目标首帧与 20 秒汇总、1999／2000 毫秒缺失边界、
持续缺失、恢复、任务前陈旧缓存、从未收到、错误航迹和无效数据不续期、自动需求及停止清理。
`task_target_monitor` 使用真实目标管理与任务执行引擎，模拟 Arm／Takeoff 同步阻塞超过 4 秒，
验证缺失提示独立运行，且取消需求先于等待行为树退出；任务失败、替换和独立 Command 不残留提示。
`serialization_vehicle_state` 验证 `inair` 默认为 `false`、`true`/`false` 布尔值往返、
缺少字段时覆盖旧值为 `false`，以及数字、字符串、`null`、数组和对象值被拒绝。
部署工具按最新运行中的上下线事件验证在线状态，不再把日志条数当作收到的位置样本数。

默认运行包须包含 `config/zenoh.json5`，由 `plane.ini` 的 `zenoh_config_file` 加载，
`transport/unicast/open_timeout` 为 `2000` 毫秒。机群预置生成 `zenoh_connect`，原生配置默认开启组播发现和 gossip。
修改该配置后，在一号机验证 AgentInfo 上报与查询、正常停止及恢复启动，
记录实际停止耗时，并确认没有触发停止脚本的强制结束。

运行日志检查须确认项目人工编写的消息和业务描述均使用中文，不出现英文句子；API 名、协议名、
类型名、枚举值、配置键、函数参数名、结构体及其字段名和第三方库原始错误等代码标识保留原文。
检查 PX4 飞控系统参数缓存日志时，四项键必须为 `MPC_XY_VEL_MAX`、`MPC_Z_VEL_MAX_UP`、
`MPC_Z_VEL_MAX_DN`、`MPC_ACC_HOR_MAX`，不得使用中文意译键名。
日志级别继续使用 spdlog 的 `[info]`、`[warn]`、`[error]` 等原始英文标识。主程序和 `gcs_sim`
应使用相同格式，线程 ID 只显示为 `[%t]`，不得带“线程”前缀，且格式中不自动输出英文函数名。


`config_simulation` 验证场景 02 的 PX4 远端 UDP 端口 `14030`～`14039`、本地端口
`14280`～`14289` 与旧 SITL 端口 `14580`～`14589` 为仿真，范围外端口和串口不判定为仿真。
配置测试还需覆盖同一预置按不同本机 IP 选择不同飞机 ID 和 RTSP、UDP 与串口飞控、
仅包含一架飞机的预置，以及 `profile` 缺失或为空、配置读取失败、重复 ID/IP、必填项无效、零匹配和多匹配。
配置加载失败必须保留原有效配置，切换预置时不得残留另一种飞控连接的信息。
中制导配置测试覆盖巡航与前馈参数、重规划周期与冻结时长、内部末端确认及其阈值、
`recovery_vz_deadband_m_s`、`recovery_altitude_tolerance_m` 的默认值与自定义值、
`target_history_size` 的默认值 20、自定义边界值 5 和 30、越界值及非法整数输入、
`update_dt_s` 与行为树周期一致和不一致的情况，以及省略 `update_dt_s` 时的周期换算。
`fleetctl` 单测验证测试清单默认路径、`--config` 覆盖及完整运行包的唯一配置来源：
板端已有配置损坏、格式陈旧或为空均不影响有效包部署；包内 `profile` 缺失、为空、引用包外文件或预置无效时，
必须在停止板端程序前拒绝部署。验证包内 `safe_height_m = 20` 覆盖板端的 `30`，
全部预置被覆盖更新、包外遗留文件删除、`logs/` 和 `logs/core/` 保留。
SITL 使用包内预置端口及清单主机并检查实际监听，串口连接与包内预置一致。

`radar_target_filter` 位于 `tools/gcs_sim/tests`，验证不带速度和方位角的雷达航迹、JSON 容错、
空扫描、coasting 忽略、confirmed 过滤、严格时间戳输入、多友机全局匹配、
50 米门限、NED 状态外推、过期状态失败关闭，以及“友机未出现在雷达帧时仍转发靶机”。
`midcourse_controller` 使用真实 `libmidcourse_alg.so`，验证任务前历史不参与本次初始化、从反无任务启动起按配置累计 N 帧、增量更新、
INI／PX4 限值来源、两种输出模式、会话重置和错误路径，以及目标更新超时的初始化、运行和边界场景；
独立日志测试使用可控单调时间、日志捕获与算法门面测试替身，验证 1 Hz 成对日志的 500／1000 ms 边界、
首次调用、失败立即输出、会话重启和逐 tick 调用数量；真实算法库回归用于验证集成行为。
`attitude_conversion` 验证四元数归一化与姿态转换。
按板端 `logs/plane.log` 的时间顺序检查聚合门面初始化、Offboard 启动和首次增量更新。
生效配置应在等待目标历史前完成校验，初始化调用日志记录限值来源和传入的全部字段。
接口失败时保留原始错误字符串。

```bash
ctest --test-dir build --output-on-failure
python3 tests/unit/test_fleetctl.py
```

#### 日志刷新验证

```bash
ctest --test-dir build -R '^(logging_flush|config_simulation|task_scheduler_takeover|midcourse_controller_log)$' --output-on-failure
```

- `config_simulation` 覆盖 `logging.flush_interval_ms` 的默认值 1000、自定义正整数、零、负数及非法输入；
  加载失败保留原有效配置。
- `logging_flush` 使用真实文件分别验证前台与 `--no-console`：只写一条 `[info]` 后停止输出，
  内容仍按周期可读；`[warn]`、`[error]` 写入返回后即可读取；正常退出保留末尾日志。
  直接构造无效 `LogOptions` 不得启动刷新线程，重新初始化须应用新周期、回收旧线程，不重复刷新或妨碍退出。
- `task_scheduler_takeover` 覆盖 Task、Command、自检的成功、失败和中止结束摘要，
  以真实文件或 sink 刷新计数确认立即刷新，不依赖下一条警告或错误触发。
- `midcourse_controller_log` 回归 1 Hz 日志的数量、配对和失败即时记录，确认刷新周期不改变算法调用或控制频率。
- 在 uav1 以相同日志内容、速率、条数和终端输出模式，分别采样修改前后各 60 秒，
  记录单核 100% 口径的进程 CPU 平均占用、刷新次数与日志条数；CPU 增量目标不超过 1 个百分点。
  定时刷新按配置周期统计，警告、错误、终态和退出的额外刷新单独解释。
- 本地通过后全量部署到 `/home/firefly/plane`，验证 `logs/plane.log` 的实际可见延迟及关键摘要，
  并确认清单指定 SITL 主机、包内端口的实际连接和遥测。实际反无任务由用户下发，未执行项明确报告。

将本次构建的 `logging_flush_test` 放入 uav1 的独立临时测试目录，在该目录执行受控负载基准：

```bash
log_benchmark_dir=$(mktemp -d /tmp/plane-log-flush.XXXXXX)
LD_LIBRARY_PATH=/home/firefly/plane/lib ./logging_flush_test --benchmark \
  --duration-seconds 60 --rate-hz 50 --console --log-dir "$log_benchmark_dir" \
  > "$log_benchmark_dir/report.json"
```

该进程先采样基线 60 秒，再采样定时刷新策略 60 秒。`--console` 对应前台模式；省略时对应不带终端 sink 的模式。
JSON 报告记录各阶段 CPU、刷新次数和消息数；阶段进度输出到标准错误，终端日志保存于测试目录的 `console.log`。
基准只写临时日志，不发送飞控或网络指令，测试证据须归档到本轮验证目录。

本节检查 `fflush` 后的文件可见性，不将结果解释为断电持久化保证；无需注入掉电故障。

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
- 首条进度回调到达前日志提示等待，收到后每 5 秒记录当前航点，飞向第 10 个航点时显示 `10/100`。
  飞向最后一个航点时显示 `100/100`，完成时记录节点结果。
- 板端实际执行航点或航线时，每次 `MissionProgress` 回调均记录 MAVSDK 原始 `current` 和 `total`。

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
- 高度调整使用 `behavior_tree.node_timeout_ms`，常规中止和超时时切换 Hold，人工接管中止时保留飞手选定的模式；
- 起飞节点使用指定高度或默认安全高度下发起飞指令，观测到 `Takeoff` 后等待 `Hold` 和 `InAir` 才判定完成。
- 起飞运行日志每 5 秒输出 `geoPos.height` 和 `geoPos.altitude`，分别为相对高度和海拔，固定保留 1 位小数。
  高度来自本机状态缓存，高度为零时显示实际数值。飞控未连接或高度数值无效时显示「位置遥测不可用」。
  完成日志记录两项高度，运行及结果日志均省略 tick 次数。

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
| 飞控状态文本 | 飞控连接后产生 `STATUSTEXT`，在 `logs/plane.log` 中检查 `[info]` 的 `飞控 STATUSTEXT` 日志，`type` 和 `text` 与飞控消息一致，原文不翻译；不同类型的消息均输出 |
| 仿真 GCS 预检 | 场景 02 或旧 SITL 中启动 plane 后，PX4 不再报告 `Preflight Fail: No connection to the GCS`；日志包含 `GroundStation` 身份，随后 Arm 可被接受 |
| 串口连接 | 使用配置的设备和波特率连接 PX4 |
| Arm/Disarm | 指令成功且以真实 armed 状态判定完成 |
| SelfCheck | 实际解锁后自动恢复上锁，飞机保持在地面，地面端收到可关联的成功或失败结果 |
| Takeoff | 使用指定高度；未指定时使用安全高度；进入 `Takeoff` 后确认在空中（`InAir`）且切换回 `Hold` 模式后完成，默认超时 600 秒 |
| Land/RTL | 以实际离地状态判定完成 |
| Hover | PX4 接受 Hold 指令 |
| Waypoint | 上传单航点并以 Mission Progress 判定完成 |
| Route | 未起飞时自动解锁并起飞；开始前调整到首航点相对高度；随后按顺序执行多航点并以 Mission Progress 判定完成；飞行超过普通节点超时仍继续等待完成，新指令仍可替换航线 |
| StopMotor | 仅接受地面站人工指令，不由任务自主触发 |
| Attitude Offboard | 先发送合法姿态／推力设定值再启动 Offboard，按 50 ms 重发；起飞完成切换覆盖 Offboard 时能够限频恢复；停止任务时停止 Offboard 并清空缓存 |

实机执行 StopMotor 和撞击相关测试前必须确认安全区域和测试授权。

### 6.1 人工接管

运行 `ctest --test-dir build -R '^(manual_takeover|task_scheduler_takeover)$' --output-on-failure` 验证模式基线、快速切换、起飞结束的正常 `Hold` 交接、航点完成遥测核对、接管前后模式、任务中止，以及接管操作日志和告警的单次记录。

将机载程序部署到 `uav1` 的 `/home/firefly/plane`。
连接 `192.168.1.56` 的 PX4 SITL，使用 `mavlink status` 确认连到 `uav1` 的机载链路，再在该 PX4 实例的控制台调高机载链路 `HEARTBEAT` 频率。命令中的端口取 PX4 侧实例的本地 UDP 端口。

```console
mavlink stream -u <机载链路的PX4本地UDP端口> -s HEARTBEAT -r 20
```

抓取机载实际收到的 `HEARTBEAT`，核对其模式和接收间隔。用 SITL 模式控制模拟三档拨动，约 100 ms 的短时切换以实际收到的模式帧为准。实体遥控器的三档映射在台架上复验。

| 场景 | 通过标准 |
| --- | --- |
| 飞前 `Position`，随后由程序接手 | 正常起飞并进入预期自动模式。启动和重连时的首个模式不触发接管。 |
| 正常 `Hold` | 起飞完成后的 `Takeoff → Hold` 直接按正常交接处理，不受遥测晚到影响；`Hover`、临时高度航点与正式航点正常结束均不记录接管操作日志或发送接管告警。起飞尾段短暂覆盖 Offboard 后仍可正常交接。 |
| `Mission` 或 `Offboard` 中切入 `Loiter`、`Stabilized`、`Position` | 逐个独立验证三个入口，检查 PX4 的实际模式。接管操作日志和 `manual_takeover` 告警均包含前后模式 `previous`、`value`、停止自动控制操作，以及被中止的 `taskId` 或 `commandId`，二者内容一致。航线未完成时进入 `Loiter`，在核对窗口结束前暂停下发，随后中止。接管后的出站抓包没有旧飞控指令、Offboard 设定值或切换 `Hold` 的清理报文。 |
| `Hold` 核对期间快速切换 | `Mission → Hold → Mission` 核对超时后仍按首次 `Mission → Hold` 判定，日志与告警保留 `previous=Mission value=Hold`。另行验证 `Mission → Hold → Stabilized` 时立即接管，记录 `previous=Hold value=Stabilized`。 |
| 快速拨动 `Loiter → Stabilized → Position` | 第一次符合接管条件即中止。同一次执行只产生一条接管操作日志和一条 `manual_takeover` 告警，Task 上报 `Aborted`。旧报文重传只回 `Ack`，清理完成后新的 Task 或 Command 可以执行。 |
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
| 离地状态 | uav1 连接清单指定的 SITL，订阅 `sendToAll`，确认 `VehicleState.inair` 是布尔值，随地面、起飞、降落呈 `false → true → false`；`gcs_sim` 的飞机状态日志输出 `inair`，同时核对 `fcLinkState=connected` |
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
4. `[midcourse].target_history_size` 默认 20，范围 5～30，同时决定每航迹缓存深度和初始化所需帧数 N。
   机载从进程启动起持续缓存目标，但任务前缓存不参与本次算法初始化。从反无任务启动起为锁定航迹累计
   N 帧有效且时间戳严格递增的数据，Arm／Takeoff 期间收到的数据参与累计；收齐前不得调用
   `MidcourseAlgorithm::init` 或启动 Offboard，进入 Midcourse 时已收齐且数据未超时则立即初始化并启动 Offboard；
   第 N+1 帧产生首次增量更新。`MidcourseAlgorithm::update` 每个 50 ms tick 先于本 tick 的 `updateTarget` 调用。
5. `[midcourse_algorithm]` 默认使用 INI 的四项限值，飞控缓存未就绪仍可校验并初始化聚合算法。
   `use_px4_limits = true` 时分别使用 `MPC_XY_VEL_MAX`、`MPC_Z_VEL_MAX_UP`、`MPC_Z_VEL_MAX_DN`、`MPC_ACC_HOR_MAX` 缓存值，
   缓存未就绪时任务失败，断连时缓存失效，重连后重新获取。
6. 使用 `VelocityAcceleration` 时，SITL 实际进入 Offboard。检查发往 SITL 的 `SET_POSITION_TARGET_LOCAL_NED`，
   三轴速度和加速度均有效，跟踪时至少一帧具有非零加速度，位置字段无效，航向使用弧度。使用 `Attitude` 时，
   校验 body→NED 四元数及推力，横向推力接近零，使用换算后的姿态和推力设定值。
7. `[midcourse].target_timeout_ms` 默认 5000 ms，自定义正整数值生效，零、负数及非法输入加载失败并保留原配置。
   目标超时检查从进入 Midcourse 起生效；本次任务已有目标数据时按最后接收时间检查，尚无数据时首帧必须在 5 秒内到达。
   之后每帧本次任务接受的锁定航迹有效更新都重新开始 5 秒计时；恰好等于阈值时继续，
   严格超过阈值后下一次 BT tick 失败，失败 tick 不再调用算法或下发控制。重复、乱序和其他航迹数据
   不计入本次 N 帧，也不刷新计时。
   默认配置下，3 秒预测宽限期结束后至目标更新超时前，任务继续执行算法保持指令；
   超时后停止 Offboard、清空设定值、重置会话并取消目标需求，上报一次关联原 `taskId` 的 `Failed`。
   检查中文日志包含 `trackId`、未更新时长、实际阈值及本次会话从未收到或目标更新中断的说明；
   恢复目标更新不得恢复旧任务。持续有有效数据但不足 N 帧时，原 30 秒历史初始化上限仍生效，
   Arm／Takeoff 不受目标更新超时限制。Task Stop、新控制内容和异常退出均停止 Offboard 并重置会话。
8. GCS 从 Zenoh 获取防守机状态，只过滤 50 米内实际匹配到的雷达航迹；雷达帧没有友机航迹时仍转发靶机，状态缺失或过期时暂停整帧。
9. 使用非法捕获窗口触发提前配置校验失败，确认不等待 N 帧历史且失败原因保留原始校验错误，
   会话结束后清空 Offboard 设定值，`gcs_sim` 收到一次关联原 `taskId` 的 `TaskState`，
   其中 `status = "Failed"`、`currentBehaviorTree = "AntiUav"`、`currentNode = "Midcourse"`。
10. 按调用顺序检查 `[info]` 的算法输入输出日志：`MidcourseAlgorithm::init` 的全部 N 帧 `history`、
    `ned_ref`、完整 `MidCourseConfig`、返回值和原始 `error`；`MidcourseAlgorithm::update` 的完整
    `vehicle`（含实际 `yaw_rad`）和 `output`；`MidcourseAlgorithm::updateTarget` 的 `measurement`、返回值和原始 `error`。
    检查所有地理点的 `latitude`、`longitude`、`height`、`altitude`，
    两种控制指令的全部字段、`yaw_rad=nullopt`、`eta_seconds`、`following`、`terminal_confirmed`。
    `init/updateTarget` 每次调用均记录；`update` 输入输出成对按 1 Hz 记录，会话首次调用和调用失败立即记录，
    可控时间验证首拍立即记录、500 ms 不增加日志、1000 ms 增加一对日志；不足 1000 ms 时发生 `update` 失败也立即记录，
    stop／begin 后重新首拍记录。算法调用及控制下发仍逐 tick 执行，并通过调用计数核对。
    每组日志均保留返回值和原始 `error`，失败时不将出参标为有效值，
    高频日志不附加会话号、调用号或更新号；
    机载不调用 `getTelemetry()`。
    可在板端使用已部署的真实算法库运行 `midcourse_controller_test` 核对日志；该测试使用模拟飞控，
    不向飞控发送控制指令，不能替代实际飞行任务验收。

板端验收只有在用户确认 SITL 已启动后才允许发送控制；网络、权限或仿真环境缺失时必须报告阻塞。
报告保存在 `logs/fleet/<run_id>/midcourse-report.json`，并保留各阶段 GCS 和机载原始日志。

## 9. 已移除任务流程

原本依赖本地视觉检测的任务启动时必须明确返回不可用，不启动截短的任务树，也不误报完成。

## 10. 目标和安全验收

| 场景 | 通过标准 |
| --- | --- |
| GCS 目标 | 按 trackId 保存最新值，每个目标最多保留 `midcourse.target_history_size` 条历史 |
| 目标日志汇总 | 每航迹首帧立即输出，约 1 Hz 连续更新至少 45 秒时后续约每 20 秒汇总，计数正确；无新数据不输出汇总 |
| 所需目标缺失 | AntiUav 创建满 2 秒且所需航迹连续 2 秒无有效更新后，每 2 秒输出提示，包含真实接收时间、时区偏移、timestamp_ms 和未更新时长 |
| 目标提示边界 | 错误航迹、重复或乱序数据不续期；从未收到明确标注；恢复后重新计时；停止、替换、接管、失败及完成后不再提示 |
| 历史就绪判断 | fleetctl 按 `midcourse.target_history_size` 验证匹配航迹从反无任务启动起累计 N 帧，包含 Arm／Takeoff 期间数据，不使用任务前缓存，不累计日志条数，也不跨航迹累计 |
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

## Alert 诊断上报验证

构建后运行相关自动化测试：

```bash
ctest --test-dir build -R '^(flight_control_diagnostics|alert_queue|task_scheduler_takeover|mission_node_timeout|route_preflight|self_check_node|midcourse_controller|agent_info)$' --output-on-failure
```

| 场景 | 通过标准 |
| --- | --- |
| 飞控文本 | `Debug` 不转发；`Info` 及以上保留原始级别和完整文本。同级同文 5 秒内只报一次，不同级别或文本立即上报，重连后可重新上报。 |
| 启动与队列 | 通信启动前可缓存 Alert，启动后按入队顺序发送；最多保留 256 条，溢出淘汰最旧消息且记录本地日志。停止后拒绝入队，并发发布不死锁。 |
| 指令与任务生命周期 | 开始先于结束；正常完成、启动失败、执行失败、超时、停止、替换及接管均有正确 ID、类型、状态与原因。自然结束和中止竞争、重复接管及迟到结果不重复产生终态。 |
| 任务内飞控动作 | Arm、Takeoff、自检、航线上传/启动等实际低频调用前有开始信息；SDK 拒绝或前置条件失败有具体原因。正常接管阻止旧执行不产生重复错误。 |
| 失败详情 | 派发失败保留原始错误，节点超时保留节点和超时信息，明确的完成条件失败及中制导失败传入结束详情。 |
| 状态基线与变化 | 每次连接首份有效模式、armed 和 inair 按当前状态上报，随后仅变化时上报；断开、恢复可见，旧连接回调不污染新状态。 |
| Offboard 降噪 | 连续运行不逐帧上报设定值；连续失败只报首次和原因变化，接口恢复成功后报一次。启动、重启和停止属于低频操作。 |

板端联调使用本次完整运行包部署到 uav1，连接 `tests/fleet.ini` 中的 SITL 主机，端口以包内预置为准。
通过实际 Zenoh 订阅接收 `Alert` 的 `type`、`detail`，不能以机载本地日志替代上报证据。
`gcs_sim` 解码并打印 Alert 的类型和详情，可作为联调接收端。
验证实际飞控连接及有效遥测，再检查独立指令、任务开始/结束、任务内动作、飞控文本和状态变化。
飞控拒绝、断连与 Offboard 故障通过受控仿真或自动化故障注入验证；未触发的场景须明确标记未执行。
相关回归仍检查 `TaskState`、`SelfCheckResult`、`manual_takeover` 和低电压原有规则。
Alert 采用尽力发送，测试不要求持久化、断网历史补发或进程退出期间必达。

## AgentInfo 信息接口验证

`agent_info` CTest 使用关闭组播和 gossip 的 TCP 回环会话，验证启动上报、多个地面站标记上线和重连、
重复标记去重、已有标记的 history 发现，以及 AgentInfo 查询。回环测试还验证不同发送方的控制报文分发
和 Ack，不连接飞控。

部署并在一号机验证：

```bash
python3 tools/fleetctl.py test-agent-info --count 1
```

该命令只预检一号机，保留整目录回退备份，全量部署本次运行包和 gcs_sim。
SITL UDP 端口来自包内预置，主机统一为清单中的 `192.168.1.56`，并验证实际监听、飞控连接和遥测；
节点身份及视频地址从包内预置读取，主配置必须引用包内可匹配板卡的预置。
gcs_sim 在板端独立进程中运行；测试地面站之间显式建立回环 TCP 连接，等待查询所需的身份信息和连接就绪后再执行查询。验证：

1. 地面站已订阅时启动机载，收到一次 `AgentInfo`，观察期间没有周期重复。
2. 两套地面站身份在线时，均收到机载信息，机载也记录两个地面站的上报。地面站重连后再次收到上报。
3. 按 ID 查询机载与地面站，验证 `agentType`、地址及 `isSim`。独立查询期间机载上报计数为零。
4. 不存在的 Agent 返回无响应或超时，非法查询类型明确失败。
5. 板端二进制校验和依赖自检成功，机载保持运行，SITL 环境中验证飞控连接和遥测。

不重启 SITL、不发送控制报文、不连接或处理视频流。证据保存到
`logs/fleet/<run_id>/agent-info-report.json` 及对应原始日志。
