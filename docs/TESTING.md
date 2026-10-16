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
1）192.168.1.56 aeroarx/1, 登录后到home/aeroarx/zsh/PX4-Autopilot/scenarios/02_rocket_two_axis_pod执行
    先停止，再启动全新的环境
    ./start.sh 脚本启动测试仿真环境，默认3架， -n 参数用于指定启动的飞机架次；
    ./stop.sh停止仿真环境
2）start脚本执行后会输出几架飞机使用的飞机url、吊舱视频流和吊舱控制端口，从中解析出信息来修改飞机中对应的配置文件

#### 测试环境
1）每架机载对应一块板子
2）1号飞机，192.168.1.100, firefly/firefly
   2号飞机，192.168.1.101，firefly/firefly
3）机器可读清单以 `config/fleet.ini` 为准。请求飞机数超过其中声明的板卡数时，必须在启动任何程序前报告环境不足。

#### 多机自动化

执行约束：

- 用户要求启动或验证多架飞机时，必须使用 `python3 tools/fleetctl.py`，不得临时拼接远端命令绕过预检。
- 环境清单以 `config/fleet.ini` 为准，不得扫描或猜测未声明的板卡。
- 请求数量超过清单声明数量时，只报告环境不足，不启动部分 SITL 或机载程序。
- 板端架构或 Ubuntu 版本与 `fleet.ini` 不一致时，必须在修改远端环境前终止，不得跨发行版混装 APT 依赖。
- SITL 端点必须从本次启动输出动态解析，并通过数量和唯一性校验，不得使用猜测的端点。

只做连接和环境预检：

```bash
python3 tools/fleetctl.py preflight --count 2
```

启动两架飞机并验证双方至少连续解析到 3 个友机状态样本：

```bash
python3 tools/fleetctl.py test --count 2
```

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
2. 先安装 `libopencv-dev`，再补装其余 APT 运行库；
3. 安装 `build/plane/rknn/` 中的 RKNN 用户态库并运行 `ldconfig`；
4. 重启 SITL，实时解析本次输出中的飞控和 RTSP 地址；
5. 同步到 `/home/firefly/plane`，生成逐机配置，启动并持续分析日志。

Zenoh 使用默认发现，不依赖 GCS 地址。测试证据写入 `logs/fleet/<run_id>/`；验证通过后环境保持运行，以便继续检查，失败时也保留现场并提示使用 `down` 停止。

### 2.2 软件出包

执行媒体测试前应具备：

- 已构建的 `build/plane/plane`；
- `build/plugins` 下所需插件；
- 可用的检测模型和算法配置；
- `ffprobe`、`ffmpeg` 和 `curl`；
- RTSP/WebRTC 所需网络端口未被占用。
- 如果是sitl测试要检查仿真是否正常启动了

测试脚本支持以下环境变量：

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `PLANE_BIN` | `build/plane/plane` | 待测试程序 |
| `PLANE_PLUGINS_DIR` | `build/plugins` | 插件目录 |
| `PLANE_FIXTURES_DIR` | `tests/fixtures` | 测试配置目录 |
| `SKIP_HW` | `0` | 为 `1` 时跳过脚本中标记的硬件场景 |

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

### 4.1 插件加载契约

```bash
bash tests/contract/test_plugin_load.sh build
```

验证 `build/plugins` 中的 ImageInput、DetMot 和 VideoStreamOutput Provider 均可被宿主发现和实例化。

### 4.2 插件异常降级

```bash
bash tests/integration/test_failfast.sh
```

验证插件目录缺失、检测模型不可用时：

- 输出包含插件名称和失败原因的错误日志；
- 不输出插件 `[fatal]` 日志；
- 核心程序进入就绪状态并保持运行。

### 4.3 配置切换

```bash
bash tests/integration/test_config_switch.sh
```

验证只修改配置即可切换：

- RTSP 输入与 MIPI 输入；
- RTSP 输出与 WebRTC 输出；
- 插件实际收到的类型和参数与配置一致。

### 4.4 取图、检测和推流日志

```bash
bash tests/integration/test_pipeline_logs.sh
```

通过标准：日志能够确认插件加载、取图初始化、检测初始化、推流初始化、首帧、检测结果、推流地址和推流状态。无观看者时允许推流插件进入 Ready 并暂停编码。

### 4.5 视频流

```bash
bash tests/integration/test_pipeline_stream.sh
```

验证：

- RTSP 输出可由 `ffprobe` 拉取，编码为 H.264，分辨率符合测试配置；
- RTSP 输出可由 `ffmpeg` 连续解码并产生视频帧；
- WebRTC 观看地址可访问，返回 HTTP 200、301 或 302。

### 4.6 航线执行超时回归

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build --target mission_node_timeout_test -j
ctest --test-dir build -R '^mission_node_timeout$' --output-on-failure
```

使用模拟飞控与测试时间源验证，无需仿真飞控或真实等待：

- Route 在等待首次进度、飞行时间超过普通节点超时和长时间运行时均保持 RUNNING，直到任务进度完成。
- Waypoint 仍遵守普通节点超时。
- 长航线仍支持中止和重新启动，飞控分发失败仍返回 FAILURE。

### 4.7 航线起飞与高度调整

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

## 5. 媒体性能场景

推流输出目标帧率约为 30 FPS。测试时必须记录：

- 输入源帧率；
- 检测耗时或检测吞吐；
- 分辨率转换平均耗时；
- 编码和输出帧率；
- 是否存在断流、重连或无观看者暂停编码。

当前不设未经确认的固定容差。若实测帧率明显偏离 30 FPS，不能只报告失败，必须分析瓶颈属于取图、缩放、检测、叠加、编码、网络还是观看端，并向用户说明测量值和证据。

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
| NED 控制 | Offboard 持续发送速度，停止任务时退出 Offboard 控制 |

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
| 友机状态 | 按 planeId 更新和查询其他无人机最新状态 |
| 任务状态 | 地面站能获得状态、行为树和当前节点；无任务时为 None |
| GCS 链路 | 收到 GCS 报文变为 connected，超过阈值变为 disconnected |
| 队列上限 | 接收队列满时丢弃最旧消息，进程保持可用 |
| 确认重试 | 可靠消息超时后按配置重试，超过次数产生明确错误 |

## 8. 反无人机任务验收

完整目标流程为：

```text
Arm → Takeoff → Midcourse → Detection → Tracking → Guidance/Impact
```

逐阶段验证：

1. 指定 `targetTrackId` 时选择对应 GCS 目标；未指定时只有唯一目标才允许自动选择。
2. 中段制导根据本机与目标位置生成 NED 速度和朝向。
3. 到达配置的检测启动距离后进入 Detection。
4. Detection 输出有效目标后按规则选择跟踪目标。
5. Tracking 持续更新目标；超过丢失阈值后任务失败。
6. Guidance 输出被转换为飞控控制；达到撞击完成条件后任务完成。
7. 任一阶段超时或失败时停止后续阶段并清理检测及速度控制。

当前代码只实现到 Detection。Tracking 和 Guidance/Impact 场景应标记为“未实现”，不得标记为测试通过。

## 9. 感知、目标和安全验收

| 场景 | 通过标准 |
| --- | --- |
| MIPI 输入 | 能自动发现或使用指定设备持续获取图像 |
| RTSP 输入 | 能连接、断流后按配置重试并恢复取图 |
| 分辨率转换 | 输出尺寸等于检测输入尺寸，ARM64 优先使用 RGA |
| 检测/MOT | 插件输出被保存为最新视觉目标并叠加到视频 |
| GCS 目标 | 按 trackId 保存最新值，每个目标最多保留 20 条历史 |
| 动态安全高度 | SystemParam 更新后，后续默认起飞使用新值 |
| 实机低电压 | 飞控连接正常且首次低于阈值时发送一次告警 |
| 电压恢复 | 电压恢复后解除锁存，再次低电压可重新告警 |
| 仿真低电压 | 仿真飞机不发送低电压告警 |
| 插件初始化/配置失败 | 记录插件和原因，禁用对应能力，核心程序保持运行 |
| 插件运行失败 | 记录插件和原因，有序关闭并返回非 0 |

## 10. 测试报告

交付时至少记录：

```text
构建：通过/失败/未执行，命令与结果
自动化测试：逐项状态
硬件与联调测试：逐项状态、环境和设备
媒体指标：分辨率、编码、输入/输出 FPS
未执行项：原因和风险
失败项：日志位置、现象和初步定位
```
