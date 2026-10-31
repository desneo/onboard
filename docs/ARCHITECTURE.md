# 系统架构与工程规范

> 本文定义实现架构、模块边界、工程目录、编码依赖、验证交付和 Git 规范。业务要求见
> [`SPEC.md`](SPEC.md)，使用方法见 [`../README.md`](../README.md)，测试细则见
> [`TESTING.md`](TESTING.md)。修改代码、依赖或构建方式前应阅读相关章节。

## 1. 信息与变更原则

信息发生冲突时按以下顺序处理：

1. 用户最新明确要求；
2. 现有代码、配置和测试体现的实际行为；
3. 已确认的产品与架构文档；
4. 开发者或 Agent 的推测。

不得静默处理冲突。涉及产品行为、架构边界或外部接口变化时，应先说明影响并等待确认。

实现遵循：

- 最小必要修改，避免无关重构；
- 职责、所有权和生命周期明确；
- 优先复用已有接口、事件和工具；
- 不为未知需求提前设计复杂框架；
- 不覆盖或回退任务范围外的已有修改；
- 无法确认外部协议、硬件行为或需求时不自行假设。

## 2. 总体架构

系统采用单进程模块化架构：

```text
GCS / Other UAVs ── Zenoh ── Communication
Radar UDP :7790 ── gcs_sim ──┘
        │
 EventBus / C++ API
        │
┌───────┴─────────────────────────────────────┐
│ Core Runtime                               │
│ Task Scheduler → Task Execution Engine     │
│ UAV State / StateMonitor / Target           │
│ Flight Control / Camera / Geographic Tool  │
└───────┬─────────────────────────────────────┘
        │
 PX4 / Camera control
```

核心职责关系：

- Task 表达“做什么”；
- Task Execution Engine 决定执行策略；
- BehaviorTree 负责步骤编排和状态判断；
- Capability Module 提供明确能力；
- Adapter 和 Plugin 隔离具体外部技术。

## 3. 模块边界

Perception、检测节点、视觉目标接口和视频推流已完全移除，不保留 DisabledPerception。
相机控制模块保留，用于吊舱指令与状态，不传递图像帧。中制导通过 common 中的唯一公共接口
直接链接 `libmidcourse_alg_3588.so`，不通过 Plugin Management 或 Pluma 加载。构建及运行不依赖
OpenCV、RGA、Pluma 或 RKNN。
通用 Plugin Management 模块保留在 `src/plugin`，当前为空实现，仅接入模块生命周期，供后续插件扩展。
`AntiUav` 只编排 Arm／Takeoff／Midcourse，不包含视觉或末制导；独立飞行指令保持可用。

| 模块 | 主要职责 |
| --- | --- |
| Core Runtime | 配置、日志、事件总线、模块生命周期和致命退出 |
| Plugin Management | 通用插件管理入口，当前为空实现 |
| Communication | Zenoh 收发、报文序列化、确认重试和状态上报 |
| UAV State Management | 本机、友机和地面站链路的最新状态 |
| StateMonitor | 安全状态监测和告警 |
| Flight Control | PX4/MAVSDK 连接、遥测、连接后参数预读缓存、飞行动作和 Attitude／NED Offboard 重发 |
| Task Scheduler | Task 与独立 Command 的唯一仲裁者 |
| Task Execution Engine | 创建并运行任务树和指令树 |
| Target Management | 雷达／地面站目标的最新值与最近 20 条严格递增历史 |
| Midcourse Adapter | 隔离中制导发布包类型，直创目标估计器和规划器并保持预编译 ABI |
| Camera Control | 相机链路、状态和控制能力 |
| Geographic Tool | 地理距离和方位计算 |

安全高度等跨模块配置由 Core Runtime 的系统参数能力维护，启动时加载默认值并接受地面站动态更新。当前不单独设置场景管理模块。

### 3.1 控制边界

- 任意时刻只允许一个 Task 或独立 Command 执行，不维护等待队列。
- 新控制内容由 Task Scheduler 先中止当前内容，再启动最新内容。
- Task Scheduler 不创建 BehaviorTree，也不直接控制 PX4。
- Task Execution Engine 可同步调用 Flight Control 和 State 能力。
- Task 不包含 Command 序列，GCS 不定义机载 BehaviorTree。
- 其他无人机不得向本机下发 Task。
- StopMotor 只能由地面站人工触发。

### 3.2 通信边界

Communication 通过 ZenohTransport 声明 `plane/liveliness/<plane_id>` 并订阅 `plane/liveliness/*`，
开启 history 以发现先上线的节点。仅记录其他飞机的上下线变化；重复通知不重复打印。
存活性判断由 Zenoh 负责，状态更新日志不作为在线心跳。所有参与此发现机制的飞机均需声明存活标记。

公共 message.h 追加 VehicleInfo，不改变既有 MessageType 的枚举值。
公共 message.h 同时声明 topic_queryToPlane 和 topic_stationLiveliness，作为其他工程接入的协议地址；
common_include 保存共享数据模型、协议常量与接口定义，不提供查询实现，也不新增公共 QueryApi 或 query.h。
查询接口直接声明在本工程 src/communication/communication_api.h 中；查询地址和地面站存活标记地址
均从公共模板生成，参数校验、地址拼接和通信实现放在 src/communication/plane_query.cpp。
旧地面站需要升级以识别新类型并声明存活标记。
Communication 启动时、配置的地面站上线时，经工作线程向 sendToStation 发送 VehicleInfo；
启动与 history 通知尚未发送的上报合并，同一在线状态的重复通知不重复发送。
IP 与吊舱视频 URL 在启动时加载到主程序持有的 AppConfig，每次上报或查询直接读取 config_，不另存快照。
VehicleInfo.isSim 为 bool，使用同一 AppConfig 中已有的 is_sim 判定值；不修改 VehicleState。
解码旧 VehicleInfo 缺少 isSim 时默认为 false，字段存在时必须是 JSON 布尔值。
当前不增加配置热更新入口；将来并发更新配置时必须同步读写。

ZenohTransport 管理原生 queryable 的生命周期和带超时的 get；plane_query 校验响应 key、信封类型、
发送方和载荷飞机 ID。查询接口 queryPlane(planeId, MessageType, reply, error, timeout_ms)
目前只允许 VehicleInfo；服务端订阅 queryToPlane/<plane_id>/*，对其他类型返回原生错误响应。
查询不经控制消息处理路径，不产生 Ack 或任务事件。停止时先释放 queryable，再关闭 Session。
地面站声明 station/liveliness/<station_id> 前必须先建立上行订阅；该标记用于补发机载信息，
不改变现有基于业务报文的地面站链路超时状态。


| 场景 | 机制 |
| --- | --- |
| GCS/UAV 网络通信 | Zenoh，仅限 Communication 模块 |
| 状态查询、目标查询、明确能力调用 | 同步 C++ API |
| 状态变化、接收消息、执行进度和结果 | EventBus/eventpp 事件 |
| 飞控动作和中制导姿态下发 | 同步调用 FlightControl 并检查结果 |

同级模块不得交叉包含对方实现头文件。同步协作依赖稳定 API，事件协作使用唯一事件 ID。业务模块不得以事件“发出成功”替代需要确认结果的飞控调用。

Communication 对外提供 `sendToStation`、`sendToPlane` 和 `sendToAll` 三类发送能力。网络输入由 Communication 接收为字符串，根据 `MessageType` 使用 nlohmann-json 转换成对应数据对象，再通过 EventBus 交给业务模块；JSON 类型和解析逻辑不得进入业务模块。

`gcs_sim` 是 7790 雷达 UDP 与 Zenoh 之间的适配工具。防守机状态只从 `sendToAll` 的
`VehicleState` 缓存获得，雷达 UDP 不承担友机状态输入。适配器按扫描时间用 NED 速度外推友机，
对 confirmed 航迹执行允许“友机未出现在扫描中”的全局一对一门限匹配；只过滤 50 米内实际匹配的
航迹。尚无友机状态或状态过期时失败关闭，友机未匹配到雷达航迹时不阻塞靶机转发。

### 3.3 外部技术边界

业务模块不得直接依赖：

- Zenoh API；
- MAVSDK、PX4 或 MAVLink 细节；
- GeographicLib API；
- nlohmann-json。

这些技术只能出现在相应 Adapter、Capability 或基础设施模块内。不得形成模块循环依赖，也不得让外部适配层反向依赖业务实现。

## 4. 工程目录

工程根目录指本仓库根目录，以下称为 plane 工程根。

| 路径 | 职责 |
| --- | --- |
| `src/main.cpp` | 模块装配、启动和关闭入口 |
| `src/core` | Runtime、配置、日志、事件、系统参数和构建信息 |
| `src/plugin` | 通用插件管理模块，当前为空实现 |
| `src/communication` | Zenoh Transport、通信业务和 JSON 序列化 |
| `src/state` | 本机/友机状态和地面站链路监测 |
| `src/flight_control` | MAVSDK 飞控适配 |
| `src/task` | 调度器、执行引擎、上下文和行为树节点 |
| `src/target` | 地面站目标存储 |
| `src/camera` | 相机控制能力 |
| `src/geo` | 地理计算封装 |
| `config` | 默认运行配置 |
| `tests/contract` | 插件和外部接口契约测试 |
| `tests/integration` | 进程及配置集成测试 |
| `tests/fixtures` | 测试配置 |
| `tests/harness` | 测试公共脚本 |
| `scripts` | 源码树中的程序启停脚本；打包后复制到运行目录根部 |
| `tools` | 打包和辅助工具 |
| `cmake` | CMake 依赖和平台配置 |
| `docs` | 产品、架构和测试文档 |

明确允许读取的工程外目录：

| 路径 | 职责 |
| --- | --- |
| `../common/common_include` | 跨工程报文、地理类型和插件接口 |
| `../../../3rdparty/common` | 本地预编译第三方库 |
| `../algorithm/midcourse_alg/midcourse_alg-0.1.0-aarch64` | 中制导 AArch64 预编译动态库；随包公共头仅用于提供方校验，消费方引用 common |

禁止使用 `find /`、全盘 `rg` 或类似方式扫描工程外目录。需要确认公共定义时直接读取上述准确路径；若检索意外命中其他外部路径，应停止并说明。

## 5. 公共接口

跨工程数据结构和插件接口统一放在 `../common/common_include`：

- 数据模型使用 `plane::model`；
- 插件接口使用 `plane::algorithm`；
- 数据模型头文件只依赖 C++ 标准库；
- 本工程不引用图像插件接口，不引入 OpenCV 或图像帧类型；
- JSON 序列化只在 Communication 内实现；
- 业务模块不得另行定义等价的报文或插件结构。

项目内部模块通过 `*_api.h` 暴露同步能力，通过 `*_events.h` 暴露事件契约。新增接口前应先确认职责和依赖方向。

## 6. 生命周期与并发

- Runtime 按注册顺序执行 `init/start`，按逆序执行 `stop/release`。
- Flight Control 在飞控连接建立后读取并缓存通用 PX4 飞控系统参数；断连使缓存失效，重连后后台重读。
  中制导只是缓存读取方之一，Task 不得在行为树 tick 中访问 PX4 参数。
- Target Management 独立于任务生命周期持续保留每个航迹最近 20 帧；中制导启动和停止不得清除此历史。
- 每个模块负责自身资源创建、线程停止和资源释放，Runtime 只协调顺序。
- 系统使用少量长期线程处理网络、飞控遥测和行为树，不采用“一个模块固定一个线程”的原则。
- EventBus 处理函数不得执行不可控的长时间阻塞、AI 推理或视频编码。
- 阻塞 IO、推理和编码可使用专用线程，但应有明确停止条件并能被生命周期管理。
- Plugin 初始化或运行失败可请求 fatal；主线程必须有序关闭并返回非零状态。

## 7. 编码与依赖

### 7.1 基本规范

- 语言标准：C++17；
- 构建系统：CMake；
- 根命名空间：`plane`；
- 日志：spdlog，项目代码不使用 `printf` 或 `std::cout`；
- 人工编写的运行日志和业务描述必须使用中文；
- 日志级别保留 spdlog 的 `[info]`、`[warn]`、`[error]` 等原始英文标识；
- 高频接口输入输出日志不重复打印可由日志顺序判断的会话号、调用号或更新号，会话结束日志可保留累计统计；
- API 名、协议名、类型名、枚举值、配置键、函数参数名、结构体及其字段名和第三方库原始错误等
  代码标识保留原文；
- 日志数据存在公共字段名、配置键、函数参数名或外部参数名时必须直接使用原名，禁止中文意译；
- 格式：现有 clang-format，Google Style、4 空格缩进、120 列、左侧指针对齐；
- 头文件顶层禁止 `using namespace`；
- 优先使用清晰的普通实现，避免复杂语法和过度抽象。

### 7.2 依赖规范

- 第三方库优先通过 CMake `find_package` 使用已有本地构建。
- 三方库统一来自 `../../../3rdparty/common`，不得擅自联网下载或换用其他来源。
- 缺少依赖时应报告准确名称和路径，请求用户补充，不在项目内重新实现。
- CMake 依赖传播应正确选择 `PRIVATE`、`PUBLIC` 或 `INTERFACE`。
- 中制导必须使用 imported target 直接链接发布包的 `libmidcourse_alg_3588.so`，按无 SONAME 库原名打包并由 `$ORIGIN/lib` 加载；不得使用 Pluma 或 `libpluma`。
- 构建不得引入已经移除的图像处理或媒体运行库。
- 枚举与字符串转换统一使用项目已有的 `magic_enum`，不重复实现映射工具。

## 8. 验证与交付

- 修改后至少执行与风险相称的构建或静态检查。
- 优先运行与改动直接相关的现有测试，测试选择和命令见 `TESTING.md`。
- 修改公共接口、插件加载、配置解析或媒体链路时，必须运行相应契约或集成测试。
- 硬件、飞控、摄像头或真实流环境不可用时，明确记录未执行项，不得宣称通过。
- 不通过删除断言、放宽标准或静默跳过来掩盖失败。
- 交付说明应包含修改内容、实际执行的验证、未验证项和剩余风险。
- 产品行为变化同步更新 `SPEC.md`；架构或工程规则变化同步更新本文；测试标准变化同步更新 `TESTING.md`；使用方式变化同步更新 README。

## 9. Git 规范

- `git log` 中的 commit message 必须使用中文描述。
- 描述应明确说明本次变更内容，避免只写“修改”“更新”“修复”等无具体对象的词语。
- 未经用户明确要求，不主动提交、推送、合并或改写 Git 历史。
- onboard 每次向主仓库 `origin` 成功 push 后，必须立即将完整代码同步 push 到
  `https://github.com/desneo/onboard`，保持完整目录结构、对应分支和提交历史，并同步本次推送涉及的标签；
  不得仅上传变更文件或补丁。
- GitHub 同步属于该次推送的必要后续步骤，无需重复确认。同步失败时必须明确报告原因和未完成状态，
  不得将主仓库推送成功报告为全部推送完成。
- 不回退、不覆盖任务范围外的用户改动。

示例：

```text
完善视频推流配置校验
修复任务替换时未停止速度控制的问题
补充插件加载失败测试场景
```

## 10. 架构红线

以下变化必须先说明影响、获得确认并更新架构记录：

1. 从单进程改为多进程；
2. 替换 Zenoh、EventBus/eventpp、PX4/MAVSDK 或 BehaviorTree 主路径；
3. 合并 Task Scheduler 与 Task Execution Engine；
4. 让 Task 包含 Command 序列或让 GCS 定义 BehaviorTree；
5. 让业务模块直接依赖具体插件、MAVSDK、Zenoh 或 GeographicLib；
6. 允许地面站直接下发 NED 速度；
7. 允许机载任务自主触发 StopMotor；
8. 允许其他无人机向本机下发 Task；
9. 引入模块循环依赖；
10. 改变公共报文或插件接口而不评估其他工程兼容性。

## 11. 已确认的架构决策

| 决策 | 结论 |
| --- | --- |
| 部署 | 单进程模块化应用 |
| 网络通信 | Zenoh |
| 进程内事件 | 基于 eventpp 的 EventBus |
| 飞控 | PX4 + MAVSDK + MAVLink |
| 任务执行 | BehaviorTree.CPP |
| 控制仲裁 | Task Scheduler 统一管理 Task 和独立 Command |
| 执行职责 | Scheduler 与 Execution Engine 分离 |
| 感知帧模型 | 消费最新帧，不积压旧帧 |
| 地理计算 | 统一封装 GeographicLib |
| GCS 目标历史 | 每个目标保留最近 20 条 |
| 中制导集成 | 引用 `common_include/algorithm/midcourse_alg.h`，直连无 SONAME 的 `libmidcourse_alg_3588.so`，不使用 Pluma |
| 中制导输出 | Attitude Offboard；首帧先发设定值再启动，50 ms 重发；遥测发现模式被覆盖时限频重启 Offboard |
| 雷达友机过滤 | 友机状态来自 Zenoh；只过滤门限内实际匹配航迹，友机未出现在雷达帧中不阻塞靶机 |

重大决策变化时应在本节补充背景、选择、替代方案和影响，不得只修改代码。
