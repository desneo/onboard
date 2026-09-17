# M1 Change Design（通信基础）

> 状态：已确认（2026-09-17）。任务级计划见 `m1-communication-tasks.md`。
> 来源：`DEV_PLAN.md` 路线图 M1。
> 输入：SPEC §9、ARCHITECTURE §3.3/§18/§22/§24、common_include/message.h。

---

## 1. 目标与范围

M1 交付 Communication 模块的通信底座：

- Zenoh Session 生命周期与连接配置；
- 收发 API：`sendToStation`、`sendToPlane`、`sendToAll`（topic 常量取自 `message.h`）；
- 报文信封解析：`std::string` → `Message` → 按 `MessageType` 解码内部 class → eventcpp 事件分发；JSON 不出 Communication；
- 入站控制消息（TaskInfo/Command）的接收确认（Ack）；
- 出站按类型可靠性：控制类带 pending/超时/重试，状态类 best-effort；
- 进程内 EventBus（core，eventcpp 封装）；
- 模拟 GCS 联调工具 `gcs_sim`；
- 进程常驻运行循环（SIGINT/SIGTERM 优雅退出，Task 9 补充）。

不在 M1：状态管理、周期上报（M2）、任务/指令业务处理（M4）、告警业务（M9）、8 架规模验证（M10）。

## 2. 已确认决策（用户 2026-09-17）

| # | 决策 |
|---|---|
| 1 | 线格式 `payload` 为 JSON 字符串（与 `message.h` 注释一致） |
| 2 | Topic 映射：GCS→UAV 用 `sendToPlane/<plane_id>`；UAV→GCS（状态/Ack）用 `sendToStation`；广播用 `sendToAll` |
| 3 | 出站可靠性按类型：Alert/Ack 可靠（超时重试），VehicleState/TaskState best-effort |
| 4 | `Message` 信封增加 `senderId` 字段（跨仓修改 `common_include/message.h`） |

## 3. 跨仓变更：common_include

`../common/common_include/message.h`：

```cpp
struct Message {
    std::string senderId;   ///< 发送方标识（GCS 或 planeId）
    std::string messageId;  ///< 消息唯一标识
    MessageType type;       ///< 消息类型
    std::string payload;    ///< JSON 字符串
};
```

同步文档（需求变化已确认，实施时执行）：

- `docs/ARCHITECTURE.md` §28.6「报文统一为 `Message{messageId, type, payload}`」→ `Message{senderId, messageId, type, payload}`；
- ADR-017 Consequences 补充「Message 增加 senderId，用于 Ack 路由与来源标识」。

影响评估：仅新增字段；现有代码与插件接口未使用 `Message` 聚合初始化，向后兼容。`common` 为独立 git 仓，提交需用户确认。

## 4. Topic 与角色

| 场景 | Topic |
|---|---|
| GCS → 指定 UAV | `sendToPlane/<plane_id>` |
| UAV → GCS（状态、Ack、告警） | `sendToStation` |
| 广播（任意方） | `sendToAll` |
| UAV 订阅 | `sendToPlane/<config.plane_id>`、`sendToAll` |

- Ack 路由：`senderId == station_id` → 发 `sendToStation`；否则发 `sendToPlane/<senderId>`。
- 自消息过滤：`senderId == plane_id` 的入站消息丢弃（防止 `sendToAll` 回环）。
- 连接：默认使用 Zenoh 默认发现；可通过配置 `zenoh_connect` / `zenoh_listen` 指定 endpoint，应对无多播网络。
- 注意：`topic_sendToPlane` 常量值 `sendToPlane/{planeId}` 作为前缀模板使用，订阅键为 `sendToPlane/<plane_id>`（不做字符串格式化假设）。
- Zenoh 1.x 的 key expression 不允许前导 `/`；`message.h` 的 topic 常量已统一为无前导 `/` 的形式（2026-09-17 用户确认）。Zenoh 边界保留去 `/` 规范化以兼容旧值。

## 5. 线格式

```json
{
  "senderId": "gcs",
  "messageId": "gcs-1",
  "type": "TaskInfo",
  "payload": "{\"taskId\":\"t1\",\"taskCode\":\"suicide\",\"targetTrackId\":\"\",\"taskAction\":\"Start\"}"
}
```

- `type` 与 `MessageType` 用 magic_enum 互转；未知 type → 丢弃 + warn + `communication.error`。
- `payload` 为字符串，内部按类型二次 parse 为对应 struct；解析失败不中断通信。
- messageId：`<plane_id>-<单调递增计数>`（加锁生成，第一版不做持久化）。

## 6. EventBus（core）

新增 `src/core/event_bus.h/.cpp`：

```cpp
class EventBus {
   public:
    template <typename T>
    void subscribe(std::string_view event_id, std::function<void(const T&)> handler);

    template <typename T>
    void publish(std::string_view event_id, const T& event);
};
```

- 实现：`std::type_index` → `eventpp::EventDispatcher<std::string, void(const T&)>`；eventpp 默认 MultipleThreading，可跨线程订阅/发布。
- 事件 ID 为字符串常量，集中定义在 `src/communication/communication_events.h`（M1）；其他模块新增事件时在各自公开接口头中定义，保证全局唯一。
- 生命周期：main 创建 EventBus，先于模块创建并注入（组合根），不引入单例。

Communication 事件表：

| Event ID | 载荷 |
|---|---|
| `communication.task_info` | `plane::model::TaskInfo` |
| `communication.command` | `plane::model::Command` |
| `communication.system_param` | `plane::model::SystemParam` |
| `communication.gcs_target` | `plane::model::GcsTarget` |
| `communication.error` | `plane::communication::CommunicationError{detail}` |

Ack 不产生业务事件，由 Communication 内部消费。

## 7. Communication 内部结构

文件：

- `src/communication/communication_api.h`：对外稳定能力接口与出站载荷类型；
- `src/communication/communication_events.h`：事件 ID 常量与 `CommunicationError`；
- `src/communication/serialization.h/.cpp`：信封编解码、payload 编解码、类型互转、messageId 生成；
- `src/communication/zenoh_transport.h/.cpp`：Session open/close、订阅、put；Zenoh 异常统一转换；
- `src/communication/communication_module.h/.cpp`：`core::IModule` + `CommunicationApi`，RX 队列与工作线程、pending/重试。

对外接口：

```cpp
using OutgoingMessage =
    std::variant<plane::model::VehicleState, plane::model::TaskState, plane::model::Alert>;

class CommunicationApi {
   public:
    virtual ~CommunicationApi() = default;
    virtual bool sendToStation(const OutgoingMessage& message) = 0;
    virtual bool sendToPlane(const std::string& plane_id, const OutgoingMessage& message) = 0;
    virtual bool sendToAll(const OutgoingMessage& message) = 0;
};
```

可靠性策略（内部按类型决定，不给调用方暴露 JSON/可靠性参数）：Alert/Ack 走可靠发送；VehicleState/TaskState 直接 put。

RX 流程：Zenoh 回调（Zenoh 线程）→ 原始字符串入有界队列 → 工作线程出队 → 解析信封 → senderId 过滤 → 按类型解码 payload → 发布事件 → TaskInfo/Command 回 Ack。

TX 流程：构造 `Message{senderId=plane_id, messageId, type, payload}` → dump 为字符串 → put；可靠类型登记 pending。

pending/重试：`messageId → {wire, topic, deadline, attempts}`；工作线程每 50 ms 扫描，超时重发（≤ `comm_ack_max_retries`），超限发布 `communication.error`。

线程与生命周期：模块内 1 个通信工作线程 + Zenoh 内部线程；`start` 开 Session/订阅/线程，`stop` 停线程、join、关闭 Session。RX 解析失败只告警不断链。

错误处理：`zenoh::ZException`（或返回错误码）统一转 `communication.error` 事件 + spdlog error；队列满时丢弃最旧消息并记 warn。

## 8. 配置项

`AppConfig` 与 `config/onboard.conf` 新增：

| key | 默认 | 说明 |
|---|---|---|
| `plane_id` | `uav1` | 本机标识，用于订阅键与 senderId |
| `station_id` | `gcs` | 地面站标识，用于 Ack 路由 |
| `zenoh_config_file` | 空 | 非空则 `Config::from_file` |
| `zenoh_connect` | 空 | 逗号分隔 endpoint，逐条 `insert_json5("connect/endpoints", ...)` |
| `zenoh_listen` | 空 | 逗号分隔 endpoint，逐条 `insert_json5("listen/endpoints", ...)` |
| `comm_ack_timeout_ms` | 500 | 可靠发送超时 |
| `comm_ack_max_retries` | 3 | 可靠发送最大重试次数 |
| `comm_rx_queue_max` | 1024 | RX 队列上限 |

## 9. gcs_sim 联调工具

新增 target `gcs_sim`（`tools/gcs_sim/main.cpp`），依赖 `plane::common_include`、`plane::nlohmann_json`、`zenohc::lib`、`zenohcxx::zenohc`、`spdlog::spdlog`、`fmt::fmt`，不链接 `plane::deps`（避免拖入 MAVSDK/ZLMediaKit）。

默认行为（模拟一次 GCS 交互）：

1. 订阅 `sendToStation`；
2. 向 `sendToPlane/<plane-id>` 发送 TaskInfo（Start）；
3. 等待并打印对应 Ack（按 ackId 匹配）；
4. 向 `sendToPlane/<plane-id>` 发送 Command（Arm）；
5. 等待 Ack，打印；
6. 继续打印收到的 `sendToStation` 消息 3 秒后退出；
7. 收到 Alert/Ack 类消息按协议回 Ack。

选项：

| 选项 | 说明 |
|---|---|
| `--plane-id <id>` | 目标 UAV（默认 uav1） |
| `--sender-id <id>` | 本端标识（默认 gcs；用于验证自消息过滤） |
| `--no-ack` | 不回应任何消息，用于触发 onboard 重试与超时 |
| `--listen-seconds <n>` | 第 6 步持续时间（默认 3） |

## 10. M1 验收

1. 构建：x64 Release 通过；
2. 终端 A `./build/onboard`，终端 B `./build/gcs_sim --plane-id uav1`：
   - onboard 日志显示收到 TaskInfo/Command 并发布对应事件；
   - gcs_sim 收到 2 个 Ack，messageId 与它发出的消息匹配；
3. `./build/gcs_sim --no-ack`：onboard 出现重试日志（≥1 次），最终 `communication.error`（超时耗尽）；
4. 自消息过滤：`./build/gcs_sim --sender-id uav1` 发送广播/单播后，onboard 不处理该消息（日志无事件发布）；
5. `clang-format --dry-run --Werror` 通过，`src/` 无 `std::cout/printf`。

出站 `sendToPlane`/`sendToAll` 的业务级端到端验证从 M2（状态上报）开始覆盖。

## 11. M1 不做

- 周期状态上报与状态管理（M2）；
- 任务/指令的业务处理（M4）；
- 告警业务与阈值（M9）；
- Zenoh Session 断线重建策略（依赖 Zenoh 自带重连，出现问题时再设计）；
- 消息持久化与离线补发。

## 12. 风险与说明

| # | 事项 | 处理 |
|---|---|---|
| 1 | Topic/线格式属于与地面站的对接口径 | 本文档为第一版约定，需地面站侧按同口径实现；如对方已有约定需回炉确认 |
| 2 | `common_include/message.h` 为跨组件共用头 | 仅新增 senderId，向后兼容；common 仓提交需用户确认 |
| 3 | Zenoh 默认多播发现在部分机载网络不可用 | 提供 `zenoh_connect`/`zenoh_listen` 配置 |
| 4 | `Message.payload` 为字符串，需二次解析 | 第一版内网规模可接受；如需优化再评估嵌套对象 |
| 5 | 可靠重试依赖对端回 Ack | gcs_sim 实现 GCS 侧回 Ack；无对端时超时会走完重试并告警 |
