# M1 任务级计划（通信基础）

> 设计依据：`m1-communication-design.md`（已确认）。冲突时以设计文档与 `ARCHITECTURE.md` 为准。
> 执行位置：工程根 `onboard/`。
> 状态：已完成并通过 Task 9 验收（2026-09-17）。执行偏差与证据见文末「执行记录与偏差」。
> 验证原则：每个 Task 结束后 x64 Release 构建通过；不写测试代码（SPEC §11）。运行时行为在 Task 7/8/9 集中验证。
> 提交：每个 Task 一个提交（Task 1 涉及 common 仓，提交需用户确认）。

---

## Task 1：common_include 增加 senderId 与文档同步

**Files**

- Modify: `../common/common_include/message.h`
- Modify: `docs/ARCHITECTURE.md`（§28.6 数据模型要点、ADR-017 Consequences）

**Interfaces**

- Produces: `plane::model::Message{senderId, messageId, type, payload}`（后续 Task 4/6 依赖）

- [x] Step 1: 修改 `message.h` 的 `Message` 定义

```cpp
/// 消息体最外层包裹。
struct Message {
    std::string senderId;   ///< 发送方标识（GCS 或 planeId）
    std::string messageId;  ///< 消息唯一标识
    MessageType type;       ///< 消息类型，用于选择反序列化目标
    std::string payload;    ///< JSON 字符串，序列化由通信模块负责
};
```

- [x] Step 2: 同步 `docs/ARCHITECTURE.md` §28.6

将数据模型要点中的：

```text
- 报文统一为 `Message{messageId, type, payload}`，`payload` 为 JSON 字符串；...
```

改为：

```text
- 报文统一为 `Message{senderId, messageId, type, payload}`，`payload` 为 JSON 字符串；...
```

- [x] Step 3: 同步 ADR-017 Consequences，追加一条

```text
- `Message` 增加 `senderId`，用于 Ack 路由与消息来源标识。
```

- [x] Step 4: 构建验证

```bash
cmake --build build -j
```

Expected: 构建通过（`config.cpp` 引用 `message.h`，可验证头文件兼容）。

- [x] Step 5: 提交

onboard 仓（文档）：

```bash
git add docs/ARCHITECTURE.md
git commit -m "docs(m1): Message 信封增加 senderId"
```

common 仓（**需用户确认后执行**）：

```bash
git -C ../common add common_include/message.h
git -C ../common commit -m "feat(common): Message 信封增加 senderId"
```

---

## Task 2：EventBus 与通信接口头

**Files**

- Create: `src/core/event_bus.h`
- Create: `src/communication/communication_events.h`
- Create: `src/communication/communication_api.h`
- Modify: `src/main.cpp`

**Interfaces**

- Produces: `plane::core::EventBus::subscribe<T>/publish<T>`
- Produces: `plane::communication::OutgoingMessage`、`CommunicationApi`
- Produces: event ID 常量与 `CommunicationError`

- [x] Step 1: 写 `src/core/event_bus.h`

```cpp
#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <typeindex>
#include <unordered_map>
#include <utility>

#include <eventpp/eventdispatcher.h>

namespace plane::core {

/// 进程内异步事件总线（ARCHITECTURE §3.2/§22）。
/// 事件标识为字符串常量；载荷类型由 subscribe<T>/publish<T> 决定。
class EventBus {
   public:
    template <typename T>
    void subscribe(std::string_view event_id, std::function<void(const T&)> handler) {
        channel<T>().dispatcher.appendListener(std::string(event_id), std::move(handler));
    }

    template <typename T>
    void publish(std::string_view event_id, const T& event) {
        channel<T>().dispatcher.dispatch(std::string(event_id), event);
    }

   private:
    struct ChannelBase {
        virtual ~ChannelBase() = default;
    };

    template <typename T>
    struct Channel final : ChannelBase {
        eventpp::EventDispatcher<std::string, void(const T&)> dispatcher;
    };

    template <typename T>
    Channel<T>& channel() {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::type_index key(typeid(T));
        auto it = channels_.find(key);
        if (it == channels_.end()) {
            auto channel = std::make_unique<Channel<T>>();
            Channel<T>* raw = channel.get();
            channels_.emplace(key, std::move(channel));
            return *raw;
        }
        return static_cast<Channel<T>&>(*it->second);
    }

    std::mutex mutex_;
    std::unordered_map<std::type_index, std::unique_ptr<ChannelBase>> channels_;
};

}  // namespace plane::core
```

- [x] Step 2: 写 `src/communication/communication_api.h`

```cpp
#pragma once

#include <string>
#include <variant>

#include "message.h"

namespace plane::communication {

/// 出站载荷（Ack 仅供 Communication 内部使用）。
using OutgoingMessage = std::variant<plane::model::VehicleState, plane::model::TaskState,
                                     plane::model::Alert, plane::model::Ack>;

/// Communication 对业务模块暴露的稳定能力接口。
class CommunicationApi {
   public:
    virtual ~CommunicationApi() = default;

    virtual bool sendToStation(const OutgoingMessage& message) = 0;
    virtual bool sendToPlane(const std::string& plane_id, const OutgoingMessage& message) = 0;
    virtual bool sendToAll(const OutgoingMessage& message) = 0;
};

}  // namespace plane::communication
```

- [x] Step 3: 写 `src/communication/communication_events.h`

```cpp
#pragma once

#include <string>
#include <string_view>

namespace plane::communication {

/// 通信错误事件载荷。
struct CommunicationError {
    std::string detail;
};

namespace events {

inline constexpr std::string_view kTaskInfoReceived = "communication.task_info";
inline constexpr std::string_view kCommandReceived = "communication.command";
inline constexpr std::string_view kSystemParamReceived = "communication.system_param";
inline constexpr std::string_view kGcsTargetReceived = "communication.gcs_target";
inline constexpr std::string_view kError = "communication.error";

}  // namespace events

}  // namespace plane::communication
```

- [x] Step 4: 修改 `src/main.cpp`，创建 EventBus 并订阅错误事件

在 `main` 中 `runtime` 创建之前插入：

```cpp
    plane::core::EventBus event_bus;
    event_bus.subscribe<plane::communication::CommunicationError>(
        plane::communication::events::kError,
        [](const plane::communication::CommunicationError& error) {
            spdlog::error("通信错误事件: {}", error.detail);
        });
```

并在 include 区加入：

```cpp
#include "communication/communication_events.h"
#include "core/event_bus.h"
```

- [x] Step 5: 构建验证

```bash
cmake --build build -j
./build/onboard; echo "exit=$?"
```

Expected: 构建通过；运行输出与 M0 一致（事件订阅无消费者触发），exit=0。

- [x] Step 6: 提交

```bash
git add src/core/event_bus.h src/communication/communication_events.h \
        src/communication/communication_api.h src/main.cpp
git commit -m "feat(m1): 增加 EventBus 与通信对外接口"
```

---

## Task 3：通信配置项扩展

**Files**

- Modify: `src/core/config.h`
- Modify: `src/core/config.cpp`
- Modify: `config/onboard.conf`

**Interfaces**

- Produces: `AppConfig{plane_id, station_id, zenoh_config_file, zenoh_connect, zenoh_listen, comm_ack_timeout_ms, comm_ack_max_retries, comm_rx_queue_max}`（Task 5/6 依赖）

- [x] Step 1: 修改 `src/core/config.h` 的 `AppConfig`

```cpp
struct AppConfig {
    plane::model::SystemParam system_param;
    std::string log_dir = "logs";
    int state_report_period_ms = 1000;

    std::string plane_id = "uav1";
    std::string station_id = "gcs";
    std::string zenoh_config_file;
    std::string zenoh_connect;
    std::string zenoh_listen;
    int comm_ack_timeout_ms = 500;
    int comm_ack_max_retries = 3;
    int comm_rx_queue_max = 1024;
};
```

- [x] Step 2: 修改 `src/core/config.cpp` 的解析分支

在 `state_report_period_ms` 分支之后、`else` 之前追加：

```cpp
        } else if (key == "plane_id") {
            out.plane_id = value;
        } else if (key == "station_id") {
            out.station_id = value;
        } else if (key == "zenoh_config_file") {
            out.zenoh_config_file = value;
        } else if (key == "zenoh_connect") {
            out.zenoh_connect = value;
        } else if (key == "zenoh_listen") {
            out.zenoh_listen = value;
        } else if (key == "comm_ack_timeout_ms") {
            out.comm_ack_timeout_ms = std::stoi(value);
        } else if (key == "comm_ack_max_retries") {
            out.comm_ack_max_retries = std::stoi(value);
        } else if (key == "comm_rx_queue_max") {
            out.comm_rx_queue_max = std::stoi(value);
```

- [x] Step 3: 修改 `config/onboard.conf`，追加

```text
# 通信配置
plane_id = uav1
station_id = gcs
zenoh_config_file =
zenoh_connect =
zenoh_listen =
comm_ack_timeout_ms = 500
comm_ack_max_retries = 3
comm_rx_queue_max = 1024
```

- [x] Step 4: 构建并运行

```bash
cmake --build build -j
./build/onboard; echo "exit=$?"
```

Expected: 构建通过；配置加载无报错，输出与 M0 一致（通信模块在 Task 7 才接入），exit=0。

- [x] Step 5: 提交

```bash
git add src/core/config.h src/core/config.cpp config/onboard.conf
git commit -m "feat(m1): 增加通信相关配置项"
```

---

## Task 4：序列化（信封与 payload）

**Files**

- Create: `src/communication/serialization.h`
- Create: `src/communication/serialization.cpp`
- Modify: `CMakeLists.txt`

**Interfaces**

- Consumes: `message.h`（Task 1）、`OutgoingMessage`（Task 2）
- Produces: `to_payload/from_payload`（8 类载荷）、`encode_envelope/decode_envelope`、`message_type_to_string/from_string`、`next_message_id`、`make_message`

- [x] Step 1: 写 `src/communication/serialization.h`

```cpp
#pragma once

#include <string>

#include "communication/communication_api.h"
#include "message.h"

namespace plane::communication {

bool to_payload(const plane::model::TaskInfo& value, std::string& out, std::string& error);
bool to_payload(const plane::model::Command& value, std::string& out, std::string& error);
bool to_payload(const plane::model::SystemParam& value, std::string& out, std::string& error);
bool to_payload(const plane::model::GcsTarget& value, std::string& out, std::string& error);
bool to_payload(const plane::model::Alert& value, std::string& out, std::string& error);
bool to_payload(const plane::model::VehicleState& value, std::string& out, std::string& error);
bool to_payload(const plane::model::TaskState& value, std::string& out, std::string& error);
bool to_payload(const plane::model::Ack& value, std::string& out, std::string& error);

bool from_payload(const std::string& payload, plane::model::TaskInfo& out, std::string& error);
bool from_payload(const std::string& payload, plane::model::Command& out, std::string& error);
bool from_payload(const std::string& payload, plane::model::SystemParam& out, std::string& error);
bool from_payload(const std::string& payload, plane::model::GcsTarget& out, std::string& error);
bool from_payload(const std::string& payload, plane::model::Ack& out, std::string& error);

std::string encode_envelope(const plane::model::Message& message);
bool decode_envelope(const std::string& wire, plane::model::Message& out, std::string& error);

std::string message_type_to_string(plane::model::MessageType type);
bool message_type_from_string(const std::string& name, plane::model::MessageType& out);

std::string next_message_id(const std::string& sender_id);

bool make_message(const std::string& sender_id, const OutgoingMessage& value,
                  plane::model::Message& out, std::string& error);

}  // namespace plane::communication
```

- [x] Step 2: 写 `src/communication/serialization.cpp`

```cpp
#include "communication/serialization.h"

#include <atomic>
#include <cstdint>
#include <string>
#include <type_traits>
#include <utility>

#include <magic_enum/magic_enum.hpp>
#include <nlohmann/json.hpp>

namespace plane::communication {
namespace {

using nlohmann::json;

json to_json(const plane::model::GeoPoint& value) {
    return json{{"latitude", value.latitude},
                {"longitude", value.longitude},
                {"height", value.height},
                {"altitude", value.altitude}};
}

void geo_point_from_json(const json& in, plane::model::GeoPoint& out) {
    out.latitude = in.at("latitude").get<double>();
    out.longitude = in.at("longitude").get<double>();
    out.height = in.at("height").get<float>();
    out.altitude = in.at("altitude").get<float>();
}

template <typename Enum>
bool enum_from_string(const std::string& name, Enum& out) {
    const auto value = magic_enum::enum_cast<Enum>(name);
    if (!value.has_value()) {
        return false;
    }
    out = *value;
    return true;
}

template <typename T>
struct MessageTypeOf;

template <>
struct MessageTypeOf<plane::model::Alert> {
    static constexpr plane::model::MessageType value = plane::model::MessageType::Alert;
};

template <>
struct MessageTypeOf<plane::model::Ack> {
    static constexpr plane::model::MessageType value = plane::model::MessageType::Ack;
};

template <>
struct MessageTypeOf<plane::model::VehicleState> {
    static constexpr plane::model::MessageType value = plane::model::MessageType::VehicleState;
};

template <>
struct MessageTypeOf<plane::model::TaskState> {
    static constexpr plane::model::MessageType value = plane::model::MessageType::TaskState;
};

}  // namespace

bool to_payload(const plane::model::TaskInfo& value, std::string& out, std::string& error) {
    try {
        out = json{{"taskId", value.taskId},
                   {"taskCode", value.taskCode},
                   {"targetTrackId", value.targetTrackId},
                   {"taskAction", std::string(magic_enum::enum_name(value.taskAction))}}
                  .dump();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool from_payload(const std::string& payload, plane::model::TaskInfo& out, std::string& error) {
    try {
        const json in = json::parse(payload);
        out.taskId = in.at("taskId").get<std::string>();
        out.taskCode = in.at("taskCode").get<std::string>();
        out.targetTrackId = in.value("targetTrackId", std::string{});
        if (!enum_from_string(in.at("taskAction").get<std::string>(), out.taskAction)) {
            error = "未知 TaskAction";
            return false;
        }
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool to_payload(const plane::model::Command& value, std::string& out, std::string& error) {
    try {
        json waypoints = json::array();
        for (const auto& point : value.waypoints) {
            waypoints.push_back(to_json(point));
        }
        out = json{{"commandId", value.commandId},
                   {"type", std::string(magic_enum::enum_name(value.type))},
                   {"takeoffHeight_m", value.takeoffHeight_m},
                   {"waypoints", std::move(waypoints)}}
                  .dump();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool from_payload(const std::string& payload, plane::model::Command& out, std::string& error) {
    try {
        const json in = json::parse(payload);
        out.commandId = in.at("commandId").get<std::string>();
        if (!enum_from_string(in.at("type").get<std::string>(), out.type)) {
            error = "未知 CommandType";
            return false;
        }
        out.takeoffHeight_m = in.value("takeoffHeight_m", 0.0F);
        out.waypoints.clear();
        for (const auto& item : in.value("waypoints", json::array())) {
            plane::model::GeoPoint point;
            geo_point_from_json(item, point);
            out.waypoints.push_back(point);
        }
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool to_payload(const plane::model::SystemParam& value, std::string& out, std::string& error) {
    try {
        out = json{{"safeHeight_m", value.safeHeight_m}}.dump();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool from_payload(const std::string& payload, plane::model::SystemParam& out, std::string& error) {
    try {
        const json in = json::parse(payload);
        out.safeHeight_m = in.value("safeHeight_m", 0.0);
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool to_payload(const plane::model::GcsTarget& value, std::string& out, std::string& error) {
    try {
        out = json{{"trackId", value.trackId},
                   {"geoPos", to_json(value.geoPos)},
                   {"velocity_m_s", value.velocity_m_s},
                   {"direction_deg", value.direction_deg}}
                  .dump();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool from_payload(const std::string& payload, plane::model::GcsTarget& out, std::string& error) {
    try {
        const json in = json::parse(payload);
        out.trackId = in.at("trackId").get<std::string>();
        geo_point_from_json(in.at("geoPos"), out.geoPos);
        out.velocity_m_s = in.value("velocity_m_s", 0.0F);
        out.direction_deg = in.value("direction_deg", 0.0F);
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool to_payload(const plane::model::Alert& value, std::string& out, std::string& error) {
    try {
        out = json{{"type", value.type}, {"detail", value.detail}}.dump();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool to_payload(const plane::model::Ack& value, std::string& out, std::string& error) {
    try {
        out = json{{"ackId", value.ackId}}.dump();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool from_payload(const std::string& payload, plane::model::Ack& out, std::string& error) {
    try {
        const json in = json::parse(payload);
        out.ackId = in.at("ackId").get<std::string>();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool to_payload(const plane::model::VehicleState& value, std::string& out, std::string& error) {
    try {
        out = json{{"planeId", value.planeId},
                   {"geoPos", to_json(value.geoPos)},
                   {"startPos", to_json(value.startPos)},
                   {"roll_deg", value.roll_deg},
                   {"pitch_deg", value.pitch_deg},
                   {"yaw_deg", value.yaw_deg},
                   {"vn_m_s", value.vn_m_s},
                   {"ve_m_s", value.ve_m_s},
                   {"vd_m_s", value.vd_m_s},
                   {"airspeed_m_s", value.airspeed_m_s},
                   {"voltage_v", value.voltage_v},
                   {"flightMode", value.flightMode},
                   {"fcLinkState", value.fcLinkState},
                   {"currentBehaviorTree", value.currentBehaviorTree},
                   {"currentNode", value.currentNode},
                   {"cameraAngle_roll_deg", value.cameraAngle_roll_deg},
                   {"cameraAngle_pitch_deg", value.cameraAngle_pitch_deg},
                   {"cameraAngle_yaw_deg", value.cameraAngle_yaw_deg}}
                  .dump();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool to_payload(const plane::model::TaskState& value, std::string& out, std::string& error) {
    try {
        out = json{{"taskId", value.taskId},
                   {"status", std::string(magic_enum::enum_name(value.status))},
                   {"currentBehaviorTree", value.currentBehaviorTree},
                   {"currentNode", value.currentNode}}
                  .dump();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

std::string encode_envelope(const plane::model::Message& message) {
    const json envelope{{"senderId", message.senderId},
                        {"messageId", message.messageId},
                        {"type", message_type_to_string(message.type)},
                        {"payload", message.payload}};
    return envelope.dump();
}

bool decode_envelope(const std::string& wire, plane::model::Message& out, std::string& error) {
    try {
        const json envelope = json::parse(wire);
        out.senderId = envelope.at("senderId").get<std::string>();
        out.messageId = envelope.at("messageId").get<std::string>();
        const std::string type_name = envelope.at("type").get<std::string>();
        if (!message_type_from_string(type_name, out.type)) {
            error = "未知 MessageType: " + type_name;
            return false;
        }
        out.payload = envelope.at("payload").get<std::string>();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

std::string message_type_to_string(plane::model::MessageType type) {
    return std::string(magic_enum::enum_name(type));
}

bool message_type_from_string(const std::string& name, plane::model::MessageType& out) {
    return enum_from_string(name, out);
}

std::string next_message_id(const std::string& sender_id) {
    static std::atomic<std::uint64_t> counter{0};
    return sender_id + "-" + std::to_string(++counter);
}

bool make_message(const std::string& sender_id, const OutgoingMessage& value,
                  plane::model::Message& out, std::string& error) {
    return std::visit(
        [&](const auto& payload_value) -> bool {
            using T = std::decay_t<decltype(payload_value)>;
            std::string payload;
            if (!to_payload(payload_value, payload, error)) {
                return false;
            }
            out.senderId = sender_id;
            out.messageId = next_message_id(sender_id);
            out.type = MessageTypeOf<T>::value;
            out.payload = std::move(payload);
            return true;
        },
        value);
}

}  // namespace plane::communication
```

- [x] Step 3: 更新 `CMakeLists.txt` 的 `add_executable`，追加

```cmake
  src/communication/serialization.cpp
```

- [x] Step 4: 构建验证

```bash
cmake --build build -j
```

Expected: 构建通过，无警告。

- [x] Step 5: 提交

```bash
git add src/communication/serialization.h src/communication/serialization.cpp CMakeLists.txt
git commit -m "feat(m1): 增加通信报文序列化"
```

---

## Task 5：Zenoh 传输封装

**Files**

- Create: `src/communication/zenoh_transport.h`
- Create: `src/communication/zenoh_transport.cpp`
- Modify: `CMakeLists.txt`

**Interfaces**

- Produces: `plane::communication::ZenohTransport`（open/close/subscribe/put），Zenoh 头文件不出 .cpp

- [x] Step 1: 写 `src/communication/zenoh_transport.h`

```cpp
#pragma once

#include <functional>
#include <memory>
#include <string>

namespace plane::communication {

/// Zenoh 连接参数（来自 AppConfig）。
struct ZenohEndpoints {
    std::string config_file;
    std::string connect;
    std::string listen;
};

/// Zenoh 最小封装：Session 生命周期、订阅与发送。
/// 设计约束：业务模块不直接依赖 Zenoh；Zenoh 头文件仅出现在本类实现中。
class ZenohTransport {
   public:
    using SampleHandler = std::function<void(const std::string& payload)>;

    ZenohTransport();
    ~ZenohTransport();
    ZenohTransport(const ZenohTransport&) = delete;
    ZenohTransport& operator=(const ZenohTransport&) = delete;

    bool open(const ZenohEndpoints& endpoints, std::string& error);
    void close();
    bool subscribe(const std::string& key_expr, SampleHandler handler, std::string& error);
    bool put(const std::string& key_expr, const std::string& payload, std::string& error);

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace plane::communication
```

- [x] Step 2: 写 `src/communication/zenoh_transport.cpp`

```cpp
#include "communication/zenoh_transport.h"

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <zenoh/api.hxx>

namespace plane::communication {
namespace {

std::string endpoints_json5_array(const std::string& comma_separated) {
    std::string result = "[";
    bool first = true;
    std::size_t begin = 0;
    while (begin <= comma_separated.size()) {
        const std::size_t end = comma_separated.find(',', begin);
        std::string item = comma_separated.substr(begin, end - begin);
        const auto not_space = item.find_first_not_of(" \t");
        const auto last = item.find_last_not_of(" \t");
        item = not_space == std::string::npos ? std::string{} : item.substr(not_space, last - not_space + 1);
        if (!item.empty()) {
            if (!first) {
                result += ",";
            }
            result += "\"" + item + "\"";
            first = false;
        }
        if (end == std::string::npos) {
            break;
        }
        begin = end + 1;
    }
    result += "]";
    return result;
}

}  // namespace

struct ZenohTransport::Impl {
    std::optional<zenoh::Session> session;
    std::vector<zenoh::Subscriber<void>> subscribers;
};

ZenohTransport::ZenohTransport() = default;
ZenohTransport::~ZenohTransport() = default;

bool ZenohTransport::open(const ZenohEndpoints& endpoints, std::string& error) {
    if (!impl_) {
        impl_ = std::make_unique<Impl>();
    }
    try {
        zenoh::Config config = endpoints.config_file.empty()
                                   ? zenoh::Config::create_default()
                                   : zenoh::Config::from_file(endpoints.config_file);
        if (!endpoints.connect.empty()) {
            config.insert_json5("connect/endpoints", endpoints_json5_array(endpoints.connect));
        }
        if (!endpoints.listen.empty()) {
            config.insert_json5("listen/endpoints", endpoints_json5_array(endpoints.listen));
        }
        zenoh::ZResult result = Z_OK;
        zenoh::Session session = zenoh::Session::open(
            std::move(config), zenoh::Session::SessionOptions::create_default(), &result);
        if (result != Z_OK) {
            error = "Zenoh Session 打开失败";
            return false;
        }
        impl_->session.emplace(std::move(session));
        return true;
    } catch (const zenoh::ZException& e) {
        error = e.what();
        return false;
    }
}

void ZenohTransport::close() {
    if (!impl_) {
        return;
    }
    impl_->subscribers.clear();
    impl_->session.reset();
}

bool ZenohTransport::subscribe(const std::string& key_expr, SampleHandler handler, std::string& error) {
    if (!impl_ || !impl_->session.has_value()) {
        error = "Session 未打开";
        return false;
    }
    try {
        zenoh::ZResult result = Z_OK;
        zenoh::Subscriber<void> subscriber = impl_->session->declare_subscriber(
            zenoh::KeyExpr(key_expr),
            [handler = std::move(handler)](zenoh::Sample& sample) {
                handler(sample.get_payload().as_string());
            },
            []() {}, zenoh::Session::SubscriberOptions::create_default(), &result);
        if (result != Z_OK) {
            error = "订阅失败: " + key_expr;
            return false;
        }
        impl_->subscribers.push_back(std::move(subscriber));
        return true;
    } catch (const zenoh::ZException& e) {
        error = e.what();
        return false;
    }
}

bool ZenohTransport::put(const std::string& key_expr, const std::string& payload, std::string& error) {
    if (!impl_ || !impl_->session.has_value()) {
        error = "Session 未打开";
        return false;
    }
    try {
        zenoh::ZResult result = Z_OK;
        impl_->session->put(zenoh::KeyExpr(key_expr), zenoh::Bytes(payload),
                            zenoh::Session::PutOptions::create_default(), &result);
        if (result != Z_OK) {
            error = "发送失败: " + key_expr;
            return false;
        }
        return true;
    } catch (const zenoh::ZException& e) {
        error = e.what();
        return false;
    }
}

}  // namespace plane::communication
```

- [x] Step 3: 更新 `CMakeLists.txt` 的 `add_executable`，追加

```cmake
  src/communication/zenoh_transport.cpp
```

- [x] Step 4: 构建验证

```bash
cmake --build build -j
```

Expected: 构建通过，无警告。运行时行为在 Task 7/8 验证。

- [x] Step 5: 提交

```bash
git add src/communication/zenoh_transport.h src/communication/zenoh_transport.cpp CMakeLists.txt
git commit -m "feat(m1): 增加 Zenoh 传输封装"
```

---

## Task 6：CommunicationModule

**Files**

- Create: `src/communication/communication_module.h`
- Create: `src/communication/communication_module.cpp`
- Modify: `CMakeLists.txt`

**Interfaces**

- Consumes: `EventBus`（Task 2）、`AppConfig`（Task 3）、`serialization`（Task 4）、`ZenohTransport`（Task 5）
- Produces: `plane::communication::CommunicationModule`（`IModule` + `CommunicationApi`）

- [x] Step 1: 写 `src/communication/communication_module.h`

```cpp
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "communication/communication_api.h"
#include "communication/zenoh_transport.h"
#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"

namespace plane::communication {

/// 通信模块：Zenoh 收发、信封编解码、事件分发、Ack 与可靠重试。
class CommunicationModule final : public plane::core::IModule, public CommunicationApi {
   public:
    CommunicationModule(plane::core::EventBus& event_bus, const plane::core::AppConfig& config);
    ~CommunicationModule() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    bool sendToStation(const OutgoingMessage& message) override;
    bool sendToPlane(const std::string& plane_id, const OutgoingMessage& message) override;
    bool sendToAll(const OutgoingMessage& message) override;

   private:
    struct PendingMessage {
        std::string wire;
        std::string topic;
        std::int64_t deadline_ms = 0;
        int attempts = 0;
    };

    void enqueue_wire(std::string raw);
    void worker_loop();
    void on_wire_message(const std::string& raw);
    void handle_envelope(const plane::model::Message& envelope);
    void handle_ack(const plane::model::Message& envelope);
    void check_pending();
    bool send_message(const std::string& topic, const OutgoingMessage& message);
    bool send_ack(const plane::model::Message& envelope);
    void report_error(const std::string& detail) const;
    std::int64_t now_ms() const;

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    ZenohTransport transport_;
    std::mutex rx_mutex_;
    std::condition_variable rx_cv_;
    std::deque<std::string> rx_queue_;
    std::mutex pending_mutex_;
    std::unordered_map<std::string, PendingMessage> pending_;
    std::thread worker_;
    std::atomic<bool> running_{false};
};

}  // namespace plane::communication
```

- [x] Step 2: 写 `src/communication/communication_module.cpp`

```cpp
#include "communication/communication_module.h"

#include <chrono>
#include <cstddef>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

#include "communication/communication_events.h"
#include "communication/serialization.h"

namespace plane::communication {
namespace {

constexpr int kWorkerTickMs = 50;

bool is_reliable(plane::model::MessageType type) {
    return type == plane::model::MessageType::Alert || type == plane::model::MessageType::Ack;
}

}  // namespace

CommunicationModule::CommunicationModule(plane::core::EventBus& event_bus,
                                         const plane::core::AppConfig& config)
    : event_bus_(event_bus), config_(config) {}

CommunicationModule::~CommunicationModule() {
    stop();
}

std::string CommunicationModule::name() const {
    return "Communication";
}

bool CommunicationModule::init() {
    return true;
}

bool CommunicationModule::start() {
    ZenohEndpoints endpoints;
    endpoints.config_file = config_.zenoh_config_file;
    endpoints.connect = config_.zenoh_connect;
    endpoints.listen = config_.zenoh_listen;

    std::string error;
    if (!transport_.open(endpoints, error)) {
        report_error("Zenoh 打开失败: " + error);
        return false;
    }
    const auto rx_handler = [this](const std::string& payload) { enqueue_wire(payload); };
    if (!transport_.subscribe("sendToPlane/" + config_.plane_id, rx_handler, error)) {
        report_error("订阅 sendToPlane 失败: " + error);
        return false;
    }
    if (!transport_.subscribe(std::string(plane::model::topic_sendToAll), rx_handler, error)) {
        report_error("订阅 sendToAll 失败: " + error);
        return false;
    }

    running_ = true;
    worker_ = std::thread([this]() { worker_loop(); });
    spdlog::info("Communication 已启动: plane_id={}, station_id={}", config_.plane_id,
                 config_.station_id);
    return true;
}

void CommunicationModule::stop() {
    const bool was_running = running_.exchange(false);
    if (was_running) {
        rx_cv_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
    }
    transport_.close();
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_.clear();
}

void CommunicationModule::release() {}

bool CommunicationModule::sendToStation(const OutgoingMessage& message) {
    return send_message(std::string(plane::model::topic_sendToStation), message);
}

bool CommunicationModule::sendToPlane(const std::string& plane_id, const OutgoingMessage& message) {
    return send_message("sendToPlane/" + plane_id, message);
}

bool CommunicationModule::sendToAll(const OutgoingMessage& message) {
    return send_message(std::string(plane::model::topic_sendToAll), message);
}

void CommunicationModule::enqueue_wire(std::string raw) {
    {
        std::lock_guard<std::mutex> lock(rx_mutex_);
        const auto limit = static_cast<std::size_t>(config_.comm_rx_queue_max);
        if (rx_queue_.size() >= limit) {
            rx_queue_.pop_front();
            spdlog::warn("通信接收队列已满，丢弃最旧消息");
        }
        rx_queue_.push_back(std::move(raw));
    }
    rx_cv_.notify_one();
}

void CommunicationModule::worker_loop() {
    while (running_) {
        std::deque<std::string> batch;
        {
            std::unique_lock<std::mutex> lock(rx_mutex_);
            rx_cv_.wait_for(lock, std::chrono::milliseconds(kWorkerTickMs),
                            [this]() { return !rx_queue_.empty() || !running_; });
            batch.swap(rx_queue_);
        }
        for (const auto& raw : batch) {
            on_wire_message(raw);
        }
        check_pending();
    }
}

void CommunicationModule::on_wire_message(const std::string& raw) {
    plane::model::Message envelope;
    std::string error;
    if (!decode_envelope(raw, envelope, error)) {
        report_error("报文解析失败: " + error);
        return;
    }
    if (envelope.senderId == config_.plane_id) {
        return;
    }
    handle_envelope(envelope);
}

void CommunicationModule::handle_envelope(const plane::model::Message& envelope) {
    std::string error;
    switch (envelope.type) {
        case plane::model::MessageType::TaskInfo: {
            plane::model::TaskInfo value;
            if (!from_payload(envelope.payload, value, error)) {
                report_error("TaskInfo 解码失败: " + error);
                return;
            }
            event_bus_.publish(events::kTaskInfoReceived, value);
            send_ack(envelope);
            break;
        }
        case plane::model::MessageType::Command: {
            plane::model::Command value;
            if (!from_payload(envelope.payload, value, error)) {
                report_error("Command 解码失败: " + error);
                return;
            }
            event_bus_.publish(events::kCommandReceived, value);
            send_ack(envelope);
            break;
        }
        case plane::model::MessageType::SystemParam: {
            plane::model::SystemParam value;
            if (!from_payload(envelope.payload, value, error)) {
                report_error("SystemParam 解码失败: " + error);
                return;
            }
            event_bus_.publish(events::kSystemParamReceived, value);
            break;
        }
        case plane::model::MessageType::GcsTarget: {
            plane::model::GcsTarget value;
            if (!from_payload(envelope.payload, value, error)) {
                report_error("GcsTarget 解码失败: " + error);
                return;
            }
            event_bus_.publish(events::kGcsTargetReceived, value);
            break;
        }
        case plane::model::MessageType::Ack: {
            handle_ack(envelope);
            break;
        }
        default: {
            spdlog::warn("忽略未处理的消息类型: {}", message_type_to_string(envelope.type));
            break;
        }
    }
}

void CommunicationModule::handle_ack(const plane::model::Message& envelope) {
    plane::model::Ack ack;
    std::string error;
    if (!from_payload(envelope.payload, ack, error)) {
        report_error("Ack 解码失败: " + error);
        return;
    }
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (pending_.erase(ack.ackId) > 0) {
        spdlog::debug("收到 Ack: {}", ack.ackId);
    }
}

bool CommunicationModule::send_message(const std::string& topic, const OutgoingMessage& message) {
    plane::model::Message envelope;
    std::string error;
    if (!make_message(config_.plane_id, message, envelope, error)) {
        report_error("消息编码失败: " + error);
        return false;
    }
    const std::string wire = encode_envelope(envelope);
    if (!transport_.put(topic, wire, error)) {
        report_error("发送失败: " + error);
        return false;
    }
    if (is_reliable(envelope.type)) {
        PendingMessage pending;
        pending.wire = wire;
        pending.topic = topic;
        pending.deadline_ms = now_ms() + config_.comm_ack_timeout_ms;
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_[envelope.messageId] = std::move(pending);
    }
    return true;
}

bool CommunicationModule::send_ack(const plane::model::Message& envelope) {
    plane::model::Ack ack;
    ack.ackId = envelope.messageId;
    const std::string topic = envelope.senderId == config_.station_id
                                  ? std::string(plane::model::topic_sendToStation)
                                  : "sendToPlane/" + envelope.senderId;
    return send_message(topic, OutgoingMessage{ack});
}

void CommunicationModule::check_pending() {
    const std::int64_t now = now_ms();
    std::vector<PendingMessage> resend;
    std::vector<std::string> expired;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        for (auto it = pending_.begin(); it != pending_.end();) {
            if (now < it->second.deadline_ms) {
                ++it;
                continue;
            }
            if (it->second.attempts < config_.comm_ack_max_retries) {
                it->second.attempts += 1;
                it->second.deadline_ms = now + config_.comm_ack_timeout_ms;
                resend.push_back(it->second);
                ++it;
            } else {
                expired.push_back(it->first + " -> " + it->second.topic);
                it = pending_.erase(it);
            }
        }
    }
    for (const auto& pending : resend) {
        std::string error;
        if (transport_.put(pending.topic, pending.wire, error)) {
            spdlog::warn("可靠消息重试: topic={}, attempts={}", pending.topic, pending.attempts);
        } else {
            report_error("重试发送失败: " + error);
        }
    }
    for (const auto& item : expired) {
        report_error("可靠消息确认超时: " + item);
    }
}

void CommunicationModule::report_error(const std::string& detail) const {
    CommunicationError event;
    event.detail = detail;
    event_bus_.publish(events::kError, event);
}

std::int64_t CommunicationModule::now_ms() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace plane::communication
```

- [x] Step 3: 更新 `CMakeLists.txt` 的 `add_executable`，追加

```cmake
  src/communication/communication_module.cpp
```

- [x] Step 4: 构建验证

```bash
cmake --build build -j
```

Expected: 构建通过，无警告。运行时行为在 Task 7/8 验证。

- [x] Step 5: 提交

```bash
git add src/communication/communication_module.h src/communication/communication_module.cpp CMakeLists.txt
git commit -m "feat(m1): 增加 CommunicationModule"
```

---

## Task 7：接入真实 Communication 模块

**Files**

- Modify: `src/core/placeholder_modules.h`
- Modify: `src/core/placeholder_modules.cpp`
- Modify: `src/main.cpp`

**Interfaces**

- Consumes: `CommunicationModule`（Task 6）
- Produces: `make_placeholder_modules(skip)`；运行时真实通信模块替换占位

- [x] Step 1: 修改 `src/core/placeholder_modules.h`

```cpp
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "core/module.h"

namespace plane::core {

/// M0 占位模块：仅输出生命周期日志，后续里程碑逐个替换为真实模块。
/// skip 中列出的名称不生成占位（由真实模块替代）。
std::vector<std::unique_ptr<IModule>> make_placeholder_modules(
    const std::vector<std::string>& skip = {});

}  // namespace plane::core
```

- [x] Step 2: 修改 `src/core/placeholder_modules.cpp`

在 include 区加入 `<algorithm>`，函数体改为：

```cpp
std::vector<std::unique_ptr<IModule>> make_placeholder_modules(const std::vector<std::string>& skip) {
    const char* names[] = {
        "Plugin Management",    "Communication",    "UAV State Management",
        "StateMonitor",         "Target Management", "Flight Control",
        "Camera Control",       "Perception",       "Video Streaming",
        "Task Scheduler",       "Task Execution Engine",
    };
    std::vector<std::unique_ptr<IModule>> modules;
    modules.reserve(std::size(names));
    for (const char* module_name : names) {
        if (std::find(skip.begin(), skip.end(), module_name) != skip.end()) {
            continue;
        }
        modules.push_back(std::make_unique<PlaceholderModule>(module_name));
    }
    return modules;
}
```

- [x] Step 3: 修改 `src/main.cpp`，注册真实通信模块

将 M0 的模块注册块替换为：

```cpp
    plane::core::Runtime runtime;
    runtime.add_module(
        std::make_unique<plane::communication::CommunicationModule>(event_bus, config));
    for (auto& module : plane::core::make_placeholder_modules({"Communication"})) {
        runtime.add_module(std::move(module));
    }
```

并在 include 区加入：

```cpp
#include <memory>

#include "communication/communication_module.h"
```

- [x] Step 4: 构建并运行

```bash
cmake --build build -j
./build/onboard; echo "exit=$?"
```

Expected: 构建通过；日志中 `Communication 已启动: plane_id=uav1, station_id=gcs`，其余 10 个占位模块 init/start/stop 正常，exit=0。

- [x] Step 5: 提交

```bash
git add src/core/placeholder_modules.h src/core/placeholder_modules.cpp src/main.cpp
git commit -m "feat(m1): 接入真实 Communication 模块"
```

---

## Task 8：gcs_sim 联调工具

**Files**

- Create: `tools/gcs_sim/main.cpp`
- Modify: `CMakeLists.txt`

**Interfaces**

- Consumes: `message.h`（Task 1）、Zenoh
- Produces: 可执行目标 `gcs_sim`（供 Task 9 验收使用）

- [x] Step 1: 写 `tools/gcs_sim/main.cpp`

```cpp
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <set>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <zenoh/api.hxx>

#include "message.h"

namespace {

struct Options {
    std::string plane_id = "uav1";
    std::string sender_id = "gcs";
    bool no_ack = false;
    int listen_seconds = 3;
    int ack_timeout_ms = 2000;
};

struct Envelope {
    std::string sender_id;
    std::string message_id;
    std::string type;
    std::string payload;
};

bool parse_envelope(const std::string& wire, Envelope& out) {
    const nlohmann::json envelope = nlohmann::json::parse(wire, nullptr, false);
    if (envelope.is_discarded() || !envelope.is_object()) {
        return false;
    }
    out.sender_id = envelope.value("senderId", std::string{});
    out.message_id = envelope.value("messageId", std::string{});
    out.type = envelope.value("type", std::string{});
    out.payload = envelope.value("payload", std::string{});
    return !out.message_id.empty() && !out.type.empty();
}

Options parse_options(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--plane-id" && i + 1 < argc) {
            options.plane_id = argv[++i];
        } else if (arg == "--sender-id" && i + 1 < argc) {
            options.sender_id = argv[++i];
        } else if (arg == "--no-ack") {
            options.no_ack = true;
        } else if (arg == "--listen-seconds" && i + 1 < argc) {
            options.listen_seconds = std::stoi(argv[++i]);
        } else if (arg == "--ack-timeout-ms" && i + 1 < argc) {
            options.ack_timeout_ms = std::stoi(argv[++i]);
        }
    }
    return options;
}

std::string build_wire(const std::string& sender_id, const std::string& message_id,
                       const std::string& type, const std::string& payload) {
    const nlohmann::json envelope{{"senderId", sender_id},
                                  {"messageId", message_id},
                                  {"type", type},
                                  {"payload", payload}};
    return envelope.dump();
}

std::string send_typed(const zenoh::Session& session, const Options& options,
                       const std::string& type, const std::string& payload,
                       std::uint64_t sequence) {
    const std::string message_id = options.sender_id + "-" + std::to_string(sequence);
    const std::string topic = "sendToPlane/" + options.plane_id;
    session.put(zenoh::KeyExpr(topic), zenoh::Bytes(build_wire(options.sender_id, message_id, type, payload)));
    spdlog::info("已发送 {}: messageId={}", type, message_id);
    return message_id;
}

}  // namespace

int main(int argc, char** argv) {
    const Options options = parse_options(argc, argv);

    std::mutex mutex;
    std::condition_variable cv;
    std::set<std::string> acked_ids;
    std::atomic<std::uint64_t> sequence{100};

    zenoh::Session session = zenoh::Session::open(zenoh::Config::create_default());
    zenoh::Subscriber<void> subscriber = session.declare_subscriber(
        zenoh::KeyExpr(std::string(plane::model::topic_sendToStation)),
        [&](zenoh::Sample& sample) {
            Envelope envelope;
            if (!parse_envelope(sample.get_payload().as_string(), envelope)) {
                spdlog::warn("收到无法解析的报文");
                return;
            }
            if (envelope.sender_id == options.sender_id) {
                return;
            }
            if (envelope.type == "Ack") {
                const nlohmann::json payload = nlohmann::json::parse(envelope.payload, nullptr, false);
                if (!payload.is_discarded() && payload.contains("ackId")) {
                    std::lock_guard<std::mutex> lock(mutex);
                    acked_ids.insert(payload.at("ackId").get<std::string>());
                    cv.notify_all();
                }
            }
            spdlog::info("收到 {}: senderId={}, messageId={}", envelope.type, envelope.sender_id,
                         envelope.message_id);
            if (!options.no_ack && (envelope.type == "Alert" || envelope.type == "Ack")) {
                const nlohmann::json payload{{"ackId", envelope.message_id}};
                const std::string message_id =
                    options.sender_id + "-ack-" + std::to_string(++sequence);
                const std::string topic = "sendToPlane/" + envelope.sender_id;
                session.put(zenoh::KeyExpr(topic),
                            zenoh::Bytes(build_wire(options.sender_id, message_id, "Ack", payload.dump())));
            }
        },
        []() {});

    const std::string task_payload =
        nlohmann::json{{"taskId", "task-1"},
                       {"taskCode", "suicide"},
                       {"targetTrackId", ""},
                       {"taskAction", "Start"}}
            .dump();
    const std::string task_id = send_typed(session, options, "TaskInfo", task_payload, ++sequence);

    const std::string command_payload =
        nlohmann::json{{"commandId", "cmd-1"},
                       {"type", "Arm"},
                       {"takeoffHeight_m", 0.0F},
                       {"waypoints", nlohmann::json::array()}}
            .dump();
    const std::string command_id =
        send_typed(session, options, "Command", command_payload, ++sequence);

    bool task_acked = false;
    bool command_acked = false;
    {
        std::unique_lock<std::mutex> lock(mutex);
        task_acked = cv.wait_for(lock, std::chrono::milliseconds(options.ack_timeout_ms),
                                 [&]() { return acked_ids.count(task_id) > 0; });
        command_acked = cv.wait_for(lock, std::chrono::milliseconds(options.ack_timeout_ms),
                                    [&]() { return acked_ids.count(command_id) > 0; });
    }
    spdlog::info("Ack 结果: TaskInfo={}, Command={}", task_acked, command_acked);

    std::this_thread::sleep_for(std::chrono::seconds(options.listen_seconds));
    return task_acked && command_acked ? 0 : 1;
}
```

- [x] Step 2: 更新 `CMakeLists.txt`，在 `onboard` 目标之后追加

```cmake
add_executable(gcs_sim tools/gcs_sim/main.cpp)
target_include_directories(gcs_sim PRIVATE src)
target_compile_options(gcs_sim PRIVATE -Wall -Wextra)
target_link_libraries(gcs_sim PRIVATE
  plane::common_include
  plane::nlohmann_json
  fmt::fmt
  spdlog::spdlog
  zenohc::lib
  zenohcxx::zenohc
)
```

- [x] Step 3: 构建冒烟

```bash
cmake --build build -j
test -x build/gcs_sim && echo "gcs_sim ok"
```

Expected: 构建通过；`gcs_sim ok`。与 onboard 的联合验证在 Task 9。

- [x] Step 4: 提交

```bash
git add tools/gcs_sim/main.cpp CMakeLists.txt
git commit -m "feat(m1): 增加 gcs_sim 联调工具"
```

---

## Task 9：M1 验收

**Files**

- Modify: `docs/plans/m1-communication-tasks.md`（勾选状态）
- Modify: `docs/DEV_PLAN.md`（索引状态）

- [x] Step 1: 干净构建

```bash
rm -rf build logs
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Expected: 无 error；本项目代码无 warning。

- [x] Step 2: 入站控制与 Ack（终端 A/B）

终端 A：

```bash
./build/onboard
```

终端 B：

```bash
./build/gcs_sim --plane-id uav1
```

Expected：

- onboard 日志出现收到的 TaskInfo/Command 相关处理与 `Communication 已启动`；
- gcs_sim 打印 `Ack 结果: TaskInfo=true, Command=true` 且退出码 0。

- [x] Step 3: 重试与超时（终端 A 保持运行）

终端 B：

```bash
./build/gcs_sim --plane-id uav1 --no-ack; echo "exit=$?"
```

Expected：onboard 出现 `可靠消息重试: topic=sendToStation ...`（≥1 次）与 `通信错误事件: 可靠消息确认超时...`；gcs_sim exit=0（它仍收到了 onboard 的 Ack，只是不回 ack）。

- [x] Step 4: 自消息过滤

终端 B：

```bash
./build/gcs_sim --plane-id uav1 --sender-id uav1
```

Expected：onboard 不产生 TaskInfo/Command 事件日志（senderId 等于本机 plane_id，被丢弃）。

- [x] Step 5: 代码规范

```bash
clang-format --dry-run --Werror $(git ls-files src tools)
grep -rn "std::cout\|printf(" src/ tools/ || echo "no cout/printf"
```

- [x] Step 6: DoD 走查并更新文档

逐条核对 DEV_PLAN §10 DoD，更新：

- `docs/DEV_PLAN.md` §11 索引中 M1 状态 → 已完成；
- 本文档头部状态与各 Task 勾选。

- [x] Step 7: 收尾提交（仅当有改动）

```bash
git add docs src tools CMakeLists.txt config
git commit -m "chore(m1): M1 验收与文档更新"
```

---

## 附：M1 执行后的文档动作

- 若实现证明设计需要调整（例如 Zenoh 配置键、Ack 口径），先更新 `m1-communication-design.md` 并说明原因；
- 架构层变化（如 Message 信封字段）需同步 `docs/ARCHITECTURE.md` 并经用户确认；
- common 仓提交需用户确认。

---

## 执行记录与偏差

1. **zenoh-cpp Options 为嵌套类型**（Task 5）：本地 zenoh-cpp 1.10.0 中 `SessionOptions`/`SubscriberOptions`/`PutOptions` 是 `zenoh::Session` 的嵌套类型，按实际 API 使用 `zenoh::Session::XxxOptions`。
2. **Zenoh key expression 不允许前导 `/`**（Task 7 发现）：`message.h` 的 topic 常量已改为无前导 `/`（common 仓 `b819f45`）并同步本设计 §4；`communication_module` 与 `gcs_sim` 同步修改；Zenoh 边界保留去 `/` 规范化兼容旧值。
3. **进程常驻运行循环**（Task 9 发现计划缺口）：原计划 main 在模块启动后立即退出，无法联调。已补充 SIGINT/SIGTERM 驱动的运行循环与优雅关闭；M0 的「启动后正常退出」由本里程碑升级为「常驻直到退出信号」。
4. **Task 9 验收证据（2026-09-17）**
   - Test A（正常联调）：gcs_sim `Ack 结果: TaskInfo=true, Command=true`，exit=0；onboard 无重试、无错误事件。
   - Test B（`--no-ack`）：onboard 出现 6 次 `可靠消息重试`（2 条消息 × 3 次），随后 2 条 `通信错误事件: 可靠消息确认超时`；gcs_sim exit=0。
   - Test C（`--sender-id uav1` 自消息过滤）：gcs_sim `Ack 结果: TaskInfo=false, Command=false`，exit=1；onboard 无重试、无错误事件（消息被 senderId 过滤丢弃）。
   - Task 9 Step 2 中「onboard 日志出现 TaskInfo/Command 处理」以 gcs_sim 收到匹配的 Ack 为证据；事件发布本身在 M2 出现消费者后才可观测。
5. `clang-format --dry-run --Werror $(git ls-files src tools)` 通过；`src/`、`tools/` 无 `std::cout/printf`。
