# M2 任务级计划（UAV 状态管理与周期上报）

> 设计依据：`m2-uav-state-design.md`（已确认，GCS 链路按 2 秒判定）。
> 状态：已完成并通过 Task 7 验收（2026-09-17）。执行偏差与证据见文末。
> 执行位置：工程根 `onboard/`。
> 验证原则：每个 Task 结束后 x64 Release 构建通过；不写测试代码（SPEC §11）。运行时行为在 Task 6/7 集中验证。
> 提交：每个 Task 一个提交。

---

## Task 1：状态接口与类型

**Files**

- Create: `src/state/state_types.h`
- Create: `src/state/state_store_api.h`
- Create: `src/state/state_events.h`
- Create: `src/task/task_state_provider.h`

**Interfaces**

- Produces: `plane::state::GcsLinkState`、`StateStoreApi`、状态事件常量、`plane::task::TaskStateProvider` / `NoTaskStateProvider`

- [x] Step 1: 写 `src/state/state_types.h`

```cpp
#pragma once

#include <cstdint>

namespace plane::state {

/// 地面站通信链路状态（SPEC §4.3）。
struct GcsLinkState {
    bool connected = false;
    std::int64_t last_seen_ms = 0;
};

}  // namespace plane::state
```

- [x] Step 2: 写 `src/state/state_store_api.h`

```cpp
#pragma once

#include <string>
#include <vector>

#include "message.h"
#include "state/state_types.h"

namespace plane::state {

/// 状态管理对外稳定接口：查询 + 写入（写入仅供数据来源模块调用）。
class StateStoreApi {
   public:
    virtual ~StateStoreApi() = default;

    virtual plane::model::VehicleState own_state() const = 0;
    virtual std::vector<plane::model::VehicleState> peer_states() const = 0;
    virtual bool peer_state(const std::string& plane_id, plane::model::VehicleState& out) const = 0;
    virtual GcsLinkState gcs_link() const = 0;

    virtual void update_own_state(const plane::model::VehicleState& state) = 0;
    virtual void update_peer_state(const plane::model::VehicleState& state) = 0;
    virtual void notify_station_seen() = 0;
};

}  // namespace plane::state
```

- [x] Step 3: 写 `src/state/state_events.h`

```cpp
#pragma once

#include <string_view>

namespace plane::state {
namespace events {

inline constexpr std::string_view kOwnStateChanged = "state.own_changed";
inline constexpr std::string_view kPeerStateChanged = "state.peer_changed";
inline constexpr std::string_view kGcsLinkChanged = "state.gcs_link_changed";

}  // namespace events
}  // namespace plane::state
```

- [x] Step 4: 写 `src/task/task_state_provider.h`

```cpp
#pragma once

#include "message.h"

namespace plane::task {

/// 当前任务状态数据源；M5 由 Task Scheduler 实现（ARCHITECTURE §18）。
class TaskStateProvider {
   public:
    virtual ~TaskStateProvider() = default;

    virtual plane::model::TaskState current_task_state() const = 0;
};

/// M2 默认实现：无任务（TaskStatus::None）。
class NoTaskStateProvider final : public TaskStateProvider {
   public:
    plane::model::TaskState current_task_state() const override { return {}; }
};

}  // namespace plane::task
```

- [x] Step 5: 构建

```bash
cmake --build build -j
```

Expected: 构建通过（接口头暂未被引用，编译验证在 Task 3/5）。

- [x] Step 6: 提交

```bash
git add src/state/state_types.h src/state/state_store_api.h src/state/state_events.h src/task/task_state_provider.h
git commit -m "feat(m2): 增加状态与任务状态接口"
```

---

## Task 2：GCS 链路配置项

**Files**

- Modify: `src/core/config.h`
- Modify: `src/core/config.cpp`
- Modify: `config/onboard.conf`

**Interfaces**

- Produces: `AppConfig::gcs_link_timeout_ms`（默认 2000）

- [x] Step 1: `src/core/config.h` 的 `AppConfig` 中，在 `comm_rx_queue_max` 之后追加

```cpp
    int gcs_link_timeout_ms = 2000;
```

- [x] Step 2: `src/core/config.cpp` 解析分支中，在 `comm_rx_queue_max` 分支之后追加

```cpp
        } else if (key == "gcs_link_timeout_ms") {
            out.gcs_link_timeout_ms = std::stoi(value);
```

- [x] Step 3: `config/onboard.conf` 末尾追加

```text
# 地面站链路判定（2 秒未收到 GCS 报文判定断开）
gcs_link_timeout_ms = 2000
```

- [x] Step 4: 构建并运行

```bash
cmake --build build -j
./build/onboard; echo "exit=$?"
```

Expected: 构建通过；运行后无「未知配置项」警告，随后按 Ctrl+C 退出（exit=0）。

- [x] Step 5: 提交

```bash
git add src/core/config.h src/core/config.cpp config/onboard.conf
git commit -m "feat(m2): 增加 GCS 链路超时配置"
```

---

## Task 3：StateStore 实现

**Files**

- Create: `src/state/state_store.h`
- Create: `src/state/state_store.cpp`
- Modify: `CMakeLists.txt`

**Interfaces**

- Consumes: `StateStoreApi`/`GcsLinkState`（Task 1）、`AppConfig::gcs_link_timeout_ms`（Task 2）
- Produces: `plane::state::StateStore`（`core::IModule` + `StateStoreApi`）

- [x] Step 1: 写 `src/state/state_store.h`

```cpp
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"
#include "state/state_store_api.h"
#include "state/state_types.h"

namespace plane::state {

/// UAV 状态管理：本机、其他无人机与地面站链路的统一存储与查询（ARCHITECTURE §11）。
class StateStore final : public plane::core::IModule, public StateStoreApi {
   public:
    StateStore(plane::core::EventBus& event_bus, const plane::core::AppConfig& config);
    ~StateStore() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    plane::model::VehicleState own_state() const override;
    std::vector<plane::model::VehicleState> peer_states() const override;
    bool peer_state(const std::string& plane_id, plane::model::VehicleState& out) const override;
    GcsLinkState gcs_link() const override;

    void update_own_state(const plane::model::VehicleState& state) override;
    void update_peer_state(const plane::model::VehicleState& state) override;
    void notify_station_seen() override;

   private:
    void tick_loop();
    std::int64_t now_ms() const;

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    mutable std::mutex mutex_;
    plane::model::VehicleState own_;
    std::unordered_map<std::string, plane::model::VehicleState> peers_;
    bool gcs_connected_ = false;
    std::int64_t gcs_last_seen_ms_ = 0;
    std::thread tick_;
    std::atomic<bool> running_{false};
};

}  // namespace plane::state
```

- [x] Step 2: 写 `src/state/state_store.cpp`

```cpp
#include "state/state_store.h"

#include <chrono>
#include <utility>

#include <spdlog/spdlog.h>

#include "state/state_events.h"

namespace plane::state {
namespace {

constexpr int kTickIntervalMs = 200;

}  // namespace

StateStore::StateStore(plane::core::EventBus& event_bus, const plane::core::AppConfig& config)
    : event_bus_(event_bus), config_(config) {
    own_.planeId = config.plane_id;
}

StateStore::~StateStore() {
    stop();
}

std::string StateStore::name() const {
    return "UAV State Management";
}

bool StateStore::init() {
    return true;
}

bool StateStore::start() {
    running_ = true;
    tick_ = std::thread([this]() { tick_loop(); });
    return true;
}

void StateStore::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    if (tick_.joinable()) {
        tick_.join();
    }
}

void StateStore::release() {}

plane::model::VehicleState StateStore::own_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return own_;
}

std::vector<plane::model::VehicleState> StateStore::peer_states() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<plane::model::VehicleState> result;
    result.reserve(peers_.size());
    for (const auto& item : peers_) {
        result.push_back(item.second);
    }
    return result;
}

bool StateStore::peer_state(const std::string& plane_id, plane::model::VehicleState& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = peers_.find(plane_id);
    if (it == peers_.end()) {
        return false;
    }
    out = it->second;
    return true;
}

GcsLinkState StateStore::gcs_link() const {
    std::lock_guard<std::mutex> lock(mutex_);
    GcsLinkState link;
    link.connected = gcs_connected_;
    link.last_seen_ms = gcs_last_seen_ms_;
    return link;
}

void StateStore::update_own_state(const plane::model::VehicleState& state) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        own_ = state;
    }
    event_bus_.publish(events::kOwnStateChanged, state);
}

void StateStore::update_peer_state(const plane::model::VehicleState& state) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        peers_[state.planeId] = state;
    }
    spdlog::info("他机状态更新: {}", state.planeId);
    event_bus_.publish(events::kPeerStateChanged, state);
}

void StateStore::notify_station_seen() {
    bool became_connected = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        gcs_last_seen_ms_ = now_ms();
        if (!gcs_connected_) {
            gcs_connected_ = true;
            became_connected = true;
        }
    }
    if (became_connected) {
        GcsLinkState link;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            link.connected = gcs_connected_;
            link.last_seen_ms = gcs_last_seen_ms_;
        }
        spdlog::info("地面站链路: connected=true");
        event_bus_.publish(events::kGcsLinkChanged, link);
    }
}

void StateStore::tick_loop() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kTickIntervalMs));
        bool became_disconnected = false;
        GcsLinkState link;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (gcs_connected_ && now_ms() - gcs_last_seen_ms_ > config_.gcs_link_timeout_ms) {
                gcs_connected_ = false;
                became_disconnected = true;
            }
            link.connected = gcs_connected_;
            link.last_seen_ms = gcs_last_seen_ms_;
        }
        if (became_disconnected) {
            spdlog::info("地面站链路: connected=false");
            event_bus_.publish(events::kGcsLinkChanged, link);
        }
    }
}

std::int64_t StateStore::now_ms() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace plane::state
```

- [x] Step 3: `CMakeLists.txt` 的 `add_executable(onboard ...)` 文件列表中追加一行

```cmake
  src/state/state_store.cpp
```

- [x] Step 4: 构建并运行

```bash
cmake --build build -j
./build/onboard; echo "exit=$?"
```

Expected: 构建通过；运行日志中占位模块 `UAV State Management` 仍存在（Task 6 才替换），Ctrl+C 退出，exit=0。

- [x] Step 5: 提交

```bash
git add src/state/state_store.h src/state/state_store.cpp CMakeLists.txt
git commit -m "feat(m2): 增加 StateStore 状态管理"
```

---

## Task 4：serialization 增加 VehicleState 解码

**Files**

- Modify: `src/communication/serialization.h`
- Modify: `src/communication/serialization.cpp`

**Interfaces**

- Produces: `bool from_payload(const std::string&, plane::model::VehicleState&, std::string&)`

- [x] Step 1: `serialization.h` 的 `from_payload` 声明区追加

```cpp
bool from_payload(const std::string& payload, plane::model::VehicleState& out, std::string& error);
```

- [x] Step 2: `serialization.cpp` 在 `to_payload(const VehicleState&)` 之后新增实现

```cpp
bool from_payload(const std::string& payload, plane::model::VehicleState& out, std::string& error) {
    try {
        const json in = json::parse(payload);
        out.planeId = in.at("planeId").get<std::string>();
        geo_point_from_json(in.at("geoPos"), out.geoPos);
        if (in.contains("startPos")) {
            geo_point_from_json(in.at("startPos"), out.startPos);
        }
        out.roll_deg = in.value("roll_deg", 0.0F);
        out.pitch_deg = in.value("pitch_deg", 0.0F);
        out.yaw_deg = in.value("yaw_deg", 0.0F);
        out.vn_m_s = in.value("vn_m_s", 0.0F);
        out.ve_m_s = in.value("ve_m_s", 0.0F);
        out.vd_m_s = in.value("vd_m_s", 0.0F);
        out.airspeed_m_s = in.value("airspeed_m_s", 0.0F);
        out.voltage_v = in.value("voltage_v", 0.0F);
        out.flightMode = in.value("flightMode", std::string{});
        out.fcLinkState = in.value("fcLinkState", std::string{});
        out.currentBehaviorTree = in.value("currentBehaviorTree", std::string{});
        out.currentNode = in.value("currentNode", std::string{});
        out.cameraAngle_roll_deg = in.value("cameraAngle_roll_deg", 0.0F);
        out.cameraAngle_pitch_deg = in.value("cameraAngle_pitch_deg", 0.0F);
        out.cameraAngle_yaw_deg = in.value("cameraAngle_yaw_deg", 0.0F);
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}
```

- [x] Step 3: 构建与格式

```bash
cmake --build build -j
clang-format --dry-run --Werror src/communication/serialization.h src/communication/serialization.cpp
```

Expected: 构建通过；格式通过（有差异先 `clang-format -i` 再构建）。

- [x] Step 4: 提交

```bash
git add src/communication/serialization.h src/communication/serialization.cpp
git commit -m "feat(m2): 支持 VehicleState 解码"
```

---

## Task 5：Communication 扩展与 main 接线

**Files**

- Modify: `src/communication/communication_module.h`
- Modify: `src/communication/communication_module.cpp`
- Modify: `src/main.cpp`

**Interfaces**

- Consumes: `StateStoreApi`（Task 1/3）、`TaskStateProvider`（Task 1）、`from_payload(VehicleState)`（Task 4）
- Produces: 构造签名 `CommunicationModule(EventBus&, const AppConfig&, state::StateStoreApi&, task::TaskStateProvider&)`

- [x] Step 1: `communication_module.h` 修改

1) include 区追加：

```cpp
#include "state/state_store_api.h"
#include "task/task_state_provider.h"
```

2) 构造函数声明改为：

```cpp
    CommunicationModule(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                        plane::state::StateStoreApi& state_store,
                        plane::task::TaskStateProvider& task_state_provider);
```

3) private 方法区追加：

```cpp
    void report_states();
```

4) 成员区追加：

```cpp
    plane::state::StateStoreApi& state_store_;
    plane::task::TaskStateProvider& task_state_provider_;
    std::int64_t last_report_ms_ = 0;
```

- [x] Step 2: `communication_module.cpp` 修改

1) 构造函数定义改为：

```cpp
CommunicationModule::CommunicationModule(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                                         plane::state::StateStoreApi& state_store,
                                         plane::task::TaskStateProvider& task_state_provider)
    : event_bus_(event_bus),
      config_(config),
      state_store_(state_store),
      task_state_provider_(task_state_provider) {}
```

2) `worker_loop()` 中 `check_pending();` 之前插入：

```cpp
        report_states();
```

3) `handle_envelope()` 开头（`std::string error;` 之后）插入：

```cpp
    if (envelope.senderId == config_.station_id) {
        state_store_.notify_station_seen();
    }
```

4) `handle_envelope()` 的 switch 中，在 `case MessageType::Ack` 之前插入：

```cpp
        case plane::model::MessageType::VehicleState: {
            if (envelope.senderId == config_.station_id) {
                break;
            }
            plane::model::VehicleState value;
            if (!from_payload(envelope.payload, value, error)) {
                report_error("VehicleState 解码失败: " + error);
                return;
            }
            state_store_.update_peer_state(value);
            break;
        }
```

5) 在 `check_pending()` 实现之前新增：

```cpp
void CommunicationModule::report_states() {
    const std::int64_t now = now_ms();
    if (now - last_report_ms_ < config_.state_report_period_ms) {
        return;
    }
    last_report_ms_ = now;
    sendToAll(state_store_.own_state());
    sendToStation(task_state_provider_.current_task_state());
}
```

- [x] Step 3: `src/main.cpp` 接线

include 区追加：

```cpp
#include "state/state_store.h"
#include "task/task_state_provider.h"
```

模块注册块改为：

```cpp
    plane::task::NoTaskStateProvider task_state_provider;
    auto state_store = std::make_unique<plane::state::StateStore>(event_bus, config);
    auto communication = std::make_unique<plane::communication::CommunicationModule>(
        event_bus, config, *state_store, task_state_provider);

    plane::core::Runtime runtime;
    runtime.add_module(std::move(state_store));
    runtime.add_module(std::move(communication));
    for (auto& module : plane::core::make_placeholder_modules({"Communication", "UAV State Management"})) {
        runtime.add_module(std::move(module));
    }
```

（`make_placeholder_modules` 的 skip 参数在 M1 已支持，无需改 placeholder_modules.cpp。）

- [x] Step 4: 构建并运行

```bash
cmake --build build -j
./build/onboard; echo "exit=$?"
```

Expected: 构建通过；日志出现 `Communication 已启动`，真实 `UAV State Management` + 9 个占位模块，Ctrl+C 退出 exit=0。

- [x] Step 5: 格式与提交

```bash
clang-format --dry-run --Werror src/main.cpp src/communication/communication_module.h src/communication/communication_module.cpp
git add src/main.cpp src/communication/communication_module.h src/communication/communication_module.cpp
git commit -m "feat(m2): Communication 写入状态并周期上报"
```

---

## Task 6：gcs_sim 状态观测与友机模拟

**Files**

- Modify: `tools/gcs_sim/main.cpp`

**Interfaces**

- Consumes: `sendToAll` 订阅（M2 上报）、`VehicleState`/`TaskState` 线格式
- Produces: `--send-peer-state <plane-id>`、状态计数汇总

- [x] Step 1: Options 增加字段与解析

```cpp
struct Options {
    std::string plane_id = "uav1";
    std::string sender_id = "gcs";
    std::string peer_plane_id;
    bool no_ack = false;
    int listen_seconds = 3;
    int ack_timeout_ms = 2000;
};
```

解析循环追加：

```cpp
        } else if (arg == "--send-peer-state" && i + 1 < argc) {
            options.peer_plane_id = argv[++i];
```

- [x] Step 2: 订阅 `sendToAll` 并统计状态报文

1) 在 main 中新增计数器与订阅（与 `sendToStation` 使用同一回调）：

```cpp
    std::atomic<std::uint64_t> vehicle_state_count{0};
    std::atomic<std::uint64_t> task_state_count{0};

    auto on_sample = [&](zenoh::Sample& sample) {
        Envelope envelope;
        if (!parse_envelope(sample.get_payload().as_string(), envelope)) {
            spdlog::warn("收到无法解析的报文");
            return;
        }
        if (envelope.sender_id == options.sender_id) {
            return;
        }
        if (envelope.type == "VehicleState") {
            const nlohmann::json payload = nlohmann::json::parse(envelope.payload, nullptr, false);
            if (!payload.is_discarded()) {
                const nlohmann::json geo = payload.value("geoPos", nlohmann::json::object());
                spdlog::info("收到 VehicleState: planeId={}, voltage={}, lat={}, lon={}, height={}",
                             payload.value("planeId", std::string{}), payload.value("voltage_v", 0.0F),
                             geo.value("latitude", 0.0), geo.value("longitude", 0.0),
                             geo.value("height", 0.0F));
            }
            vehicle_state_count.fetch_add(1);
            return;
        }
        if (envelope.type == "TaskState") {
            const nlohmann::json payload = nlohmann::json::parse(envelope.payload, nullptr, false);
            if (!payload.is_discarded()) {
                spdlog::info("收到 TaskState: taskId={}, status={}, node={}",
                             payload.value("taskId", std::string{}),
                             payload.value("status", std::string{}),
                             payload.value("currentNode", std::string{}));
            }
            task_state_count.fetch_add(1);
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
            const std::string message_id = options.sender_id + "-ack-" + std::to_string(++sequence);
            const std::string topic = "sendToPlane/" + envelope.sender_id;
            session.put(zenoh::KeyExpr(topic),
                        zenoh::Bytes(build_wire(options.sender_id, message_id, "Ack", payload.dump())));
        }
    };

    zenoh::Subscriber<void> station_subscriber =
        session.declare_subscriber(zenoh::KeyExpr(std::string(plane::model::topic_sendToStation)), on_sample, []() {});
    zenoh::Subscriber<void> all_subscriber =
        session.declare_subscriber(zenoh::KeyExpr(std::string(plane::model::topic_sendToAll)), on_sample, []() {});
```

2) 删除原 `subscriber` 内联 lambda，保留其余逻辑。

- [x] Step 3: 发送友机状态（在 Command 发送与 Ack 等待之后、listen 之前）

```cpp
    if (!options.peer_plane_id.empty()) {
        const nlohmann::json payload{
            {"planeId", options.peer_plane_id},
            {"geoPos", {{"latitude", 30.5}, {"longitude", 120.5}, {"height", 80.0F}, {"altitude", 90.0F}}},
            {"voltage_v", 12.5F},
            {"airspeed_m_s", 15.0F},
            {"flightMode", "AUTO.MISSION"},
            {"fcLinkState", "connected"}};
        const std::string message_id = options.peer_plane_id + "-" + std::to_string(++sequence);
        session.put(zenoh::KeyExpr("sendToPlane/" + options.plane_id),
                    zenoh::Bytes(build_wire(options.peer_plane_id, message_id, "VehicleState", payload.dump())));
        spdlog::info("已发送友机状态: planeId={}", options.peer_plane_id);
    }
```

- [x] Step 4: listen 结束后输出汇总

```cpp
    std::this_thread::sleep_for(std::chrono::seconds(options.listen_seconds));
    spdlog::info("状态统计: VehicleState={}, TaskState={}", vehicle_state_count.load(),
                 task_state_count.load());
    return task_acked && command_acked ? 0 : 1;
```

- [x] Step 5: 构建与格式

```bash
cmake --build build -j
clang-format --dry-run --Werror tools/gcs_sim/main.cpp
```

Expected: 构建通过；格式通过（有差异先 `clang-format -i` 再构建）。

- [x] Step 6: 提交

```bash
git add tools/gcs_sim/main.cpp
git commit -m "feat(m2): gcs_sim 支持状态观测与友机模拟"
```

---

## Task 7：M2 验收

**Files**

- Modify: `docs/plans/m2-uav-state-tasks.md`（勾选）
- Modify: `docs/DEV_PLAN.md`（索引状态）

- [x] Step 1: 干净构建

```bash
rm -rf build logs
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

- [x] Step 2: 周期上报（终端 A/B）

终端 A：

```bash
./build/onboard
```

终端 B：

```bash
./build/gcs_sim --plane-id uav1 --listen-seconds 4
```

Expected：

- gcs_sim 每秒收到 `VehicleState`（planeId=uav1）与 `TaskState`（status 为空或 None）；
- gcs_sim 末尾 `状态统计: VehicleState≈4, TaskState≈4`；Ack 结果 true。

- [x] Step 3: 周期可配

```bash
printf 'state_report_period_ms = 300\ngcs_link_timeout_ms = 2000\n' > /tmp/opencode/m2_fast.conf
./build/onboard --config /tmp/opencode/m2_fast.conf
```

终端 B 运行 `./build/gcs_sim --plane-id uav1 --listen-seconds 3`，观察到约 3 条/秒。

- [x] Step 4: 他机状态

终端 B：

```bash
./build/gcs_sim --plane-id uav1 --send-peer-state uav2 --listen-seconds 1
```

Expected：onboard 日志出现 `他机状态更新: uav2`。

- [x] Step 5: GCS 链路通断

观测 onboard 日志：

- gcs_sim 发送报文后出现 `地面站链路: connected=true`；
- gcs_sim 退出后超过 2 秒出现 `地面站链路: connected=false`。

- [x] Step 6: 规范与回归

```bash
clang-format --dry-run --Werror $(git ls-files src tools)
grep -rn "std::cout\|printf(" src/ tools/ || echo "no cout/printf"
./build/gcs_sim --plane-id uav1   # M1 回归：Ack 结果应为 true
```

- [x] Step 7: DoD 走查与文档更新

- 更新 `docs/DEV_PLAN.md` §11 索引：M2 状态 → 已完成；
- 勾选本文档；
- 记录执行偏差（如有）。

- [x] Step 8: 收尾提交

```bash
git add docs src tools CMakeLists.txt config
git commit -m "chore(m2): M2 验收与文档更新"
```

---

## 执行记录与偏差

1. **Task 6 首次执行失败（外部目录权限被拒）**：子代理为核对 `VehicleState` 定义，执行 `find / -name "message.h" -path "*plane*"`，命中历史工程 `/home/zsh/code/counterswarm/plane/source/domain/message.h`，触发外部目录权限询问后被用户拒绝；未产生任何文件改动。已在 `AGENTS.MD` 增加「检索与外部目录约束」（提交 `9936429`），明确公共头文件路径为 `../common/common_include` 且禁止扫描工程根之外；重跑后完成。
2. **验收证据（2026-09-17）**
   - 周期上报：`gcs_sim --listen-seconds 4` 收到 `VehicleState=4, TaskState=4`，Ack 结果 true，exit=0；
   - 周期可配：`state_report_period_ms=300` 时 3 秒收到 `VehicleState=10, TaskState=10`；
   - 他机状态：`--send-peer-state uav2` 后 onboard 日志 `他机状态更新: uav2`；
   - GCS 链路：收到 GCS 报文时 `地面站链路: connected=true`，gcs_sim 退出超过 2 秒后 `地面站链路: connected=false`；
   - M1 回归：TaskInfo/Command 的 Ack 行为正常。
3. `clang-format --dry-run --Werror $(git ls-files src tools)` 通过；`src/`、`tools/` 无 `std::cout/printf`。
4. 本机状态为默认值（M3 接入 `update_own_state` 后填充真实数据），属 M2 预期范围。
