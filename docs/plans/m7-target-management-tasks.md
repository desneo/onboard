# M7 任务级计划（Target Management）

> 设计依据：`m7-target-management-design.md`（逐项确认）。
> 执行位置：工程根 `onboard/`。
> 验证原则：每个 Task 结束后 x64 Release 构建通过；不写测试代码（SPEC §11）。
> 无 SITL/无图像源：Task 5 为部分验收，真实感知源目标更新待用户验证。

---

## Task 1：Perception 自检发布结果事件

**Files**

- Modify: `src/perception/perception.cpp`

说明：自检当前只打日志不发事件，TargetStore 无法观测；补发 `perception.detection/tracking/guidance` 事件。

- [ ] Step 1: `run_selftest` 中 detect 成功后发布事件

将：

```cpp
    std::vector<plane::model::DetectTarget> targets;
    if (detection_->detect(frame, targets, error)) {
        spdlog::info("自检 detect 成功: targets={}", targets.size());
    } else {
```

改为：

```cpp
    std::vector<plane::model::DetectTarget> targets;
    if (detection_->detect(frame, targets, error)) {
        spdlog::info("自检 detect 成功: targets={}", targets.size());
        DetectionResult result;
        result.targets = targets;
        event_bus_.publish(events::kDetection, result);
    } else {
```

- [ ] Step 2: track 成功后发布事件

将：

```cpp
    plane::model::DetectTarget tracked;
    if (tracking_->track(frame, tracked, error)) {
        spdlog::info("自检 track 成功: type={}", tracked.type);
    } else {
```

改为：

```cpp
    plane::model::DetectTarget tracked;
    if (tracking_->track(frame, tracked, error)) {
        spdlog::info("自检 track 成功: type={}", tracked.type);
        TrackingResult result;
        result.target = tracked;
        result.lost = false;
        event_bus_.publish(events::kTracking, result);
    } else {
```

- [ ] Step 3: guide 成功后发布事件

将：

```cpp
    if (guidance_->guide(tracked, state_store_.own_state(), velocity, error)) {
        spdlog::info("自检 guide 成功: vn={} ve={} vd={}", velocity.north_m_s, velocity.east_m_s,
                     velocity.down_m_s);
    } else {
```

改为：

```cpp
    if (guidance_->guide(tracked, state_store_.own_state(), velocity, error)) {
        spdlog::info("自检 guide 成功: vn={} ve={} vd={}", velocity.north_m_s, velocity.east_m_s,
                     velocity.down_m_s);
        GuidanceResult result;
        result.velocity = velocity;
        result.valid = true;
        event_bus_.publish(events::kGuidance, result);
    } else {
```

- [ ] Step 4: 构建、格式与提交

```bash
cmake --build build -j
clang-format --dry-run --Werror src/perception/perception.cpp
git add src/perception/perception.cpp
git commit -m "feat(m7): 感知自检发布结果事件"
```

---

## Task 2：TargetStore

**Files**

- Create: `src/target/target_store_api.h`
- Create: `src/target/target_store.h`
- Create: `src/target/target_store.cpp`
- Modify: `CMakeLists.txt`

- [ ] Step 1: 写 `src/target/target_store_api.h`

```cpp
#pragma once

#include <string>
#include <vector>

#include "message.h"

namespace plane::target {

/// 目标统一查询接口（ARCHITECTURE §16/§22）。
class TargetStoreApi {
   public:
    virtual ~TargetStoreApi() = default;

    virtual std::vector<plane::model::DetectTarget> vision_targets() const = 0;
    virtual bool tracked_target(plane::model::DetectTarget& out) const = 0;

    virtual std::vector<plane::model::GcsTarget> gcs_targets() const = 0;
    virtual bool gcs_target(const std::string& track_id, plane::model::GcsTarget& out) const = 0;
    virtual std::vector<plane::model::GcsTarget> gcs_target_history(const std::string& track_id) const = 0;
};

}  // namespace plane::target
```

- [ ] Step 2: 写 `src/target/target_store.h`

```cpp
#pragma once

#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/event_bus.h"
#include "core/module.h"
#include "perception/perception_api.h"
#include "target/target_store_api.h"

namespace plane::target {

/// 目标管理：机载感知目标（最新）与 GCS 目标（最新 + 历史）（ARCHITECTURE §16）。
class TargetStore final : public plane::core::IModule, public TargetStoreApi {
   public:
    explicit TargetStore(plane::core::EventBus& event_bus);
    ~TargetStore() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    std::vector<plane::model::DetectTarget> vision_targets() const override;
    bool tracked_target(plane::model::DetectTarget& out) const override;
    std::vector<plane::model::GcsTarget> gcs_targets() const override;
    bool gcs_target(const std::string& track_id, plane::model::GcsTarget& out) const override;
    std::vector<plane::model::GcsTarget> gcs_target_history(const std::string& track_id) const override;

   private:
    void on_gcs_target(const plane::model::GcsTarget& target);
    void on_detection(const plane::perception::DetectionResult& result);
    void on_tracking(const plane::perception::TrackingResult& result);

    plane::core::EventBus& event_bus_;
    mutable std::mutex mutex_;
    std::vector<plane::model::DetectTarget> vision_targets_;
    std::optional<plane::model::DetectTarget> tracked_target_;
    std::unordered_map<std::string, plane::model::GcsTarget> gcs_latest_;
    std::unordered_map<std::string, std::deque<plane::model::GcsTarget>> gcs_history_;
};

}  // namespace plane::target
```

- [ ] Step 3: 写 `src/target/target_store.cpp`

```cpp
#include "target/target_store.h"

#include <cstddef>
#include <string>
#include <utility>

#include <spdlog/spdlog.h>

#include "communication/communication_events.h"
#include "perception/perception_events.h"

namespace plane::target {
namespace {

constexpr std::size_t kGcsTargetHistoryDepth = 20;

}  // namespace

TargetStore::TargetStore(plane::core::EventBus& event_bus) : event_bus_(event_bus) {}

TargetStore::~TargetStore() {
    stop();
}

std::string TargetStore::name() const {
    return "Target Management";
}

bool TargetStore::init() {
    return true;
}

bool TargetStore::start() {
    event_bus_.subscribe<plane::model::GcsTarget>(
        plane::communication::events::kGcsTargetReceived,
        [this](const plane::model::GcsTarget& target) { on_gcs_target(target); });
    event_bus_.subscribe<plane::perception::DetectionResult>(
        plane::perception::events::kDetection,
        [this](const plane::perception::DetectionResult& result) { on_detection(result); });
    event_bus_.subscribe<plane::perception::TrackingResult>(
        plane::perception::events::kTracking,
        [this](const plane::perception::TrackingResult& result) { on_tracking(result); });
    return true;
}

void TargetStore::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    vision_targets_.clear();
    tracked_target_.reset();
    gcs_latest_.clear();
    gcs_history_.clear();
}

void TargetStore::release() {}

std::vector<plane::model::DetectTarget> TargetStore::vision_targets() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return vision_targets_;
}

bool TargetStore::tracked_target(plane::model::DetectTarget& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!tracked_target_.has_value()) {
        return false;
    }
    out = *tracked_target_;
    return true;
}

std::vector<plane::model::GcsTarget> TargetStore::gcs_targets() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<plane::model::GcsTarget> result;
    result.reserve(gcs_latest_.size());
    for (const auto& item : gcs_latest_) {
        result.push_back(item.second);
    }
    return result;
}

bool TargetStore::gcs_target(const std::string& track_id, plane::model::GcsTarget& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = gcs_latest_.find(track_id);
    if (it == gcs_latest_.end()) {
        return false;
    }
    out = it->second;
    return true;
}

std::vector<plane::model::GcsTarget> TargetStore::gcs_target_history(
    const std::string& track_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = gcs_history_.find(track_id);
    if (it == gcs_history_.end()) {
        return {};
    }
    return {it->second.begin(), it->second.end()};
}

void TargetStore::on_gcs_target(const plane::model::GcsTarget& target) {
    std::size_t history_size = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        gcs_latest_[target.trackId] = target;
        auto& history = gcs_history_[target.trackId];
        history.push_back(target);
        if (history.size() > kGcsTargetHistoryDepth) {
            history.pop_front();
        }
        history_size = history.size();
    }
    spdlog::info("GCS 目标更新: trackId={}, 历史={}", target.trackId, history_size);
}

void TargetStore::on_detection(const plane::perception::DetectionResult& result) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        vision_targets_ = result.targets;
    }
    spdlog::info("视觉目标更新: {} 个", result.targets.size());
}

void TargetStore::on_tracking(const plane::perception::TrackingResult& result) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        tracked_target_ = result.target;
    }
    spdlog::info("跟踪目标更新: type={}, lost={}", result.target.type, result.lost ? 1 : 0);
}

}  // namespace plane::target
```

- [ ] Step 4: `CMakeLists.txt` 的 `add_executable(onboard ...)` 文件列表追加

```cmake
  src/target/target_store.cpp
```

- [ ] Step 5: 构建与格式

```bash
cmake --build build -j
clang-format --dry-run --Werror $(git ls-files src/target)
```

Expected: 构建通过且无 warning；格式通过（差异先 `-i` 再构建）。

- [ ] Step 6: 提交

```bash
git add src/target CMakeLists.txt
git commit -m "feat(m7): 增加 TargetStore 目标管理"
```

---

## Task 3：gcs_sim GCS 目标发送选项

**Files**

- Modify: `tools/gcs_sim/main.cpp`

- [ ] Step 1: `Options` 增加

```cpp
    std::string gcs_target_id;
    int gcs_target_repeat = 1;
```

- [ ] Step 2: 解析追加

```cpp
        } else if (arg == "--send-gcs-target" && i + 1 < argc) {
            options.gcs_target_id = argv[++i];
        } else if (arg == "--gcs-target-repeat" && i + 1 < argc) {
            options.gcs_target_repeat = std::stoi(argv[++i]);
```

- [ ] Step 3: 发送逻辑（放在 Command 发送之后、友机状态/ listen 之前）

```cpp
    if (!options.gcs_target_id.empty()) {
        for (int i = 0; i < options.gcs_target_repeat; ++i) {
            const std::string payload =
                nlohmann::json{
                    {"trackId", options.gcs_target_id},
                    {"geoPos",
                     {{"latitude", 30.0 + 0.0001 * i},
                      {"longitude", 120.0},
                      {"height", 50.0F},
                      {"altitude", 60.0F}}},
                    {"velocity_m_s", 10.0F},
                    {"direction_deg", 90.0F}}
                    .dump();
            const std::string message_id =
                options.gcs_target_id + "-" + std::to_string(++sequence);
            session.put(zenoh::KeyExpr("sendToPlane/" + options.plane_id),
                        zenoh::Bytes(build_wire(options.sender_id, message_id, "GcsTarget", payload)));
        }
        spdlog::info("已发送 GcsTarget: trackId={}, count={}", options.gcs_target_id,
                     options.gcs_target_repeat);
    }
```

- [ ] Step 4: 构建、格式与提交

```bash
cmake --build build -j
clang-format --dry-run --Werror tools/gcs_sim/main.cpp
git add tools/gcs_sim/main.cpp
git commit -m "feat(m7): gcs_sim 支持发送 GCS 目标"
```

---

## Task 4：main 接线

**Files**

- Modify: `src/main.cpp`

- [ ] Step 1: include 追加

```cpp
#include "target/target_store.h"
```

- [ ] Step 2: 创建并注册（在 Perception 之后、TaskExecutionEngine 之前）

```cpp
    auto target_store = std::make_unique<plane::target::TargetStore>(event_bus);
```

注册顺序追加：

```cpp
    runtime.add_module(std::move(target_store));
```

占位 skip 列表增加 `"Target Management"`。

- [ ] Step 3: 构建、运行与格式

```bash
cmake --build build -j
timeout --preserve-status 2 ./build/onboard; echo "exit=$?"
clang-format --dry-run --Werror src/main.cpp
```

Expected: 构建通过；`Target Management` 为真实模块，exit=0。

- [ ] Step 4: 提交

```bash
git add src/main.cpp
git commit -m "feat(m7): 接线 TargetStore"
```

---

## Task 5：M7 部分验收

**Files**

- Modify: `docs/plans/m7-target-management-tasks.md`
- Modify: `docs/DEV_PLAN.md`

- [ ] Step 1: 干净构建

```bash
rm -rf build logs
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

- [ ] Step 2: 自检驱动视觉/跟踪目标更新

```bash
timeout --preserve-status 3 ./build/onboard --config /tmp/opencode/m6_selftest.conf 2>&1 | grep -E "视觉目标更新|跟踪目标更新"
```

Expected：`视觉目标更新: 0 个`、`跟踪目标更新: type=stub, lost=0`。

- [ ] Step 3: GCS 目标与历史 cap（终端 A/B）

终端 A：`./build/onboard`

终端 B：

```bash
./build/gcs_sim --no-task --no-command --send-gcs-target t1 --gcs-target-repeat 25 --listen-seconds 2
```

Expected：onboard 出现 25 条 `GCS 目标更新: trackId=t1`，最后一条 `历史=20`。

- [ ] Step 4: 规范与回归

```bash
clang-format --dry-run --Werror $(git ls-files src tools plugins)
grep -rn "std::cout\|printf(" src/ tools/ plugins/ || echo "no cout/printf"
```

- [ ] Step 5: 文档与收尾提交

- 勾选本文档并记录验收证据与偏差；
- 更新 `docs/DEV_PLAN.md` §11 索引：M7 → 已完成（部分验收）。

```bash
git add docs
git commit -m "docs(m7): M7 部分验收与文档更新"
```

---

## 附：待用户补充验证（M7 交付）

1. 有真实感知源（RTSP/MIPI）后：检测列表与跟踪目标随 `perception.*` 事件更新，`TargetStoreApi` 查询正确；
2. 多 trackId 的 GCS 目标并发更新与历史隔离。
