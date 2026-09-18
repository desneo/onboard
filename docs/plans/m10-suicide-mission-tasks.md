# M10 任务级计划（自杀式反无任务全链路）

> 设计依据：`m10-suicide-mission-design.md`（用户 2026-09-18 确认）。
> 执行位置：工程根 `onboard/`。
> 验证原则：每个 Task 结束后 x64 Release 构建通过；不写测试代码（SPEC §11）。
> 状态：已完成并部分验收（2026-09-18）；SITL/实流待验。

---

## Task 1：架构文档与 ADR 更新（用户已授权）

**Files**

- Modify: `docs/ARCHITECTURE.md`（§15/§22/§23/§30）
- Modify: `docs/DEV_PLAN.md`（§11 索引）
- Modify: `docs/plans/m10-suicide-mission-design.md`（状态行）

- [x] Step 1: `docs/ARCHITECTURE.md` §15.3，在「Detection / Tracking / Guidance 结果也通过 eventcpp 向相关模块发布。」之后追加

```text
导引阶段的飞控控制下发：Perception 在导引启用且跟踪有效时，将 Guidance Plugin 输出的
NedVelocity 通过 FlightControlApi::set_ned_velocity 同步直连下发并获取下发结果（ADR-018）。
启停仍由 Task Execution Engine 经 PerceptionApi 控制；导引停止或任务中止时调用 clear_ned_velocity。
```

- [x] Step 2: `docs/ARCHITECTURE.md` §22 表格中，在「异步控制请求」行之后追加一行

```markdown
| 下发飞控指令（含导引速度） | C++ 同步调用 FlightControl，必须获取下发结果 |
```

- [x] Step 3: `docs/ARCHITECTURE.md` §23，在「Task Scheduler 可以向 Flight Control 派发独立 Command。」之后追加

```text
Perception 仅在下发导引速度时直连 Flight Control（ADR-018）。

统一原则：业务模块下发飞控指令必须采用同步调用并获取结果，不得以「事件发布」代替直连下发。
```

- [x] Step 4: `docs/ARCHITECTURE.md` §30，在 ADR-017 末尾 `---` 之后、`# 31. 架构演进原则` 之前新增

```markdown
## ADR-018：业务模块下发飞控指令采用同步直连

### Context

导引阶段需要将 Guidance Plugin 输出的 NED 速度持续下发给飞控，并立即知道本次下发是否被飞控接受；
事件发布方式无法让下发方获得成功/失败结果。

### Decision

- 下发飞控指令统一采用 C++ 同步能力调用（`FlightControlApi`），调用方必须获取并处理 `bool + error` 结果；
- Perception 被允许在导引启用且跟踪有效时直连 FlightControl 下发 NED 速度（仅此一项能力），
  并在停止导引/任务中止时调用 `clear_ned_velocity()`；
- 启停与阶段控制仍由 Task Execution Engine（经 `PerceptionApi`）负责，Perception 不承担任务编排。

### Alternatives

- 保持仅事件发布（`perception.guidance`），由任务执行层消费后再下发；
- 新增独立桥接模块转发事件到飞控。

### Rationale

- 下发方需要拿到下发结果，才能对失败做出业务反应（日志、事件、任务失败判定），
  符合 SPEC §5.8「不能仅以指令已发送成功为准」的精神；
- 避免为单一下发通路引入额外模块；
- 阶段控制留在任务执行层，Perception 只承担「算法输出 → 下发」的窄职责。

### Consequences

- Perception 依赖 `FlightControlApi`（稳定能力接口），不是实现依赖；不产生循环依赖；
- 依赖方向图中新增 Perception → Flight Control（仅导引速度）；
- 后续任何模块下发飞控指令，均需同步直连并获得结果，不再新增事件式下发通路。
```

- [x] Step 5: `docs/DEV_PLAN.md` §11 索引表末尾追加两行

```markdown
| `plans/m10-suicide-mission-design.md` | M10 Change Design（自杀式反无任务全链路） | 已确认（2026-09-18），实施中 |
| `plans/m10-suicide-mission-tasks.md` | M10 任务级计划与执行记录 | 执行中 |
```

- [x] Step 6: `docs/plans/m10-suicide-mission-design.md` 状态行改为

```markdown
> 状态：已确认（2026-09-18）。任务级计划见 `m10-suicide-mission-tasks.md`。
```

- [x] Step 7: 提交

```bash
git add docs/ARCHITECTURE.md docs/DEV_PLAN.md docs/plans/m10-suicide-mission-design.md docs/plans/m10-suicide-mission-tasks.md
git commit -m "docs(m10): M10 设计确认与架构 ADR-018"
```

Expected: 文档变更仅限上述内容；未改动 SPEC/requirement。

---

## Task 2：suicide 配置项

**Files**

- Modify: `src/core/config.h`
- Modify: `src/core/config.cpp`
- Modify: `config/onboard.conf`

- [x] Step 1: `src/core/config.h`，在 `alert_voltage_threshold_v` 之后追加

```cpp
    double suicide_detection_start_distance_m = 200.0;
    double suicide_approach_speed_m_s = 10.0;
    double suicide_track_approach_speed_m_s = 5.0;
    double suicide_climb_speed_m_s = 2.0;
    std::string suicide_target_select_rule = "nearest_center";
    int suicide_guidance_start_stable_ms = 500;
    int suicide_track_lost_timeout_ms = 3000;
    int suicide_phase_timeout_ms = 300000;
    double suicide_impact_height_m = 2.0;
    double suicide_impact_vd_m_s = 3.0;
```

- [x] Step 2: `src/core/config.cpp`，在 `alert_voltage_threshold_v` 分支之后追加

```cpp
        } else if (key == "suicide_detection_start_distance_m") {
            out.suicide_detection_start_distance_m = std::stod(value);
        } else if (key == "suicide_approach_speed_m_s") {
            out.suicide_approach_speed_m_s = std::stod(value);
        } else if (key == "suicide_track_approach_speed_m_s") {
            out.suicide_track_approach_speed_m_s = std::stod(value);
        } else if (key == "suicide_climb_speed_m_s") {
            out.suicide_climb_speed_m_s = std::stod(value);
        } else if (key == "suicide_target_select_rule") {
            out.suicide_target_select_rule = value;
        } else if (key == "suicide_guidance_start_stable_ms") {
            out.suicide_guidance_start_stable_ms = std::stoi(value);
        } else if (key == "suicide_track_lost_timeout_ms") {
            out.suicide_track_lost_timeout_ms = std::stoi(value);
        } else if (key == "suicide_phase_timeout_ms") {
            out.suicide_phase_timeout_ms = std::stoi(value);
        } else if (key == "suicide_impact_height_m") {
            out.suicide_impact_height_m = std::stod(value);
        } else if (key == "suicide_impact_vd_m_s") {
            out.suicide_impact_vd_m_s = std::stod(value);
```

- [x] Step 3: `config/onboard.conf` 末尾追加

```text
# 自杀式反无任务（阶段阈值与失败判定，联调时按实际调整）
suicide_detection_start_distance_m = 200.0
suicide_approach_speed_m_s = 10.0
suicide_track_approach_speed_m_s = 5.0
suicide_climb_speed_m_s = 2.0
suicide_target_select_rule = nearest_center
suicide_guidance_start_stable_ms = 500
suicide_track_lost_timeout_ms = 3000
suicide_phase_timeout_ms = 300000
suicide_impact_height_m = 2.0
suicide_impact_vd_m_s = 3.0
```

- [x] Step 4: 构建并提交

```bash
cmake --build build -j
timeout --preserve-status 2 ./build/onboard; echo "exit=$?"
clang-format --dry-run --Werror src/core/config.h src/core/config.cpp
git add src/core/config.h src/core/config.cpp config/onboard.conf
git commit -m "feat(m10): 增加 suicide 任务配置项"
```

Expected: 构建通过；运行无「未知配置项」警告；exit=0。

---

## Task 3：Geographic Tool（`src/geo/`）

**Files**

- Create: `src/geo/geo_tool.h`
- Create: `src/geo/geo_tool.cpp`
- Modify: `CMakeLists.txt`

**Produces**（后续任务依赖的精确接口）

```cpp
namespace plane::geo {
class GeoCalculator {
   public:
    static double distance_m(const plane::model::GeoPoint& from, const plane::model::GeoPoint& to);
    static double bearing_deg(const plane::model::GeoPoint& from, const plane::model::GeoPoint& to);
};
}
```

- [x] Step 1: 写 `src/geo/geo_tool.h`

```cpp
#pragma once

#include "message.h"

namespace plane::geo {

/// 统一地理计算工具（ARCHITECTURE §21/ADR-012）；GeographicLib 类型不出本模块。
class GeoCalculator {
   public:
    /// 两点间大地线距离（米）。
    static double distance_m(const plane::model::GeoPoint& from, const plane::model::GeoPoint& to);

    /// from 到 to 的方位角（度，北 0，顺时针，0–360）。
    static double bearing_deg(const plane::model::GeoPoint& from, const plane::model::GeoPoint& to);
};

}  // namespace plane::geo
```

- [x] Step 2: 写 `src/geo/geo_tool.cpp`

```cpp
#include "geo/geo_tool.h"

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/Geodesic.hpp>

namespace plane::geo {
namespace {

const GeographicLib::Geodesic& geodesic() {
    static const GeographicLib::Geodesic instance(GeographicLib::Constants::WGS84_a(),
                                                  GeographicLib::Constants::WGS84_f());
    return instance;
}

}  // namespace

double GeoCalculator::distance_m(const plane::model::GeoPoint& from, const plane::model::GeoPoint& to) {
    double distance = 0.0;
    double azimuth = 0.0;
    double reverse_azimuth = 0.0;
    geodesic().Inverse(from.latitude, from.longitude, to.latitude, to.longitude, distance, azimuth, reverse_azimuth);
    return distance;
}

double GeoCalculator::bearing_deg(const plane::model::GeoPoint& from, const plane::model::GeoPoint& to) {
    double distance = 0.0;
    double azimuth = 0.0;
    double reverse_azimuth = 0.0;
    geodesic().Inverse(from.latitude, from.longitude, to.latitude, to.longitude, distance, azimuth, reverse_azimuth);
    double bearing = azimuth;
    if (bearing < 0.0) {
        bearing += 360.0;
    }
    return bearing;
}

}  // namespace plane::geo
```

- [x] Step 3: `CMakeLists.txt` 的 `add_executable(onboard ...)` 列表中，在 `src/flight_control/flight_control_module.cpp` 之后追加

```cmake
  src/geo/geo_tool.cpp
```

- [x] Step 4: 构建并提交

```bash
cmake --build build -j
clang-format --dry-run --Werror src/geo/geo_tool.h src/geo/geo_tool.cpp
git add src/geo/geo_tool.h src/geo/geo_tool.cpp CMakeLists.txt
git commit -m "feat(m10): 增加 Geographic Tool"
```

Expected: 构建通过（`GeographicLib::GeographicLib` 由 `plane::deps` 传递）。

---

## Task 4：TargetStore 跟踪状态扩展

**Files**

- Modify: `src/target/target_store_api.h`
- Modify: `src/target/target_store.h`
- Modify: `src/target/target_store.cpp`

**Produces**

```cpp
struct TrackingState {
    plane::model::DetectTarget target;
    bool valid = false;
    bool lost = false;
};

virtual TrackingState tracking_state() const = 0;  // TargetStoreApi
```

- [x] Step 1: `src/target/target_store_api.h`，在 `class TargetStoreApi` 之前追加

```cpp
/// 跟踪目标状态快照（M10）。
struct TrackingState {
    plane::model::DetectTarget target;  ///< 最后已知目标（valid=false 时无意义）
    bool valid = false;                 ///< 是否收到过跟踪结果
    bool lost = false;                  ///< 最新一次跟踪结果是否丢失
};
```

- [x] Step 2: `src/target/target_store_api.h`，在 `tracked_target` 声明之后追加

```cpp
    virtual TrackingState tracking_state() const = 0;
```

- [x] Step 3: `src/target/target_store.h`，在 `tracked_target` 声明之后追加

```cpp
    TrackingState tracking_state() const override;
```

并在成员区追加

```cpp
    bool tracking_valid_ = false;
    bool tracking_lost_ = false;
```

- [x] Step 4: `src/target/target_store.cpp`，在 `tracked_target()` 实现之后追加重载

```cpp
TrackingState TargetStore::tracking_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    TrackingState state;
    if (tracked_target_.has_value()) {
        state.target = *tracked_target_;
        state.valid = tracking_valid_;
        state.lost = tracking_lost_;
    }
    return state;
}
```

- [x] Step 5: `src/target/target_store.cpp` 的 `stop()` 中，清空目标后追加状态复位

```cpp
    tracking_valid_ = false;
    tracking_lost_ = false;
```

- [x] Step 6: `src/target/target_store.cpp` 的 `on_tracking()` 中，更新 `tracked_target_` 后追加

```cpp
        tracking_valid_ = true;
        tracking_lost_ = result.lost;
```

- [x] Step 7: 构建并提交

```bash
cmake --build build -j
clang-format --dry-run --Werror src/target/target_store_api.h src/target/target_store.h src/target/target_store.cpp
git add src/target/target_store_api.h src/target/target_store.h src/target/target_store.cpp
git commit -m "feat(m10): TargetStore 暴露跟踪状态"
```

Expected: 构建通过；M7 回归（GCS 目标日志）不受影响。

---

## Task 5：Perception 直连 FlightControl

**Files**

- Modify: `src/perception/perception_api.h`
- Modify: `src/perception/perception.h`
- Modify: `src/perception/perception.cpp`
- Modify: `src/main.cpp`

**Produces**

```cpp
struct GuidanceResult {
    plane::model::NedVelocity velocity;
    bool valid = false;
    bool applied = false;  ///< 是否已成功下发飞控
};

Perception(EventBus&, const AppConfig&, PluginManagerApi&, StateStoreApi&, FlightControlApi&, VideoStreamingApi&);
```

- [x] Step 1: `src/perception/perception_api.h` 的 `GuidanceResult` 追加字段

```cpp
struct GuidanceResult {
    plane::model::NedVelocity velocity;
    bool valid = false;
    bool applied = false;  ///< 是否已成功下发飞控（M10 直连）
};
```

- [x] Step 2: `src/perception/perception.h`：增加 include 与构造参数/成员

在 `#include "core/system_param_api.h"` 之后（按字母序放合适位置）追加

```cpp
#include "flight_control/flight_control_api.h"
```

构造函数改为

```cpp
    Perception(plane::core::EventBus& event_bus, const plane::core::AppConfig& config, PluginManagerApi& plugin_manager,
               plane::state::StateStoreApi& state_store, plane::flight_control::FlightControlApi& flight_control,
               plane::video::VideoStreamingApi& video_streaming);
```

成员区在 `plane::state::StateStoreApi& state_store_;` 之后追加

```cpp
    plane::flight_control::FlightControlApi& flight_control_;
```

- [x] Step 3: `src/perception/perception.cpp` 构造函数同步修改

```cpp
Perception::Perception(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                       PluginManagerApi& plugin_manager, plane::state::StateStoreApi& state_store,
                       plane::flight_control::FlightControlApi& flight_control,
                       plane::video::VideoStreamingApi& video_streaming)
    : event_bus_(event_bus),
      config_(config),
      plugin_manager_(plugin_manager),
      state_store_(state_store),
      flight_control_(flight_control),
      video_streaming_(video_streaming) {}
```

- [x] Step 4: `src/perception/perception.cpp` 的 `stop_guidance()` 改为

```cpp
void Perception::stop_guidance() {
    guidance_enabled_ = false;
    flight_control_.clear_ned_velocity();
}
```

- [x] Step 5: `src/perception/perception.cpp` 的 `stop()` 中，在重置插件之后追加

```cpp
    flight_control_.clear_ned_velocity();
```

- [x] Step 6: `src/perception/perception.cpp` 的 `process_loop()` 导引分支改为

```cpp
        if (guidance_enabled_ && tracking_enabled_) {
            std::string error;
            if (ensure_guidance(error)) {
                plane::model::DetectTarget target;
                {
                    std::lock_guard<std::mutex> lock(target_mutex_);
                    target = tracking_target_;
                }
                plane::model::NedVelocity velocity;
                if (guidance_->guide(target, state_store_.own_state(), velocity, error)) {
                    GuidanceResult result;
                    result.velocity = velocity;
                    result.valid = true;
                    std::string dispatch_error;
                    if (!flight_control_.set_ned_velocity(velocity, dispatch_error)) {
                        report_error("导引速度下发失败: " + dispatch_error);
                    } else {
                        result.applied = true;
                    }
                    event_bus_.publish(events::kGuidance, result);
                } else {
                    GuidanceResult result;
                    result.valid = false;
                    event_bus_.publish(events::kGuidance, result);
                }
            }
        }
```

- [x] Step 7: `src/main.cpp` 的 Perception 构造改为

```cpp
    auto perception = std::make_unique<plane::perception::Perception>(event_bus, config, *plugin_management,
                                                                      *state_store, *flight_control, *video_streaming);
```

- [x] Step 8: 构建并提交

```bash
cmake --build build -j
timeout --preserve-status 2 ./build/onboard; echo "exit=$?"
clang-format --dry-run --Werror src/perception/perception_api.h src/perception/perception.h src/perception/perception.cpp src/main.cpp
git add src/perception/perception_api.h src/perception/perception.h src/perception/perception.cpp src/main.cpp
git commit -m "feat(m10): Perception 直连飞控下发导引速度"
```

Expected: 构建通过；运行正常退出；M6 自检路径不受影响。

---

## Task 6：节点基类超时覆写 + suicide 上下文 + ApproachController

**Files**

- Modify: `src/task/nodes/flight_action_node.h`
- Create: `src/task/suicide_task_context.h`
- Create: `src/task/nodes/approach_controller.h`
- Create: `src/task/nodes/approach_controller.cpp`
- Modify: `CMakeLists.txt`

**Produces**

```cpp
// FlightActionNode 追加
virtual std::int64_t node_timeout_ms() const { return app_config_->bt_node_timeout_ms; }

// plane::task
struct SuicideTaskContext { std::string targetTrackId; };

class ApproachController {
   public:
    ApproachController(FlightControlApi*, StateStoreApi*, TargetStoreApi*, const AppConfig*);
    bool resolve_target(const std::string& track_id, plane::model::GcsTarget& out) const;
    double horizontal_distance_m(const plane::model::GcsTarget& target) const;
    bool approach(const plane::model::GcsTarget& target, double speed_m_s, std::string& error);
};
```

- [x] Step 1: `src/task/nodes/flight_action_node.h` 的 `onRunning()` 改为

```cpp
    BT::NodeStatus onRunning() override {
        if (now_ms() - start_ms_ > node_timeout_ms()) {
            spdlog::error("节点 {} 超时 ({} ms)", name(), node_timeout_ms());
            return BT::NodeStatus::FAILURE;
        }
        return check();
    }
```

并在 `onHalted()` 之后追加

```cpp
    /// 节点超时；默认取全局配置，长阶段节点可覆写（M10）。
    virtual std::int64_t node_timeout_ms() const { return app_config_->bt_node_timeout_ms; }
```

- [x] Step 2: 写 `src/task/suicide_task_context.h`

```cpp
#pragma once

#include <string>

namespace plane::task {

/// suicide 任务参数上下文（引擎持有，节点只读）。
struct SuicideTaskContext {
    std::string targetTrackId;  ///< TaskInfo.targetTrackId，可空
};

}  // namespace plane::task
```

- [x] Step 3: 写 `src/task/nodes/approach_controller.h`

```cpp
#pragma once

#include <string>

#include "core/config.h"
#include "flight_control/flight_control_api.h"
#include "state/state_store_api.h"
#include "target/target_store_api.h"

namespace plane::task {

/// suicide 各阶段共享的逼近控制：解析 GCS 目标、计算距离、下发逼近速度。
class ApproachController {
   public:
    ApproachController(plane::flight_control::FlightControlApi* flight_control,
                       plane::state::StateStoreApi* state_store, plane::target::TargetStoreApi* target_store,
                       const plane::core::AppConfig* app_config);

    /// 解析目标：track_id 非空按 id 查询；为空且仅有一个 GCS 目标时使用该目标。
    bool resolve_target(const std::string& track_id, plane::model::GcsTarget& out) const;

    /// 到目标的地面水平距离（米）。
    double horizontal_distance_m(const plane::model::GcsTarget& target) const;

    /// 朝目标下发 NED 速度（水平 speed，垂直按高度差限幅，机头指向目标）。
    bool approach(const plane::model::GcsTarget& target, double speed_m_s, std::string& error);

   private:
    plane::flight_control::FlightControlApi* flight_control_;
    plane::state::StateStoreApi* state_store_;
    plane::target::TargetStoreApi* target_store_;
    const plane::core::AppConfig* app_config_;
};

}  // namespace plane::task
```

- [x] Step 4: 写 `src/task/nodes/approach_controller.cpp`

```cpp
#include "task/nodes/approach_controller.h"

#include <algorithm>
#include <cmath>

#include "geo/geo_tool.h"

namespace plane::task {
namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

ApproachController::ApproachController(plane::flight_control::FlightControlApi* flight_control,
                                       plane::state::StateStoreApi* state_store,
                                       plane::target::TargetStoreApi* target_store,
                                       const plane::core::AppConfig* app_config)
    : flight_control_(flight_control), state_store_(state_store), target_store_(target_store),
      app_config_(app_config) {}

bool ApproachController::resolve_target(const std::string& track_id, plane::model::GcsTarget& out) const {
    if (!track_id.empty()) {
        return target_store_->gcs_target(track_id, out);
    }
    const auto targets = target_store_->gcs_targets();
    if (targets.size() != 1) {
        return false;
    }
    out = targets.front();
    return true;
}

double ApproachController::horizontal_distance_m(const plane::model::GcsTarget& target) const {
    const auto own = state_store_->own_state();
    return plane::geo::GeoCalculator::distance_m(own.geoPos, target.geoPos);
}

bool ApproachController::approach(const plane::model::GcsTarget& target, double speed_m_s, std::string& error) {
    const auto own = state_store_->own_state();
    const double bearing_deg = plane::geo::GeoCalculator::bearing_deg(own.geoPos, target.geoPos);
    const double bearing_rad = bearing_deg * kPi / 180.0;

    plane::model::NedVelocity velocity;
    velocity.north_m_s = speed_m_s * std::cos(bearing_rad);
    velocity.east_m_s = speed_m_s * std::sin(bearing_rad);
    if (target.geoPos.height > 0.0F) {
        const double diff = static_cast<double>(target.geoPos.height) - own.geoPos.height;
        if (std::fabs(diff) <= 1.0) {
            velocity.down_m_s = 0.0;
        } else {
            velocity.down_m_s =
                std::max(-app_config_->suicide_climb_speed_m_s, std::min(app_config_->suicide_climb_speed_m_s, -diff));
        }
    }
    velocity.yaw_deg = bearing_deg;
    velocity.yaw_rate_deg_s = 0.0;
    return flight_control_->set_ned_velocity(velocity, error);
}

}  // namespace plane::task
```

- [x] Step 5: `CMakeLists.txt` 的 `add_executable(onboard ...)` 列表中，在 `src/task/nodes/stop_motor_node.cpp` 之后追加

```cmake
  src/task/nodes/approach_controller.cpp
```

- [x] Step 6: 构建并提交

```bash
cmake --build build -j
clang-format --dry-run --Werror src/task/nodes/flight_action_node.h src/task/suicide_task_context.h src/task/nodes/approach_controller.h src/task/nodes/approach_controller.cpp
git add src/task/nodes/flight_action_node.h src/task/suicide_task_context.h src/task/nodes/approach_controller.h src/task/nodes/approach_controller.cpp CMakeLists.txt
git commit -m "feat(m10): 增加 suicide 节点基类超时与逼近控制"
```

Expected: 构建通过；M5 节点行为不变（默认超时未变）。

---

## Task 7：MidcourseNode 与 DetectNode

**Files**

- Create: `src/task/nodes/midcourse_node.h`
- Create: `src/task/nodes/midcourse_node.cpp`
- Create: `src/task/nodes/detect_node.h`
- Create: `src/task/nodes/detect_node.cpp`
- Modify: `CMakeLists.txt`

**Interfaces**

- Consumes: `ApproachController`（Task 6）、`SuicideTaskContext`（Task 6）、`PerceptionApi`（M6）、`TargetStoreApi::vision_targets/gcs_*`（M7）
- Produces: 节点类型名 `"Midcourse"`、`"Detect"`（Task 9 建树使用）

- [x] Step 1: 写 `src/task/nodes/midcourse_node.h`

```cpp
#pragma once

#include "task/nodes/approach_controller.h"
#include "task/nodes/flight_action_node.h"
#include "task/suicide_task_context.h"

namespace plane::task {

/// 中段制导：Offboard 速度逼近最新 GCS 目标，进入检测距离即完成（SPEC §6-3）。
class MidcourseNode final : public FlightActionNode {
   public:
    MidcourseNode(const std::string& name, const BT::NodeConfig& config,
                  plane::flight_control::FlightControlApi* flight_control, plane::state::StateStoreApi* state_store,
                  plane::target::TargetStoreApi* target_store, const plane::core::AppConfig* app_config,
                  const SuicideTaskContext* context)
        : FlightActionNode(name, config, flight_control, state_store, app_config),
          controller_(flight_control, state_store, target_store, app_config), app_config_(app_config),
          context_(context) {}

    static BT::PortsList providedPorts() { return {}; }

   protected:
    bool dispatch(std::string& error) override;
    BT::NodeStatus check() override;
    void onHalted() override;
    std::int64_t node_timeout_ms() const override;

   private:
    ApproachController controller_;
    const plane::core::AppConfig* app_config_;
    const SuicideTaskContext* context_;
};

}  // namespace plane::task
```

- [x] Step 2: 写 `src/task/nodes/midcourse_node.cpp`

```cpp
#include "task/nodes/midcourse_node.h"

#include <spdlog/spdlog.h>

namespace plane::task {

bool MidcourseNode::dispatch(std::string& error) {
    plane::model::GcsTarget target;
    if (!controller_.resolve_target(context_->targetTrackId, target)) {
        error = context_->targetTrackId.empty() ? "无法确定 GCS 目标（targetTrackId 为空且 GCS 目标数不为 1）"
                                                : "GCS 目标不存在: " + context_->targetTrackId;
        return false;
    }
    spdlog::info("中段制导目标: trackId={}, lat={}, lon={}, height={}", target.trackId, target.geoPos.latitude,
                 target.geoPos.longitude, target.geoPos.height);
    return true;
}

BT::NodeStatus MidcourseNode::check() {
    plane::model::GcsTarget target;
    if (!controller_.resolve_target(context_->targetTrackId, target)) {
        spdlog::error("中段制导目标不可用");
        return BT::NodeStatus::FAILURE;
    }
    const double distance = controller_.horizontal_distance_m(target);
    if (distance <= app_config_->suicide_detection_start_distance_m) {
        spdlog::info("中段制导到达检测启动距离: {:.1f} m", distance);
        return BT::NodeStatus::SUCCESS;
    }
    std::string error;
    if (!controller_.approach(target, app_config_->suicide_approach_speed_m_s, error)) {
        spdlog::error("中段制导速度下发失败 (距离 {:.1f} m): {}", distance, error);
        return BT::NodeStatus::FAILURE;
    }
    spdlog::debug("中段制导: 距离 {:.1f} m", distance);
    return BT::NodeStatus::RUNNING;
}

void MidcourseNode::onHalted() {
    FlightActionNode::onHalted();
    flight_control_->clear_ned_velocity();
}

std::int64_t MidcourseNode::node_timeout_ms() const { return app_config_->suicide_phase_timeout_ms; }

}  // namespace plane::task
```

- [x] Step 3: 写 `src/task/nodes/detect_node.h`

```cpp
#pragma once

#include "perception/perception_api.h"
#include "task/nodes/approach_controller.h"
#include "task/nodes/flight_action_node.h"
#include "task/suicide_task_context.h"
#include "target/target_store_api.h"

namespace plane::task {

/// 目标检测：启动检测、继续逼近并按规则选定跟踪目标（SPEC §6-4）。
class DetectNode final : public FlightActionNode {
   public:
    DetectNode(const std::string& name, const BT::NodeConfig& config,
               plane::flight_control::FlightControlApi* flight_control, plane::state::StateStoreApi* state_store,
               plane::perception::PerceptionApi* perception, plane::target::TargetStoreApi* target_store,
               const plane::core::AppConfig* app_config, const SuicideTaskContext* context)
        : FlightActionNode(name, config, flight_control, state_store, app_config),
          controller_(flight_control, state_store, target_store, app_config), perception_(perception),
          target_store_(target_store), app_config_(app_config), context_(context) {}

    static BT::PortsList providedPorts() { return {}; }

   protected:
    bool dispatch(std::string& error) override;
    BT::NodeStatus check() override;
    void onHalted() override;
    std::int64_t node_timeout_ms() const override;

   private:
    ApproachController controller_;
    plane::perception::PerceptionApi* perception_;
    plane::target::TargetStoreApi* target_store_;
    const plane::core::AppConfig* app_config_;
    const SuicideTaskContext* context_;
};

}  // namespace plane::task
```

- [x] Step 4: 写 `src/task/nodes/detect_node.cpp`

```cpp
#include "task/nodes/detect_node.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <opencv2/core.hpp>
#include <vector>

namespace plane::task {
namespace {

double center_distance_squared(const plane::model::DetectTarget& target, double center_x, double center_y) {
    const double dx = target.bbox.x + target.bbox.width / 2.0 - center_x;
    const double dy = target.bbox.y + target.bbox.height / 2.0 - center_y;
    return dx * dx + dy * dy;
}

plane::model::DetectTarget select_target(const std::vector<plane::model::DetectTarget>& targets, const cv::Mat* frame,
                                         const std::string& rule) {
    if (rule == "first") {
        return targets.front();
    }
    if (rule == "highest_confidence") {
        return *std::max_element(targets.begin(), targets.end(),
                                 [](const auto& a, const auto& b) { return a.confidence < b.confidence; });
    }
    if (frame == nullptr || frame->empty()) {
        return targets.front();
    }
    const double center_x = frame->cols / 2.0;
    const double center_y = frame->rows / 2.0;
    return *std::min_element(targets.begin(), targets.end(), [&](const auto& a, const auto& b) {
        return center_distance_squared(a, center_x, center_y) < center_distance_squared(b, center_x, center_y);
    });
}

}  // namespace

bool DetectNode::dispatch(std::string& error) {
    if (!perception_->start_detection(error)) {
        return false;
    }
    spdlog::info("目标检测已启动");
    return true;
}

BT::NodeStatus DetectNode::check() {
    plane::model::GcsTarget gcs_target;
    if (controller_.resolve_target(context_->targetTrackId, gcs_target)) {
        std::string approach_error;
        if (!controller_.approach(gcs_target, app_config_->suicide_approach_speed_m_s, approach_error)) {
            spdlog::error("检测阶段逼近速度下发失败: {}", approach_error);
            return BT::NodeStatus::FAILURE;
        }
    }
    const auto targets = target_store_->vision_targets();
    if (targets.empty()) {
        return BT::NodeStatus::RUNNING;
    }
    cv::Mat frame;
    const bool has_frame = perception_->latest_frame(frame);
    const plane::model::DetectTarget selected =
        select_target(targets, has_frame ? &frame : nullptr, app_config_->suicide_target_select_rule);
    std::string error;
    if (!perception_->select_target(selected, error)) {
        spdlog::error("建立跟踪失败: {}", error);
        return BT::NodeStatus::FAILURE;
    }
    spdlog::info("已选定跟踪目标: type={}, confidence={:.2f}", selected.type, selected.confidence);
    return BT::NodeStatus::SUCCESS;
}

void DetectNode::onHalted() {
    FlightActionNode::onHalted();
    perception_->stop_detection();
}

std::int64_t DetectNode::node_timeout_ms() const { return app_config_->suicide_phase_timeout_ms; }

}  // namespace plane::task
```

- [x] Step 5: `CMakeLists.txt` 的 `add_executable(onboard ...)` 列表中追加

```cmake
  src/task/nodes/midcourse_node.cpp
  src/task/nodes/detect_node.cpp
```

- [x] Step 6: 构建并提交

```bash
cmake --build build -j
clang-format --dry-run --Werror src/task/nodes/midcourse_node.h src/task/nodes/midcourse_node.cpp src/task/nodes/detect_node.h src/task/nodes/detect_node.cpp
git add src/task/nodes/midcourse_node.h src/task/nodes/midcourse_node.cpp src/task/nodes/detect_node.h src/task/nodes/detect_node.cpp CMakeLists.txt
git commit -m "feat(m10): 增加中段制导与目标检测节点"
```

Expected: 构建通过（节点尚未注册使用，无行为变化）。

---

## Task 8：TrackNode 与 GuideNode

**Files**

- Create: `src/task/nodes/track_node.h`
- Create: `src/task/nodes/track_node.cpp`
- Create: `src/task/nodes/guide_node.h`
- Create: `src/task/nodes/guide_node.cpp`
- Modify: `CMakeLists.txt`

**Interfaces**

- Consumes: `TargetStoreApi::tracking_state()`（Task 4）、`PerceptionApi::start_guidance/stop_guidance`、`FlightControlApi::flight_status/link_state/clear_ned_velocity`
- Produces: 节点类型名 `"Track"`、`"Guide"`（Task 9 建树使用）

- [x] Step 1: 写 `src/task/nodes/track_node.h`

```cpp
#pragma once

#include <cstdint>

#include "perception/perception_api.h"
#include "task/nodes/approach_controller.h"
#include "task/nodes/flight_action_node.h"
#include "task/suicide_task_context.h"
#include "target/target_store_api.h"

namespace plane::task {

/// 单目标跟踪：丢失超限则失败，稳定达到时长后进入导引（SPEC §6-5）。
class TrackNode final : public FlightActionNode {
   public:
    TrackNode(const std::string& name, const BT::NodeConfig& config,
              plane::flight_control::FlightControlApi* flight_control, plane::state::StateStoreApi* state_store,
              plane::perception::PerceptionApi* perception, plane::target::TargetStoreApi* target_store,
              const plane::core::AppConfig* app_config, const SuicideTaskContext* context)
        : FlightActionNode(name, config, flight_control, state_store, app_config),
          controller_(flight_control, state_store, target_store, app_config), perception_(perception),
          target_store_(target_store), app_config_(app_config), context_(context) {}

    static BT::PortsList providedPorts() { return {}; }

   protected:
    bool dispatch(std::string& error) override;
    BT::NodeStatus check() override;
    std::int64_t node_timeout_ms() const override;

   private:
    ApproachController controller_;
    plane::perception::PerceptionApi* perception_;
    plane::target::TargetStoreApi* target_store_;
    const plane::core::AppConfig* app_config_;
    const SuicideTaskContext* context_;
    std::int64_t lost_since_ms_ = 0;
    std::int64_t stable_since_ms_ = 0;
};

}  // namespace plane::task
```

- [x] Step 2: 写 `src/task/nodes/track_node.cpp`

```cpp
#include "task/nodes/track_node.h"

#include <spdlog/spdlog.h>

#include <chrono>

namespace plane::task {
namespace {

std::int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace

bool TrackNode::dispatch(std::string&) {
    spdlog::info("单目标跟踪监视开始");
    return true;
}

BT::NodeStatus TrackNode::check() {
    plane::model::GcsTarget gcs_target;
    if (controller_.resolve_target(context_->targetTrackId, gcs_target)) {
        std::string error;
        if (!controller_.approach(gcs_target, app_config_->suicide_track_approach_speed_m_s, error)) {
            spdlog::error("跟踪阶段逼近速度下发失败: {}", error);
            return BT::NodeStatus::FAILURE;
        }
    }
    const plane::target::TrackingState state = target_store_->tracking_state();
    const std::int64_t now = now_ms();
    if (!state.valid) {
        if (lost_since_ms_ == 0) {
            lost_since_ms_ = now;
        }
        if (now - lost_since_ms_ >= app_config_->suicide_track_lost_timeout_ms) {
            spdlog::error("跟踪未建立且超时");
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }
    if (state.lost) {
        stable_since_ms_ = 0;
        if (lost_since_ms_ == 0) {
            lost_since_ms_ = now;
            spdlog::warn("跟踪目标丢失，等待恢复");
        }
        if (now - lost_since_ms_ >= app_config_->suicide_track_lost_timeout_ms) {
            spdlog::error("跟踪丢失超过 {} ms，任务失败", app_config_->suicide_track_lost_timeout_ms);
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }
    lost_since_ms_ = 0;
    if (stable_since_ms_ == 0) {
        stable_since_ms_ = now;
    }
    if (now - stable_since_ms_ >= app_config_->suicide_guidance_start_stable_ms) {
        spdlog::info("跟踪稳定，进入导引阶段");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

std::int64_t TrackNode::node_timeout_ms() const { return app_config_->suicide_phase_timeout_ms; }

}  // namespace plane::task
```

- [x] Step 3: 写 `src/task/nodes/guide_node.h`

```cpp
#pragma once

#include <cstdint>

#include "perception/perception_api.h"
#include "task/nodes/flight_action_node.h"
#include "target/target_store_api.h"

namespace plane::task {

/// 导引撞击：启动导引算法，按遥测异常判定撞击完成（SPEC §6-6）。
class GuideNode final : public FlightActionNode {
   public:
    GuideNode(const std::string& name, const BT::NodeConfig& config,
              plane::flight_control::FlightControlApi* flight_control, plane::state::StateStoreApi* state_store,
              plane::perception::PerceptionApi* perception, plane::target::TargetStoreApi* target_store,
              const plane::core::AppConfig* app_config)
        : FlightActionNode(name, config, flight_control, state_store, app_config), perception_(perception),
          target_store_(target_store), app_config_(app_config) {}

    static BT::PortsList providedPorts() { return {}; }

   protected:
    bool dispatch(std::string& error) override;
    BT::NodeStatus check() override;
    void onHalted() override;
    std::int64_t node_timeout_ms() const override;

   private:
    plane::perception::PerceptionApi* perception_;
    plane::target::TargetStoreApi* target_store_;
    const plane::core::AppConfig* app_config_;
    std::int64_t lost_since_ms_ = 0;
};

}  // namespace plane::task
```

- [x] Step 4: 写 `src/task/nodes/guide_node.cpp`

```cpp
#include "task/nodes/guide_node.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <string>

namespace plane::task {
namespace {

std::int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace

bool GuideNode::dispatch(std::string& error) {
    if (!perception_->start_guidance(error)) {
        return false;
    }
    spdlog::info("图像导引已启动");
    return true;
}

BT::NodeStatus GuideNode::check() {
    const std::string link = flight_control_->link_state();
    if (link != "connected") {
        spdlog::error("导引阶段飞控连接异常: {}", link);
        return BT::NodeStatus::FAILURE;
    }
    const auto status = flight_control_->flight_status();
    const auto own = state_store_->own_state();
    if (!status.in_air || !status.armed) {
        spdlog::info("检测到落地/碰撞，判定撞击完成");
        return BT::NodeStatus::SUCCESS;
    }
    if (own.geoPos.height <= static_cast<float>(app_config_->suicide_impact_height_m)) {
        spdlog::info("检测到高度异常 ({} m)，判定撞击完成", own.geoPos.height);
        return BT::NodeStatus::SUCCESS;
    }
    if (own.vd_m_s >= static_cast<float>(app_config_->suicide_impact_vd_m_s)) {
        spdlog::info("检测到下坠速度异常 ({} m/s)，判定撞击完成", own.vd_m_s);
        return BT::NodeStatus::SUCCESS;
    }
    const plane::target::TrackingState tracking = target_store_->tracking_state();
    const std::int64_t now = now_ms();
    if (!tracking.valid || tracking.lost) {
        if (lost_since_ms_ == 0) {
            lost_since_ms_ = now;
            spdlog::warn("导引阶段跟踪丢失，等待恢复");
        }
        if (now - lost_since_ms_ >= app_config_->suicide_track_lost_timeout_ms) {
            spdlog::error("导引阶段跟踪丢失超过 {} ms，任务失败", app_config_->suicide_track_lost_timeout_ms);
            return BT::NodeStatus::FAILURE;
        }
    } else {
        lost_since_ms_ = 0;
    }
    return BT::NodeStatus::RUNNING;
}

void GuideNode::onHalted() {
    FlightActionNode::onHalted();
    perception_->stop_guidance();
}

std::int64_t GuideNode::node_timeout_ms() const { return app_config_->suicide_phase_timeout_ms; }

}  // namespace plane::task
```

- [x] Step 5: `CMakeLists.txt` 的 `add_executable(onboard ...)` 列表中追加

```cmake
  src/task/nodes/track_node.cpp
  src/task/nodes/guide_node.cpp
```

- [x] Step 6: 构建并提交

```bash
cmake --build build -j
clang-format --dry-run --Werror src/task/nodes/track_node.h src/task/nodes/track_node.cpp src/task/nodes/guide_node.h src/task/nodes/guide_node.cpp
git add src/task/nodes/track_node.h src/task/nodes/track_node.cpp src/task/nodes/guide_node.h src/task/nodes/guide_node.cpp CMakeLists.txt
git commit -m "feat(m10): 增加跟踪与导引节点"
```

Expected: 构建通过。

---

## Task 9：引擎注册 suicide 树与终结清理

**Files**

- Modify: `src/task/task_execution_engine.h`
- Modify: `src/task/task_execution_engine.cpp`
- Modify: `src/main.cpp`

**Interfaces**

- Consumes: Task 5 `PerceptionApi&`、Task 4 `TargetStoreApi&`、Task 6-8 节点类型
- Produces: `TaskCode::AntiUav`（wire `"anti-uav"`）可执行；行为树名 `anti-uav`

- [x] Step 1: `src/task/task_execution_engine.h` 增加 include

```cpp
#include "perception/perception_api.h"
#include "target/target_store_api.h"
#include "task/suicide_task_context.h"
```

构造函数改为

```cpp
    TaskExecutionEngine(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                        plane::flight_control::FlightControlApi& flight_control,
                        plane::state::StateStoreApi& state_store, plane::core::SystemParamApi& system_param,
                        plane::perception::PerceptionApi& perception, plane::target::TargetStoreApi& target_store);
```

私有区追加

```cpp
    BT::Tree build_anti_uav();
    void cleanup_control();
```

成员区追加

```cpp
    plane::perception::PerceptionApi& perception_;
    plane::target::TargetStoreApi& target_store_;
    std::string current_tree_name_;
    SuicideTaskContext suicide_context_;
```

- [x] Step 2: `src/task/task_execution_engine.cpp` 构造函数改为

```cpp
TaskExecutionEngine::TaskExecutionEngine(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                                         plane::flight_control::FlightControlApi& flight_control,
                                         plane::state::StateStoreApi& state_store,
                                         plane::core::SystemParamApi& system_param,
                                         plane::perception::PerceptionApi& perception,
                                         plane::target::TargetStoreApi& target_store)
    : event_bus_(event_bus),
      config_(config),
      flight_control_(flight_control),
      state_store_(state_store),
      system_param_(system_param),
      perception_(perception),
      target_store_(target_store) {}
```

- [x] Step 3: `src/task/task_execution_engine.cpp` 增加 include

```cpp
#include "task/nodes/detect_node.h"
#include "task/nodes/guide_node.h"
#include "task/nodes/midcourse_node.h"
#include "task/nodes/track_node.h"
```

- [x] Step 4: 新增 `build_anti_uav()`（原 `build_suicide()`，后随 taskCode 枚举化重命名）

```cpp
BT::Tree TaskExecutionEngine::build_anti_uav() {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ArmNode>("Arm", &flight_control_, &state_store_, &config_);
    factory.registerNodeType<TakeoffNode>("Takeoff", &flight_control_, &state_store_, &config_, &system_param_);
    factory.registerNodeType<MidcourseNode>("Midcourse", &flight_control_, &state_store_, &target_store_, &config_,
                                            &suicide_context_);
    factory.registerNodeType<DetectNode>("Detect", &flight_control_, &state_store_, &perception_, &target_store_,
                                         &config_, &suicide_context_);
    factory.registerNodeType<TrackNode>("Track", &flight_control_, &state_store_, &perception_, &target_store_, &config_,
                                        &suicide_context_);
    factory.registerNodeType<GuideNode>("Guide", &flight_control_, &state_store_, &perception_, &target_store_,
                                        &config_);

    auto subtree = std::make_shared<BT::Tree::Subtree>();
    subtree->blackboard = BT::Blackboard::create();
    subtree->tree_ID = "anti-uav";
    subtree->instance_name = "anti-uav";

    BT::NodeConfig node_config;
    node_config.blackboard = subtree->blackboard;

    auto root = factory.instantiateTreeNode("Sequence", "Sequence", node_config);
    auto arm = factory.instantiateTreeNode("Arm", "Arm", node_config);
    auto takeoff = factory.instantiateTreeNode("Takeoff", "Takeoff", node_config);
    auto midcourse = factory.instantiateTreeNode("Midcourse", "Midcourse", node_config);
    auto detect = factory.instantiateTreeNode("Detect", "Detect", node_config);
    auto track = factory.instantiateTreeNode("Track", "Track", node_config);
    auto guide = factory.instantiateTreeNode("Guide", "Guide", node_config);

    auto* control = dynamic_cast<BT::ControlNode*>(root.get());
    control->addChild(arm.get());
    control->addChild(takeoff.get());
    control->addChild(midcourse.get());
    control->addChild(detect.get());
    control->addChild(track.get());
    control->addChild(guide.get());

    subtree->nodes.push_back(std::move(root));
    subtree->nodes.push_back(std::move(arm));
    subtree->nodes.push_back(std::move(takeoff));
    subtree->nodes.push_back(std::move(midcourse));
    subtree->nodes.push_back(std::move(detect));
    subtree->nodes.push_back(std::move(track));
    subtree->nodes.push_back(std::move(guide));

    BT::Tree tree;
    tree.subtrees.push_back(subtree);
    tree.initialize();
    return tree;
}
```

- [x] Step 5: `start()` 建树（taskCode 枚举化后的最终实现）

```cpp
bool TaskExecutionEngine::start(const plane::model::TaskInfo& task, std::string& error) {
    abort();
    std::lock_guard<std::mutex> lock(mutex_);
    try {
        suicide_context_.targetTrackId = task.targetTrackId;
        tree_ = build_anti_uav();
    } catch (const std::exception& e) {
        error = std::string("行为树创建失败: ") + e.what();
        return false;
    }
    current_tree_name_ = "anti-uav";
    tree_ready_ = true;
    current_node_.clear();
    ticking_ = true;
    tick_thread_ = std::thread([this]() { tick_loop(); });
    spdlog::info("行为树已创建: {}, taskId={}, taskCode={}", current_tree_name_, task.taskId,
                 std::string(magic_enum::enum_name(task.taskCode)));
    return true;
}
```

- [x] Step 6: `publish_progress()` 与 `finish()` 改为使用 `current_tree_name_`，并增加 `cleanup_control()`

```cpp
void TaskExecutionEngine::publish_progress(const std::string& node) {
    TaskProgress progress;
    progress.currentBehaviorTree = current_tree_name_;
    progress.currentNode = node;
    event_bus_.publish(events::kExecutionProgress, progress);
    spdlog::info("任务执行进度: {} / {}", current_tree_name_, node);
}

void TaskExecutionEngine::finish(plane::model::TaskStatus status, const std::string& detail) {
    ticking_ = false;
    cleanup_control();
    TaskEngineResult result;
    result.status = status;
    result.detail = detail;
    event_bus_.publish(events::kEngineResult, result);
    spdlog::info("行为树结束: {}, status={}, detail={}", current_tree_name_,
                 std::string(magic_enum::enum_name(status)), detail);
}

void TaskExecutionEngine::cleanup_control() {
    flight_control_.clear_ned_velocity();
    perception_.stop_detection();
    perception_.stop_tracking();
    perception_.stop_guidance();
}
```

- [x] Step 7: `abort()` 在树停止后调用 `cleanup_control()`（互斥锁范围之外）

```cpp
void TaskExecutionEngine::abort() {
    ticking_ = false;
    if (tick_thread_.joinable()) {
        tick_thread_.join();
    }
    bool had_tree = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        had_tree = tree_ready_;
        if (tree_ready_) {
            tree_.haltTree();
        }
        tree_ready_ = false;
        current_node_.clear();
    }
    cleanup_control();
    if (had_tree) {
        spdlog::info("任务中止，行为树已停止");
    }
}
```

- [x] Step 8: `src/main.cpp` 的 TaskExecutionEngine 构造改为

```cpp
    auto task_engine = std::make_unique<plane::task::TaskExecutionEngine>(
        event_bus, config, *flight_control, *state_store, *system_param_store, *perception, *target_store);
```

- [x] Step 9: 构建并提交

```bash
cmake --build build -j
timeout --preserve-status 2 ./build/onboard; echo "exit=$?"
clang-format --dry-run --Werror src/task/task_execution_engine.h src/task/task_execution_engine.cpp src/main.cpp
git add src/task/task_execution_engine.h src/task/task_execution_engine.cpp src/main.cpp
git commit -m "feat(m10): 引擎注册 suicide 任务树与终结清理"
```

Expected: 构建通过；默认 `--task-code anti-uav` 正常建树（旧 `suicide` 会被序列化层拒绝）。

---

## Task 10：stub 插件合成输出（联调辅助）

**Files**

- Modify: `plugins/detection_stub/detection_stub.cpp`
- Modify: `plugins/tracking_stub/tracking_stub.cpp`
- Modify: `plugins/guidance_stub/guidance_stub.cpp`
- Modify: `config/onboard.conf`（注释说明）

- [x] Step 1: `plugins/detection_stub/detection_stub.cpp` 改为

```cpp
#include <spdlog/spdlog.h>

#include <Pluma/Connector.hpp>
#include <Pluma/Pluma.hpp>
#include <cstdlib>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "functional/detection_api.h"

namespace plane::algorithm {
namespace {

class DetectionStub final : public DetectionPlugin {
   public:
    bool init(const DetectConfig& config) override {
        config_ = config;
        const char* mode = std::getenv("PLANE_STUB_DETECT");
        synthetic_center_ = (mode != nullptr && std::string(mode) == "center");
        spdlog::info("DetectionStub init: model_path={}, synthetic_center={}", config_.model_path,
                     synthetic_center_);
        return true;
    }

    bool detect(const cv::Mat& frame, std::vector<DetectTarget>& out, std::string&) override {
        out.clear();
        if (synthetic_center_ && !frame.empty()) {
            DetectTarget target;
            target.type = "stub";
            target.confidence = 0.9F;
            target.bbox.width = 100;
            target.bbox.height = 100;
            target.bbox.x = frame.cols / 2 - target.bbox.width / 2;
            target.bbox.y = frame.rows / 2 - target.bbox.height / 2;
            out.push_back(target);
        }
        return true;
    }

   private:
    DetectConfig config_;
    bool synthetic_center_ = false;
};

}  // namespace

PLUMA_INHERIT_PROVIDER(DetectionStub, DetectionPlugin);
PLUMA_PROVIDER_SOURCE(DetectionPlugin, 1, 1);

PLUMA_CONNECTOR
bool connectToPlumaHost(pluma::Host& host) {
    host.add(new DetectionStubProvider());
    return true;
}

}  // namespace plane::algorithm
```

- [x] Step 2: `plugins/tracking_stub/tracking_stub.cpp` 改为

```cpp
#include <spdlog/spdlog.h>

#include <Pluma/Connector.hpp>
#include <Pluma/Pluma.hpp>
#include <cstdlib>
#include <memory>
#include <opencv2/core.hpp>
#include <string>

#include "functional/tracking_api.h"

namespace plane::algorithm {
namespace {

class TrackingStub final : public TrackingPlugin {
   public:
    bool init(const TrackConfig& config) override {
        config_ = config;
        const char* lost_after = std::getenv("PLANE_STUB_TRACK_LOST_AFTER");
        if (lost_after != nullptr) {
            try {
                lost_after_ = std::stoi(lost_after);
            } catch (const std::exception&) {
                lost_after_ = 0;
            }
        }
        spdlog::info("TrackingStub init: model_path={}, lost_after={}", config_.model_path, lost_after_);
        return true;
    }

    bool start(const cv::Mat&, const DetectTarget& initTarget, std::string&) override {
        target_ = initTarget;
        return true;
    }

    bool track(const cv::Mat&, DetectTarget& out, std::string& error) override {
        ++call_count_;
        out = target_;
        if (lost_after_ > 0 && call_count_ > lost_after_) {
            error = "stub 模拟跟踪丢失";
            return false;
        }
        return true;
    }

   private:
    TrackConfig config_;
    DetectTarget target_;
    int lost_after_ = 0;
    int call_count_ = 0;
};

}  // namespace

PLUMA_INHERIT_PROVIDER(TrackingStub, TrackingPlugin);
PLUMA_PROVIDER_SOURCE(TrackingPlugin, 1, 1);

PLUMA_CONNECTOR
bool connectToPlumaHost(pluma::Host& host) {
    host.add(new TrackingStubProvider());
    return true;
}

}  // namespace plane::algorithm
```

- [x] Step 3: `plugins/guidance_stub/guidance_stub.cpp` 改为

```cpp
#include <spdlog/spdlog.h>

#include <Pluma/Connector.hpp>
#include <Pluma/Pluma.hpp>
#include <cstdlib>
#include <memory>
#include <string>

#include "functional/guidance_api.h"

namespace plane::algorithm {
namespace {

class GuidanceStub final : public GuidancePlugin {
   public:
    bool init(const GuidanceConfig& config) override {
        config_ = config;
        const char* mode = std::getenv("PLANE_STUB_GUIDE");
        forward_ = (mode != nullptr && std::string(mode) == "forward");
        spdlog::info("GuidanceStub init: desired_distance_m={}, forward={}", config_.desired_distance_m, forward_);
        return true;
    }

    bool guide(const DetectTarget&, const VehicleState&, NedVelocity& out, std::string&) override {
        out = NedVelocity{};
        if (forward_) {
            out.north_m_s = 1.0;
        }
        return true;
    }

   private:
    GuidanceConfig config_;
    bool forward_ = false;
};

}  // namespace

PLUMA_INHERIT_PROVIDER(GuidanceStub, GuidancePlugin);
PLUMA_PROVIDER_SOURCE(GuidancePlugin, 1, 1);

PLUMA_CONNECTOR
bool connectToPlumaHost(pluma::Host& host) {
    host.add(new GuidanceStubProvider());
    return true;
}

}  // namespace plane::algorithm
```

- [x] Step 4: `config/onboard.conf` 的自检注释追加联调说明

```text
# 无真实算法/图像源时的联调开关（仅影响 stub 插件）：
#   PLANE_STUB_DETECT=center          检测返回画面中心合成目标
#   PLANE_STUB_GUIDE=forward          导引输出 1 m/s 前向速度
#   PLANE_STUB_TRACK_LOST_AFTER=<n>   第 n 次跟踪后模拟丢失
```

- [x] Step 5: 构建并提交

```bash
cmake --build build -j
clang-format --dry-run --Werror plugins/detection_stub/detection_stub.cpp plugins/tracking_stub/tracking_stub.cpp plugins/guidance_stub/guidance_stub.cpp
git add plugins/detection_stub/detection_stub.cpp plugins/tracking_stub/tracking_stub.cpp plugins/guidance_stub/guidance_stub.cpp config/onboard.conf
git commit -m "feat(m10): stub 插件支持合成输出联调"
```

Expected: 构建通过；默认行为不变（未设环境变量时与 M6 一致）。

---

## Task 11：gcs_sim 支持 targetTrackId

**Files**

- Modify: `tools/gcs_sim/main.cpp`

- [x] Step 1: `Options` 增加字段

```cpp
    std::string target_track_id;
```

- [x] Step 2: 参数解析中，在 `--task-code` 分支之后追加

```cpp
        } else if (arg == "--target-track-id" && i + 1 < argc) {
            options.target_track_id = argv[++i];
```

- [x] Step 3: TaskInfo payload 改为

```cpp
        const std::string task_payload = nlohmann::json{
            {"taskId", "task-1"},
            {"taskCode", options.task_code},
            {"targetTrackId", options.target_track_id},
            {"taskAction",
             options.task_action}}.dump();
```

- [x] Step 4: 构建并提交

```bash
cmake --build build -j
clang-format --dry-run --Werror tools/gcs_sim/main.cpp
git add tools/gcs_sim/main.cpp
git commit -m "feat(m10): gcs_sim 支持 targetTrackId"
```

Expected: 构建通过；`--target-track-id t1` 时 TaskInfo 携带该字段。

---

## Task 12：部署打包脚本

**Files**

- Create: `tools/package.sh`

> 设计 §12 推荐 A（CMake install）；此处选择脚本方案（B）：CMake install 会把源配置原样复制，其中 `plugins_dir = build/plugins` 在 dist 布局下不可用；脚本可在打包时改写为 `plugins`，dist 开箱可运行。属设计偏差，记录于本任务。

- [x] Step 1: 写 `tools/package.sh`

```bash
#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${1:-${ROOT}/build}"
DIST_DIR="${ROOT}/dist"

cmake --build "${BUILD_DIR}" -j
rm -rf "${DIST_DIR}"
mkdir -p "${DIST_DIR}/plugins"
install -m 0755 "${BUILD_DIR}/onboard" "${DIST_DIR}/onboard"
install -m 0755 "${BUILD_DIR}"/plugins/*.so "${DIST_DIR}/plugins/"
sed 's|^plugins_dir = .*|plugins_dir = plugins|' "${ROOT}/config/onboard.conf" > "${DIST_DIR}/onboard.conf"

echo "打包完成: ${DIST_DIR}"
echo "运行: cd dist && ./onboard --config onboard.conf"
echo "注意: 三方运行库 (mavsdk/zenoh/pluma/opencv/ffmpeg/ZLMediaKit) 需按部署环境提供"
```

- [x] Step 2: 赋可执行权限、验证并提交

```bash
chmod +x tools/package.sh
./tools/package.sh
ls dist dist/plugins | head -20
git add tools/package.sh
git commit -m "chore(m10): 增加部署打包脚本"
```

Expected: `dist/` 含 `onboard`、`onboard.conf`、`plugins/*.so`。

---

## Task 13：验收、SPEC §12 走查与文档收尾

**Files**

- Modify: `docs/plans/m10-suicide-mission-tasks.md`（勾选与执行记录）
- Modify: `docs/DEV_PLAN.md`（§11 索引状态）

- [x] Step 1: 构建与规范检查

```bash
cmake --build build -j
clang-format --dry-run --Werror $(git ls-files src tools plugins)
grep -rn "std::cout\|printf(" src/ tools/ plugins/ || echo "no cout/printf"
```

Expected: 无 warning、格式通过、无 cout/printf。

- [x] Step 2: anti-uav 任务无 SITL 路径

```bash
# 终端 1
./build/onboard
# 终端 2
./build/gcs_sim --task-code anti-uav --target-track-id t1 --send-gcs-target t1 \
  --gcs-target-repeat 5 --listen-seconds 4
```

Expected（onboard）：`行为树已创建: anti-uav` → （SITL 下继续中段）→ `行为树结束: anti-uav, status=Failed`（无 SITL 时在 Arm 节点失败）；
gcs_sim：`TaskState status=None`、`Ack 结果: TaskInfo=true`。

- [x] Step 3: 目标缺失失败路径

```bash
./build/gcs_sim --task-code anti-uav --no-command --listen-seconds 4
```

Expected：`无法确定 GCS 目标（targetTrackId 为空且 GCS 目标数不为 1）` → 任务 Failed。

- [x] Step 4: stub 合成输出验证（无图像源）

```bash
PLANE_STUB_DETECT=center PLANE_STUB_GUIDE=forward PLANE_STUB_TRACK_LOST_AFTER=3 \
  timeout --preserve-status 3 ./build/onboard
```

将 `config/onboard.conf` 临时设 `perception_selftest_on_start = true` 后运行：

Expected：`检测插件已实例化` → `自检 detect 成功: targets=1` → `自检 track 成功` → `自检 guide 成功: vn=1`。
（验证后恢复 `perception_selftest_on_start = false`。）

- [x] Step 5: SPEC §12 验收清单走查

在本文档末尾「执行记录」中逐项记录 SPEC §12 结果：

- 飞控 4 项、指令与任务 8 项、安全高度、状态与通信、相机/图像/AI/目标、安全告警：沿用 M1–M9 已记录证据；
- 自杀式反无 5 项：阶段顺序/失败路径/替换由代码与日志确认，实际飞行项标注「待 SITL」；
- 无法在当前环境验证的项统一标注「待 SITL/实流」。

- [x] Step 6: 勾选本文档并记录执行偏差，更新 DEV_PLAN §11

```markdown
| `plans/m10-suicide-mission-tasks.md` | M10 任务级计划与执行记录 | 已完成（部分验收，SITL/实流待验） |
```

- [x] Step 7: 提交

```bash
git add docs
git commit -m "docs(m10): M10 部分验收与文档更新"
```

---

## 附：SITL/实流待用户验证清单（M10 交付）

1. 中段制导实际逼近：Offboard 速度生效、检测距离阈值切换；
2. 检测→选定→跟踪→导引完整阶段链路（真实/rtsp 源 + 算法插件）；
3. 跟踪丢失恢复与超时失败（`PLANE_STUB_TRACK_LOST_AFTER` 或真实算法）；
4. 导引阶段飞控控制（Perception 直连下发、`applied=true`）与撞击完成判定；
5. 执行中替换/中止：`haltTree` 后 Offboard 停止（`clear_ned_velocity`）、检测/跟踪/导引全部退出；
6. PX4 SITL 下 anti-uav 全流程演练与 SPEC §12 自杀式反无 5 项验收。

---

## 执行记录与偏差

### 提交清单

| 提交 | 内容 |
|---|---|
| `cf90490` | docs(m10): M10 设计确认与架构 ADR-018（ARCHITECTURE §15/§22/§23 + ADR-018、DEV_PLAN §11） |
| `1894051` | feat(m10): 增加 suicide 任务配置项 |
| `3012ad8` | feat(m10): 增加 Geographic Tool |
| `4702eee` | feat(m10): TargetStore 暴露跟踪状态 |
| `0d4454b` | feat(m10): Perception 直连飞控下发导引速度 |
| `43ac868` | feat(m10): 增加 suicide 节点基类超时与逼近控制 |
| `82177d5` | feat(m10): 增加中段制导与目标检测节点 |
| `a68969e` | feat(m10): 增加跟踪与导引节点 |
| `a6329ee` | fix(m10): 跟踪/导引节点本地时间函数重命名 |
| `0e4805d` | feat(m10): 引擎注册 suicide 任务树与终结清理 |
| `08311ac` | feat(m10): stub 插件支持合成输出联调 |
| `c2ab0e5` | feat(m10): gcs_sim 支持 targetTrackId |
| `3e44762` | chore(m10): 增加部署打包脚本 |

### 验收证据（2026-09-18，无 SITL/无实流）

1. **构建与规范**：构建 0 warning；`clang-format --dry-run --Werror`（src/tools/plugins 全部 .h/.cpp）通过；无 `std::cout/printf`。
2. **任务无 SITL 路径**（见「后续变更：taskCode 枚举化」最新证据）：`gcs_sim --task-code anti-uav --target-track-id t1 --send-gcs-target t1 --gcs-target-repeat 5` →
   onboard `行为树已创建: suicide, taskId=task-1` → `任务结束: status=Failed, detail=节点执行失败: Arm`（飞控未连接，首个节点失败即任务失败，符合 SPEC §5.8）；
   gcs_sim `Ack 结果: TaskInfo=true, Command=true`、`TaskState status=None`（终态回无任务）、`GCS 目标更新: trackId=t1, 历史=5`。
3. **stub 合成输出**：`PLANE_STUB_DETECT=center PLANE_STUB_GUIDE=forward PLANE_STUB_TRACK_LOST_AFTER=3` + 临时 `perception_selftest_on_start=true` →
   `DetectionStub init: synthetic_center=true`、`自检 detect 成功: targets=1`、`自检 track 成功: type=stub`、`GuidanceStub init: forward=true`、`自检 guide 成功: vn=1 ve=0 vd=0`。
4. **部署打包**：`tools/package.sh` 产出 `dist/{onboard, onboard.conf, plugins/*.so}`，`plugins_dir` 自动改写为 `plugins`。
5. **架构变更**：ARCHITECTURE §15/§22/§23 已更新，ADR-018 已记录；飞控下发通路现状核查（Scheduler/Engine 已直连，Perception 本次直连导引速度）。

### 偏差

1. **Task 8 编译修复**：`track_node.cpp`/`guide_node.cpp` 的匿名命名空间 `now_ms()` 与 `FlightActionNode` 私有成员同名，改用 `clock_now_ms()`（提交 `a6329ee`）。
2. **Task 9 析构顺序修复**：`~TaskExecutionEngine` 原调用 `stop()`→`abort()`→`cleanup_control()` 会解引用同级模块；`Runtime` 的 `modules_.clear()` 按正序析构，Perception 可能先于引擎析构（实测 exit=139）。拆出 `halt_tree()`：析构仅停止自身行为树，`abort()` = `halt_tree()` + `cleanup_control()`（提交 `0e4805d`）。
3. **Task 12 方案调整**：设计 §12 推荐 A（CMake install），实际采用脚本方案 B。原因：CMake install 原样复制 `config/onboard.conf`，其中 `plugins_dir = build/plugins` 在 dist 布局不可用；脚本打包时改写为 `plugins`，dist 可直接运行。另将 `dist/` 加入 `.gitignore`。
4. **Task 13 阶段逻辑验收受限**：无 SITL 时 `Arm` 节点即失败，`Midcourse/Detect/Track/Guide` 的中段目标解析、距离/方位、阶段切换、丢失失败、撞击判定与 Perception 直连下发路径无法在运行期触发，整体列入 SITL 待验清单。

### SPEC §12 验收清单走查

| 分类 | 项数 | 当前结论 |
|---|---|---|
| 飞控（连接/2 秒判定/重连） | 4 | 待 SITL（M3 遗留） |
| 指令与任务（执行/替换/ACK/顺序/中止/BT/状态上报） | 8 | ACK、替换、状态上报已验证；实际动作与顺序执行待 SITL（M4/M5 遗留 + M10） |
| 自杀式反无（阶段顺序/中段/算法调用/失败/替换） | 5 | 树与代码就绪；全部待 SITL（M10） |
| 安全高度 | 1 | 动态下发已验证；起飞使用待 SITL（M9） |
| 状态与通信 | 6 | 已验证（M1/M2；8 机规模待联调） |
| 相机/图像/AI/目标 | 3 | 链路已验收；实流取图/推流待源（M6/M7/M8） |
| 安全告警 | 1 | 待 SITL（M9） |

> 待 SITL 项按本文档「附：SITL/实流待用户验证清单」执行，结果回填本文档。

---

## 后续变更：taskCode 枚举化（用户 2026-09-18 追加）

### 变更内容

- `../common/common_include/message.h`：`TaskInfo.taskCode` 由 `std::string` 改为 `enum class TaskCode { AntiUav };`（当前仅一种任务类型）；
- wire JSON 的 `taskCode` 保持字符串 `"anti-uav"`，枚举↔字符串映射由 Communication 层负责（`serialization.cpp`；common_include 不依赖第三方库）；
- `TaskExecutionEngine`：删除 `basic_flight` 内置验证任务与 `build_basic_flight()`，只注册 anti-uav 树；行为树名/`currentBehaviorTree` 改为 `anti-uav`；
- `gcs_sim`：默认 `--task-code anti-uav`；
- 配置项 `suicide_*` 名称保留（指任务阶段参数，不随任务类型命名变化）。

### 提交

| 提交 | 内容 |
|---|---|
| `e1684f8` | refactor(m10): taskCode 改为枚举 TaskCode::AntiUav（onboard 侧） |
| `8ad0052` | feat(common): TaskInfo.taskCode 改为 TaskCode 枚举（common 仓库） |

### 验证证据（2026-09-18）

1. 构建 0 warning；
2. `gcs_sim --target-track-id t1 --send-gcs-target t1 --no-command`（默认 anti-uav）→ onboard
   `行为树已创建: anti-uav, taskId=task-1, taskCode=AntiUav` → `行为树结束: anti-uav, status=Failed, detail=节点执行失败: Arm`（无 SITL），gcs_sim `Ack 结果: TaskInfo=true`；
3. `gcs_sim --task-code suicide`（旧值）→ onboard `通信错误事件: TaskInfo 解码失败: 未知 TaskCode: suicide`，gcs_sim `TaskInfo=false`（消息被拒绝，任务未启动）。

### 影响与待办

- 地面站需按 `"anti-uav"` 下发 `taskCode`；旧 `"suicide"`/`"basic_flight"` 不再被接受；
- GCS 侧协议文档若引用旧值需同步；
- common 仓库变更已提交（`8ad0052`），onboard 与其他仓库（GCS 等）据此对齐。
