# M7 Change Design（Target Management）

> 状态：待用户确认（逐项）。任务级计划待确认后生成 `m7-target-management-tasks.md`。
> 来源：`DEV_PLAN.md` 路线图 M7。
> 输入：SPEC §8.4、ARCHITECTURE §16/ADR-015/§22、M6 产物（`perception.*` 事件）、M1 产物（`communication.gcs_target`）。

---

## 1. 目标与范围

M7 交付 Target Management：

- 统一维护两类目标：机载感知目标（最新检测列表 + 当前跟踪目标）与 GCS 目标（最新 + 每 trackId 20 条历史）；
- 提供同步查询接口 `TargetStoreApi`；
- 订阅 `communication.gcs_target`、`perception.detection`、`perception.tracking` 更新内部状态；
- 第一版约束：不接雷达；视觉目标只维护图像位置（不转 WGS84）；不做轨迹平滑（SPEC §8.4）。

不在 M7：目标选择规则（M10 任务参数）、轨迹评价、事件发布、持久化。

## 2. 已确认决策（用户 2026-09-18）

| # | 决策 |
|---|---|
| 1 | GCS 目标历史含最新一条，共 20 条（新记录顶出最旧） |
| 2 | 不新增目标事件，消费方同步查询（ARCH §22） |
| 3 | 视觉目标分开保存：最新检测列表 + 当前跟踪目标 |
| 4 | 验证用 gcs_sim 新增 `--send-gcs-target/--gcs-target-repeat` + TargetStore 更新日志 |
| 5 | 部分验收（无 SITL/无图像源） |

## 3. 模块与文件（`src/target/`）

| 文件 | 职责 |
|---|---|
| `target_store_api.h` | 同步查询接口 |
| `target_store.h/.cpp` | `core::IModule` + `TargetStoreApi`；订阅事件更新状态 |

```cpp
class TargetStoreApi {
   public:
    virtual ~TargetStoreApi() = default;

    virtual std::vector<plane::model::DetectTarget> vision_targets() const = 0;
    virtual bool tracked_target(plane::model::DetectTarget& out) const = 0;

    virtual std::vector<plane::model::GcsTarget> gcs_targets() const = 0;
    virtual bool gcs_target(const std::string& track_id, plane::model::GcsTarget& out) const = 0;
    virtual std::vector<plane::model::GcsTarget> gcs_target_history(const std::string& track_id) const = 0;
};
```

## 4. 数据模型与行为

- 视觉检测：`std::vector<DetectTarget> vision_targets_`，每收到一次 `perception.detection` 全量替换（最新为准，不保存历史）；
- 跟踪目标：`std::optional<DetectTarget> tracked_target_`，收到 `perception.tracking` 更新（`lost=true` 时保留最后已知位置，与插件接口语义一致）；
- GCS 目标：`std::unordered_map<std::string, GcsTarget> gcs_latest_` + `std::unordered_map<std::string, std::deque<GcsTarget>> gcs_history_`；
  - 每次更新：latest 覆盖；history `push_back` 后超过 `kGcsTargetHistoryDepth = 20` 弹出队首；
  - `gcs_target_history()` 返回按时间顺序的快照（旧→新，含最新）。
- 线程安全：内部一把互斥锁；事件回调与查询可能来自不同线程。

## 5. 日志（验收可观测性）

| 时机 | 日志 |
|---|---|
| detection 更新 | `视觉目标更新: N 个` |
| tracking 更新 | `跟踪目标更新: type=..., lost=0/1` |
| gcs target 更新 | `GCS 目标更新: trackId=..., 历史=M`（M 最大 20） |

## 6. gcs_sim 扩展

| 选项 | 说明 |
|---|---|
| `--send-gcs-target <trackId>` | 发送一条 GcsTarget（可重复） |
| `--gcs-target-repeat <n>` | 重复发送次数（默认 1） |

- GcsTarget 属状态类消息（非 Task/Command），无需 Ack；payload：`{trackId, geoPos, velocity_m_s, direction_deg}`；发送到 `sendToPlane/<plane-id>`。

## 7. main 接线

```text
StateStore → FlightControl → PluginManagement → CameraControl → Perception
→ TargetStore → TaskExecutionEngine → TaskScheduler → Communication → 其余占位
```

- 占位 skip 增加 `"Target Management"`。

## 8. M7 验收（部分验收）

1. 构建与格式通过；
2. 自检联动（`perception_selftest_on_start=true`）：TargetStore 日志出现 `视觉目标更新: 0 个` 与 `跟踪目标更新: type=stub, lost=0`；
3. GCS 目标与历史 cap：`gcs_sim --no-task --no-command --send-gcs-target t1 --gcs-target-repeat 25 --listen-seconds 2` → onboard 出现 25 条 `GCS 目标更新: trackId=t1`，最后一条 `历史=20`；
4. 回归与规范（M1–M6 行为不变；clang-format；无 cout/printf）。

待用户验证（有真实感知源时）：检测/跟踪结果驱动 `vision_targets/tracked_target` 更新。

## 9. M7 不做

- 目标选择与跟踪启动策略（M10）；
- 轨迹平滑、历史轨迹、轨迹评价；
- 雷达目标、视觉目标转 WGS84（SPEC §13.2）；
- 目标状态上报 GCS（SPEC 未要求）。

## 10. 风险与说明

| # | 事项 | 处理 |
|---|---|---|
| 1 | GCS 目标历史常驻内存，无失效清理 | 20 条/目标、对象数有限（态势规模小），可接受 |
| 2 | `tracked_target` 在 lost 时保留旧值 | 与跟踪插件接口注释一致；M10 依据 `lost` 与任务参数判定失败 |
| 3 | 无目标事件，异步消费者需轮询/同步查询 | 已确认；ARCH §22 将 Target 查询定义为同步调用 |
| 4 | `vision_targets` 全量替换，检测与跟踪可能短暂不一致 | 两类数据独立，符合「视觉目标只维护最新信息」 |
