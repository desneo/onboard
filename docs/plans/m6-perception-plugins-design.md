# M6 Change Design（相机、图像、插件框架与感知）

> 状态：待用户确认（逐项）。任务级计划待确认后生成 `m6-perception-plugins-tasks.md`。
> 来源：`DEV_PLAN.md` 路线图 M6。
> 输入：SPEC §8.1–§8.3、ARCHITECTURE §14/§15/§19/§20/ADR-010、`common_include/functional/*.h`、Pluma 示例与头文件。

---

## 1. 目标与范围

M6 交付：

- Camera Control：`CameraControlApi`（建联/状态/控制指令）+ Noop 实现（硬件 SDK 未提供）；
- Plugin Management：基于 Pluma 的宿主侧加载与按需实例化（`PluginManagerApi`）；
- 示例插件：Detection/Tracking/Guidance 三个 stub（打通加载与调用链路，不含算法）；
- Perception：MIPI/RTSP 图像源、最新帧模型、按需调度三类算法、结果事件发布、启动自检开关。

不在 M6：相机 SDK/真实控制、模型推理、MIPI 实机验证、目标管理（M7）、推流（M8）、M10 任务阶段接入。

## 2. 已确认决策（用户 2026-09-18）

| # | 决策 |
|---|---|
| 1 | 相机控制：接口 + Noop 实现（日志与状态占位），硬件/SDK 接入时替换实现 |
| 2 | 图像源单键 `image_source`：空=无源；纯数字=MIPI/V4L2 索引；其余=RTSP/OpenCV 可识别 URL；失败周期重连 |
| 3 | 多插件取第一个可用 provider，多个时日志提示 |
| 4 | stub 插件：成功 + 空/原样结果（不做算法） |
| 5 | 三接口验证：配置 `perception_selftest_on_start`，启动用合成灰帧跑一轮 detect/track/guide |
| 6 | 暂无 RTSP 源：M6 部分验收，实流取图待用户提供源 |

## 3. Camera Control

### 3.1 文件与接口（`src/camera/`）

| 文件 | 职责 |
|---|---|
| `camera_control_api.h` | 稳定接口 + `CameraStatus` |
| `camera_control_module.h/.cpp` | `core::IModule` + Noop 实现 |

```cpp
struct CameraStatus {
    bool connected = false;
    std::string detail;
};

class CameraControlApi {
   public:
    virtual ~CameraControlApi() = default;
    virtual bool connect(std::string& error) = 0;
    virtual void disconnect() = 0;
    virtual CameraStatus status() const = 0;
    /// 控制指令下发；指令集待相机 SDK 明确，第一版按字符串占位与日志记录。
    virtual bool send_command(const std::string& command, std::string& error) = 0;
};
```

- Noop 行为：`connect` 置 connected=true 并日志；`send_command` 记录日志返回 true；`disconnect` 复位；状态变化发布 `camera.status_changed` 事件（载荷 `CameraStatus`）。
- 说明：SPEC §8.1「具体指令集在接口设计阶段确定」，待 SDK 到位后细化并可能引入枚举/结构体。

## 4. Plugin Management（Pluma 宿主侧）

### 4.1 文件（`src/perception/`）

| 文件 | 职责 |
|---|---|
| `plugin_manager_api.h` | 对外接口 |
| `plugin_management.h/.cpp` | `core::IModule` + Pluma 宿主封装 |
| `plugin_providers.cpp` | 宿主侧 `PLUMA_PROVIDER_SOURCE` 定义（三类各 1,1） |

```cpp
class PluginManagerApi {
   public:
    virtual ~PluginManagerApi() = default;
    virtual std::shared_ptr<plane::algorithm::DetectionPlugin> create_detection() = 0;
    virtual std::shared_ptr<plane::algorithm::TrackingPlugin> create_tracking() = 0;
    virtual std::shared_ptr<plane::algorithm::GuidancePlugin> create_guidance() = 0;
};
```

- `start()`：`acceptProviderType<XxxProvider>()`（三类）→ `loadFromFolder(plugins_dir)` → 统计并日志每类 provider 数量（0 则 warn，不致命）；
- `create_xxx()`：取该类型第一个 provider → `create()` → 包装为 `shared_ptr`（默认 deleter）；无 provider 返回 `nullptr`；
- Pluma 模式（宿主/插件两侧都需要）：
  - 宿主 `plugin_providers.cpp` 定义 `PLUMA_PROVIDER_SOURCE(XxxPlugin, 1, 1)`；
  - 插件导出 `PLUMA_CONNECTOR bool connectToPlumaHost(pluma::Host&)` 并 `host.add(new XxxStubProvider())`。
- `stop()`：在 Perception 之后（注册顺序保证）`unloadAll()`。

## 5. 示例插件（`plugins/`）

| 目录 | 库名 | 行为 |
|---|---|---|
| `plugins/detection_stub/` | `libdetection_stub.so` | `init` 记录配置；`detect` 返回空列表成功 |
| `plugins/tracking_stub/` | `libtracking_stub.so` | `init`；`start` 记录目标；`track` 回传目标成功 |
| `plugins/guidance_stub/` | `libguidance_stub.so` | `init`；`guide` 输出零速度成功 |

- 均按 `common_include/functional/*.h` 接口实现，`PLUMA_INHERIT_PROVIDER` + `PLUMA_PROVIDER_SOURCE(1,1)` + `connectToPlumaHost`；
- CMake：`add_library(... SHARED)`，链接 `plane::pluma`、`plane::common_include`、`opencv_core`，输出到 `${CMAKE_BINARY_DIR}/plugins`；
- 算法本体不在本项目，替换插件即替换算法。

## 6. Perception

### 6.1 文件与事件（`src/perception/`）

| 文件 | 职责 |
|---|---|
| `perception_api.h` | 业务级控制接口与结果结构 |
| `perception_events.h` | 事件 ID（4 个） |
| `perception.h/.cpp` | `core::IModule` + `PerceptionApi`：采集线程 + 处理线程 + 插件调度 |

```cpp
struct DetectionResult { std::vector<plane::model::DetectTarget> targets; };
struct TrackingResult { plane::model::DetectTarget target; bool lost = false; };
struct GuidanceResult { plane::model::NedVelocity velocity; bool valid = false; };
struct PerceptionError { std::string detail; };

class PerceptionApi {
   public:
    virtual ~PerceptionApi() = default;
    virtual bool start_detection(std::string& error) = 0;
    virtual void stop_detection() = 0;
    virtual bool select_target(const plane::model::DetectTarget& target, std::string& error) = 0;
    virtual void stop_tracking() = 0;
    virtual bool start_guidance(std::string& error) = 0;
    virtual void stop_guidance() = 0;
    virtual bool latest_frame(cv::Mat& out) const = 0;
};
```

| Event ID | 载荷 |
|---|---|
| `perception.detection` | `DetectionResult` |
| `perception.tracking` | `TrackingResult` |
| `perception.guidance` | `GuidanceResult` |
| `perception.error` | `PerceptionError` |

### 6.2 图像采集（最新帧模型）

- 采集线程：按 `image_source` 打开（`cv::VideoCapture`）；
  - 空 → warn「图像源未配置」并等待配置/退出；
  - 纯数字 → MIPI/V4L2 索引；其余 → URL（RTSP）；
- 读取成功：帧写入最新帧缓存（互斥锁，覆盖旧帧，不建队列，ADR-009）；首次成功日志「图像源已打开 + 首帧尺寸」；
- 读取失败：关闭并每 `image_reconnect_interval_ms` 重试，warn 日志；不阻塞其他模块。

### 6.3 算法调度（处理线程）

- 周期 `perception_process_interval_ms`：
  1. 取最新帧（无帧则跳过）；
  2. 若检测开启：惰性创建 detection 插件 → `detect(frame, targets)` → 发布 `perception.detection`；失败发布 `perception.error`；
  3. 若跟踪开启：`track(frame, target, error)` → 发布 `perception.tracking`（`lost = track 返回 false`）；
  4. 若导引开启且存在跟踪目标：`guide(target, state_store.own_state(), velocity)` → 发布 `perception.guidance`；
- 插件实例惰性创建（ARCHITECTURE §19：库加载与对象实例化分离，默认按需实例化），插件配置先用默认值（模型接入时再扩展配置项）；
- 启动自检（`perception_selftest_on_start=true`）：处理线程首轮用 `cv::Mat::zeros(480,640,CV_8UC3)` 跑一轮 detect →（无目标则构造中心框作为跟踪初始目标）track → guide，并日志输出结果。

## 7. 配置

| key | 默认 | 说明 |
|---|---|---|
| `plugins_dir` | `build/plugins` | 插件目录（相对运行目录） |
| `image_source` | 空 | 空=无源；数字=MIPI/V4L2 索引；其余=RTSP/OpenCV URL |
| `image_reconnect_interval_ms` | 2000 | 采集失败重连间隔 |
| `perception_process_interval_ms` | 100 | 检测/跟踪/导引调度周期 |
| `perception_selftest_on_start` | false | 启动自检开关（合成灰帧跑一轮三接口） |

## 8. main 接线

```text
StateStore → FlightControl → PluginManagement → CameraControl → Perception
→ TaskExecutionEngine → TaskScheduler → Communication → 其余占位
```

- 占位 skip 增加 `"Plugin Management"`、`"Camera Control"`、`"Perception"`；
- 关闭顺序（Runtime 逆序）保证 Perception 先于 PluginManagement 停止，插件对象先释放再 `unloadAll()`。

## 9. M6 验收（部分验收）

1. 构建与格式通过；`build/plugins/` 下生成三个 stub `.so`；
2. 默认运行：日志 `Plugin Management` 加载结果（detection/tracking/guidance 各 1 个 provider）、`图像源未配置` warn，进程稳定；
3. 自检：临时配置 `perception_selftest_on_start=true` 启动 →
   - 日志：插件实例化、`自检 detect targets=0`、`自检 track 成功`、`自检 guide 有效速度=0`；
   - 发布对应事件（日志体现）；
4. 回归：M1–M5 行为正常；
5. 待用户 RTSP 源：`image_source=rtsp://...` → `图像源已打开` + 首帧尺寸；随后可开启检测观察周期结果（接口已就绪）。

## 10. M6 不做

- 相机 SDK 接入与真实控制指令集（接口已留）；
- 模型推理与真实算法（替换插件即可）；
- 目标管理（M7）与轨迹处理；
- 推流（M8）；
- M10 任务阶段调度接入（节点调用 `PerceptionApi`）。

## 11. 风险与说明

| # | 事项 | 处理 |
|---|---|---|
| 1 | Camera Noop 无真实效果 | 已确认；SDK 到位后仅替换模块内部实现 |
| 2 | `send_command` 为字符串占位 | SPEC 允许指令集后定；届时可能调整接口并同步文档 |
| 3 | 插件与宿主 ABI/依赖版本需一致 | stub 与宿主同工程构建；第三方插件需同 C++/OpenCV/Pluma 版本 |
| 4 | `PLUMA_PROVIDER_SOURCE` 版本号宿主/插件须一致（1,1） | 在插件模板与宿主中统一 |
| 5 | 无 RTSP 源时取图路径未实流验证 | 部分验收；用户提供源后补验 |
| 6 | 处理线程直接调用插件可能耗时 | 第一版接受；性能问题在 M10 联调时评估（ARCH §24.2 允许独立线程） |
