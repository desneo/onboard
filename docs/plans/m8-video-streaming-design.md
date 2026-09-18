# M8 Change Design（Video Streaming：H264 + ZLMediaKit RTSP）

> 状态：待用户确认（逐项）。任务级计划待确认后生成 `m8-video-streaming-tasks.md`。
> 来源：`DEV_PLAN.md` 路线图 M8。
> 输入：SPEC §8.2/§9.1、ARCHITECTURE §17/ADR-011、M6 产物（Perception）、ZLMediaKit C API 头文件、系统 FFmpeg 实测。

---

## 1. 目标与范围

M8 交付 Video Streaming：

- Video Streaming Capability Module：接收 Perception 帧与目标叠加信息、编码 H264、经 ZLMediaKit 对外提供 RTSP 流；
- H264 编码抽象：x86 使用系统 FFmpeg（libavcodec + libx264，已实测可用），RK3588 后续替换 MPP 实现；
- ZLMediaKit 进程内 RTSP 服务（`mk_env_init2` + `mk_media` + `mk_media_input_h264`）；
- ZLMediaKit 日志接管到文件（抑制 stdout 噪音）；
- 推流独立于图像处理与检测开关：Perception 每周期交付最新帧。

不在 M8：硬件 MPP 编码、音频、录像、多路流、码率自适应、RTMP/HLS/WebRTC、推流鉴权。

## 2. 已确认决策（用户 2026-09-18）

| # | 决策 |
|---|---|
| 1 | x86 系统 FFmpeg（libavcodec/libavutil/libswscale，pkg-config 接入）；RK3588 后续换 MPP |
| 2 | 进程内 ZLMediaKit 作 RTSP 服务，客户端拉流 |
| 3 | Perception 同步 `push_frame` + 最新帧槽（编码线程消费，不阻塞感知）；推流独立于检测开关 |
| 4 | 编码前绘制检测/跟踪框，`video_overlay_enabled` 可关；配置 `rtsp_port/rtsp_app/rtsp_stream/video_fps/video_bitrate_kbps` |
| 5 | ZLMediaKit 日志到 `logs/zlmediakit.log`，级别 warn，抑制 stdout |
| 6 | 验收：构建与启动部分验收；实流拉取由用户提供源后验证（不新增 synthetic 源与 rtsp_probe） |

## 3. 模块与文件（`src/video/`）

| 文件 | 职责 |
|---|---|
| `video_streaming_api.h` | `VideoOverlay` + `VideoStreamingApi` |
| `h264_encoder.h` | 编码器抽象接口（FFmpeg/MPP 可替换） |
| `ffmpeg_h264_encoder.h/.cpp` | 系统 FFmpeg 实现（libx264，Annex-B 输出） |
| `rtsp_media_server.h/.cpp` | ZLMediaKit C API 封装（初始化/媒体源/输入帧/释放） |
| `video_streaming.h/.cpp` | `core::IModule` + `VideoStreamingApi`：最新帧槽、编码线程、叠加绘制、喂流 |

```cpp
struct VideoOverlay {
    std::vector<plane::model::DetectTarget> targets;  ///< 检测框
    bool has_tracked = false;
    plane::model::DetectTarget tracked;               ///< 跟踪框（高亮）
};

class VideoStreamingApi {
   public:
    virtual ~VideoStreamingApi() = default;
    virtual bool start_stream(std::string& error) = 0;
    virtual void stop_stream() = 0;
    virtual bool push_frame(const cv::Mat& frame, const VideoOverlay& overlay, std::string& error) = 0;
};

class H264Encoder {
   public:
    virtual ~H264Encoder() = default;
    virtual bool init(int width, int height, int fps, int bitrate_kbps, std::string& error) = 0;
    virtual bool encode(const cv::Mat& bgr_frame, std::vector<std::uint8_t>& out, bool& keyframe,
                        std::string& error) = 0;
};
```

## 4. 编码（FFmpeg 实现要点）

- `avcodec_find_encoder(AV_CODEC_ID_H264)`（实测解析到 `libx264`）；`pix_fmt=YUV420P`、`time_base={1,fps}`、`bit_rate`、`g=fps*2`、`max_b_frames=0`，`AVDictionary{preset=veryfast, tune=zerolatency}`；
- 每帧：`sws_scale` BGR→YUV420P → `avcodec_send_frame` → `avcodec_receive_packet`（可能有多个包）→ 输出 Annex-B 数据与 keyframe 标记；
- 编码器在首帧或分辨率变化时重建；`dts/pts` 由帧序号与 fps 计算（ms）。

## 5. RTSP 服务（ZLMediaKit）

- `mk_env_init2(thread_num=2, log_level=3(warn), log_mask=LOG_FILE, "logs/zlmediakit.log", 1, 0, ini, 0, nullptr, nullptr)`；`ini` 内容设置 RTSP 端口：
  ```ini
  [rtsp]
  port=8554
  ```
- 首个帧到达时惰性创建媒体源：`mk_media_create("__defaultVhost__", app, stream, 0,0,0)` → `mk_track_create(MKCodecH264, {w,h,fps})` → `mk_media_init_track` → `mk_media_init_complete`；
- 每帧：`mk_media_input_h264(media, data, len, dts_ms, pts_ms)`；
- 关闭：`mk_media_release`（服务进程内运行，不依赖 MediaServer 可执行文件）；
- 拉流地址：`rtsp://<ip>:<rtsp_port>/<rtsp_app>/<rtsp_stream>`。

## 6. 帧交付、叠加与线程

- Perception 处理线程每周期在拿到帧后调用 `video_streaming_.push_frame(frame, overlay, error)`（不等待编码）；
- push_frame：把帧与叠加信息写入「最新帧槽」（覆盖旧帧，不排队），`condition_variable` 唤醒编码线程；
- 编码线程：取最新帧 → 若 `video_overlay_enabled` 在克隆帧上绘制检测框（绿）与跟踪框（红）→ 编码 → 喂 ZLMediaKit；
- 帧率控制：按 `video_fps` 节流（避免满速编码）；
- `start_stream/stop_stream` 控制编码线程与媒体源生命周期；模块 `start` 时默认启动推流（SPEC §8.2 要求图像处理按任务阶段推流，但第一版采用常开，简化且符合“独立于图像处理”）。

## 7. 配置

| key | 默认 | 说明 |
|---|---|---|
| `rtsp_port` | 8554 | ZLMediaKit RTSP 服务端口（非 root 可用） |
| `rtsp_app` | `live` | 应用名 |
| `rtsp_stream` | `onboard` | 流名 |
| `video_fps` | 25 | 编码帧率（同时用于节流与时间戳） |
| `video_bitrate_kbps` | 2000 | 编码码率 |
| `video_overlay_enabled` | true | 是否叠加目标框 |

## 8. 依赖接入与 main 接线

- `cmake/Dependencies.cmake`：`find_package(PkgConfig REQUIRED)` + `pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavutil libswscale)`；onboard 链接 `PkgConfig::FFMPEG`；
- 注册顺序调整为 `... CameraControl → VideoStreaming → Perception → ...`（Perception 持 `VideoStreamingApi&`）；占位 skip 增加 `"Video Streaming"`；
- Perception 构造注入 `VideoStreamingApi&`。

## 9. M8 验收（部分验收）

1. 构建与格式通过；
2. 启动：无图像源时 VideoStreaming 正常初始化（ZLMediaKit 日志写入文件、stdout 无 codec 噪音）；进程稳定退出；
3. 回归：M1–M7 行为不变；
4. 待用户提供源（本地视频/RTSP/MIPI）：
   - 帧进入推流（日志：`推流首帧: WxH`）；
   - 用 ffprobe/播放器拉 `rtsp://<ip>:8554/live/onboard` 成功；
   - `video_overlay_enabled` 开/关对比；
   - 停止推流不影响 Perception 取图（停止后检测仍运行）。

## 10. M8 不做

- RK3588 MPP 硬编（后续替换 `H264Encoder` 实现）；
- 音频、录像、多路流、RTMP/HLS/WebRTC、鉴权；
- 码率自适应与拥塞控制；
- 目标叠加字段扩展（第一版检测框+跟踪框）。

## 11. 风险与说明

| # | 事项 | 处理 |
|---|---|---|
| 1 | 新增系统依赖 FFmpeg 开发包（已在环境具备） | 已确认；部署机需安装 libavcodec-dev 等（M10 部署清单记录） |
| 2 | ZLMediaKit 默认 ini 未显式配置端口时使用 554 | 通过 ini 内容设置 8554，非 root 运行 |
| 3 | BGR→YUV 软件转换为 CPU 开销 | 第一版接受；MPP/硬编在 RK3588 替换 |
| 4 | 编码线程慢时丢帧 | 最新帧槽覆盖策略，符合实时优先 |
| 5 | 无源环境无法端到端验证 | 用户提供源后验证；本任务只保证初始化与稳定 |
| 6 | ZLMediaKit 静态初始化噪音在 mk_env_init 前可能出现 | 首次调用即初始化环境；日志文件承接后续输出 |
