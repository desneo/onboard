# M8 任务级计划（H264 + ZLMediaKit RTSP 推流）

> 设计依据：`m8-video-streaming-design.md`（逐项确认；fps 与码率为配置项）。
> 执行位置：工程根 `onboard/`。
> 验证原则：每个 Task 结束后 x64 Release 构建通过；不写测试代码（SPEC §11）。
> 状态：已完成并部分验收（2026-09-18）；实流拉取由用户提供源后验证。

---

## Task 1：推流配置项

**Files**

- Modify: `src/core/config.h`
- Modify: `src/core/config.cpp`
- Modify: `config/onboard.conf`

- [x] Step 1: `src/core/config.h` 的 `AppConfig` 中，在 `perception_selftest_on_start` 之后追加

```cpp
    int rtsp_port = 8554;
    std::string rtsp_app = "live";
    std::string rtsp_stream = "onboard";
    int video_fps = 25;
    int video_bitrate_kbps = 2000;
    bool video_overlay_enabled = true;
```

- [x] Step 2: `src/core/config.cpp` 解析分支中，在 `perception_selftest_on_start` 分支之后追加

```cpp
        } else if (key == "rtsp_port") {
            out.rtsp_port = std::stoi(value);
        } else if (key == "rtsp_app") {
            out.rtsp_app = value;
        } else if (key == "rtsp_stream") {
            out.rtsp_stream = value;
        } else if (key == "video_fps") {
            out.video_fps = std::stoi(value);
        } else if (key == "video_bitrate_kbps") {
            out.video_bitrate_kbps = std::stoi(value);
        } else if (key == "video_overlay_enabled") {
            out.video_overlay_enabled = (value == "true" || value == "1");
```

- [x] Step 3: `config/onboard.conf` 末尾追加

```text
# 视频推流
rtsp_port = 8554
rtsp_app = live
rtsp_stream = onboard
video_fps = 25
video_bitrate_kbps = 2000
video_overlay_enabled = true
```

- [x] Step 4: 构建、运行与提交

```bash
cmake --build build -j
timeout --preserve-status 2 ./build/onboard; echo "exit=$?"
clang-format --dry-run --Werror src/core/config.h src/core/config.cpp
git add src/core/config.h src/core/config.cpp config/onboard.conf
git commit -m "feat(m8): 增加视频推流配置"
```

Expected: 构建通过；运行无「未知配置项」，exit=0。

---

## Task 2：FFmpeg 依赖与 H264 编码器

**Files**

- Modify: `cmake/Dependencies.cmake`
- Modify: `CMakeLists.txt`
- Create: `src/video/h264_encoder.h`
- Create: `src/video/ffmpeg_h264_encoder.h`
- Create: `src/video/ffmpeg_h264_encoder.cpp`

- [x] Step 1: `cmake/Dependencies.cmake` 末尾追加

```cmake
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavutil libswscale)
```

- [x] Step 2: `CMakeLists.txt` 的 `target_link_libraries(onboard PRIVATE ...)` 追加

```cmake
  PkgConfig::FFMPEG
  opencv_imgproc
```

- [x] Step 3: 写 `src/video/h264_encoder.h`

```cpp
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace plane::video {

/// H264 编码器抽象；x86 用 FFmpeg，RK3588 后续替换 MPP 实现。
class H264Encoder {
   public:
    virtual ~H264Encoder() = default;

    virtual bool init(int width, int height, int fps, int bitrate_kbps, std::string& error) = 0;
    virtual bool encode(const cv::Mat& bgr_frame, std::vector<std::uint8_t>& out, bool& keyframe,
                        std::string& error) = 0;
};

}  // namespace plane::video
```

- [x] Step 4: 写 `src/video/ffmpeg_h264_encoder.h`

```cpp
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "video/h264_encoder.h"

struct AVCodecContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;

namespace plane::video {

/// 系统 FFmpeg（libx264）实现，输出 Annex-B。
class FfmpegH264Encoder final : public H264Encoder {
   public:
    FfmpegH264Encoder();
    ~FfmpegH264Encoder() override;

    bool init(int width, int height, int fps, int bitrate_kbps, std::string& error) override;
    bool encode(const cv::Mat& bgr_frame, std::vector<std::uint8_t>& out, bool& keyframe,
                std::string& error) override;

   private:
    bool open_codec(int width, int height, int fps, int bitrate_kbps, std::string& error);
    void close();

    AVCodecContext* codec_ = nullptr;
    AVFrame* frame_ = nullptr;
    AVPacket* packet_ = nullptr;
    SwsContext* sws_ = nullptr;
    std::int64_t pts_ = 0;
    int width_ = 0;
    int height_ = 0;
    int fps_ = 0;
};

}  // namespace plane::video
```

- [x] Step 5: 写 `src/video/ffmpeg_h264_encoder.cpp`

```cpp
#include "video/ffmpeg_h264_encoder.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <cstdint>

namespace plane::video {

FfmpegH264Encoder::FfmpegH264Encoder() = default;

FfmpegH264Encoder::~FfmpegH264Encoder() {
    close();
}

bool FfmpegH264Encoder::init(int width, int height, int fps, int bitrate_kbps, std::string& error) {
    if (codec_ != nullptr && width == width_ && height == height_ && fps == fps_) {
        return true;
    }
    close();
    return open_codec(width, height, fps, bitrate_kbps, error);
}

bool FfmpegH264Encoder::open_codec(int width, int height, int fps, int bitrate_kbps,
                                   std::string& error) {
    const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (codec == nullptr) {
        error = "未找到 H264 编码器";
        return false;
    }
    codec_ = avcodec_alloc_context3(codec);
    if (codec_ == nullptr) {
        error = "编码器上下文分配失败";
        return false;
    }
    codec_->width = width;
    codec_->height = height;
    codec_->time_base = AVRational{1, fps};
    codec_->framerate = AVRational{fps, 1};
    codec_->pix_fmt = AV_PIX_FMT_YUV420P;
    codec_->bit_rate = static_cast<std::int64_t>(bitrate_kbps) * 1000;
    codec_->gop_size = fps * 2;
    codec_->max_b_frames = 0;

    AVDictionary* options = nullptr;
    av_dict_set(&options, "preset", "veryfast", 0);
    av_dict_set(&options, "tune", "zerolatency", 0);
    const int open_result = avcodec_open2(codec_, codec, &options);
    av_dict_free(&options);
    if (open_result < 0) {
        error = "H264 编码器打开失败";
        close();
        return false;
    }

    frame_ = av_frame_alloc();
    packet_ = av_packet_alloc();
    if (frame_ == nullptr || packet_ == nullptr) {
        error = "编码帧/包分配失败";
        close();
        return false;
    }
    frame_->format = codec_->pix_fmt;
    frame_->width = width;
    frame_->height = height;
    if (av_frame_get_buffer(frame_, 32) < 0) {
        error = "编码帧缓冲分配失败";
        close();
        return false;
    }

    sws_ = sws_getContext(width, height, AV_PIX_FMT_BGR24, width, height, AV_PIX_FMT_YUV420P,
                          SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (sws_ == nullptr) {
        error = "颜色转换上下文创建失败";
        close();
        return false;
    }

    width_ = width;
    height_ = height;
    fps_ = fps;
    pts_ = 0;
    return true;
}

bool FfmpegH264Encoder::encode(const cv::Mat& bgr_frame, std::vector<std::uint8_t>& out,
                               bool& keyframe, std::string& error) {
    out.clear();
    keyframe = false;
    if (codec_ == nullptr) {
        error = "编码器未初始化";
        return false;
    }
    if (bgr_frame.empty() || bgr_frame.cols != width_ || bgr_frame.rows != height_) {
        error = "帧尺寸与编码器不一致";
        return false;
    }
    const std::uint8_t* src_data[1] = {bgr_frame.data};
    const int src_linesize[1] = {static_cast<int>(bgr_frame.step)};
    sws_scale(sws_, src_data, src_linesize, 0, height_, frame_->data, frame_->linesize);

    frame_->pts = pts_++;
    int result = avcodec_send_frame(codec_, frame_);
    if (result < 0) {
        error = "发送编码帧失败";
        return false;
    }
    while (true) {
        result = avcodec_receive_packet(codec_, packet_);
        if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) {
            break;
        }
        if (result < 0) {
            error = "接收编码包失败";
            return false;
        }
        out.insert(out.end(), packet_->data, packet_->data + packet_->size);
        if ((packet_->flags & AV_PKT_FLAG_KEY) != 0) {
            keyframe = true;
        }
        av_packet_unref(packet_);
    }
    return true;
}

void FfmpegH264Encoder::close() {
    if (sws_ != nullptr) {
        sws_freeContext(sws_);
        sws_ = nullptr;
    }
    if (frame_ != nullptr) {
        av_frame_free(&frame_);
    }
    if (packet_ != nullptr) {
        av_packet_free(&packet_);
    }
    if (codec_ != nullptr) {
        avcodec_free_context(&codec_);
    }
    width_ = 0;
    height_ = 0;
    fps_ = 0;
    pts_ = 0;
}

}  // namespace plane::video
```

- [x] Step 6: 构建与格式

```bash
cmake --build build -j
clang-format --dry-run --Werror src/video/h264_encoder.h src/video/ffmpeg_h264_encoder.h src/video/ffmpeg_h264_encoder.cpp
```

Expected: 构建通过且无 warning；格式通过（差异先 `-i` 再构建）。

- [x] Step 7: 提交

```bash
git add cmake/Dependencies.cmake CMakeLists.txt src/video
git commit -m "feat(m8): 增加 FFmpeg H264 编码器抽象与实现"
```

---

## Task 3：ZLMediaKit RTSP 服务封装

**Files**

- Create: `src/video/rtsp_media_server.h`
- Create: `src/video/rtsp_media_server.cpp`

- [x] Step 1: 写 `src/video/rtsp_media_server.h`

```cpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace plane::video {

/// ZLMediaKit 进程内 RTSP 服务封装（PIMPL，C API 不外泄）。
class RtspMediaServer {
   public:
    RtspMediaServer();
    ~RtspMediaServer();
    RtspMediaServer(const RtspMediaServer&) = delete;
    RtspMediaServer& operator=(const RtspMediaServer&) = delete;

    bool init(int rtsp_port, std::string& error);
    bool ensure_media(const std::string& app, const std::string& stream, int width, int height,
                      int fps, int bitrate_kbps, std::string& error);
    bool input_h264(const std::uint8_t* data, std::size_t size, std::uint64_t dts_ms);
    void close();

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace plane::video
```

- [x] Step 2: 写 `src/video/rtsp_media_server.cpp`

```cpp
#include "video/rtsp_media_server.h"

#include <string>
#include <utility>

#include <spdlog/spdlog.h>

#include "mk_mediakit.h"

namespace plane::video {

struct RtspMediaServer::Impl {
    mk_media media = nullptr;
    bool env_initialized = false;
    int width = 0;
    int height = 0;
    int fps = 0;
};

RtspMediaServer::RtspMediaServer() : impl_(std::make_unique<Impl>()) {}

RtspMediaServer::~RtspMediaServer() {
    close();
}

bool RtspMediaServer::init(int rtsp_port, std::string& error) {
    if (impl_->env_initialized) {
        return true;
    }
    const std::string ini = "[rtsp]\nport=" + std::to_string(rtsp_port);
    mk_env_init2(2, 3, LOG_FILE, "logs/zlmediakit.log", 1, 0, ini.c_str(), 0, nullptr, nullptr);
    mk_set_log(50, 5);
    impl_->env_initialized = true;
    spdlog::info("ZLMediaKit 环境已初始化: rtsp_port={}, 日志=logs/zlmediakit.log", rtsp_port);
    return true;
}

bool RtspMediaServer::ensure_media(const std::string& app, const std::string& stream, int width,
                                   int height, int fps, int bitrate_kbps, std::string& error) {
    if (impl_->media != nullptr && width == impl_->width && height == impl_->height &&
        fps == impl_->fps) {
        return true;
    }
    if (impl_->media != nullptr) {
        mk_media_release(impl_->media);
        impl_->media = nullptr;
    }
    mk_media media = mk_media_create("__defaultVhost__", app.c_str(), stream.c_str(), 0, 0, 0);
    if (media == nullptr) {
        error = "媒体源创建失败";
        return false;
    }
    codec_args args;
    args.video.width = width;
    args.video.height = height;
    args.video.fps = fps;
    mk_track track = mk_track_create(MKCodecH264, &args);
    mk_media_init_track(media, track);
    mk_media_init_complete(media);
    mk_track_unref(track);
    impl_->media = media;
    impl_->width = width;
    impl_->height = height;
    impl_->fps = fps;
    spdlog::info("RTSP 流已创建: rtsp://<ip>:<port>/{}/{} ({}x{}@{}fps, {}kbps)", app, stream,
                 width, height, fps, bitrate_kbps);
    return true;
}

bool RtspMediaServer::input_h264(const std::uint8_t* data, std::size_t size, std::uint64_t dts_ms) {
    if (impl_->media == nullptr) {
        return false;
    }
    return mk_media_input_h264(impl_->media, data, static_cast<int>(size), dts_ms, dts_ms) != 0;
}

void RtspMediaServer::close() {
    if (impl_ && impl_->media != nullptr) {
        mk_media_release(impl_->media);
        impl_->media = nullptr;
    }
}

}  // namespace plane::video
```

- [x] Step 3: 构建与格式

```bash
cmake --build build -j
clang-format --dry-run --Werror src/video/rtsp_media_server.h src/video/rtsp_media_server.cpp
```

Expected: 构建通过且无 warning；格式通过（差异先 `-i` 再构建）。

- [x] Step 4: 提交

```bash
git add src/video/rtsp_media_server.h src/video/rtsp_media_server.cpp
git commit -m "feat(m8): 增加 ZLMediaKit RTSP 服务封装"
```

---

## Task 4：VideoStreaming 模块

**Files**

- Create: `src/video/video_streaming_api.h`
- Create: `src/video/video_streaming.h`
- Create: `src/video/video_streaming.cpp`
- Modify: `CMakeLists.txt`

- [x] Step 1: 写 `src/video/video_streaming_api.h`

```cpp
#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "message.h"

namespace plane::video {

/// 目标叠加信息（检测框 + 跟踪框）。
struct VideoOverlay {
    std::vector<plane::model::DetectTarget> targets;
    bool has_tracked = false;
    plane::model::DetectTarget tracked;
};

/// 推流对外接口（ARCHITECTURE §17）。
class VideoStreamingApi {
   public:
    virtual ~VideoStreamingApi() = default;

    virtual bool start_stream(std::string& error) = 0;
    virtual void stop_stream() = 0;
    virtual bool push_frame(const cv::Mat& frame, const VideoOverlay& overlay, std::string& error) = 0;
};

}  // namespace plane::video
```

- [x] Step 2: 写 `src/video/video_streaming.h`

```cpp
#pragma once

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core.hpp>

#include "core/config.h"
#include "core/module.h"
#include "video/h264_encoder.h"
#include "video/rtsp_media_server.h"
#include "video/video_streaming_api.h"

namespace plane::video {

/// 视频推流：最新帧槽 + H264 编码 + ZLMediaKit RTSP 输出（ARCHITECTURE §17）。
class VideoStreaming final : public plane::core::IModule, public VideoStreamingApi {
   public:
    explicit VideoStreaming(const plane::core::AppConfig& config);
    ~VideoStreaming() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    bool start_stream(std::string& error) override;
    void stop_stream() override;
    bool push_frame(const cv::Mat& frame, const VideoOverlay& overlay, std::string& error) override;

   private:
    void worker_loop();
    bool encode_and_send(const cv::Mat& frame, const VideoOverlay& overlay, std::string& error);

    const plane::core::AppConfig& config_;
    std::unique_ptr<H264Encoder> encoder_;
    RtspMediaServer server_;

    std::mutex slot_mutex_;
    std::condition_variable slot_cv_;
    cv::Mat latest_frame_;
    VideoOverlay latest_overlay_;
    bool has_frame_ = false;
    bool stream_running_ = false;

    std::thread worker_;
    std::int64_t frame_index_ = 0;
    std::int64_t last_encode_ms_ = 0;
    bool first_frame_logged_ = false;
};

}  // namespace plane::video
```

- [x] Step 3: 写 `src/video/video_streaming.cpp`

```cpp
#include "video/video_streaming.h"

#include <algorithm>
#include <chrono>

#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

#include "video/ffmpeg_h264_encoder.h"

namespace plane::video {
namespace {

std::int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace

VideoStreaming::VideoStreaming(const plane::core::AppConfig& config)
    : config_(config), encoder_(std::make_unique<FfmpegH264Encoder>()) {}

VideoStreaming::~VideoStreaming() {
    stop();
}

std::string VideoStreaming::name() const {
    return "Video Streaming";
}

bool VideoStreaming::init() {
    return true;
}

bool VideoStreaming::start() {
    std::string error;
    if (!start_stream(error)) {
        spdlog::error("推流启动失败: {}", error);
        return false;
    }
    return true;
}

void VideoStreaming::stop() {
    stop_stream();
}

void VideoStreaming::release() {}

bool VideoStreaming::start_stream(std::string& error) {
    if (stream_running_) {
        return true;
    }
    if (!server_.init(config_.rtsp_port, error)) {
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(slot_mutex_);
        stream_running_ = true;
    }
    worker_ = std::thread([this]() { worker_loop(); });
    spdlog::info("推流已启动: rtsp://<ip>:{} /{}/{}", config_.rtsp_port, config_.rtsp_app,
                 config_.rtsp_stream);
    return true;
}

void VideoStreaming::stop_stream() {
    {
        std::lock_guard<std::mutex> lock(slot_mutex_);
        if (!stream_running_) {
            return;
        }
        stream_running_ = false;
    }
    slot_cv_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
    server_.close();
    spdlog::info("推流已停止");
}

bool VideoStreaming::push_frame(const cv::Mat& frame, const VideoOverlay& overlay,
                                std::string&) {
    {
        std::lock_guard<std::mutex> lock(slot_mutex_);
        latest_frame_ = frame;
        latest_overlay_ = overlay;
        has_frame_ = true;
    }
    slot_cv_.notify_one();
    return true;
}

void VideoStreaming::worker_loop() {
    while (true) {
        cv::Mat frame;
        VideoOverlay overlay;
        {
            std::unique_lock<std::mutex> lock(slot_mutex_);
            slot_cv_.wait_for(lock, std::chrono::milliseconds(100),
                              [this]() { return has_frame_ || !stream_running_; });
            if (!stream_running_) {
                break;
            }
            if (!has_frame_) {
                continue;
            }
            frame = latest_frame_;
            overlay = latest_overlay_;
            has_frame_ = false;
        }
        const std::int64_t interval_ms = 1000 / std::max(1, config_.video_fps);
        const std::int64_t elapsed_ms = now_ms() - last_encode_ms_;
        if (elapsed_ms < interval_ms) {
            std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms - elapsed_ms));
        }
        last_encode_ms_ = now_ms();
        std::string error;
        if (!encode_and_send(frame, overlay, error)) {
            spdlog::warn("推流编码/发送失败: {}", error);
        }
    }
}

bool VideoStreaming::encode_and_send(const cv::Mat& frame, const VideoOverlay& overlay,
                                     std::string& error) {
    if (frame.empty()) {
        error = "空帧";
        return false;
    }
    if (!encoder_->init(frame.cols, frame.rows, config_.video_fps, config_.video_bitrate_kbps,
                        error)) {
        return false;
    }
    cv::Mat canvas = frame.clone();
    if (config_.video_overlay_enabled) {
        for (const auto& target : overlay.targets) {
            const auto& bbox = target.bbox;
            if (bbox.width > 0 && bbox.height > 0) {
                cv::rectangle(canvas, cv::Rect(bbox.x, bbox.y, bbox.width, bbox.height),
                              cv::Scalar(0, 255, 0), 2);
            }
        }
        if (overlay.has_tracked && overlay.tracked.bbox.width > 0 &&
            overlay.tracked.bbox.height > 0) {
            const auto& bbox = overlay.tracked.bbox;
            cv::rectangle(canvas, cv::Rect(bbox.x, bbox.y, bbox.width, bbox.height),
                          cv::Scalar(0, 0, 255), 2);
        }
    }
    std::vector<std::uint8_t> encoded;
    bool keyframe = false;
    if (!encoder_->encode(canvas, encoded, keyframe, error)) {
        return false;
    }
    if (!server_.ensure_media(config_.rtsp_app, config_.rtsp_stream, canvas.cols, canvas.rows,
                              config_.video_fps, config_.video_bitrate_kbps, error)) {
        return false;
    }
    const std::int64_t dts_ms = frame_index_ * 1000 / std::max(1, config_.video_fps);
    if (!server_.input_h264(encoded.data(), encoded.size(),
                            static_cast<std::uint64_t>(dts_ms))) {
        error = "喂流失败";
        return false;
    }
    if (!first_frame_logged_) {
        first_frame_logged_ = true;
        spdlog::info("推流首帧: {}x{}", canvas.cols, canvas.rows);
    }
    ++frame_index_;
    return true;
}

}  // namespace plane::video
```

- [x] Step 4: `CMakeLists.txt` 的 `add_executable(onboard ...)` 文件列表追加

```cmake
  src/video/ffmpeg_h264_encoder.cpp
  src/video/rtsp_media_server.cpp
  src/video/video_streaming.cpp
```

- [x] Step 5: 构建与格式

```bash
cmake --build build -j
clang-format --dry-run --Werror $(git ls-files src/video)
```

Expected: 构建通过且无 warning；格式通过（差异先 `-i` 再构建）。

- [x] Step 6: 提交

```bash
git add src/video CMakeLists.txt
git commit -m "feat(m8): 增加 VideoStreaming 推流模块"
```

---

## Task 5：Perception 集成推流

**Files**

- Modify: `src/perception/perception.h`
- Modify: `src/perception/perception.cpp`

- [x] Step 1: `perception.h` 修改

1) include 追加：

```cpp
#include "video/video_streaming_api.h"
```

2) 构造函数声明改为：

```cpp
    Perception(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
               PluginManagerApi& plugin_manager, plane::state::StateStoreApi& state_store,
               plane::video::VideoStreamingApi& video_streaming);
```

3) 成员区追加（`tracking_target_` 附近）：

```cpp
    plane::video::VideoStreamingApi& video_streaming_;
    std::vector<plane::model::DetectTarget> last_detection_targets_;
```

- [x] Step 2: `perception.cpp` 修改

1) 构造函数定义改为：

```cpp
Perception::Perception(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                       PluginManagerApi& plugin_manager, plane::state::StateStoreApi& state_store,
                       plane::video::VideoStreamingApi& video_streaming)
    : event_bus_(event_bus),
      config_(config),
      plugin_manager_(plugin_manager),
      state_store_(state_store),
      video_streaming_(video_streaming) {}
```

2) `process_loop()` 中 `if (!has_frame) { continue; }` 之后插入：

```cpp
        plane::video::VideoOverlay overlay;
        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            overlay.targets = last_detection_targets_;
            if (tracking_enabled_) {
                overlay.has_tracked = true;
                overlay.tracked = tracking_target_;
            }
        }
        std::string video_error;
        if (!video_streaming_.push_frame(frame, overlay, video_error)) {
            spdlog::warn("推流帧提交失败: {}", video_error);
        }
```

3) 检测成功分支保存最新检测结果：

将：

```cpp
                if (detection_->detect(frame, targets, error)) {
                    DetectionResult result;
                    result.targets = std::move(targets);
                    event_bus_.publish(events::kDetection, result);
                } else {
```

改为：

```cpp
                if (detection_->detect(frame, targets, error)) {
                    DetectionResult result;
                    result.targets = std::move(targets);
                    {
                        std::lock_guard<std::mutex> lock(target_mutex_);
                        last_detection_targets_ = result.targets;
                    }
                    event_bus_.publish(events::kDetection, result);
                } else {
```

- [x] Step 3: 构建与格式

```bash
cmake --build build -j
clang-format --dry-run --Werror src/perception/perception.h src/perception/perception.cpp
```

Expected: 构建通过且无 warning；格式通过（差异先 `-i` 再构建）。

- [x] Step 4: 提交

```bash
git add src/perception/perception.h src/perception/perception.cpp
git commit -m "feat(m8): Perception 集成推流帧交付"
```

---

## Task 6：main 接线

**Files**

- Modify: `src/main.cpp`

- [x] Step 1: include 追加

```cpp
#include "video/video_streaming.h"
```

- [x] Step 2: 创建与注册

在 `camera_control` 之后创建：

```cpp
    auto video_streaming = std::make_unique<plane::video::VideoStreaming>(config);
```

`perception` 构造改为：

```cpp
    auto perception = std::make_unique<plane::perception::Perception>(
        event_bus, config, *plugin_management, *state_store, *video_streaming);
```

注册顺序调整为：

```cpp
    runtime.add_module(std::move(state_store));
    runtime.add_module(std::move(flight_control));
    runtime.add_module(std::move(plugin_management));
    runtime.add_module(std::move(camera_control));
    runtime.add_module(std::move(video_streaming));
    runtime.add_module(std::move(perception));
    runtime.add_module(std::move(target_store));
    runtime.add_module(std::move(task_engine));
    runtime.add_module(std::move(task_scheduler));
    runtime.add_module(std::move(communication));
```

占位 skip 列表增加 `"Video Streaming"`。

- [x] Step 3: 构建、运行与格式

```bash
cmake --build build -j
timeout --preserve-status 3 ./build/onboard; echo "exit=$?"
clang-format --dry-run --Werror src/main.cpp
```

Expected: 构建通过；日志含 `ZLMediaKit 环境已初始化`、`推流已启动: rtsp://<ip>:8554 /live/onboard`；exit=0。

- [x] Step 4: 提交

```bash
git add src/main.cpp
git commit -m "feat(m8): 接线 VideoStreaming"
```

---

## Task 7：M8 部分验收

**Files**

- Modify: `docs/plans/m8-video-streaming-tasks.md`
- Modify: `docs/DEV_PLAN.md`

- [x] Step 1: 干净构建与运行

```bash
rm -rf build logs
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
timeout --preserve-status 3 ./build/onboard > /tmp/opencode/m8_run.log 2>&1; echo "exit=$?"
grep -E "ZLMediaKit 环境已初始化|推流已启动" /tmp/opencode/m8_run.log
ls -l logs/zlmediakit.log
```

Expected: 构建无 warning；两条日志出现；`logs/zlmediakit.log` 存在；exit=0。

- [x] Step 2: stdout 噪音检查

```bash
grep -icE "zlmedia|load codec" /tmp/opencode/m8_run.log
```

Expected: 0（codec 注册等 ZLMediaKit 日志不再打到 stdout）。若静态初始化仍有少量输出，记录实际行数作为偏差。

- [x] Step 3: 回归与规范

```bash
clang-format --dry-run --Werror $(git ls-files src tools plugins)
grep -rn "std::cout\|printf(" src/ tools/ plugins/ || echo "no cout/printf"
```

- [x] Step 4: 文档与收尾提交

- 勾选本文档并记录验收证据与偏差；
- 更新 `docs/DEV_PLAN.md` §11 索引：M8 → 已完成（部分验收，实流待用户提供源）。

```bash
git add docs
git commit -m "docs(m8): M8 部分验收与文档更新"
```

---

## 附：待用户补充验证（M8 交付）

用户提供图像源后（本地视频文件 / RTSP 输入 / MIPI）：

1. 日志 `推流首帧: WxH`；
2. 用 ffprobe/播放器拉 `rtsp://<ip>:8554/live/onboard` 成功；
3. `video_overlay_enabled` 开/关对比；
4. 停止推流后 Perception 取图/检测仍正常（`stop_stream` 路径）。

---

## 执行记录与偏差

1. **验收证据（2026-09-18，无图像源）**
   - 干净构建无 warning；运行 exit=0；
   - `ZLMediaKit 环境已初始化: rtsp_port=8554, 日志=logs/zlmediakit.log`、`推流已启动: rtsp://<ip>:8554 /live/onboard`；
   - 无源时编码线程等待帧，进程稳定（无崩溃/无高频日志）。
2. **偏差**
   - ZLMediaKit 的 `log_file_path` 按目录处理：实际文件为 `logs/zlmediakit.log/2026-09-18_00.log`（当天为空，因级别 warn 且无告警）；
   - 库静态初始化在 `mk_env_init` 之前仍向 stdout 打印 14 行 `registerPlugin | Load codec: ...`（链接加载期产生，配置无法抑制）；`mk_env_init2` 之后的 ZLM 日志按 mask 仅写文件。若需彻底静默，需评估延迟加载（dlopen）方案。
3. `clang-format` 通过；无 `std::cout/printf`；M1–M7 回归正常。
4. **待用户验证（提供源后）**：`推流首帧: WxH`、ffprobe/播放器拉流、叠加开关对比、停止推流不影响感知。
