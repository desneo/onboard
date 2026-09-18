#include "video/video_streaming.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <opencv2/imgproc.hpp>

#include "video/ffmpeg_h264_encoder.h"

namespace plane::video {
namespace {

std::int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace

VideoStreaming::VideoStreaming(const plane::core::AppConfig& config)
    : config_(config), encoder_(std::make_unique<FfmpegH264Encoder>()) {}

VideoStreaming::~VideoStreaming() { stop(); }

std::string VideoStreaming::name() const { return "Video Streaming"; }

bool VideoStreaming::init() { return true; }

bool VideoStreaming::start() {
    std::string error;
    if (!start_stream(error)) {
        spdlog::error("推流启动失败: {}", error);
        return false;
    }
    return true;
}

void VideoStreaming::stop() { stop_stream(); }

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
    spdlog::info("推流已启动: rtsp://<ip>:{} /{}/{}", config_.rtsp_port, config_.rtsp_app, config_.rtsp_stream);
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

bool VideoStreaming::push_frame(const cv::Mat& frame, const VideoOverlay& overlay, std::string&) {
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

bool VideoStreaming::encode_and_send(const cv::Mat& frame, const VideoOverlay& overlay, std::string& error) {
    if (frame.empty()) {
        error = "空帧";
        return false;
    }
    if (!encoder_->init(frame.cols, frame.rows, config_.video_fps, config_.video_bitrate_kbps, error)) {
        return false;
    }
    cv::Mat canvas = frame.clone();
    if (config_.video_overlay_enabled) {
        for (const auto& target : overlay.targets) {
            const auto& bbox = target.bbox;
            if (bbox.width > 0 && bbox.height > 0) {
                cv::rectangle(canvas, cv::Rect(bbox.x, bbox.y, bbox.width, bbox.height), cv::Scalar(0, 255, 0), 2);
            }
        }
        if (overlay.has_tracked && overlay.tracked.bbox.width > 0 && overlay.tracked.bbox.height > 0) {
            const auto& bbox = overlay.tracked.bbox;
            cv::rectangle(canvas, cv::Rect(bbox.x, bbox.y, bbox.width, bbox.height), cv::Scalar(0, 0, 255), 2);
        }
    }
    std::vector<std::uint8_t> encoded;
    bool keyframe = false;
    if (!encoder_->encode(canvas, encoded, keyframe, error)) {
        return false;
    }
    if (!server_.ensure_media(config_.rtsp_app, config_.rtsp_stream, canvas.cols, canvas.rows, config_.video_fps,
                              config_.video_bitrate_kbps, error)) {
        return false;
    }
    const std::int64_t dts_ms = frame_index_ * 1000 / std::max(1, config_.video_fps);
    if (!server_.input_h264(encoded.data(), encoded.size(), static_cast<std::uint64_t>(dts_ms))) {
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
