#include "video/video_streaming.h"

#include <spdlog/spdlog.h>

#include <string>

#include "core/runtime.h"

namespace plane::video {
namespace {

plane::algorithm::StreamType parse_stream_type(const std::string& value) {
    if (value == "Rtsp") {
        return plane::algorithm::StreamType::Rtsp;
    }
    return plane::algorithm::StreamType::WebRtc;
}

const char* stream_type_name(plane::algorithm::StreamType type) {
    return type == plane::algorithm::StreamType::Rtsp ? "Rtsp" : "WebRtc";
}

}  // namespace

VideoStreaming::VideoStreaming(const plane::core::AppConfig& config,
                               plane::perception::PluginManagerApi& plugin_manager)
    : config_(config), plugin_manager_(plugin_manager) {}

VideoStreaming::~VideoStreaming() { stop(); }

std::string VideoStreaming::name() const { return "Video Streaming"; }

bool VideoStreaming::init() { return true; }

bool VideoStreaming::start() {
    std::string error;
    if (!start_stream(error)) {
        SPDLOG_ERROR("推流启动失败: {}", error);
        return false;
    }
    return true;
}

void VideoStreaming::stop() { stop_stream(); }

void VideoStreaming::release() {}

plane::algorithm::StreamConfig VideoStreaming::make_stream_config() const {
    plane::algorithm::StreamConfig stream;
    stream.type = parse_stream_type(config_.stream_type);
    stream.url = config_.stream_url;
    stream.app = config_.stream_app;
    stream.stream = config_.stream_name;
    stream.rtsp_port = config_.stream_rtsp_port;
    stream.http_port = config_.stream_http_port;
    stream.rtc_port = config_.stream_rtc_port;
    stream.width = config_.stream_width;
    stream.height = config_.stream_height;
    stream.max_fps = config_.stream_max_fps;
    stream.bitrate_kbps = config_.stream_bitrate_kbps;
    stream.gop = config_.stream_gop;
    return stream;
}

bool VideoStreaming::start_stream(std::string& error) {
    if (output_) {
        return true;
    }
    output_ = plugin_manager_.create_video_output();
    if (!output_) {
        error = "无可用推流插件 (VideoStreamOutputPlugin)";
        plane::core::request_fatal("videostream_output", error);
        return false;
    }
    const plane::algorithm::StreamConfig stream = make_stream_config();
    if (!output_->setConfig(stream, error)) {
        plane::core::request_fatal("videostream_output", error);
        output_.reset();
        return false;
    }
    SPDLOG_INFO(
        "[plugin:videostream_output] type={} app={} stream={} rtsp_port={} http_port={} rtc_port={} size={}x{} "
        "max_fps={} bitrate_kbps={} gop={}",
        stream_type_name(stream.type), stream.app, stream.stream, stream.rtsp_port, stream.http_port, stream.rtc_port,
        stream.width, stream.height, stream.max_fps, stream.bitrate_kbps, stream.gop);
    SPDLOG_INFO("推流地址={}", output_->streamUrl());
    return true;
}

void VideoStreaming::stop_stream() {
    if (!output_) {
        return;
    }
    output_->stop();
    output_.reset();
    SPDLOG_INFO("推流已停止");
}

bool VideoStreaming::push_frame(const cv::Mat& frame, const std::vector<VisionTarget>& targets, std::string& error) {
    if (!output_) {
        error = "推流未启动";
        return false;
    }
    return output_->putImage(frame, targets, plane::algorithm::OverlayKind::Detection, error);
}

plane::algorithm::StreamStatus VideoStreaming::status() const {
    if (!output_) {
        return {};
    }
    return output_->status();
}

std::string VideoStreaming::stream_url() const {
    if (!output_) {
        return {};
    }
    return output_->streamUrl();
}

}  // namespace plane::video
