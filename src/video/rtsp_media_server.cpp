#include "video/rtsp_media_server.h"

#include <spdlog/spdlog.h>

#include <string>
#include <utility>

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

RtspMediaServer::~RtspMediaServer() { close(); }

bool RtspMediaServer::init(int rtsp_port, std::string&) {
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

bool RtspMediaServer::ensure_media(const std::string& app, const std::string& stream, int width, int height, int fps,
                                   int bitrate_kbps, std::string& error) {
    if (impl_->media != nullptr && width == impl_->width && height == impl_->height && fps == impl_->fps) {
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
    spdlog::info("RTSP 流已创建: rtsp://<ip>:<port>/{}/{} ({}x{}@{}fps, {}kbps)", app, stream, width, height, fps,
                 bitrate_kbps);
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
