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
    bool ensure_media(const std::string& app, const std::string& stream, int width, int height, int fps,
                      int bitrate_kbps, std::string& error);
    bool input_h264(const std::uint8_t* data, std::size_t size, std::uint64_t dts_ms);
    void close();

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace plane::video
