#pragma once

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <string>
#include <thread>

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
