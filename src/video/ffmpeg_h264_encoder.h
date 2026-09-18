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
    bool encode(const cv::Mat& bgr_frame, std::vector<std::uint8_t>& out, bool& keyframe, std::string& error) override;

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
