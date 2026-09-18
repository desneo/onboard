#pragma once

#include <cstdint>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

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
