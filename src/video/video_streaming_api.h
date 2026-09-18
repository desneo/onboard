#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

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
