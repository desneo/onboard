#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "functional/videostream_output_api.h"
#include "message.h"

namespace plane::video {

using plane::model::VisionTarget;

/// 推流对外接口（ARCHITECTURE §17）。
class VideoStreamingApi {
   public:
    virtual ~VideoStreamingApi() = default;

    virtual bool start_stream(std::string& error) = 0;
    virtual void stop_stream() = 0;
    virtual bool push_frame(const cv::Mat& frame, const std::vector<VisionTarget>& targets, std::string& error) = 0;

    /// 当前推流状态（State/fps/观看人数）；插件不可用时返回默认状态。
    virtual plane::algorithm::StreamStatus status() const = 0;

    /// 当前推流的完整地址（浏览器可用 WebRTC 地址或 RTSP 拉流地址）。
    virtual std::string stream_url() const = 0;
};

}  // namespace plane::video
