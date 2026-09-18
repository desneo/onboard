#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "message.h"

namespace plane::perception {

struct DetectionResult {
    std::vector<plane::model::DetectTarget> targets;
};

struct TrackingResult {
    plane::model::DetectTarget target;
    bool lost = false;
};

struct GuidanceResult {
    plane::model::NedVelocity velocity;
    bool valid = false;
    bool applied = false;  ///< 是否已成功下发飞控（M10 直连）
};

struct PerceptionError {
    std::string detail;
};

/// 感知业务级控制接口（ARCHITECTURE §15.3）。
class PerceptionApi {
   public:
    virtual ~PerceptionApi() = default;

    virtual bool start_detection(std::string& error) = 0;
    virtual void stop_detection() = 0;
    virtual bool select_target(const plane::model::DetectTarget& target, std::string& error) = 0;
    virtual void stop_tracking() = 0;
    virtual bool start_guidance(std::string& error) = 0;
    virtual void stop_guidance() = 0;
    virtual bool latest_frame(cv::Mat& out) const = 0;
};

}  // namespace plane::perception
