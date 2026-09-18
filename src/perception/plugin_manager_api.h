#pragma once

#include <memory>

#include "functional/detection_api.h"
#include "functional/guidance_api.h"
#include "functional/tracking_api.h"

namespace plane::perception {

/// 插件按需实例化接口（ARCHITECTURE §19）。
class PluginManagerApi {
   public:
    virtual ~PluginManagerApi() = default;

    virtual std::shared_ptr<plane::algorithm::DetectionPlugin> create_detection() = 0;
    virtual std::shared_ptr<plane::algorithm::TrackingPlugin> create_tracking() = 0;
    virtual std::shared_ptr<plane::algorithm::GuidancePlugin> create_guidance() = 0;
};

}  // namespace plane::perception
