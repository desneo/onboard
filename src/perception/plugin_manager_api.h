#pragma once

#include <memory>

#include "functional/det_mot_api.h"
#include "functional/image_input_api.h"
#include "functional/videostream_output_api.h"

namespace plane::perception {

/// 插件按需实例化接口（ARCHITECTURE §19）。
class PluginManagerApi {
   public:
    virtual ~PluginManagerApi() = default;

    virtual std::shared_ptr<plane::algorithm::ImageInputPlugin> create_image_input() = 0;
    virtual std::shared_ptr<plane::algorithm::DetMotPlugin> create_det_mot() = 0;
    virtual std::shared_ptr<plane::algorithm::VideoStreamOutputPlugin> create_video_output() = 0;
};

}  // namespace plane::perception
