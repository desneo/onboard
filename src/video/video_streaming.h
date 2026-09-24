#pragma once

#include <memory>
#include <string>

#include "core/config.h"
#include "core/module.h"
#include "perception/plugin_manager_api.h"
#include "video/video_streaming_api.h"

namespace plane::video {

/// 视频推流：由正式推流插件（VideoStreamOutputPlugin）承接编码/服务/叠加（ARCHITECTURE §17）。
class VideoStreaming final : public plane::core::IModule, public VideoStreamingApi {
   public:
    VideoStreaming(const plane::core::AppConfig& config, plane::perception::PluginManagerApi& plugin_manager);
    ~VideoStreaming() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    bool start_stream(std::string& error) override;
    void stop_stream() override;
    bool push_frame(const cv::Mat& frame, const std::vector<VisionTarget>& targets, std::string& error) override;
    plane::algorithm::StreamStatus status() const override;
    std::string stream_url() const override;

   private:
    plane::algorithm::StreamConfig make_stream_config() const;

    const plane::core::AppConfig& config_;
    plane::perception::PluginManagerApi& plugin_manager_;
    std::shared_ptr<plane::algorithm::VideoStreamOutputPlugin> output_;
};

}  // namespace plane::video
