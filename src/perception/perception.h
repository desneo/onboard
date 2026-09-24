#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <string>
#include <thread>
#include <vector>

#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"
#include "functional/det_mot_api.h"
#include "functional/image_input_api.h"
#include "perception/perception_api.h"
#include "perception/plugin_manager_api.h"
#include "video/video_streaming_api.h"

namespace plane::perception {

/// 图像输入（插件）→ 目标检测（插件）→ 推流（插件）调度与结果发布（ARCHITECTURE §15）。
class Perception final : public plane::core::IModule, public PerceptionApi {
   public:
    Perception(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
               PluginManagerApi& plugin_manager, plane::video::VideoStreamingApi& video_streaming);
    ~Perception() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    bool start_detection(std::string& error) override;
    void stop_detection() override;
    bool latest_frame(cv::Mat& out) const override;

   private:
    bool ensure_image_input(std::string& error);
    bool ensure_detection(std::string& error);
    void capture_loop();
    void process_loop();
    void report_error(const std::string& detail) const;
    std::shared_ptr<cv::Mat> latest_frame_shared() const;

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    PluginManagerApi& plugin_manager_;
    plane::video::VideoStreamingApi& video_streaming_;

    mutable std::mutex plugin_mutex_;
    std::shared_ptr<plane::algorithm::ImageInputPlugin> image_input_;
    std::shared_ptr<plane::algorithm::DetMotPlugin> detection_;

    mutable std::mutex frame_mutex_;
    std::shared_ptr<cv::Mat> latest_frame_;

    std::atomic<bool> detection_enabled_{false};
    std::mutex target_mutex_;
    std::vector<plane::model::VisionTarget> last_detection_targets_;

    std::thread capture_thread_;
    std::thread process_thread_;
    std::atomic<bool> running_{false};
    std::int64_t last_detection_log_ms_ = 0;
    std::int64_t last_status_log_ms_ = 0;
    bool first_frame_logged_ = false;
};

}  // namespace plane::perception
