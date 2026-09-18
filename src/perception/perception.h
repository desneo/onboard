#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <string>
#include <thread>

#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"
#include "flight_control/flight_control_api.h"
#include "functional/detection_api.h"
#include "functional/guidance_api.h"
#include "functional/tracking_api.h"
#include "perception/perception_api.h"
#include "perception/plugin_manager_api.h"
#include "state/state_store_api.h"
#include "video/video_streaming_api.h"

namespace cv {
class VideoCapture;
}

namespace plane::perception {

/// MIPI/RTSP 图像输入、算法插件调度与结果发布（ARCHITECTURE §15）。
class Perception final : public plane::core::IModule, public PerceptionApi {
   public:
    Perception(plane::core::EventBus& event_bus, const plane::core::AppConfig& config, PluginManagerApi& plugin_manager,
               plane::state::StateStoreApi& state_store, plane::flight_control::FlightControlApi& flight_control,
               plane::video::VideoStreamingApi& video_streaming);
    ~Perception() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    bool start_detection(std::string& error) override;
    void stop_detection() override;
    bool select_target(const plane::model::DetectTarget& target, std::string& error) override;
    void stop_tracking() override;
    bool start_guidance(std::string& error) override;
    void stop_guidance() override;
    bool latest_frame(cv::Mat& out) const override;

   private:
    void capture_loop();
    void process_loop();
    bool open_source();
    bool ensure_detection(std::string& error);
    bool ensure_tracking(std::string& error);
    bool ensure_guidance(std::string& error);
    void run_selftest(const cv::Mat& frame);
    void report_error(const std::string& detail) const;
    bool latest_frame_locked(cv::Mat& out) const;

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    PluginManagerApi& plugin_manager_;
    plane::state::StateStoreApi& state_store_;
    plane::flight_control::FlightControlApi& flight_control_;

    std::unique_ptr<cv::VideoCapture> capture_;
    mutable std::mutex frame_mutex_;
    cv::Mat latest_frame_;

    std::mutex plugin_mutex_;
    std::shared_ptr<plane::algorithm::DetectionPlugin> detection_;
    std::shared_ptr<plane::algorithm::TrackingPlugin> tracking_;
    std::shared_ptr<plane::algorithm::GuidancePlugin> guidance_;

    std::atomic<bool> detection_enabled_{false};
    std::atomic<bool> tracking_enabled_{false};
    std::atomic<bool> guidance_enabled_{false};
    std::mutex target_mutex_;
    plane::model::DetectTarget tracking_target_;
    plane::video::VideoStreamingApi& video_streaming_;
    std::vector<plane::model::DetectTarget> last_detection_targets_;

    std::thread capture_thread_;
    std::thread process_thread_;
    std::atomic<bool> running_{false};
    bool selftest_done_ = false;
};

}  // namespace plane::perception
