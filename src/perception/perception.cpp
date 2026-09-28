#include "perception/perception.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <string>
#include <utility>
#include <vector>

#include "core/runtime.h"
#include "perception/perception_events.h"

namespace plane::perception {
namespace {

std::int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

plane::algorithm::ImageSourceType parse_source_type(const std::string& value) {
    if (value == "Rtsp") {
        return plane::algorithm::ImageSourceType::Rtsp;
    }
    return plane::algorithm::ImageSourceType::Mipi;
}

const char* source_type_name(plane::algorithm::ImageSourceType type) {
    return type == plane::algorithm::ImageSourceType::Rtsp ? "Rtsp" : "Mipi";
}

const char* source_status_name(plane::algorithm::SourceStatus status) {
    switch (status) {
        case plane::algorithm::SourceStatus::Idle:
            return "Idle";
        case plane::algorithm::SourceStatus::Connecting:
            return "Connecting";
        case plane::algorithm::SourceStatus::Streaming:
            return "Streaming";
        case plane::algorithm::SourceStatus::Stopped:
            return "Stopped";
    }
    return "Unknown";
}

const char* stream_state_name(plane::algorithm::StreamState state) {
    switch (state) {
        case plane::algorithm::StreamState::Idle:
            return "Idle";
        case plane::algorithm::StreamState::Ready:
            return "Ready";
        case plane::algorithm::StreamState::Streaming:
            return "Streaming";
        case plane::algorithm::StreamState::Stopped:
            return "Stopped";
    }
    return "Unknown";
}

}  // namespace

Perception::Perception(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                       PluginManagerApi& plugin_manager, plane::video::VideoStreamingApi& video_streaming)
    : event_bus_(event_bus),
      config_(config),
      plugin_manager_(plugin_manager),
      video_streaming_(video_streaming) {}

Perception::~Perception() { stop(); }

std::string Perception::name() const { return "Perception"; }

bool Perception::init() { return true; }

bool Perception::start() {
    std::string error;
    if (!ensure_image_input(error)) {
        plane::core::request_fatal("image_input", error);
        return false;
    }
    if (!ensure_detection(error)) {
        plane::core::request_fatal("det_mot", error);
        return false;
    }
    detection_enabled_ = true;
    running_ = true;
    capture_thread_ = std::thread([this]() { capture_loop(); });
    process_thread_ = std::thread([this]() { process_loop(); });
    return true;
}

void Perception::stop() {
    running_ = false;
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    if (process_thread_.joinable()) {
        process_thread_.join();
    }
    std::lock_guard<std::mutex> lock(plugin_mutex_);
    if (image_input_) {
        image_input_->stop();
        image_input_.reset();
    }
    detection_.reset();
}

void Perception::release() {}

bool Perception::start_detection(std::string& error) {
    std::lock_guard<std::mutex> lock(plugin_mutex_);
    if (!detection_) {
        error = "检测插件未初始化";
        return false;
    }
    detection_enabled_ = true;
    return true;
}

void Perception::stop_detection() { detection_enabled_ = false; }

bool Perception::latest_frame(cv::Mat& out) const {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!latest_frame_ || latest_frame_->empty()) {
        return false;
    }
    out = *latest_frame_;
    return true;
}

std::shared_ptr<cv::Mat> Perception::latest_frame_shared() const {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return latest_frame_;
}

bool Perception::ensure_image_input(std::string& error) {
    std::lock_guard<std::mutex> lock(plugin_mutex_);
    if (image_input_) {
        return true;
    }
    image_input_ = plugin_manager_.create_image_input();
    if (!image_input_) {
        error = "无可用取图插件 (ImageInputPlugin)";
        return false;
    }
    plane::algorithm::ImageSourceConfig source;
    source.type = parse_source_type(config_.image_source_type);
    source.mipi_device = config_.image_mipi_device;
    source.rtsp_url = config_.image_rtsp_url;
    source.reconnect_interval_s = config_.image_reconnect_interval_s;
    if (!image_input_->setConfig(source, error)) {
        SPDLOG_ERROR("取图插件 setConfig 失败: {}", error);
        image_input_.reset();
        return false;
    }
    SPDLOG_INFO("[plugin:image_input] type={} device={} rtsp={} reconnect_s={}", source_type_name(source.type),
                source.mipi_device.empty() ? "auto" : source.mipi_device, source.rtsp_url,
                source.reconnect_interval_s);
    return true;
}

bool Perception::ensure_detection(std::string& error) {
    std::lock_guard<std::mutex> lock(plugin_mutex_);
    if (detection_) {
        return true;
    }
    detection_ = plugin_manager_.create_det_mot();
    if (!detection_) {
        error = "无可用检测插件 (DetMotPlugin)";
        return false;
    }
    plane::algorithm::DetMotConfig detect_config;
    detect_config.model_path = config_.det_model_path;
    detect_config.config_dir = config_.det_config_dir;
    detect_config.confidence_threshold = config_.det_confidence_threshold;
    detect_config.nms_threshold = config_.det_nms_threshold;
    detect_config.track_thresh = config_.det_track_thresh;
    detect_config.track_buffer = config_.det_track_buffer;
    if (!detection_->setConfig(detect_config)) {
        error = "检测插件 setConfig 失败 (model_path=" + detect_config.model_path +
                ", config_dir=" + detect_config.config_dir + ")";
        SPDLOG_ERROR("{}", error);
        detection_.reset();
        return false;
    }
    SPDLOG_INFO("[plugin:det_mot] model_path={} config_dir={} conf={} nms={} track_thresh={} track_buffer={}",
                detect_config.model_path, detect_config.config_dir, detect_config.confidence_threshold,
                detect_config.nms_threshold, detect_config.track_thresh, detect_config.track_buffer);
    return true;
}

void Perception::capture_loop() {
    while (running_) {
        std::shared_ptr<plane::algorithm::ImageInputPlugin> input;
        {
            std::lock_guard<std::mutex> lock(plugin_mutex_);
            input = image_input_;
        }
        if (!input) {
            break;
        }
        plane::algorithm::FrameMeta meta;
        std::shared_ptr<cv::Mat> frame = input->getFrame(&meta);
        if (!frame || frame->empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        if (!first_frame_logged_) {
            first_frame_logged_ = true;
            SPDLOG_INFO("首帧 size={}x{}", frame->cols, frame->rows);
        }
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_frame_ = std::move(frame);
    }
}

void Perception::process_loop() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        std::shared_ptr<cv::Mat> frame = latest_frame_shared();
        if (!frame || frame->empty()) {
            continue;
        }

        std::shared_ptr<plane::algorithm::DetMotPlugin> detector;
        bool detection_enabled = false;
        {
            std::lock_guard<std::mutex> lock(plugin_mutex_);
            detector = detection_;
            detection_enabled = detection_enabled_;
        }

        std::vector<plane::model::VisionTarget> targets;
        if (detection_enabled) {
            if (!detector) {
                plane::core::request_fatal("det_mot", "检测插件未初始化");
                return;
            }
            std::string error;
            if (!detector->detMotExe(frame, targets, error)) {
                plane::core::request_fatal("det_mot", error);
                return;
            }
            {
                std::lock_guard<std::mutex> lock(target_mutex_);
                last_detection_targets_ = targets;
            }
            if(config_.debug_target_log){
                SPDLOG_INFO("检测 targets={}", targets.size());
            }
            DetectionResult result;
            result.targets = targets;
            event_bus_.publish(events::kDetection, result);
        }

        std::string video_error;
        if (!video_streaming_.push_frame(*frame, targets, video_error)) {
            plane::core::request_fatal("videostream_output", video_error);
            return;
        }

        const std::int64_t now = now_ms();
        if (now - last_status_log_ms_ >= 2000) {
            last_status_log_ms_ = now;
            const plane::algorithm::StreamStatus status = video_streaming_.status();
            plane::algorithm::SourceStatus source_status = plane::algorithm::SourceStatus::Idle;
            {
                std::lock_guard<std::mutex> lock(plugin_mutex_);
                if (image_input_) {
                    source_status = image_input_->status();
                }
            }
            SPDLOG_INFO("推流状态 state={} fps={:.1f} viewers={} source={}", stream_state_name(status.state),
                        status.current_fps, status.viewer_count, source_status_name(source_status));
        }
    }
}

void Perception::report_error(const std::string& detail) const {
    SPDLOG_ERROR("Perception: {}", detail);
    PerceptionError event;
    event.detail = detail;
    event_bus_.publish(events::kError, event);
}

}  // namespace plane::perception
