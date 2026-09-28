#include "perception/perception.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <magic_enum/magic_enum.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <utility>
#include <vector>

#if defined(__aarch64__)
#include <rga/im2d.h>
#include <rga/rga.h>
#endif

#include "core/runtime.h"
#include "core/time_utils.h"
#include "perception/perception_events.h"

namespace plane::perception {

using plane::core::now_ms;

namespace {

#if defined(__aarch64__)
/// 用 RK3588 RGA 硬件把 BGR 帧缩放到目标分辨率；成功返回 true（失败时由调用方回退 CPU）。
bool rga_resize_bgr(const cv::Mat& src, cv::Mat& dst, int width, int height) {
    if (!src.isContinuous()) {
        return false;
    }
    dst.create(height, width, src.type());
    if (!dst.isContinuous()) {
        return false;
    }
    rga_buffer_t src_buf = wrapbuffer_virtualaddr(static_cast<void*>(src.data), src.cols, src.rows, RK_FORMAT_BGR_888);
    rga_buffer_t dst_buf = wrapbuffer_virtualaddr(static_cast<void*>(dst.data), width, height, RK_FORMAT_BGR_888);
    return imresize(src_buf, dst_buf) == IM_STATUS_SUCCESS;
}
#endif

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
    source.type = magic_enum::enum_cast<plane::algorithm::ImageSourceType>(config_.image_source_type)
                      .value_or(plane::algorithm::ImageSourceType::Mipi);
    source.mipi_device = config_.image_mipi_device;
    source.rtsp_url = config_.image_rtsp_url;
    source.reconnect_interval_s = config_.image_reconnect_interval_s;
    source.width = config_.image_width;
    source.height = config_.image_height;
    if (!image_input_->setConfig(source, error)) {
        SPDLOG_ERROR("取图插件 setConfig 失败: {}", error);
        image_input_.reset();
        return false;
    }
    SPDLOG_INFO("[plugin:image_input] type={} device={} rtsp={} reconnect_s={} size={}x{}",
                magic_enum::enum_name(source.type), source.mipi_device.empty() ? "auto" : source.mipi_device,
                source.rtsp_url, source.reconnect_interval_s, source.width, source.height);
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
            SPDLOG_INFO("首帧 size={}x{}, 检测输入={}x{}", frame->cols, frame->rows, config_.det_input_width,
                        config_.det_input_height);
        }
        // 读流后先统一到检测模型输入分辨率（优先 RGA 硬件），检测与推流均使用该帧。
        const int target_w = config_.det_input_width;
        const int target_h = config_.det_input_height;
        if (target_w > 0 && target_h > 0 && (frame->cols != target_w || frame->rows != target_h)) {
            resize_src_w_ = frame->cols;
            resize_src_h_ = frame->rows;
            auto resized = std::make_shared<cv::Mat>();
            bool resized_by_rga = false;
            const auto resize_begin = std::chrono::steady_clock::now();
#if defined(__aarch64__)
            resized_by_rga = rga_resize_bgr(*frame, *resized, target_w, target_h);
#endif
            if (!resized_by_rga) {
                cv::resize(*frame, *resized, cv::Size(target_w, target_h));
            }
            const auto resize_end = std::chrono::steady_clock::now();
            if (!resize_backend_logged_) {
                resize_backend_logged_ = true;
                SPDLOG_INFO("分辨率转换后端: {}", resized_by_rga ? "RGA(硬件)" : "OpenCV(CPU)");
            }
            resize_total_us_ +=
                std::chrono::duration_cast<std::chrono::microseconds>(resize_end - resize_begin).count();
            ++resize_count_;
            const std::int64_t now = now_ms();
            if (last_resize_log_ms_ == 0) {
                last_resize_log_ms_ = now;
            }
            if (resize_count_ >= 120 || now - last_resize_log_ms_ >= 2000) {
                SPDLOG_INFO("分辨率转换耗时: {}x{} -> {}x{}, avg={:.0f}us (n={})", resize_src_w_, resize_src_h_,
                            target_w, target_h,
                            static_cast<double>(resize_total_us_) / static_cast<double>(resize_count_), resize_count_);
                resize_total_us_ = 0;
                resize_count_ = 0;
                last_resize_log_ms_ = now;
            }
            frame = std::move(resized);
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
        if (now - last_status_log_ms_ >= 20000) {
            last_status_log_ms_ = now;
            const plane::algorithm::StreamStatus status = video_streaming_.status();
            plane::algorithm::SourceStatus source_status = plane::algorithm::SourceStatus::Idle;
            {
                std::lock_guard<std::mutex> lock(plugin_mutex_);
                if (image_input_) {
                    source_status = image_input_->status();
                }
            }
            SPDLOG_INFO("推流状态 state={} fps={:.1f} viewers={} source={}", magic_enum::enum_name(status.state),
                        status.current_fps, status.viewer_count, magic_enum::enum_name(source_status));
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
