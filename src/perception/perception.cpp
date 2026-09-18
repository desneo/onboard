#include "perception/perception.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <opencv2/videoio.hpp>
#include <string>
#include <utility>
#include <vector>

#include "perception/perception_events.h"

namespace plane::perception {
namespace {

bool is_numeric(const std::string& text) {
    return !text.empty() && std::all_of(text.begin(), text.end(), [](unsigned char c) { return std::isdigit(c) != 0; });
}

}  // namespace

Perception::Perception(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                       PluginManagerApi& plugin_manager, plane::state::StateStoreApi& state_store)
    : event_bus_(event_bus), config_(config), plugin_manager_(plugin_manager), state_store_(state_store) {}

Perception::~Perception() { stop(); }

std::string Perception::name() const { return "Perception"; }

bool Perception::init() { return true; }

bool Perception::start() {
    running_ = true;
    capture_thread_ = std::thread([this]() { capture_loop(); });
    process_thread_ = std::thread([this]() { process_loop(); });
    return true;
}

void Perception::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    if (process_thread_.joinable()) {
        process_thread_.join();
    }
    if (capture_) {
        capture_->release();
        capture_.reset();
    }
    {
        std::lock_guard<std::mutex> lock(plugin_mutex_);
        detection_.reset();
        tracking_.reset();
        guidance_.reset();
    }
}

void Perception::release() {}

bool Perception::start_detection(std::string& error) {
    if (!ensure_detection(error)) {
        report_error("启动检测失败: " + error);
        return false;
    }
    detection_enabled_ = true;
    return true;
}

void Perception::stop_detection() { detection_enabled_ = false; }

bool Perception::select_target(const plane::model::DetectTarget& target, std::string& error) {
    cv::Mat frame;
    if (!latest_frame_locked(frame)) {
        error = "当前无图像帧";
        return false;
    }
    if (!ensure_tracking(error)) {
        report_error("启动跟踪失败: " + error);
        return false;
    }
    if (!tracking_->start(frame, target, error)) {
        report_error("建立跟踪失败: " + error);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        tracking_target_ = target;
    }
    tracking_enabled_ = true;
    return true;
}

void Perception::stop_tracking() { tracking_enabled_ = false; }

bool Perception::start_guidance(std::string& error) {
    if (!ensure_guidance(error)) {
        report_error("启动导引失败: " + error);
        return false;
    }
    guidance_enabled_ = true;
    return true;
}

void Perception::stop_guidance() { guidance_enabled_ = false; }

bool Perception::latest_frame(cv::Mat& out) const { return latest_frame_locked(out); }

bool Perception::latest_frame_locked(cv::Mat& out) const {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (latest_frame_.empty()) {
        return false;
    }
    out = latest_frame_;
    return true;
}

bool Perception::open_source() {
    capture_ = std::make_unique<cv::VideoCapture>();
    const std::string& source = config_.image_source;
    bool opened = false;
    if (is_numeric(source)) {
        opened = capture_->open(std::stoi(source));
    } else {
        opened = capture_->open(source, cv::CAP_ANY);
    }
    return opened && capture_->isOpened();
}

void Perception::capture_loop() {
    while (running_) {
        if (config_.image_source.empty()) {
            spdlog::warn("图像源未配置 (image_source 为空)");
            std::this_thread::sleep_for(std::chrono::milliseconds(config_.image_reconnect_interval_ms));
            continue;
        }
        if (!open_source()) {
            report_error("图像源打开失败: " + config_.image_source);
            std::this_thread::sleep_for(std::chrono::milliseconds(config_.image_reconnect_interval_ms));
            continue;
        }
        spdlog::info("图像源已打开: {}", config_.image_source);
        bool first_frame = true;
        while (running_) {
            cv::Mat frame;
            if (!capture_->read(frame) || frame.empty()) {
                spdlog::warn("图像读取失败，尝试重连: {}", config_.image_source);
                break;
            }
            if (first_frame) {
                first_frame = false;
                spdlog::info("首帧: {}x{}", frame.cols, frame.rows);
            }
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = frame;
        }
        capture_->release();
    }
}

void Perception::process_loop() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(config_.perception_process_interval_ms));
        cv::Mat frame;
        const bool has_frame = latest_frame_locked(frame);
        if (config_.perception_selftest_on_start && !selftest_done_) {
            selftest_done_ = true;
            const cv::Mat selftest_frame = has_frame ? frame : cv::Mat::zeros(480, 640, CV_8UC3);
            run_selftest(selftest_frame);
        }
        if (!has_frame) {
            continue;
        }
        if (detection_enabled_) {
            std::string error;
            if (ensure_detection(error)) {
                std::vector<plane::model::DetectTarget> targets;
                if (detection_->detect(frame, targets, error)) {
                    DetectionResult result;
                    result.targets = std::move(targets);
                    event_bus_.publish(events::kDetection, result);
                } else {
                    report_error("检测失败: " + error);
                }
            }
        }
        if (tracking_enabled_) {
            std::string error;
            if (ensure_tracking(error)) {
                plane::model::DetectTarget target;
                if (tracking_->track(frame, target, error)) {
                    TrackingResult result;
                    result.target = target;
                    result.lost = false;
                    {
                        std::lock_guard<std::mutex> lock(target_mutex_);
                        tracking_target_ = target;
                    }
                    event_bus_.publish(events::kTracking, result);
                } else {
                    TrackingResult result;
                    {
                        std::lock_guard<std::mutex> lock(target_mutex_);
                        result.target = tracking_target_;
                    }
                    result.lost = true;
                    event_bus_.publish(events::kTracking, result);
                }
            }
        }
        if (guidance_enabled_ && tracking_enabled_) {
            std::string error;
            if (ensure_guidance(error)) {
                plane::model::DetectTarget target;
                {
                    std::lock_guard<std::mutex> lock(target_mutex_);
                    target = tracking_target_;
                }
                plane::model::NedVelocity velocity;
                if (guidance_->guide(target, state_store_.own_state(), velocity, error)) {
                    GuidanceResult result;
                    result.velocity = velocity;
                    result.valid = true;
                    event_bus_.publish(events::kGuidance, result);
                } else {
                    GuidanceResult result;
                    result.valid = false;
                    event_bus_.publish(events::kGuidance, result);
                }
            }
        }
    }
}

void Perception::run_selftest(const cv::Mat& frame) {
    spdlog::info("感知自检开始 (frame {}x{})", frame.cols, frame.rows);
    std::string error;
    if (!ensure_detection(error)) {
        report_error("自检检测插件不可用: " + error);
        return;
    }
    std::vector<plane::model::DetectTarget> targets;
    if (detection_->detect(frame, targets, error)) {
        spdlog::info("自检 detect 成功: targets={}", targets.size());
        DetectionResult result;
        result.targets = targets;
        event_bus_.publish(events::kDetection, result);
    } else {
        report_error("自检 detect 失败: " + error);
        return;
    }
    if (!ensure_tracking(error)) {
        report_error("自检跟踪插件不可用: " + error);
        return;
    }
    plane::model::DetectTarget init;
    if (targets.empty()) {
        init.bbox.x = frame.cols / 2 - 50;
        init.bbox.y = frame.rows / 2 - 50;
        init.bbox.width = 100;
        init.bbox.height = 100;
        init.confidence = 0.9F;
        init.type = "stub";
    } else {
        init = targets.front();
    }
    if (!tracking_->start(frame, init, error)) {
        report_error("自检 track start 失败: " + error);
        return;
    }
    plane::model::DetectTarget tracked;
    if (tracking_->track(frame, tracked, error)) {
        spdlog::info("自检 track 成功: type={}", tracked.type);
        TrackingResult result;
        result.target = tracked;
        result.lost = false;
        event_bus_.publish(events::kTracking, result);
    } else {
        report_error("自检 track 失败: " + error);
        return;
    }
    if (!ensure_guidance(error)) {
        report_error("自检导引插件不可用: " + error);
        return;
    }
    plane::model::NedVelocity velocity;
    if (guidance_->guide(tracked, state_store_.own_state(), velocity, error)) {
        spdlog::info("自检 guide 成功: vn={} ve={} vd={}", velocity.north_m_s, velocity.east_m_s, velocity.down_m_s);
        GuidanceResult result;
        result.velocity = velocity;
        result.valid = true;
        event_bus_.publish(events::kGuidance, result);
    } else {
        report_error("自检 guide 失败: " + error);
    }
}

bool Perception::ensure_detection(std::string& error) {
    std::lock_guard<std::mutex> lock(plugin_mutex_);
    if (detection_) {
        return true;
    }
    detection_ = plugin_manager_.create_detection();
    if (!detection_) {
        error = "无可用检测插件";
        return false;
    }
    plane::algorithm::DetectConfig detect_config;
    if (!detection_->init(detect_config)) {
        detection_.reset();
        error = "检测插件初始化失败";
        return false;
    }
    spdlog::info("检测插件已实例化");
    return true;
}

bool Perception::ensure_tracking(std::string& error) {
    std::lock_guard<std::mutex> lock(plugin_mutex_);
    if (tracking_) {
        return true;
    }
    tracking_ = plugin_manager_.create_tracking();
    if (!tracking_) {
        error = "无可用跟踪插件";
        return false;
    }
    plane::algorithm::TrackConfig track_config;
    if (!tracking_->init(track_config)) {
        tracking_.reset();
        error = "跟踪插件初始化失败";
        return false;
    }
    spdlog::info("跟踪插件已实例化");
    return true;
}

bool Perception::ensure_guidance(std::string& error) {
    std::lock_guard<std::mutex> lock(plugin_mutex_);
    if (guidance_) {
        return true;
    }
    guidance_ = plugin_manager_.create_guidance();
    if (!guidance_) {
        error = "无可用导引插件";
        return false;
    }
    plane::algorithm::GuidanceConfig guidance_config;
    if (!guidance_->init(guidance_config)) {
        guidance_.reset();
        error = "导引插件初始化失败";
        return false;
    }
    spdlog::info("导引插件已实例化");
    return true;
}

void Perception::report_error(const std::string& detail) const {
    spdlog::error("Perception: {}", detail);
    PerceptionError event;
    event.detail = detail;
    event_bus_.publish(events::kError, event);
}

}  // namespace plane::perception
