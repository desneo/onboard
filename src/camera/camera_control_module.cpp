#include "camera/camera_control_module.h"

#include <spdlog/spdlog.h>

#include <utility>

#include "camera/pod_driver.h"
#include "core/time_utils.h"

namespace plane::camera {
namespace {

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

constexpr auto kWorkerPeriod = 20ms;
constexpr auto kRatePeriod = 100ms;
constexpr auto kRateTimeout = 500ms;
constexpr auto kLinkTimeout = 1s;
constexpr auto kOpenRetryPeriod = 1s;
constexpr std::size_t kCommandQueueLimit = 32;

bool is_rate(const model::PodCommand& command) { return command.type == model::PodCommandType::SetRate; }

bool is_stop(const model::PodCommand& command) {
    return is_rate(command) && command.pitchRate_deg_s == 0.0 && command.yawRate_deg_s == 0.0;
}

}  // namespace

CameraControlModule::CameraControlModule(const core::AppConfig& config) : CameraControlModule(config, nullptr) {}

CameraControlModule::CameraControlModule(const core::AppConfig& config, std::unique_ptr<PodDriver> driver)
    : config_(config), driver_(std::move(driver)) {}

CameraControlModule::~CameraControlModule() { stop(); }

std::string CameraControlModule::name() const { return "相机控制"; }

bool CameraControlModule::init() {
    if (config_.pod_model.empty()) {
        return true;
    }
    std::string error;
    if (!driver_) {
        auto transport = make_pod_transport(config_.pod_control_url, error);
        if (transport) {
            driver_ = make_pod_driver(config_.pod_model, std::move(transport), error);
        }
    }
    state_.emplace();
    state_->planeId = config_.plane_id;
    state_->model = config_.pod_model;
    state_->canMove = driver_ && driver_->can_move();
    if (!driver_) {
        set_control_error(std::move(error));
    }
    return true;
}

bool CameraControlModule::start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (driver_ && !running_) {
        running_ = true;
        worker_ = std::thread(&CameraControlModule::worker_loop, this);
    }
    return true;
}

void CameraControlModule::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        commands_.clear();
    }
    wake_.notify_one();
    if (worker_.joinable()) {
        worker_.join();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_) {
        state_->connected = false;
        state_->fields.clear();
    }
}

void CameraControlModule::release() {}

void CameraControlModule::submit_command(const model::PodCommand& command, Clock::time_point received_at) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (running_) {
            if (!commands_.empty() && is_rate(command) && is_rate(commands_.back().command)) {
                commands_.back() = {command, received_at};
            } else {
                if (commands_.size() == kCommandQueueLimit) {
                    commands_.pop_front();
                }
                commands_.push_back({command, received_at});
            }
            wake_.notify_one();
            return;
        }
    }
    if (!driver_ && config_.pod_model.empty()) {
        set_control_error("未配置吊舱");
    }
}

std::optional<model::PodState> CameraControlModule::pod_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = state_;
    if (result && !control_error_.empty()) {
        result->fields["控制提示"] = control_error_;
    }
    return result;
}

void CameraControlModule::set_control_error(std::string error) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!error.empty() && error != control_error_) {
        SPDLOG_WARN("吊舱控制: pod_model={} {}", config_.pod_model, error);
    }
    control_error_ = std::move(error);
}

void CameraControlModule::worker_loop() {
    std::optional<ReceivedCommand> active_rate;
    std::optional<Clock::time_point> last_telemetry;
    Clock::time_point last_rate_sent{};
    Clock::time_point next_open{};
    bool opened = false;
    const auto send_stop = [&](Clock::time_point now) {
        std::string error;
        if (!driver_->apply(model::PodCommand{}, now, error)) {
            set_control_error(std::move(error));
        }
        active_rate.reset();
    };
    while (true) {
        std::deque<ReceivedCommand> commands;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            wake_.wait_for(lock, kWorkerPeriod, [this]() { return !running_ || !commands_.empty(); });
            if (!running_) {
                break;
            }
            commands.swap(commands_);
        }
        const auto now = Clock::now();
        if (!opened && now >= next_open) {
            std::string error;
            opened = driver_->start(error);
            set_control_error(std::move(error));
            next_open = now + kOpenRetryPeriod;
        }
        if (last_telemetry && now - *last_telemetry >= kLinkTimeout) {
            if (active_rate) {
                send_stop(now);
            }
            last_telemetry.reset();
            {
                std::lock_guard<std::mutex> lock(mutex_);
                state_->connected = false;
                state_->fields.clear();
            }
            SPDLOG_WARN("吊舱遥测超时: pod_model={}", config_.pod_model);
        }
        if (active_rate && now - active_rate->received_at >= kRateTimeout) {
            send_stop(now);
        }
        for (const auto& input : commands) {
            if (is_rate(input.command) && !is_stop(input.command) && now - input.received_at >= kRateTimeout) {
                continue;
            }
            if (!driver_->can_move()) {
                set_control_error("该型号不支持机械转向");
                continue;
            }
            if (!opened || (!last_telemetry && !is_stop(input.command))) {
                set_control_error("吊舱遥测未就绪");
                continue;
            }
            if (is_stop(input.command)) {
                active_rate.reset();
            }
            std::string error;
            if (!driver_->apply(input.command, now, error)) {
                set_control_error(std::move(error));
                continue;
            }
            set_control_error({});
            active_rate.reset();
            if (is_rate(input.command) && !is_stop(input.command)) {
                active_rate = input;
                last_rate_sent = now;
            }
        }
        if (active_rate && now - last_rate_sent >= kRatePeriod) {
            std::string error;
            if (!driver_->apply(active_rate->command, now, error)) {
                send_stop(now);
                set_control_error(std::move(error));
            }
            last_rate_sent = now;
        }
        if (opened) {
            if (auto telemetry = driver_->tick(now)) {
                if (!last_telemetry) {
                    SPDLOG_INFO("收到有效吊舱遥测: pod_model={}", config_.pod_model);
                }
                last_telemetry = now;
                std::lock_guard<std::mutex> lock(mutex_);
                state_->connected = true;
                state_->timestamp_ms = core::now_ms();
                state_->fields = std::move(telemetry->fields);
            }
        }
    }
    if (opened) {
        if (active_rate) {
            send_stop(Clock::now());
        }
        driver_->stop();
    }
}

}  // namespace plane::camera
