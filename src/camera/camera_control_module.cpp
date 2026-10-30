#include "camera/camera_control_module.h"

#include <spdlog/spdlog.h>

#include "camera/camera_control_events.h"

namespace plane::camera {

CameraControlModule::CameraControlModule(plane::core::EventBus& event_bus) : event_bus_(event_bus) {}

CameraControlModule::~CameraControlModule() { stop(); }

std::string CameraControlModule::name() const { return "相机控制"; }

bool CameraControlModule::init() { return true; }

bool CameraControlModule::start() {
    stopped_ = false;
    std::string error;
    if (!connect(error)) {
        SPDLOG_ERROR("相机链路自动建联失败: {}", error);
        return false;
    }
    return true;
}

void CameraControlModule::stop() {
    if (stopped_) {
        return;
    }
    stopped_ = true;
    disconnect();
}

void CameraControlModule::release() {}

bool CameraControlModule::connect(std::string&) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.connected = true;
        status_.detail = "noop";
    }
    SPDLOG_INFO("相机链路已建立（空实现）");
    event_bus_.publish(events::kStatusChanged, status());
    return true;
}

void CameraControlModule::disconnect() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.connected = false;
        status_.detail = "noop";
    }
    SPDLOG_INFO("相机链路已断开（空实现）");
    event_bus_.publish(events::kStatusChanged, status());
}

CameraStatus CameraControlModule::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

bool CameraControlModule::send_command(const std::string& command, std::string&) {
    SPDLOG_INFO("相机指令（空实现）: {}", command);
    return true;
}

}  // namespace plane::camera
