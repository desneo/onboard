#pragma once

#include <mutex>
#include <string>

#include "camera/camera_control_api.h"
#include "core/event_bus.h"
#include "core/module.h"

namespace plane::camera {

/// 第一版 Noop 实现：仅维护链路状态与日志；相机 SDK 接入时替换内部实现。
class CameraControlModule final : public plane::core::IModule, public CameraControlApi {
   public:
    explicit CameraControlModule(plane::core::EventBus& event_bus);
    ~CameraControlModule() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    bool connect(std::string& error) override;
    void disconnect() override;
    CameraStatus status() const override;
    bool send_command(const std::string& command, std::string& error) override;

   private:
    plane::core::EventBus& event_bus_;
    mutable std::mutex mutex_;
    CameraStatus status_;
    bool stopped_ = false;
};

}  // namespace plane::camera
