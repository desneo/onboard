#pragma once

#include <string>

namespace plane::camera {

/// 相机链路状态。
struct CameraStatus {
    bool connected = false;
    std::string detail;
};

/// 相机链路与相机能力接口（SPEC §8.1）；控制指令集待 SDK 到位后细化。
class CameraControlApi {
   public:
    virtual ~CameraControlApi() = default;

    virtual bool connect(std::string& error) = 0;
    virtual void disconnect() = 0;
    virtual CameraStatus status() const = 0;
    virtual bool send_command(const std::string& command, std::string& error) = 0;
};

}  // namespace plane::camera
