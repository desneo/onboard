#pragma once

#include <chrono>
#include <optional>

#include "message.h"

namespace plane::camera {

class CameraControlApi {
   public:
    virtual ~CameraControlApi() = default;

    /// 接收北向指令，吊舱 I/O 由 Camera 工作线程执行。
    virtual void submit_command(const model::PodCommand& command,
                                std::chrono::steady_clock::time_point received_at) = 0;
    virtual std::optional<model::PodState> pod_state() const = 0;
};

}  // namespace plane::camera
