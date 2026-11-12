#pragma once

#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "camera/camera_control_api.h"
#include "core/config.h"
#include "core/module.h"

namespace plane::camera {

class PodDriver;

class CameraControlModule final : public plane::core::IModule, public CameraControlApi {
   public:
    explicit CameraControlModule(const core::AppConfig& config);
    CameraControlModule(const core::AppConfig& config, std::unique_ptr<PodDriver> driver);
    ~CameraControlModule() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    void submit_command(const model::PodCommand& command, std::chrono::steady_clock::time_point received_at) override;
    std::optional<model::PodState> pod_state() const override;

   private:
    struct ReceivedCommand {
        model::PodCommand command;
        std::chrono::steady_clock::time_point received_at;
    };

    void worker_loop();
    void set_control_error(std::string error);

    const core::AppConfig& config_;
    std::unique_ptr<PodDriver> driver_;
    mutable std::mutex mutex_;
    std::condition_variable wake_;
    std::optional<model::PodState> state_;
    std::string control_error_;
    std::deque<ReceivedCommand> commands_;
    bool running_ = false;
    std::thread worker_;
};

}  // namespace plane::camera
