#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include "communication/communication_api.h"
#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"
#include "flight_control/flight_control_events.h"
#include "task/task_events.h"
#include "task/task_execution_engine_api.h"
#include "task/task_state_provider.h"

namespace plane::task {

/// Task Scheduler：Task 与独立 Command 的唯一控制仲裁者（ARCHITECTURE §8）。
class TaskScheduler final : public plane::core::IModule, public TaskStateProvider {
   public:
    TaskScheduler(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                  TaskExecutionEngineApi* engine = nullptr);
    ~TaskScheduler() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    plane::model::TaskState current_task_state() const override;

    /// 两阶段接线：Communication 构造完成后注入，用于告警和自检结果发送。
    void set_communication(plane::communication::CommunicationApi& communication);

   private:
    enum class ControlKind { None, Command, Task, SelfCheck };

    using ControlTime = std::chrono::steady_clock::time_point;

    void on_task_info(const plane::model::TaskInfo& task, ControlTime received_at);
    void on_command(const plane::model::Command& command, ControlTime received_at);
    void on_manual_takeover(const plane::flight_control::ManualTakeoverEvent& event);
    void on_engine_result(const TaskEngineResult& result);
    void on_execution_progress(const TaskProgress& progress);
    bool prepare_control(ControlTime received_at);
    void clear_control_locked();
    void report_self_check_result(const plane::model::SelfCheckResult& result);
    void send_alert(const std::string& detail, const std::string& type = "command_dispatch_failed");
    void report_error(const std::string& detail) const;

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    plane::communication::CommunicationApi* communication_ = nullptr;
    TaskExecutionEngineApi* engine_;

    /// 派发和飞控接管中止串行执行。等待引擎退出时不持有状态锁。
    std::mutex dispatch_mutex_;
    bool running_ = false;
    ControlTime last_takeover_finished_at_ = ControlTime::min();
    mutable std::mutex mutex_;
    ControlKind control_ = ControlKind::None;
    std::uint64_t next_execution_id_ = 0;
    std::uint64_t execution_id_ = 0;
    std::string command_id_;
    plane::model::TaskState task_state_;
};

}  // namespace plane::task
