#pragma once

#include <mutex>
#include <string>

#include "communication/communication_api.h"
#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"
#include "flight_control/flight_control_api.h"
#include "task/task_events.h"
#include "task/task_execution_engine_api.h"
#include "task/task_state_provider.h"

namespace plane::task {

/// Task Scheduler：Task 与独立 Command 的唯一控制仲裁者（ARCHITECTURE §8）。
class TaskScheduler final : public plane::core::IModule, public TaskStateProvider {
   public:
    TaskScheduler(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                  plane::flight_control::FlightControlApi& flight_control, TaskExecutionEngineApi* engine = nullptr);
    ~TaskScheduler() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    plane::model::TaskState current_task_state() const override;

    /// 两阶段接线：Communication 构造完成后注入，用于告警发送。
    void set_communication(plane::communication::CommunicationApi& communication);

   private:
    enum class ControlKind { None, Command, Task };

    void on_task_info(const plane::model::TaskInfo& task);
    void on_command(const plane::model::Command& command);
    void on_engine_result(const TaskEngineResult& result);
    void on_execution_progress(const TaskProgress& progress);
    bool dispatch_command(const plane::model::Command& command);
    void send_alert(const std::string& detail);
    void report_error(const std::string& detail) const;

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    plane::flight_control::FlightControlApi& flight_control_;
    plane::communication::CommunicationApi* communication_ = nullptr;
    TaskExecutionEngineApi* engine_;

    mutable std::mutex mutex_;
    ControlKind control_ = ControlKind::None;
    plane::model::TaskState task_state_;
};

}  // namespace plane::task
