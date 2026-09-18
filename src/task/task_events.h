#pragma once

#include <string>
#include <string_view>

#include "message.h"

namespace plane::task {

/// 引擎执行结果（M5 发布，Scheduler 消费）。
struct TaskEngineResult {
    plane::model::TaskStatus status = plane::model::TaskStatus::None;
    std::string detail;
};

/// 引擎执行进度（M5 发布，Scheduler 消费）。
struct TaskProgress {
    std::string currentBehaviorTree;
    std::string currentNode;
};

/// Task Scheduler 错误事件载荷。
struct TaskSchedulerError {
    std::string detail;
};

namespace events {

inline constexpr std::string_view kEngineResult = "task.engine_result";
inline constexpr std::string_view kExecutionProgress = "task.execution_progress";
inline constexpr std::string_view kError = "task_scheduler.error";

}  // namespace events

}  // namespace plane::task
