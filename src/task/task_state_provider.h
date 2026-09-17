#pragma once

#include "message.h"

namespace plane::task {

/// 当前任务状态数据源；M5 由 Task Scheduler 实现（ARCHITECTURE §18）。
class TaskStateProvider {
   public:
    virtual ~TaskStateProvider() = default;

    virtual plane::model::TaskState current_task_state() const = 0;
};

/// M2 默认实现：无任务（TaskStatus::None）。
class NoTaskStateProvider final : public TaskStateProvider {
   public:
    plane::model::TaskState current_task_state() const override { return {}; }
};

}  // namespace plane::task
