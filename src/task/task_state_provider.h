#pragma once

#include "message.h"

namespace plane::task {

/// 当前任务状态数据源（ARCHITECTURE §18）。
class TaskStateProvider {
   public:
    virtual ~TaskStateProvider() = default;

    virtual plane::model::TaskState current_task_state() const = 0;
};

}  // namespace plane::task
