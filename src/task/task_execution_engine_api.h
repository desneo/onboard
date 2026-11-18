#pragma once

#include <cstdint>
#include <string>

#include "message.h"

namespace plane::task {

/// 任务执行引擎接口；M5 实现（ARCHITECTURE §8/§9）。
class TaskExecutionEngineApi {
   public:
    virtual ~TaskExecutionEngineApi() = default;

    virtual bool start(std::uint64_t execution_id, const plane::model::TaskInfo& task, std::string& error) = 0;

    /// 启动基础指令行为树；成功表示指令已被接受并开始执行。
    virtual bool start_command(std::uint64_t execution_id, const plane::model::Command& command,
                               std::string& error) = 0;

    /// 已进入接管时，正常中止保留该次执行，等待调度器处理接管结果。
    virtual bool abort(bool acknowledge_takeover = false) = 0;
    virtual bool control_can_advance() const = 0;
};

}  // namespace plane::task
