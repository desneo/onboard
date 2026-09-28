#pragma once

#include <string>

#include "message.h"

namespace plane::task {

/// 任务执行引擎接口；M5 实现（ARCHITECTURE §8/§9）。
class TaskExecutionEngineApi {
   public:
    virtual ~TaskExecutionEngineApi() = default;

    virtual bool start(const plane::model::TaskInfo& task, std::string& error) = 0;

    /// 启动基础指令行为树；成功表示指令已被接受并开始执行。
    virtual bool start_command(const plane::model::Command& command, std::string& error) = 0;

    virtual void abort() = 0;
};

}  // namespace plane::task
