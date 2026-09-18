#pragma once

#include <string>

namespace plane::task {

/// suicide 任务参数上下文（引擎持有，节点只读）。
struct SuicideTaskContext {
    std::string targetTrackId;  ///< TaskInfo.targetTrackId，可空
};

}  // namespace plane::task
