#pragma once

#include <cstdint>
#include <string>

namespace plane::task {

/// suicide 任务参数上下文（引擎持有，节点只读）。
struct SuicideTaskContext {
    std::string targetTrackId;  ///< TaskInfo.targetTrackId，可空
    std::uint64_t targetHistoryStartRevision = 0;  ///< 任务启动时的目标版本，之后的数据用于中制导初始化。
};

}  // namespace plane::task
