#pragma once

#include <string>
#include <vector>

#include "message.h"

namespace plane::task {

/// 指令行为树运行上下文（引擎持有，节点只读）。
struct CommandContext {
    std::string commandId;                         ///< 来源指令 id
    plane::model::CommandType type = plane::model::CommandType::Arm;  ///< 指令类型
    float takeoffHeight_m = 0.0F;                  ///< 已套用安全高度后的起飞高度
    std::vector<plane::model::GeoPoint> waypoints; ///< Waypoint/Route 使用
};

}  // namespace plane::task
