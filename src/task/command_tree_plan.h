#pragma once

#include <string>
#include <vector>

#include "flight_control/flight_control_api.h"
#include "message.h"

namespace plane::task {

/// 根据指令及接收指令时的飞行状态生成顺序节点列表。
std::vector<std::string> command_node_names(plane::model::CommandType type,
                                            const plane::flight_control::FlightStatus& flight_status);

/// AntiUav 任务按当前飞行状态生成前置节点，末节点始终为持续运行的 Midcourse。
std::vector<std::string> anti_uav_node_names(const plane::flight_control::FlightStatus& flight_status);

}  // namespace plane::task
