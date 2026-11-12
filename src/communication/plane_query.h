#pragma once

#include "communication/zenoh_transport.h"
#include "message.h"

namespace plane::communication {

/// 使用公共协议模板构造地面站存活标记地址。
std::string station_liveliness_key(const std::string& station_id);

/// 使用公共协议模板构造飞机存活标记地址。
std::string plane_liveliness_key(const std::string& plane_id);

/// 构造指定节点的 AgentInfo 查询地址，非法 ID 返回空字符串。
std::string agent_query_key(const std::string& agent_id);

/// 原生 Zenoh 查询及消息校验；供 Communication 与地面站联调工具复用。
bool queryAgent(ZenohTransport& transport, const std::string& agent_id, plane::model::Message& reply,
                std::string& error, int timeout_ms = 2000);

}  // namespace plane::communication
