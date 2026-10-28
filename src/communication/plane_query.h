#pragma once

#include "communication/zenoh_transport.h"
#include "message.h"

namespace plane::communication {

/// 使用公共协议模板构造本机查询服务的订阅地址。
std::string plane_query_selector(const std::string& plane_id);

/// 使用公共协议模板构造地面站存活标记地址。
std::string station_liveliness_key(const std::string& station_id);

/// 当前仅支持 VehicleInfo；空 ID、通配 ID 或不支持的类型返回空字符串。
std::string plane_query_key(const std::string& plane_id, plane::model::MessageType type);

/// 原生 Zenoh 查询及消息校验；供 Communication 与地面站联调工具复用。
bool queryPlane(ZenohTransport& transport, const std::string& plane_id, plane::model::MessageType type,
                plane::model::Message& reply, std::string& error, int timeout_ms = 2000);

}  // namespace plane::communication
