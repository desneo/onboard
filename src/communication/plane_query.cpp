#include "communication/plane_query.h"

#include <utility>

#include "communication/serialization.h"

namespace plane::communication {
namespace {

std::string query_address(const std::string& plane_id, const std::string& type) {
    std::string key(plane::model::topic_queryToPlane);
    const std::string plane_placeholder = "{planeId}";
    const std::string type_placeholder = "{MessageType}";
    key.replace(key.find(type_placeholder), type_placeholder.size(), type);
    key.replace(key.find(plane_placeholder), plane_placeholder.size(), plane_id);
    return key;
}

}  // namespace

std::string plane_query_selector(const std::string& plane_id) { return query_address(plane_id, "*"); }

std::string station_liveliness_key(const std::string& station_id) {
    std::string key(plane::model::topic_stationLiveliness);
    const std::string placeholder = "{stationId}";
    key.replace(key.find(placeholder), placeholder.size(), station_id);
    return key;
}

std::string plane_query_key(const std::string& plane_id, plane::model::MessageType type) {
    if (plane_id.empty() || plane_id.find_first_of("/*?#$") != std::string::npos ||
        type != plane::model::MessageType::VehicleInfo) {
        return {};
    }
    return query_address(plane_id, message_type_to_string(type));
}

bool queryPlane(ZenohTransport& transport, const std::string& plane_id, plane::model::MessageType type,
                plane::model::Message& reply, std::string& error, int timeout_ms) {
    error.clear();
    const std::string key = plane_query_key(plane_id, type);
    if (key.empty()) {
        error = "非法飞机 ID 或不支持的查询类型（当前仅支持 VehicleInfo）";
        return false;
    }
    std::string wire;
    if (!transport.query(key, wire, error, timeout_ms)) return false;
    plane::model::Message result;
    if (!decode_envelope(wire, result, error)) return false;
    if (result.type != type || result.senderId != plane_id || result.messageId.empty()) {
        error = "查询响应类型、发送方或消息 ID 不匹配";
        return false;
    }
    plane::model::VehicleInfo info;
    if (!from_payload(result.payload, info, error)) return false;
    if (info.planeId != plane_id) {
        error = "查询响应 planeId 不匹配";
        return false;
    }
    reply = std::move(result);
    return true;
}

}  // namespace plane::communication
