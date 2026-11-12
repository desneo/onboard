#include "communication/plane_query.h"

#include <string_view>
#include <utility>

#include "communication/serialization.h"

namespace plane::communication {
namespace {

std::string fill_key(std::string_view pattern, std::string_view placeholder, const std::string& id) {
    std::string key(pattern);
    key.replace(key.find(placeholder), placeholder.size(), id);
    return key;
}

}  // namespace

std::string station_liveliness_key(const std::string& station_id) {
    return fill_key(plane::model::topic_stationLiveliness, "{stationId}", station_id);
}

std::string plane_liveliness_key(const std::string& plane_id) {
    return fill_key(plane::model::topic_planeLiveliness, "{planeId}", plane_id);
}

std::string agent_query_key(const std::string& agent_id) {
    if (!is_valid_agent_id(agent_id)) {
        return {};
    }
    return fill_key(plane::model::topic_queryToAgent, "{agentId}", agent_id);
}

bool queryAgent(ZenohTransport& transport, const std::string& agent_id, plane::model::Message& reply,
                std::string& error, int timeout_ms) {
    error.clear();
    const std::string key = agent_query_key(agent_id);
    if (key.empty()) {
        error = "查询 agentId 非法";
        return false;
    }
    std::string wire;
    if (!transport.query(key, wire, error, timeout_ms)) return false;
    plane::model::Message result;
    if (!decode_envelope(wire, result, error)) return false;
    if (result.type != plane::model::MessageType::AgentInfo || result.senderId != agent_id ||
        result.messageId.empty()) {
        error = "查询响应类型、发送方或消息 ID 不匹配";
        return false;
    }
    plane::model::AgentInfo info;
    if (!from_payload(result.payload, info, error)) return false;
    if (info.agentId != agent_id) {
        error = "查询响应 agentId 不匹配";
        return false;
    }
    reply = std::move(result);
    return true;
}

}  // namespace plane::communication
