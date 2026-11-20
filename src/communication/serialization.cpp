#include "communication/serialization.h"

#include <cmath>
#include <limits>
#include <magic_enum/magic_enum.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>
#include <variant>

namespace plane::communication {
namespace {

using nlohmann::json;

template <typename Number>
Number finite_or_default(Number value, Number fallback = Number{}) {
    return std::isfinite(value) ? value : fallback;
}

template <typename Number>
Number number_or_default(const json& in, const char* key, Number fallback = Number{}) {
    const auto item = in.find(key);
    if (item == in.end() || item->is_null()) {
        return fallback;
    }
    return item->get<Number>();
}

json to_json(const plane::model::GeoPoint& value) {
    return json{{"latitude", finite_or_default(value.latitude)},
                {"longitude", finite_or_default(value.longitude)},
                {"height", finite_or_default(value.height)},
                {"altitude", finite_or_default(value.altitude)}};
}

void geo_point_from_json(const json& in, plane::model::GeoPoint& out) {
    out.latitude = number_or_default<double>(in, "latitude");
    out.longitude = number_or_default<double>(in, "longitude");
    out.height = number_or_default<float>(in, "height");
    out.altitude = number_or_default<float>(in, "altitude");
}

template <typename Enum>
bool enum_from_string(const std::string& name, Enum& out) {
    const auto value = magic_enum::enum_cast<Enum>(name);
    if (!value.has_value()) {
        return false;
    }
    out = *value;
    return true;
}

}  // namespace

bool is_valid_agent_id(std::string_view agent_id) {
    return !agent_id.empty() && agent_id.find_first_of("/*?#$") == std::string_view::npos;
}

bool to_payload(const plane::model::PodCommand& value, std::string& out, std::string& error) {
    using plane::model::PodCommandType;
    try {
        json payload{{"type", std::string(magic_enum::enum_name(value.type))}};
        switch (value.type) {
            case PodCommandType::SetRate:
                if (!std::isfinite(value.pitchRate_deg_s) || !std::isfinite(value.yawRate_deg_s)) {
                    error = "PodCommand 角速度必须为有限数值";
                    return false;
                }
                payload["pitchRate_deg_s"] = value.pitchRate_deg_s;
                payload["yawRate_deg_s"] = value.yawRate_deg_s;
                break;
            case PodCommandType::SetAngles:
                if (!std::isfinite(value.pitch_deg) || !std::isfinite(value.yaw_deg)) {
                    error = "PodCommand 角度必须为有限数值";
                    return false;
                }
                payload["pitch_deg"] = value.pitch_deg;
                payload["yaw_deg"] = value.yaw_deg;
                break;
            case PodCommandType::ZeroPitch:
            case PodCommandType::ZeroYaw:
                break;
            default:
                error = "未知 PodCommandType";
                return false;
        }
        out = payload.dump();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool from_payload(const std::string& payload, plane::model::PodCommand& out, std::string& error) {
    using plane::model::PodCommandType;
    try {
        const json in = json::parse(payload);
        plane::model::PodCommand value;
        if (!enum_from_string(in.at("type").get<std::string>(), value.type)) {
            error = "未知 PodCommandType";
            return false;
        }
        auto read_number = [&](const char* key, double& number) {
            const auto it = in.find(key);
            if (it != in.end() && it->is_number()) {
                number = it->get<double>();
                if (std::isfinite(number)) {
                    return true;
                }
            }
            error = std::string("PodCommand 缺少有效数值 ") + key;
            return false;
        };
        switch (value.type) {
            case PodCommandType::SetRate:
                if (!read_number("pitchRate_deg_s", value.pitchRate_deg_s) ||
                    !read_number("yawRate_deg_s", value.yawRate_deg_s)) {
                    return false;
                }
                break;
            case PodCommandType::SetAngles:
                if (!read_number("pitch_deg", value.pitch_deg) || !read_number("yaw_deg", value.yaw_deg)) {
                    return false;
                }
                break;
            case PodCommandType::ZeroPitch:
            case PodCommandType::ZeroYaw:
                break;
        }
        out = value;
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool to_payload(const plane::model::PodState& value, std::string& out, std::string& error) {
    try {
        json fields = json::object();
        for (const auto& [name, field] : value.fields) {
            if (const auto* number = std::get_if<double>(&field); number && !std::isfinite(*number)) {
                error = "PodState.fields 包含非有限数值";
                return false;
            }
            std::visit([&](const auto& item) { fields[name] = item; }, field);
        }
        out = json{{"planeId", value.planeId},
                   {"model", value.model},
                   {"connected", value.connected},
                   {"canMove", value.canMove},
                   {"timestamp_ms", value.timestamp_ms},
                   {"fields", std::move(fields)}}
                  .dump();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool from_payload(const std::string& payload, plane::model::PodState& out, std::string& error) {
    try {
        const json in = json::parse(payload);
        const auto& fields = in.at("fields");
        const auto& timestamp = in.at("timestamp_ms");
        auto fits_int64 = [](const json& number) {
            return number.is_number_integer() &&
                   (!number.is_number_unsigned() ||
                    number.get<std::uint64_t>() <= static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()));
        };
        if (!fields.is_object() || !fits_int64(timestamp)) {
            error = "PodState.fields 必须为对象且 timestamp_ms 必须为整数";
            return false;
        }
        plane::model::PodState value;
        value.planeId = in.at("planeId").get<std::string>();
        value.model = in.at("model").get<std::string>();
        value.connected = in.at("connected").get<bool>();
        value.canMove = in.at("canMove").get<bool>();
        value.timestamp_ms = timestamp.get<std::int64_t>();
        for (auto it = fields.begin(); it != fields.end(); ++it) {
            if (it->is_boolean()) {
                value.fields.emplace(it.key(), it->get<bool>());
            } else if (fits_int64(*it)) {
                value.fields.emplace(it.key(), it->get<std::int64_t>());
            } else if (it->is_number_float() && std::isfinite(it->get<double>())) {
                value.fields.emplace(it.key(), it->get<double>());
            } else if (it->is_string()) {
                value.fields.emplace(it.key(), it->get<std::string>());
            } else {
                error = "PodState.fields 必须使用布尔值、整数、有限浮点数或字符串";
                return false;
            }
        }
        out = std::move(value);
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool to_payload(const plane::model::AgentInfo& value, std::string& out, std::string& error) {
    try {
        out = json{{"agentId", value.agentId},
                   {"agentType", std::string(magic_enum::enum_name(value.agentType))},
                   {"boardIp", value.boardIp},
                   {"videoUrl", value.videoUrl},
                   {"isSim", value.isSim}}
                  .dump();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool from_payload(const std::string& payload, plane::model::AgentInfo& out, std::string& error) {
    try {
        const json in = json::parse(payload);
        if (!in.is_object() || in.size() != 5) {
            error = "AgentInfo 必须包含五个公共字段";
            return false;
        }
        plane::model::AgentInfo value;
        value.agentId = in.at("agentId").get<std::string>();
        if (!is_valid_agent_id(value.agentId)) {
            error = "AgentInfo.agentId 非法";
            return false;
        }
        if (!enum_from_string(in.at("agentType").get<std::string>(), value.agentType)) {
            error = "未知 AgentType";
            return false;
        }
        value.boardIp = in.at("boardIp").get<std::string>();
        value.videoUrl = in.at("videoUrl").get<std::string>();
        value.isSim = in.at("isSim").get<bool>();
        out = std::move(value);
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool to_payload(const plane::model::TaskInfo& value, std::string& out, std::string& error) {
    try {
        out = json{{"taskId", value.taskId},
                   {"taskCode", std::string(magic_enum::enum_name(value.taskCode))},
                   {"targetTrackId", value.targetTrackId},
                   {"taskAction", std::string(magic_enum::enum_name(value.taskAction))}}
                  .dump();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool from_payload(const std::string& payload, plane::model::TaskInfo& out, std::string& error) {
    try {
        const json in = json::parse(payload);
        out.taskId = in.at("taskId").get<std::string>();
        const std::string task_code = in.at("taskCode").get<std::string>();
        if (!enum_from_string(task_code, out.taskCode)) {
            error = "未知 TaskCode: " + task_code;
            return false;
        }
        out.targetTrackId = in.value("targetTrackId", std::string{});
        if (!enum_from_string(in.at("taskAction").get<std::string>(), out.taskAction)) {
            error = "未知 TaskAction";
            return false;
        }
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool to_payload(const plane::model::Command& value, std::string& out, std::string& error) {
    try {
        json waypoints = json::array();
        for (const auto& point : value.waypoints) {
            waypoints.push_back(to_json(point));
        }
        out = json{{"commandId", value.commandId},
                   {"type", std::string(magic_enum::enum_name(value.type))},
                   {"takeoffHeight_m", value.takeoffHeight_m},
                   {"waypoints", std::move(waypoints)}}
                  .dump();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool from_payload(const std::string& payload, plane::model::Command& out, std::string& error) {
    try {
        const json in = json::parse(payload);
        out.commandId = in.at("commandId").get<std::string>();
        if (!enum_from_string(in.at("type").get<std::string>(), out.type)) {
            error = "未知 CommandType";
            return false;
        }
        if (out.type == plane::model::CommandType::SelfCheck && out.commandId.empty()) {
            error = "SelfCheck 缺少 commandId";
            return false;
        }
        out.takeoffHeight_m = in.value("takeoffHeight_m", 0.0F);
        out.waypoints.clear();
        for (const auto& item : in.value("waypoints", json::array())) {
            plane::model::GeoPoint point;
            geo_point_from_json(item, point);
            out.waypoints.push_back(point);
        }
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool to_payload(const plane::model::SelfCheckResult& value, std::string& out, std::string& error) {
    try {
        out = json{{"commandId", value.commandId}, {"success", value.success}, {"detail", value.detail}}.dump();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool from_payload(const std::string& payload, plane::model::SelfCheckResult& out, std::string& error) {
    try {
        const json in = json::parse(payload);
        auto command_id = in.at("commandId").get<std::string>();
        if (command_id.empty()) {
            error = "SelfCheckResult 缺少 commandId";
            return false;
        }
        out = {std::move(command_id), in.at("success").get<bool>(), in.at("detail").get<std::string>()};
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool to_payload(const plane::model::SystemParam& value, std::string& out, std::string& error) {
    try {
        out = json{{"safeHeight_m", value.safeHeight_m}}.dump();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool from_payload(const std::string& payload, plane::model::SystemParam& out, std::string& error) {
    try {
        const json in = json::parse(payload);
        out.safeHeight_m = in.value("safeHeight_m", 0.0);
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool from_payload(const std::string& payload, plane::model::GcsTarget& out, std::string& error) {
    try {
        const json in = json::parse(payload);
        out.trackId = in.at("trackId").get<std::string>();
        geo_point_from_json(in.at("geoPos"), out.geoPos);
        out.timestamp_ms = in.at("timestamp_ms").get<decltype(out.timestamp_ms)>();
        if (out.trackId.empty() || out.timestamp_ms <= 0 || !std::isfinite(out.geoPos.latitude) ||
            !std::isfinite(out.geoPos.longitude) || !std::isfinite(out.geoPos.altitude)) {
            error = "GcsTarget 缺少有效 trackId、timestamp_ms 或地理位置";
            return false;
        }
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool to_payload(const plane::model::Alert& value, std::string& out, std::string& error) {
    try {
        out = json{{"type", value.type}, {"detail", value.detail}}.dump();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool to_payload(const plane::model::Ack& value, std::string& out, std::string& error) {
    try {
        out = json{{"ackId", value.ackId}}.dump();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool from_payload(const std::string& payload, plane::model::Ack& out, std::string& error) {
    try {
        const json in = json::parse(payload);
        out.ackId = in.at("ackId").get<std::string>();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool to_payload(const plane::model::VehicleState& value, std::string& out, std::string& error) {
    try {
        out = json{{"planeId", value.planeId},
                   {"geoPos", to_json(value.geoPos)},
                   {"startPos", to_json(value.startPos)},
                   {"roll_deg", finite_or_default(value.roll_deg)},
                   {"pitch_deg", finite_or_default(value.pitch_deg)},
                   {"yaw_deg", finite_or_default(value.yaw_deg)},
                   {"vn_m_s", finite_or_default(value.vn_m_s)},
                   {"ve_m_s", finite_or_default(value.ve_m_s)},
                   {"vd_m_s", finite_or_default(value.vd_m_s)},
                   {"airspeed_m_s", finite_or_default(value.airspeed_m_s)},
                   {"voltage_v", finite_or_default(value.voltage_v)},
                   {"flightMode", value.flightMode},
                   {"fcLinkState", value.fcLinkState},
                   {"inair", value.inair},
                   {"currentBehaviorTree", value.currentBehaviorTree},
                   {"currentNode", value.currentNode},
                   {"cameraAngle_roll_deg", finite_or_default(value.cameraAngle_roll_deg)},
                   {"cameraAngle_pitch_deg", finite_or_default(value.cameraAngle_pitch_deg)},
                   {"cameraAngle_yaw_deg", finite_or_default(value.cameraAngle_yaw_deg)}}
                  .dump();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool from_payload(const std::string& payload, plane::model::VehicleState& out, std::string& error) {
    try {
        const json in = json::parse(payload);
        out.planeId = in.at("planeId").get<std::string>();
        geo_point_from_json(in.at("geoPos"), out.geoPos);
        if (in.contains("startPos")) {
            geo_point_from_json(in.at("startPos"), out.startPos);
        }
        out.roll_deg = number_or_default<float>(in, "roll_deg");
        out.pitch_deg = number_or_default<float>(in, "pitch_deg");
        out.yaw_deg = number_or_default<float>(in, "yaw_deg");
        out.vn_m_s = number_or_default<float>(in, "vn_m_s");
        out.ve_m_s = number_or_default<float>(in, "ve_m_s");
        out.vd_m_s = number_or_default<float>(in, "vd_m_s");
        out.airspeed_m_s = number_or_default<float>(in, "airspeed_m_s");
        out.voltage_v = number_or_default<float>(in, "voltage_v");
        out.flightMode = in.value("flightMode", std::string{});
        out.fcLinkState = in.value("fcLinkState", std::string{});
        out.inair = in.value("inair", false);
        out.currentBehaviorTree = in.value("currentBehaviorTree", std::string{});
        out.currentNode = in.value("currentNode", std::string{});
        out.cameraAngle_roll_deg = number_or_default<float>(in, "cameraAngle_roll_deg");
        out.cameraAngle_pitch_deg = number_or_default<float>(in, "cameraAngle_pitch_deg");
        out.cameraAngle_yaw_deg = number_or_default<float>(in, "cameraAngle_yaw_deg");
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

bool to_payload(const plane::model::TaskState& value, std::string& out, std::string& error) {
    try {
        out = json{{"taskId", value.taskId},
                   {"status", std::string(magic_enum::enum_name(value.status))},
                   {"currentBehaviorTree", value.currentBehaviorTree},
                   {"currentNode", value.currentNode}}
                  .dump();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

std::string encode_envelope(const plane::model::Message& message) {
    const json envelope{{"senderId", message.senderId},
                        {"messageId", message.messageId},
                        {"type", message_type_to_string(message.type)},
                        {"payload", message.payload}};
    return envelope.dump();
}

bool decode_envelope(const std::string& wire, plane::model::Message& out, std::string& error) {
    try {
        const json envelope = json::parse(wire);
        out.senderId = envelope.at("senderId").get<std::string>();
        out.messageId = envelope.at("messageId").get<std::string>();
        const std::string type_name = envelope.at("type").get<std::string>();
        if (!message_type_from_string(type_name, out.type)) {
            error = "未知 MessageType: " + type_name;
            return false;
        }
        out.payload = envelope.at("payload").get<std::string>();
        return true;
    } catch (const json::exception& e) {
        error = e.what();
        return false;
    }
}

std::string message_type_to_string(plane::model::MessageType type) { return std::string(magic_enum::enum_name(type)); }

bool message_type_from_string(const std::string& name, plane::model::MessageType& out) {
    return enum_from_string(name, out);
}

}  // namespace plane::communication
