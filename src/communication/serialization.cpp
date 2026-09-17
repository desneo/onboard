#include "communication/serialization.h"

#include <atomic>
#include <cstdint>
#include <magic_enum/magic_enum.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <type_traits>
#include <utility>

namespace plane::communication {
namespace {

using nlohmann::json;

json to_json(const plane::model::GeoPoint& value) {
    return json{{"latitude", value.latitude},
                {"longitude", value.longitude},
                {"height", value.height},
                {"altitude", value.altitude}};
}

void geo_point_from_json(const json& in, plane::model::GeoPoint& out) {
    out.latitude = in.at("latitude").get<double>();
    out.longitude = in.at("longitude").get<double>();
    out.height = in.at("height").get<float>();
    out.altitude = in.at("altitude").get<float>();
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

template <typename T>
struct MessageTypeOf;

template <>
struct MessageTypeOf<plane::model::Alert> {
    static constexpr plane::model::MessageType value = plane::model::MessageType::Alert;
};

template <>
struct MessageTypeOf<plane::model::Ack> {
    static constexpr plane::model::MessageType value = plane::model::MessageType::Ack;
};

template <>
struct MessageTypeOf<plane::model::VehicleState> {
    static constexpr plane::model::MessageType value = plane::model::MessageType::VehicleState;
};

template <>
struct MessageTypeOf<plane::model::TaskState> {
    static constexpr plane::model::MessageType value = plane::model::MessageType::TaskState;
};

}  // namespace

bool to_payload(const plane::model::TaskInfo& value, std::string& out, std::string& error) {
    try {
        out = json{{"taskId", value.taskId},
                   {"taskCode", value.taskCode},
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
        out.taskCode = in.at("taskCode").get<std::string>();
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

bool to_payload(const plane::model::GcsTarget& value, std::string& out, std::string& error) {
    try {
        out = json{{"trackId", value.trackId},
                   {"geoPos", to_json(value.geoPos)},
                   {"velocity_m_s", value.velocity_m_s},
                   {"direction_deg", value.direction_deg}}
                  .dump();
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
        out.velocity_m_s = in.value("velocity_m_s", 0.0F);
        out.direction_deg = in.value("direction_deg", 0.0F);
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
                   {"roll_deg", value.roll_deg},
                   {"pitch_deg", value.pitch_deg},
                   {"yaw_deg", value.yaw_deg},
                   {"vn_m_s", value.vn_m_s},
                   {"ve_m_s", value.ve_m_s},
                   {"vd_m_s", value.vd_m_s},
                   {"airspeed_m_s", value.airspeed_m_s},
                   {"voltage_v", value.voltage_v},
                   {"flightMode", value.flightMode},
                   {"fcLinkState", value.fcLinkState},
                   {"currentBehaviorTree", value.currentBehaviorTree},
                   {"currentNode", value.currentNode},
                   {"cameraAngle_roll_deg", value.cameraAngle_roll_deg},
                   {"cameraAngle_pitch_deg", value.cameraAngle_pitch_deg},
                   {"cameraAngle_yaw_deg", value.cameraAngle_yaw_deg}}
                  .dump();
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

std::string next_message_id(const std::string& sender_id) {
    static std::atomic<std::uint64_t> counter{0};
    return sender_id + "-" + std::to_string(++counter);
}

bool make_message(const std::string& sender_id, const OutgoingMessage& value, plane::model::Message& out,
                  std::string& error) {
    return std::visit(
        [&](const auto& payload_value) -> bool {
            using T = std::decay_t<decltype(payload_value)>;
            std::string payload;
            if (!to_payload(payload_value, payload, error)) {
                return false;
            }
            out.senderId = sender_id;
            out.messageId = next_message_id(sender_id);
            out.type = MessageTypeOf<T>::value;
            out.payload = std::move(payload);
            return true;
        },
        value);
}

}  // namespace plane::communication
