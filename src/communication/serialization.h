#pragma once

#include <string>
#include <string_view>

#include "message.h"

namespace plane::communication {

bool is_valid_agent_id(std::string_view agent_id);

bool to_payload(const plane::model::TaskInfo& value, std::string& out, std::string& error);
bool to_payload(const plane::model::Command& value, std::string& out, std::string& error);
bool to_payload(const plane::model::SelfCheckResult& value, std::string& out, std::string& error);
bool to_payload(const plane::model::PodCommand& value, std::string& out, std::string& error);
bool to_payload(const plane::model::PodState& value, std::string& out, std::string& error);
bool to_payload(const plane::model::SystemParam& value, std::string& out, std::string& error);
bool to_payload(const plane::model::Alert& value, std::string& out, std::string& error);
bool to_payload(const plane::model::AgentInfo& value, std::string& out, std::string& error);
bool to_payload(const plane::model::VehicleState& value, std::string& out, std::string& error);
bool to_payload(const plane::model::TaskState& value, std::string& out, std::string& error);
bool to_payload(const plane::model::Ack& value, std::string& out, std::string& error);

bool from_payload(const std::string& payload, plane::model::TaskInfo& out, std::string& error);
bool from_payload(const std::string& payload, plane::model::Command& out, std::string& error);
bool from_payload(const std::string& payload, plane::model::SelfCheckResult& out, std::string& error);
bool from_payload(const std::string& payload, plane::model::PodCommand& out, std::string& error);
bool from_payload(const std::string& payload, plane::model::PodState& out, std::string& error);
bool from_payload(const std::string& payload, plane::model::SystemParam& out, std::string& error);
bool from_payload(const std::string& payload, plane::model::GcsTarget& out, std::string& error);
bool from_payload(const std::string& payload, plane::model::Ack& out, std::string& error);
bool from_payload(const std::string& payload, plane::model::AgentInfo& out, std::string& error);
bool from_payload(const std::string& payload, plane::model::VehicleState& out, std::string& error);

std::string encode_envelope(const plane::model::Message& message);
bool decode_envelope(const std::string& wire, plane::model::Message& out, std::string& error);

std::string message_type_to_string(plane::model::MessageType type);
bool message_type_from_string(const std::string& name, plane::model::MessageType& out);

}  // namespace plane::communication
