#pragma once

#include <string>

#include "communication/communication_api.h"
#include "message.h"

namespace plane::communication {

bool to_payload(const plane::model::TaskInfo& value, std::string& out, std::string& error);
bool to_payload(const plane::model::Command& value, std::string& out, std::string& error);
bool to_payload(const plane::model::SystemParam& value, std::string& out, std::string& error);
bool to_payload(const plane::model::GcsTarget& value, std::string& out, std::string& error);
bool to_payload(const plane::model::Alert& value, std::string& out, std::string& error);
bool to_payload(const plane::model::VehicleInfo& value, std::string& out, std::string& error);
bool to_payload(const plane::model::VehicleState& value, std::string& out, std::string& error);
bool to_payload(const plane::model::TaskState& value, std::string& out, std::string& error);
bool to_payload(const plane::model::Ack& value, std::string& out, std::string& error);

bool from_payload(const std::string& payload, plane::model::TaskInfo& out, std::string& error);
bool from_payload(const std::string& payload, plane::model::Command& out, std::string& error);
bool from_payload(const std::string& payload, plane::model::SystemParam& out, std::string& error);
bool from_payload(const std::string& payload, plane::model::GcsTarget& out, std::string& error);
bool from_payload(const std::string& payload, plane::model::Ack& out, std::string& error);
bool from_payload(const std::string& payload, plane::model::VehicleInfo& out, std::string& error);
bool from_payload(const std::string& payload, plane::model::VehicleState& out, std::string& error);

std::string encode_envelope(const plane::model::Message& message);
bool decode_envelope(const std::string& wire, plane::model::Message& out, std::string& error);

std::string message_type_to_string(plane::model::MessageType type);
bool message_type_from_string(const std::string& name, plane::model::MessageType& out);

std::string next_message_id(const std::string& sender_id);

bool make_message(const std::string& sender_id, const OutgoingMessage& value, plane::model::Message& out,
                  std::string& error);

}  // namespace plane::communication
