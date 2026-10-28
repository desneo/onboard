#pragma once

#include <string>
#include <variant>

#include "message.h"

namespace plane::communication {

/// 出站载荷（Ack 仅供 Communication 内部使用）。
using OutgoingMessage = std::variant<plane::model::VehicleState, plane::model::TaskState, plane::model::Alert,
                                     plane::model::Ack, plane::model::VehicleInfo>;

/// Communication 对业务模块暴露的稳定能力接口。
class CommunicationApi {
   public:
    virtual ~CommunicationApi() = default;

    virtual bool sendToStation(const OutgoingMessage& message) = 0;
    virtual bool sendToPlane(const std::string& plane_id, const OutgoingMessage& message) = 0;
    virtual bool sendToAll(const OutgoingMessage& message) = 0;

    /// 原生查询指定飞机的信息；当前仅支持 VehicleInfo，失败通过 error 返回原因。
    virtual bool queryPlane(const std::string& plane_id, plane::model::MessageType type, plane::model::Message& reply,
                            std::string& error, int timeout_ms = 2000) = 0;
};

}  // namespace plane::communication
