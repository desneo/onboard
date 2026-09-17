#pragma once

#include <string>
#include <string_view>

namespace plane::communication {

/// 通信错误事件载荷。
struct CommunicationError {
    std::string detail;
};

namespace events {

inline constexpr std::string_view kTaskInfoReceived = "communication.task_info";
inline constexpr std::string_view kCommandReceived = "communication.command";
inline constexpr std::string_view kSystemParamReceived = "communication.system_param";
inline constexpr std::string_view kGcsTargetReceived = "communication.gcs_target";
inline constexpr std::string_view kError = "communication.error";

}  // namespace events

}  // namespace plane::communication
