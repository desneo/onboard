#pragma once

#include <chrono>
#include <string>
#include <string_view>

namespace plane::communication {

/// 通信错误事件载荷。
struct CommunicationError {
    std::string detail;
};

/// 地面站飞行控制请求与机载通信层的实际收包时间。
template <typename T>
struct ReceivedControl {
    T value;
    std::chrono::steady_clock::time_point received_at;
};

namespace events {

inline constexpr std::string_view kTaskInfoReceived = "communication.task_info";
inline constexpr std::string_view kCommandReceived = "communication.command";
inline constexpr std::string_view kSystemParamReceived = "communication.system_param";
inline constexpr std::string_view kGcsTargetReceived = "communication.gcs_target";
inline constexpr std::string_view kAgentInfoReceived = "communication.agent_info";
inline constexpr std::string_view kError = "communication.error";

}  // namespace events

}  // namespace plane::communication
