#include <arpa/inet.h>
#include <netinet/in.h>
#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "communication/plane_query.h"
#include "communication/serialization.h"
#include "communication/zenoh_transport.h"
#include "core/log_pattern.h"
#include "message.h"
#include "radar_target_filter.h"

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;
std::mutex g_send_mutex;

void handle_stop_signal(int) { g_stop_requested = 1; }

std::int64_t unix_now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
        .count();
}

/// 一次待发送的控制内容，按命令行出现顺序执行。
enum class ActionKind {
    Command,
    Task,
    PodCommand,
};

struct Action {
    ActionKind kind = ActionKind::Command;
    std::string type;         ///< CommandType 或 TaskCode
    std::string task_action;  ///< TaskAction，仅 Task 使用
    plane::model::PodCommand pod{};
    std::chrono::milliseconds pod_rate_duration{0};
    bool pod_release = true;
};

struct Options {
    std::string plane_id = "uav1";
    std::string sender_id = "gcs-sim";
    std::string board_ip;
    std::string peer_plane_id;
    bool no_ack = false;
    std::optional<int> listen_seconds;
    int ack_timeout_ms = 2000;
    int self_check_timeout_ms = 35000;
    int control_settle_ms = 0;
    bool help = false;
    std::string query_type;
    std::string query_agent_id;
    int query_timeout_ms = 2000;
    bool announce_presence = true;
    plane::communication::ZenohEndpoints endpoints;
    std::vector<Action> actions;
    std::string task_id;
    std::string target_track_id;
    float takeoff_height_m = 10.0F;
    std::vector<plane::model::GeoPoint> waypoints;
    std::string gcs_target_id;
    int gcs_target_repeat = 1;
    std::vector<std::pair<std::string, std::string>> send_params;
    bool radar_udp_enabled = true;
    std::string radar_bind = "0.0.0.0";
    int radar_port = 7790;
    double friendly_match_max_distance_m = 50.0;
    int friendly_state_timeout_ms = 3000;
};

struct Envelope {
    std::string sender_id;
    std::string message_id;
    std::string type;
    std::string payload;
};

void print_usage() {
    const std::string usage =
        "用法: gcs_sim [选项]\n"
        "\n"
        "联调工具：模拟地面站(GCS)向指定飞机发送控制指令/任务，并监听其上报。\n"
        "\n"
        "目标与标识:\n"
        "  --plane-id <id>          目标飞机 id (默认: uav1)\n"
        "  --sender-id <id>         地面站 Agent 标识 (默认: gcs-sim)\n"
        "  --board-ip <ip>          地面站 Agent 上报的通信 IP (默认: 空)\n"
        "  -h, --help               显示本帮助并退出\n"
        "\n"
        "控制指令 (按出现顺序发往 sendToPlane/<plane-id>):\n"
        "  --arm                    解锁 (Arm)\n"
        "  --self-check             自检 (SelfCheck)，等待执行结果\n"
        "  --disarm                 上锁 (Disarm)\n"
        "  --takeoff                起飞 (Takeoff)，高度取 --takeoff-height\n"
        "  --land                   降落 (Land)\n"
        "  --rtl                    返航 (RTL)\n"
        "  --hover                  悬停 (Hover)\n"
        "  --stop-motor             停桨 (StopMotor)\n"
        "  --takeoff-height <m>     起飞高度(米)，默认 10；填 0 表示使用安全高度\n"
        "  --waypoint <lat,lon,height[,altitude]>  添加航点，可重复指定\n"
        "  --command-type <type>    直接指定指令类型 (Arm/Disarm/Takeoff/Land/RTL/...)\n"
        "\n"
        "吊舱控制 (PodCommand):\n"
        "  --pod-rate <pitch,yaw,ms>          按住持续转动 (度/秒)，结束时发双零松手\n"
        "  --pod-rate-timeout <pitch,yaw,ms>  按住持续转动 (度/秒)，结束时停发以验证超时停车\n"
        "  --pod-zero-pitch                   俯仰归零\n"
        "  --pod-zero-yaw                     偏航归零\n"
        "  --pod-angles <pitch,yaw>           指定机械俯仰角、偏航角 (度)\n"
        "\n"
        "任务 (按出现顺序发往 sendToPlane/<plane-id>):\n"
        "  --anti-uav               下发反无人机中制导任务 (TaskAction=Start)\n"
        "  --stop-task              终止当前任务 (TaskAction=Stop)\n"
        "  --task-id <id>           指定 Start/Stop 共用的任务 id\n"
        "  --target-track-id <id>   任务关联的目标跟踪 id (可空)\n"
        "  --task-code <code>       任务类型，兼容项 (默认: AntiUav)\n"
        "  --task-action <action>   任务动作 Start/Stop，兼容项\n"
        "\n"
        "信息查询与连接:\n"
        "  --query AgentInfo        原生查询指定 Agent\n"
        "  --agent-id <id>          查询目标的 Agent 标识 (默认: --plane-id)\n"
        "  --query-timeout-ms <ms>  查询超时（默认 2000）\n"
        "  --no-presence            跳过自身上线标记、信息上报和查询响应\n"
        "  --zenoh-config <path>    Zenoh 配置文件\n"
        "  --connect <endpoint>     远端端点，多个用逗号分隔\n"
        "  --listen <endpoint>      本地监听端点\n"
        "\n"
        "其他:\n"
        "  --listen-seconds <s>     额外监听时间；0 表示持续运行，执行自检后默认退出\n"
        "  --ack-timeout-ms <ms>    等待 Ack 超时 (默认: 2000)\n"
        "  --self-check-timeout-ms <ms>  等待 SelfCheckResult 超时 (默认: 35000)\n"
        "  --control-settle-ms <ms> 控制报文发送前等待 Zenoh 路由匹配 (默认: 0)\n"
        "  --no-ack                 不回复收到的告警/Ack\n"
        "  --send-gcs-target <id>   下发地面站目标轨迹\n"
        "  --gcs-target-repeat <n>  地面站目标重复发送次数 (默认: 1)\n"
        "  --send-param <k=v>       下发系统参数，可多次 (如 safe_height_m=30)\n"
        "  --send-peer-state <id>   模拟友机状态上报\n"
        "  --radar-bind <address>   雷达 UDP 绑定地址 (默认: 0.0.0.0)\n"
        "  --radar-port <port>      雷达 UDP 端口 (默认: 7790)\n"
        "  --no-radar-udp           关闭雷达 UDP 入口\n"
        "  --friendly-match-max-distance-m <m>  友机航迹最大匹配距离 (默认: 50)\n"
        "  --friendly-state-timeout-ms <ms>     友机状态有效期 (默认: 3000)\n"
        "\n"
        "示例:\n"
        "  gcs_sim --plane-id uav1 --arm\n"
        "  gcs_sim --plane-id uav1 --takeoff --takeoff-height 20\n"
        "  gcs_sim --plane-id uav1 --sender-id GCS --waypoint 32.2,119.1,40 --command-type Route\n"
        "  gcs_sim --plane-id uav1 --no-radar-udp --pod-rate 8,-5,1500 --listen-seconds 3\n"
        "  gcs_sim --plane-id uav1 --anti-uav --target-track-id t-1\n"
        "  gcs_sim --plane-id uav1 --arm --takeoff --takeoff-height 20 --anti-uav\n";
    spdlog::info("{}", usage);
}

bool parse_envelope(const std::string& wire, Envelope& out) {
    const nlohmann::json envelope = nlohmann::json::parse(wire, nullptr, false);
    if (envelope.is_discarded() || !envelope.is_object()) {
        return false;
    }
    try {
        out.sender_id = envelope.value("senderId", std::string{});
        out.message_id = envelope.value("messageId", std::string{});
        out.type = envelope.value("type", std::string{});
        out.payload = envelope.value("payload", std::string{});
    } catch (const nlohmann::json::exception&) {
        return false;
    }
    return !out.message_id.empty() && !out.type.empty();
}

Action parse_pod_action(const std::string& option, const std::string& value) {
    const bool rate = option != "--pod-angles";
    std::stringstream stream(value);
    std::vector<std::string> fields;
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    if (fields.size() != (rate ? 3U : 2U) || value.back() == ',') {
        throw std::invalid_argument(option + (rate ? " 格式应为 pitch,yaw,ms" : " 格式应为 pitch,yaw"));
    }
    const auto parse_number = [&](const std::string& text) {
        std::size_t end = 0;
        const double number = std::stod(text, &end);
        if (end != text.size() || !std::isfinite(number)) {
            throw std::invalid_argument(option + " 的角度或角速度必须为有限数值");
        }
        return number;
    };
    const double pitch = parse_number(fields[0]);
    const double yaw = parse_number(fields[1]);

    Action action{ActionKind::PodCommand, {}, {}};
    if (rate) {
        std::size_t end = 0;
        const int duration_ms = std::stoi(fields[2], &end);
        if (end != fields[2].size() || duration_ms <= 0) {
            throw std::invalid_argument(option + " 的持续时间必须为正整数毫秒");
        }
        action.pod.type = plane::model::PodCommandType::SetRate;
        action.pod.pitchRate_deg_s = pitch;
        action.pod.yawRate_deg_s = yaw;
        action.pod_rate_duration = std::chrono::milliseconds(duration_ms);
        action.pod_release = option == "--pod-rate";
    } else {
        action.pod.type = plane::model::PodCommandType::SetAngles;
        action.pod.pitch_deg = pitch;
        action.pod.yaw_deg = yaw;
    }
    return action;
}

Options parse_options(int argc, char** argv) {
    Options options;
    std::string pending_task_code = "AntiUav";

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            options.help = true;
            return options;
        }
        if (arg == "--plane-id" && i + 1 < argc) {
            options.plane_id = argv[++i];
        } else if (arg == "--query" && i + 1 < argc) {
            options.query_type = argv[++i];
        } else if (arg == "--agent-id" && i + 1 < argc) {
            options.query_agent_id = argv[++i];
        } else if (arg == "--query-timeout-ms" && i + 1 < argc) {
            options.query_timeout_ms = std::stoi(argv[++i]);
        } else if (arg == "--no-presence") {
            options.announce_presence = false;
        } else if (arg == "--zenoh-config" && i + 1 < argc) {
            options.endpoints.config_file = argv[++i];
        } else if (arg == "--connect" && i + 1 < argc) {
            options.endpoints.connect = argv[++i];
        } else if (arg == "--listen" && i + 1 < argc) {
            options.endpoints.listen = argv[++i];
        } else if (arg == "--sender-id" && i + 1 < argc) {
            options.sender_id = argv[++i];
        } else if (arg == "--board-ip" && i + 1 < argc) {
            options.board_ip = argv[++i];
        } else if (arg == "--send-peer-state" && i + 1 < argc) {
            options.peer_plane_id = argv[++i];
        } else if (arg == "--no-ack") {
            options.no_ack = true;
        } else if (arg == "--listen-seconds" && i + 1 < argc) {
            options.listen_seconds = std::stoi(argv[++i]);
        } else if (arg == "--ack-timeout-ms" && i + 1 < argc) {
            options.ack_timeout_ms = std::stoi(argv[++i]);
        } else if (arg == "--self-check-timeout-ms" && i + 1 < argc) {
            options.self_check_timeout_ms = std::stoi(argv[++i]);
            if (options.self_check_timeout_ms <= 0) {
                throw std::invalid_argument("--self-check-timeout-ms 必须大于 0");
            }
        } else if (arg == "--control-settle-ms" && i + 1 < argc) {
            options.control_settle_ms = std::stoi(argv[++i]);
        } else if (arg == "--takeoff-height" && i + 1 < argc) {
            options.takeoff_height_m = std::stof(argv[++i]);
        } else if (arg == "--waypoint" && i + 1 < argc) {
            std::stringstream stream(argv[++i]);
            std::string field;
            std::vector<std::string> fields;
            while (std::getline(stream, field, ',')) {
                fields.push_back(field);
            }
            if (fields.size() != 3 && fields.size() != 4) {
                throw std::invalid_argument("--waypoint 格式应为 lat,lon,height[,altitude]");
            }
            plane::model::GeoPoint point;
            point.latitude = std::stod(fields[0]);
            point.longitude = std::stod(fields[1]);
            point.height = std::stof(fields[2]);
            if (fields.size() == 4) {
                point.altitude = std::stof(fields[3]);
            }
            options.waypoints.push_back(point);
        } else if (arg == "--arm") {
            options.actions.push_back({ActionKind::Command, "Arm", {}});
        } else if (arg == "--self-check") {
            options.actions.push_back({ActionKind::Command, "SelfCheck", {}});
        } else if (arg == "--disarm") {
            options.actions.push_back({ActionKind::Command, "Disarm", {}});
        } else if (arg == "--takeoff") {
            options.actions.push_back({ActionKind::Command, "Takeoff", {}});
        } else if (arg == "--land") {
            options.actions.push_back({ActionKind::Command, "Land", {}});
        } else if (arg == "--rtl") {
            options.actions.push_back({ActionKind::Command, "RTL", {}});
        } else if (arg == "--hover") {
            options.actions.push_back({ActionKind::Command, "Hover", {}});
        } else if (arg == "--stop-motor") {
            options.actions.push_back({ActionKind::Command, "StopMotor", {}});
        } else if (arg == "--command-type" && i + 1 < argc) {
            options.actions.push_back({ActionKind::Command, argv[++i], {}});
        } else if (arg == "--pod-zero-pitch" || arg == "--pod-zero-yaw") {
            Action action{ActionKind::PodCommand, {}, {}};
            action.pod.type = arg == "--pod-zero-pitch" ? plane::model::PodCommandType::ZeroPitch
                                                       : plane::model::PodCommandType::ZeroYaw;
            options.actions.push_back(action);
        } else if (arg == "--pod-angles" || arg == "--pod-rate" || arg == "--pod-rate-timeout") {
            if (i + 1 >= argc) throw std::invalid_argument(arg + " 缺少参数");
            options.actions.push_back(parse_pod_action(arg, argv[++i]));
        } else if (arg == "--anti-uav") {
            options.actions.push_back({ActionKind::Task, pending_task_code, "Start"});
        } else if (arg == "--stop-task") {
            options.actions.push_back({ActionKind::Task, pending_task_code, "Stop"});
        } else if (arg == "--task-code" && i + 1 < argc) {
            pending_task_code = argv[++i];
        } else if (arg == "--task-action" && i + 1 < argc) {
            options.actions.push_back({ActionKind::Task, pending_task_code, argv[++i]});
        } else if (arg == "--task-id" && i + 1 < argc) {
            options.task_id = argv[++i];
        } else if (arg == "--target-track-id" && i + 1 < argc) {
            options.target_track_id = argv[++i];
        } else if (arg == "--send-gcs-target" && i + 1 < argc) {
            options.gcs_target_id = argv[++i];
        } else if (arg == "--gcs-target-repeat" && i + 1 < argc) {
            options.gcs_target_repeat = std::stoi(argv[++i]);
        } else if (arg == "--radar-bind" && i + 1 < argc) {
            options.radar_bind = argv[++i];
        } else if (arg == "--radar-port" && i + 1 < argc) {
            options.radar_port = std::stoi(argv[++i]);
        } else if (arg == "--no-radar-udp") {
            options.radar_udp_enabled = false;
        } else if (arg == "--friendly-match-max-distance-m" && i + 1 < argc) {
            options.friendly_match_max_distance_m = std::stod(argv[++i]);
        } else if (arg == "--friendly-state-timeout-ms" && i + 1 < argc) {
            options.friendly_state_timeout_ms = std::stoi(argv[++i]);
        } else if (arg == "--send-param" && i + 1 < argc) {
            const std::string pair = argv[++i];
            const auto pos = pair.find('=');
            if (pos != std::string::npos) {
                options.send_params.emplace_back(pair.substr(0, pos), pair.substr(pos + 1));
            }
        } else {
            SPDLOG_WARN("忽略未知参数: {}", arg);
        }
    }
    return options;
}

std::string build_wire(const std::string& sender_id, const std::string& message_id, const std::string& type,
                       const std::string& payload) {
    const nlohmann::json envelope{
        {"senderId", sender_id}, {"messageId", message_id}, {"type", type}, {"payload", payload}};
    return envelope.dump();
}

std::string encode_target_payload(const plane::model::GcsTarget& target) {
    const auto& position = target.geoPos;
    return nlohmann::json{{"trackId", target.trackId},
                          {"geoPos",
                           {{"latitude", position.latitude},
                            {"longitude", position.longitude},
                            {"height", position.height},
                            {"altitude", position.altitude}}},
                          {"timestamp_ms", target.timestamp_ms}}
        .dump();
}

void send_wire(plane::communication::ZenohTransport& session, const std::string& topic, const std::string& wire) {
    std::string error;
    std::lock_guard<std::mutex> lock(g_send_mutex);
    if (!session.put(topic, wire, error)) throw std::runtime_error(error);
}

void send_typed(plane::communication::ZenohTransport& session, const Options& options, const std::string& type,
                const std::string& payload, const std::string& message_id) {
    const std::string topic = "sendToPlane/" + options.plane_id;
    send_wire(session, topic, build_wire(options.sender_id, message_id, type, payload));
    SPDLOG_INFO("已发送 {}: messageId={}", type, message_id);
}

}  // namespace

int main(int argc, char** argv) {
    plane::core::set_project_log_pattern();
    Options options;
    try {
        options = parse_options(argc, argv);
    } catch (const std::exception& exception) {
        SPDLOG_ERROR("参数无效: {}", exception.what());
        return 2;
    }
    if (options.help) {
        print_usage();
        return 0;
    }
    if (!options.query_type.empty() && options.query_type != "AgentInfo") {
        SPDLOG_ERROR("不支持的查询类型: {}", options.query_type);
        return 2;
    }

    std::mutex mutex;
    std::condition_variable cv;
    std::set<std::string> acked_ids;
    std::string pending_self_check;
    std::optional<plane::model::SelfCheckResult> self_check_result;
    std::atomic<std::uint64_t> sequence{100};

    plane::communication::ZenohTransport session;
    std::string error;
    if (!session.open(options.endpoints, error)) {
        SPDLOG_ERROR("{}", error);
        return 2;
    }

    std::atomic<std::uint64_t> vehicle_state_count{0};
    std::atomic<std::uint64_t> task_state_count{0};
    std::atomic<std::uint64_t> pod_state_count{0};
    std::atomic<std::uint64_t> agent_info_count{0};
    std::atomic<std::uint64_t> radar_datagram_count{0};
    std::atomic<std::uint64_t> radar_valid_scan_count{0};
    std::atomic<std::uint64_t> radar_no_target_scan_count{0};
    std::atomic<std::uint64_t> radar_target_count{0};
    plane::gcs_sim::RadarFilterConfig radar_filter_config;
    radar_filter_config.friendly_match_max_distance_m = options.friendly_match_max_distance_m;
    radar_filter_config.friendly_state_timeout_ms = options.friendly_state_timeout_ms;
    plane::gcs_sim::RadarTargetFilter radar_filter(radar_filter_config);

    struct CloseSession {
        plane::communication::ZenohTransport& session;
        ~CloseSession() { session.close(); }
    };

    auto on_sample = [&](const std::string& wire, bool from_broadcast) {
        Envelope envelope;
        if (!parse_envelope(wire, envelope)) {
            SPDLOG_WARN("收到无法解析的报文");
            return;
        }
        if (envelope.sender_id == options.sender_id) {
            return;
        }
        if (envelope.type == "AgentInfo") {
            plane::model::AgentInfo info;
            std::string decode_error;
            if (from_broadcast && plane::communication::from_payload(envelope.payload, info, decode_error) &&
                envelope.sender_id == info.agentId) {
                agent_info_count.fetch_add(1);
                SPDLOG_INFO("收到 AgentInfo: {}", envelope.payload);
            } else {
                SPDLOG_WARN("AgentInfo 无效: senderId={} {}", envelope.sender_id, decode_error);
            }
            return;
        }
        if (envelope.type == "VehicleState") {
            if (!from_broadcast) {
                SPDLOG_WARN("忽略非 sendToAll 来源的飞机状态: senderId={}", envelope.sender_id);
                return;
            }
            plane::model::VehicleState state;
            std::string decode_error;
            if (plane::communication::from_payload(envelope.payload, state, decode_error)) {
                std::string filter_error;
                if (!radar_filter.update_friendly(envelope.sender_id, state, unix_now_ms(), filter_error)) {
                    SPDLOG_WARN("忽略非法友机状态: senderId={} error={}", envelope.sender_id, filter_error);
                }
                SPDLOG_INFO(
                    "收到飞机状态: planeId={} fcLinkState={} flightMode={} voltage_v={} "
                    "geoPos.latitude={} geoPos.longitude={} geoPos.height={} geoPos.altitude={}",
                    state.planeId, state.fcLinkState, state.flightMode, state.voltage_v, state.geoPos.latitude,
                    state.geoPos.longitude, state.geoPos.height, state.geoPos.altitude);
            } else {
                SPDLOG_WARN("飞机状态解析失败: senderId={} error={}", envelope.sender_id, decode_error);
            }
            vehicle_state_count.fetch_add(1);
            return;
        }
        if (envelope.type == "TaskState") {
            const nlohmann::json payload = nlohmann::json::parse(envelope.payload, nullptr, false);
            if (!payload.is_discarded()) {
                SPDLOG_INFO("收到任务状态: taskId={} status={} currentNode={}", payload.value("taskId", std::string{}),
                            payload.value("status", std::string{}), payload.value("currentNode", std::string{}));
            }
            task_state_count.fetch_add(1);
            return;
        }
        if (envelope.type == "PodState") {
            if (from_broadcast || envelope.sender_id != options.plane_id) return;
            plane::model::PodState state;
            std::string decode_error;
            if (!plane::communication::from_payload(envelope.payload, state, decode_error) ||
                state.planeId != envelope.sender_id) {
                SPDLOG_WARN("PodState 无效: senderId={} {}", envelope.sender_id, decode_error);
                return;
            }
            pod_state_count.fetch_add(1);
            SPDLOG_INFO("收到 PodState: {}", envelope.payload);
            return;
        }
        if (envelope.type == "SelfCheckResult") {
            if (from_broadcast || envelope.sender_id != options.plane_id) return;
            plane::model::SelfCheckResult result;
            std::string decode_error;
            if (!plane::communication::from_payload(envelope.payload, result, decode_error)) {
                SPDLOG_WARN("SelfCheckResult 解码失败: senderId={} {}", envelope.sender_id, decode_error);
                return;
            }
            SPDLOG_INFO("收到 SelfCheckResult: senderId={} commandId={} success={} detail={}", envelope.sender_id,
                        result.commandId, result.success, result.detail);
            std::lock_guard<std::mutex> lock(mutex);
            if (pending_self_check == result.commandId && !self_check_result) {
                self_check_result = std::move(result);
                cv.notify_all();
            }
            return;
        }
        if (envelope.type == "Ack") {
            const nlohmann::json payload = nlohmann::json::parse(envelope.payload, nullptr, false);
            if (!payload.is_discarded() && payload.contains("ackId")) {
                std::lock_guard<std::mutex> lock(mutex);
                acked_ids.insert(payload.at("ackId").get<std::string>());
                cv.notify_all();
            }
        }
        SPDLOG_INFO("收到 {}: senderId={} messageId={}", envelope.type, envelope.sender_id, envelope.message_id);
        if (!options.no_ack && (envelope.type == "Alert" || envelope.type == "Ack")) {
            const nlohmann::json payload{{"ackId", envelope.message_id}};
            const std::string message_id = options.sender_id + "-ack-" + std::to_string(++sequence);
            const std::string topic = "sendToPlane/" + envelope.sender_id;
            send_wire(session, topic, build_wire(options.sender_id, message_id, "Ack", payload.dump()));
        }
    };

    const std::string station_token = plane::communication::station_liveliness_key(options.sender_id);
    const std::string agent_key = plane::communication::agent_query_key(options.sender_id);
    const plane::model::AgentInfo agent_info{
        options.sender_id, plane::model::AgentType::GroundStation, options.board_ip, "", true};
    std::string agent_info_payload;
    std::mutex presence_mutex;
    std::set<std::string> online_agent_keys;
    std::atomic<bool> agent_info_pending{false};
    const auto on_liveliness = [&](const std::string& key, bool online) {
        if (key == station_token) return;
        std::lock_guard<std::mutex> lock(presence_mutex);
        if (!online) {
            online_agent_keys.erase(key);
        } else if (online_agent_keys.insert(key).second) {
            agent_info_pending = true;
        }
    };
    const auto agent_info_wire = [&]() {
        return plane::communication::encode_envelope(
            {options.sender_id, options.sender_id + "-info-" + std::to_string(unix_now_ms()) + "-" +
                                    std::to_string(++sequence),
             plane::model::MessageType::AgentInfo, agent_info_payload});
    };
    const auto publish_agent_info = [&]() {
        std::string publish_error;
        std::lock_guard<std::mutex> lock(g_send_mutex);
        if (!session.put(std::string(plane::model::topic_sendToAll), agent_info_wire(), publish_error)) {
            SPDLOG_ERROR("发布 AgentInfo 失败: {}", publish_error);
            return false;
        }
        SPDLOG_INFO("已发布 AgentInfo: agentId={}", options.sender_id);
        return true;
    };
    // 先停止所有回调，再析构被回调引用的局部状态。
    CloseSession close_session{session};

    if (!session.subscribe(
            std::string(plane::model::topic_sendToStation), [&](const std::string& wire) { on_sample(wire, false); },
            error) ||
        !session.subscribe(
            std::string(plane::model::topic_sendToAll), [&](const std::string& wire) { on_sample(wire, true); },
            error)) {
        SPDLOG_ERROR("{}", error);
        return 2;
    }
    if (options.announce_presence) {
        if (agent_key.empty()) {
            SPDLOG_ERROR("Agent 标识无效: agentId={}", options.sender_id);
            return 2;
        }
        if (!plane::communication::to_payload(agent_info, agent_info_payload, error) ||
            !session.declare_queryable(
                agent_key,
                [&](const std::string& key, std::string& reply, std::string& query_error) {
                    if (key != agent_key) {
                        query_error = "AgentInfo 查询地址不匹配";
                        return false;
                    }
                    reply = agent_info_wire();
                    return true;
                },
                error) ||
            !session.subscribe_liveliness(plane::communication::plane_liveliness_key("*"), on_liveliness, error) ||
            !session.subscribe_liveliness(plane::communication::station_liveliness_key("*"), on_liveliness, error) ||
            !session.declare_liveliness_token(station_token, error)) {
            SPDLOG_ERROR("{}", error);
            return 2;
        }
        agent_info_pending = false;
        if (!publish_agent_info()) return 2;
    }
    SPDLOG_INFO("地面站订阅已就绪: sender_id={}", options.sender_id);

    if (options.control_settle_ms < 0) {
        SPDLOG_ERROR("control_settle_ms 不能为负数");
        return 2;
    }
    if (options.control_settle_ms > 0) {
        SPDLOG_INFO("控制会话稳定等待: control_settle_ms={}", options.control_settle_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(options.control_settle_ms));
    }

    std::atomic<bool> radar_running{false};
    int radar_socket = -1;
    std::thread radar_thread;
    struct RadarCleanup {
        std::atomic<bool>& running;
        int& socket;
        std::thread& thread;

        void stop() {
            running = false;
            if (thread.joinable()) thread.join();
            if (socket >= 0) {
                ::close(socket);
                socket = -1;
            }
        }
        ~RadarCleanup() { stop(); }
    } radar_cleanup{radar_running, radar_socket, radar_thread};
    if (options.radar_udp_enabled) {
        if (options.radar_port <= 0 || options.radar_port > 65535 || options.friendly_match_max_distance_m <= 0.0 ||
            options.friendly_state_timeout_ms <= 0) {
            SPDLOG_ERROR("雷达 UDP 或友机过滤参数非法");
            return 2;
        }
        radar_socket = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (radar_socket < 0) {
            SPDLOG_ERROR("创建雷达 UDP 套接字失败: {}", std::strerror(errno));
            return 2;
        }
        timeval timeout{};
        timeout.tv_usec = 200000;
        ::setsockopt(radar_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(static_cast<std::uint16_t>(options.radar_port));
        if (::inet_pton(AF_INET, options.radar_bind.c_str(), &address.sin_addr) != 1 ||
            ::bind(radar_socket, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
            SPDLOG_ERROR("绑定雷达 UDP {}:{} 失败: {}", options.radar_bind, options.radar_port, std::strerror(errno));
            ::close(radar_socket);
            return 2;
        }
        radar_running = true;
        radar_thread = std::thread([&]() {
            std::vector<char> buffer(65536);
            std::int64_t last_warning_ms = 0;
            std::int64_t last_statistics_ms = 0;
            const auto log_statistics_if_needed = [&](std::int64_t now_ms) {
                if (now_ms - last_statistics_ms < 5000) return;
                last_statistics_ms = now_ms;
                SPDLOG_INFO(
                    "雷达接收统计: UDP数据报={} radar_tracks有效扫描={} 无靶机输出扫描={} 已发布靶机={}",
                    radar_datagram_count.load(), radar_valid_scan_count.load(), radar_no_target_scan_count.load(),
                    radar_target_count.load());
            };
            while (radar_running && g_stop_requested == 0) {
                const ssize_t received = ::recvfrom(radar_socket, buffer.data(), buffer.size(), 0, nullptr, nullptr);
                if (received < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
                    SPDLOG_ERROR("接收雷达 UDP 失败: {}", std::strerror(errno));
                    break;
                }
                radar_datagram_count.fetch_add(1);
                std::vector<plane::model::GcsTarget> targets;
                std::string filter_error;
                const std::string payload(buffer.data(), static_cast<std::size_t>(received));
                if (!radar_filter.process_scan(payload, unix_now_ms(), targets, filter_error)) {
                    if (filter_error.rfind("忽略非 radar_tracks 报文", 0) == 0) {
                        SPDLOG_DEBUG("{}", filter_error);
                        log_statistics_if_needed(unix_now_ms());
                        continue;
                    }
                    const std::int64_t now = unix_now_ms();
                    if (now - last_warning_ms >= 1000) {
                        last_warning_ms = now;
                        SPDLOG_WARN("雷达扫描暂停转发: {}", filter_error);
                    }
                    log_statistics_if_needed(now);
                    continue;
                }
                radar_valid_scan_count.fetch_add(1);
                if (targets.empty()) {
                    radar_no_target_scan_count.fetch_add(1);
                }
                for (const auto& target : targets) {
                    try {
                        const std::string message_id = options.sender_id + "-radar-" + std::to_string(++sequence);
                        send_wire(session, "sendToPlane/" + options.plane_id,
                                  build_wire(options.sender_id, message_id, "GcsTarget", encode_target_payload(target)));
                        radar_target_count.fetch_add(1);
                    } catch (const std::exception& exception) {
                        SPDLOG_ERROR("雷达靶机数据发送失败: {}", exception.what());
                    }
                }
                log_statistics_if_needed(unix_now_ms());
                SPDLOG_DEBUG("雷达扫描处理完成: targets.size={}", targets.size());
            }
        });
        SPDLOG_INFO(
            "雷达 UDP 已监听: radar_bind={} radar_port={} friendly_state_timeout_ms={} "
            "friendly_match_max_distance_m={}",
            options.radar_bind, options.radar_port, options.friendly_state_timeout_ms,
            options.friendly_match_max_distance_m);
    }
    if (!options.query_type.empty()) {
        plane::model::Message reply;
        const auto& query_agent_id = options.query_agent_id.empty() ? options.plane_id : options.query_agent_id;
        if (!plane::communication::queryAgent(session, query_agent_id, reply, error,
                                              options.query_timeout_ms)) {
            SPDLOG_ERROR("查询失败: {}", error);
            return 2;
        }
        SPDLOG_INFO("AgentInfo 查询成功: {}", reply.payload);
    }

    std::signal(SIGINT, handle_stop_signal);
    std::signal(SIGTERM, handle_stop_signal);
    const std::string pod_id_prefix = options.sender_id + "-pod-" + std::to_string(unix_now_ms()) + "-" +
                                      std::to_string(::getpid()) + "-";
    const auto send_pod = [&](const plane::model::PodCommand& command) {
        std::string payload;
        std::string error;
        if (!plane::communication::to_payload(command, payload, error)) {
            throw std::runtime_error(error);
        }
        const std::string id = pod_id_prefix + std::to_string(++sequence);
        send_wire(session, "sendToPlane/" + options.plane_id,
                  plane::communication::encode_envelope(
                      {options.sender_id, id, plane::model::MessageType::PodCommand, std::move(payload)}));
        return id;
    };

    bool all_acked = true;
    bool self_check_executed = false;
    std::size_t command_index = 0;
    std::size_t task_index = 0;
    for (const auto& action : options.actions) {
        if (g_stop_requested != 0) return 1;
        if (g_stop_requested != 0) break;
        if (action.kind == ActionKind::PodCommand) {
            try {
                if (action.pod.type == plane::model::PodCommandType::SetRate) {
                    SPDLOG_INFO("开始发送吊舱速度: pitchRate_deg_s={} yawRate_deg_s={}", action.pod.pitchRate_deg_s,
                                action.pod.yawRate_deg_s);
                    auto next = std::chrono::steady_clock::now();
                    const auto end = next + action.pod_rate_duration;
                    do {
                        send_pod(action.pod);
                        next += std::chrono::milliseconds(100);
                        if (next <= std::chrono::steady_clock::now()) {
                            next = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
                        }
                        std::this_thread::sleep_until(std::min(end, next));
                    } while (g_stop_requested == 0 && std::chrono::steady_clock::now() < end);
                    if (action.pod_release) {
                        send_pod(plane::model::PodCommand{plane::model::PodCommandType::SetRate});
                        SPDLOG_INFO("已发送吊舱松手双零速度");
                    } else {
                        SPDLOG_INFO("吊舱速度已停发，等待机载超时停车");
                    }
                } else {
                    const std::string message_id = send_pod(action.pod);
                    std::unique_lock<std::mutex> lock(mutex);
                    const bool acked = cv.wait_for(lock, std::chrono::milliseconds(options.ack_timeout_ms),
                                                   [&]() { return acked_ids.count(message_id) > 0; });
                    SPDLOG_INFO("PodCommand 接收确认: messageId={} {}", message_id, acked ? "成功" : "超时");
                    all_acked = all_acked && acked;
                }
            } catch (const std::exception& exception) {
                SPDLOG_ERROR("吊舱指令发送失败: {}", exception.what());
                all_acked = false;
            }
            continue;
        }
        std::string type;
        std::string payload;
        const auto sequence_suffix = std::to_string(++sequence);
        const bool self_check = action.kind == ActionKind::Command && action.type == "SelfCheck";
        std::string message_id = options.sender_id + "-" + sequence_suffix;
        if (action.kind == ActionKind::Command) {
            type = "Command";
            if (self_check) {
                message_id = options.sender_id + "-self-check-" + std::to_string(unix_now_ms()) + "-" +
                             std::to_string(::getpid()) + "-" + sequence_suffix;
                payload = nlohmann::json{{"commandId", message_id}, {"type", action.type}}.dump();
                std::lock_guard<std::mutex> lock(mutex);
                pending_self_check = message_id;
                self_check_result.reset();
            } else {
                nlohmann::json waypoints = nlohmann::json::array();
                for (const auto& point : options.waypoints) {
                    waypoints.push_back({{"latitude", point.latitude},
                                         {"longitude", point.longitude},
                                         {"height", point.height},
                                         {"altitude", point.altitude}});
                }
                payload = nlohmann::json{{"commandId", "cmd-" + std::to_string(++command_index)},
                                         {"type", action.type},
                                         {"takeoffHeight_m", options.takeoff_height_m},
                                         {"waypoints", std::move(waypoints)}}
                              .dump();
            }
        } else {
            type = "TaskInfo";
            const std::string task_id =
                options.task_id.empty() ? "task-" + std::to_string(++task_index) : options.task_id;
            payload = nlohmann::json{{"taskId", task_id},
                                     {"taskCode", action.type},
                                     {"targetTrackId", options.target_track_id},
                                     {"taskAction", action.task_action}}
                          .dump();
        }
        send_typed(session, options, type, payload, message_id);
        std::unique_lock<std::mutex> lock(mutex);
        const bool acked = cv.wait_for(lock, std::chrono::milliseconds(options.ack_timeout_ms),
                                       [&]() { return acked_ids.count(message_id) > 0; });
        SPDLOG_INFO("{} {} 确认结果: {}", type, action.type, acked ? "成功" : "失败");
        if (!self_check) {
            all_acked = all_acked && acked;
        } else {
            const auto deadline = std::chrono::steady_clock::now() +
                                  std::chrono::milliseconds(options.self_check_timeout_ms);
            while (!self_check_result && g_stop_requested == 0 && std::chrono::steady_clock::now() < deadline) {
                const auto next_poll = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
                cv.wait_until(lock, std::min(next_poll, deadline));
            }
            if (!self_check_result) {
                SPDLOG_ERROR("未收到 SelfCheckResult: commandId={}，{}", message_id,
                             g_stop_requested == 0 ? "等待超时" : "等待已中止");
                return 1;
            }
            if (g_stop_requested != 0 || !self_check_result->success) return 1;
            pending_self_check.clear();
            self_check_executed = true;
        }
    }
    if (options.actions.empty()) {
        SPDLOG_INFO("未指定控制指令或任务，仅监听上报");
    }

    if (!options.gcs_target_id.empty()) {
        const std::int64_t first_timestamp_ms = unix_now_ms();
        for (int i = 0; i < options.gcs_target_repeat; ++i) {
            const plane::model::GcsTarget target{
                options.gcs_target_id, {30.0 + 0.0001 * i, 120.0, 50.0F, 60.0F}, first_timestamp_ms + i * 1000LL};
            const std::string message_id = options.gcs_target_id + "-" + std::to_string(++sequence);
            send_wire(session, "sendToPlane/" + options.plane_id,
                      build_wire(options.sender_id, message_id, "GcsTarget", encode_target_payload(target)));
        }
        SPDLOG_INFO("已发送 GCS 目标: trackId={} gcs_target_repeat={}", options.gcs_target_id,
                    options.gcs_target_repeat);
    }

    for (const auto& [key, value] : options.send_params) {
        double numeric = 0.0;
        try {
            numeric = std::stod(value);
        } catch (const std::exception&) {
            SPDLOG_WARN("忽略非法系统参数: {}={}", key, value);
            continue;
        }
        nlohmann::json payload = nlohmann::json::object();
        const std::string wire_key = (key == "safe_height_m") ? "safeHeight_m" : key;
        payload[wire_key] = numeric;
        const std::string message_id = options.sender_id + "-param-" + std::to_string(++sequence);
        send_wire(session, "sendToPlane/" + options.plane_id,
                  build_wire(options.sender_id, message_id, "SystemParam", payload.dump()));
    }
    if (!options.send_params.empty()) {
        SPDLOG_INFO("已发送系统参数: {} 条", options.send_params.size());
    }

    if (!options.peer_plane_id.empty()) {
        const nlohmann::json payload{
            {"planeId", options.peer_plane_id},
            {"geoPos", {{"latitude", 30.5}, {"longitude", 120.5}, {"height", 80.0F}, {"altitude", 90.0F}}},
            {"voltage_v", 12.5F},
            {"airspeed_m_s", 15.0F},
            {"flightMode", "AUTO.MISSION"},
            {"fcLinkState", "connected"}};
        const std::string message_id = options.peer_plane_id + "-" + std::to_string(++sequence);
        send_wire(session, std::string(plane::model::topic_sendToAll),
                  build_wire(options.peer_plane_id, message_id, "VehicleState", payload.dump()));
        SPDLOG_INFO("已发送友机状态: planeId={}", options.peer_plane_id);
    }

    const auto listen_start = std::chrono::steady_clock::now();
    const int listen_seconds = options.listen_seconds.value_or(0);
    while (g_stop_requested == 0 && (!self_check_executed || options.listen_seconds.has_value())) {
        if (agent_info_pending.exchange(false)) publish_agent_info();
        if (listen_seconds > 0 &&
            std::chrono::steady_clock::now() - listen_start >= std::chrono::seconds(listen_seconds)) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    radar_cleanup.stop();
    SPDLOG_INFO("状态统计: vehicle_state_count={} task_state_count={}", vehicle_state_count.load(),
                task_state_count.load());
    SPDLOG_INFO("收到 PodState 共 {} 条", pod_state_count.load());
    SPDLOG_INFO("信息统计: agent_info_count={}", agent_info_count.load());
    SPDLOG_INFO("雷达统计: UDP数据报={} radar_tracks有效扫描={} 无靶机输出扫描={} 已发布靶机={}",
                radar_datagram_count.load(), radar_valid_scan_count.load(), radar_no_target_scan_count.load(),
                radar_target_count.load());
    return all_acked ? 0 : 1;
}
