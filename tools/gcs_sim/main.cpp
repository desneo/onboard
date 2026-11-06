#include <arpa/inet.h>
#include <netinet/in.h>
#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <nlohmann/json.hpp>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "communication/plane_query.h"
#include "communication/serialization.h"
#include "communication/zenoh_transport.h"
#include "core/log.h"
#include "message.h"
#include "tools/gcs_sim/radar_target_filter.h"

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;
std::mutex g_send_mutex;

void handle_stop_signal(int) { g_stop_requested = 1; }

std::int64_t unix_now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
        .count();
}

/// 一次待发送的控制内容：基础指令或任务，按命令行出现顺序执行。
enum class ActionKind {
    Command,
    Task,
};

struct Action {
    ActionKind kind = ActionKind::Command;
    std::string type;         ///< CommandType 或 TaskCode
    std::string task_action;  ///< TaskAction，仅 Task 使用
};

struct Options {
    std::string plane_id = "uav1";
    std::string sender_id = "GCS";
    std::string peer_plane_id;
    bool no_ack = false;
    int listen_seconds = 0;
    int ack_timeout_ms = 2000;
    bool help = false;
    std::string query_type;
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
        "  --sender-id <id>         本机(GCS)标识，需与飞机 station_id 一致 (默认: GCS)\n"
        "  -h, --help               显示本帮助并退出\n"
        "\n"
        "控制指令 (按出现顺序发往 sendToPlane/<plane-id>):\n"
        "  --arm                    解锁 (Arm)\n"
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
        "任务 (按出现顺序发往 sendToPlane/<plane-id>):\n"
        "  --anti-uav               下发反无人机中制导任务 (TaskAction=Start)\n"
        "  --stop-task              终止当前任务 (TaskAction=Stop)\n"
        "  --task-id <id>           指定 Start/Stop 共用的任务 id\n"
        "  --target-track-id <id>   任务关联的目标跟踪 id (可空)\n"
        "  --task-code <code>       任务类型，兼容项 (默认: AntiUav)\n"
        "  --task-action <action>   任务动作 Start/Stop，兼容项\n"
        "\n"
        "信息查询与连接:\n"
        "  --query <MessageType>    原生查询指定飞机，当前支持 VehicleInfo\n"
        "  --query-timeout-ms <ms>  查询超时（默认 2000）\n"
        "  --no-presence            不声明地面站存活标记（仅查询场景）\n"
        "  --zenoh-config <path>    Zenoh 配置文件\n"
        "  --connect <endpoint>     远端端点，多个用逗号分隔\n"
        "  --listen <endpoint>      本地监听端点\n"
        "\n"
        "其他:\n"
        "  --listen-seconds <s>     运行时长；0 表示持续运行 (默认: 0)\n"
        "  --ack-timeout-ms <ms>    等待 Ack 超时 (默认: 2000)\n"
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
        "  gcs_sim --plane-id uav1 --anti-uav --target-track-id t-1\n"
        "  gcs_sim --plane-id uav1 --arm --takeoff --takeoff-height 20 --anti-uav\n";
    spdlog::info("{}", usage);
}

bool parse_envelope(const std::string& wire, Envelope& out) {
    const nlohmann::json envelope = nlohmann::json::parse(wire, nullptr, false);
    if (envelope.is_discarded() || !envelope.is_object()) {
        return false;
    }
    out.sender_id = envelope.value("senderId", std::string{});
    out.message_id = envelope.value("messageId", std::string{});
    out.type = envelope.value("type", std::string{});
    out.payload = envelope.value("payload", std::string{});
    return !out.message_id.empty() && !out.type.empty();
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
        } else if (arg == "--send-peer-state" && i + 1 < argc) {
            options.peer_plane_id = argv[++i];
        } else if (arg == "--no-ack") {
            options.no_ack = true;
        } else if (arg == "--listen-seconds" && i + 1 < argc) {
            options.listen_seconds = std::stoi(argv[++i]);
        } else if (arg == "--ack-timeout-ms" && i + 1 < argc) {
            options.ack_timeout_ms = std::stoi(argv[++i]);
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

void send_wire(plane::communication::ZenohTransport& session, const std::string& topic, const std::string& wire) {
    std::string error;
    std::lock_guard<std::mutex> lock(g_send_mutex);
    if (!session.put(topic, wire, error)) throw std::runtime_error(error);
}

std::string send_typed(plane::communication::ZenohTransport& session, const Options& options, const std::string& type,
                       const std::string& payload, std::uint64_t sequence) {
    const std::string message_id = options.sender_id + "-" + std::to_string(sequence);
    const std::string topic = "sendToPlane/" + options.plane_id;
    send_wire(session, topic, build_wire(options.sender_id, message_id, type, payload));
    SPDLOG_INFO("已发送 {}: messageId={}", type, message_id);
    return message_id;
}

}  // namespace

int main(int argc, char** argv) {
    plane::core::set_project_log_pattern();
    const Options options = parse_options(argc, argv);
    if (options.help) {
        print_usage();
        return 0;
    }

    std::mutex mutex;
    std::condition_variable cv;
    std::set<std::string> acked_ids;
    std::atomic<std::uint64_t> sequence{100};

    plane::communication::ZenohTransport session;
    std::string error;
    if (!session.open(options.endpoints, error)) {
        SPDLOG_ERROR("{}", error);
        return 2;
    }

    std::atomic<std::uint64_t> vehicle_state_count{0};
    std::atomic<std::uint64_t> task_state_count{0};
    std::atomic<std::uint64_t> vehicle_info_count{0};
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
    } close_session{session};

    auto on_sample = [&](const std::string& wire, bool from_broadcast) {
        Envelope envelope;
        if (!parse_envelope(wire, envelope)) {
            SPDLOG_WARN("收到无法解析的报文");
            return;
        }
        if (envelope.sender_id == options.sender_id) {
            return;
        }
        if (envelope.type == "VehicleInfo") {
            plane::model::VehicleInfo info;
            std::string decode_error;
            if (plane::communication::from_payload(envelope.payload, info, decode_error) &&
                envelope.sender_id == info.planeId && info.planeId == options.plane_id) {
                vehicle_info_count.fetch_add(1);
                SPDLOG_INFO("收到飞机信息（VehicleInfo）: {}", envelope.payload);
            } else {
                SPDLOG_WARN("飞机信息（VehicleInfo）无效或非目标飞机: {}", decode_error);
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

    if (!session.subscribe(
            std::string(plane::model::topic_sendToStation), [&](const std::string& wire) { on_sample(wire, false); },
            error) ||
        !session.subscribe(
            std::string(plane::model::topic_sendToAll), [&](const std::string& wire) { on_sample(wire, true); },
            error)) {
        SPDLOG_ERROR("{}", error);
        return 2;
    }
    // 必须先建立上行订阅，再宣告地面站可接收 VehicleInfo。
    if (options.announce_presence &&
        !session.declare_liveliness_token(plane::communication::station_liveliness_key(options.sender_id), error)) {
        SPDLOG_ERROR("{}", error);
        return 2;
    }
    SPDLOG_INFO("地面站订阅已就绪: sender_id={}", options.sender_id);

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
                std::vector<plane::target::GcsTarget> targets;
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
                    std::string target_payload;
                    std::string encode_error;
                    if (!plane::communication::to_payload(target, target_payload, encode_error)) {
                        SPDLOG_ERROR("GCS 目标序列化失败: {}", encode_error);
                        continue;
                    }
                    try {
                        const std::string message_id = options.sender_id + "-radar-" + std::to_string(++sequence);
                        send_wire(session, "sendToPlane/" + options.plane_id,
                                  build_wire(options.sender_id, message_id, "GcsTarget", target_payload));
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
        plane::model::MessageType type;
        plane::model::Message reply;
        if (!plane::communication::message_type_from_string(options.query_type, type)) {
            SPDLOG_ERROR("不支持的查询类型: {}", options.query_type);
            return 2;
        }
        if (!plane::communication::queryPlane(session, options.plane_id, type, reply, error,
                                              options.query_timeout_ms)) {
            SPDLOG_ERROR("查询失败: {}", error);
            return 2;
        }
        SPDLOG_INFO("飞机信息查询成功: {}", reply.payload);
    }

    bool all_acked = true;
    std::size_t command_index = 0;
    std::size_t task_index = 0;
    for (const auto& action : options.actions) {
        std::string type;
        std::string payload;
        if (action.kind == ActionKind::Command) {
            type = "Command";
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
        const std::string message_id = send_typed(session, options, type, payload, ++sequence);
        std::unique_lock<std::mutex> lock(mutex);
        const bool acked = cv.wait_for(lock, std::chrono::milliseconds(options.ack_timeout_ms),
                                       [&]() { return acked_ids.count(message_id) > 0; });
        SPDLOG_INFO("{} {} 确认结果: {}", type, action.type, acked ? "成功" : "失败");
        all_acked = all_acked && acked;
    }
    if (options.actions.empty()) {
        SPDLOG_INFO("未指定控制指令或任务，仅监听上报");
    }

    if (!options.gcs_target_id.empty()) {
        const std::int64_t first_timestamp_ms = unix_now_ms();
        for (int i = 0; i < options.gcs_target_repeat; ++i) {
            const std::string payload = nlohmann::json{
                {"trackId", options.gcs_target_id},
                {"geoPos",
                 {{"latitude", 30.0 + 0.0001 * i}, {"longitude", 120.0}, {"height", 50.0F}, {"altitude", 60.0F}}},
                {"timestamp_ms", first_timestamp_ms + i * 1000},
                {"radial_speed_m_s", 10.0F},
                {"azimuth_deg", 90.0F}}.dump();
            const std::string message_id = options.gcs_target_id + "-" + std::to_string(++sequence);
            send_wire(session, "sendToPlane/" + options.plane_id,
                      build_wire(options.sender_id, message_id, "GcsTarget", payload));
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

    std::signal(SIGINT, handle_stop_signal);
    std::signal(SIGTERM, handle_stop_signal);
    const auto listen_start = std::chrono::steady_clock::now();
    while (g_stop_requested == 0) {
        if (options.listen_seconds > 0 &&
            std::chrono::steady_clock::now() - listen_start >= std::chrono::seconds(options.listen_seconds)) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    radar_cleanup.stop();
    SPDLOG_INFO("状态统计: vehicle_state_count={} task_state_count={}", vehicle_state_count.load(),
                task_state_count.load());
    SPDLOG_INFO("信息统计: vehicle_info_count={}", vehicle_info_count.load());
    SPDLOG_INFO("雷达统计: UDP数据报={} radar_tracks有效扫描={} 无靶机输出扫描={} 已发布靶机={}",
                radar_datagram_count.load(), radar_valid_scan_count.load(), radar_no_target_scan_count.load(),
                radar_target_count.load());
    // 先停止所有回调，之后再析构被回调引用的计数器。
    session.close();
    return all_acked ? 0 : 1;
}
