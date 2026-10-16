#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <nlohmann/json.hpp>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <zenoh/api.hxx>

#include "message.h"

namespace {

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
    std::string sender_id = "gcs";
    std::string peer_plane_id;
    bool no_ack = false;
    int listen_seconds = 3;
    int ack_timeout_ms = 2000;
    bool help = false;
    std::vector<Action> actions;
    std::string target_track_id;
    float takeoff_height_m = 10.0F;
    std::vector<plane::model::GeoPoint> waypoints;
    std::string gcs_target_id;
    int gcs_target_repeat = 1;
    std::vector<std::pair<std::string, std::string>> send_params;
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
        "  --sender-id <id>         本机(GCS)标识，需与飞机 station_id 一致 (默认: gcs)\n"
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
        "  --anti-uav               下发自杀式反无任务 (TaskAction=Start)\n"
        "  --stop-task              终止当前任务 (TaskAction=Stop)\n"
        "  --target-track-id <id>   任务关联的目标跟踪 id (可空)\n"
        "  --task-code <code>       任务类型，兼容项 (默认: anti-uav)\n"
        "  --task-action <action>   任务动作 Start/Stop，兼容项\n"
        "\n"
        "其他:\n"
        "  --listen-seconds <s>     发送后监听时长 (默认: 3)\n"
        "  --ack-timeout-ms <ms>    等待 Ack 超时 (默认: 2000)\n"
        "  --no-ack                 不回复收到的告警/Ack\n"
        "  --send-gcs-target <id>   下发地面站目标轨迹\n"
        "  --gcs-target-repeat <n>  地面站目标重复发送次数 (默认: 1)\n"
        "  --send-param <k=v>       下发系统参数，可多次 (如 safe_height_m=30)\n"
        "  --send-peer-state <id>   模拟友机状态上报\n"
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
    std::string pending_task_code = "anti-uav";

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            options.help = true;
            return options;
        }
        if (arg == "--plane-id" && i + 1 < argc) {
            options.plane_id = argv[++i];
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
        } else if (arg == "--target-track-id" && i + 1 < argc) {
            options.target_track_id = argv[++i];
        } else if (arg == "--send-gcs-target" && i + 1 < argc) {
            options.gcs_target_id = argv[++i];
        } else if (arg == "--gcs-target-repeat" && i + 1 < argc) {
            options.gcs_target_repeat = std::stoi(argv[++i]);
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

std::string send_typed(const zenoh::Session& session, const Options& options, const std::string& type,
                       const std::string& payload, std::uint64_t sequence) {
    const std::string message_id = options.sender_id + "-" + std::to_string(sequence);
    const std::string topic = "sendToPlane/" + options.plane_id;
    session.put(zenoh::KeyExpr(topic), zenoh::Bytes(build_wire(options.sender_id, message_id, type, payload)));
    SPDLOG_INFO("已发送 {}: messageId={}", type, message_id);
    return message_id;
}

}  // namespace

int main(int argc, char** argv) {
    const Options options = parse_options(argc, argv);
    if (options.help) {
        print_usage();
        return 0;
    }

    std::mutex mutex;
    std::condition_variable cv;
    std::set<std::string> acked_ids;
    std::atomic<std::uint64_t> sequence{100};

    zenoh::Session session = zenoh::Session::open(zenoh::Config::create_default());

    std::atomic<std::uint64_t> vehicle_state_count{0};
    std::atomic<std::uint64_t> task_state_count{0};

    auto on_sample = [&](zenoh::Sample& sample) {
        Envelope envelope;
        if (!parse_envelope(sample.get_payload().as_string(), envelope)) {
            SPDLOG_WARN("收到无法解析的报文");
            return;
        }
        if (envelope.sender_id == options.sender_id) {
            return;
        }
        if (envelope.type == "VehicleState") {
            const nlohmann::json payload = nlohmann::json::parse(envelope.payload, nullptr, false);
            if (!payload.is_discarded()) {
                const nlohmann::json geo = payload.value("geoPos", nlohmann::json::object());
                SPDLOG_INFO(
                    "收到 VehicleState: planeId={}, fcLinkState={}, flightMode={}, voltage={}, lat={}, lon={}, "
                    "height={}",
                    payload.value("planeId", std::string{}), payload.value("fcLinkState", std::string{}),
                    payload.value("flightMode", std::string{}), payload.value("voltage_v", 0.0F),
                    geo.value("latitude", 0.0), geo.value("longitude", 0.0), geo.value("height", 0.0F));
            }
            vehicle_state_count.fetch_add(1);
            return;
        }
        if (envelope.type == "TaskState") {
            const nlohmann::json payload = nlohmann::json::parse(envelope.payload, nullptr, false);
            if (!payload.is_discarded()) {
                SPDLOG_INFO("收到 TaskState: taskId={}, status={}, node={}", payload.value("taskId", std::string{}),
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
        SPDLOG_INFO("收到 {}: senderId={}, messageId={}", envelope.type, envelope.sender_id, envelope.message_id);
        if (!options.no_ack && (envelope.type == "Alert" || envelope.type == "Ack")) {
            const nlohmann::json payload{{"ackId", envelope.message_id}};
            const std::string message_id = options.sender_id + "-ack-" + std::to_string(++sequence);
            const std::string topic = "sendToPlane/" + envelope.sender_id;
            session.put(zenoh::KeyExpr(topic),
                        zenoh::Bytes(build_wire(options.sender_id, message_id, "Ack", payload.dump())));
        }
    };

    zenoh::Subscriber<void> station_subscriber =
        session.declare_subscriber(zenoh::KeyExpr(std::string(plane::model::topic_sendToStation)), on_sample, []() {});
    zenoh::Subscriber<void> all_subscriber =
        session.declare_subscriber(zenoh::KeyExpr(std::string(plane::model::topic_sendToAll)), on_sample, []() {});

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
            payload = nlohmann::json{{"taskId", "task-" + std::to_string(++task_index)},
                                     {"taskCode", action.type},
                                     {"targetTrackId", options.target_track_id},
                                     {"taskAction", action.task_action}}
                          .dump();
        }
        const std::string message_id = send_typed(session, options, type, payload, ++sequence);
        std::unique_lock<std::mutex> lock(mutex);
        const bool acked = cv.wait_for(lock, std::chrono::milliseconds(options.ack_timeout_ms),
                                       [&]() { return acked_ids.count(message_id) > 0; });
        SPDLOG_INFO("{} {} Ack: {}", type, action.type, acked);
        all_acked = all_acked && acked;
    }
    if (options.actions.empty()) {
        SPDLOG_INFO("未指定控制指令或任务，仅监听上报");
    }

    if (!options.gcs_target_id.empty()) {
        for (int i = 0; i < options.gcs_target_repeat; ++i) {
            const std::string payload = nlohmann::json{
                {"trackId", options.gcs_target_id},
                {"geoPos",
                 {{"latitude", 30.0 + 0.0001 * i}, {"longitude", 120.0}, {"height", 50.0F}, {"altitude", 60.0F}}},
                {"velocity_m_s", 10.0F},
                {"direction_deg", 90.0F}}.dump();
            const std::string message_id = options.gcs_target_id + "-" + std::to_string(++sequence);
            session.put(zenoh::KeyExpr("sendToPlane/" + options.plane_id),
                        zenoh::Bytes(build_wire(options.sender_id, message_id, "GcsTarget", payload)));
        }
        SPDLOG_INFO("已发送 GcsTarget: trackId={}, count={}", options.gcs_target_id, options.gcs_target_repeat);
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
        session.put(zenoh::KeyExpr("sendToPlane/" + options.plane_id),
                    zenoh::Bytes(build_wire(options.sender_id, message_id, "SystemParam", payload.dump())));
    }
    if (!options.send_params.empty()) {
        SPDLOG_INFO("已发送 SystemParam: {} 条", options.send_params.size());
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
        session.put(zenoh::KeyExpr("sendToPlane/" + options.plane_id),
                    zenoh::Bytes(build_wire(options.peer_plane_id, message_id, "VehicleState", payload.dump())));
        SPDLOG_INFO("已发送友机状态: planeId={}", options.peer_plane_id);
    }

    std::this_thread::sleep_for(std::chrono::seconds(options.listen_seconds));
    SPDLOG_INFO("状态统计: VehicleState={}, TaskState={}", vehicle_state_count.load(), task_state_count.load());
    return all_acked ? 0 : 1;
}
