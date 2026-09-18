#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <zenoh/api.hxx>

#include "message.h"

namespace {

struct Options {
    std::string plane_id = "uav1";
    std::string sender_id = "gcs";
    std::string peer_plane_id;
    bool no_ack = false;
    int listen_seconds = 3;
    int ack_timeout_ms = 2000;
    bool send_task = true;
    bool send_command = true;
    std::string task_action = "Start";
    std::string task_code = "suicide";
    std::string command_type = "Arm";
    float takeoff_height_m = 0.0F;
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
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
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
        } else if (arg == "--no-command") {
            options.send_command = false;
        } else if (arg == "--no-task") {
            options.send_task = false;
        } else if (arg == "--task-action" && i + 1 < argc) {
            options.task_action = argv[++i];
        } else if (arg == "--task-code" && i + 1 < argc) {
            options.task_code = argv[++i];
        } else if (arg == "--command-type" && i + 1 < argc) {
            options.command_type = argv[++i];
        } else if (arg == "--takeoff-height" && i + 1 < argc) {
            options.takeoff_height_m = std::stof(argv[++i]);
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
    spdlog::info("已发送 {}: messageId={}", type, message_id);
    return message_id;
}

}  // namespace

int main(int argc, char** argv) {
    const Options options = parse_options(argc, argv);

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
            spdlog::warn("收到无法解析的报文");
            return;
        }
        if (envelope.sender_id == options.sender_id) {
            return;
        }
        if (envelope.type == "VehicleState") {
            const nlohmann::json payload = nlohmann::json::parse(envelope.payload, nullptr, false);
            if (!payload.is_discarded()) {
                const nlohmann::json geo = payload.value("geoPos", nlohmann::json::object());
                spdlog::info(
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
                spdlog::info("收到 TaskState: taskId={}, status={}, node={}", payload.value("taskId", std::string{}),
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
        spdlog::info("收到 {}: senderId={}, messageId={}", envelope.type, envelope.sender_id, envelope.message_id);
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

    bool task_acked = true;
    if (options.send_task) {
        const std::string task_payload = nlohmann::json{
            {"taskId", "task-1"},
            {"taskCode", options.task_code},
            {"targetTrackId", ""},
            {"taskAction",
             options.task_action}}.dump();
        const std::string task_id = send_typed(session, options, "TaskInfo", task_payload, ++sequence);
        std::unique_lock<std::mutex> lock(mutex);
        task_acked = cv.wait_for(lock, std::chrono::milliseconds(options.ack_timeout_ms),
                                 [&]() { return acked_ids.count(task_id) > 0; });
    }
    bool command_acked = true;
    if (options.send_command) {
        const std::string command_payload = nlohmann::json{
            {"commandId", "cmd-1"},
            {"type", options.command_type},
            {"takeoffHeight_m", options.takeoff_height_m},
            {"waypoints",
             nlohmann::json::array()}}.dump();
        const std::string command_id = send_typed(session, options, "Command", command_payload, ++sequence);
        std::unique_lock<std::mutex> lock(mutex);
        command_acked = cv.wait_for(lock, std::chrono::milliseconds(options.ack_timeout_ms),
                                    [&]() { return acked_ids.count(command_id) > 0; });
    }
    spdlog::info("Ack 结果: TaskInfo={}, Command={}", task_acked, command_acked);

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
        spdlog::info("已发送 GcsTarget: trackId={}, count={}", options.gcs_target_id, options.gcs_target_repeat);
    }

    for (const auto& [key, value] : options.send_params) {
        double numeric = 0.0;
        try {
            numeric = std::stod(value);
        } catch (const std::exception&) {
            spdlog::warn("忽略非法系统参数: {}={}", key, value);
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
        spdlog::info("已发送 SystemParam: {} 条", options.send_params.size());
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
        spdlog::info("已发送友机状态: planeId={}", options.peer_plane_id);
    }

    std::this_thread::sleep_for(std::chrono::seconds(options.listen_seconds));
    spdlog::info("状态统计: VehicleState={}, TaskState={}", vehicle_state_count.load(), task_state_count.load());
    return task_acked && command_acked ? 0 : 1;
}
