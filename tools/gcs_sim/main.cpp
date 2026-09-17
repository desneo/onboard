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
#include <zenoh/api.hxx>

#include "message.h"

namespace {

struct Options {
    std::string plane_id = "uav1";
    std::string sender_id = "gcs";
    bool no_ack = false;
    int listen_seconds = 3;
    int ack_timeout_ms = 2000;
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
        } else if (arg == "--no-ack") {
            options.no_ack = true;
        } else if (arg == "--listen-seconds" && i + 1 < argc) {
            options.listen_seconds = std::stoi(argv[++i]);
        } else if (arg == "--ack-timeout-ms" && i + 1 < argc) {
            options.ack_timeout_ms = std::stoi(argv[++i]);
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
    zenoh::Subscriber<void> subscriber = session.declare_subscriber(
        zenoh::KeyExpr(std::string(plane::model::topic_sendToStation)),
        [&](zenoh::Sample& sample) {
            Envelope envelope;
            if (!parse_envelope(sample.get_payload().as_string(), envelope)) {
                spdlog::warn("收到无法解析的报文");
                return;
            }
            if (envelope.sender_id == options.sender_id) {
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
        },
        []() {});

    const std::string task_payload = nlohmann::json{
        {"taskId", "task-1"},
        {"taskCode", "suicide"},
        {"targetTrackId", ""},
        {"taskAction", "Start"}}.dump();
    const std::string task_id = send_typed(session, options, "TaskInfo", task_payload, ++sequence);

    const std::string command_payload = nlohmann::json{
        {"commandId", "cmd-1"},
        {"type", "Arm"},
        {"takeoffHeight_m", 0.0F},
        {"waypoints",
         nlohmann::json::array()}}.dump();
    const std::string command_id = send_typed(session, options, "Command", command_payload, ++sequence);

    bool task_acked = false;
    bool command_acked = false;
    {
        std::unique_lock<std::mutex> lock(mutex);
        task_acked = cv.wait_for(lock, std::chrono::milliseconds(options.ack_timeout_ms),
                                 [&]() { return acked_ids.count(task_id) > 0; });
        command_acked = cv.wait_for(lock, std::chrono::milliseconds(options.ack_timeout_ms),
                                    [&]() { return acked_ids.count(command_id) > 0; });
    }
    spdlog::info("Ack 结果: TaskInfo={}, Command={}", task_acked, command_acked);

    std::this_thread::sleep_for(std::chrono::seconds(options.listen_seconds));
    return task_acked && command_acked ? 0 : 1;
}
