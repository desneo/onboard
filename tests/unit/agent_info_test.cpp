#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "communication/communication_events.h"
#include "communication/communication_module.h"
#include "communication/plane_query.h"
#include "communication/serialization.h"
#include "core/config.h"
#include "state/state_store.h"

namespace {
void require(bool condition, const std::string& detail) {
    if (!condition) throw std::runtime_error(detail);
}

int available_port() {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    require(fd >= 0, "socket failed");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    int bound = bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address));
    socklen_t size = sizeof(address);
    int named = getsockname(fd, reinterpret_cast<sockaddr*>(&address), &size);
    close(fd);
    require(bound == 0 && named == 0, "bind failed");
    return ntohs(address.sin_port);
}

struct Tasks : plane::task::TaskStateProvider {
    plane::model::TaskState current_task_state() const override { return {}; }
};
}  // namespace

int main() {
    using namespace plane::communication;
    using plane::model::AgentInfo;
    using plane::model::AgentType;
    using plane::model::Message;
    using plane::model::MessageType;
    static_assert(static_cast<int>(MessageType::VehicleState) == 7, "既有类型序号改变");
    static_assert(static_cast<int>(MessageType::AgentInfo) == 8, "AgentInfo 序号错误");
    char path[] = "/tmp/plane-agent-info-XXXXXX";
    require(mkdtemp(path) != nullptr, "temporary directory failed");
    struct Cleanup {
        const char* path;
        ~Cleanup() { std::filesystem::remove_all(path); }
    } cleanup{path};
    std::string zenoh_config = std::string(path) + "/zenoh.json5";
    {
        std::ofstream file(zenoh_config);
        file
            << R"({"mode":"peer","scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}},"listen":{"endpoints":[]}})";
    }
    const std::string config_path = std::string(path) + "/plane.ini";
    const std::string profile_path = std::string(path) + "/profile.ini";
    {
        std::ofstream file(config_path);
        file << "[deployment]\nprofile = " << profile_path << '\n';
    }
    {
        std::ofstream file(profile_path);
        file << "[metadata-test]\nboard_ip = 192.168.1.100\npod_video_url = rtsp://pod/live?a=1&b=2\n"
             << "fc_url = udpout://127.0.0.1:14580\n";
    }
    plane::core::AppConfig config;
    const std::vector<std::string> local_addresses{"192.168.1.100"};
    require(plane::core::load_config(config_path, config, &local_addresses), "load config failed");
    require(config.plane_id == "metadata-test" && config.board_ip == "192.168.1.100" &&
                config.pod_video_url == "rtsp://pod/live?a=1&b=2" && config.is_sim,
            "metadata config mismatch");
    config.state_report_period_ms = 100;
    config.zenoh_config_file = zenoh_config;

    std::string error;
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<AgentInfo> pushes;
    std::vector<AgentInfo> received;
    std::unordered_map<std::string, std::size_t> station_pushes;
    std::vector<std::string> message_ids;
    std::vector<std::string> ack_ids;
    bool command_received = false;
    bool task_received = false;
    ZenohTransport observer;
    ZenohEndpoints endpoints;
    endpoints.config_file = zenoh_config;
    endpoints.listen = "tcp/127.0.0.1:" + std::to_string(available_port());
    require(observer.open(endpoints, error), error);
    require(observer.subscribe(
                "sendToStation",
                [&](const std::string& wire) {
                    plane::model::Message envelope;
                    std::string error;
                    plane::model::Ack ack;
                    if (!decode_envelope(wire, envelope, error) || envelope.type != MessageType::Ack ||
                        !from_payload(envelope.payload, ack, error)) return;
                    std::lock_guard<std::mutex> lock(mutex);
                    ack_ids.push_back(ack.ackId);
                    changed.notify_all();
                },
                error),
            error);
    require(observer.subscribe(
                "sendToAll",
                [&](const std::string& wire) {
                    Message envelope;
                    std::string error;
                    AgentInfo info;
                    if (!decode_envelope(wire, envelope, error) || envelope.type != MessageType::AgentInfo ||
                        envelope.senderId != config.plane_id || !from_payload(envelope.payload, info, error)) return;
                    std::lock_guard<std::mutex> lock(mutex);
                    pushes.push_back(info);
                    message_ids.push_back(envelope.messageId);
                    changed.notify_all();
                },
                error),
            error);
    auto wait_count = [&](std::size_t count) {
        std::unique_lock<std::mutex> lock(mutex);
        require(changed.wait_for(lock, std::chrono::seconds(4), [&]() { return pushes.size() >= count; }),
                "缺少 AgentInfo 上报");
        require(pushes.size() == count, "AgentInfo 重复上报");
        require(pushes.back().agentId == config.plane_id && pushes.back().agentType == AgentType::Vehicle &&
                    pushes.back().boardIp == config.board_ip && pushes.back().videoUrl == config.pod_video_url &&
                    pushes.back().isSim == config.is_sim && !message_ids.back().empty(),
                "上报的节点信息错误");
    };
    auto quiet = [&](std::size_t count) {
        std::unique_lock<std::mutex> lock(mutex);
        require(!changed.wait_for(lock, std::chrono::milliseconds(400), [&]() { return pushes.size() != count; }),
                "unexpected periodic or repeated push");
    };
    config.zenoh_connect = endpoints.listen;
    config.zenoh_listen = "tcp/127.0.0.1:" + std::to_string(available_port());
    plane::core::EventBus bus;
    bus.subscribe<AgentInfo>(events::kAgentInfoReceived, [&](const AgentInfo& info) {
        std::lock_guard<std::mutex> lock(mutex);
        received.push_back(info);
        changed.notify_all();
    });
    bus.subscribe<plane::model::Command>(events::kCommandReceived, [&](const plane::model::Command& command) {
        std::lock_guard<std::mutex> lock(mutex);
        command_received = command.commandId == "control-1" && command.type == plane::model::CommandType::Hover;
    });
    bus.subscribe<plane::model::TaskInfo>(events::kTaskInfoReceived, [&](const plane::model::TaskInfo& task) {
        std::lock_guard<std::mutex> lock(mutex);
        task_received = task.taskId == "task-1" && task.taskAction == plane::model::TaskAction::Stop;
    });
    plane::state::StateStore store(bus, config);
    Tasks tasks;
    plane::communication::CommunicationModule module(bus, config, store, tasks);
    require(module.init() && module.start(), "communication start failed");
    wait_count(1);  // 没有 GCS token 时，启动仍主动上报。
    quiet(1);

    const std::string command_topic = "sendToPlane/" + config.plane_id;
    auto send_and_wait_for_ack = [&](const plane::model::Message& message) {
        require(observer.put(command_topic, encode_envelope(message), error), error);
        std::unique_lock<std::mutex> lock(mutex);
        require(changed.wait_for(lock, std::chrono::seconds(4), [&]() {
                    return !ack_ids.empty() && ack_ids.back() == message.messageId;
                }),
                "control Ack missing on sendToStation");
    };
    send_and_wait_for_ack({config.plane_id, "command-message", MessageType::Command,
                           R"({"commandId":"control-1","type":"Hover"})"});
    send_and_wait_for_ack({"task-sender", "task-message", MessageType::TaskInfo,
                           R"({"taskId":"task-1","taskCode":"AntiUav","taskAction":"Stop"})"});
    {
        std::lock_guard<std::mutex> lock(mutex);
        require(command_received && task_received, "valid control messages from different senders not delivered");
    }
    require(store.gcs_link().connected, "valid controls did not update station activity");

    Message reply;
    require(queryAgent(observer, config.plane_id, reply, error), error);
    AgentInfo info;
    require(from_payload(reply.payload, info, error) && info.agentType == AgentType::Vehicle &&
                info.boardIp == config.board_ip && info.videoUrl == config.pod_video_url && info.isSim,
            "查询结果与本机配置不一致");
    require(!queryAgent(observer, "*", reply, error) && !error.empty(), "接受通配 agentId");
    require(!queryAgent(observer, "invalid/id", reply, error) && !error.empty(), "接受多段 agentId");
    require(!queryAgent(observer, config.plane_id, reply, error, 0), "接受无效超时");
    require(!queryAgent(observer, "missing-agent", reply, error, 100) && !error.empty(), "不存在的节点查询成功");
    std::string raw;
    require(!observer.query("queryToAgent/" + config.plane_id + "/Unknown", raw, error, 500),
            "接受无效查询地址");
    require(!observer.query("queryToAgent/*/AgentInfo", raw, error, 500), "接受通配查询地址");
    require(!from_payload(R"({"agentId":"bad","agentType":"Unknown","boardIp":"","videoUrl":"","isSim":true})",
                          info, error), "接受未知 AgentType");
    require(!from_payload(R"({"agentId":"bad","agentType":"Vehicle","boardIp":"","videoUrl":"","isSim":"true"})",
                          info, error), "接受字符串 isSim");
    require(!from_payload(R"({"agentId":"bad","agentType":"Vehicle","boardIp":"","videoUrl":""})", info, error),
            "接受缺字段 AgentInfo");

    ZenohTransport station_a;
    ZenohTransport station_b;
    ZenohEndpoints station_endpoints;
    station_endpoints.config_file = zenoh_config;
    station_endpoints.connect = config.zenoh_listen;
    const AgentInfo a_info{"test-station-a", AgentType::GroundStation, "192.168.1.2", "", false};
    const AgentInfo b_info{"test-station-b", AgentType::GroundStation, "", "", false};
    auto agent_message = [&](const AgentInfo& value, const std::string& message_id) {
        Message message{value.agentId, message_id, MessageType::AgentInfo, {}};
        require(to_payload(value, message.payload, error), error);
        return message;
    };
    auto declare_reply = [&](ZenohTransport& transport, const std::string& agent_id, const Message& message) {
        require(transport.declare_queryable(
                    agent_query_key(agent_id),
                    [message](const std::string&, std::string& wire, std::string&) {
                        wire = encode_envelope(message);
                        return true;
                    },
                    error),
                error);
    };
    auto start_station = [&](ZenohTransport& station, const AgentInfo& value, std::size_t expected_pushes) {
        require(station.open(station_endpoints, error), error);
        const Message message = agent_message(value, value.agentId + "-info");
        require(station.subscribe(
                    "sendToAll",
                    [&, station_id = value.agentId](const std::string& wire) {
                        Message envelope;
                        std::string decode_error;
                        if (!decode_envelope(wire, envelope, decode_error) || envelope.type != MessageType::AgentInfo ||
                            envelope.senderId != config.plane_id) return;
                        std::lock_guard<std::mutex> lock(mutex);
                        ++station_pushes[station_id];
                        changed.notify_all();
                    },
                    error),
                error);
        declare_reply(station, value.agentId, message);
        std::size_t previous_pushes;
        {
            std::lock_guard<std::mutex> lock(mutex);
            previous_pushes = station_pushes[value.agentId];
        }
        require(station.declare_liveliness_token(station_liveliness_key(value.agentId), error), error);
        wait_count(expected_pushes);
        {
            std::unique_lock<std::mutex> lock(mutex);
            require(changed.wait_for(lock, std::chrono::seconds(4), [&]() {
                        return station_pushes[value.agentId] > previous_pushes;
                    }),
                    "地面站未收到机载 AgentInfo");
        }
        require(station.put("sendToAll", encode_envelope(message), error), error);
    };
    auto wait_received = [&](std::size_t count, const AgentInfo& expected) {
        std::unique_lock<std::mutex> lock(mutex);
        require(changed.wait_for(lock, std::chrono::seconds(4), [&]() { return received.size() >= count; }),
                "机载未收到 AgentInfo");
        require(received.size() == count && received.back().agentId == expected.agentId &&
                    received.back().agentType == expected.agentType && received.back().boardIp == expected.boardIp &&
                    received.back().videoUrl == expected.videoUrl && received.back().isSim == expected.isSim,
                "机载接收的节点信息错误");
    };
    start_station(station_a, a_info, 2);
    wait_received(1, a_info);
    require(station_a.declare_liveliness_token(station_liveliness_key(a_info.agentId), error), error);
    wait_count(2);
    quiet(2);
    start_station(station_b, b_info, 3);
    wait_received(2, b_info);
    require(module.queryAgent(a_info.agentId, reply, error), error);
    require(from_payload(reply.payload, info, error) && info.agentType == AgentType::GroundStation &&
                info.boardIp == a_info.boardIp && !info.isSim,
            "机载查询地面站信息失败");
    Message invalid = agent_message(b_info, "wrong-sender");
    invalid.senderId = "unknown-station";
    require(station_b.put("sendToAll", encode_envelope(invalid), error), error);
    quiet(3);
    {
        std::lock_guard<std::mutex> lock(mutex);
        require(received.size() == 2, "接受 agentId 与 senderId 不一致的广播");
    }
    require(observer.declare_liveliness_token(plane_liveliness_key("peer-aircraft"), error), error);
    wait_count(4);
    quiet(4);
    station_a.close();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    start_station(station_a, a_info, 5);
    wait_received(3, a_info);
    quiet(5);
    {
        std::lock_guard<std::mutex> lock(mutex);
        for (std::size_t index = 1; index < message_ids.size(); ++index) {
            require(message_ids[index - 1] != message_ids[index], "报文 ID 重复");
        }
    }
    // GCS 先在线再启动飞机：history 发现与启动待发送合并。
    module.stop();
    require(module.start(), "机载重启失败");
    wait_count(6);
    quiet(6);
    module.stop();
    station_a.close();
    station_b.close();
    require(!queryAgent(observer, config.plane_id, reply, error, 100), "停机后查询接口仍然有效");
    declare_reply(observer, "wrong-sender", agent_message(a_info, "invalid-reply-sender"));
    require(!queryAgent(observer, "wrong-sender", reply, error), "接受发送方不匹配的查询响应");
    Message wrong_payload = agent_message(a_info, "invalid-reply-payload");
    wrong_payload.senderId = "wrong-payload";
    declare_reply(observer, "wrong-payload", wrong_payload);
    require(!queryAgent(observer, "wrong-payload", reply, error), "接受 agentId 不匹配的查询响应");
    declare_reply(observer, "wrong-type", {"wrong-type", "invalid-reply-type", MessageType::VehicleState, "{}"});
    require(!queryAgent(observer, "wrong-type", reply, error), "接受类型不匹配的查询响应");
    observer.close();
    return 0;
}
