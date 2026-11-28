#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "camera/camera_control_api.h"
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

class TestCamera final : public plane::camera::CameraControlApi {
   public:
    TestCamera(std::mutex& mutex, std::condition_variable& changed, plane::model::PodState state)
        : mutex_(mutex), changed_(changed), state_(std::move(state)) {}

    void submit_command(const plane::model::PodCommand& command,
                        std::chrono::steady_clock::time_point received_at) override {
        std::lock_guard<std::mutex> lock(mutex_);
        commands.push_back(command);
        received_times.push_back(received_at);
        changed_.notify_all();
    }

    std::optional<plane::model::PodState> pod_state() const override { return state_; }

    std::vector<plane::model::PodCommand> commands;
    std::vector<std::chrono::steady_clock::time_point> received_times;

   private:
    std::mutex& mutex_;
    std::condition_variable& changed_;
    const plane::model::PodState state_;
};
}  // namespace

int main() {
    using namespace plane::communication;
    using plane::model::AgentInfo;
    using plane::model::AgentType;
    using plane::model::Message;
    using plane::model::MessageType;
    using plane::model::PodCommand;
    using plane::model::PodCommandType;
    using plane::model::PodState;
    using plane::model::SelfCheckResult;
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
        file << "[deployment]\nprofile = " << profile_path << '\n'
             << "[camera]\npod_state_report_hz = 5\n"
             << "[behavior_tree]\ntree_update_rate_hz = 20\n"
             << "[midcourse]\ntarget_history_size = 20\ntarget_timeout_ms = 5000\n";
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
    config.pod_state_report_hz = 4;
    config.zenoh_config_file = zenoh_config;

    std::string error;
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<AgentInfo> pushes;
    std::vector<AgentInfo> received;
    std::unordered_map<std::string, std::size_t> station_pushes;
    std::vector<std::string> message_ids;
    std::vector<std::string> ack_ids;
    std::vector<PodState> pod_states;
    std::vector<std::chrono::steady_clock::time_point> pod_report_times;
    std::vector<SelfCheckResult> self_check_results;
    int command_count = 0;
    int self_check_count = 0;
    int task_count = 0;
    bool expected_flight_command_received = false;
    bool expected_task_received = false;
    const PodState expected_pod_state{config.plane_id,
                                      "zhongke_d80",
                                      true,
                                      true,
                                      123456,
                                      {{"机械俯仰角（°）", -4.5},
                                       {"限位", false},
                                       {"状态", std::string("待机")},
                                       {"通道", std::int64_t{1}}}};
    TestCamera camera(mutex, changed, expected_pod_state);
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
                    PodState pod_state;
                    SelfCheckResult self_check_result;
                    if (!decode_envelope(wire, envelope, error) || envelope.senderId != config.plane_id) return;
                    std::lock_guard<std::mutex> lock(mutex);
                    if (envelope.type == MessageType::Ack && from_payload(envelope.payload, ack, error)) {
                        ack_ids.push_back(ack.ackId);
                    } else if (envelope.type == MessageType::PodState &&
                               from_payload(envelope.payload, pod_state, error)) {
                        pod_states.push_back(std::move(pod_state));
                        pod_report_times.push_back(std::chrono::steady_clock::now());
                    } else if (envelope.type == MessageType::SelfCheckResult &&
                               from_payload(envelope.payload, self_check_result, error)) {
                        self_check_results.push_back(std::move(self_check_result));
                    } else {
                        return;
                    }
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
    using ReceivedCommand = plane::communication::ReceivedControl<plane::model::Command>;
    using ReceivedTask = plane::communication::ReceivedControl<plane::model::TaskInfo>;
    bus.subscribe<ReceivedCommand>(events::kCommandReceived, [&](const ReceivedCommand& request) {
        const auto& command = request.value;
        std::lock_guard<std::mutex> lock(mutex);
        if (command.type == plane::model::CommandType::SelfCheck) {
            ++self_check_count;
            return;
        }
        expected_flight_command_received = command.commandId == "control-1" &&
                                           command.type == plane::model::CommandType::Hover;
        ++command_count;
    });
    bus.subscribe<ReceivedTask>(events::kTaskInfoReceived, [&](const ReceivedTask& request) {
        const auto& task = request.value;
        std::lock_guard<std::mutex> lock(mutex);
        expected_task_received = task.taskId == "task-1" && task.taskAction == plane::model::TaskAction::Stop;
        ++task_count;
    });
    plane::state::StateStore store(bus, config);
    Tasks tasks;
    plane::communication::CommunicationModule module(bus, config, store, tasks, &camera);
    require(module.init() && module.start(), "communication start failed");
    wait_count(1);  // 没有 GCS token 时，启动仍主动上报。
    quiet(1);

    const std::string command_topic = "sendToPlane/" + config.plane_id;
    auto send_and_wait_for_ack = [&](const plane::model::Message& message) {
        std::size_t previous_count;
        {
            std::lock_guard<std::mutex> lock(mutex);
            previous_count = ack_ids.size();
        }
        require(observer.put(command_topic, encode_envelope(message), error), error);
        std::unique_lock<std::mutex> lock(mutex);
        require(changed.wait_for(lock, std::chrono::seconds(4), [&]() {
                    return ack_ids.size() > previous_count && ack_ids.back() == message.messageId;
                }),
                "control Ack missing on sendToStation");
    };
    const Message flight_command{config.plane_id, "command-message", MessageType::Command,
                                 R"({"commandId":"control-1","type":"Hover"})"};
    const Message task_stop{"task-sender", "task-message", MessageType::TaskInfo,
                            R"({"taskId":"task-1","taskCode":"AntiUav","taskAction":"Stop"})"};
    send_and_wait_for_ack(flight_command);
    send_and_wait_for_ack(task_stop);
    send_and_wait_for_ack(flight_command);
    send_and_wait_for_ack(task_stop);
    {
        std::lock_guard<std::mutex> lock(mutex);
        require(expected_flight_command_received && expected_task_received && command_count == 1 && task_count == 1,
                "飞行指令或任务未送达，或重传报文再次进入调度");
    }
    require(store.gcs_link().connected, "valid controls did not update station activity");

    const Message self_check{"check-station", "self-check", MessageType::Command,
                             R"({"commandId":"check-1","type":"SelfCheck"})"};
    require(observer.put("sendToAll", encode_envelope(self_check), error), error);
    require(observer.put(command_topic,
                         encode_envelope({"check-station", "bad-self-check", MessageType::Command,
                                          R"({"commandId":"","type":"SelfCheck"})"}), error),
            error);
    send_and_wait_for_ack(self_check);
    send_and_wait_for_ack(self_check);
    require(module.sendToStation(SelfCheckResult{"check-1", false, "飞控拒绝解锁"}), "自检失败结果发送失败");
    require(module.sendToStation(SelfCheckResult{"check-2", true, "解锁已确认"}), "自检成功结果发送失败");
    {
        std::unique_lock<std::mutex> lock(mutex);
        require(changed.wait_for(lock, std::chrono::seconds(4), [&]() { return self_check_results.size() >= 2; }),
                "未收到自检结果");
        require(self_check_count == 1, "自检重传或广播再次进入调度");
        require(self_check_results[0].commandId == "check-1" && !self_check_results[0].success &&
                    self_check_results[0].detail == "飞控拒绝解锁" &&
                    self_check_results[1].commandId == "check-2" && self_check_results[1].success &&
                    self_check_results[1].detail == "解锁已确认",
                "自检结果未保留 commandId、success 或 detail");
    }
    SelfCheckResult invalid_result;
    require(!from_payload(R"({"commandId":"check-1","success":"false","detail":"失败"})", invalid_result,
                          error),
            "接受字符串作为自检结果的 success");

    PodCommand decoded_pod_command;
    require(!from_payload(R"({"type":"SetRate","pitchRate_deg_s":true,"yawRate_deg_s":0})", decoded_pod_command,
                          error),
            "接受布尔值作为吊舱角速度");
    require(!from_payload(R"({"type":"SetAngles","pitch_deg":3})", decoded_pod_command, error),
            "接受缺少偏航角的吊舱指令");
    require(!from_payload(R"({"type":"SetRate","pitchRate_deg_s":1e999,"yawRate_deg_s":0})", decoded_pod_command,
                          error),
            "接受超出表示范围的吊舱角速度");

    const Message zero_pitch{"pod-station", "zero-pitch", MessageType::PodCommand, R"({"type":"ZeroPitch"})"};
    const Message zero_yaw{"pod-station", "zero-yaw", MessageType::PodCommand, R"({"type":"ZeroYaw"})"};
    require(observer.put("sendToAll", encode_envelope(zero_pitch), error), error);
    require(observer.put("sendToPlane/other-plane", encode_envelope(zero_yaw), error), error);
    require(observer.put(command_topic,
                         encode_envelope({"pod-station", "bad-angle", MessageType::PodCommand,
                                          R"({"type":"SetAngles","pitch_deg":3})"}), error),
            error);
    send_and_wait_for_ack(zero_pitch);
    send_and_wait_for_ack(zero_pitch);
    send_and_wait_for_ack({"another-pod-station", zero_pitch.messageId, MessageType::PodCommand, zero_pitch.payload});
    send_and_wait_for_ack(zero_yaw);

    auto publish_rate = [&](const std::string& id, double pitch, double yaw) {
        Message message{"pod-station", id, MessageType::PodCommand, {}};
        require(to_payload(PodCommand{PodCommandType::SetRate, pitch, yaw}, message.payload, error), error);
        require(observer.put(command_topic, encode_envelope(message), error), error);
    };
    const auto before_rate = std::chrono::steady_clock::now();
    publish_rate("rate", 2.5, -3.5);
    publish_rate("stop", 0.0, 0.0);
    send_and_wait_for_ack({"pod-station", "angles", MessageType::PodCommand,
                           R"({"type":"SetAngles","pitch_deg":-12.0,"yaw_deg":20.0})"});
    {
        std::unique_lock<std::mutex> lock(mutex);
        require(changed.wait_for(lock, std::chrono::seconds(4), [&]() { return pod_states.size() >= 3; }),
                "缺少周期吊舱状态");
        require(camera.commands.size() == 6 && camera.commands[0].type == PodCommandType::ZeroPitch &&
                    camera.commands[1].type == PodCommandType::ZeroPitch &&
                    camera.commands[2].type == PodCommandType::ZeroYaw &&
                    camera.commands[3].type == PodCommandType::SetRate && camera.commands[3].pitchRate_deg_s == 2.5 &&
                    camera.commands[3].yawRate_deg_s == -3.5 && camera.commands[4].type == PodCommandType::SetRate &&
                    camera.commands[4].pitchRate_deg_s == 0.0 && camera.commands[4].yawRate_deg_s == 0.0 &&
                    camera.commands[5].type == PodCommandType::SetAngles && camera.commands[5].pitch_deg == -12.0 &&
                    camera.commands[5].yaw_deg == 20.0 && camera.received_times[3] >= before_rate &&
                    camera.received_times[4] >= camera.received_times[3],
                "吊舱指令单播、解码、去重或顺序有误");
        require(ack_ids == std::vector<std::string>{"command-message", "task-message", "command-message",
                                                    "task-message", "self-check", "self-check", "zero-pitch",
                                                    "zero-pitch", "zero-pitch", "zero-yaw", "angles"},
                "飞行控制或吊舱指令回执有误");
        require(command_count == 1 && task_count == 1, "吊舱操作误入飞行指令或任务流程");
        require(pod_states.back().planeId == expected_pod_state.planeId &&
                    pod_states.back().model == expected_pod_state.model && pod_states.back().connected &&
                    pod_states.back().canMove && pod_states.back().timestamp_ms == expected_pod_state.timestamp_ms &&
                    pod_states.back().fields == expected_pod_state.fields,
                "吊舱状态及中文键或 JSON 标量类型有误");
        const auto report_span = pod_report_times[2] - pod_report_times[0];
        require(report_span >= std::chrono::milliseconds(420) && report_span <= std::chrono::milliseconds(700),
                "吊舱状态上报与 4 Hz 配置不符");
    }

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
    config.pod_state_report_hz = 5;
    {
        std::lock_guard<std::mutex> lock(mutex);
        pod_report_times.clear();
    }
    require(module.start(), "机载重启失败");
    wait_count(6);
    quiet(6);
    {
        std::unique_lock<std::mutex> lock(mutex);
        require(changed.wait_for(lock, std::chrono::seconds(4), [&]() { return pod_report_times.size() >= 3; }),
                "缺少默认频率的吊舱状态");
        const auto report_span = pod_report_times[2] - pod_report_times[0];
        require(report_span >= std::chrono::milliseconds(330) && report_span <= std::chrono::milliseconds(480),
                "吊舱状态上报与默认 5 Hz 不符");
    }
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
