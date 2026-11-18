#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "communication/plane_query.h"
#include "communication/serialization.h"

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

int available_port() {
    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    require(fd >= 0, "创建测试 socket 失败");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    const int result = bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address));
    socklen_t size = sizeof(address);
    const int named = getsockname(fd, reinterpret_cast<sockaddr*>(&address), &size);
    close(fd);
    require(result == 0 && named == 0, "获取测试端口失败");
    return ntohs(address.sin_port);
}

void test_existing_liveliness(const char* path) {
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<std::pair<std::string, bool>> events;
    plane::communication::ZenohTransport observer;
    plane::communication::ZenohTransport peer;
    plane::communication::ZenohEndpoints observer_endpoints;
    observer_endpoints.config_file = path;
    observer_endpoints.listen = "tcp/127.0.0.1:" + std::to_string(available_port());
    auto peer_endpoints = observer_endpoints;
    peer_endpoints.listen.clear();
    peer_endpoints.connect = observer_endpoints.listen;
    std::string error;
    require(observer.open(observer_endpoints, error), error);
    require(peer.open(peer_endpoints, error), error);

    // 先声明 token，再订阅，验证晚启动节点也能发现已在线节点。
    require(peer.declare_liveliness_token("plane/liveliness/test-peer", error), error);
    require(observer.subscribe_liveliness(
                "plane/liveliness/*",
                [&](const std::string& key, bool online) {
                    std::lock_guard<std::mutex> lock(mutex);
                    events.emplace_back(key, online);
                    changed.notify_all();
                },
                error),
            error);
    auto wait_for = [&](std::size_t count, bool online) {
        std::unique_lock<std::mutex> lock(mutex);
        require(changed.wait_for(lock, std::chrono::seconds(8), [&]() { return events.size() >= count; }),
                "未收到预期的存活性事件");
        require(events.size() == count &&
                    events.back() == std::make_pair(std::string("plane/liveliness/test-peer"), online),
                "存活性事件内容或数量错误");
    };
    wait_for(1, true);
    peer.close();
    wait_for(2, false);
    require(peer.open(peer_endpoints, error), error);
    require(peer.declare_liveliness_token("plane/liveliness/test-peer", error), error);
    wait_for(3, true);
    peer.close();
    wait_for(4, false);
    observer.close();
}

void test_static_tcp_mesh(const char* path) {
    using namespace std::chrono_literals;
    using plane::communication::ZenohEndpoints;
    using plane::communication::ZenohTransport;
    using plane::model::AgentInfo;
    using plane::model::AgentType;
    using plane::model::Message;
    using plane::model::MessageType;

    const std::string port = std::to_string(available_port());
    const std::string station_address = "tcp/127.0.0.1:" + port;
    const std::string first_address = "tcp/127.0.0.2:" + port;
    const std::string second_address = "tcp/127.0.0.3:" + port;
    const ZenohEndpoints station_endpoints{path, "", station_address};
    const ZenohEndpoints first_endpoints{path, station_address + "," + second_address, first_address};
    const ZenohEndpoints second_endpoints{path, station_address + "," + first_address, second_address};
    const AgentInfo station_info{"station", AgentType::GroundStation, "127.0.0.1", "", true};
    const AgentInfo first_info{"uav1", AgentType::Vehicle, "127.0.0.2", "", true};
    const AgentInfo second_info{"uav2", AgentType::Vehicle, "127.0.0.3", "", true};

    std::mutex mutex;
    std::condition_variable changed;
    std::set<std::pair<std::string, std::string>> received;
    bool second_online = false;
    ZenohTransport station;
    ZenohTransport first;
    ZenohTransport second;
    std::string error;

    auto start = [&](ZenohTransport& transport, const ZenohEndpoints& endpoints, const AgentInfo& info) {
        using namespace plane::communication;
        require(transport.open(endpoints, error), error);
        Message message{info.agentId, info.agentId + "-info", MessageType::AgentInfo, {}};
        require(to_payload(info, message.payload, error), error);
        require(transport.declare_queryable(
                    agent_query_key(info.agentId),
                    [wire = encode_envelope(message)](const std::string&, std::string& reply, std::string&) {
                        reply = wire;
                        return true;
                    },
                    error),
                error);
        require(transport.subscribe(
                    "test/static-tcp/" + info.agentId,
                    [&, agent_id = info.agentId](const std::string& sample) {
                        std::lock_guard<std::mutex> lock(mutex);
                        received.emplace(agent_id, sample);
                        changed.notify_all();
                    },
                    error),
                error);
        const auto key = info.agentType == AgentType::GroundStation ? station_liveliness_key(info.agentId)
                                                                   : plane_liveliness_key(info.agentId);
        require(transport.declare_liveliness_token(key, error), error);
    };
    auto wait_for_second = [&](bool online) {
        std::unique_lock<std::mutex> lock(mutex);
        require(changed.wait_for(lock, 8s, [&] { return second_online == online; }),
                online ? "地面站未发现 uav2 上线" : "地面站未发现 uav2 下线");
    };
    auto verify_sample = [&](ZenohTransport& publisher, const AgentInfo& from, const AgentInfo& to,
                             const std::string& phase) {
        const std::string key = "test/static-tcp/" + to.agentId;
        const std::string sample = from.agentId + "/" + phase;
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        do {
            require(publisher.put(key, sample, error), error);
            std::unique_lock<std::mutex> lock(mutex);
            if (changed.wait_for(lock, 50ms, [&] { return received.count({to.agentId, sample}) > 0; })) return;
        } while (std::chrono::steady_clock::now() < deadline);
        throw std::runtime_error("TCP 单播未收到 " + from.agentId + " 到 " + to.agentId + " 的报文");
    };
    auto verify_mesh = [&](const std::string& phase) {
        verify_sample(station, station_info, first_info, phase);
        verify_sample(first, first_info, station_info, phase);
        verify_sample(station, station_info, second_info, phase);
        verify_sample(second, second_info, station_info, phase);
        verify_sample(first, first_info, second_info, phase);
        verify_sample(second, second_info, first_info, phase);

        Message reply;
        require(plane::communication::queryAgent(station, second_info.agentId, reply, error), error);
        AgentInfo info;
        require(plane::communication::from_payload(reply.payload, info, error), error);
        require(info.boardIp == second_info.boardIp, "地面站查询得到的 uav2 boardIp 不匹配");
    };

    start(station, station_endpoints, station_info);
    require(station.subscribe_liveliness(
                plane::communication::plane_liveliness_key(second_info.agentId),
                [&](const std::string&, bool online) {
                    std::lock_guard<std::mutex> lock(mutex);
                    second_online = online;
                    changed.notify_all();
                },
                error),
            error);
    start(first, first_endpoints, first_info);
    start(second, second_endpoints, second_info);
    wait_for_second(true);
    verify_mesh("首次上线");

    second.close();
    wait_for_second(false);
    start(second, second_endpoints, second_info);
    wait_for_second(true);
    verify_mesh("重新上线");
}

}  // namespace

int main() {
    char path[] = "/tmp/plane-zenoh-liveliness-XXXXXX.json5";
    const int fd = mkstemps(path, 6);
    require(fd >= 0, "创建测试配置失败");
    close(fd);
    struct Cleanup {
        const char* path;
        ~Cleanup() { std::filesystem::remove(path); }
    } cleanup{path};
    {
        std::ofstream file(path);
        file
            << R"({"mode":"peer","scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}},"listen":{"endpoints":[]}})";
    }

    test_existing_liveliness(path);
    test_static_tcp_mesh(path);
    return 0;
}
