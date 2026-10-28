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
#include <vector>

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
    using plane::model::MessageType;
    static_assert(static_cast<int>(MessageType::VehicleState) == 7, "existing enum values changed");
    static_assert(static_cast<int>(MessageType::VehicleInfo) == 8, "VehicleInfo must be appended");
    char path[] = "/tmp/plane-vehicle-info-XXXXXX";
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
    std::string config_path = std::string(path) + "/plane.conf";
    {
        std::ofstream file(config_path);
        file << "[system]\nboard_ip = 192.168.1.100\n[camera]\npod_video_url = rtsp://pod/live?a=1&b=2\n"
             << "[flight_control]\nlink_type = udp\nudp_url = udpout://127.0.0.1:14580\n";
    }
    plane::core::AppConfig config;
    require(plane::core::load_config(config_path, config), "load config failed");
    require(config.board_ip == "192.168.1.100" && config.pod_video_url == "rtsp://pod/live?a=1&b=2" && config.is_sim,
            "metadata config mismatch");
    config.plane_id = "metadata-test";
    config.station_id = "test-gcs";
    config.state_report_period_ms = 100;
    config.zenoh_config_file = zenoh_config;

    std::string error;
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<plane::model::VehicleInfo> pushes;
    std::vector<std::string> message_ids;
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
                    if (!decode_envelope(wire, envelope, error) || envelope.type != MessageType::VehicleInfo) return;
                    plane::model::VehicleInfo info;
                    require(from_payload(envelope.payload, info, error), error);
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
                "VehicleInfo push missing");
        require(pushes.size() == count, "duplicate VehicleInfo push");
        require(pushes.back().planeId == config.plane_id && pushes.back().boardIp == config.board_ip &&
                    pushes.back().podVideoUrl == config.pod_video_url && pushes.back().isSim == config.is_sim,
                "push metadata mismatch");
    };
    auto quiet = [&](std::size_t count) {
        std::unique_lock<std::mutex> lock(mutex);
        require(!changed.wait_for(lock, std::chrono::milliseconds(400), [&]() { return pushes.size() != count; }),
                "unexpected periodic or repeated push");
    };
    config.zenoh_connect = endpoints.listen;
    config.zenoh_listen = "tcp/127.0.0.1:" + std::to_string(available_port());
    plane::core::EventBus bus;
    plane::state::StateStore store(bus, config);
    Tasks tasks;
    plane::communication::CommunicationModule module(bus, config, store, tasks);
    require(module.init() && module.start(), "communication start failed");
    wait_count(1);  // 没有 GCS token 时，启动仍主动上报。
    quiet(1);

    plane::model::Message reply;
    require(queryPlane(observer, config.plane_id, MessageType::VehicleInfo, reply, error), error);
    plane::model::VehicleInfo info;
    require(from_payload(reply.payload, info, error) && info.boardIp == config.board_ip &&
                info.podVideoUrl == config.pod_video_url && info.isSim,
            "query metadata mismatch");
    require(!queryPlane(observer, "*", MessageType::VehicleInfo, reply, error) && !error.empty(), "wildcard accepted");
    require(!queryPlane(observer, config.plane_id, MessageType::VehicleState, reply, error) && !error.empty(),
            "unsupported type accepted");
    require(!queryPlane(observer, config.plane_id, MessageType::VehicleInfo, reply, error, 0),
            "invalid timeout accepted");
    require(!queryPlane(observer, "missing-plane", MessageType::VehicleInfo, reply, error, 100) && !error.empty(),
            "nonexistent plane query succeeded");
    std::string raw;
    require(!observer.query("queryToPlane/" + config.plane_id + "/Unknown", raw, error, 500) &&
                error.find("不支持") != std::string::npos,
            "server did not reject unsupported type");
    require(!from_payload(R"({"planeId":"bad","boardIp":123,"podVideoUrl":""})", info, error),
            "invalid VehicleInfo accepted");
    require(!from_payload(R"({"planeId":"bad","boardIp":"","podVideoUrl":"","isSim":"true"})", info, error),
            "string isSim accepted");
    require(from_payload(R"({"planeId":"legacy","boardIp":"","podVideoUrl":""})", info, error) && !info.isSim,
            "legacy VehicleInfo default isSim mismatch");

    ZenohTransport station;
    ZenohEndpoints station_endpoints;
    station_endpoints.config_file = zenoh_config;
    station_endpoints.connect = config.zenoh_listen;
    require(station.open(station_endpoints, error), error);
    require(station.declare_liveliness_token("station/liveliness/other-gcs", error), error);
    quiet(1);  // 其他地面站不能触发。
    // 没有并发配置写入；下一次发送和查询应读取新值，而非启动时缓存。
    config.board_ip = "192.168.1.199";
    config.pod_video_url.clear();
    config.is_sim = false;
    require(station.declare_liveliness_token("station/liveliness/test-gcs", error), error);
    wait_count(2);
    require(station.declare_liveliness_token("station/liveliness/test-gcs", error), error);
    quiet(2);
    require(queryPlane(observer, config.plane_id, MessageType::VehicleInfo, reply, error), error);
    require(from_payload(reply.payload, info, error) && info.boardIp == "192.168.1.199" && info.podVideoUrl.empty() &&
                !info.isSim,
            "query returned startup snapshot");
    station.close();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    require(station.open(station_endpoints, error), error);
    require(station.declare_liveliness_token("station/liveliness/test-gcs", error), error);
    wait_count(3);
    quiet(3);
    {
        std::lock_guard<std::mutex> lock(mutex);
        require(message_ids[0] != message_ids[1] && message_ids[1] != message_ids[2], "message IDs reused");
    }
    // GCS 先在线再启动飞机：history 发现与启动待发送合并。
    module.stop();
    require(module.start(), "restart failed");
    wait_count(4);
    quiet(4);
    module.stop();
    station.close();
    require(!queryPlane(observer, config.plane_id, MessageType::VehicleInfo, reply, error, 100),
            "queryable remained after stop");

    // 校验不匹配的发送方和载荷不能作为指定飞机的结果。
    require(observer.declare_queryable(
                "queryToPlane/wrong/VehicleInfo",
                [](const std::string&, std::string& wire, std::string& error) {
                    plane::model::Message message;
                    if (!make_message("other", plane::model::VehicleInfo{"other", "", ""}, message, error))
                        return false;
                    wire = encode_envelope(message);
                    return true;
                },
                error),
            error);
    require(!queryPlane(observer, "wrong", MessageType::VehicleInfo, reply, error), "wrong sender accepted");
    observer.close();
    return 0;
}
