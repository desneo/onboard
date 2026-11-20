#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "communication/communication_module.h"
#include "communication/serialization.h"
#include "flight_control/flight_control_events.h"
#include "state/state_store.h"

namespace {

void require(bool condition, const std::string& detail) {
    if (!condition) throw std::runtime_error(detail);
}

int available_port() {
    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    require(fd >= 0, "无法创建测试 socket");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    const int bound = bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address));
    socklen_t size = sizeof(address);
    const int named = getsockname(fd, reinterpret_cast<sockaddr*>(&address), &size);
    close(fd);
    require(bound == 0 && named == 0, "无法分配测试端口");
    return ntohs(address.sin_port);
}

struct Tasks final : plane::task::TaskStateProvider {
    plane::model::TaskState current_task_state() const override { return {}; }
};

}  // namespace

int main() {
    using namespace plane::communication;
    using plane::model::Alert;
    using plane::model::Message;
    using plane::model::MessageType;

    char path[] = "/tmp/plane-alert-queue-XXXXXX";
    require(mkdtemp(path) != nullptr, "无法创建测试临时目录");
    struct Cleanup {
        const char* path;
        ~Cleanup() { std::filesystem::remove_all(path); }
    } cleanup{path};
    const std::string zenoh_config = std::string(path) + "/zenoh.json5";
    {
        std::ofstream file(zenoh_config);
        file << R"({"mode":"peer","scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}},"listen":{"endpoints":[]}})";
    }

    plane::core::AppConfig config;
    config.plane_id = "alert-queue-test";
    config.is_sim = true;
    config.zenoh_config_file = zenoh_config;
    config.state_report_period_ms = 100;
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<std::string> details;
    std::set<std::string> message_ids;
    std::atomic<bool> invalid_alert{false};

    ZenohTransport observer;
    ZenohEndpoints endpoints;
    endpoints.config_file = zenoh_config;
    endpoints.listen = "tcp/127.0.0.1:" + std::to_string(available_port());
    std::string error;
    require(observer.open(endpoints, error), error);
    require(observer.subscribe(
                "sendToStation",
                [&](const std::string& wire) {
                    Message envelope;
                    std::string decode_error;
                    if (!decode_envelope(wire, envelope, decode_error) || envelope.type != MessageType::Alert ||
                        envelope.senderId != config.plane_id) {
                        return;
                    }
                    const auto payload = nlohmann::json::parse(envelope.payload, nullptr, false);
                    std::lock_guard<std::mutex> lock(mutex);
                    if (payload.is_discarded() || !payload.is_object() || payload.size() != 2 ||
                        !payload.contains("type") || !payload.at("type").is_string() ||
                        payload.at("type").get<std::string>() != "queue_test" || !payload.contains("detail") ||
                        !payload.at("detail").is_string() || envelope.messageId.empty() ||
                        !message_ids.insert(envelope.messageId).second) {
                        invalid_alert = true;
                    } else {
                        details.push_back(payload.at("detail").get<std::string>());
                    }
                    changed.notify_all();
                },
                error),
            error);

    config.zenoh_connect = endpoints.listen;
    config.zenoh_listen = "tcp/127.0.0.1:" + std::to_string(available_port());
    plane::core::EventBus bus;
    plane::state::StateStore state(bus, config);
    Tasks tasks;
    CommunicationModule communication(bus, config, state, tasks);
    require(!communication.sendToStation(Alert{"queue_test", "初始化之前"}), "初始化之前不得接受告警");
    require(communication.init(), "通信初始化失败");

    // 初始化时已订阅飞控诊断；通信工作线程尚未启动，队列必须保留最新的 256 条。
    for (int index = 0; index < 300; ++index) {
        const Alert alert{"queue_test", std::to_string(index)};
        if (index % 2 == 0) {
            bus.publish(plane::flight_control::events::kDiagnostic, alert);
        } else {
            require(communication.sendToStation(alert), "启动前告警入队失败");
        }
    }
    {
        std::lock_guard<std::mutex> lock(mutex);
        require(details.empty(), "工作线程启动前不得发送告警");
    }
    require(communication.start(), "通信启动失败");
    {
        std::unique_lock<std::mutex> lock(mutex);
        require(changed.wait_for(lock, std::chrono::seconds(6), [&]() {
                    return details.size() >= 256 || invalid_alert.load();
                }),
                "未收到全部启动缓存告警");
        require(!invalid_alert && details.size() == 256, "启动缓存出现非法报文或重复告警");
        for (int index = 0; index < 256; ++index) {
            require(details[index] == std::to_string(index + 44), "队列未按顺序保留最新的 256 条告警");
        }
    }

    require(communication.sendToStation(Alert{"queue_test", "启动后第一条"}), "运行中告警入队失败");
    bus.publish(plane::flight_control::events::kDiagnostic, Alert{"queue_test", "启动后第二条"});
    {
        std::unique_lock<std::mutex> lock(mutex);
        require(changed.wait_for(lock, std::chrono::seconds(3), [&]() { return details.size() >= 258; }),
                "未收到运行中的告警");
        require(details.size() == 258 && details[256] == "启动后第一条" && details[257] == "启动后第二条",
                "飞控事件与直接发送的告警没有共享同一顺序");
    }

    // 生产线程持续发布诊断，与停止动作交错；停止后所有直接入队均须被拒绝。
    std::atomic<bool> done{false};
    std::atomic<int> produced{0};
    std::thread producer([&]() {
        while (!done.load()) {
            bus.publish(plane::flight_control::events::kDiagnostic, Alert{"queue_test", "停止并发事件"});
            communication.sendToStation(Alert{"queue_test", "停止并发发送"});
            ++produced;
            changed.notify_all();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
    {
        std::unique_lock<std::mutex> lock(mutex);
        changed.wait_for(lock, std::chrono::seconds(1), [&]() { return produced.load() >= 4; });
    }
    communication.stop();
    done = true;
    producer.join();
    require(produced >= 4, "未执行停止时的并发发布");
    for (int index = 0; index < 128; ++index) {
        require(!communication.sendToStation(Alert{"queue_test", "停止后发送"}), "停止后仍接受告警");
        bus.publish(plane::flight_control::events::kDiagnostic, Alert{"queue_test", "停止后事件"});
    }
    {
        std::unique_lock<std::mutex> lock(mutex);
        require(!changed.wait_for(lock, std::chrono::milliseconds(200), [&]() {
                    for (const auto& detail : details) {
                        if (detail == "停止后发送" || detail == "停止后事件") return true;
                    }
                    return invalid_alert.load();
                }),
                "停止后仍发送告警或出现非法报文");
    }
    observer.close();
}
