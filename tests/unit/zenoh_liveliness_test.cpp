#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "communication/zenoh_transport.h"

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
    return 0;
}
