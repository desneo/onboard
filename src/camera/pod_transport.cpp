#include "camera/pod_transport.h"

#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <charconv>
#include <cstring>
#include <utility>

namespace plane::camera {
namespace {

class UdpPodTransport final : public PodTransport {
   public:
    UdpPodTransport(std::string host, std::string port) : host_(std::move(host)), port_(std::move(port)) {}
    ~UdpPodTransport() override { stop(); }

    bool start(std::string& error) override {
        if (socket_ >= 0) {
            return true;
        }

        addrinfo hints{};
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;
        hints.ai_flags = AI_NUMERICSERV;
        addrinfo* addresses = nullptr;
        const int result = ::getaddrinfo(host_.c_str(), port_.c_str(), &hints, &addresses);
        if (result != 0) {
            error = std::string("解析吊舱 UDP 地址失败：") + ::gai_strerror(result);
            return false;
        }
        const std::unique_ptr<addrinfo, decltype(&::freeaddrinfo)> address_list(addresses, ::freeaddrinfo);

        int last_error = EADDRNOTAVAIL;
        for (const auto* address = address_list.get(); address != nullptr; address = address->ai_next) {
            const int fd = ::socket(address->ai_family, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, address->ai_protocol);
            if (fd < 0) {
                last_error = errno;
                continue;
            }
            // 已连接的 UDP socket 只接收指定 IP 和端口的数据报，并复用同一个源端口。
            if (::connect(fd, address->ai_addr, address->ai_addrlen) == 0) {
                socket_ = fd;
                return true;
            }
            last_error = errno;
            ::close(fd);
        }
        error = std::string("打开吊舱 UDP 链路失败：") + std::strerror(last_error);
        return false;
    }

    bool send(const PodFrame& frame, std::string& error) override {
        if (socket_ < 0) {
            error = "吊舱 UDP 链路未打开";
            return false;
        }
        ssize_t sent;
        do {
            sent = ::send(socket_, frame.data(), frame.size(), 0);
        } while (sent < 0 && errno == EINTR);
        if (sent < 0) {
            error = std::string("发送吊舱 UDP 帧失败：") + std::strerror(errno);
            return false;
        }
        if (static_cast<std::size_t>(sent) != frame.size()) {
            error = "吊舱 UDP 帧未完整发送";
            return false;
        }
        return true;
    }

    bool receive(PodFrame& frame) override {
        if (socket_ < 0) {
            return false;
        }
        std::array<std::uint8_t, 65536> buffer;
        ssize_t size;
        do {
            size = ::recv(socket_, buffer.data(), buffer.size(), MSG_TRUNC);
        } while (size < 0 && errno == EINTR);
        if (size < 0 || static_cast<std::size_t>(size) > buffer.size()) {
            return false;
        }
        frame.assign(buffer.begin(), buffer.begin() + size);
        return true;
    }

    void stop() override {
        if (socket_ >= 0) {
            ::close(socket_);
            socket_ = -1;
        }
    }

   private:
    std::string host_;
    std::string port_;
    int socket_ = -1;
};

}  // namespace

std::unique_ptr<PodTransport> make_pod_transport(std::string_view url, std::string& error) {
    constexpr std::string_view prefix = "udp://";
    if (url.substr(0, prefix.size()) != prefix) {
        error = "pod_control_url 的传输类型暂未支持，仅支持 udp://";
        return nullptr;
    }
    url.remove_prefix(prefix.size());
    const auto separator = url.rfind(':');
    if (separator == std::string_view::npos || separator == 0 || separator + 1 == url.size()) {
        error = "pod_control_url 缺少 UDP 主机或端口";
        return nullptr;
    }
    auto host = url.substr(0, separator);
    const auto port = url.substr(separator + 1);
    unsigned number = 0;
    const auto parsed = std::from_chars(port.data(), port.data() + port.size(), number);
    if (parsed.ec != std::errc{} || parsed.ptr != port.data() + port.size() || number == 0 || number > 65535) {
        error = "pod_control_url 的 UDP 端口无效";
        return nullptr;
    }
    if (host.front() == '[' && host.back() == ']') {
        host.remove_prefix(1);
        host.remove_suffix(1);
    } else if (host.find(':') != std::string_view::npos) {
        error = "pod_control_url 的 IPv6 地址须使用方括号";
        return nullptr;
    }
    if (host.empty() || host.find_first_of(" \t\r\n/[]?#@") != std::string_view::npos ||
        host.find('\0') != std::string_view::npos) {
        error = "pod_control_url 的 UDP 主机无效";
        return nullptr;
    }
    return std::make_unique<UdpPodTransport>(std::string(host), std::to_string(number));
}

}  // namespace plane::camera
