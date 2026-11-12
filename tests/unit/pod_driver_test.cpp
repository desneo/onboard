#include "camera/pod_driver.h"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using namespace std::chrono_literals;
using plane::camera::make_pod_driver;
using plane::camera::make_pod_transport;
using plane::camera::PodDriver;
using plane::camera::PodFrame;
using plane::camera::PodTransport;
using plane::model::PodCommand;
using plane::model::PodCommandType;
using Clock = std::chrono::steady_clock;

void require(bool condition, const char* detail) {
    if (!condition) {
        std::cerr << detail << '\n';
        std::exit(1);
    }
}

class MemoryTransport final : public PodTransport {
   public:
    bool start(std::string&) override { return true; }
    void stop() override { incoming.clear(); }
    bool send(const PodFrame& frame, std::string&) override {
        outgoing.push_back(frame);
        return true;
    }
    bool receive(PodFrame& frame) override {
        if (incoming.empty()) {
            return false;
        }
        frame = std::move(incoming.front());
        incoming.pop_front();
        return true;
    }

    std::deque<PodFrame> incoming;
    std::vector<PodFrame> outgoing;
};

struct DriverFixture {
    explicit DriverFixture(const char* model) {
        auto memory = std::make_unique<MemoryTransport>();
        transport = memory.get();
        driver = make_pod_driver(model, std::move(memory), error);
        require(driver && driver->start(error), "启动内存吊舱驱动失败");
    }

    Clock::time_point now{1000ms};
    std::string error;
    MemoryTransport* transport = nullptr;
    std::unique_ptr<PodDriver> driver;
};

void seal(PodFrame& frame, std::size_t begin) {
    frame.back() = 0;
    for (auto i = begin; i < frame.size() - 1; ++i) {
        frame.back() += frame[i];
    }
}

PodFrame aisheng_telemetry(bool locked) {
    PodFrame frame(104, 0);
    frame[0] = 0x55;
    frame[1] = 0xAA;
    frame[2] = 0x68;
    frame[3] = 1;
    frame[5] = 0x50;
    frame[6] = 0xFB;  // -12.00°
    frame[7] = 0xBC;
    frame[8] = 0x02;  // +7.00°
    frame[26] = locked ? 0x20 : 0x80;
    seal(frame, 3);
    return frame;
}

PodFrame zhongke_telemetry() {
    PodFrame frame(32, 0);
    frame[0] = 0xEE;
    frame[1] = 0x16;
    frame[2] = 0x04;
    frame[3] = 0x08;
    frame[4] = 20;
    frame[6] = 40;  // +2.00°
    frame[8] = 0xEC;
    frame[9] = 0xFF;  // -1.00°
    frame[12] = 0x18;
    frame[13] = 0xFC;  // -10.00°
    frame[14] = 0xC2;
    frame[15] = 0x06;  // +17.30°
    frame[28] = 0x80;
    seal(frame, 0);
    return frame;
}

void test_aisheng() {
    DriverFixture fixture("aisheng_asd2");
    auto& driver = *fixture.driver;
    auto& transport = *fixture.transport;
    auto now = fixture.now;
    require(driver.can_move(), "艾晟未报告机械转动能力");
    require(!driver.apply(PodCommand{PodCommandType::ZeroPitch}, now, fixture.error), "艾晟接受无遥测归零");

    auto bad_frame = aisheng_telemetry(true);
    ++bad_frame.back();
    transport.incoming.push_back(bad_frame);
    require(!driver.tick(now), "艾晟接收校验和错误的帧");
    require(transport.outgoing.size() == 1 && transport.outgoing.back().size() == 45 &&
                transport.outgoing.back()[0] == 0xAA && transport.outgoing.back()[3] == 0,
            "艾晟未发送标准心跳");

    transport.incoming.push_back(aisheng_telemetry(true));
    const auto telemetry = driver.tick(now + 10ms);
    require(telemetry && std::get<double>(telemetry->fields.at("俯仰角（°）")) == -12.0 &&
                std::get<double>(telemetry->fields.at("偏航角（°）")) == 7.0 &&
                std::get<std::string>(telemetry->fields.at("伺服状态")) == "电锁",
            "艾晟机械角或电锁解码错误");
    require(driver.apply(PodCommand{PodCommandType::ZeroPitch}, now + 11ms, fixture.error) &&
                transport.outgoing.size() == 2 && transport.outgoing.back()[3] == 0x9E,
            "艾晟未在运动前解除电锁");
    transport.incoming.push_back(aisheng_telemetry(true));
    driver.tick(now + 12ms);
    require(transport.outgoing.size() == 2, "艾晟在确认解锁之前发送了运动指令");
    transport.incoming.push_back(aisheng_telemetry(false));
    driver.tick(now + 13ms);
    const auto& zero_pitch = transport.outgoing.back();
    require(transport.outgoing.size() == 3 && zero_pitch[3] == 0x90 && zero_pitch[4] == 0xBC &&
                zero_pitch[5] == 0x02 && zero_pitch[6] == 0 && zero_pitch[7] == 0,
            "艾晟俯仰归零未保持偏航角");

    const PodCommand rate{PodCommandType::SetRate, -60, 30};
    require(driver.apply(rate, now + 15ms, fixture.error), "艾晟拒绝双轴速度");
    const auto& speed = transport.outgoing.back();
    require(speed[3] == 0xB1 && speed[4] == 50 && speed[5] == 0 && speed[6] == 0x9C && speed[7] == 0xFF,
            "艾晟双轴速度的符号、量纲或字节序错误");

    transport.incoming.push_back(aisheng_telemetry(true));
    driver.tick(now + 20ms);
    require(driver.apply(rate, now + 21ms, fixture.error), "艾晟拒绝排队速度");
    require(driver.apply(PodCommand{PodCommandType::SetRate}, now + 22ms, fixture.error), "艾晟拒绝零速");
    const auto stopped_size = transport.outgoing.size();
    transport.incoming.push_back(aisheng_telemetry(false));
    driver.tick(now + 23ms);
    require(transport.outgoing.size() == stopped_size, "艾晟松手后执行了延迟速度指令");

    transport.incoming.push_back(aisheng_telemetry(true));
    driver.tick(now + 24ms);
    require(driver.apply(PodCommand{PodCommandType::ZeroYaw}, now + 25ms, fixture.error), "艾晟拒绝排队归零");
    transport.incoming.push_back(aisheng_telemetry(false));
    driver.tick(now + 1024ms);
    require(transport.outgoing.back()[3] == 0, "艾晟遥测断流恢复后执行了延迟归零");

    transport.incoming.push_back(aisheng_telemetry(true));
    driver.tick(now + 1030ms);
    require(driver.apply(PodCommand{PodCommandType::ZeroYaw}, now + 1031ms, fixture.error) &&
                transport.outgoing.back()[3] == 0x9E,
            "艾晟未请求解除电锁");
    driver.tick(now + 1231ms);
    require(transport.outgoing.back()[3] == 0x9E, "艾晟在未解除电锁时未重试");
    transport.incoming.push_back(aisheng_telemetry(false));
    driver.tick(now + 1240ms);
    require(transport.outgoing.back()[3] == 0x90, "艾晟重试解锁后未发送待执行指令");

    auto unknown_servo = aisheng_telemetry(true);
    unknown_servo[26] = 0;
    seal(unknown_servo, 3);
    transport.incoming.push_back(unknown_servo);
    driver.tick(now + 1241ms);
    require(!driver.apply(rate, now + 1242ms, fixture.error), "艾晟在未知伺服状态下接受了运动指令");
}

void test_zhongke() {
    DriverFixture fixture("zhongke_d80");
    auto& driver = *fixture.driver;
    auto& transport = *fixture.transport;
    const auto now = fixture.now;
    auto wrong_header = zhongke_telemetry();
    wrong_header[0] = 0x55;
    seal(wrong_header, 0);
    transport.incoming.push_back(wrong_header);
    require(driver.can_move() && !driver.tick(now), "中科未报告转动能力或接受了错误帧头");
    require(transport.outgoing.size() == 1 && transport.outgoing.back().size() == 16 &&
                transport.outgoing.back()[2] == 0 && transport.outgoing.back().back() == 0x7B,
            "中科未发送空帧保活");

    transport.incoming.push_back(zhongke_telemetry());
    const auto telemetry = driver.tick(now + 1ms);
    require(telemetry && std::get<double>(telemetry->fields.at("俯仰角（°）")) == -10.0 &&
                std::get<double>(telemetry->fields.at("偏航角（°）")) == 17.3 &&
                std::get<double>(telemetry->fields.at("俯仰失调角（°）")) == -1.0 &&
                std::get<double>(telemetry->fields.at("偏航失调角（°）")) == 2.0 &&
                std::get<bool>(telemetry->fields.at("自检完成")),
            "中科遥测物理量或状态位解码错误");

    require(driver.apply(PodCommand{PodCommandType::SetRate, -30, 40}, now + 2ms, fixture.error), "中科拒绝双轴速度");
    const PodFrame expected_rate{0xEB, 0x90, 0x24, 0xC8, 0, 0x38, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0x9E};
    require(transport.outgoing.back() == expected_rate, "中科双轴速度限幅、字节序或校验和错误");
    require(driver.apply(PodCommand{PodCommandType::ZeroYaw}, now + 3ms, fixture.error), "中科拒绝偏航归零");
    const auto& zero_yaw = transport.outgoing.back();
    require(zero_yaw[2] == 0x26 && zero_yaw[3] == 0 && zero_yaw[4] == 0 && zero_yaw[5] == 0x18 &&
                zero_yaw[6] == 0xFC,
            "中科偏航归零未保持实测俯仰角");
    require(!driver.apply(PodCommand{PodCommandType::SetAngles, 0, 0, 31, 10}, now + 4ms, fixture.error),
            "中科接受了超出机械限位的指定角");
    require(!driver.apply(PodCommand{PodCommandType::ZeroPitch}, now + 1001ms, fixture.error), "中科使用过期遥测归零");
    require(driver.apply(PodCommand{PodCommandType::SetRate}, now + 1002ms, fixture.error) &&
                transport.outgoing.back()[2] == 0x24 && transport.outgoing.back()[3] == 0 &&
                transport.outgoing.back()[4] == 0 && transport.outgoing.back()[5] == 0 && transport.outgoing.back()[6] == 0,
            "中科遥测过期时无法发送零速");
}

void test_jielian() {
    DriverFixture fixture("jielian_a2416a");
    auto& transport = *fixture.transport;
    const auto now = fixture.now;
    auto frame = aisheng_telemetry(false);
    frame[28] = 4;
    frame[40] = 4;
    frame[76] = 3;  // 变倍 4×
    seal(frame, 3);
    transport.incoming.push_back(frame);
    const auto telemetry = fixture.driver->tick(now);
    require(telemetry && !fixture.driver->can_move() && telemetry->fields.count("俯仰角（°）") == 0 &&
                telemetry->fields.count("偏航角（°）") == 0 &&
                std::get<std::int64_t>(telemetry->fields.at("有效目标数")) == 2 &&
                std::get<std::int64_t>(telemetry->fields.at("变倍（×）")) == 4,
            "捷联机械角、目标状态或变倍解码错误");
    require(!fixture.driver->apply(PodCommand{PodCommandType::SetRate, 1, 1}, now, fixture.error) &&
                transport.outgoing.size() == 1 && transport.outgoing.back()[3] == 0,
            "捷联机械指令被转发");
}

class LocalSocket {
   public:
    LocalSocket() : fd(::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0)) {
        require(fd >= 0, "创建测试 UDP socket 失败");
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        require(::bind(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == 0,
                "绑定测试 UDP socket 失败");
        socklen_t length = sizeof(address);
        require(::getsockname(fd, reinterpret_cast<sockaddr*>(&address), &length) == 0, "读取测试端口失败");
    }
    ~LocalSocket() { ::close(fd); }
    LocalSocket(const LocalSocket&) = delete;
    LocalSocket& operator=(const LocalSocket&) = delete;

    int fd;
    sockaddr_in address{};
};

void test_udp() {
    std::string error;
    require(!make_pod_transport("udp://localhost:65536", error) &&
                !make_pod_transport("udp://:8001", error) && !make_pod_transport("serial:///dev/ttyS0:115200", error),
            "接受了非法或当前不支持的 pod_control_url");
    LocalSocket gateway;
    LocalSocket wrong_peer;
    auto transport = make_pod_transport("udp://127.0.0.1:" + std::to_string(ntohs(gateway.address.sin_port)), error);
    require(transport && transport->start(error) && transport->send({0x42}, error), "测试 UDP 发送失败");
    pollfd readable{gateway.fd, POLLIN, 0};
    require(::poll(&readable, 1, 500) == 1, "测试网关未收到帧");
    sockaddr_in peer{};
    socklen_t peer_size = sizeof(peer);
    std::uint8_t byte = 0;
    require(::recvfrom(gateway.fd, &byte, 1, 0, reinterpret_cast<sockaddr*>(&peer), &peer_size) == 1 && byte == 0x42,
            "测试网关收到了错误的内容");
    byte = 0xCC;
    require(::sendto(wrong_peer.fd, &byte, 1, 0, reinterpret_cast<const sockaddr*>(&peer), peer_size) == 1,
            "测试错误对端发送失败");
    byte = 0xDD;
    require(::sendto(gateway.fd, &byte, 1, 0, reinterpret_cast<const sockaddr*>(&peer), peer_size) == 1,
            "测试配置对端发送失败");
    PodFrame reply;
    const auto deadline = Clock::now() + 500ms;
    while (!transport->receive(reply) && Clock::now() < deadline) {
        std::this_thread::sleep_for(2ms);
    }
    require(reply == PodFrame{0xDD}, "UDP 未使用同一源端口收发或未过滤错误对端");
    require(!transport->receive(reply), "UDP 接收了错误对端的数据报");
}

}  // namespace

int main() {
    test_aisheng();
    test_zhongke();
    test_jielian();
    test_udp();
    return 0;
}
