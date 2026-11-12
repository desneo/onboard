#include "camera/pod_driver.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace plane::camera {
namespace {

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using plane::model::PodCommand;
using plane::model::PodCommandType;

constexpr auto kTelemetryTimeout = 1s;
constexpr int kMaxFramesPerTick = 64;

std::uint8_t checksum(const PodFrame& frame, std::size_t offset) {
    unsigned sum = 0;
    for (; offset + 1 < frame.size(); ++offset) {
        sum += frame[offset];
    }
    return static_cast<std::uint8_t>(sum);
}

std::uint16_t read_u16(const PodFrame& frame, std::size_t offset) {
    return static_cast<std::uint16_t>(frame[offset] | (frame[offset + 1] << 8));
}

int read_i16(const PodFrame& frame, std::size_t offset) {
    const int value = read_u16(frame, offset);
    return value >= 0x8000 ? value - 0x10000 : value;
}

std::uint32_t read_u32(const PodFrame& frame, std::size_t offset) {
    return static_cast<std::uint32_t>(read_u16(frame, offset)) |
           (static_cast<std::uint32_t>(read_u16(frame, offset + 2)) << 16);
}

void write_u16(PodFrame& frame, std::size_t offset, std::uint16_t value) {
    frame[offset] = static_cast<std::uint8_t>(value);
    frame[offset + 1] = static_cast<std::uint8_t>(value >> 8);
}

void write_u32(PodFrame& frame, std::size_t offset, std::uint32_t value) {
    write_u16(frame, offset, static_cast<std::uint16_t>(value));
    write_u16(frame, offset + 2, static_cast<std::uint16_t>(value >> 16));
}

namespace aisheng_frame {

constexpr std::uint8_t kUnlock = 0x9E;
constexpr std::uint8_t kSetRate = 0xB1;
constexpr std::uint8_t kSetAngles = 0x90;

PodFrame encode(std::uint8_t command, long yaw = 0, long pitch = 0, std::uint32_t timestamp_ms = 0) {
    PodFrame frame(45, 0);
    frame[0] = 0xAA;
    frame[1] = 0x55;
    frame[2] = 0x2D;
    frame[3] = command;
    write_u16(frame, 4, static_cast<std::uint16_t>(yaw));
    write_u16(frame, 6, static_cast<std::uint16_t>(pitch));
    write_u32(frame, 40, timestamp_ms);
    frame.back() = checksum(frame, 3);
    return frame;
}

bool valid(const PodFrame& frame) {
    return frame.size() == 104 && frame[0] == 0x55 && frame[1] == 0xAA && frame[2] == 0x68 &&
           frame.back() == checksum(frame, 3);
}

PodTelemetry decode_shared(const PodFrame& frame) {
    const bool tracking = (frame[4] & 0x01) != 0;
    const int zoom = frame[76] & 0x03;
    const char* self_check = "未知";
    switch (frame[3]) {
        case 1: self_check = "正常"; break;
        case 2: self_check = "故障"; break;
        case 3: self_check = "自检中"; break;
        default: break;
    }
    PodTelemetry result;
    auto& fields = result.fields;
    fields.emplace("自检状态", std::string(self_check));
    fields.emplace("跟踪有效", tracking);
    fields.emplace("工作状态字", std::int64_t{frame[4]});
    fields.emplace("视频推流已启用", (frame[4] & 0x80) != 0);
    fields.emplace("自动锁定已启用", frame[27] == 1);
    fields.emplace("设备上电时长（ms）", std::int64_t{read_u32(frame, 99)});
    if (zoom != 0) {
        fields.emplace("变倍（×）", std::int64_t{1} << (zoom - 1));
    }
    if (tracking) {
        fields.emplace("俯仰失调角（°）", read_i16(frame, 17) / 100.0);
        fields.emplace("偏航失调角（°）", read_i16(frame, 19) / 100.0);
        fields.emplace("波门中心 X（px）", std::int64_t{read_i16(frame, 9)});
        fields.emplace("波门中心 Y（px）", std::int64_t{read_i16(frame, 11)});
        fields.emplace("波门宽度（px）", std::int64_t{read_u16(frame, 77)});
        fields.emplace("波门高度（px）", std::int64_t{read_u16(frame, 79)});
        fields.emplace("跟踪置信度", frame[21] / 10.0);
    }
    return result;
}

}  // namespace aisheng_frame

namespace zhongke_frame {

constexpr std::uint8_t kSetRate = 0x24;
constexpr std::uint8_t kSetAngles = 0x26;

PodFrame encode(std::uint8_t command, long yaw = 0, long pitch = 0) {
    PodFrame frame(16, 0);
    frame[0] = 0xEB;
    frame[1] = 0x90;
    frame[2] = command;
    write_u16(frame, 3, static_cast<std::uint16_t>(yaw));
    write_u16(frame, 5, static_cast<std::uint16_t>(pitch));
    frame.back() = checksum(frame, 0);
    return frame;
}

bool valid(const PodFrame& frame) {
    return frame.size() == 32 && frame[0] == 0xEE && frame[1] == 0x16 && frame.back() == checksum(frame, 0);
}

}  // namespace zhongke_frame

// 会话保活由 UDP 接入工厂提供。帧编码、遥测解码和命令执行只依赖 PodTransport。
struct SessionKeepalive {
    PodFrame frame;
    Clock::duration period;
};

class FramePodDriver : public PodDriver {
   public:
    FramePodDriver(std::unique_ptr<PodTransport> transport, SessionKeepalive keepalive)
        : transport_(std::move(transport)), keepalive_(std::move(keepalive)) {}

    bool start(std::string& error) override { return transport_->start(error); }

    std::optional<PodTelemetry> tick(Clock::time_point now) override {
        if (last_sample_ && !fresh(now)) {
            last_sample_.reset();
            on_telemetry_lost();
        }

        if (now >= next_keepalive_) {
            std::string error;
            send(keepalive_.frame, error);
            next_keepalive_ = now + keepalive_.period;
        }

        std::optional<PodTelemetry> latest;
        PodFrame frame;
        for (int count = 0; count < kMaxFramesPerTick && transport_->receive(frame); ++count) {
            if (auto sample = decode(frame)) {
                last_sample_ = now;
                latest = std::move(sample);
            }
        }
        after_receive(now);
        return latest;
    }

    void stop() override {
        transport_->stop();
        next_keepalive_ = Clock::time_point::min();
        last_sample_.reset();
        on_telemetry_lost();
    }

   protected:
    bool fresh(Clock::time_point now) const { return last_sample_ && now - *last_sample_ < kTelemetryTimeout; }

    bool send(const PodFrame& frame, std::string& error) { return transport_->send(frame, error); }

   private:
    virtual std::optional<PodTelemetry> decode(const PodFrame& frame) = 0;
    virtual void on_telemetry_lost() {}
    virtual void after_receive(Clock::time_point) {}

    std::unique_ptr<PodTransport> transport_;
    SessionKeepalive keepalive_;
    Clock::time_point next_keepalive_ = Clock::time_point::min();
    std::optional<Clock::time_point> last_sample_;
};

struct MotionLimits {
    double max_yaw_deg;
    double min_pitch_deg;
    double max_pitch_deg;
    double max_rate_deg_s;
};

class MovingPodDriver : public FramePodDriver {
   public:
    MovingPodDriver(std::unique_ptr<PodTransport> transport, SessionKeepalive keepalive, MotionLimits limits)
        : FramePodDriver(std::move(transport), std::move(keepalive)), limits_(limits) {}

    bool can_move() const override { return true; }

    bool apply(const PodCommand& command, Clock::time_point now, std::string& error) override {
        if (command.type == PodCommandType::SetRate) {
            if (!std::isfinite(command.yawRate_deg_s) || !std::isfinite(command.pitchRate_deg_s)) {
                error = "吊舱转速必须是有限数值";
                return false;
            }
            if ((command.yawRate_deg_s != 0.0 || command.pitchRate_deg_s != 0.0) && !fresh(now)) {
                error = "吊舱遥测未就绪";
                return false;
            }
            const double limit = limits_.max_rate_deg_s;
            return set_rate(std::clamp(command.yawRate_deg_s, -limit, limit),
                            std::clamp(command.pitchRate_deg_s, -limit, limit), now, error);
        }

        if (!fresh(now)) {
            error = "吊舱遥测未就绪";
            return false;
        }
        double yaw = command.yaw_deg;
        double pitch = command.pitch_deg;
        switch (command.type) {
            case PodCommandType::ZeroPitch:
                yaw = measured_yaw_;
                pitch = 0.0;
                break;
            case PodCommandType::ZeroYaw:
                yaw = 0.0;
                pitch = measured_pitch_;
                break;
            case PodCommandType::SetAngles:
                break;
            default:
                error = "不支持的吊舱控制指令";
                return false;
        }
        if (!std::isfinite(yaw) || !std::isfinite(pitch) || std::abs(yaw) > limits_.max_yaw_deg ||
            pitch < limits_.min_pitch_deg || pitch > limits_.max_pitch_deg) {
            error = "指定吊舱角度超出机械限位";
            return false;
        }
        return set_angles(yaw, pitch, now, error);
    }

   protected:
    void set_measurement(PodTelemetry& telemetry, double yaw, double pitch) {
        measured_yaw_ = yaw;
        measured_pitch_ = pitch;
        telemetry.fields.emplace("俯仰角（°）", pitch);
        telemetry.fields.emplace("偏航角（°）", yaw);
    }

   private:
    virtual bool set_rate(double yaw, double pitch, Clock::time_point now, std::string& error) = 0;
    virtual bool set_angles(double yaw, double pitch, Clock::time_point now, std::string& error) = 0;

    MotionLimits limits_;
    double measured_yaw_ = 0.0;
    double measured_pitch_ = 0.0;
};

class AishengAsd2Driver final : public MovingPodDriver {
   public:
    AishengAsd2Driver(std::unique_ptr<PodTransport> transport, SessionKeepalive keepalive)
        : MovingPodDriver(std::move(transport), std::move(keepalive), {60.0, -80.0, 30.0, 60.0}) {}

   private:
    enum class ServoState { Unknown, Locked, Ready };

    std::optional<PodTelemetry> decode(const PodFrame& frame) override {
        if (!aisheng_frame::valid(frame)) {
            return std::nullopt;
        }
        auto telemetry = aisheng_frame::decode_shared(frame);
        set_measurement(telemetry, read_i16(frame, 7) / 100.0, read_i16(frame, 5) / 100.0);
        auto& fields = telemetry.fields;
        fields.emplace("俯仰角速度（°/s）", read_i16(frame, 13) / 100.0);
        fields.emplace("偏航角速度（°/s）", read_i16(frame, 15) / 100.0);
        fields.emplace("活动镜头", std::string((frame[76] & 0x10) != 0 ? "长焦" : "广角"));
        const unsigned servo = (frame[26] >> 5) & 0x07;
        servo_state_ = ServoState::Unknown;
        const char* servo_state = "未知";
        switch (servo) {
            case 1:
                servo_state_ = ServoState::Locked;
                servo_state = "电锁";
                break;
            case 4:
                servo_state_ = ServoState::Ready;
                servo_state = "自稳或自由";
                break;
            case 6:
                servo_state_ = ServoState::Ready;
                servo_state = "跟踪";
                break;
            default: break;
        }
        fields.emplace("伺服状态", std::string(servo_state));
        return telemetry;
    }

    bool set_rate(double yaw, double pitch, Clock::time_point now, std::string& error) override {
        const long raw_yaw = std::lround(yaw / 0.6);
        const long raw_pitch = std::lround(pitch / 0.6);
        const auto frame = encode(aisheng_frame::kSetRate, raw_yaw, raw_pitch, now);
        if (raw_yaw == 0 && raw_pitch == 0) {
            pending_.clear();
            return send(frame, error);
        }
        return send_motion(frame, now, 500ms, error);
    }

    bool set_angles(double yaw, double pitch, Clock::time_point now, std::string& error) override {
        const auto frame = encode(aisheng_frame::kSetAngles, std::lround(yaw * 100.0), std::lround(pitch * 100.0), now);
        return send_motion(frame, now, 1s, error);
    }

    static PodFrame encode(std::uint8_t command, long yaw, long pitch, Clock::time_point now) {
        const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
        return aisheng_frame::encode(command, yaw, pitch, static_cast<std::uint32_t>(milliseconds.count()));
    }

    bool send_motion(const PodFrame& frame, Clock::time_point now, Clock::duration expiry, std::string& error) {
        pending_.clear();
        if (servo_state_ == ServoState::Ready) {
            return send(frame, error);
        }
        if (servo_state_ == ServoState::Unknown) {
            error = "艾晟伺服状态未知";
            return false;
        }
        if (!request_unlock(now, error)) {
            return false;
        }
        pending_ = frame;
        pending_deadline_ = now + expiry;
        return true;
    }

    bool request_unlock(Clock::time_point now, std::string& error) {
        if (now < next_unlock_) {
            return true;
        }
        next_unlock_ = now + 200ms;
        return send(encode(aisheng_frame::kUnlock, 0, 0, now), error);
    }

    void on_telemetry_lost() override {
        servo_state_ = ServoState::Unknown;
        pending_.clear();
        next_unlock_ = Clock::time_point::min();
    }

    void after_receive(Clock::time_point now) override {
        if (pending_.empty()) {
            return;
        }
        if (now >= pending_deadline_ || !fresh(now) || servo_state_ == ServoState::Unknown) {
            pending_.clear();
            return;
        }
        std::string error;
        if (servo_state_ == ServoState::Locked) {
            request_unlock(now, error);
        } else if (send(pending_, error)) {
            pending_.clear();
        }
    }

    ServoState servo_state_ = ServoState::Unknown;
    PodFrame pending_;
    Clock::time_point pending_deadline_{};
    Clock::time_point next_unlock_ = Clock::time_point::min();
};

class JielianA2416aDriver final : public FramePodDriver {
   public:
    using FramePodDriver::FramePodDriver;

    bool can_move() const override { return false; }

    bool apply(const PodCommand&, Clock::time_point, std::string& error) override {
        error = "捷联 A2416A 不支持机械转动";
        return false;
    }

   private:
    std::optional<PodTelemetry> decode(const PodFrame& frame) override {
        if (!aisheng_frame::valid(frame)) {
            return std::nullopt;
        }
        auto telemetry = aisheng_frame::decode_shared(frame);
        std::int64_t target_count = 0;
        for (std::size_t offset = 28; offset <= 64; offset += 12) {
            target_count += frame[offset] != 0;
        }
        telemetry.fields.emplace("有效目标数", target_count);
        return telemetry;
    }
};

class ZhongkeD80Driver final : public MovingPodDriver {
   public:
    ZhongkeD80Driver(std::unique_ptr<PodTransport> transport, SessionKeepalive keepalive)
        : MovingPodDriver(std::move(transport), std::move(keepalive), {90.0, -90.0, 30.0, 20.0}) {}

   private:
    std::optional<PodTelemetry> decode(const PodFrame& frame) override {
        if (!zhongke_frame::valid(frame)) {
            return std::nullopt;
        }
        PodTelemetry telemetry;
        set_measurement(telemetry, read_i16(frame, 14) / 100.0, read_i16(frame, 12) / 100.0);
        auto& fields = telemetry.fields;
        const bool tracking = (frame[2] & 0x04) != 0;
        fields.emplace("跟踪有效", tracking);
        fields.emplace("电机已使能", (frame[3] & 0x08) != 0);
        fields.emplace("变倍（×）", (frame[4] | ((frame[5] & 0x0F) << 8)) / 10.0);
        fields.emplace("俯仰角速度（°/s）", read_i16(frame, 22) / 100.0);
        fields.emplace("偏航角速度（°/s）", read_i16(frame, 24) / 100.0);
        fields.emplace("状态字 1", std::int64_t{frame[2]});
        fields.emplace("状态字 2", std::int64_t{frame[3]});
        fields.emplace("状态字 3", std::int64_t{frame[5]});
        fields.emplace("自检完成", (frame[28] & 0x80) != 0);
        fields.emplace("控制故障", (frame[28] & 0x02) != 0);
        fields.emplace("成像故障", (frame[28] & 0x01) != 0);
        if (tracking) {
            fields.emplace("俯仰失调角（°）", read_i16(frame, 8) / 20.0);
            fields.emplace("偏航失调角（°）", read_i16(frame, 6) / 20.0);
        }
        return telemetry;
    }

    bool set_rate(double yaw, double pitch, Clock::time_point, std::string& error) override {
        return send(zhongke_frame::encode(zhongke_frame::kSetRate, std::lround(yaw * 10.0), std::lround(pitch * 10.0)),
                    error);
    }

    bool set_angles(double yaw, double pitch, Clock::time_point, std::string& error) override {
        return send(zhongke_frame::encode(zhongke_frame::kSetAngles, std::lround(yaw * 100.0),
                                          std::lround(pitch * 100.0)), error);
    }
};

}  // namespace

std::unique_ptr<PodDriver> make_pod_driver(std::string_view model, std::unique_ptr<PodTransport> transport,
                                          std::string& error) {
    if (!transport) {
        error = "吊舱驱动缺少有效传输";
        return nullptr;
    }
    if (model == "aisheng_asd2") {
        return std::make_unique<AishengAsd2Driver>(std::move(transport), SessionKeepalive{aisheng_frame::encode(0), 100ms});
    }
    if (model == "jielian_a2416a") {
        return std::make_unique<JielianA2416aDriver>(std::move(transport),
                                                   SessionKeepalive{aisheng_frame::encode(0), 100ms});
    }
    if (model == "zhongke_d80") {
        return std::make_unique<ZhongkeD80Driver>(std::move(transport), SessionKeepalive{zhongke_frame::encode(0), 200ms});
    }
    error = "未接入的 pod_model：" + std::string(model);
    return nullptr;
}

}  // namespace plane::camera
