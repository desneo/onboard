#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "communication/communication_events.h"
#include "target/target_store.h"
#include "task/nodes/midcourse_controller.h"

namespace {

std::int64_t test_now_ms = 1'000'000;

struct AlgorithmCalls {
    int init = 0;
    int update = 0;
    int update_target = 0;
    bool fail_update = false;
    bool omit_yaw = false;
    std::int64_t init_advance_ms = 0;
    std::int64_t update_advance_ms = 0;
} algorithm_calls;

void require(bool condition, const std::string& detail) {
    if (!condition) throw std::runtime_error(detail);
}

struct Entry {
    std::string text;
    spdlog::level::level_enum level;
    std::int64_t time_ms;
};

class CaptureSink final : public spdlog::sinks::base_sink<std::mutex> {
   public:
    std::vector<Entry> matching(const std::string& text) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<Entry> result;
        for (const auto& entry : entries_) {
            if (entry.text.find(text) != std::string::npos) result.push_back(entry);
        }
        return result;
    }

   protected:
    void sink_it_(const spdlog::details::log_msg& message) override {
        entries_.push_back({std::string(message.payload.data(), message.payload.size()), message.level, test_now_ms});
    }
    void flush_() override {}

   private:
    std::vector<Entry> entries_;
};

class FakeFlightControl final : public plane::flight_control::FlightControlApi {
   public:
    int velocity_calls = 0;
    int attitude_calls = 0;
    bool arm(std::string&) override { return true; }
    bool disarm(std::string&) override { return true; }
    bool takeoff(float, std::string&) override { return true; }
    bool land(std::string&) override { return true; }
    bool return_to_launch(std::string&) override { return true; }
    bool hold(std::string&) override { return true; }
    bool stop_motor(std::string&) override { return true; }
    bool goto_waypoint(const plane::model::GeoPoint&, std::string&) override { return true; }
    bool execute_route(const std::vector<plane::model::GeoPoint>&, std::string&) override { return true; }
    bool set_ned_velocity(const plane::model::NedVelocity&, std::string&) override { return true; }
    bool set_ned_velocity_acceleration(const plane::algorithm::VelocityAccelerationCommand&, double,
                                       std::string&) override {
        ++velocity_calls;
        return true;
    }
    bool set_attitude(const plane::flight_control::AttitudeSetpoint&, std::string&) override {
        ++attitude_calls;
        return true;
    }
    bool get_cached_flight_control_params(plane::flight_control::FlightControlParams&, std::string&) const override {
        return false;
    }
    void clear_offboard() override {}
    std::string link_state() const override { return "connected"; }
    plane::flight_control::FlightStatus flight_status() const override { return {}; }
};

class FakeStateStore final : public plane::state::StateStoreApi {
   public:
    plane::model::VehicleState value;
    plane::model::VehicleState own_state() const override { return value; }
    std::vector<plane::model::VehicleState> peer_states() const override { return {}; }
    bool peer_state(const std::string&, plane::model::VehicleState&) const override { return false; }
    plane::state::GcsLinkState gcs_link() const override { return {}; }
    void update_own_state(const plane::model::VehicleState& state) override { value = state; }
    void update_peer_state(const plane::model::VehicleState&) override {}
    void notify_station_seen() override {}
};

plane::model::GcsTarget target(int index) {
    plane::model::GcsTarget value;
    value.trackId = "log-target";
    value.timestamp_ms = 1'800'000'000'000LL + index * 1000LL;
    value.geoPos.latitude = 30.0 + index * 0.00001;
    value.geoPos.longitude = 120.0;
    value.geoPos.height = 20.0F;
    value.geoPos.altitude = 300.0F;
    return value;
}

void require_fields(const std::string& text, const std::vector<std::string>& fields) {
    for (const auto& field : fields) {
        require(text.find(field) != std::string::npos, "算法日志缺少字段或原始值: " + field);
    }
}

void require_update_pairs(CaptureSink& sink, const std::vector<std::int64_t>& times) {
    const auto entries = sink.matching("MidcourseAlgorithm::update ");
    require(entries.size() == times.size() * 2, "update 输入输出日志数量不符合降频预期");
    for (std::size_t index = 0; index < times.size(); ++index) {
        const auto& input = entries[index * 2];
        const auto& output = entries[index * 2 + 1];
        require(input.text.find("::update 输入:") != std::string::npos &&
                    output.text.find("::update 输出:") != std::string::npos,
                "update 输入输出日志未按调用成对记录");
        require(input.time_ms == times[index] && output.time_ms == times[index],
                "update 日志未在预期的单调时间输出");
        require(input.level == spdlog::level::info && output.level == spdlog::level::info,
                "update 输入输出必须保留 info 级别");
    }
}

}  // namespace

namespace plane::core {
std::int64_t now_ms() { return test_now_ms; }
}  // namespace plane::core

namespace plane::algorithm {

// 仅在本测试目标内替代算法门面，精确控制失败时机并统计每次调用。
struct MidcourseAlgorithm::Impl {
    MidcourseControlMode mode = MidcourseControlMode::VelocityAcceleration;
};

MidcourseAlgorithm::MidcourseAlgorithm() : impl_(std::make_unique<Impl>()) {}
MidcourseAlgorithm::~MidcourseAlgorithm() = default;
MidcourseAlgorithm::MidcourseAlgorithm(MidcourseAlgorithm&&) noexcept = default;
MidcourseAlgorithm& MidcourseAlgorithm::operator=(MidcourseAlgorithm&&) noexcept = default;

bool MidcourseAlgorithm::init(const std::vector<TargetMeasurement>& history, const plane::model::GeoPoint&,
                             const MidCourseConfig& config, std::string& error) {
    require(history.size() == 20, "日志测试初始化未获得完整 20 帧历史");
    ++algorithm_calls.init;
    impl_->mode = config.control_output_mode;
    test_now_ms += algorithm_calls.init_advance_ms;
    error.clear();
    return true;
}

bool MidcourseAlgorithm::updateTarget(const TargetMeasurement&, std::string& error) {
    ++algorithm_calls.update_target;
    error.clear();
    return true;
}

bool MidcourseAlgorithm::update(const MidcourseVehicleState&, MidCourseOutput& output, std::string& error) {
    ++algorithm_calls.update;
    test_now_ms += algorithm_calls.update_advance_ms;
    if (algorithm_calls.fail_update) {
        error = "测试注入的算法原始错误";
        return false;
    }
    output.cmd.mode = impl_->mode;
    if (impl_->mode == MidcourseControlMode::VelocityAcceleration) {
        VelocityAccelerationCommand command{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, std::nullopt};
        if (!algorithm_calls.omit_yaw) command.yaw_rad = 0.7;
        output.cmd.value = command;
    } else {
        AttitudeCommand command;
        command.thrust_body[2] = 0.3;
        output.cmd.value = command;
    }
    output.eta_seconds = 12.5;
    output.following = true;
    output.terminal_confirmed = false;
    error.clear();
    return true;
}

}  // namespace plane::algorithm

int main() {
    try {
        auto sink = std::make_shared<CaptureSink>();
        auto logger = std::make_shared<spdlog::logger>("中制导日志测试", sink);
        logger->set_level(spdlog::level::trace);
        spdlog::set_default_logger(logger);
        plane::core::EventBus events;
        plane::core::AppConfig config;
        FakeFlightControl flight;
        FakeStateStore state;
        state.value.geoPos = target(0).geoPos;
        state.value.geoPos.latitude = -30.12345678;
        state.value.startPos = target(1).geoPos;
        state.value.vn_m_s = 7.0;
        state.value.ve_m_s = 8.0;
        state.value.vd_m_s = 9.0;
        state.value.yaw_deg = 42.0F;
        plane::target::TargetStore store(events, config);
        require(store.start(), "日志测试目标缓存启动失败");
        plane::task::MidcourseController controller(events, config, flight, state, store);
        controller.start();
        int next_target_index = 0;
        auto publish_session_history = [&]() {
            for (int count = 0; count < 20; ++count) {
                events.publish(plane::communication::events::kGcsTargetReceived, target(next_target_index++));
            }
        };
        publish_session_history();
        std::string error;
        auto tick = [&](std::int64_t elapsed_ms) {
            test_now_ms = 1'000'000 + elapsed_ms;
            error.clear();
            require(controller.tick(error) == plane::task::MidcourseController::TickResult::Running,
                    "日志测试控制 tick 失败: " + error);
        };
        require(controller.begin("log-target", store.gcs_target_revision(), error), "日志测试会话启动失败");
        publish_session_history();
        tick(0);
        require_update_pairs(*sink, {1'000'000});
        for (int index = 1; index <= 9; ++index) {
            test_now_ms = 1'000'000 + index * 50;
            if (index <= 2) {
                events.publish(plane::communication::events::kGcsTargetReceived, target(next_target_index++));
            }
            tick(index * 50);
        }
        tick(499);
        require_update_pairs(*sink, {1'000'000});
        require(algorithm_calls.update == 11 && flight.velocity_calls == 11,
                "日志降频期间未逐 tick 调用算法并下发控制");
        tick(500);
        require_update_pairs(*sink, {1'000'000});
        tick(1000);
        require_update_pairs(*sink, {1'000'000, 1'001'000});
        require(algorithm_calls.update == 13 && flight.velocity_calls == 13,
                "1000 毫秒日志边界影响了逐 tick 算法或控制调用");
        require(algorithm_calls.update_target == 2 && sink->matching("::updateTarget 输入:").size() == 2 &&
                    sink->matching("::updateTarget 输出:").size() == 2,
                "updateTarget 日志不应跟随 update 降频");
        const auto velocity_output = sink->matching("::update 输出:").front().text;
        require_fields(velocity_output, {"mode=VelocityAcceleration", "VelocityAccelerationCommand{", "vn_m_s=1",
                                        "ve_m_s=2", "vd_m_s=3", "an_m_s2=4", "ae_m_s2=5", "ad_m_s2=6",
                                        "yaw_rad=0.7", "eta_seconds=12.5", "following=true",
                                        "terminal_confirmed=false", "返回值=true", "error="});
        require_fields(sink->matching("::update 输入:").front().text,
                       {"geoPos={latitude=-30.123457 longitude=120.000000 height=20.0 altitude=300.0}", "startPos={latitude=",
                        "vn_m_s=7", "ve_m_s=8", "vd_m_s=9", "yaw_rad="});
        require_fields(sink->matching("::init 输入:").front().text,
                       {"history=[", "timestamp_ms=1800000020000", "timestamp_ms=1800000039000", "ned_ref={",
                        "max_horizontal_speed_m_s=", "update_dt_s=", "recovery_vz_deadband_m_s=0.1",
                        "recovery_altitude_tolerance_m=0.3", "confirm_hold_s="});

        // 新会话不继承上一会话的日志时间，两个输出模式都保留完整字段。
        controller.stop();
        config.midcourse_algorithm.control_output_mode = plane::algorithm::MidcourseControlMode::Attitude;
        test_now_ms = 1'001'001;
        require(controller.begin("log-target", store.gcs_target_revision(), error), "姿态日志会话启动失败");
        publish_session_history();
        tick(1001);
        require_update_pairs(*sink, {1'000'000, 1'001'000, 1'001'001});
        require(algorithm_calls.init == 2 && sink->matching("::init 输入:").size() == 2 &&
                    sink->matching("::init 输出:").size() == 2,
                "每次 init 调用都应记录完整输入输出");
        require_fields(sink->matching("::update 输出:").back().text,
                       {"mode=Attitude", "AttitudeCommand{", "quat_body_to_ned=[1, 0, 0, 0]",
                        "thrust_body=[0, 0, 0.3]", "返回值=true"});

        // 上次记录后仅经过 1 毫秒，update 失败仍须立即成对记录原始错误。
        algorithm_calls.fail_update = true;
        test_now_ms = 1'001'002;
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Failure,
                "注入的 update 失败未终止当前 tick");
        require_update_pairs(*sink, {1'000'000, 1'001'000, 1'001'001, 1'001'002});
        require_fields(sink->matching("::update 输出:").back().text,
                       {"返回值=false", "output=<无有效输出>", "error=测试注入的算法原始错误"});
        require(error == "中制导 update 失败: 测试注入的算法原始错误", "update 失败未保留原始错误");
        require(algorithm_calls.update == 15 && flight.velocity_calls == 13 && flight.attitude_calls == 1,
                "失败的 update 仍下发了控制指令");

        controller.stop();
        algorithm_calls.fail_update = false;
        algorithm_calls.omit_yaw = true;
        config.midcourse_algorithm.control_output_mode = plane::algorithm::MidcourseControlMode::VelocityAcceleration;
        test_now_ms = 1'001'003;
        require(controller.begin("log-target", store.gcs_target_revision(), error), "缺省航向日志会话启动失败");
        publish_session_history();
        tick(1003);
        require_fields(sink->matching("::update 输出:").back().text, {"yaw_rad=nullopt", "返回值=true"});
        require_update_pairs(*sink, {1'000'000, 1'001'000, 1'001'001, 1'001'002, 1'001'003});
        controller.stop();

        // init 自身耗时跨过超时阈值时，不应再调用 update 或下发控制。
        config.midcourse_target_timeout_ms = 10;
        test_now_ms = 1'002'000;
        algorithm_calls.init_advance_ms = 11;
        const int init_before_slow_init = algorithm_calls.init;
        const int update_before_slow_init = algorithm_calls.update;
        const int controls_before_slow_init = flight.velocity_calls + flight.attitude_calls;
        require(controller.begin("log-target", store.gcs_target_revision(), error), "初始化耗时超时场景启动失败");
        publish_session_history();
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Failure &&
                    error.find("中制导目标雷达信息超时:") == 0 &&
                    error.find("未更新时长=11毫秒 target_timeout_ms=10") != std::string::npos,
                "init 耗时跨过阈值后应返回目标超时: " + error);
        require(algorithm_calls.init == init_before_slow_init + 1 &&
                    algorithm_calls.update == update_before_slow_init &&
                    flight.velocity_calls + flight.attitude_calls == controls_before_slow_init,
                "init 耗时跨过阈值后仍调用了 update 或下发控制");
        require_update_pairs(*sink, {1'000'000, 1'001'000, 1'001'001, 1'001'002, 1'001'003});
        controller.stop();

        // update 返回成功但计算期间跨过超时阈值时，仍须拦截本次控制下发。
        algorithm_calls.init_advance_ms = 0;
        algorithm_calls.update_advance_ms = 11;
        test_now_ms = 1'003'000;
        const int update_before_slow_update = algorithm_calls.update;
        const int controls_before_slow_update = flight.velocity_calls + flight.attitude_calls;
        require(controller.begin("log-target", store.gcs_target_revision(), error), "算法计算耗时超时场景启动失败");
        publish_session_history();
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Failure &&
                    error.find("中制导目标雷达信息超时:") == 0 &&
                    error.find("未更新时长=11毫秒 target_timeout_ms=10") != std::string::npos,
                "update 耗时跨过阈值后应返回目标超时: " + error);
        require(algorithm_calls.update == update_before_slow_update + 1 &&
                    flight.velocity_calls + flight.attitude_calls == controls_before_slow_update,
                "update 耗时跨过阈值后仍下发控制");
        require_update_pairs(*sink,
                             {1'000'000, 1'001'000, 1'001'001, 1'001'002, 1'001'003, 1'003'011});
        controller.stop();
    } catch (const std::exception& exception) {
        spdlog::set_default_logger(spdlog::stderr_color_mt("中制导日志测试错误"));
        SPDLOG_ERROR("中制导日志测试失败: {}", exception.what());
        return 1;
    }
    return 0;
}
