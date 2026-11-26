#include "task/nodes/midcourse_controller.h"

#include <spdlog/spdlog.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "communication/communication_events.h"
#include "core/config.h"
#include "core/event_bus.h"
#include "flight_control/flight_control_api.h"
#include "state/state_store_api.h"
#include "target/target_store.h"

namespace {

std::int64_t test_now_ms = 1'000'000;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

class FakeFlightControl final : public plane::flight_control::FlightControlApi {
   public:
    bool cached_flight_params_available = true;
    mutable int cached_flight_params_gets = 0;
    int attitude_calls = 0;
    int velocity_calls = 0;
    int clear_calls = 0;
    plane::flight_control::AttitudeSetpoint last_attitude;
    std::optional<plane::algorithm::VelocityAccelerationCommand> last_velocity;

    bool arm(std::string&) override { return true; }
    bool disarm(std::string&) override { return true; }
    bool takeoff(float, std::string&) override { return true; }
    bool land(std::string&) override { return true; }
    bool return_to_launch(std::string&) override { return true; }
    bool hold(std::string&) override { return true; }
    bool stop_motor(std::string&) override { return true; }
    bool goto_waypoint(const plane::model::GeoPoint&, std::string&) override { return true; }
    bool execute_route(const std::vector<plane::model::GeoPoint>&, std::string&) override { return true; }
    bool set_ned_velocity(const plane::model::NedVelocity&, std::string&) override {
        throw std::runtime_error("中制导下发速度时遗漏加速度前馈");
    }
    bool set_ned_velocity_acceleration(const plane::algorithm::VelocityAccelerationCommand& value,
                                       double current_yaw_deg, std::string&) override {
        require(std::isfinite(value.an_m_s2) && std::isfinite(value.ae_m_s2) && std::isfinite(value.ad_m_s2),
                "中制导下发的加速度必须是有限数");
        require(current_yaw_deg == 42.0, "未传入当前航向用于缺省航向");
        last_velocity = value;
        ++velocity_calls;
        return true;
    }
    bool set_attitude(const plane::flight_control::AttitudeSetpoint& value, std::string&) override {
        last_attitude = value;
        ++attitude_calls;
        return true;
    }
    bool get_cached_flight_control_params(plane::flight_control::FlightControlParams& value,
                                          std::string& error) const override {
        ++cached_flight_params_gets;
        if (!cached_flight_params_available) {
            error = "PX4 参数缓存未就绪";
            return false;
        }
        value.mpc_xy_vel_max = 15.0;
        value.mpc_z_vel_max_up = 5.0;
        value.mpc_z_vel_max_dn = 5.0;
        value.mpc_acc_hor_max = 6.0;
        return true;
    }
    void clear_offboard() override { ++clear_calls; }
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

plane::model::GcsTarget target(const std::string& id, int index) {
    plane::model::GcsTarget result;
    result.trackId = id;
    result.timestamp_ms = 1'800'000'000'000LL + static_cast<std::int64_t>(index) * 1000;
    result.geoPos.latitude = 30.010 + 0.00001 * static_cast<double>(index);
    result.geoPos.longitude = 120.0;
    result.geoPos.altitude = 300.0F;
    return result;
}

struct TimeoutFixture {
    explicit TimeoutFixture(int target_history_size = 20)
        : store(events, config), controller(events, config, flight, state, store) {
        test_now_ms = 1'000'000;
        config.midcourse_target_history_size = target_history_size;
        state.value.geoPos = target("", 0).geoPos;
        state.value.startPos = state.value.geoPos;
        state.value.yaw_deg = 42.0F;
        require(store.start(), "超时场景目标存储启动失败");
        controller.start();
    }

    ~TimeoutFixture() {
        controller.stop();
        store.stop();
    }

    void seed(int count = 20) {
        for (int index = 0; index < count; ++index) send("target", index);
    }

    void send(const std::string& id, int index) {
        events.publish(plane::communication::events::kGcsTargetReceived, target(id, index));
    }

    void begin(const std::string& id) {
        error.clear();
        require(controller.begin(id, error), "超时场景会话启动失败: " + error);
    }

    void running() {
        error.clear();
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Running,
                "目标尚未超时，应保持运行: " + error);
    }

    void timed_out(const std::string& reason, int threshold) {
        error.clear();
        const int before = flight.velocity_calls + flight.attitude_calls;
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Failure &&
                    error.find("中制导目标雷达信息超时:") == 0 && error.find(reason) != std::string::npos &&
                    error.find("target_timeout_ms=" + std::to_string(threshold)) != std::string::npos,
                "应返回含原因及生效阈值的雷达超时: " + error);
        require(flight.velocity_calls + flight.attitude_calls == before, "超时 tick 不得下发控制");
    }

    plane::core::EventBus events;
    plane::core::AppConfig config;
    FakeFlightControl flight;
    FakeStateStore state;
    plane::target::TargetStore store;
    plane::task::MidcourseController controller;
    std::string error;
};

void test_target_timeout_boundaries_and_restart() {
    for (const std::string id : {"target", ""}) {
        TimeoutFixture fixture;
        fixture.begin(id);
        fixture.running();
        test_now_ms += 4999;
        fixture.running();
        ++test_now_ms;
        fixture.running();
        ++test_now_ms;
        fixture.timed_out("本次会话从未收到", 5000);
        require(fixture.error.find("未更新时长=5001毫秒") != std::string::npos &&
                    fixture.error.find("trackId=" + (id.empty() ? std::string("<自动>") : id)) != std::string::npos,
                "本次会话从未收到场景应保留目标和真实等待时长");
        fixture.controller.stop();
        fixture.begin(id);
        fixture.running();
        require(fixture.flight.velocity_calls == 0 && fixture.flight.attitude_calls == 0,
                "没有初始化历史不得启动 Offboard");
    }
}

void test_new_session_ignores_cached_history() {
    for (const std::string id : {"target", ""}) {
        TimeoutFixture fixture;
        fixture.seed();
        test_now_ms += 5001;
        fixture.begin(id);
        fixture.running();
        require(fixture.flight.velocity_calls == 0 && fixture.flight.attitude_calls == 0,
                "任务前缓存不得初始化新会话或下发控制");
        test_now_ms += 5000;
        fixture.running();
        ++test_now_ms;
        fixture.timed_out("本次会话从未收到", 5000);
        require(fixture.error.find("trackId=target") != std::string::npos,
                "自动和指定模式均应独立监测本次会话目标");
        fixture.controller.stop();

        fixture.begin(id);
        for (int index = 20; index < 39; ++index) fixture.send("target", index);
        fixture.running();
        require(fixture.flight.velocity_calls == 0 && fixture.flight.attitude_calls == 0,
                "本次会话不足 20 帧时不得复用任务前缓存初始化");
        fixture.send("target", 39);
        fixture.running();
        require(fixture.flight.velocity_calls == 1,
                "新会话收到 20 帧有效更新后应使用本次会话历史初始化");
    }
}

void test_history_wait_uses_last_reception() {
    TimeoutFixture fixture;
    fixture.seed(3);
    fixture.begin("target");
    fixture.running();
    test_now_ms += 4000;
    fixture.send("target", 3);
    test_now_ms += 5000;
    fixture.running();
    ++test_now_ms;
    fixture.timed_out("目标更新中断", 5000);
    require(fixture.flight.velocity_calls == 0, "不足 20 帧时不得下发控制");
}

void test_configured_history_size() {
    TimeoutFixture fixture(30);
    fixture.begin("target");
    for (int index = 0; index < 29; ++index) fixture.send("target", index);
    fixture.running();
    require(fixture.flight.velocity_calls == 0 && fixture.flight.attitude_calls == 0,
            "target_history_size=30 时收到 29 帧不得初始化或下发控制");
    fixture.send("target", 29);
    fixture.running();
    require(fixture.flight.velocity_calls == 1, "target_history_size=30 时第 30 帧应完成初始化并下发控制");
    fixture.send("target", 30);
    fixture.running();
    require(fixture.flight.velocity_calls == 2, "target_history_size=30 时第 31 帧应完成首次增量更新");
}

void test_only_accepted_current_target_refreshes_timeout() {
    TimeoutFixture fixture;
    fixture.config.midcourse_target_timeout_ms = 700;
    fixture.seed();
    fixture.begin("target");
    for (int index = 20; index < 40; ++index) fixture.send("target", index);
    fixture.running();
    test_now_ms += 599;
    fixture.send("target", 40);
    fixture.running();
    const auto last_received = test_now_ms;
    test_now_ms += 600;
    fixture.running();
    fixture.send("other", 0);
    fixture.send("target", 40);
    fixture.send("target", 39);
    fixture.send("", 21);
    auto invalid = target("target", 41);
    invalid.timestamp_ms = 0;
    fixture.events.publish(plane::communication::events::kGcsTargetReceived, invalid);
    require(fixture.store.gcs_target_last_received_ms("target") == last_received,
            "其他航迹、重复、乱序和非法帧不得刷新目标接收时间");
    test_now_ms += 100;
    fixture.running();
    ++test_now_ms;
    fixture.timed_out("目标更新中断", 700);
}

void test_independent_initialization_timeout_and_invalid_config() {
    TimeoutFixture fixture;
    fixture.config.midcourse_init_timeout_ms = 200;
    fixture.config.midcourse_target_history_size = 30;
    fixture.config.midcourse_target_timeout_ms = 100;
    fixture.seed(1);
    fixture.begin("target");
    fixture.running();
    for (int index = 1; index <= 3; ++index) {
        test_now_ms += 60;
        fixture.send("target", index);
        fixture.running();
    }
    test_now_ms += 21;
    require(fixture.controller.tick(fixture.error) == plane::task::MidcourseController::TickResult::Failure &&
                fixture.error == "等待中制导 30 帧雷达历史超时，当前 4/30",
            "持续有更新但历史不足仍须遵守独立初始化超时: " + fixture.error);
    fixture.controller.stop();
    for (const int invalid : {0, -1}) {
        fixture.config.midcourse_target_timeout_ms = invalid;
        require(!fixture.controller.begin("target", fixture.error) &&
                    fixture.error.find("target_timeout_ms 必须为正整数") != std::string::npos,
                "直接构造非法 AppConfig 时不得启动会话");
    }
    fixture.config.midcourse_target_timeout_ms = 100;
    for (const int invalid : {4, 31}) {
        fixture.config.midcourse_target_history_size = invalid;
        require(!fixture.controller.begin("target", fixture.error) &&
                    fixture.error.find("target_history_size 必须为 5 到 30 的整数") != std::string::npos,
                "直接构造越界 target_history_size 时不得启动会话");
    }
}

}  // namespace

namespace plane::core {
std::int64_t now_ms() { return test_now_ms; }
}  // namespace plane::core

int main() {
    try {
        test_target_timeout_boundaries_and_restart();
        test_new_session_ignores_cached_history();
        test_history_wait_uses_last_reception();
        test_configured_history_size();
        test_only_accepted_current_target_refreshes_timeout();
        test_independent_initialization_timeout_and_invalid_config();
        test_now_ms = 1'000'000;
        plane::core::EventBus events;
        plane::core::AppConfig config;
        config.bt_tick_interval_ms = 50;
        config.midcourse_init_timeout_ms = 30000;
        FakeFlightControl flight;
        flight.cached_flight_params_available = false;
        FakeStateStore state;
        state.value.geoPos.latitude = 30.0;
        state.value.geoPos.longitude = 120.0;
        state.value.geoPos.altitude = 100.0F;
        state.value.startPos = state.value.geoPos;
        state.value.startPos.altitude = 0.0F;
        state.value.yaw_deg = 42.0F;
        plane::target::TargetStore store(events, config);
        require(store.start(), "目标存储启动失败");
        plane::task::MidcourseController controller(events, config, flight, state, store);
        controller.start();

        std::string error;
        config.midcourse_algorithm.r_capture_max_m = config.midcourse_algorithm.r_capture_min_m;
        require(controller.begin("", error), "自动选目标非法配置场景未启动");
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Failure &&
                    error == "中制导配置校验失败: r_capture 窗口非法",
                "自动模式无目标时应先校验配置而不是等待目标: " + error);
        controller.stop();
        config.midcourse_algorithm.r_capture_max_m = 500.0;

        for (int index = 0; index < 20; ++index) {
            events.publish(plane::communication::events::kGcsTargetReceived, target("target", index));
        }
        require(controller.begin("target", error), "中制导会话启动失败");
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Running,
                "等待本次会话 20 帧失败: " + error);
        require(flight.velocity_calls == 0 && flight.attitude_calls == 0,
                "会话启动前缓存不得初始化算法或下发控制");
        for (int index = 20; index < 40; ++index) {
            events.publish(plane::communication::events::kGcsTargetReceived, target("target", index));
        }
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Running,
                "使用本次会话 20 帧初始化失败: " + error);
        require(flight.cached_flight_params_gets == 0 && flight.velocity_calls == 1,
                "INI 限值模式不应读取 PX4 缓存，且应下发首个速度及加速度指令");
        constexpr double kPi = 3.14159265358979323846;
        const double actual_yaw_rad = 42.0 * kPi / 180.0;
        const double max_first_step_rad = config.midcourse_algorithm.yaw_rate_max_deg_s * kPi / 180.0 *
                                          config.midcourse_algorithm.update_dt_s;
        require(flight.last_velocity && flight.last_velocity->yaw_rad &&
                    std::isfinite(*flight.last_velocity->yaw_rad) &&
                    std::abs(std::remainder(*flight.last_velocity->yaw_rad - actual_yaw_rad, 2.0 * kPi)) <=
                        max_first_step_rad + 1e-9,
                "中制导航向首拍未从实际 yaw_rad 按配置速率限幅");

        events.publish(plane::communication::events::kGcsTargetReceived, target("target", 40));
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Running,
                "第 21 帧更新失败: " + error);
        require(flight.velocity_calls == 2, "第 21 帧后未继续下发 NED 速度");

        for (int tick = 0; tick < 65; ++tick) {
            test_now_ms += 50;
            require(controller.tick(error) == plane::task::MidcourseController::TickResult::Running,
                    "雷达中断超过 3 秒后任务不应失败: " + error);
        }
        require(flight.velocity_calls == 67, "雷达中断期间未逐 tick 执行规划和 NED 速度");

        controller.stop();
        config.midcourse_use_px4_limits = true;
        config.midcourse_algorithm.control_output_mode = plane::algorithm::MidcourseControlMode::Attitude;
        flight.cached_flight_params_available = true;
        require(controller.begin("target", error), "重复会话启动失败");
        for (int index = 41; index < 61; ++index) {
            events.publish(plane::communication::events::kGcsTargetReceived, target("target", index));
        }
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Running,
                "重复会话未从本次新收 20 帧重新初始化: " + error);
        require(flight.cached_flight_params_gets == 1, "PX4 限值模式未获取飞控参数缓存");
        require(flight.attitude_calls == 1 && flight.velocity_calls == 67, "姿态档未下发姿态指令");
        double norm_squared = 0.0;
        for (double value : flight.last_attitude.quat_body_to_ned) norm_squared += value * value;
        require(std::abs(norm_squared - 1.0) < 1e-6 && std::isfinite(flight.last_attitude.thrust_body[2]),
                "姿态档输出的四元数或推力无效");
        controller.stop();

        flight.cached_flight_params_available = false;
        require(controller.begin("target", error), "参数失败场景未启动");
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Failure,
                "PX4 参数缓存未就绪应终止中制导");
        controller.stop();

        config.midcourse_use_px4_limits = false;
        config.midcourse_algorithm.r_capture_max_m = config.midcourse_algorithm.r_capture_min_m;
        require(controller.begin("无历史航迹", error), "非法捕获窗口场景未启动");
        error.clear();
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Failure &&
                    error == "中制导配置校验失败: r_capture 窗口非法",
                "非法捕获窗口应在等待雷达历史前终止中制导: " + error);
        require(flight.velocity_calls == 67 && flight.attitude_calls == 1,
                "中制导配置校验失败后仍下发了 Offboard 指令");
        controller.stop();

        config.midcourse_algorithm.r_capture_max_m = 500.0;
        require(controller.begin("bad-init-target", error), "非法初始化量测场景未启动");
        for (int index = 0; index < 20; ++index) {
            auto init_measurement = target("bad-init-target", index);
            if (index == 19) init_measurement.geoPos.latitude = std::numeric_limits<double>::quiet_NaN();
            events.publish(plane::communication::events::kGcsTargetReceived, init_measurement);
        }
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Failure &&
                    error.rfind("中制导 init 失败: ", 0) == 0,
                "MidcourseAlgorithm::init 拒绝非法历史时应终止中制导: " + error);
        require(flight.velocity_calls == 67 && flight.attitude_calls == 1,
                "算法初始化失败后仍下发了 Offboard 指令");
        controller.stop();

        state.value.yaw_deg = std::numeric_limits<float>::quiet_NaN();
        require(controller.begin("target", error), "非法航向场景未启动");
        for (int index = 61; index < 81; ++index) {
            events.publish(plane::communication::events::kGcsTargetReceived, target("target", index));
        }
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Failure &&
                    error == "中制导本机实际航向 yaw_deg 必须为有限数",
                "非有限实际航向应终止中制导: " + error);
        require(flight.velocity_calls == 67 && flight.attitude_calls == 1,
                "航向校验失败后仍下发了 Offboard 指令");
        controller.stop();
        state.value.yaw_deg = 42.0F;
        config.midcourse_algorithm.control_output_mode = plane::algorithm::MidcourseControlMode::VelocityAcceleration;

        require(controller.begin("bad-target", error), "非法增量量测场景未启动");
        for (int index = 0; index < 20; ++index) {
            events.publish(plane::communication::events::kGcsTargetReceived, target("bad-target", index));
        }
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Running,
                "非法增量量测场景初始化失败: " + error);
        auto bad_measurement = target("bad-target", 20);
        bad_measurement.geoPos.latitude = std::numeric_limits<double>::quiet_NaN();
        events.publish(plane::communication::events::kGcsTargetReceived, bad_measurement);
        const int velocity_calls_before_bad_measurement = flight.velocity_calls;
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Failure &&
                    error.rfind("中制导 updateTarget 失败: ", 0) == 0,
                "updateTarget 拒绝非法量测时应终止中制导: " + error);
        require(flight.velocity_calls == velocity_calls_before_bad_measurement + 1,
                "新量测应在本 tick 控制下发后处理并从下一 tick 生效");
        controller.stop();
    } catch (const std::exception& exception) {
        SPDLOG_ERROR("中制导控制器测试失败: {}", exception.what());
        return 1;
    }
    return 0;
}
