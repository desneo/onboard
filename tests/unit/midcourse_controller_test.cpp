#include "task/nodes/midcourse_controller.h"

#include <spdlog/spdlog.h>

#include <cstdint>
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
    int clear_calls = 0;
    plane::flight_control::AttitudeSetpoint last_attitude;

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

plane::target::GcsTarget target(const std::string& id, int index) {
    plane::target::GcsTarget result;
    result.trackId = id;
    result.timestamp_ms = 1'800'000'000'000LL + static_cast<std::int64_t>(index) * 1000;
    result.geoPos.latitude = 30.010 + 0.00001 * static_cast<double>(index);
    result.geoPos.longitude = 120.0;
    result.geoPos.altitude = 300.0F;
    return result;
}

}  // namespace

namespace plane::core {
std::int64_t now_ms() { return test_now_ms; }
}  // namespace plane::core

int main() {
    try {
        plane::core::EventBus events;
        plane::core::AppConfig config;
        config.bt_tick_interval_ms = 50;
        config.midcourse_init_timeout_ms = 30000;
        config.midcourse_target_invalid_grace_s = 3.0;
        config.midcourse_r_min_m = 50.0;
        FakeFlightControl flight;
        FakeStateStore state;
        state.value.geoPos.latitude = 30.0;
        state.value.geoPos.longitude = 120.0;
        state.value.geoPos.altitude = 100.0F;
        state.value.startPos = state.value.geoPos;
        plane::target::TargetStore store(events, config);
        require(store.start(), "目标存储启动失败");
        plane::task::MidcourseController controller(events, config, flight, state, store);
        controller.start();

        std::string error;
        for (int index = 0; index < 20; ++index) {
            events.publish(plane::communication::events::kGcsTargetReceived, target("target", index));
        }
        require(controller.begin("target", error), "中制导会话启动失败");
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Running,
                "使用会话启动前持续缓存的 20 帧初始化失败: " + error);
        require(flight.cached_flight_params_gets == 1 && flight.attitude_calls == 1,
                "未使用飞控参数缓存或未下发首个 Attitude");

        events.publish(plane::communication::events::kGcsTargetReceived, target("target", 20));
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Running,
                "第 21 帧更新失败: " + error);
        require(flight.attitude_calls == 2, "第 21 帧后未继续下发 Attitude");

        for (int tick = 0; tick < 65; ++tick) {
            require(controller.tick(error) == plane::task::MidcourseController::TickResult::Running,
                    "雷达中断超过 3 秒后任务不应失败: " + error);
        }
        require(flight.attitude_calls == 67, "雷达中断期间未逐 tick 执行规划和 Attitude");

        controller.stop();
        require(controller.begin("target", error), "重复会话启动失败");
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Running,
                "重复会话未从已有 20 帧重新初始化: " + error);
        require(flight.cached_flight_params_gets == 2, "重复会话未获取飞控参数缓存");
        controller.stop();

        flight.cached_flight_params_available = false;
        require(controller.begin("target", error), "参数失败场景未启动");
        require(controller.tick(error) == plane::task::MidcourseController::TickResult::Failure,
                "PX4 参数缓存未就绪应终止中制导");
        controller.stop();
    } catch (const std::exception& exception) {
        SPDLOG_ERROR("中制导控制器测试失败: {}", exception.what());
        return 1;
    }
    return 0;
}
