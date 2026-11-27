#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "algorithm/midcourse_alg.h"
#include "message.h"

namespace plane::flight_control {

/// 飞控运行状态快照；模块内部使用 MAVSDK 枚举，对外只暴露基础字段。
struct FlightStatus {
    bool armed = false;
    bool in_air = false;
    int mission_current = 0;
    int mission_total = 0;
    float takeoff_altitude_m = 0.0F;  ///< 最近一次 Takeoff 指令高度
    bool connected = false;
    bool on_ground = false;
    bool landed_in_air = false;  ///< 飞控 landed_state 处于 InAir（区分 TakingOff 过渡态）
    std::chrono::steady_clock::time_point armed_updated{};
    std::chrono::steady_clock::time_point ground_updated{};
    std::chrono::steady_clock::time_point mode_updated{};
    std::string flight_mode;
    std::uint64_t link_generation = 0;
    bool mission_progress_received = false;  ///< 已收到航点进度回调
};

inline constexpr auto kSelfCheckTelemetryMaxAge = std::chrono::seconds(3);

inline bool self_check_telemetry_fresh(std::chrono::steady_clock::time_point updated,
                                       std::chrono::steady_clock::time_point now) {
    return updated != std::chrono::steady_clock::time_point{} && updated <= now &&
           now - updated <= kSelfCheckTelemetryMaxAge;
}

inline std::string self_check_ground_error(const FlightStatus& status, std::chrono::steady_clock::time_point now,
                                          bool require_fresh_mode = true) {
    if (!status.connected) return "飞控未连接";
    if (!self_check_telemetry_fresh(status.ground_updated, now) || !status.on_ground || status.in_air) {
        return "未确认飞机在地面";
    }
    if (require_fresh_mode && !self_check_telemetry_fresh(status.mode_updated, now)) return "飞行模式遥测未就绪";
    if (status.mode_updated != std::chrono::steady_clock::time_point{} &&
        status.flight_mode != "Posctl" && status.flight_mode != "Altctl" &&
        status.flight_mode != "Stabilized" && status.flight_mode != "Manual") {
        return "飞行模式不允许执行自检（flight_mode=" + status.flight_mode + "）";
    }
    return {};
}

enum class SelfCheckArmResult { Accepted, Rejected, Uncertain };

/// 飞控连接后缓存的 PX4 飞控系统参数；业务层不直接依赖 MAVSDK 参数类型。
struct FlightControlParams {
    double mpc_xy_vel_max = 0.0;
    double mpc_z_vel_max_up = 0.0;
    double mpc_z_vel_max_dn = 0.0;
    double mpc_acc_hor_max = 0.0;
};

/// 中制导 Attitude 输出；四元数为机体到 NED，推力为机体系归一化值。
using AttitudeSetpoint = plane::algorithm::AttitudeCommand;

/// Flight Control 对外稳定能力接口（ARCHITECTURE §13）。
class FlightControlApi {
   public:
    virtual ~FlightControlApi() = default;

    virtual void begin_control(std::uint64_t) {}
    /// 接管已触发时须由调度器确认清理完成，才能解除旧执行的飞控发送限制。
    virtual bool end_control(std::uint64_t, bool = false) { return true; }
    virtual bool control_can_advance() const { return true; }

    virtual bool arm(std::string& error) = 0;
    virtual bool disarm(std::string& error) = 0;
    virtual SelfCheckArmResult self_check_arm(const FlightStatus&, const std::atomic<bool>*, std::string& error) {
        error = "飞控不支持自检解锁";
        return SelfCheckArmResult::Rejected;
    }
    virtual bool self_check_disarm(const FlightStatus&, const std::atomic<bool>*, std::string& error) {
        error = "飞控不支持自检上锁";
        return false;
    }
    virtual bool takeoff(float height_m, std::string& error) = 0;
    virtual bool land(std::string& error) = 0;
    virtual bool return_to_launch(std::string& error) = 0;
    virtual bool hold(std::string& error) = 0;
    virtual bool stop_motor(std::string& error) = 0;
    virtual bool goto_waypoint(const plane::model::GeoPoint& point, std::string& error) = 0;
    virtual bool execute_route(const std::vector<plane::model::GeoPoint>& points, std::string& error) = 0;
    virtual bool set_ned_velocity(const plane::model::NedVelocity& velocity, std::string& error) = 0;
    virtual bool set_ned_velocity_acceleration(const plane::algorithm::VelocityAccelerationCommand& command,
                                               double current_yaw_deg, std::string& error) = 0;
    virtual bool set_attitude(const AttitudeSetpoint& setpoint, std::string& error) = 0;
    /// 返回飞控连接建立后预先读取的 PX4 飞控系统参数；本调用不访问 PX4。
    virtual bool get_cached_flight_control_params(FlightControlParams& params, std::string& error) const = 0;
    virtual void clear_offboard() = 0;
    virtual void clear_ned_velocity() { clear_offboard(); }

    virtual std::string link_state() const = 0;
    virtual plane::flight_control::FlightStatus flight_status() const = 0;
};

}  // namespace plane::flight_control
