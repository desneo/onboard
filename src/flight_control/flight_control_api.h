#pragma once

#include <array>
#include <string>
#include <vector>

#include "message.h"

namespace plane::flight_control {

/// 飞控运行状态快照；模块内部使用 MAVSDK 枚举，对外只暴露基础字段。
struct FlightStatus {
    bool armed = false;
    bool in_air = false;
    int mission_current = 0;
    int mission_total = 0;
    float takeoff_altitude_m = 0.0F;  ///< 最近一次 Takeoff 指令高度
};

/// 飞控连接后缓存的 PX4 飞控系统参数；业务层不直接依赖 MAVSDK 参数类型。
struct FlightControlParams {
    double mpc_xy_vel_max = 0.0;
    double mpc_z_vel_max_up = 0.0;
    double mpc_z_vel_max_dn = 0.0;
    double mpc_acc_hor_max = 0.0;
};

/// 中制导 Attitude 输出；四元数为机体到 NED，推力为机体系归一化值。
struct AttitudeSetpoint {
    std::array<double, 4> quat_body_to_ned = {1.0, 0.0, 0.0, 0.0};
    std::array<double, 3> thrust_body = {0.0, 0.0, 0.0};
};

/// Flight Control 对外稳定能力接口（ARCHITECTURE §13）。
class FlightControlApi {
   public:
    virtual ~FlightControlApi() = default;

    virtual bool arm(std::string& error) = 0;
    virtual bool disarm(std::string& error) = 0;
    virtual bool takeoff(float height_m, std::string& error) = 0;
    virtual bool land(std::string& error) = 0;
    virtual bool return_to_launch(std::string& error) = 0;
    virtual bool hold(std::string& error) = 0;
    virtual bool stop_motor(std::string& error) = 0;
    virtual bool goto_waypoint(const plane::model::GeoPoint& point, std::string& error) = 0;
    virtual bool execute_route(const std::vector<plane::model::GeoPoint>& points, std::string& error) = 0;
    virtual bool set_ned_velocity(const plane::model::NedVelocity& velocity, std::string& error) = 0;
    virtual bool set_attitude(const AttitudeSetpoint& setpoint, std::string& error) = 0;
    /// 返回飞控连接建立后预先读取的 PX4 飞控系统参数；本调用不访问 PX4。
    virtual bool get_cached_flight_control_params(FlightControlParams& params, std::string& error) const = 0;
    virtual void clear_offboard() = 0;
    virtual void clear_ned_velocity() { clear_offboard(); }

    virtual std::string link_state() const = 0;
    virtual plane::flight_control::FlightStatus flight_status() const = 0;
};

}  // namespace plane::flight_control
