#pragma once

#include <string>
#include <vector>

#include "message.h"

namespace plane::flight_control {

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
    virtual void clear_ned_velocity() = 0;

    virtual std::string link_state() const = 0;
};

}  // namespace plane::flight_control
