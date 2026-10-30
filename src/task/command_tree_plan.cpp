#include "task/command_tree_plan.h"

namespace plane::task {

std::vector<std::string> command_node_names(plane::model::CommandType type,
                                            const plane::flight_control::FlightStatus& flight_status) {
    switch (type) {
        case plane::model::CommandType::Takeoff:
            return {"Arm", "Takeoff"};
        case plane::model::CommandType::Arm:
            return {"Arm"};
        case plane::model::CommandType::Disarm:
            return {"Disarm"};
        case plane::model::CommandType::Land:
            return {"Land"};
        case plane::model::CommandType::RTL:
            return {"RTL"};
        case plane::model::CommandType::Hover:
            return {"Hover"};
        case plane::model::CommandType::StopMotor:
            return {"StopMotor"};
        case plane::model::CommandType::Waypoint:
            return {"Mission"};
        case plane::model::CommandType::Route: {
            std::vector<std::string> nodes;
            if (!flight_status.in_air) {
                if (!flight_status.armed) {
                    nodes.emplace_back("Arm");
                }
                nodes.emplace_back("Takeoff");
            }
            nodes.emplace_back("AdjustHeight");
            nodes.emplace_back("Mission");
            return nodes;
        }
    }
    return {};
}

std::vector<std::string> anti_uav_node_names(const plane::flight_control::FlightStatus& flight_status) {
    std::vector<std::string> nodes;
    if (!flight_status.in_air) {
        if (!flight_status.armed) nodes.emplace_back("Arm");
        nodes.emplace_back("Takeoff");
    }
    nodes.emplace_back("Midcourse");
    return nodes;
}

}  // namespace plane::task
