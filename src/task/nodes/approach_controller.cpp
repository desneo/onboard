#include "task/nodes/approach_controller.h"

#include <algorithm>
#include <cmath>

#include "geo/geo_tool.h"

namespace plane::task {
namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

ApproachController::ApproachController(plane::flight_control::FlightControlApi* flight_control,
                                       plane::state::StateStoreApi* state_store,
                                       plane::target::TargetStoreApi* target_store,
                                       const plane::core::AppConfig* app_config)
    : flight_control_(flight_control),
      state_store_(state_store),
      target_store_(target_store),
      app_config_(app_config) {}

bool ApproachController::resolve_target(const std::string& track_id, plane::model::GcsTarget& out) const {
    if (!track_id.empty()) {
        return target_store_->gcs_target(track_id, out);
    }
    const auto targets = target_store_->gcs_targets();
    if (targets.size() != 1) {
        return false;
    }
    out = targets.front();
    return true;
}

double ApproachController::horizontal_distance_m(const plane::model::GcsTarget& target) const {
    const auto own = state_store_->own_state();
    return plane::geo::GeoCalculator::distance_m(own.geoPos, target.geoPos);
}

bool ApproachController::approach(const plane::model::GcsTarget& target, double speed_m_s, std::string& error) {
    const auto own = state_store_->own_state();
    const double bearing_deg = plane::geo::GeoCalculator::bearing_deg(own.geoPos, target.geoPos);
    const double bearing_rad = bearing_deg * kPi / 180.0;

    plane::model::NedVelocity velocity;
    velocity.north_m_s = speed_m_s * std::cos(bearing_rad);
    velocity.east_m_s = speed_m_s * std::sin(bearing_rad);
    if (target.geoPos.height > 0.0F) {
        const double diff = static_cast<double>(target.geoPos.height) - own.geoPos.height;
        if (std::fabs(diff) <= 1.0) {
            velocity.down_m_s = 0.0;
        } else {
            velocity.down_m_s =
                std::max(-app_config_->suicide_climb_speed_m_s, std::min(app_config_->suicide_climb_speed_m_s, -diff));
        }
    }
    velocity.yaw_deg = bearing_deg;
    velocity.yaw_rate_deg_s = 0.0;
    return flight_control_->set_ned_velocity(velocity, error);
}

}  // namespace plane::task
