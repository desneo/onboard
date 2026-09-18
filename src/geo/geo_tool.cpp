#include "geo/geo_tool.h"

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/Geodesic.hpp>

namespace plane::geo {
namespace {

const GeographicLib::Geodesic& geodesic() {
    static const GeographicLib::Geodesic instance(GeographicLib::Constants::WGS84_a(),
                                                  GeographicLib::Constants::WGS84_f());
    return instance;
}

}  // namespace

double GeoCalculator::distance_m(const plane::model::GeoPoint& from, const plane::model::GeoPoint& to) {
    double distance = 0.0;
    double azimuth = 0.0;
    double reverse_azimuth = 0.0;
    geodesic().Inverse(from.latitude, from.longitude, to.latitude, to.longitude, distance, azimuth, reverse_azimuth);
    return distance;
}

double GeoCalculator::bearing_deg(const plane::model::GeoPoint& from, const plane::model::GeoPoint& to) {
    double distance = 0.0;
    double azimuth = 0.0;
    double reverse_azimuth = 0.0;
    geodesic().Inverse(from.latitude, from.longitude, to.latitude, to.longitude, distance, azimuth, reverse_azimuth);
    double bearing = azimuth;
    if (bearing < 0.0) {
        bearing += 360.0;
    }
    return bearing;
}

}  // namespace plane::geo
