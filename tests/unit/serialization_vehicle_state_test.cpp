#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>
#include <string>

#include "communication/serialization.h"
#include "target/gcs_target.h"

int main() {
    plane::model::VehicleState source;
    source.planeId = "uav1";
    source.geoPos.latitude = std::numeric_limits<double>::quiet_NaN();
    source.geoPos.longitude = std::numeric_limits<double>::infinity();
    source.geoPos.height = std::numeric_limits<float>::quiet_NaN();
    source.roll_deg = std::numeric_limits<float>::quiet_NaN();
    source.airspeed_m_s = std::numeric_limits<float>::infinity();

    std::string payload;
    std::string error;
    assert(plane::communication::to_payload(source, payload, error));
    assert(payload.find("null") == std::string::npos);

    plane::model::VehicleState decoded;
    assert(plane::communication::from_payload(payload, decoded, error));
    assert(decoded.planeId == "uav1");
    assert(decoded.geoPos.latitude == 0.0);
    assert(decoded.geoPos.longitude == 0.0);
    assert(decoded.geoPos.height == 0.0F);
    assert(decoded.roll_deg == 0.0F);
    assert(decoded.airspeed_m_s == 0.0F);

    const std::string legacy_null_payload = R"({
        "planeId":"uav2",
        "geoPos":{"latitude":null,"longitude":null,"height":null,"altitude":null},
        "startPos":{"latitude":null,"longitude":null,"height":null,"altitude":null},
        "roll_deg":null,"pitch_deg":null,"yaw_deg":null,
        "vn_m_s":null,"ve_m_s":null,"vd_m_s":null,
        "airspeed_m_s":null,"voltage_v":null,
        "cameraAngle_roll_deg":null,"cameraAngle_pitch_deg":null,
        "cameraAngle_yaw_deg":null
    })";
    assert(plane::communication::from_payload(legacy_null_payload, decoded, error));
    assert(decoded.planeId == "uav2");
    assert(decoded.geoPos.latitude == 0.0);
    assert(decoded.roll_deg == 0.0F);
    assert(decoded.voltage_v == 0.0F);

    plane::target::GcsTarget target;
    target.trackId = "target-7";
    target.geoPos.latitude = 30.1;
    target.geoPos.longitude = 120.2;
    target.geoPos.altitude = 321.0F;
    target.timestamp_ms = 1'800'000'000'123LL;
    target.radial_speed_m_s = -12.5F;
    target.azimuth_deg = 47.0F;
    assert(plane::communication::to_payload(target, payload, error));
    assert(payload.find("timestamp_ms") != std::string::npos);
    assert(payload.find("velocity_m_s") == std::string::npos);
    plane::target::GcsTarget decoded_target;
    assert(plane::communication::from_payload(payload, decoded_target, error));
    assert(decoded_target.trackId == target.trackId);
    assert(decoded_target.timestamp_ms == target.timestamp_ms);
    assert(decoded_target.geoPos.altitude == target.geoPos.altitude);
    assert(decoded_target.radial_speed_m_s == target.radial_speed_m_s);
    assert(decoded_target.azimuth_deg == target.azimuth_deg);

    const std::string old_target = R"({"trackId":"old","geoPos":{"latitude":30,"longitude":120,"altitude":100}})";
    assert(!plane::communication::from_payload(old_target, decoded_target, error));

    return 0;
}
