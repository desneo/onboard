#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>
#include <string>

#include "communication/serialization.h"

int main() {
    plane::model::VehicleState source;
    assert(!source.inair);
    source.planeId = "uav1";
    source.inair = true;
    source.geoPos.latitude = std::numeric_limits<double>::quiet_NaN();
    source.geoPos.longitude = std::numeric_limits<double>::infinity();
    source.geoPos.height = std::numeric_limits<float>::quiet_NaN();
    source.roll_deg = std::numeric_limits<float>::quiet_NaN();
    source.airspeed_m_s = std::numeric_limits<float>::infinity();

    std::string payload;
    std::string error;
    assert(plane::communication::to_payload(source, payload, error));
    assert(payload.find("null") == std::string::npos);
    assert(payload.find("\"inair\":true") != std::string::npos);

    plane::model::VehicleState decoded;
    assert(plane::communication::from_payload(payload, decoded, error));
    assert(decoded.planeId == "uav1");
    assert(decoded.inair);
    assert(decoded.geoPos.latitude == 0.0);
    assert(decoded.geoPos.longitude == 0.0);
    assert(decoded.geoPos.height == 0.0F);
    assert(decoded.roll_deg == 0.0F);
    assert(decoded.airspeed_m_s == 0.0F);

    source.inair = false;
    assert(plane::communication::to_payload(source, payload, error));
    assert(payload.find("\"inair\":false") != std::string::npos);
    assert(plane::communication::from_payload(payload, decoded, error));
    assert(!decoded.inair);

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
    decoded.inair = true;
    assert(plane::communication::from_payload(legacy_null_payload, decoded, error));
    assert(decoded.planeId == "uav2");
    assert(!decoded.inair);
    assert(decoded.geoPos.latitude == 0.0);
    assert(decoded.roll_deg == 0.0F);
    assert(decoded.voltage_v == 0.0F);

    constexpr const char* invalid_inair_values[] = {"1", "\"true\"", "null", "[]", "{}"};
    for (const char* invalid_inair : invalid_inair_values) {
        const std::string invalid_payload =
            std::string(R"({"planeId":"uav3","geoPos":{},"inair":)") + invalid_inair + "}";
        error.clear();
        assert(!plane::communication::from_payload(invalid_payload, decoded, error));
        assert(!error.empty());
    }

    const std::string target_payload = R"({
        "trackId":"target-7","geoPos":{"latitude":30.1,"longitude":120.2,"altitude":321},
        "timestamp_ms":1800000000123
    })";
    plane::model::GcsTarget decoded_target;
    assert(plane::communication::from_payload(target_payload, decoded_target, error));
    assert(decoded_target.trackId == "target-7");
    assert(decoded_target.timestamp_ms == 1'800'000'000'123LL);
    assert(decoded_target.geoPos.latitude == 30.1);
    assert(decoded_target.geoPos.longitude == 120.2);
    assert(decoded_target.geoPos.altitude == 321.0F);

    const std::string extended_target = R"({
        "trackId":"extended","geoPos":{"latitude":30,"longitude":120,"altitude":100},
        "timestamp_ms":1800000000124,"radial_speed_m_s":-12.5,"azimuth_deg":47
    })";
    assert(plane::communication::from_payload(extended_target, decoded_target, error));
    assert(decoded_target.trackId == "extended");
    assert(decoded_target.timestamp_ms == 1'800'000'000'124LL);

    const std::string no_timestamp = R"({"trackId":"old","geoPos":{"latitude":30,"longitude":120,"altitude":100}})";
    assert(!plane::communication::from_payload(no_timestamp, decoded_target, error));

    return 0;
}
