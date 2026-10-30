#ifdef NDEBUG
#undef NDEBUG
#endif
#include "tools/gcs_sim/radar_target_filter.h"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace {

constexpr double kMetersPerLatitudeDegree = 111319.49079327357;

plane::model::VehicleState friendly(const std::string& id, double north_m, double vn_m_s = 0.0) {
    plane::model::VehicleState state;
    state.planeId = id;
    state.geoPos.latitude = 30.0 + north_m / kMetersPerLatitudeDegree;
    state.geoPos.longitude = 120.0;
    state.geoPos.altitude = 100.0F;
    state.vn_m_s = static_cast<float>(vn_m_s);
    return state;
}

nlohmann::json track(const std::string& id, double north_m, const std::string& state = "confirmed") {
    return {{"track_id", id},
            {"state", state},
            {"latitude", 30.0 + north_m / kMetersPerLatitudeDegree},
            {"longitude", 120.0},
            {"altitude_msl_m", 100.0},
            {"radial_speed_m_s", 12.0},
            {"azimuth_deg", 90.0},
            {"unknown_field", "ignored"}};
}

std::string scan(std::int64_t timestamp_ms, std::vector<nlohmann::json> tracks) {
    return nlohmann::json{{"type", "radar_tracks"},
                          {"schema_version", 999},
                          {"timestamp_ms", timestamp_ms},
                          {"unknown_top_level", true},
                          {"tracks", std::move(tracks)}}
        .dump();
}

bool contains_id(const std::vector<plane::target::GcsTarget>& targets, const std::string& id) {
    return std::any_of(targets.begin(), targets.end(), [&](const auto& target) { return target.trackId == id; });
}

}  // namespace

int main() {
    using plane::gcs_sim::RadarFilterConfig;
    using plane::gcs_sim::RadarTargetFilter;

    constexpr std::int64_t kTime = 1'800'000'000'000LL;
    std::vector<plane::target::GcsTarget> output;
    std::string error;

    RadarTargetFilter no_state;
    assert(!no_state.process_scan(scan(kTime, {track("unknown", 100.0)}), kTime, output, error));

    // 贪心匹配会选 F1->+4 m，导致 F2->-5 m 超出 10 m；全局最优为 F1->-5、F2->+4。
    RadarTargetFilter global(RadarFilterConfig{10.0, 3000});
    assert(global.update_friendly("uav1", friendly("uav1", 0.0), kTime, error));
    assert(global.update_friendly("uav2", friendly("uav2", 8.0), kTime, error));
    const std::string global_scan = scan(kTime, {track("self-a", 4.0), track("self-b", -5.0), track("target-a", 100.0),
                                                 track("target-b", 200.0), track("tentative", 300.0, "tentative")});
    assert(global.process_scan(global_scan, kTime, output, error));
    assert(output.size() == 2);
    assert(contains_id(output, "target-a"));
    assert(contains_id(output, "target-b"));
    assert(!contains_id(output, "self-a"));
    assert(!contains_id(output, "self-b"));
    assert(output.front().timestamp_ms == kTime);

    const std::string double_encoded = nlohmann::json(global_scan).dump();
    assert(global.process_scan(double_encoded, kTime, output, error));
    assert(output.size() == 2);

    RadarTargetFilter projected(RadarFilterConfig{2.0, 3000});
    assert(projected.update_friendly("uav1", friendly("uav1", 0.0, 10.0), kTime, error));
    assert(projected.process_scan(scan(kTime + 1000, {track("projected-self", 10.0), track("target", 100.0)}),
                                  kTime + 1000, output, error));
    assert(output.size() == 1 && output.front().trackId == "target");

    RadarTargetFilter stale;
    assert(stale.update_friendly("uav1", friendly("uav1", 0.0), kTime, error));
    assert(!stale.process_scan(scan(kTime + 4000, {track("self", 0.0)}), kTime + 4000, output, error));

    RadarTargetFilter unmatched(RadarFilterConfig{5.0, 3000});
    assert(unmatched.update_friendly("uav1", friendly("uav1", 0.0), kTime, error));
    assert(unmatched.process_scan(scan(kTime, {track("target-only", 100.0)}), kTime, output, error));
    assert(output.size() == 1 && output.front().trackId == "target-only");

    // 7790 只传输雷达航迹；雷达帧不需要包含 Zenoh 中的每一架友机。
    RadarTargetFilter fewer_tracks(RadarFilterConfig{5.0, 3000});
    assert(fewer_tracks.update_friendly("uav1", friendly("uav1", 0.0), kTime, error));
    assert(fewer_tracks.update_friendly("uav2", friendly("uav2", 50.0), kTime, error));
    assert(fewer_tracks.process_scan(scan(kTime, {track("target", 100.0)}), kTime, output, error));
    assert(output.size() == 1 && output.front().trackId == "target");

    RadarTargetFilter malformed;
    assert(!malformed.process_scan("{bad json", kTime, output, error));
    return 0;
}
