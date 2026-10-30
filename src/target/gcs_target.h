#pragma once

#include <cstdint>
#include <string>

#include "geo_types.h"

namespace plane::target {

/// 雷达/GCS 靶机目标 v2；仅在 onboard 与配套 GCS 工具内使用。
struct GcsTarget {
    std::string trackId;
    plane::model::GeoPoint geoPos;
    std::int64_t timestamp_ms = 0;
    float radial_speed_m_s = 0.0F;
    float azimuth_deg = 0.0F;
};

}  // namespace plane::target
