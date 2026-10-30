#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "message.h"
#include "target/gcs_target.h"

namespace plane::gcs_sim {

struct RadarFilterConfig {
    double friendly_match_max_distance_m = 50.0;
    std::int64_t friendly_state_timeout_ms = 3000;
};

/// 雷达扫描解析与友机过滤；成功时 out 只包含未匹配到友机的 confirmed 航迹。
class RadarTargetFilter {
   public:
    explicit RadarTargetFilter(RadarFilterConfig config = {});

    bool update_friendly(const std::string& sender_id, const plane::model::VehicleState& state,
                         std::int64_t received_unix_ms, std::string& error);
    bool process_scan(const std::string& payload, std::int64_t now_unix_ms,
                      std::vector<plane::target::GcsTarget>& out, std::string& error) const;

   private:
    struct FriendlySample {
        plane::model::VehicleState state;
        std::int64_t received_unix_ms = 0;
    };

    RadarFilterConfig config_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, FriendlySample> friendlies_;
};

}  // namespace plane::gcs_sim
