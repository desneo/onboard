#include "tools/gcs_sim/radar_target_filter.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <nlohmann/json.hpp>
#include <utility>

namespace plane::gcs_sim {
namespace {

constexpr double kEarthRadiusM = 6378137.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kInvalidAssignmentCost = 1e9;

double deg_to_rad(double value) { return value * kPi / 180.0; }

bool valid_geo(const plane::model::GeoPoint& point) {
    return std::isfinite(point.latitude) && std::isfinite(point.longitude) && std::isfinite(point.altitude) &&
           point.latitude >= -90.0 && point.latitude <= 90.0 && point.longitude >= -180.0 && point.longitude <= 180.0;
}

plane::model::GeoPoint project_position(const plane::model::VehicleState& state, std::int64_t received_unix_ms,
                                        std::int64_t timestamp_ms) {
    const double dt_s = static_cast<double>(timestamp_ms - received_unix_ms) / 1000.0;
    plane::model::GeoPoint projected = state.geoPos;
    projected.latitude += state.vn_m_s * dt_s / kEarthRadiusM * 180.0 / kPi;
    const double cos_lat = std::max(1e-6, std::fabs(std::cos(deg_to_rad(state.geoPos.latitude))));
    projected.longitude += state.ve_m_s * dt_s / (kEarthRadiusM * cos_lat) * 180.0 / kPi;
    projected.altitude -= state.vd_m_s * dt_s;
    return projected;
}

double distance_3d_m(const plane::model::GeoPoint& first, const plane::model::GeoPoint& second) {
    const double lat1 = deg_to_rad(first.latitude);
    const double lat2 = deg_to_rad(second.latitude);
    const double delta_lat = lat2 - lat1;
    const double delta_lon = deg_to_rad(second.longitude - first.longitude);
    const double sin_lat = std::sin(delta_lat / 2.0);
    const double sin_lon = std::sin(delta_lon / 2.0);
    const double a = sin_lat * sin_lat + std::cos(lat1) * std::cos(lat2) * sin_lon * sin_lon;
    const double horizontal = 2.0 * kEarthRadiusM * std::asin(std::sqrt(std::clamp(a, 0.0, 1.0)));
    const double vertical = static_cast<double>(second.altitude) - first.altitude;
    return std::hypot(horizontal, vertical);
}

std::string track_id_string(const nlohmann::json& value) {
    if (value.is_string()) {
        return value.get<std::string>();
    }
    if (value.is_number_integer()) {
        return std::to_string(value.get<std::int64_t>());
    }
    if (value.is_number_unsigned()) {
        return std::to_string(value.get<std::uint64_t>());
    }
    return {};
}

/// n<=m 的矩形 Hungarian，返回每一行匹配的列下标。
std::vector<std::size_t> minimum_assignment(const std::vector<std::vector<double>>& cost) {
    const std::size_t n = cost.size();
    const std::size_t m = cost.front().size();
    std::vector<double> u(n + 1), v(m + 1);
    std::vector<std::size_t> p(m + 1), way(m + 1);
    for (std::size_t i = 1; i <= n; ++i) {
        p[0] = i;
        std::size_t j0 = 0;
        std::vector<double> minv(m + 1, std::numeric_limits<double>::infinity());
        std::vector<bool> used(m + 1, false);
        do {
            used[j0] = true;
            const std::size_t i0 = p[j0];
            double delta = std::numeric_limits<double>::infinity();
            std::size_t j1 = 0;
            for (std::size_t j = 1; j <= m; ++j) {
                if (used[j]) continue;
                const double current = cost[i0 - 1][j - 1] - u[i0] - v[j];
                if (current < minv[j]) {
                    minv[j] = current;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }
            for (std::size_t j = 0; j <= m; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do {
            const std::size_t j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }
    std::vector<std::size_t> assignment(n, m);
    for (std::size_t j = 1; j <= m; ++j) {
        if (p[j] != 0) assignment[p[j] - 1] = j - 1;
    }
    return assignment;
}

}  // namespace

RadarTargetFilter::RadarTargetFilter(RadarFilterConfig config) : config_(config) {}

bool RadarTargetFilter::update_friendly(const std::string& sender_id, const plane::model::VehicleState& state,
                                        std::int64_t received_unix_ms, std::string& error) {
    if (sender_id.empty() || state.planeId.empty() || sender_id != state.planeId || received_unix_ms <= 0 ||
        !valid_geo(state.geoPos) || !std::isfinite(state.vn_m_s) || !std::isfinite(state.ve_m_s) ||
        !std::isfinite(state.vd_m_s)) {
        error = "友机状态的发送方、时间戳、位置或速度非法";
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    friendlies_[sender_id] = FriendlySample{state, received_unix_ms};
    return true;
}

bool RadarTargetFilter::process_scan(const std::string& payload, std::int64_t now_unix_ms,
                                     std::vector<plane::target::GcsTarget>& out, std::string& error) const {
    out.clear();
    nlohmann::json scan;
    try {
        scan = nlohmann::json::parse(payload);
        // 部分 UDP 发送框架会对已经 dump 的 JSON 再序列化一次。兼容这类
        // 顶层 JSON 字符串，但仍要求解开后是 radar_tracks 对象。
        if (scan.is_string()) {
            scan = nlohmann::json::parse(scan.get<std::string>());
        }
        const auto type = scan.is_object() ? scan.find("type") : scan.end();
        if (!scan.is_object() || type == scan.end() || !type->is_string() ||
            type->get<std::string>() != "radar_tracks") {
            const std::string actual = type != scan.end() ? type->dump() : "<missing>";
            error = "忽略非 radar_tracks 报文：实际 type=" + actual + "，顶层=" + std::string(scan.type_name());
            return false;
        }
        const std::int64_t timestamp_ms = scan.at("timestamp_ms").get<std::int64_t>();
        if (timestamp_ms <= 0 || !scan.at("tracks").is_array()) {
            error = "雷达报文 timestamp_ms 或 tracks 非法";
            return false;
        }

        std::vector<plane::target::GcsTarget> confirmed;
        for (const auto& track : scan.at("tracks")) {
            if (!track.is_object() || track.value("state", std::string{}) != "confirmed") {
                continue;
            }
            plane::target::GcsTarget target;
            target.trackId = track_id_string(track.at("track_id"));
            target.timestamp_ms = timestamp_ms;
            target.geoPos.latitude = track.at("latitude").get<double>();
            target.geoPos.longitude = track.at("longitude").get<double>();
            target.geoPos.altitude = track.at("altitude_msl_m").get<float>();
            target.geoPos.height = 0.0F;
            target.radial_speed_m_s = track.at("radial_speed_m_s").get<float>();
            target.azimuth_deg = track.at("azimuth_deg").get<float>();
            if (target.trackId.empty() || !valid_geo(target.geoPos) || !std::isfinite(target.radial_speed_m_s) ||
                !std::isfinite(target.azimuth_deg)) {
                error = "confirmed 雷达航迹包含非法必需字段";
                return false;
            }
            confirmed.push_back(std::move(target));
        }

        std::vector<FriendlySample> friendlies;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            friendlies.reserve(friendlies_.size());
            for (const auto& [sender, sample] : friendlies_) {
                (void)sender;
                if (now_unix_ms < sample.received_unix_ms ||
                    now_unix_ms - sample.received_unix_ms > config_.friendly_state_timeout_ms) {
                    error = "存在超过有效期的友机状态，暂停整帧转发";
                    return false;
                }
                if (std::llabs(timestamp_ms - sample.received_unix_ms) > config_.friendly_state_timeout_ms) {
                    error = "雷达扫描与友机状态时间差超过有效期，暂停整帧转发";
                    return false;
                }
                friendlies.push_back(sample);
            }
        }
        if (friendlies.empty()) {
            error = "尚未收到友机状态，暂停整帧转发";
            return false;
        }
        // 雷达扫描不要求包含每一架友机。为每架友机增加一个专用的“未匹配”
        // 虚拟列，使 Hungarian 仍能完成全局一对一最优匹配；只有门限内实际
        // 匹配到的雷达航迹才会被过滤，未匹配的友机不会阻塞整帧靶机数据。
        const std::size_t real_track_count = confirmed.size();
        const std::size_t assignment_column_count = real_track_count + friendlies.size();
        const double unmatched_cost = config_.friendly_match_max_distance_m + 1.0;
        std::vector<std::vector<double>> cost(friendlies.size(),
                                              std::vector<double>(assignment_column_count, unmatched_cost));
        std::vector<std::vector<double>> distance(friendlies.size(), std::vector<double>(confirmed.size()));
        for (std::size_t i = 0; i < friendlies.size(); ++i) {
            const plane::model::GeoPoint projected =
                project_position(friendlies[i].state, friendlies[i].received_unix_ms, timestamp_ms);
            for (std::size_t j = 0; j < confirmed.size(); ++j) {
                distance[i][j] = distance_3d_m(projected, confirmed[j].geoPos);
                cost[i][j] =
                    distance[i][j] <= config_.friendly_match_max_distance_m ? distance[i][j] : kInvalidAssignmentCost;
            }
        }
        const std::vector<std::size_t> assignment = minimum_assignment(cost);
        std::vector<bool> friendly_track(confirmed.size(), false);
        for (std::size_t i = 0; i < assignment.size(); ++i) {
            if (assignment[i] < real_track_count &&
                distance[i][assignment[i]] <= config_.friendly_match_max_distance_m) {
                friendly_track[assignment[i]] = true;
            }
        }
        for (std::size_t index = 0; index < confirmed.size(); ++index) {
            if (!friendly_track[index]) out.push_back(std::move(confirmed[index]));
        }
        return true;
    } catch (const nlohmann::json::exception& exception) {
        error = std::string("雷达 JSON 缺少或包含非法必需字段: ") + exception.what();
        return false;
    }
}

}  // namespace plane::gcs_sim
