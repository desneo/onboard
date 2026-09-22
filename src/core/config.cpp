#include "core/config.h"

#include <spdlog/spdlog.h>

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace plane::core {
namespace {

std::string trim(const std::string& text) {
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

/// 仿真飞机判定：udp 链路且 fc_udp_url 端口处于 14200-14299。
bool detect_is_simulation(const AppConfig& config) {
    if (config.fc_link_type != "udp") {
        return false;
    }
    const std::size_t sep = config.fc_udp_url.find_last_of(':');
    if (sep == std::string::npos) {
        return false;
    }
    try {
        const int port = std::stoi(config.fc_udp_url.substr(sep + 1));
        return port >= 14200 && port <= 14299;
    } catch (const std::invalid_argument&) {
        return false;
    } catch (const std::out_of_range&) {
        return false;
    }
}

}  // namespace

bool load_config(const std::string& path, AppConfig& out) {
    std::ifstream file(path);
    if (!file.is_open()) {
        SPDLOG_WARN("配置文件不存在，使用默认配置: {}", path);
        out.is_sim = detect_is_simulation(out);
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        line = trim(line);
        if (line.empty() || line.front() == '#') {
            continue;
        }
        const auto pos = line.find('=');
        if (pos == std::string::npos) {
            SPDLOG_WARN("忽略无法解析的配置行: {}", line);
            continue;
        }
        const std::string key = trim(line.substr(0, pos));
        const std::string value = trim(line.substr(pos + 1));
        if (key == "safe_height_m") {
            out.system_param.safeHeight_m = std::stod(value);
        } else if (key == "log_dir") {
            out.log_dir = value;
        } else if (key == "state_report_period_ms") {
            out.state_report_period_ms = std::stoi(value);
        } else if (key == "plane_id") {
            out.plane_id = value;
        } else if (key == "station_id") {
            out.station_id = value;
        } else if (key == "zenoh_config_file") {
            out.zenoh_config_file = value;
        } else if (key == "zenoh_connect") {
            out.zenoh_connect = value;
        } else if (key == "zenoh_listen") {
            out.zenoh_listen = value;
        } else if (key == "comm_ack_timeout_ms") {
            out.comm_ack_timeout_ms = std::stoi(value);
        } else if (key == "comm_ack_max_retries") {
            out.comm_ack_max_retries = std::stoi(value);
        } else if (key == "comm_rx_queue_max") {
            out.comm_rx_queue_max = std::stoi(value);
        } else if (key == "gcs_link_timeout_ms") {
            out.gcs_link_timeout_ms = std::stoi(value);
        } else if (key == "fc_link_type") {
            out.fc_link_type = value;
        } else if (key == "fc_udp_url") {
            out.fc_udp_url = value;
        } else if (key == "fc_serial_device") {
            out.fc_serial_device = value;
        } else if (key == "fc_serial_baud") {
            out.fc_serial_baud = std::stoi(value);
        } else if (key == "fc_heartbeat_timeout_ms") {
            out.fc_heartbeat_timeout_ms = std::stoi(value);
        } else if (key == "fc_route_speed_m_s") {
            out.fc_route_speed_m_s = std::stod(value);
        } else if (key == "bt_tick_interval_ms") {
            out.bt_tick_interval_ms = std::stoi(value);
        } else if (key == "bt_node_timeout_ms") {
            out.bt_node_timeout_ms = std::stoi(value);
        } else if (key == "bt_takeoff_tolerance_m") {
            out.bt_takeoff_tolerance_m = std::stod(value);
        } else if (key == "plugins_dir") {
            out.plugins_dir = value;
        } else if (key == "image_source") {
            out.image_source = value;
        } else if (key == "image_reconnect_interval_ms") {
            out.image_reconnect_interval_ms = std::stoi(value);
        } else if (key == "perception_process_interval_ms") {
            out.perception_process_interval_ms = std::stoi(value);
        } else if (key == "perception_selftest_on_start") {
            out.perception_selftest_on_start = (value == "true" || value == "1");
        } else if (key == "rtsp_port") {
            out.rtsp_port = std::stoi(value);
        } else if (key == "rtsp_app") {
            out.rtsp_app = value;
        } else if (key == "rtsp_stream") {
            out.rtsp_stream = value;
        } else if (key == "video_fps") {
            out.video_fps = std::stoi(value);
        } else if (key == "video_bitrate_kbps") {
            out.video_bitrate_kbps = std::stoi(value);
        } else if (key == "video_overlay_enabled") {
            out.video_overlay_enabled = (value == "true" || value == "1");
        } else if (key == "alert_voltage_threshold_v") {
            out.alert_voltage_threshold_v = std::stod(value);
        } else if (key == "suicide_detection_start_distance_m") {
            out.suicide_detection_start_distance_m = std::stod(value);
        } else if (key == "suicide_approach_speed_m_s") {
            out.suicide_approach_speed_m_s = std::stod(value);
        } else if (key == "suicide_track_approach_speed_m_s") {
            out.suicide_track_approach_speed_m_s = std::stod(value);
        } else if (key == "suicide_climb_speed_m_s") {
            out.suicide_climb_speed_m_s = std::stod(value);
        } else if (key == "suicide_target_select_rule") {
            out.suicide_target_select_rule = value;
        } else if (key == "suicide_guidance_start_stable_ms") {
            out.suicide_guidance_start_stable_ms = std::stoi(value);
        } else if (key == "suicide_track_lost_timeout_ms") {
            out.suicide_track_lost_timeout_ms = std::stoi(value);
        } else if (key == "suicide_phase_timeout_ms") {
            out.suicide_phase_timeout_ms = std::stoi(value);
        } else if (key == "suicide_impact_height_m") {
            out.suicide_impact_height_m = std::stod(value);
        } else if (key == "suicide_impact_vd_m_s") {
            out.suicide_impact_vd_m_s = std::stod(value);
        } else {
            SPDLOG_WARN("未知配置项: {}", key);
        }
    }
    out.is_sim = detect_is_simulation(out);
    return true;
}

}  // namespace plane::core
