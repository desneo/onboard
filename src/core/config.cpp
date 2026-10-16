#include "core/config.h"

#include <spdlog/spdlog.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

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

bool parse_bool(const std::string& value) {
    return value == "1" || value == "true" || value == "True" || value == "on";
}

std::string canonical_key(const std::string& section, const std::string& key) {
    static const std::unordered_map<std::string, std::string> keys = {
        {"system.safe_height_m", "safe_height_m"},
        {"system.log_dir", "log_dir"},
        {"system.state_report_period_ms", "state_report_period_ms"},
        {"communication.plane_id", "plane_id"},
        {"communication.station_id", "station_id"},
        {"communication.zenoh_config_file", "zenoh_config_file"},
        {"communication.zenoh_connect", "zenoh_connect"},
        {"communication.zenoh_listen", "zenoh_listen"},
        {"communication.debug_log", "debug_zenoh_log"},
        {"communication.ack_timeout_ms", "comm_ack_timeout_ms"},
        {"communication.ack_max_retries", "comm_ack_max_retries"},
        {"communication.rx_queue_max", "comm_rx_queue_max"},
        {"communication.gcs_link_timeout_ms", "gcs_link_timeout_ms"},
        {"flight_control.link_type", "fc_link_type"},
        {"flight_control.udp_url", "fc_udp_url"},
        {"flight_control.serial_device", "fc_serial_device"},
        {"flight_control.serial_baud", "fc_serial_baud"},
        {"flight_control.heartbeat_timeout_ms", "fc_heartbeat_timeout_ms"},
        {"flight_control.route_speed_m_s", "fc_route_speed_m_s"},
        {"behavior_tree.tick_interval_ms", "bt_tick_interval_ms"},
        {"behavior_tree.node_timeout_ms", "bt_node_timeout_ms"},
        {"behavior_tree.takeoff_tolerance_m", "bt_takeoff_tolerance_m"},
        {"behavior_tree.height_adjust_timeout_ms", "bt_height_adjust_timeout_ms"},
        {"behavior_tree.height_adjust_tolerance_m", "bt_height_adjust_tolerance_m"},
        {"plugins.directory", "plugins_dir"},
        {"image_input.type", "image_source_type"},
        {"image_input.mipi_device", "image_mipi_device"},
        {"image_input.rtsp_url", "image_rtsp_url"},
        {"image_input.reconnect_interval_s", "image_reconnect_interval_s"},
        {"image_input.width", "image_width"},
        {"image_input.height", "image_height"},
        {"detection.model_path", "det_model_path"},
        {"detection.config_dir", "det_config_dir"},
        {"detection.confidence_threshold", "det_confidence_threshold"},
        {"detection.nms_threshold", "det_nms_threshold"},
        {"detection.track_thresh", "det_track_thresh"},
        {"detection.track_buffer", "det_track_buffer"},
        {"detection.input_width", "det_input_width"},
        {"detection.input_height", "det_input_height"},
        {"video_output.type", "stream_type"},
        {"video_output.url", "stream_url"},
        {"video_output.app", "stream_app"},
        {"video_output.stream", "stream_name"},
        {"video_output.rtsp_port", "stream_rtsp_port"},
        {"video_output.http_port", "stream_http_port"},
        {"video_output.rtc_port", "stream_rtc_port"},
        {"video_output.width", "stream_width"},
        {"video_output.height", "stream_height"},
        {"video_output.max_fps", "stream_max_fps"},
        {"video_output.bitrate_kbps", "stream_bitrate_kbps"},
        {"video_output.gop", "stream_gop"},
        {"debug.target_log", "debug_target_log"},
        {"safety.alert_voltage_threshold_v", "alert_voltage_threshold_v"},
        {"suicide_mission.detection_start_distance_m", "suicide_detection_start_distance_m"},
        {"suicide_mission.approach_speed_m_s", "suicide_approach_speed_m_s"},
        {"suicide_mission.track_approach_speed_m_s", "suicide_track_approach_speed_m_s"},
        {"suicide_mission.climb_speed_m_s", "suicide_climb_speed_m_s"},
        {"suicide_mission.target_select_rule", "suicide_target_select_rule"},
        {"suicide_mission.guidance_start_stable_ms", "suicide_guidance_start_stable_ms"},
        {"suicide_mission.track_lost_timeout_ms", "suicide_track_lost_timeout_ms"},
        {"suicide_mission.phase_timeout_ms", "suicide_phase_timeout_ms"},
        {"suicide_mission.impact_height_m", "suicide_impact_height_m"},
        {"suicide_mission.impact_vd_m_s", "suicide_impact_vd_m_s"},
    };
    const std::string qualified = section + "." + key;
    const auto it = keys.find(qualified);
    return it == keys.end() ? qualified : it->second;
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
    std::string section;
    while (std::getline(file, line)) {
        line = trim(line);
        if (line.empty() || line.front() == '#' || line.front() == ';') {
            continue;
        }
        if (line.front() == '[') {
            if (line.size() < 3 || line.back() != ']') {
                SPDLOG_WARN("忽略无法解析的配置 section: {}", line);
                continue;
            }
            section = trim(line.substr(1, line.size() - 2));
            continue;
        }
        const auto pos = line.find('=');
        if (pos == std::string::npos) {
            SPDLOG_WARN("忽略无法解析的配置行: {}", line);
            continue;
        }
        if (section.empty()) {
            SPDLOG_WARN("忽略缺少 section 的配置项: {}", line);
            continue;
        }
        const std::string raw_key = trim(line.substr(0, pos));
        const std::string key = canonical_key(section, raw_key);
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
        } else if (key == "debug_zenoh_log") {
            out.debug_zenoh_log = parse_bool(value);
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
        } else if (key == "bt_height_adjust_timeout_ms") {
            out.bt_height_adjust_timeout_ms = std::stoi(value);
        } else if (key == "bt_height_adjust_tolerance_m") {
            out.bt_height_adjust_tolerance_m = std::stod(value);
        } else if (key == "plugins_dir") {
            out.plugins_dir = value;
        } else if (key == "image_source_type") {
            out.image_source_type = value;
        } else if (key == "image_mipi_device") {
            out.image_mipi_device = value;
        } else if (key == "image_rtsp_url") {
            out.image_rtsp_url = value;
        } else if (key == "image_reconnect_interval_s") {
            out.image_reconnect_interval_s = std::stoi(value);
        } else if (key == "image_width") {
            out.image_width = std::stoi(value);
        } else if (key == "image_height") {
            out.image_height = std::stoi(value);
        } else if (key == "det_model_path") {
            out.det_model_path = value;
        } else if (key == "det_config_dir") {
            out.det_config_dir = value;
        } else if (key == "det_confidence_threshold") {
            out.det_confidence_threshold = std::stof(value);
        } else if (key == "det_nms_threshold") {
            out.det_nms_threshold = std::stof(value);
        } else if (key == "det_track_thresh") {
            out.det_track_thresh = std::stof(value);
        } else if (key == "det_track_buffer") {
            out.det_track_buffer = std::stoi(value);
        } else if (key == "det_input_width") {
            out.det_input_width = std::stoi(value);
        } else if (key == "det_input_height") {
            out.det_input_height = std::stoi(value);
        } else if (key == "stream_type") {
            out.stream_type = value;
        } else if (key == "stream_url") {
            out.stream_url = value;
        } else if (key == "stream_app") {
            out.stream_app = value;
        } else if (key == "stream_name") {
            out.stream_name = value;
        } else if (key == "stream_rtsp_port") {
            out.stream_rtsp_port = std::stoi(value);
        } else if (key == "stream_http_port") {
            out.stream_http_port = std::stoi(value);
        } else if (key == "stream_rtc_port") {
            out.stream_rtc_port = std::stoi(value);
        } else if (key == "stream_width") {
            out.stream_width = std::stoi(value);
        } else if (key == "stream_height") {
            out.stream_height = std::stoi(value);
        } else if (key == "stream_max_fps") {
            out.stream_max_fps = std::stoi(value);
        } else if (key == "stream_bitrate_kbps") {
            out.stream_bitrate_kbps = std::stoi(value);
        } else if (key == "stream_gop") {
            out.stream_gop = std::stoi(value);
        } else if (key == "debug_target_log") {
            out.debug_target_log = parse_bool(value);
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
            SPDLOG_WARN("未知配置项: {}.{}", section, raw_key);
        }
    }
    out.is_sim = detect_is_simulation(out);
    return true;
}

}  // namespace plane::core
