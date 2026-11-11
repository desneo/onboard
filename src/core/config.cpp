#include "core/config.h"

#include <spdlog/spdlog.h>

#include <cmath>
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

bool load_midcourse_algorithm_option(const std::string& key, const std::string& value, AppConfig& out) {
    using plane::algorithm::MidCourseConfig;
    using plane::algorithm::MidcourseControlMode;
    static const std::unordered_map<std::string, double MidCourseConfig::*> numeric_fields = {
        {"max_horizontal_speed_m_s", &MidCourseConfig::max_horizontal_speed_m_s},
        {"max_climb_speed_m_s", &MidCourseConfig::max_climb_speed_m_s},
        {"max_descent_speed_m_s", &MidCourseConfig::max_descent_speed_m_s},
        {"cmd_accel_max_m_s2", &MidCourseConfig::cmd_accel_max_m_s2},
        {"vehicle_mass_kg", &MidCourseConfig::vehicle_mass_kg},
        {"max_thrust_n", &MidCourseConfig::max_thrust_n},
        {"r_min_m", &MidCourseConfig::r_min_m},
        {"target_invalid_grace_s", &MidCourseConfig::target_invalid_grace_s},
        {"yaw_rate_max_deg_s", &MidCourseConfig::yaw_rate_max_deg_s},
        {"half_fov_deg", &MidCourseConfig::half_fov_deg},
        {"r_capture_min_m", &MidCourseConfig::r_capture_min_m},
        {"r_capture_max_m", &MidCourseConfig::r_capture_max_m},
        {"r_capture_nominal_m", &MidCourseConfig::r_capture_nominal_m},
        {"capture_discount", &MidCourseConfig::capture_discount},
        {"sigma_dot_max_deg_s", &MidCourseConfig::sigma_dot_max_deg_s},
        {"speed_ratio", &MidCourseConfig::speed_ratio},
        {"tol_pos_m", &MidCourseConfig::tol_pos_m},
        {"tol_vel_m", &MidCourseConfig::tol_vel_m},
        {"tol_accel_m_s2", &MidCourseConfig::tol_accel_m_s2},
        {"min_altitude_m", &MidCourseConfig::min_altitude_m},
        {"w_time", &MidCourseConfig::w_time},
        {"w_speed", &MidCourseConfig::w_speed},
        {"w_accel", &MidCourseConfig::w_accel},
        {"replan_stitch_s", &MidCourseConfig::replan_stitch_s},
        {"track_gain_k", &MidCourseConfig::track_gain_k},
        {"track_max_corr", &MidCourseConfig::track_max_corr},
        {"recovery_gain_k", &MidCourseConfig::recovery_gain_k},
        {"recovery_margin_m", &MidCourseConfig::recovery_margin_m},
        {"recovery_exit_hold_s", &MidCourseConfig::recovery_exit_hold_s},
        {"follow_gain_k", &MidCourseConfig::follow_gain_k},
        {"follow_filter_a", &MidCourseConfig::follow_filter_a},
        {"follow_vel_tau_s", &MidCourseConfig::follow_vel_tau_s},
    };
    const auto field = numeric_fields.find(key);
    try {
        if (field != numeric_fields.end()) {
            std::size_t parsed = 0;
            const double number = std::stod(value, &parsed);
            if (parsed == value.size() && std::isfinite(number)) {
                out.midcourse_algorithm.*(field->second) = number;
                return true;
            }
        } else if (key == "control_output_mode") {
            if (value == "VelocityAcceleration" || value == "Attitude") {
                out.midcourse_algorithm.control_output_mode = value == "Attitude"
                    ? MidcourseControlMode::Attitude : MidcourseControlMode::VelocityAcceleration;
                return true;
            }
        } else if (key == "yaw_mode") {
            if (value == "0" || value == "1") {
                out.midcourse_algorithm.yaw_mode = value == "1" ? 1 : 0;
                return true;
            }
        } else if (key == "use_px4_limits" || key == "use_cost_search") {
            if (value == "true" || value == "false") {
                if (key == "use_px4_limits") {
                    out.midcourse_use_px4_limits = value == "true";
                } else {
                    out.midcourse_algorithm.use_cost_search = value == "true";
                }
                return true;
            }
        } else {
            return false;
        }
    } catch (const std::invalid_argument&) {
    } catch (const std::out_of_range&) {
    }
    throw std::invalid_argument("中制导算法配置值非法: " + key + "=" + value);
}

std::string canonical_key(const std::string& section, const std::string& key) {
    static const std::unordered_map<std::string, std::string> keys = {
        {"system.safe_height_m", "safe_height_m"},
        {"system.log_dir", "log_dir"},
        {"system.board_ip", "board_ip"},
        {"camera.pod_video_url", "pod_video_url"},
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
        {"safety.alert_voltage_threshold_v", "alert_voltage_threshold_v"},
        {"midcourse.init_timeout_ms", "midcourse_init_timeout_ms"},
        {"midcourse.target_invalid_grace_s", "midcourse_target_invalid_grace_s"},
        {"midcourse.r_min_m", "midcourse_r_min_m"},
    };
    const std::string qualified = section + "." + key;
    const auto it = keys.find(qualified);
    return it == keys.end() ? qualified : it->second;
}

/// 仿真飞机判定：udp 链路且 fc_udp_url 使用场景 02 的本地／远端端口或旧 SITL 端口。
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
        return (port >= 14030 && port <= 14039) || (port >= 14280 && port <= 14289) ||
               (port >= 14580 && port <= 14589);
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
                SPDLOG_WARN("忽略无法解析的配置节: {}", line);
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
            SPDLOG_WARN("忽略缺少配置节的配置项: {}", line);
            continue;
        }
        const std::string raw_key = trim(line.substr(0, pos));
        const std::string value = trim(line.substr(pos + 1));
        if (section == "midcourse_algorithm") {
            if (!load_midcourse_algorithm_option(raw_key, value, out)) {
                SPDLOG_WARN("未知配置项: {}.{}", section, raw_key);
            }
            continue;
        }
        const std::string key = canonical_key(section, raw_key);
        if (key == "safe_height_m") {
            out.system_param.safeHeight_m = std::stod(value);
        } else if (key == "log_dir") {
            out.log_dir = value;
        } else if (key == "state_report_period_ms") {
            out.state_report_period_ms = std::stoi(value);
        } else if (key == "board_ip") {
            out.board_ip = value;
        } else if (key == "pod_video_url") {
            out.pod_video_url = value;
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
        } else if (key == "alert_voltage_threshold_v") {
            out.alert_voltage_threshold_v = std::stod(value);
        } else if (key == "midcourse_init_timeout_ms") {
            out.midcourse_init_timeout_ms = std::stoi(value);
        } else if (key == "midcourse_target_invalid_grace_s") {
            load_midcourse_algorithm_option(raw_key, value, out);
        } else if (key == "midcourse_r_min_m") {
            load_midcourse_algorithm_option(raw_key, value, out);
        } else {
            SPDLOG_WARN("未知配置项: {}.{}", section, raw_key);
        }
    }
    out.is_sim = detect_is_simulation(out);
    return true;
}

}  // namespace plane::core
