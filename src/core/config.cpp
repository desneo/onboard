#include "core/config.h"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <utility>

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

int parse_positive_int(std::string_view value) {
    int number = 0;
    const auto end = value.data() + value.size();
    const auto result = std::from_chars(value.data(), end, number);
    return result.ec == std::errc{} && result.ptr == end && number > 0 ? number : 0;
}

std::vector<std::string> get_local_ipv4_addresses() {
    ifaddrs* interfaces = nullptr;
    if (getifaddrs(&interfaces) != 0) {
        throw std::system_error(errno, std::generic_category(), "枚举本机 IPv4 网卡失败");
    }
    const std::unique_ptr<ifaddrs, decltype(&freeifaddrs)> guard(interfaces, freeifaddrs);
    std::vector<std::string> addresses;
    for (const auto* entry = interfaces; entry != nullptr; entry = entry->ifa_next) {
        if (entry->ifa_addr == nullptr || entry->ifa_addr->sa_family != AF_INET ||
            (entry->ifa_flags & IFF_UP) == 0 || (entry->ifa_flags & IFF_LOOPBACK) != 0) {
            continue;
        }
        const auto* address = reinterpret_cast<const sockaddr_in*>(entry->ifa_addr);
        char buffer[INET_ADDRSTRLEN];
        if (inet_ntop(AF_INET, &address->sin_addr, buffer, sizeof(buffer)) != nullptr) {
            addresses.emplace_back(buffer);
        }
    }
    return addresses;
}

using IniOptions = std::unordered_map<std::string, std::string>;

std::string section_name(const std::string& line) {
    if (line.size() < 3 || line.back() != ']') {
        return {};
    }
    return trim(line.substr(1, line.size() - 2));
}

std::unordered_map<std::string, IniOptions> read_profile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("无法打开文件");
    }
    std::unordered_map<std::string, IniOptions> entries;
    std::string section;
    std::string line;
    int line_number = 0;
    while (std::getline(file, line)) {
        ++line_number;
        line = trim(line);
        if (line.empty() || line.front() == '#' || line.front() == ';') {
            continue;
        }
        if (line.front() == '[') {
            section = section_name(line);
            if (section.empty() || section.find_first_of("/*?#$[] \t") != std::string::npos) {
                throw std::runtime_error("飞机 ID 无效，行号=" + std::to_string(line_number));
            }
            if (!entries.try_emplace(section).second) {
                throw std::runtime_error("plane_id 重复: " + section);
            }
            continue;
        }
        const auto separator = line.find('=');
        if (section.empty() || separator == std::string::npos) {
            throw std::runtime_error("无法解析配置项，行号=" + std::to_string(line_number));
        }
        const auto key = trim(line.substr(0, separator));
        if (key.empty() || !entries.at(section).emplace(key, trim(line.substr(separator + 1))).second) {
            throw std::runtime_error("配置键为空或重复，行号=" + std::to_string(line_number));
        }
    }
    if (file.bad()) {
        throw std::runtime_error("读取文件失败");
    }
    return entries;
}

std::string profile_value(const IniOptions& options, const std::string& key) {
    const auto it = options.find(key);
    return it == options.end() ? std::string{} : it->second;
}

struct FlightEndpoint {
    std::string serial_device;
    int serial_baud = 0;
};

FlightEndpoint parse_flight_url(const std::string& url, const std::string& plane_id) {
    const auto scheme_end = url.find("://");
    const auto number_start = url.rfind(':');
    if (scheme_end != std::string::npos && number_start >= scheme_end + 3 &&
        url.find_first_of(" \t\r\n") == std::string::npos) {
        const auto scheme = url.substr(0, scheme_end);
        const auto address = url.substr(scheme_end + 3, number_start - scheme_end - 3);
        const int number = parse_positive_int(std::string_view(url).substr(number_start + 1));
        if (scheme == "serial" && !address.empty() && address.front() == '/' && number > 0) {
            return {address, number};
        }
        if ((scheme == "udp" || scheme == "udpin" || scheme == "udpout") &&
            (scheme != "udpout" || !address.empty()) && address.find_first_of("/\\?#@") == std::string::npos &&
            number > 0 && number <= 65535) {
            return {};
        }
    }
    throw std::runtime_error("fc_url 无效，plane_id=" + plane_id);
}

void apply_profile(const std::string& path, const std::vector<std::string>& local_addresses, AppConfig& config) {
    std::unordered_set<std::string> board_ips;
    std::string matched_id;
    for (const auto& [plane_id, options] : read_profile(path)) {
        const auto board_ip = profile_value(options, "board_ip");
        in_addr address{};
        if (inet_pton(AF_INET, board_ip.c_str(), &address) != 1) {
            throw std::runtime_error("board_ip 无效，plane_id=" + plane_id);
        }
        if (!board_ips.insert(board_ip).second) {
            throw std::runtime_error("board_ip 重复: " + board_ip);
        }

        const auto fc_url = profile_value(options, "fc_url");
        const auto endpoint = parse_flight_url(fc_url, plane_id);
        if (std::find(local_addresses.begin(), local_addresses.end(), board_ip) == local_addresses.end()) {
            continue;
        }
        if (!matched_id.empty()) {
            throw std::runtime_error("本机 IPv4 命中多个飞机 ID: " + matched_id + "、" + plane_id);
        }
        matched_id = plane_id;
        config.plane_id = plane_id;
        config.board_ip = board_ip;
        config.pod_model = profile_value(options, "pod_model");
        config.pod_video_url = profile_value(options, "pod_video_url");
        config.pod_control_url = profile_value(options, "pod_control_url");
        if (endpoint.serial_device.empty()) {
            config.fc_link_type = "udp";
            config.fc_udp_url = fc_url;
        } else {
            config.fc_link_type = "serial";
            config.fc_serial_device = endpoint.serial_device;
            config.fc_serial_baud = endpoint.serial_baud;
        }
    }
    if (matched_id.empty()) {
        throw std::runtime_error("本机 IPv4 未匹配任何飞机");
    }
    SPDLOG_INFO("部署预置已匹配: profile={} plane_id={} board_ip={} pod_model={}", path, config.plane_id,
                config.board_ip, config.pod_model);
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
        {"camera.pod_model", "pod_model"},
        {"camera.pod_video_url", "pod_video_url"},
        {"camera.pod_control_url", "pod_control_url"},
        {"camera.pod_state_report_hz", "pod_state_report_hz"},
        {"system.state_report_period_ms", "state_report_period_ms"},
        {"communication.plane_id", "plane_id"},
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
    const int port = parse_positive_int(std::string_view(config.fc_udp_url).substr(sep + 1));
    return (port >= 14030 && port <= 14039) || (port >= 14280 && port <= 14289) ||
           (port >= 14580 && port <= 14589);
}

std::string read_main_config(const std::string& path, AppConfig& out) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("无法打开文件");
    }

    std::string profile;
    bool profile_configured = false;
    std::string line;
    std::string section;
    while (std::getline(file, line)) {
        line = trim(line);
        if (line.empty() || line.front() == '#' || line.front() == ';') {
            continue;
        }
        if (line.front() == '[') {
            section = section_name(line);
            if (section.empty()) {
                throw std::runtime_error("无法解析配置节: " + line);
            }
            continue;
        }
        const auto pos = line.find('=');
        if (pos == std::string::npos || section.empty()) {
            throw std::runtime_error("无法解析配置项: " + line);
        }
        const std::string raw_key = trim(line.substr(0, pos));
        const std::string value = trim(line.substr(pos + 1));
        if (section == "deployment" && raw_key == "profile") {
            if (profile_configured) {
                throw std::runtime_error("deployment.profile 重复");
            }
            profile = value;
            profile_configured = true;
            continue;
        }
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
        } else if (key == "pod_model") {
            out.pod_model = value;
        } else if (key == "pod_video_url") {
            out.pod_video_url = value;
        } else if (key == "pod_control_url") {
            out.pod_control_url = value;
        } else if (key == "pod_state_report_hz") {
            out.pod_state_report_hz = parse_positive_int(value);
            if (out.pod_state_report_hz <= 0) {
                out.pod_state_report_hz = kDefaultPodStateReportHz;
                SPDLOG_WARN("吊舱状态频率配置无效，使用默认值: pod_state_report_hz={}（输入：{}）",
                            out.pod_state_report_hz, value);
            }
        } else if (key == "plane_id") {
            out.plane_id = value;
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
    if (file.bad()) {
        throw std::runtime_error("读取文件失败");
    }
    return profile;
}

}  // namespace

bool load_config(const std::string& path, AppConfig& out, const std::vector<std::string>* local_ipv4_addresses) {
    AppConfig loaded = out;
    loaded.pod_state_report_hz = kDefaultPodStateReportHz;
    std::string profile;
    try {
        profile = read_main_config(path, loaded);
        if (!profile.empty()) {
            if (local_ipv4_addresses == nullptr) {
                apply_profile(profile, get_local_ipv4_addresses(), loaded);
            } else {
                apply_profile(profile, *local_ipv4_addresses, loaded);
            }
        }
    } catch (const std::exception& error) {
        SPDLOG_ERROR("加载配置失败: path={} {}", profile.empty() ? path : profile, error.what());
        return false;
    }
    loaded.is_sim = detect_is_simulation(loaded);
    out = std::move(loaded);
    return true;
}

}  // namespace plane::core
