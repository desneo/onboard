#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

#include "core/config.h"

int main() {
    char path[] = "/tmp/plane-config-simulation-XXXXXX";
    const int fd = mkstemp(path);
    if (fd < 0) return 1;
    close(fd);
    struct Cleanup {
        const char* path;
        std::string profile_path;
        ~Cleanup() {
            std::filesystem::remove(path);
            std::filesystem::remove(profile_path);
        }
    } cleanup{path, std::string(path) + ".ini"};

    const std::vector<std::string> first{"10.0.0.2", "192.168.1.100", "192.168.1.100"};
    const auto write_profile = [&](const std::string& content) {
        std::ofstream file(cleanup.profile_path);
        file << content;
        return static_cast<bool>(file);
    };
    struct RequiredOptions {
        std::optional<std::string> pod_state_report_hz = "5";
        std::optional<std::string> tree_update_rate_hz = "20";
        std::optional<std::string> target_history_size = "20";
        std::optional<std::string> target_timeout_ms = "5000";
    };
    const auto write_main = [&](const std::string& content, const RequiredOptions& options = {}) {
        std::ofstream file(path);
        file << "[deployment]\nprofile = " << cleanup.profile_path << '\n';
        if (options.pod_state_report_hz) {
            file << "[camera]\npod_state_report_hz = " << *options.pod_state_report_hz << '\n';
        }
        if (options.tree_update_rate_hz) {
            file << "[behavior_tree]\ntree_update_rate_hz = " << *options.tree_update_rate_hz << '\n';
        }
        file << "[midcourse]\n";
        if (options.target_history_size) {
            file << "target_history_size = " << *options.target_history_size << '\n';
        }
        if (options.target_timeout_ms) {
            file << "target_timeout_ms = " << *options.target_timeout_ms << '\n';
        }
        file << content;
        return static_cast<bool>(file);
    };
    const plane::core::AppConfig defaults;
    if (!defaults.plane_id.empty() || !defaults.board_ip.empty() || !defaults.fc_link_type.empty() ||
        !defaults.fc_udp_url.empty() || !defaults.fc_serial_device.empty() || defaults.fc_serial_baud != 0 ||
        defaults.log_flush_interval_ms != 1000 || defaults.pod_state_report_hz != 0 ||
        defaults.bt_update_rate_hz != 0 || defaults.bt_tick_period_s() != 0.0 ||
        defaults.midcourse_target_timeout_ms != 0 || defaults.midcourse_target_history_size != 0 ||
        defaults.midcourse_algorithm.replan_period_s != 1.0 ||
        defaults.midcourse_algorithm.replan_freeze_s != 2.5 ||
        defaults.midcourse_algorithm.recovery_vz_deadband_m_s != 0.1 ||
        defaults.midcourse_algorithm.recovery_altitude_tolerance_m != 0.3) {
        return 28;
    }

    struct Case {
        const char* url;
        bool expected;
    };
    const Case cases[] = {
        {"udpin://127.0.0.1:14030", true},
        {"udpin://:14039", true},
        {"udpin://127.0.0.1:14029", false},
        {"udpin://127.0.0.1:14040", false},
        {"udpout://127.0.0.1:14280", true},
        {"udpin://:14289", true},
        {"udpout://127.0.0.1:14279", false},
        {"udpout://127.0.0.1:14290", false},
        {"udpout://127.0.0.1:14580", true},
        {"udpout://127.0.0.1:14583", true},
        {"udpin://:14589", true},
        {"udpout://127.0.0.1:14579", false},
        {"udpout://127.0.0.1:14590", false},
        {"serial:///dev/ttyS0:57600", false},
        {"serial:///dev/ttyUSB0:115200", false},
    };
    for (const auto& test : cases) {
        if (!write_main("") ||
            !write_profile(std::string("[uav1]\nboard_ip = 192.168.1.100\nfc_url = ") + test.url + '\n')) {
            return 2;
        }
        plane::core::AppConfig config;
        config.is_sim = !test.expected;
        if (!plane::core::load_config(path, config, &first) || config.is_sim != test.expected) return 3;
    }

    RequiredOptions custom_options;
    custom_options.target_history_size = "30";
    custom_options.target_timeout_ms = "6500";
    if (!write_main("[logging]\nflush_interval_ms = 250\n"
                    "[midcourse]\ninit_timeout_ms = 12345\n"
                    "target_invalid_grace_s = 2.0\nr_min_m = 75.0\n"
                    "[midcourse_algorithm]\nuse_px4_limits = true\nmax_horizontal_speed_m_s = 30.0\n"
                    "max_climb_speed_m_s = 8.0\nmax_descent_speed_m_s = 4.0\ncmd_accel_max_m_s2 = 7.0\n"
                    "approach_brake_k = 0.75\nhover_brake_accel_m_s2 = 4.0\ntrack_gain_d = 0.25\n"
                    "replan_period_s = 0.75\nreplan_freeze_s = 3.25\n"
                    "recovery_vz_deadband_m_s = 0.25\nrecovery_altitude_tolerance_m = 0.45\n"
                    "control_output_mode = Attitude\nyaw_mode = 0\n", custom_options)) {
        return 4;
    }
    plane::core::AppConfig config;
    if (!plane::core::load_config(path, config, &first)) return 5;
    const auto& algorithm = config.midcourse_algorithm;
    if (config.log_flush_interval_ms != 250 || config.midcourse_init_timeout_ms != 12345 ||
        config.midcourse_target_history_size != 30 || config.midcourse_target_timeout_ms != 6500 ||
        !config.midcourse_use_px4_limits ||
        algorithm.max_horizontal_speed_m_s != 30.0 || algorithm.max_climb_speed_m_s != 8.0 ||
        algorithm.max_descent_speed_m_s != 4.0 || algorithm.cmd_accel_max_m_s2 != 7.0 ||
        algorithm.approach_brake_k != 0.75 || algorithm.hover_brake_accel_m_s2 != 4.0 ||
        algorithm.track_gain_d != 0.25 || algorithm.target_invalid_grace_s != 2.0 || algorithm.r_min_m != 75.0 ||
        algorithm.replan_period_s != 0.75 || algorithm.replan_freeze_s != 3.25 ||
        algorithm.recovery_vz_deadband_m_s != 0.25 ||
        algorithm.recovery_altitude_tolerance_m != 0.45 ||
        algorithm.control_output_mode != plane::algorithm::MidcourseControlMode::Attitude ||
        algorithm.yaw_mode != 0) {
        return 6;
    }

    for (const std::string invalid : {"0", "-1", "abc", "5.0", "5000ms", "2147483648", ""}) {
        auto options = custom_options;
        options.target_timeout_ms = invalid;
        if (!write_main("", options)) return 40;
        const auto previous_plane_id = config.plane_id;
        if (plane::core::load_config(path, config, &first) || config.midcourse_target_timeout_ms != 6500 ||
            config.plane_id != previous_plane_id) {
            return 41;
        }
    }

    for (const std::string invalid : {"0", "4", "31", "-1", "abc", "5.0", "30frames", "2147483648", ""}) {
        auto options = custom_options;
        options.target_history_size = invalid;
        if (!write_main("", options)) return 44;
        const auto previous_plane_id = config.plane_id;
        if (plane::core::load_config(path, config, &first) || config.midcourse_target_history_size != 30 ||
            config.midcourse_target_timeout_ms != 6500 || config.plane_id != previous_plane_id) {
            return 45;
        }
    }
    for (const std::string invalid : {"0", "-1", "abc", "1.5", "1000ms", "2147483648", ""}) {
        if (!write_main("[logging]\nflush_interval_ms = " + invalid + '\n')) return 42;
        const auto previous_plane_id = config.plane_id;
        if (plane::core::load_config(path, config, &first) || config.log_flush_interval_ms != 250 ||
            config.midcourse_target_timeout_ms != 6500 || config.plane_id != previous_plane_id) {
            return 43;
        }
    }
    RequiredOptions minimum_history_options;
    minimum_history_options.target_history_size = "5";
    if (!write_main("", minimum_history_options) ||
        !plane::core::load_config(path, config, &first) || config.midcourse_target_history_size != 5) {
        return 46;
    }

    RequiredOptions pod_options;
    pod_options.pod_state_report_hz = "8";
    if (!write_main("", pod_options)) return 7;
    const std::string uav1 =
        "[uav1]\nboard_ip = 192.168.1.100\nfc_url = udpout://192.168.1.56:14280\n"
        "pod_model = jielian_a2416a\npod_video_url = rtsp://192.168.1.56:8554/jielian_a2416a\n"
        "pod_control_url = udp://192.168.1.56:8001\n";
    const std::string uav2 =
        "[uav2]\nboard_ip = 192.168.1.9\nfc_url = udpin://:14281\n"
        "pod_video_url = rtsp://192.168.1.56:8555/jielian_a2416a\n";
    const std::string uav3 = "[uav3]\nboard_ip = 192.168.1.102\nfc_url = serial:///dev/ttyS0:57600\n";
    if (!write_profile("[groundstation]\nip = 192.168.1.41\n" + uav1 + uav3 + uav2)) return 8;
    if (!plane::core::load_config(path, config, &first) || config.plane_id != "uav1" ||
        config.board_ip != "192.168.1.100" || config.fc_link_type != "udp" ||
        config.fc_udp_url != "udpout://192.168.1.56:14280" || !config.is_sim ||
        !config.fc_serial_device.empty() || config.fc_serial_baud != 0 ||
        config.pod_model != "jielian_a2416a" || config.pod_video_url != "rtsp://192.168.1.56:8554/jielian_a2416a" ||
        config.pod_control_url != "udp://192.168.1.56:8001" || config.pod_state_report_hz != 8 ||
        config.midcourse_target_history_size != 20 || config.midcourse_target_timeout_ms != 5000 ||
        config.log_flush_interval_ms != 1000 ||
        config.zenoh_listen != "tcp/192.168.1.100:7447" ||
        config.zenoh_connect != "tcp/192.168.1.41:7447,tcp/192.168.1.9:7447,tcp/192.168.1.102:7447") {
        return 9;
    }
    const std::vector<std::string> second{"192.168.1.9"};
    if (!write_profile(uav1 + uav2 + uav3) || !plane::core::load_config(path, config, &second) ||
        config.plane_id != "uav2" || config.board_ip != "192.168.1.9" ||
        config.fc_udp_url != "udpin://:14281" || !config.is_sim ||
        config.pod_video_url != "rtsp://192.168.1.56:8555/jielian_a2416a" ||
        !config.pod_model.empty() || !config.pod_control_url.empty() ||
        config.zenoh_listen != "tcp/192.168.1.9:7447" ||
        config.zenoh_connect != "tcp/192.168.1.100:7447,tcp/192.168.1.102:7447") {
        return 10;
    }
    const std::vector<std::string> third{"192.168.1.102"};
    if (!write_profile(uav1 + "[groundstation]\nip = 192.168.1.200\n" + uav2 + uav3) ||
        !plane::core::load_config(path, config, &third) || config.plane_id != "uav3" ||
        config.fc_link_type != "serial" || config.fc_serial_device != "/dev/ttyS0" ||
        config.fc_serial_baud != 57600 || !config.fc_udp_url.empty() || config.is_sim ||
        !config.pod_video_url.empty() ||
        config.zenoh_listen != "tcp/192.168.1.102:7447" ||
        config.zenoh_connect != "tcp/192.168.1.200:7447,tcp/192.168.1.9:7447,tcp/192.168.1.100:7447") {
        return 11;
    }
    const std::vector<std::string> unknown{"192.168.1.103"};
    const std::vector<std::string> multiple{"192.168.1.100", "192.168.1.9"};
    if (plane::core::load_config(path, config, &unknown) || plane::core::load_config(path, config, &multiple)) {
        return 12;
    }
    const std::string invalid_profiles[] = {
        uav1 + uav1,
        uav1 + "[other]\nboard_ip = 192.168.1.100\nfc_url = udp://:14540\n",
        uav1 + "[other]\nboard_ip = invalid\nfc_url = udp://:14540\n",
        uav1 + "[other]\nboard_ip = 192.168.1.104\n",
        uav1 + "[other]\nboard_ip = 192.168.1.104\nfc_url = udpout://192.168.1.56:14280junk\n",
        uav1 + "[other]\nboard_ip = 192.168.1.104\nfc_url = udpout://192.168.1.56:invalid\n",
        uav1 + "[other]\nboard_ip = 192.168.1.104\nfc_url = serial:///dev/ttyS0:0\n",
        uav1 + "[groundstation]\nip = station.local\n",
        uav1 + "[groundstation]\nip = 224.0.0.224\n",
        uav1 + "[groundstation]\nip = 255.255.255.255\n",
        uav1 + "[groundstation]\nip = 0.0.0.0\n",
        uav1 + "[groundstation]\nip = 127.0.0.1\n",
        uav1 + uav2 + "[groundstation]\nip = 192.168.1.9\n",
    };
    for (const auto& invalid : invalid_profiles) {
        if (!write_profile(invalid) || plane::core::load_config(path, config, &first) || config.plane_id != "uav3") {
            return 13;
        }
    }
    if (!write_profile(uav1 + "[groundstation]\nip =\n" + uav3) ||
        !plane::core::load_config(path, config, &first) || config.zenoh_connect != "tcp/192.168.1.102:7447") {
        return 25;
    }
    if (!write_profile(uav1 + "[groundstation]\nip =\n") || !plane::core::load_config(path, config, &first) ||
        config.zenoh_listen != "tcp/192.168.1.100:7447" || !config.zenoh_connect.empty()) {
        return 26;
    }
    const auto retained = config;
    std::filesystem::remove(cleanup.profile_path);
    if (plane::core::load_config(path, config, &first) || config.plane_id != retained.plane_id ||
        config.fc_udp_url != retained.fc_udp_url) {
        return 14;
    }
    for (const auto& content : {
             std::string("[system]\nlog_dir = changed\n"),
             std::string("[deployment]\n[system]\nlog_dir = changed\n"),
             std::string("[deployment]\nprofile =\n[system]\nlog_dir = changed\n"),
             std::string("[deployment]\nprofile = \t\n[system]\nlog_dir = changed\n"),
             std::string("[deployment]\nprofile = ") + cleanup.profile_path +
                 "\nprofile = " + cleanup.profile_path + '\n',
         }) {
        {
            std::ofstream file(path);
            file << content;
            if (!file) return 15;
        }
        if (plane::core::load_config(path, config, &first) || config.plane_id != retained.plane_id ||
            config.log_dir != retained.log_dir || config.fc_udp_url != retained.fc_udp_url ||
            config.midcourse_algorithm.update_dt_s != retained.midcourse_algorithm.update_dt_s) {
            return 16;
        }
    }
    if (!write_profile(uav1)) return 27;
    const auto valid_midcourse_config = config.midcourse_algorithm;
    for (const auto* invalid : {
             "replan_period_s = 0\n",
             "replan_period_s = -1\n",
             "replan_freeze_s = 0\n",
             "replan_freeze_s = -1\n",
         }) {
        if (!write_main(std::string("[midcourse_algorithm]\n") + invalid)) return 29;
        if (plane::core::load_config(path, config, &first) ||
            config.midcourse_algorithm.replan_period_s != valid_midcourse_config.replan_period_s ||
            config.midcourse_algorithm.replan_freeze_s != valid_midcourse_config.replan_freeze_s) {
            return 30;
        }
    }
    for (const auto* invalid : {"0", "-1", "5.5", "8Hz", "", "9999999999999999999"}) {
        RequiredOptions options;
        options.pod_state_report_hz = invalid;
        if (!write_main("", options)) return 17;
        if (plane::core::load_config(path, config, &first) || config.pod_state_report_hz != 8) return 18;
    }
    RequiredOptions tree_options;
    tree_options.tree_update_rate_hz = "10";
    if (!write_main("[midcourse_algorithm]\nv_cruise_m_s = 45.0\ntau_ff_s = 1.5\nr_min_m = 60.0\n"
                    "update_dt_s = 0.1\nenable_internal_terminal_confirmation = true\n"
                    "confirm_boresight_deg = 6.0\nconfirm_rate_deg_s = 7.0\nconfirm_hold_s = 2.5\n", tree_options)) {
        return 19;
    }
    if (!plane::core::load_config(path, config, &first) || config.bt_update_rate_hz != 10 ||
        algorithm.v_cruise_m_s != 45.0 || algorithm.tau_ff_s != 1.5 || algorithm.r_min_m != 60.0 ||
        algorithm.update_dt_s != 0.1 || !algorithm.enable_internal_terminal_confirmation ||
        algorithm.confirm_boresight_deg != 6.0 || algorithm.confirm_rate_deg_s != 7.0 ||
        algorithm.confirm_hold_s != 2.5) {
        return 20;
    }
    for (const auto* invalid : {"0.05", "0", "nan"}) {
        if (!write_main(std::string("[midcourse_algorithm]\nupdate_dt_s = ") + invalid + '\n', tree_options)) {
            return 21;
        }
        if (plane::core::load_config(path, config, &first) || algorithm.update_dt_s != 0.1) return 22;
    }
    tree_options.tree_update_rate_hz = "40";
    if (!write_main("", tree_options)) return 23;
    if (!plane::core::load_config(path, config, &first) || config.bt_update_rate_hz != 40 ||
        config.bt_tick_period_s() != 0.025 || algorithm.update_dt_s != 0.025) {
        return 24;
    }
    for (const auto* invalid : {"0", "-1", "12.5", "20Hz", "", "9999999999999999999"}) {
        RequiredOptions options;
        options.tree_update_rate_hz = invalid;
        if (!write_main("", options)) return 31;
        if (plane::core::load_config(path, config, &first) || config.bt_update_rate_hz != 40 ||
            algorithm.update_dt_s != 0.025) {
            return 32;
        }
    }
    for (const auto member : {&RequiredOptions::pod_state_report_hz, &RequiredOptions::tree_update_rate_hz,
                              &RequiredOptions::target_history_size, &RequiredOptions::target_timeout_ms}) {
        auto options = custom_options;
        options.pod_state_report_hz = "8";
        options.tree_update_rate_hz = "40";
        (options.*member).reset();
        if (!write_main("", options)) return 33;
        plane::core::AppConfig fresh;
        if (plane::core::load_config(path, fresh, &first) || fresh.pod_state_report_hz != 0 ||
            fresh.bt_update_rate_hz != 0 || fresh.midcourse_target_history_size != 0 ||
            fresh.midcourse_target_timeout_ms != 0) {
            return 34;
        }
        if (plane::core::load_config(path, config, &first) || config.pod_state_report_hz != 5 ||
            config.bt_update_rate_hz != 40 || config.midcourse_target_history_size != 20 ||
            config.midcourse_target_timeout_ms != 5000 || algorithm.update_dt_s != 0.025) {
            return 35;
        }
    }
    return 0;
}
