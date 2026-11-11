#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include "core/config.h"

int main() {
    char path[] = "/tmp/plane-config-simulation-XXXXXX";
    const int fd = mkstemp(path);
    if (fd < 0) return 1;
    close(fd);
    struct Cleanup {
        const char* path;
        ~Cleanup() { std::filesystem::remove(path); }
    } cleanup{path};

    struct Case {
        const char* link_type;
        const char* url;
        bool expected;
    };
    const Case cases[] = {
        {"udp", "udpin://127.0.0.1:14030", true},
        {"udp", "udpin://:14039", true},
        {"udp", "udpin://127.0.0.1:14029", false},
        {"udp", "udpin://127.0.0.1:14040", false},
        {"udp", "udpout://127.0.0.1:14280", true},
        {"udp", "udpin://:14289", true},
        {"udp", "udpout://127.0.0.1:14279", false},
        {"udp", "udpout://127.0.0.1:14290", false},
        {"udp", "udpout://127.0.0.1:14580", true},
        {"udp", "udpout://127.0.0.1:14583", true},
        {"udp", "udpin://:14589", true},
        {"udp", "udpout://127.0.0.1:14579", false},
        {"udp", "udpout://127.0.0.1:14590", false},
        {"serial", "udpin://127.0.0.1:14030", false},
        {"serial", "udpout://127.0.0.1:14580", false},
        {"udp", "udpout://127.0.0.1:invalid", false},
    };
    for (const auto& test : cases) {
        {
            std::ofstream file(path);
            file << "[flight_control]\nlink_type = " << test.link_type << "\nudp_url = " << test.url << '\n';
            if (!file) return 2;
        }
        plane::core::AppConfig config;
        config.is_sim = !test.expected;
        if (!plane::core::load_config(path, config) || config.is_sim != test.expected) return 3;
    }

    {
        std::ofstream file(path);
        file << "[midcourse]\ninit_timeout_ms = 12345\ntarget_invalid_grace_s = 2.0\nr_min_m = 75.0\n"
                "[midcourse_algorithm]\nuse_px4_limits = true\nmax_horizontal_speed_m_s = 30.0\n"
                "max_climb_speed_m_s = 8.0\nmax_descent_speed_m_s = 4.0\ncmd_accel_max_m_s2 = 7.0\n"
                "control_output_mode = Attitude\nyaw_mode = 0\n";
        if (!file) return 4;
    }
    plane::core::AppConfig config;
    if (!plane::core::load_config(path, config)) return 5;
    const auto& algorithm = config.midcourse_algorithm;
    if (config.midcourse_init_timeout_ms != 12345 || !config.midcourse_use_px4_limits ||
        algorithm.max_horizontal_speed_m_s != 30.0 || algorithm.max_climb_speed_m_s != 8.0 ||
        algorithm.max_descent_speed_m_s != 4.0 || algorithm.cmd_accel_max_m_s2 != 7.0 ||
        algorithm.target_invalid_grace_s != 2.0 || algorithm.r_min_m != 75.0 ||
        algorithm.control_output_mode != plane::algorithm::MidcourseControlMode::Attitude ||
        algorithm.yaw_mode != 0) {
        return 6;
    }
    return 0;
}
