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
        {"udp", "udpout://127.0.0.1:14580", true},
        {"udp", "udpout://127.0.0.1:14583", true},
        {"udp", "udpin://:14589", true},
        {"udp", "udpout://127.0.0.1:14579", false},
        {"udp", "udpout://127.0.0.1:14590", false},
        {"udp", "udpout://127.0.0.1:14280", false},
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
    return 0;
}
