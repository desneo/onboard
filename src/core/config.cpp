#include "core/config.h"

#include <spdlog/spdlog.h>

#include <fstream>
#include <sstream>

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

}  // namespace

bool load_config(const std::string& path, AppConfig& out) {
    std::ifstream file(path);
    if (!file.is_open()) {
        spdlog::warn("配置文件不存在，使用默认配置: {}", path);
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
            spdlog::warn("忽略无法解析的配置行: {}", line);
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
        } else {
            spdlog::warn("未知配置项: {}", key);
        }
    }
    return true;
}

}  // namespace plane::core
