#include "plugin/plugin_management.h"

#include <spdlog/spdlog.h>

namespace plane::plugin {

std::string PluginManagement::name() const { return "插件管理"; }

bool PluginManagement::init() {
    SPDLOG_INFO("模块初始化完成: {}", name());
    return true;
}

bool PluginManagement::start() {
    SPDLOG_INFO("模块启动完成: {}", name());
    return true;
}

void PluginManagement::stop() {}

void PluginManagement::release() {}

}  // namespace plane::plugin
