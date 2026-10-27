#include "plugin/plugin_management.h"

namespace plane::plugin {

std::string PluginManagement::name() const { return "Plugin Management"; }

bool PluginManagement::init() { return true; }

bool PluginManagement::start() { return true; }

void PluginManagement::stop() {}

void PluginManagement::release() {}

}  // namespace plane::plugin
