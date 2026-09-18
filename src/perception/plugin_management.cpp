#include "perception/plugin_management.h"

#include <spdlog/spdlog.h>

#include <vector>

namespace plane::perception {
namespace {

template <typename PluginType, typename ProviderType>
std::shared_ptr<PluginType> create_first(pluma::Pluma& pluma, const char* provider_type) {
    std::vector<ProviderType*> providers;
    pluma.getProviders(providers);
    if (providers.empty()) {
        spdlog::warn("未找到插件 provider: {}", provider_type);
        return nullptr;
    }
    return std::shared_ptr<PluginType>(providers.front()->create());
}

template <typename ProviderType>
std::size_t provider_count(pluma::Pluma& pluma) {
    std::vector<ProviderType*> providers;
    pluma.getProviders(providers);
    return providers.size();
}

}  // namespace

PluginManagement::PluginManagement(const plane::core::AppConfig& config) : config_(config) {}

PluginManagement::~PluginManagement() { stop(); }

std::string PluginManagement::name() const { return "Plugin Management"; }

bool PluginManagement::init() { return true; }

bool PluginManagement::start() {
    pluma_.acceptProviderType<plane::algorithm::DetectionPluginProvider>();
    pluma_.acceptProviderType<plane::algorithm::TrackingPluginProvider>();
    pluma_.acceptProviderType<plane::algorithm::GuidancePluginProvider>();

    const int loaded = pluma_.loadFromFolder(config_.plugins_dir);
    loaded_ = true;
    spdlog::info("插件目录加载: {} (成功加载 {} 个库)", config_.plugins_dir, loaded);
    spdlog::info("插件 provider: detection={}, tracking={}, guidance={}",
                 provider_count<plane::algorithm::DetectionPluginProvider>(pluma_),
                 provider_count<plane::algorithm::TrackingPluginProvider>(pluma_),
                 provider_count<plane::algorithm::GuidancePluginProvider>(pluma_));
    return true;
}

void PluginManagement::stop() {
    if (!loaded_) {
        return;
    }
    pluma_.unloadAll();
    loaded_ = false;
}

void PluginManagement::release() {}

std::shared_ptr<plane::algorithm::DetectionPlugin> PluginManagement::create_detection() {
    return create_first<plane::algorithm::DetectionPlugin, plane::algorithm::DetectionPluginProvider>(
        pluma_, "DetectionPlugin");
}

std::shared_ptr<plane::algorithm::TrackingPlugin> PluginManagement::create_tracking() {
    return create_first<plane::algorithm::TrackingPlugin, plane::algorithm::TrackingPluginProvider>(pluma_,
                                                                                                    "TrackingPlugin");
}

std::shared_ptr<plane::algorithm::GuidancePlugin> PluginManagement::create_guidance() {
    return create_first<plane::algorithm::GuidancePlugin, plane::algorithm::GuidancePluginProvider>(pluma_,
                                                                                                    "GuidancePlugin");
}

}  // namespace plane::perception
