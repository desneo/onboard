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
        SPDLOG_ERROR("未找到插件 provider: {}", provider_type);
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
    pluma_.acceptProviderType<plane::algorithm::ImageInputPluginProvider>();
    pluma_.acceptProviderType<plane::algorithm::DetMotPluginProvider>();
    pluma_.acceptProviderType<plane::algorithm::VideoStreamOutputPluginProvider>();

    const int loaded = pluma_.loadFromFolder(config_.plugins_dir);
    loaded_ = true;
    SPDLOG_INFO("插件加载目录={} 成功={}", config_.plugins_dir, loaded);
    SPDLOG_INFO("插件 provider: image_input={}, det_mot={}, video_output={}",
                provider_count<plane::algorithm::ImageInputPluginProvider>(pluma_),
                provider_count<plane::algorithm::DetMotPluginProvider>(pluma_),
                provider_count<plane::algorithm::VideoStreamOutputPluginProvider>(pluma_));
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

std::shared_ptr<plane::algorithm::ImageInputPlugin> PluginManagement::create_image_input() {
    return create_first<plane::algorithm::ImageInputPlugin, plane::algorithm::ImageInputPluginProvider>(
        pluma_, "ImageInputPlugin");
}

std::shared_ptr<plane::algorithm::DetMotPlugin> PluginManagement::create_det_mot() {
    return create_first<plane::algorithm::DetMotPlugin, plane::algorithm::DetMotPluginProvider>(pluma_,
                                                                                                "DetMotPlugin");
}

std::shared_ptr<plane::algorithm::VideoStreamOutputPlugin> PluginManagement::create_video_output() {
    return create_first<plane::algorithm::VideoStreamOutputPlugin, plane::algorithm::VideoStreamOutputPluginProvider>(
        pluma_, "VideoStreamOutputPlugin");
}

}  // namespace plane::perception
