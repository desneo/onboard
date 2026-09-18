#pragma once

#include <Pluma/Pluma.hpp>
#include <memory>
#include <string>

#include "core/config.h"
#include "core/module.h"
#include "perception/plugin_manager_api.h"

namespace plane::perception {

/// Plugin Management：插件库加载与对象按需实例化（ARCHITECTURE §19）。
class PluginManagement final : public plane::core::IModule, public PluginManagerApi {
   public:
    explicit PluginManagement(const plane::core::AppConfig& config);
    ~PluginManagement() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    std::shared_ptr<plane::algorithm::DetectionPlugin> create_detection() override;
    std::shared_ptr<plane::algorithm::TrackingPlugin> create_tracking() override;
    std::shared_ptr<plane::algorithm::GuidancePlugin> create_guidance() override;

   private:
    const plane::core::AppConfig& config_;
    pluma::Pluma pluma_;
    bool loaded_ = false;
};

}  // namespace plane::perception
