#pragma once

#include "core/module.h"

namespace plane::plugin {

/// 通用插件管理模块，当前保留空实现，供后续插件能力扩展。
class PluginManagement final : public plane::core::IModule {
   public:
    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;
};

}  // namespace plane::plugin
