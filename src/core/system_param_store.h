#pragma once

#include <mutex>
#include <string>

#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"
#include "core/system_param_api.h"

namespace plane::core {

/// 系统参数运行时存储：启动默认值 + GCS 动态更新（ARCHITECTURE §7）。
class SystemParamStore final : public IModule, public SystemParamApi {
   public:
    SystemParamStore(EventBus& event_bus, const AppConfig& config);
    ~SystemParamStore() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    plane::model::SystemParam current() const override;

   private:
    void on_system_param(const plane::model::SystemParam& param);

    EventBus& event_bus_;
    mutable std::mutex mutex_;
    plane::model::SystemParam param_;
};

}  // namespace plane::core
