#include "core/system_param_store.h"

#include <spdlog/spdlog.h>

#include "communication/communication_events.h"
#include "core/system_param_events.h"

namespace plane::core {

SystemParamStore::SystemParamStore(EventBus& event_bus, const AppConfig& config)
    : event_bus_(event_bus), param_(config.system_param) {}

SystemParamStore::~SystemParamStore() { stop(); }

std::string SystemParamStore::name() const { return "System Param"; }

bool SystemParamStore::init() { return true; }

bool SystemParamStore::start() {
    event_bus_.subscribe<plane::model::SystemParam>(
        plane::communication::events::kSystemParamReceived,
        [this](const plane::model::SystemParam& param) { on_system_param(param); });
    spdlog::info("系统参数就绪: safe_height_m={}", current().safeHeight_m);
    return true;
}

void SystemParamStore::stop() {}

void SystemParamStore::release() {}

plane::model::SystemParam SystemParamStore::current() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return param_;
}

void SystemParamStore::on_system_param(const plane::model::SystemParam& param) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        param_ = param;
    }
    spdlog::info("系统参数更新: safe_height_m={}", param.safeHeight_m);
    event_bus_.publish(events::kSystemParamChanged, param);
}

}  // namespace plane::core
