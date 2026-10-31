#include "core/placeholder_modules.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <iterator>
#include <string>
#include <utility>

namespace plane::core {
namespace {

class PlaceholderModule final : public IModule {
   public:
    explicit PlaceholderModule(std::string name) : name_(std::move(name)) {}

    std::string name() const override { return name_; }
    bool init() override {
        SPDLOG_INFO("模块初始化完成: {}", name());
        return true;
    }
    bool start() override {
        SPDLOG_INFO("模块启动完成: {}", name());
        return true;
    }
    void stop() override {}
    void release() override {}

   private:
    std::string name_;
};

}  // namespace

std::vector<std::unique_ptr<IModule>> make_placeholder_modules(const std::vector<std::string>& skip) {
    const char* names[] = {"通信",     "无人机状态管理", "状态监控",   "目标管理",
                           "飞行控制", "相机控制",       "任务调度器", "任务执行引擎"};
    std::vector<std::unique_ptr<IModule>> modules;
    modules.reserve(std::size(names));
    for (const char* module_name : names) {
        if (std::find(skip.begin(), skip.end(), module_name) != skip.end()) {
            continue;
        }
        modules.push_back(std::make_unique<PlaceholderModule>(module_name));
    }
    return modules;
}

}  // namespace plane::core
