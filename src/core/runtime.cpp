#include "core/runtime.h"

#include <spdlog/spdlog.h>

#include <cstddef>
#include <mutex>
#include <utility>

namespace plane::core {
namespace {

std::mutex g_fatal_mutex;
bool g_fatal_requested = false;
std::string g_fatal_reason;

}  // namespace

void request_fatal(const std::string& plugin, const std::string& reason) {
    {
        std::lock_guard<std::mutex> lock(g_fatal_mutex);
        g_fatal_requested = true;
        if (g_fatal_reason.empty()) {
            g_fatal_reason = reason;
        }
    }
    SPDLOG_ERROR("[fatal] plugin={} reason={}", plugin, reason);
}

bool fatal_requested() {
    std::lock_guard<std::mutex> lock(g_fatal_mutex);
    return g_fatal_requested;
}

std::string fatal_reason() {
    std::lock_guard<std::mutex> lock(g_fatal_mutex);
    return g_fatal_reason;
}

void Runtime::add_module(std::unique_ptr<IModule> module) { modules_.push_back(std::move(module)); }

bool Runtime::run() {
    for (std::size_t i = 0; i < modules_.size(); ++i) {
        if (!modules_[i]->init()) {
            SPDLOG_ERROR("模块 init 失败: {}", modules_[i]->name());
            shutdown();
            return false;
        }
        SPDLOG_INFO("模块 init: {}", modules_[i]->name());
    }
    for (auto& module : modules_) {
        if (!module->start()) {
            SPDLOG_ERROR("模块 start 失败: {}", module->name());
            shutdown();
            return false;
        }
        SPDLOG_INFO("模块 start: {}", module->name());
    }
    SPDLOG_INFO("系统就绪");
    return true;
}

void Runtime::shutdown() {
    for (auto it = modules_.rbegin(); it != modules_.rend(); ++it) {
        (*it)->stop();
        (*it)->release();
        SPDLOG_INFO("模块停止: {}", (*it)->name());
    }
    modules_.clear();
}

}  // namespace plane::core
