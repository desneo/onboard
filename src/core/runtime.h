#pragma once

#include <memory>
#include <string>
#include <vector>

#include "core/module.h"

namespace plane::core {

/// 按注册顺序 init/start，按逆序 stop/release（ARCHITECTURE §25）。
class Runtime {
   public:
    void add_module(std::unique_ptr<IModule> module);
    bool run();
    void shutdown();

   private:
    std::vector<std::unique_ptr<IModule>> modules_;
};

/// 请求致命退出：工作线程在插件加载/执行失败时调用，main 据此有序关闭并以非零退出。
/// 日志固定输出中文严重级别、模块和原因，供验证脚本断言。
void request_fatal(const std::string& plugin, const std::string& reason);
bool fatal_requested();
std::string fatal_reason();

}  // namespace plane::core
