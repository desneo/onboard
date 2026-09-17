#pragma once

#include <memory>
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

}  // namespace plane::core
