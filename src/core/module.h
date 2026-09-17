#pragma once

#include <string>

namespace plane::core {

/// Capability 模块统一生命周期接口（ARCHITECTURE §25）。
class IModule {
   public:
    virtual ~IModule() = default;

    virtual std::string name() const = 0;
    virtual bool init() = 0;
    virtual bool start() = 0;
    virtual void stop() = 0;
    virtual void release() = 0;
};

}  // namespace plane::core
