#pragma once

#include "message.h"

namespace plane::core {

/// 运行时系统参数查询接口（ARCHITECTURE §7）。
class SystemParamApi {
   public:
    virtual ~SystemParamApi() = default;

    virtual plane::model::SystemParam current() const = 0;
};

}  // namespace plane::core
