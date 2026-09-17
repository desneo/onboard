#pragma once

#include <memory>
#include <vector>

#include "core/module.h"

namespace plane::core {

/// M0 占位模块：仅输出生命周期日志，后续里程碑逐个替换为真实模块。
std::vector<std::unique_ptr<IModule>> make_placeholder_modules();

}  // namespace plane::core
