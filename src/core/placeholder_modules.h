#pragma once

#include <memory>
#include <string>
#include <vector>

#include "core/module.h"

namespace plane::core {

/// M0 占位模块：仅输出生命周期日志，后续里程碑逐个替换为真实模块。
/// skip 中列出的名称不生成占位（由真实模块替代）。
std::vector<std::unique_ptr<IModule>> make_placeholder_modules(const std::vector<std::string>& skip = {});

}  // namespace plane::core
