#pragma once

namespace plane::core {

/// 打印并校验所有第三方依赖是否可用；全部可用返回 0。
int run_deps_check();

}  // namespace plane::core
