#include "core/build_info.h"

#ifndef PLANE_VERSION
#define PLANE_VERSION "unknown"
#endif
#ifndef PLANE_GIT_HASH
#define PLANE_GIT_HASH "unknown"
#endif
#ifndef PLANE_BUILD_TYPE
#define PLANE_BUILD_TYPE "unknown"
#endif

namespace plane::core {

BuildInfo current_build_info() { return BuildInfo{PLANE_VERSION, PLANE_GIT_HASH, PLANE_BUILD_TYPE}; }

}  // namespace plane::core
