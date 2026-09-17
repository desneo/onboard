#include <spdlog/spdlog.h>

#include <cstring>

#include "core/build_info.h"
#include "core/deps_check.h"
#include "core/log.h"

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--deps-check") == 0) {
        return plane::core::run_deps_check();
    }

    plane::core::init_logging({});

    const plane::core::BuildInfo build_info = plane::core::current_build_info();
    spdlog::info("onboard 启动: version={} git={} build={}", build_info.version, build_info.git_hash,
                 build_info.build_type);

    plane::core::shutdown_logging();
    return 0;
}
