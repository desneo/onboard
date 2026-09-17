#include <cstring>

#include "core/deps_check.h"

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--deps-check") == 0) {
        return plane::core::run_deps_check();
    }
    return 0;
}
