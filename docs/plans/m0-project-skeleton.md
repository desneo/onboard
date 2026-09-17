# M0 任务级计划（工程骨架与依赖接入）

> 状态：已完成并验收（提交 d48569f、a6dec1b、fdb9c98、adeaaa3、81c800d、94bdad3）。
> 来源：`DEV_PLAN.md` 路线图 M0；以下按原计划勾选执行记录。
> 验证原则：x64 Release 构建通过；不写测试代码（SPEC §11）。

---

## 任务级计划

> 执行位置：工程根 `onboard/`。
> 验证原则：每个 Task 结束后 x64 Release 构建通过；不写测试代码（SPEC §11）。
> 提交：每个 Task 一个提交，提交信息用本 Task 给出的原文。

## Task 1：CMake 工程骨架与代码风格

**Files**

- Create: `.clang-format`
- Create: `.gitignore`
- Create: `CMakeLists.txt`
- Create: `src/main.cpp`

**Interfaces**

- Produces: 可执行目标 `onboard`（后续 Task 持续扩展该目标）

- [x] Step 1: 写 `.clang-format`

```yaml
BasedOnStyle: Google
Language: Cpp
Standard: c++17
IndentWidth: 4
ColumnLimit: 120
PointerAlignment: Left
DerivePointerAlignment: false
```

- [x] Step 2: 写 `.gitignore`

```text
build/
logs/
compile_commands.json
.cache/
```

- [x] Step 3: 写 `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.16)
project(onboard LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(onboard
  src/main.cpp
)
target_include_directories(onboard PRIVATE src)
target_compile_options(onboard PRIVATE -Wall -Wextra)
```

- [x] Step 4: 写 `src/main.cpp`

```cpp
int main() { return 0; }
```

（clang-format 14 的 Google 风格会合并短函数，故此处保持单行形式。）

- [x] Step 5: 构建验证

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
test -x build/onboard && echo "binary ok"
./build/onboard; echo "exit=$?"
```

Expected: 构建成功；`binary ok`；`exit=0`。

- [x] Step 6: 格式检查

```bash
clang-format --dry-run --Werror src/main.cpp
```

Expected: 无输出。

- [x] Step 7: 提交

```bash
git add .clang-format .gitignore CMakeLists.txt src/main.cpp
git commit -m "chore(m0): 建立 CMake 工程骨架与代码风格配置"
```

---

## Task 2：三方库与公共头文件接入

**Files**

- Create: `cmake/Dependencies.cmake`
- Create: `src/core/deps_check.h`
- Create: `src/core/deps_check.cpp`
- Modify: `CMakeLists.txt`
- Modify: `src/main.cpp`

**Interfaces**

- Produces: CMake targets `plane::deps`、`plane::common_include`，以及 `plane::behaviortree`、`plane::pluma`、`plane::zlmediakit`、`plane::eventpp`、`plane::magic_enum`、`plane::nlohmann_json`
- Produces: `int plane::core::run_deps_check()`（`--deps-check` 入口）

- [x] Step 1: 写 `cmake/Dependencies.cmake`

```cmake
include_guard(GLOBAL)

set(PLANE_3RDPARTY_DIR "${CMAKE_SOURCE_DIR}/../../../3rdparty/common"
    CACHE PATH "本地三方库根目录（相对工程根 onboard/）")
set(PLANE_COMMON_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../common/common_include"
    CACHE PATH "与外部交互的公共头文件目录（相对工程根 onboard/）")
set(PLANE_PLATFORM_DIR "x64/ubuntu22.04"
    CACHE STRING "三方库平台子目录；arm64 依赖补齐后再启用")

foreach(_dir "${PLANE_3RDPARTY_DIR}" "${PLANE_COMMON_INCLUDE_DIR}")
  if(NOT EXISTS "${_dir}")
    message(FATAL_ERROR "依赖目录不存在: ${_dir}；请按 requirement.md 约定补充后重试")
  endif()
endforeach()

list(PREPEND CMAKE_PREFIX_PATH
  "${PLANE_3RDPARTY_DIR}/fmt/12.2.0/${PLANE_PLATFORM_DIR}"
  "${PLANE_3RDPARTY_DIR}/spdlog/v1.17.0/${PLANE_PLATFORM_DIR}"
  "${PLANE_3RDPARTY_DIR}/geographiclib/v2.7/${PLANE_PLATFORM_DIR}"
  "${PLANE_3RDPARTY_DIR}/mavsdk/v3.17.4/${PLANE_PLATFORM_DIR}"
  "${PLANE_3RDPARTY_DIR}/zenohc/1.10.0/${PLANE_PLATFORM_DIR}"
  "${PLANE_3RDPARTY_DIR}/zenohcpp/1.10.0/${PLANE_PLATFORM_DIR}"
)

find_package(fmt REQUIRED CONFIG)
find_package(spdlog REQUIRED CONFIG)
find_package(GeographicLib REQUIRED CONFIG)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(MAVSDK REQUIRED CONFIG)
find_package(zenohc REQUIRED CONFIG)
find_package(zenohcxx REQUIRED CONFIG)

# 无 CMake config 的库统一封装为 interface target，业务模块只依赖 plane::*。
add_library(plane_behaviortree INTERFACE)
target_include_directories(plane_behaviortree INTERFACE
  "${PLANE_3RDPARTY_DIR}/behaviortree/4.10.0/${PLANE_PLATFORM_DIR}/include")
target_link_libraries(plane_behaviortree INTERFACE
  "${PLANE_3RDPARTY_DIR}/behaviortree/4.10.0/${PLANE_PLATFORM_DIR}/lib/libbehaviortree_cpp.so")

add_library(plane_pluma INTERFACE)
target_include_directories(plane_pluma INTERFACE
  "${PLANE_3RDPARTY_DIR}/pluma/${PLANE_PLATFORM_DIR}/include")
target_link_libraries(plane_pluma INTERFACE
  "${PLANE_3RDPARTY_DIR}/pluma/${PLANE_PLATFORM_DIR}/lib/libpluma.so")

add_library(plane_zlmediakit INTERFACE)
target_include_directories(plane_zlmediakit INTERFACE
  "${PLANE_3RDPARTY_DIR}/zlmediakit/4b07053/${PLANE_PLATFORM_DIR}/include")
target_link_libraries(plane_zlmediakit INTERFACE
  "${PLANE_3RDPARTY_DIR}/zlmediakit/4b07053/${PLANE_PLATFORM_DIR}/lib/libmk_api.so")

add_library(plane_eventpp INTERFACE)
target_include_directories(plane_eventpp INTERFACE
  "${PLANE_3RDPARTY_DIR}/eventcpp/v0.1.3/include")

add_library(plane_magic_enum INTERFACE)
target_include_directories(plane_magic_enum INTERFACE
  "${PLANE_3RDPARTY_DIR}/magic_enum/v0.9.8/include")

add_library(plane_nlohmann_json INTERFACE)
target_include_directories(plane_nlohmann_json INTERFACE
  "${PLANE_3RDPARTY_DIR}/nlohmann_json/v3.12.0/include")

add_library(plane_common_include INTERFACE)
target_include_directories(plane_common_include INTERFACE "${PLANE_COMMON_INCLUDE_DIR}")
target_compile_features(plane_common_include INTERFACE cxx_std_17)

add_library(plane_deps INTERFACE)
target_link_libraries(plane_deps INTERFACE
  fmt::fmt
  spdlog::spdlog
  GeographicLib::GeographicLib
  opencv_core
  MAVSDK::mavsdk
  zenohc::lib
  zenohcxx::zenohc
  plane_behaviortree
  plane_pluma
  plane_zlmediakit
  plane_eventpp
  plane_magic_enum
  plane_nlohmann_json
)

add_library(plane::deps ALIAS plane_deps)
add_library(plane::common_include ALIAS plane_common_include)
add_library(plane::behaviortree ALIAS plane_behaviortree)
add_library(plane::pluma ALIAS plane_pluma)
add_library(plane::zlmediakit ALIAS plane_zlmediakit)
add_library(plane::eventpp ALIAS plane_eventpp)
add_library(plane::magic_enum ALIAS plane_magic_enum)
add_library(plane::nlohmann_json ALIAS plane_nlohmann_json)

foreach(_target fmt::fmt spdlog::spdlog GeographicLib::GeographicLib MAVSDK::mavsdk
                zenohc::lib zenohcxx::zenohc opencv_core)
  if(NOT TARGET ${_target})
    message(FATAL_ERROR "缺少 CMake target: ${_target}")
  endif()
endforeach()
```

- [x] Step 2: 写 `src/core/deps_check.h`

```cpp
#pragma once

namespace plane::core {

/// 打印并校验所有第三方依赖是否可用；全部可用返回 0。
int run_deps_check();

}  // namespace plane::core
```

- [x] Step 3: 写 `src/core/deps_check.cpp`

```cpp
#include "core/deps_check.h"

#include <string>

#include <GeographicLib/Geodesic.hpp>
#include <Pluma/Pluma.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <eventpp/eventdispatcher.h>
#include <fmt/format.h>
#include <magic_enum/magic_enum.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <spdlog/spdlog.h>
#include <zenoh/api.hxx>

#include "mk_mediakit.h"

namespace plane::core {
namespace {

enum class CheckState { Ok, Failed };

}  // namespace

int run_deps_check() {
    const std::string state = std::string(magic_enum::enum_name(CheckState::Ok));
    const std::string json = nlohmann::json::parse(R"({"deps":"ok"})").dump();

    eventpp::EventDispatcher<int, void()> dispatcher;
    dispatcher.appendListener(1, []() {});
    dispatcher.dispatch(1);

    pluma::Pluma pluma;
    static_cast<void>(pluma);

    BT::BehaviorTreeFactory factory;
    static_cast<void>(factory);

    const GeographicLib::Geodesic& geodesic = GeographicLib::Geodesic::WGS84();
    double distance_m = 0.0;
    geodesic.Inverse(0.0, 0.0, 0.0, 0.001, distance_m);

    zenoh::Config zenoh_config = zenoh::Config::create_default();
    static_cast<void>(zenoh_config);

    const cv::Mat empty_frame;
    static_cast<void>(empty_frame);

    char* exe_dir = mk_util_get_exe_dir("");
    if (exe_dir != nullptr) {
        mk_free(exe_dir);
    }

    spdlog::info(
        "deps-check: magic_enum={} nlohmann={} fmt={} spdlog={} bt={} geodesic={:.1f}m cv={}",
        state, NLOHMANN_JSON_VERSION_MAJOR, FMT_VERSION, SPDLOG_VERSION, BT::LibraryVersionString(),
        distance_m, CV_VERSION);
    return 0;
}

}  // namespace plane::core
```

- [x] Step 4: 更新 `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.16)
project(onboard LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(cmake/Dependencies.cmake)

add_executable(onboard
  src/main.cpp
  src/core/deps_check.cpp
)
target_include_directories(onboard PRIVATE src)
target_compile_options(onboard PRIVATE -Wall -Wextra)
target_link_libraries(onboard PRIVATE plane::common_include plane::deps)
```

- [x] Step 5: 更新 `src/main.cpp`

```cpp
#include <cstring>

#include "core/deps_check.h"

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--deps-check") == 0) {
        return plane::core::run_deps_check();
    }
    return 0;
}
```

- [x] Step 6: 配置并构建

```bash
rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Expected: 配置成功，输出中可见 `zenohcxx::zenohc` 定义提示；构建成功。
若 `find_package(MAVSDK)` 失败：**停止执行**，把缺失的依赖包清单（CURL/jsoncpp/picosha2/tinyxml2/mav/LibEvents/LibLZMA 等）报告给用户补充，不要移除或替换该依赖。

- [x] Step 7: 运行依赖检查

```bash
./build/onboard --deps-check; echo "exit=$?"
ldd build/onboard | grep -E "behaviortree|pluma|mk_api|zenoh|mavsdk|GeographicLib"
```

Expected: 输出包含 `bt=4.10.0`、`cv=4.5.4`（以系统实际版本为准）等字段，`exit=0`；`ldd` 能解析到对应 `.so`。

- [x] Step 8: 提交

```bash
git add cmake/Dependencies.cmake src/core/deps_check.h src/core/deps_check.cpp CMakeLists.txt src/main.cpp
git commit -m "chore(m0): 接入三方库与公共头文件依赖"
```

---

## Task 3：日志封装与 Build Info

**Files**

- Create: `src/core/build_info.h`
- Create: `src/core/build_info.cpp`
- Create: `src/core/log.h`
- Create: `src/core/log.cpp`
- Modify: `CMakeLists.txt`
- Modify: `src/main.cpp`

**Interfaces**

- Consumes: `plane::core::run_deps_check()`（Task 2）
- Produces: `plane::core::BuildInfo plane::core::current_build_info()`
- Produces: `void plane::core::init_logging(const plane::core::LogOptions&)`、`void plane::core::shutdown_logging()`

- [x] Step 1: 写 `src/core/build_info.h`

```cpp
#pragma once

#include <string>

namespace plane::core {

/// 构建版本信息，启动时输出（ARCHITECTURE §28.3）。
struct BuildInfo {
    std::string version;
    std::string git_hash;
    std::string build_type;
};

/// 返回当前程序的构建信息，取值来自 CMake 注入的宏。
BuildInfo current_build_info();

}  // namespace plane::core
```

- [x] Step 2: 写 `src/core/build_info.cpp`

```cpp
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

BuildInfo current_build_info() {
    return BuildInfo{PLANE_VERSION, PLANE_GIT_HASH, PLANE_BUILD_TYPE};
}

}  // namespace plane::core
```

- [x] Step 3: 写 `src/core/log.h`

```cpp
#pragma once

#include <cstddef>
#include <string>

namespace plane::core {

/// 日志配置；默认单文件 50 MB、保留 5 个历史文件（ARCHITECTURE §28.3）。
struct LogOptions {
    std::string app_name = "onboard";
    std::string log_dir = "logs";
    std::size_t max_file_size_bytes = 50 * 1024 * 1024;
    std::size_t max_files = 5;
};

/// 初始化全局日志；项目代码统一通过 spdlog 输出。
void init_logging(const LogOptions& options);

/// 刷盘并释放全局日志。
void shutdown_logging();

}  // namespace plane::core
```

- [x] Step 4: 写 `src/core/log.cpp`

```cpp
#include "core/log.h"

#include <filesystem>
#include <memory>
#include <vector>

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace plane::core {

void init_logging(const LogOptions& options) {
    std::filesystem::create_directories(options.log_dir);

    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        options.log_dir + "/" + options.app_name + ".log", options.max_file_size_bytes,
        options.max_files));

    auto logger = std::make_shared<spdlog::logger>(options.app_name, sinks.begin(), sinks.end());
    logger->set_level(spdlog::level::info);
    logger->flush_on(spdlog::level::warn);
    spdlog::set_default_logger(logger);
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%t] %v");
}

void shutdown_logging() {
    spdlog::shutdown();
}

}  // namespace plane::core
```

- [x] Step 5: 更新 `CMakeLists.txt` 为完整内容

```cmake
cmake_minimum_required(VERSION 3.16)
project(onboard LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

execute_process(
  COMMAND git rev-parse --short HEAD
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  OUTPUT_VARIABLE PLANE_GIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)

include(cmake/Dependencies.cmake)

add_executable(onboard
  src/main.cpp
  src/core/build_info.cpp
  src/core/deps_check.cpp
  src/core/log.cpp
)
target_include_directories(onboard PRIVATE src)
target_compile_options(onboard PRIVATE -Wall -Wextra)
target_compile_definitions(onboard PRIVATE
  PLANE_VERSION="0.1.0"
  PLANE_GIT_HASH="${PLANE_GIT_HASH}"
  PLANE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
)
target_link_libraries(onboard PRIVATE plane::common_include plane::deps)
```

- [x] Step 6: 更新 `src/main.cpp`

```cpp
#include <cstring>

#include <spdlog/spdlog.h>

#include "core/build_info.h"
#include "core/deps_check.h"
#include "core/log.h"

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--deps-check") == 0) {
        return plane::core::run_deps_check();
    }

    plane::core::init_logging({});

    const plane::core::BuildInfo build_info = plane::core::current_build_info();
    spdlog::info("onboard 启动: version={} git={} build={}", build_info.version,
                 build_info.git_hash, build_info.build_type);

    plane::core::shutdown_logging();
    return 0;
}
```

- [x] Step 7: 构建并运行

```bash
cmake --build build -j
./build/onboard; echo "exit=$?"
ls -l logs/onboard.log
tail -n 3 logs/onboard.log
```

Expected: stdout 与 `logs/onboard.log` 均包含 `onboard 启动: version=0.1.0 ...`，`exit=0`。

- [x] Step 8: 提交

```bash
git add src/core/build_info.h src/core/build_info.cpp src/core/log.h src/core/log.cpp CMakeLists.txt src/main.cpp
git commit -m "feat(m0): 增加日志封装与构建信息输出"
```

---

## Task 4：配置加载

**Files**

- Create: `config/onboard.conf`
- Create: `src/core/config.h`
- Create: `src/core/config.cpp`
- Modify: `CMakeLists.txt`
- Modify: `src/main.cpp`

**Interfaces**

- Consumes: `plane::core::init_logging`（Task 3）、`plane::model::SystemParam`（common_include）
- Produces: `bool plane::core::load_config(const std::string& path, plane::core::AppConfig& out)`

- [x] Step 1: 写 `config/onboard.conf`

```text
# 机载程序运行配置（第一版）
# 默认飞行安全高度（米）
safe_height_m = 30.0
# 日志目录
log_dir = logs
# 状态上报周期（毫秒）
state_report_period_ms = 1000
```

- [x] Step 2: 写 `src/core/config.h`

```cpp
#pragma once

#include <string>

#include "message.h"

namespace plane::core {

/// 运行配置；第一版系统参数与 SystemParam 对齐（ARCHITECTURE §7）。
struct AppConfig {
    plane::model::SystemParam system_param;
    std::string log_dir = "logs";
    int state_report_period_ms = 1000;
};

/// 从 key = value 文本文件加载配置；文件不存在时保留默认值并返回 false。
bool load_config(const std::string& path, AppConfig& out);

}  // namespace plane::core
```

- [x] Step 3: 写 `src/core/config.cpp`

```cpp
#include "core/config.h"

#include <fstream>
#include <sstream>

#include <spdlog/spdlog.h>

namespace plane::core {
namespace {

std::string trim(const std::string& text) {
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

}  // namespace

bool load_config(const std::string& path, AppConfig& out) {
    std::ifstream file(path);
    if (!file.is_open()) {
        spdlog::warn("配置文件不存在，使用默认配置: {}", path);
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        line = trim(line);
        if (line.empty() || line.front() == '#') {
            continue;
        }
        const auto pos = line.find('=');
        if (pos == std::string::npos) {
            spdlog::warn("忽略无法解析的配置行: {}", line);
            continue;
        }
        const std::string key = trim(line.substr(0, pos));
        const std::string value = trim(line.substr(pos + 1));
        if (key == "safe_height_m") {
            out.system_param.safeHeight_m = std::stod(value);
        } else if (key == "log_dir") {
            out.log_dir = value;
        } else if (key == "state_report_period_ms") {
            out.state_report_period_ms = std::stoi(value);
        } else {
            spdlog::warn("未知配置项: {}", key);
        }
    }
    return true;
}

}  // namespace plane::core
```

- [x] Step 4: 更新 `CMakeLists.txt` 为完整内容

```cmake
cmake_minimum_required(VERSION 3.16)
project(onboard LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

execute_process(
  COMMAND git rev-parse --short HEAD
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  OUTPUT_VARIABLE PLANE_GIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)

include(cmake/Dependencies.cmake)

add_executable(onboard
  src/main.cpp
  src/core/build_info.cpp
  src/core/config.cpp
  src/core/deps_check.cpp
  src/core/log.cpp
)
target_include_directories(onboard PRIVATE src)
target_compile_options(onboard PRIVATE -Wall -Wextra)
target_compile_definitions(onboard PRIVATE
  PLANE_VERSION="0.1.0"
  PLANE_GIT_HASH="${PLANE_GIT_HASH}"
  PLANE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
)
target_link_libraries(onboard PRIVATE plane::common_include plane::deps)
```

- [x] Step 5: 更新 `src/main.cpp`

```cpp
#include <string>

#include <spdlog/spdlog.h>

#include "core/build_info.h"
#include "core/config.h"
#include "core/deps_check.h"
#include "core/log.h"

int main(int argc, char** argv) {
    std::string config_path = "config/onboard.conf";
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--deps-check") {
            return plane::core::run_deps_check();
        }
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        }
    }

    plane::core::AppConfig config;
    plane::core::load_config(config_path, config);

    plane::core::LogOptions log_options;
    log_options.log_dir = config.log_dir;
    plane::core::init_logging(log_options);

    const plane::core::BuildInfo build_info = plane::core::current_build_info();
    spdlog::info("onboard 启动: version={} git={} build={}", build_info.version,
                 build_info.git_hash, build_info.build_type);
    spdlog::info("配置: safe_height_m={} state_report_period_ms={}",
                 config.system_param.safeHeight_m, config.state_report_period_ms);

    plane::core::shutdown_logging();
    return 0;
}
```

- [x] Step 6: 构建并运行

```bash
cmake --build build -j
./build/onboard; echo "exit=$?"
./build/onboard --config config/not_exist.conf; echo "fallback_exit=$?"
```

Expected: 第一条输出 `配置: safe_height_m=30 state_report_period_ms=1000`；第二条出现 `配置文件不存在，使用默认配置` 且仍 `exit=0`。

- [x] Step 7: 提交

```bash
git add config/onboard.conf src/core/config.h src/core/config.cpp CMakeLists.txt src/main.cpp
git commit -m "feat(m0): 增加 key=value 配置加载"
```

---

## Task 5：Core Runtime 生命周期与模块占位

**Files**

- Create: `src/core/module.h`
- Create: `src/core/runtime.h`
- Create: `src/core/runtime.cpp`
- Create: `src/core/placeholder_modules.h`
- Create: `src/core/placeholder_modules.cpp`
- Modify: `CMakeLists.txt`
- Modify: `src/main.cpp`

**Interfaces**

- Consumes: `plane::core::AppConfig`、日志接口（Task 3/4）
- Produces: `plane::core::IModule`（`name/init/start/stop/release`）
- Produces: `plane::core::Runtime`（`add_module`、`run`、`shutdown`）
- Produces: `std::vector<std::unique_ptr<IModule>> plane::core::make_placeholder_modules()`（后续里程碑用真实模块替换）

- [x] Step 1: 写 `src/core/module.h`

```cpp
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
```

- [x] Step 2: 写 `src/core/runtime.h`

```cpp
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
```

- [x] Step 3: 写 `src/core/runtime.cpp`

```cpp
#include "core/runtime.h"

#include <cstddef>
#include <utility>

#include <spdlog/spdlog.h>

namespace plane::core {

void Runtime::add_module(std::unique_ptr<IModule> module) {
    modules_.push_back(std::move(module));
}

bool Runtime::run() {
    for (std::size_t i = 0; i < modules_.size(); ++i) {
        if (!modules_[i]->init()) {
            spdlog::error("模块 init 失败: {}", modules_[i]->name());
            shutdown();
            return false;
        }
        spdlog::info("模块 init: {}", modules_[i]->name());
    }
    for (auto& module : modules_) {
        if (!module->start()) {
            spdlog::error("模块 start 失败: {}", module->name());
            shutdown();
            return false;
        }
        spdlog::info("模块 start: {}", module->name());
    }
    spdlog::info("系统就绪");
    return true;
}

void Runtime::shutdown() {
    for (auto it = modules_.rbegin(); it != modules_.rend(); ++it) {
        (*it)->stop();
        (*it)->release();
        spdlog::info("模块停止: {}", (*it)->name());
    }
    modules_.clear();
}

}  // namespace plane::core
```

- [x] Step 4: 写 `src/core/placeholder_modules.h`

```cpp
#pragma once

#include <memory>
#include <vector>

#include "core/module.h"

namespace plane::core {

/// M0 占位模块：仅输出生命周期日志，后续里程碑逐个替换为真实模块。
std::vector<std::unique_ptr<IModule>> make_placeholder_modules();

}  // namespace plane::core
```

- [x] Step 5: 写 `src/core/placeholder_modules.cpp`

```cpp
#include "core/placeholder_modules.h"

#include <iterator>
#include <string>
#include <utility>

namespace plane::core {
namespace {

class PlaceholderModule final : public IModule {
   public:
    explicit PlaceholderModule(std::string name) : name_(std::move(name)) {}

    std::string name() const override { return name_; }
    bool init() override { return true; }
    bool start() override { return true; }
    void stop() override {}
    void release() override {}

   private:
    std::string name_;
};

}  // namespace

std::vector<std::unique_ptr<IModule>> make_placeholder_modules() {
    const char* names[] = {
        "Plugin Management",    "Communication",    "UAV State Management",
        "StateMonitor",         "Target Management", "Flight Control",
        "Camera Control",       "Perception",       "Video Streaming",
        "Task Scheduler",       "Task Execution Engine",
    };
    std::vector<std::unique_ptr<IModule>> modules;
    modules.reserve(std::size(names));
    for (const char* module_name : names) {
        modules.push_back(std::make_unique<PlaceholderModule>(module_name));
    }
    return modules;
}

}  // namespace plane::core
```

- [x] Step 6: 更新 `CMakeLists.txt` 为完整内容

```cmake
cmake_minimum_required(VERSION 3.16)
project(onboard LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

execute_process(
  COMMAND git rev-parse --short HEAD
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  OUTPUT_VARIABLE PLANE_GIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)

include(cmake/Dependencies.cmake)

add_executable(onboard
  src/main.cpp
  src/core/build_info.cpp
  src/core/config.cpp
  src/core/deps_check.cpp
  src/core/log.cpp
  src/core/placeholder_modules.cpp
  src/core/runtime.cpp
)
target_include_directories(onboard PRIVATE src)
target_compile_options(onboard PRIVATE -Wall -Wextra)
target_compile_definitions(onboard PRIVATE
  PLANE_VERSION="0.1.0"
  PLANE_GIT_HASH="${PLANE_GIT_HASH}"
  PLANE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
)
target_link_libraries(onboard PRIVATE plane::common_include plane::deps)
```

- [x] Step 7: 更新 `src/main.cpp`

```cpp
#include <string>
#include <utility>

#include <spdlog/spdlog.h>

#include "core/build_info.h"
#include "core/config.h"
#include "core/deps_check.h"
#include "core/log.h"
#include "core/placeholder_modules.h"
#include "core/runtime.h"

int main(int argc, char** argv) {
    std::string config_path = "config/onboard.conf";
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--deps-check") {
            return plane::core::run_deps_check();
        }
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        }
    }

    plane::core::AppConfig config;
    plane::core::load_config(config_path, config);

    plane::core::LogOptions log_options;
    log_options.log_dir = config.log_dir;
    plane::core::init_logging(log_options);

    const plane::core::BuildInfo build_info = plane::core::current_build_info();
    spdlog::info("onboard 启动: version={} git={} build={}", build_info.version,
                 build_info.git_hash, build_info.build_type);
    spdlog::info("配置: safe_height_m={} state_report_period_ms={}",
                 config.system_param.safeHeight_m, config.state_report_period_ms);

    plane::core::Runtime runtime;
    for (auto& module : plane::core::make_placeholder_modules()) {
        runtime.add_module(std::move(module));
    }
    if (!runtime.run()) {
        plane::core::shutdown_logging();
        return 1;
    }
    runtime.shutdown();

    plane::core::shutdown_logging();
    return 0;
}
```

- [x] Step 8: 构建并运行

```bash
cmake --build build -j
./build/onboard; echo "exit=$?"
```

Expected: 依次输出 11 个 `模块 init`、11 个 `模块 start`、`系统就绪`、11 个 `模块停止`，`exit=0`。

- [x] Step 9: 提交

```bash
git add src/core/module.h src/core/runtime.h src/core/runtime.cpp \
        src/core/placeholder_modules.h src/core/placeholder_modules.cpp CMakeLists.txt src/main.cpp
git commit -m "feat(m0): 增加 Core Runtime 生命周期与模块占位"
```

---

## Task 6：M0 验收与收尾

**Files**

- Modify: 如有格式问题，由 clang-format 修正
- Modify: `docs/DEV_PLAN.md`（勾选状态，可选）

- [x] Step 1: 干净构建

```bash
rm -rf build logs
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Expected: 无错误；无本项目代码引发的警告。

- [x] Step 2: 功能验证

```bash
./build/onboard --deps-check
./build/onboard
```

Expected: Task 2/5 的预期输出全部满足。

- [x] Step 3: 代码规范

```bash
clang-format --dry-run --Werror $(git ls-files src)
grep -rn "std::cout\|printf(" src/ || echo "no cout/printf"
```

Expected: 无格式错误；`no cout/printf`。若有格式错误，执行 `clang-format -i $(git ls-files src)` 后重新构建。

- [x] Step 4: DoD 走查

逐条核对 §10 的 5 条完成定义，并把结果（通过/偏离）报告给用户。

- [x] Step 5: 收尾提交（仅当 Step 3 产生格式或代码改动）

```bash
git status --short
git add src CMakeLists.txt .clang-format .gitignore config cmake
git commit -m "chore(m0): M0 收尾与格式检查"
```

文档（`docs/`、`AGENTS.MD`）由用户单独管理，不使用 `git add -A` 一并提交。

---

## 完成后的文档动作

M0 不改变 SPEC 与 ARCHITECTURE 的内容，预期无需文档更新。若实现中发现依赖路径、生命周期顺序需要调整，先按 AGENTS.md 提醒用户确认，再更新架构文档。
