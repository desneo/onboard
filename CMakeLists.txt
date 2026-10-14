cmake_minimum_required(VERSION 3.16)
project(plane LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_definitions(SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)

execute_process(
  COMMAND git rev-parse --short HEAD
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  OUTPUT_VARIABLE PLANE_GIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)

include(cmake/Dependencies.cmake)

add_executable(plane
  src/main.cpp
  src/camera/camera_control_module.cpp
  src/communication/serialization.cpp
  src/communication/zenoh_transport.cpp
  src/communication/communication_module.cpp
  src/core/build_info.cpp
  src/core/config.cpp
  src/core/deps_check.cpp
  src/core/log.cpp
  src/core/placeholder_modules.cpp
  src/core/runtime.cpp
  src/core/system_param_store.cpp
  src/core/time_utils.cpp
  src/state/state_store.cpp
  src/state/state_monitor.cpp
  src/flight_control/flight_control_module.cpp
  src/geo/geo_tool.cpp
  src/task/task_scheduler.cpp
  src/task/nodes/arm_node.cpp
  src/task/nodes/disarm_node.cpp
  src/task/nodes/mission_node.cpp
  src/task/nodes/takeoff_node.cpp
  src/task/nodes/land_node.cpp
  src/task/nodes/rtl_node.cpp
  src/task/nodes/hover_node.cpp
  src/task/nodes/stop_motor_node.cpp
  src/task/nodes/approach_controller.cpp
  src/task/nodes/midcourse_node.cpp
  src/task/nodes/detect_node.cpp
  src/task/task_execution_engine.cpp
  src/perception/plugin_management.cpp
  src/perception/plugin_providers.cpp
  src/perception/perception.cpp
  src/target/target_store.cpp
  src/video/video_streaming.cpp
)
target_include_directories(plane PRIVATE src)
target_compile_options(plane PRIVATE -Wall -Wextra)
target_compile_definitions(plane PRIVATE
  PLANE_VERSION="0.1.0"
  PLANE_GIT_HASH="${PLANE_GIT_HASH}"
  PLANE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
)
target_link_libraries(plane PRIVATE
  plane::common_include
  plane::deps
)
# 可执行文件直接输出到 build/plane/，该目录作为可整体拷贝的独立运行目录。
set_target_properties(plane PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/plane")

add_executable(gcs_sim tools/gcs_sim/main.cpp)
target_include_directories(gcs_sim PRIVATE src)
target_compile_options(gcs_sim PRIVATE -Wall -Wextra)
target_link_libraries(gcs_sim PRIVATE
  plane::common_include
  plane::nlohmann_json
  fmt::fmt
  spdlog::spdlog
  zenohc::lib
  zenohcxx::zenohc
)

# 正式插件暂存：将三个插件 .so 拷贝到 build/plugins（路径可用环境变量覆盖）。
add_custom_target(stage_plugins
  COMMAND bash "${CMAKE_SOURCE_DIR}/tools/stage_plugins.sh" "${CMAKE_BINARY_DIR}"
  COMMENT "暂存正式插件到 build/plugins"
  VERBATIM
)

# 插件加载契约测试（不进入正式运行目录）。
add_executable(plugin_load_check tests/contract/plugin_load_check.cpp)
target_include_directories(plugin_load_check PRIVATE src)
target_compile_options(plugin_load_check PRIVATE -Wall -Wextra)
target_link_libraries(plugin_load_check PRIVATE
  plane::pluma
  plane::common_include
  opencv_core
  fmt::fmt
  spdlog::spdlog
)

# 默认构建即组装 build/plane 独立运行目录：暂存正式插件、复制三方库、改写 rpath。
add_custom_target(package_plane ALL
  COMMAND bash "${CMAKE_SOURCE_DIR}/tools/bundle_plane.sh" "${CMAKE_BINARY_DIR}" "${CMAKE_SOURCE_DIR}"
  DEPENDS plane stage_plugins
  COMMENT "组装 build/plane 独立运行目录（正式插件 + 三方库 + rpath）"
  VERBATIM
)
