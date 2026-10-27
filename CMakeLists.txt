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
  src/plugin/plugin_management.cpp
  src/state/state_store.cpp
  src/state/state_monitor.cpp
  src/flight_control/flight_control_module.cpp
  src/geo/geo_tool.cpp
  src/task/task_scheduler.cpp
  src/task/command_tree_plan.cpp
  src/task/nodes/arm_node.cpp
  src/task/nodes/adjust_height_node.cpp
  src/task/nodes/disarm_node.cpp
  src/task/nodes/mission_node.cpp
  src/task/nodes/takeoff_node.cpp
  src/task/nodes/land_node.cpp
  src/task/nodes/rtl_node.cpp
  src/task/nodes/hover_node.cpp
  src/task/nodes/stop_motor_node.cpp
  src/task/nodes/approach_controller.cpp
  src/task/nodes/midcourse_node.cpp
  src/task/task_execution_engine.cpp
  src/target/target_store.cpp
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

include(CTest)
if(BUILD_TESTING)
  add_executable(config_simulation_test tests/unit/config_simulation_test.cpp src/core/config.cpp)
  target_include_directories(config_simulation_test PRIVATE src)
  target_compile_options(config_simulation_test PRIVATE -Wall -Wextra)
  target_link_libraries(config_simulation_test PRIVATE plane::common_include spdlog::spdlog)
  add_test(NAME config_simulation COMMAND config_simulation_test)

  add_executable(camera_target_test
    tests/unit/camera_target_test.cpp
    src/camera/camera_control_module.cpp
    src/target/target_store.cpp
  )
  target_include_directories(camera_target_test PRIVATE src)
  target_compile_options(camera_target_test PRIVATE -Wall -Wextra)
  target_link_libraries(camera_target_test PRIVATE plane::common_include plane::eventpp spdlog::spdlog)
  add_test(NAME camera_target COMMAND camera_target_test)

  add_executable(serialization_vehicle_state_test
    tests/unit/serialization_vehicle_state_test.cpp
    src/communication/serialization.cpp
  )
  target_include_directories(serialization_vehicle_state_test PRIVATE src)
  target_compile_options(serialization_vehicle_state_test PRIVATE -Wall -Wextra)
  target_link_libraries(serialization_vehicle_state_test PRIVATE
    plane::common_include
    plane::magic_enum
    plane::nlohmann_json
  )
  add_test(NAME serialization_vehicle_state COMMAND serialization_vehicle_state_test)

  add_executable(mission_node_timeout_test
    tests/unit/mission_node_timeout_test.cpp
    src/task/nodes/mission_node.cpp
  )
  target_include_directories(mission_node_timeout_test PRIVATE src)
  target_compile_options(mission_node_timeout_test PRIVATE -Wall -Wextra)
  target_link_libraries(mission_node_timeout_test PRIVATE
    plane::common_include
    plane::behaviortree
    spdlog::spdlog
  )
  add_test(NAME mission_node_timeout COMMAND mission_node_timeout_test)

  add_executable(route_preflight_test
    tests/unit/route_preflight_test.cpp
    src/task/command_tree_plan.cpp
    src/task/nodes/adjust_height_node.cpp
    src/task/nodes/takeoff_node.cpp
  )
  target_include_directories(route_preflight_test PRIVATE src)
  target_compile_options(route_preflight_test PRIVATE -Wall -Wextra)
  target_link_libraries(route_preflight_test PRIVATE
    plane::common_include
    plane::behaviortree
    spdlog::spdlog
  )
  add_test(NAME route_preflight COMMAND route_preflight_test)
endif()

# 默认构建即组装 build/plane 独立运行目录：复制三方库、改写 rpath；不分发图像插件。
add_custom_target(package_plane ALL
  COMMAND bash "${CMAKE_SOURCE_DIR}/tools/bundle_plane.sh" "${CMAKE_BINARY_DIR}" "${CMAKE_SOURCE_DIR}"
  DEPENDS plane
  COMMENT "组装 build/plane 独立运行目录（三方库 + rpath）"
  VERBATIM
)
