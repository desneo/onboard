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
  src/state/state_store.cpp
  src/flight_control/flight_control_module.cpp
  src/task/task_scheduler.cpp
  src/task/nodes/arm_node.cpp
  src/task/nodes/takeoff_node.cpp
  src/task/nodes/land_node.cpp
  src/task/nodes/rtl_node.cpp
  src/task/nodes/hover_node.cpp
  src/task/nodes/stop_motor_node.cpp
  src/task/task_execution_engine.cpp
  src/perception/plugin_management.cpp
  src/perception/plugin_providers.cpp
)
target_include_directories(onboard PRIVATE src)
target_compile_options(onboard PRIVATE -Wall -Wextra)
target_compile_definitions(onboard PRIVATE
  PLANE_VERSION="0.1.0"
  PLANE_GIT_HASH="${PLANE_GIT_HASH}"
  PLANE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
)
target_link_libraries(onboard PRIVATE plane::common_include plane::deps)

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

add_library(detection_stub SHARED plugins/detection_stub/detection_stub.cpp)
target_link_libraries(detection_stub PRIVATE plane::pluma plane::common_include opencv_core spdlog::spdlog)
set_target_properties(detection_stub PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/plugins")

add_library(tracking_stub SHARED plugins/tracking_stub/tracking_stub.cpp)
target_link_libraries(tracking_stub PRIVATE plane::pluma plane::common_include opencv_core spdlog::spdlog)
set_target_properties(tracking_stub PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/plugins")

add_library(guidance_stub SHARED plugins/guidance_stub/guidance_stub.cpp)
target_link_libraries(guidance_stub PRIVATE plane::pluma plane::common_include spdlog::spdlog)
set_target_properties(guidance_stub PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/plugins")
