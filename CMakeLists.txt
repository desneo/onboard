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
