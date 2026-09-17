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
