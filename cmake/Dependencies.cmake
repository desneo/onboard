include_guard(GLOBAL)

set(PLANE_3RDPARTY_DIR "${CMAKE_SOURCE_DIR}/../../../3rdparty/common"
    CACHE PATH "本地三方库根目录（相对工程根 onboard/）")
set(PLANE_COMMON_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../common/common_include"
    CACHE PATH "与外部交互的公共头文件目录（相对工程根 onboard/）")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
  set(PLANE_PLATFORM_ARCH "arm64")
else()
  set(PLANE_PLATFORM_ARCH "x64")
endif()
set(PLANE_PLATFORM_DIR "${PLANE_PLATFORM_ARCH}/ubuntu22.04"
    CACHE STRING "三方库平台子目录（按 CPU 架构自动选择，可覆盖）")

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
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
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
  opencv_imgproc
  MAVSDK::mavsdk
  zenohc::lib
  zenohcxx::zenohc
  plane_behaviortree
  plane_pluma
  plane_eventpp
  plane_magic_enum
  plane_nlohmann_json
)

# RK3588 分辨率转换使用 RGA 硬件（仅 arm64 平台存在 librga）。
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
  find_library(PLANE_RGA_LIBRARY NAMES rga)
  if(NOT PLANE_RGA_LIBRARY)
    message(FATAL_ERROR "缺少 librga（RK3588 RGA 分辨率转换依赖）")
  endif()
  target_link_libraries(plane_deps INTERFACE "${PLANE_RGA_LIBRARY}")
endif()

add_library(plane::deps ALIAS plane_deps)
add_library(plane::common_include ALIAS plane_common_include)
add_library(plane::behaviortree ALIAS plane_behaviortree)
add_library(plane::pluma ALIAS plane_pluma)
add_library(plane::eventpp ALIAS plane_eventpp)
add_library(plane::magic_enum ALIAS plane_magic_enum)
add_library(plane::nlohmann_json ALIAS plane_nlohmann_json)

foreach(_target fmt::fmt spdlog::spdlog GeographicLib::GeographicLib MAVSDK::mavsdk
                zenohc::lib zenohcxx::zenohc opencv_core opencv_imgproc)
  if(NOT TARGET ${_target})
    message(FATAL_ERROR "缺少 CMake target: ${_target}")
  endif()
endforeach()
