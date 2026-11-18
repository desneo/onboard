#!/usr/bin/env bash
# 手动打包入口：通过 CMake 组装 build/plane 独立运行目录。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${1:-${ROOT}/build}"

cmake --build "${BUILD_DIR}" -j

echo "运行: cd ${BUILD_DIR}/plane && ./plane"
echo "注意: 系统运行库 需按部署环境提供"
