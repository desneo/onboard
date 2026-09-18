#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${1:-${ROOT}/build}"
DIST_DIR="${ROOT}/dist"

cmake --build "${BUILD_DIR}" -j
rm -rf "${DIST_DIR}"
mkdir -p "${DIST_DIR}/plugins"
install -m 0755 "${BUILD_DIR}/onboard" "${DIST_DIR}/onboard"
install -m 0755 "${BUILD_DIR}"/plugins/*.so "${DIST_DIR}/plugins/"
sed 's|^plugins_dir = .*|plugins_dir = plugins|' "${ROOT}/config/onboard.conf" > "${DIST_DIR}/onboard.conf"

echo "打包完成: ${DIST_DIR}"
echo "运行: cd dist && ./onboard --config onboard.conf"
echo "注意: 三方运行库 (mavsdk/zenoh/pluma/opencv/ffmpeg/ZLMediaKit) 需按部署环境提供"
