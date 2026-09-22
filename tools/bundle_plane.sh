#!/usr/bin/env bash
# 组装 build/plane 独立运行目录（供 CMake 构建时自动调用）。
# 用法: bundle_plane.sh <build_dir> <project_root>
set -euo pipefail

BUILD_DIR="${1:?需要 build 目录}"
ROOT="${2:?需要工程根目录}"
PLANE_DIR="${BUILD_DIR}/plane"

if [ ! -x "${PLANE_DIR}/plane" ]; then
  echo "未找到可执行文件: ${PLANE_DIR}/plane（请先构建 plane 目标）" >&2
  exit 1
fi

mkdir -p "${PLANE_DIR}/config" "${PLANE_DIR}/plugins" "${PLANE_DIR}/lib"

install -m 0755 "${BUILD_DIR}"/plugins/*.so "${PLANE_DIR}/plugins/"

# 配置文件：不存在时从工程生成，plugins_dir 指向目录内 plugins。
if [ ! -f "${PLANE_DIR}/config/onboard.conf" ]; then
  sed 's|^plugins_dir = .*|plugins_dir = plugins|' "${ROOT}/config/onboard.conf" > "${PLANE_DIR}/config/onboard.conf"
fi

# 收集三方库：对可执行文件与插件做 ldd，取工程三方库目录下的依赖。
collect_libs() {
  ldd "$@" 2>/dev/null | awk '/=>/ {print $3}' | grep '/3rdparty/' || true
}

collect_libs "${PLANE_DIR}/plane" "${PLANE_DIR}"/plugins/*.so | sort -u | while read -r lib; do
  [ -f "${lib}" ] || continue
  cp -L "${lib}" "${PLANE_DIR}/lib/$(basename "${lib}")"
done

# 运行期库搜索路径改为相对 $ORIGIN，脱离绝对三方库路径，便于整体拷贝运行。
patchelf --set-rpath '$ORIGIN/lib' "${PLANE_DIR}/plane"
for so in "${PLANE_DIR}"/plugins/*.so; do
  patchelf --set-rpath '$ORIGIN/../lib' "${so}"
done

echo "已组装独立运行目录: ${PLANE_DIR}"
