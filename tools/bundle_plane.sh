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

# 正式插件已由 stage_plugins 暂存到 build/plugins，这里同步到独立运行目录（先清理旧产物）。
rm -f "${PLANE_DIR}"/plugins/*.so
install -m 0755 "${BUILD_DIR}"/plugins/*.so "${PLANE_DIR}/plugins/" 2>/dev/null || true

# 配置文件：不存在时从工程生成，plugins_dir 指向目录内 plugins。
if [ ! -f "${PLANE_DIR}/config/onboard.conf" ]; then
  sed 's|^plugins_dir = .*|plugins_dir = plugins|' "${ROOT}/config/onboard.conf" > "${PLANE_DIR}/config/onboard.conf"
fi

# 收集三方库：对可执行文件与插件做 ldd，取非系统路径的依赖。
collect_libs() {
  ldd "$@" 2>/dev/null | awk '/=>/ {print $3}' | grep -Ev '^/lib/|^/usr/lib/|^$' || true
}

collect_libs "${PLANE_DIR}/plane" "${PLANE_DIR}"/plugins/*.so | sort -u | while read -r lib; do
  [ -f "${lib}" ] || continue
  dest="${PLANE_DIR}/lib/$(basename "${lib}")"
  if [ -e "${dest}" ]; then
    continue
  fi
  cp -L "${lib}" "${dest}"
done

# 运行期库搜索路径：主程序改为相对 $ORIGIN；正式插件的 rpath 只追加不覆写，
# 避免破坏插件自身（RKNN/MPP/Pluma 等）的库搜索。
patchelf --set-rpath '$ORIGIN/lib' "${PLANE_DIR}/plane"
for so in "${PLANE_DIR}"/plugins/*.so; do
  patchelf --add-rpath '$ORIGIN/../lib' "${so}"
done

echo "已组装独立运行目录: ${PLANE_DIR}"
