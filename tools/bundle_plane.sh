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

# 启停脚本：随独立运行目录分发，现场可直接 ./runplane.sh / ./stopplane.sh 使用。
for script in runplane.sh stopplane.sh; do
  install -m 0755 "${ROOT}/scripts/${script}" "${PLANE_DIR}/${script}"
done

# 正式插件已由 stage_plugins 暂存到 build/plugins，这里同步到独立运行目录（先清理旧产物）。
rm -f "${PLANE_DIR}"/plugins/*.so
install -m 0755 "${BUILD_DIR}"/plugins/*.so "${PLANE_DIR}/plugins/" 2>/dev/null || true

# 模型目录：复制到独立运行目录（config 中 det_model_path 使用相对路径 models/...）。
# 来源默认 ../algorithm/models，可用 PLANE_MODELS_DIR 覆盖。
MODELS_SRC="${PLANE_MODELS_DIR:-${ROOT}/../algorithm/perception/models}"
if [ -d "${MODELS_SRC}" ]; then
  rm -rf "${PLANE_DIR}/models"
  cp -a "${MODELS_SRC}" "${PLANE_DIR}/models"
  echo "已复制模型目录: ${MODELS_SRC} -> ${PLANE_DIR}/models"
else
  echo "警告: 未找到模型目录 ${MODELS_SRC}，跳过" >&2
fi

# 配置文件：每次从工程生成，plugins_dir 指向目录内 plugins。
sed 's|^plugins_dir = .*|plugins_dir = plugins|' "${ROOT}/config/plane.conf" > "${PLANE_DIR}/config/plane.conf"

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
