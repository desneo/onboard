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

mkdir -p "${PLANE_DIR}/config" "${PLANE_DIR}/plugins" "${PLANE_DIR}/lib" "${PLANE_DIR}/rknn"

# 部署说明：随独立运行目录分发，部署端无需依赖源码树即可完成配置和启停。
install -m 0644 "${ROOT}/README.md" "${PLANE_DIR}/README.md"

# 启停脚本：随独立运行目录分发，现场可直接 ./runplane.sh / ./stopplane.sh 使用。
for script in runplane.sh stopplane.sh; do
  install -m 0755 "${ROOT}/scripts/${script}" "${PLANE_DIR}/${script}"
done

# 正式插件已由 stage_plugins 暂存到 build/plugins，这里同步到独立运行目录（先清理旧产物）。
rm -f "${PLANE_DIR}"/plugins/*.so
install -m 0755 "${BUILD_DIR}"/plugins/*.so "${PLANE_DIR}/plugins/" 2>/dev/null || true

# 模型目录：复制到独立运行目录（[detection] model_path 使用相对路径 models/...）。
# 来源默认 ../algorithm/models，可用 PLANE_MODELS_DIR 覆盖。
MODELS_SRC="${PLANE_MODELS_DIR:-${ROOT}/../algorithm/perception/models}"
if [ -d "${MODELS_SRC}" ]; then
  rm -rf "${PLANE_DIR}/models"
  cp -a "${MODELS_SRC}" "${PLANE_DIR}/models"
  echo "已复制模型目录: ${MODELS_SRC} -> ${PLANE_DIR}/models"
else
  echo "警告: 未找到模型目录 ${MODELS_SRC}，跳过" >&2
fi

# 检测插件配置：与模型一样随运行目录分发，禁止依赖代码机绝对路径。
DETECTION_CONFIG_SRC="${PLANE_DETECTION_CONFIG_DIR:-${ROOT}/../algorithm/perception/configs}"
if [ -d "${DETECTION_CONFIG_SRC}" ]; then
  rm -rf "${PLANE_DIR}/configs"
  cp -a "${DETECTION_CONFIG_SRC}" "${PLANE_DIR}/configs"
  echo "已复制检测配置: ${DETECTION_CONFIG_SRC} -> ${PLANE_DIR}/configs"
else
  echo "错误: 未找到检测配置目录 ${DETECTION_CONFIG_SRC}" >&2
  exit 1
fi

# 配置文件：每次从工程生成，[plugins] directory 指向目录内 plugins。
sed 's|^directory = .*|directory = plugins|' "${ROOT}/config/plane.conf" > "${PLANE_DIR}/config/plane.conf"

# 先快照依赖解析结果，再清除旧包残留。主程序打包后使用 $ORIGIN/lib；如果先清理，
# 增量打包时会暂时丢失原包中的私有库路径，导致无法重新收集。
LDD_SNAPSHOT="$(mktemp)"
LIB_STAGE="$(mktemp -d)"
mkdir -p "${LIB_STAGE}/private" "${LIB_STAGE}/rknn"
trap 'rm -f "${LDD_SNAPSHOT}"; rm -rf -- "${LIB_STAGE}"' EXIT
ldd "${PLANE_DIR}/plane" "${PLANE_DIR}"/plugins/*.so >"${LDD_SNAPSHOT}" 2>/dev/null || true

# 普通系统运行库由目标板通过 APT 安装；这里只暂存 /lib、/usr/lib 之外的项目/三方库。
awk '/=>/ {print $3}' "${LDD_SNAPSHOT}" \
  | grep -Ev '^/lib/|^/usr/lib/|^$|^not$' \
  | sort -u \
  | while read -r lib; do
  [ -f "${lib}" ] || continue
  cp -L "${lib}" "${LIB_STAGE}/private/$(basename "${lib}")"
    done

# RKNN 用户态运行库不依赖目标板 APT 仓库，单独暂存；fleetctl 会在 APT 依赖安装完成后
# 将其安装到目标板 /usr/local/lib 并执行 ldconfig。
awk '/=>/ && $1 ~ /^librknn/ {print $3}' "${LDD_SNAPSHOT}" \
  | sort -u \
  | while read -r lib; do
      [ -f "${lib}" ] || continue
      cp -L "${lib}" "${LIB_STAGE}/rknn/$(basename "${lib}")"
    done

rm -f "${PLANE_DIR}"/lib/* "${PLANE_DIR}"/rknn/*
cp -a "${LIB_STAGE}/private/." "${PLANE_DIR}/lib/"
cp -a "${LIB_STAGE}/rknn/." "${PLANE_DIR}/rknn/"

if ! compgen -G "${PLANE_DIR}/rknn/librknn*.so*" >/dev/null; then
  echo "错误: 未从构建环境解析到 RKNN 用户态运行库" >&2
  exit 1
fi

rm -f "${LDD_SNAPSHOT}"
rm -rf -- "${LIB_STAGE}"
trap - EXIT

# 运行期库搜索路径：主程序改为相对 $ORIGIN；正式插件的 rpath 只追加不覆写，
# 避免破坏插件自身（RKNN/MPP/Pluma 等）的库搜索。
patchelf --set-rpath '$ORIGIN/lib' "${PLANE_DIR}/plane"
for so in "${PLANE_DIR}"/plugins/*.so; do
  patchelf --add-rpath '$ORIGIN/../lib' "${so}"
done

echo "已组装独立运行目录: ${PLANE_DIR}"
