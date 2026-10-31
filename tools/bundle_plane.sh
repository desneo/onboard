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

mkdir -p "${PLANE_DIR}/config" "${PLANE_DIR}/lib"

# 部署说明：随独立运行目录分发，部署端无需依赖源码树即可完成配置和启停。
install -m 0644 "${ROOT}/README.md" "${PLANE_DIR}/README.md"

# 启停脚本：随独立运行目录分发，现场可直接 ./runplane.sh / ./stopplane.sh 使用。
for script in runplane.sh stopplane.sh; do
  install -m 0755 "${ROOT}/scripts/${script}" "${PLANE_DIR}/${script}"
done

# 清理旧版本遗留的图像插件、模型及配置，当前版本不再分发。
rm -rf -- "${PLANE_DIR}/plugins" "${PLANE_DIR}/models" "${PLANE_DIR}/configs" "${PLANE_DIR}/rknn"

# 配置文件：每次从工程生成。
install -m 0644 "${ROOT}/config/plane.conf" "${PLANE_DIR}/config/plane.conf"
install -m 0644 "${ROOT}/config/zenoh.json5" "${PLANE_DIR}/config/zenoh.json5"

# 先快照依赖解析结果，再清除旧包残留。主程序打包后使用 $ORIGIN/lib；如果先清理，
# 增量打包时会暂时丢失原包中的私有库路径，导致无法重新收集。
LDD_SNAPSHOT="$(mktemp)"
LIB_STAGE="$(mktemp -d)"
mkdir -p "${LIB_STAGE}/private"
trap 'rm -f "${LDD_SNAPSHOT}"; rm -rf -- "${LIB_STAGE}"' EXIT
ldd "${PLANE_DIR}/plane" >"${LDD_SNAPSHOT}" 2>/dev/null || true

# 普通系统运行库由目标板通过 APT 安装；这里只暂存 /lib、/usr/lib 之外的项目/三方库。
awk '/=>/ {print $3}' "${LDD_SNAPSHOT}" \
  | grep -Ev '^/lib/|^/usr/lib/|^$|^not$' \
  | sort -u \
  | while read -r lib; do
  [ -f "${lib}" ] || continue
  cp -L "${lib}" "${LIB_STAGE}/private/$(basename "${lib}")"
    done

rm -f "${PLANE_DIR}"/lib/*
cp -a "${LIB_STAGE}/private/." "${PLANE_DIR}/lib/"

# 中制导发布包不设置 SONAME，按其唯一运行库文件名原样分发。
# 公共接口来自 common_include，不复制算法源码或 Pluma 兼容插件。
MIDCOURSE_RELEASE_LIB="${ROOT}/../algorithm/midcourse_alg/midcourse_alg-0.1.0-aarch64/lib"
install -m 0644 "${MIDCOURSE_RELEASE_LIB}/libmidcourse_alg_3588.so" \
  "${PLANE_DIR}/lib/libmidcourse_alg_3588.so"

rm -f "${LDD_SNAPSHOT}"
rm -rf -- "${LIB_STAGE}"
trap - EXIT

# 运行期库搜索路径：主程序使用相对 $ORIGIN。
patchelf --set-rpath '$ORIGIN/lib' "${PLANE_DIR}/plane"

echo "已组装独立运行目录: ${PLANE_DIR}"
