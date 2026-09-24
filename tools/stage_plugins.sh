#!/usr/bin/env bash
# 将正式插件 .so 暂存到 <build_dir>/plugins（供宿主统一加载）。
# 用法: stage_plugins.sh [build_dir]  （默认 build）
#
# 第一步只接入取图与推流插件，此二者必须存在；检测插件为后续步骤，存在则一并暂存。
# 可用环境变量覆盖插件来源路径：
#   PLANE_IMAGE_INPUT_SO / PLANE_VIDEO_OUTPUT_SO / PLANE_DET_MOT_SO
set -euo pipefail

BUILD_DIR="${1:-build}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ANTI_UAV="$(cd -- "${ROOT}/.." && pwd)"

IMAGE_INPUT_SO="${PLANE_IMAGE_INPUT_SO:-${ANTI_UAV}/functional/image_input_3588/plugins/libimage_input_3588.so}"
VIDEO_OUTPUT_SO="${PLANE_VIDEO_OUTPUT_SO:-${ANTI_UAV}/functional/videostream_output_3588/plugins/libvideostream_output_3588.so}"
DET_MOT_SO="${PLANE_DET_MOT_SO:-${ANTI_UAV}/algorithm/perception/plugins/3588/libdet_mot_3588.so}"

DEST="${BUILD_DIR}/plugins"
mkdir -p "${DEST}"
# 清理旧的暂存产物，避免残留插件被宿主加载。
rm -f "${DEST}"/*.so

missing=0
for so in "${IMAGE_INPUT_SO}" "${VIDEO_OUTPUT_SO}"; do
  if [ -f "${so}" ]; then
    cp -f "${so}" "${DEST}/"
    echo "暂存插件: $(basename "${so}")"
  else
    echo "错误: 未找到必需插件 ${so}（可用 PLANE_*_SO 环境变量覆盖）" >&2
    missing=1
  fi
done

# 检测插件为后续步骤，存在则暂存，缺失不视为失败。
if [ -f "${DET_MOT_SO}" ]; then
  cp -f "${DET_MOT_SO}" "${DEST}/"
  echo "暂存插件(可选): $(basename "${DET_MOT_SO}")"
else
  echo "提示: 未找到检测插件 ${DET_MOT_SO}，第一步可忽略" >&2
fi

if [ "${missing}" -ne 0 ]; then
  exit 1
fi
echo "正式插件暂存完成: ${DEST}"
