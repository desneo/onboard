#!/usr/bin/env bash
# 启动机载程序 runplane.sh
# 1. 若已有 plane 在运行，则提示并退出，不重启；
# 2. 用 rknn-inspect 读取检测模型输入宽高，写回 image_input、detection、video_output section；
# 3. 运行 ./plane。
#
# 用法: runplane.sh [plane_dir]
#   plane_dir 缺省自动探测：脚本所在目录含 plane 则使用该目录，否则使用工程的 build/plane。
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

# 已在运行则不重启。
RUNNING_PIDS="$(pgrep -x plane 2>/dev/null || true)"
if [ -n "${RUNNING_PIDS}" ]; then
    echo "plane 已在运行 (pid: $(printf '%s ' ${RUNNING_PIDS}))，不重启。"
    exit 0
fi

if [ "${1:-}" != "" ]; then
    PLANE_DIR="$1"
elif [ -x "${SCRIPT_DIR}/plane" ] && [ -f "${SCRIPT_DIR}/config/plane.conf" ]; then
    PLANE_DIR="${SCRIPT_DIR}"
elif [ -x "${SOURCE_ROOT}/build/plane/plane" ]; then
    PLANE_DIR="${SOURCE_ROOT}/build/plane"
else
    echo "错误: 未找到 plane 运行目录（可用参数指定）" >&2
    exit 1
fi

CONFIG="${PLANE_DIR}/config/plane.conf"
[ -f "${CONFIG}" ] || { echo "错误: 配置文件不存在: ${CONFIG}" >&2; exit 1; }
[ -x "${PLANE_DIR}/plane" ] || { echo "错误: 可执行文件不存在: ${PLANE_DIR}/plane" >&2; exit 1; }

# 读取指定 INI section 中的键值。
get_config() { # section key
    awk -v wanted_section="$1" -v wanted_key="$2" '
        function trim(value) {
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
            return value
        }
        /^[[:space:]]*\[[^]]+\][[:space:]]*$/ {
            section = $0
            gsub(/^[[:space:]]*\[|\][[:space:]]*$/, "", section)
            next
        }
        section == wanted_section {
            pos = index($0, "=")
            if (pos == 0) next
            key = trim(substr($0, 1, pos - 1))
            if (key == wanted_key) result = trim(substr($0, pos + 1))
        }
        END { print result }
    ' "${CONFIG}"
}

# 模型路径：优先取 [detection] model_path，缺省 models/yolo11/yolo11n。
DET_MODEL_PATH="$(get_config detection model_path)"
[ -n "${DET_MODEL_PATH}" ] || DET_MODEL_PATH="models/yolo11/yolo11n"
case "${DET_MODEL_PATH}" in
    *.rknn) MODEL="${PLANE_DIR}/${DET_MODEL_PATH}" ;;
    *) MODEL="${PLANE_DIR}/${DET_MODEL_PATH}/model.rknn" ;;
esac
[ -f "${MODEL}" ] || { echo "错误: 模型文件不存在: ${MODEL}" >&2; exit 1; }

command -v rknn-inspect >/dev/null 2>&1 || { echo "错误: 未找到 rknn-inspect 命令" >&2; exit 1; }

# 解析模型 Input 的 Shape，按 Format(NHWC/NCHW) 取宽高。
INSPECT="$(rknn-inspect -i "${MODEL}" 2>/dev/null || true)"
LINE="$(printf '%s\n' "${INSPECT}" | grep -m1 'Input' || true)"
SHAPE="$(printf '%s' "${LINE}" | grep -o '\[[0-9][0-9, ]*\]' | head -1 || true)"
FORMAT="$(printf '%s' "${LINE}" | grep -o -E 'NHWC|NCHW' | head -1 || true)"

if [ -z "${SHAPE}" ]; then
    echo "警告: 无法解析模型输入尺寸，保持配置不变 (${MODEL})" >&2
else
    NUMS=($(printf '%s' "${SHAPE}" | tr -dc '0-9,' | tr ',' ' '))
    if [ "${FORMAT}" = "NHWC" ]; then
        MDL_H="${NUMS[1]:-}"
        MDL_W="${NUMS[2]:-}"
    else
        MDL_H="${NUMS[2]:-}"
        MDL_W="${NUMS[3]:-}"
    fi
    if [ -n "${MDL_W}" ] && [ -n "${MDL_H}" ]; then
        set_config() { # section key value
            local section="$1" key="$2" value="$3" tmp="${CONFIG}.tmp.$$"
            awk -v wanted_section="${section}" -v wanted_key="${key}" -v wanted_value="${value}" '
                function trim(text) {
                    gsub(/^[[:space:]]+|[[:space:]]+$/, "", text)
                    return text
                }
                /^[[:space:]]*\[[^]]+\][[:space:]]*$/ {
                    if (in_section && !written) {
                        print wanted_key " = " wanted_value
                        written = 1
                    }
                    current = $0
                    gsub(/^[[:space:]]*\[|\][[:space:]]*$/, "", current)
                    in_section = (current == wanted_section)
                    if (in_section) section_found = 1
                    print
                    next
                }
                {
                    if (in_section) {
                        pos = index($0, "=")
                        candidate = pos == 0 ? "" : trim(substr($0, 1, pos - 1))
                        if (candidate == wanted_key) {
                            print wanted_key " = " wanted_value
                            written = 1
                            next
                        }
                    }
                    print
                }
                END {
                    if (in_section && !written) {
                        print wanted_key " = " wanted_value
                    } else if (!section_found) {
                        print ""
                        print "[" wanted_section "]"
                        print wanted_key " = " wanted_value
                    }
                }
            ' "${CONFIG}" >"${tmp}"
            mv "${tmp}" "${CONFIG}"
        }
        set_config image_input width "${MDL_W}"
        set_config image_input height "${MDL_H}"
        set_config detection input_width "${MDL_W}"
        set_config detection input_height "${MDL_H}"
        set_config video_output width "${MDL_W}"
        set_config video_output height "${MDL_H}"
        echo "已按模型输入更新配置: ${MDL_W}x${MDL_H} (${MODEL})"
    else
        echo "警告: 模型输入尺寸解析失败: shape=${SHAPE} format=${FORMAT}" >&2
    fi
fi

cd "${PLANE_DIR}"
mkdir -p logs
./plane
