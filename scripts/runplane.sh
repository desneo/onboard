#!/usr/bin/env bash
# 启动机载程序 runplane.sh
# 1. 若已有 plane 在运行，则提示并退出，不重启；
# 2. 补齐运行程序和停止脚本的执行权限；
# 3. 运行 ./plane（本版本不处理图像）。
#
# 用法: runplane.sh [plane_dir]
#   plane_dir 缺省自动探测：脚本所在目录含 plane 则使用该目录，否则使用工程的 build/plane。
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

usage() {
    echo "用法: $0 [plane_dir]"
    echo "默认使用脚本所在的运行目录，或源码工程的 build/plane。"
}

case "${1:-}" in
    -h|--help) usage; exit 0 ;;
    -*) echo "错误: 未知选项: $1" >&2; usage >&2; exit 1 ;;
esac
if [ "$#" -gt 1 ]; then
    usage >&2
    exit 1
fi

# 已在运行则不重启。
RUNNING_PIDS="$(pgrep -x plane 2>/dev/null || true)"
if [ -n "${RUNNING_PIDS}" ]; then
    echo "plane 已在运行 (pid: $(printf '%s ' ${RUNNING_PIDS}))，不重启。"
    exit 0
fi

if [ "${1:-}" != "" ]; then
    PLANE_DIR="$1"
elif [ -f "${SCRIPT_DIR}/plane" ] || [ -f "${SCRIPT_DIR}/config/plane.conf" ]; then
    PLANE_DIR="${SCRIPT_DIR}"
elif [ -d "${SOURCE_ROOT}/build/plane" ]; then
    PLANE_DIR="${SOURCE_ROOT}/build/plane"
else
    echo "错误: 未找到 plane 运行目录（可用参数指定）" >&2
    exit 1
fi

CONFIG="${PLANE_DIR}/config/plane.conf"
[ -f "${CONFIG}" ] || { echo "错误: 配置文件不存在: ${CONFIG}" >&2; exit 1; }
[ -f "${PLANE_DIR}/plane" ] || { echo "错误: 可执行文件不存在: ${PLANE_DIR}/plane" >&2; exit 1; }
# 上传工具可能丢失执行权限，启动前自动补齐。
chmod +x -- "${PLANE_DIR}/plane"
if [ -f "${PLANE_DIR}/stopplane.sh" ]; then
    chmod +x -- "${PLANE_DIR}/stopplane.sh"
fi
if [ ! -x "${PLANE_DIR}/plane" ]; then
    echo "错误: plane 文件存在但不可执行: ${PLANE_DIR}/plane" >&2
    printf '请检查执行权限，可执行: chmod +x -- %q\n' "${PLANE_DIR}/plane" >&2
    exit 1
fi

cd "${PLANE_DIR}"
mkdir -p logs
./plane
