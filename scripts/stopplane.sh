#!/usr/bin/env bash
# 停止机载程序 plane。
# 受管服务通过 systemd 主动停止并取消待执行的自动重启。
# 手动模式优先发送 SIGINT，超时后依次发送 SIGTERM 和 SIGKILL。
#
# 用法: stopplane.sh [超时秒数]   （默认 10 秒）
set -euo pipefail

case "${1:-}" in
    -h|--help) printf '用法: %s [手动模式超时秒数]\n' "$0"; exit 0 ;;
esac
if [[ "$#" -gt 1 || ! "${1:-10}" =~ ^[0-9]+$ ]]; then
    printf '错误: 超时秒数须为非负整数。用法: %s [手动模式超时秒数]\n' "$0" >&2
    exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/runplane.sh"

if plane_service_status; then
    if plane_service_is_running; then
        plane_service_control stop
        echo 'plane 服务已停止。'
        exit 0
    fi
elif [[ "$?" != 1 ]]; then
    exit 1
fi

TIMEOUT_S="${1:-10}"

PIDS="$(pgrep -x plane 2>/dev/null || true)"
if [ -z "${PIDS}" ]; then
    echo "plane 未在运行。"
    exit 0
fi

echo "停止 plane (pid: $(printf '%s ' ${PIDS})) ..."
kill -INT ${PIDS} 2>/dev/null || true

# 等待优雅退出（每 0.1s 轮询一次）。
for ((i = 0; i < 10#$TIMEOUT_S * 10; i++)); do
    if ! pgrep -x plane >/dev/null 2>&1; then
        echo "plane 已停止。"
        exit 0
    fi
    sleep 0.1
done

echo "优雅退出超时（${TIMEOUT_S}s），强制结束 ..."
kill -TERM ${PIDS} 2>/dev/null || true
sleep 1
kill -KILL ${PIDS} 2>/dev/null || true
sleep 1

if pgrep -x plane >/dev/null 2>&1; then
    echo "警告: plane 仍在运行 (pid: $(printf '%s ' $(pgrep -x plane)))" >&2
    exit 1
fi
echo "plane 已停止。"
