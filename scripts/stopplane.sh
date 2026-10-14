#!/usr/bin/env bash
# 停止机载程序 plane。
# 优先发送 SIGINT 触发优雅退出（走 shutdown_logging，避免日志被截断）；
# 超时后依次 SIGTERM / SIGKILL 强制结束。
#
# 用法: stopplane.sh [超时秒数]   （默认 10 秒）
set -euo pipefail

TIMEOUT_S="${1:-10}"

PIDS="$(pgrep -x plane 2>/dev/null || true)"
if [ -z "${PIDS}" ]; then
    echo "plane 未在运行。"
    exit 0
fi

echo "停止 plane (pid: $(printf '%s ' ${PIDS})) ..."
kill -INT ${PIDS} 2>/dev/null || true

# 等待优雅退出（每 0.1s 轮询一次）。
for ((i = 0; i < TIMEOUT_S * 10; i++)); do
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
