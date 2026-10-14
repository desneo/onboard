#!/usr/bin/env bash
# 集成测试公共函数库（被 tests/integration/*.sh source）。
set -euo pipefail

PLANE_TEST_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
PLANE_BIN="${PLANE_BIN:-${PLANE_TEST_ROOT}/build/plane/plane}"
PLANE_PLUGINS_DIR="${PLANE_PLUGINS_DIR:-${PLANE_TEST_ROOT}/build/plugins}"
PLANE_FIXTURES_DIR="${PLANE_FIXTURES_DIR:-${PLANE_TEST_ROOT}/tests/fixtures}"
SKIP_HW="${SKIP_HW:-0}"

# 断言失败时打印并退出非零。
fail() {
  echo "断言失败: $*" >&2
  exit 1
}

# 构造运行目录：临时目录内含 config/plane.conf 与 plugins 软链。
make_run_dir() {
  local fixture="$1"
  local dir
  dir="$(mktemp -d "${TMPDIR:-/tmp}/plane-test.XXXXXX")"
  mkdir -p "${dir}/config"
  cp "${fixture}" "${dir}/config/plane.conf"
  ln -sfn "${PLANE_PLUGINS_DIR}" "${dir}/plugins"
  echo "${dir}"
}

PLANE_PID=""

# 后台启动 plane，日志写入 $2。
start_plane() {
  local dir="$1"
  local logfile="$2"
  ( cd "${dir}" && exec "${PLANE_BIN}" >"${logfile}" 2>&1 ) &
  PLANE_PID=$!
}

# 停止 plane（不存在时忽略）。
stop_plane() {
  if [ -n "${PLANE_PID}" ]; then
    kill -TERM "${PLANE_PID}" 2>/dev/null || true
    wait "${PLANE_PID}" 2>/dev/null || true
    PLANE_PID=""
  fi
}

# 等待 plane 退出并设置 PLANE_EXIT_CODE（超时则终止并置 143）。用法: wait_plane_exit [timeout_s]
PLANE_EXIT_CODE=""
wait_plane_exit() {
  local timeout_s="${1:-30}" code=0
  local watchdog
  ( sleep "${timeout_s}"; kill -TERM "${PLANE_PID}" 2>/dev/null || true ) &
  watchdog=$!
  wait "${PLANE_PID}" || code=$?
  kill "${watchdog}" 2>/dev/null || true
  wait "${watchdog}" 2>/dev/null || true
  PLANE_PID=""
  PLANE_EXIT_CODE="${code}"
}

# 等待日志匹配正则，超时返回非零。用法: wait_for_log <logfile> <regex> <timeout_s>
wait_for_log() {
  local logfile="$1" pattern="$2" timeout_s="${3:-30}"
  local deadline=$((SECONDS + timeout_s))
  while [ "${SECONDS}" -lt "${deadline}" ]; do
    if grep -Eq "${pattern}" "${logfile}" 2>/dev/null; then
      return 0
    fi
    if ! kill -0 "${PLANE_PID}" 2>/dev/null; then
      return 1
    fi
    sleep 0.5
  done
  return 1
}

# 从日志解析推流地址（推流地址=<url>）。
extract_stream_url() {
  local logfile="$1"
  grep -Eo '推流地址=[^[:space:]]+' "${logfile}" | tail -n1 | cut -d= -f2- || true
}

# 清理临时运行目录。
cleanup_run_dir() {
  local dir="$1"
  [ -n "${dir}" ] && rm -rf "${dir}"
}
