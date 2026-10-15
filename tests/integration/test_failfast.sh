#!/usr/bin/env bash
# 插件降级验证：插件目录缺失 / 模型不可用时，只记录错误且主程序保持运行。
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../harness/lib.sh
source "${SCRIPT_DIR}/../harness/lib.sh"

[ -x "${PLANE_BIN}" ] || fail "未找到可执行文件 ${PLANE_BIN}"

expect_degraded() {
  local fixture="$1" what="$2" error_pattern="$3"
  local run_dir log
  run_dir="$(make_run_dir "${fixture}")"
  log="${run_dir}/plane.log"
  start_plane "${run_dir}" "${log}"
  if ! wait_for_log "${log}" '系统就绪' 30; then
    tail -n 60 "${log}" >&2
    stop_plane
    cleanup_run_dir "${run_dir}"
    fail "${what}: 未进入系统就绪状态"
  fi
  if ! grep -Eq "${error_pattern}" "${log}"; then
    tail -n 60 "${log}" >&2
    stop_plane
    cleanup_run_dir "${run_dir}"
    fail "${what}: 缺少插件降级日志"
  fi
  if grep -Eq '\[fatal\] plugin=.* reason=' "${log}"; then
    tail -n 60 "${log}" >&2
    stop_plane
    cleanup_run_dir "${run_dir}"
    fail "${what}: 不应产生 fatal 日志"
  fi
  kill -0 "${PLANE_PID}" 2>/dev/null || fail "${what}: 主程序未保持运行"
  echo "${what}: 已记录错误且主程序保持运行"
  stop_plane
  cleanup_run_dir "${run_dir}"
}

expect_degraded "${PLANE_FIXTURES_DIR}/bad_plugin_dir.conf" "插件目录缺失" '无可用推流插件|无可用取图插件'
expect_degraded "${PLANE_FIXTURES_DIR}/bad_model.conf" "检测模型不可用" '检测插件 setConfig 失败'

echo "插件降级验证通过"
