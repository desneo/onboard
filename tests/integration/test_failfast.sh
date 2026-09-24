#!/usr/bin/env bash
# US3 fail-fast 验证：插件目录缺失 / 模型不可用时，打印 [fatal] 并以非零状态退出。
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../harness/lib.sh
source "${SCRIPT_DIR}/../harness/lib.sh"

[ -x "${PLANE_BIN}" ] || fail "未找到可执行文件 ${PLANE_BIN}"

expect_fail() {
  local fixture="$1" what="$2"
  local run_dir log code
  run_dir="$(make_run_dir "${fixture}")"
  log="${run_dir}/plane.log"
  start_plane "${run_dir}" "${log}"
  wait_plane_exit 30
  code="${PLANE_EXIT_CODE}"
  if [ "${code}" = "0" ]; then
    tail -n 60 "${log}" >&2
    cleanup_run_dir "${run_dir}"
    fail "${what}: 期望非零退出，实际为 0"
  fi
  if ! grep -Eq '\[fatal\] plugin=.* reason=' "${log}"; then
    tail -n 60 "${log}" >&2
    cleanup_run_dir "${run_dir}"
    fail "${what}: 缺少 [fatal] plugin= reason= 日志"
  fi
  echo "${what}: 退出码=${code}，已打印 fatal 日志"
  cleanup_run_dir "${run_dir}"
}

expect_fail "${PLANE_FIXTURES_DIR}/bad_plugin_dir.conf" "插件目录缺失"
expect_fail "${PLANE_FIXTURES_DIR}/bad_model.conf" "检测模型不可用"

echo "fail-fast 验证通过"
