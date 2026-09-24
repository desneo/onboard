#!/usr/bin/env bash
# US1/US3 日志验证：从日志断言取图→检测→推流链路各环节正常。
# 用法: test_pipeline_logs.sh
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../harness/lib.sh
source "${SCRIPT_DIR}/../harness/lib.sh"

if [ "${SKIP_HW}" = "1" ]; then
  echo "SKIP_HW=1，跳过硬件相关日志验证"
  exit 0
fi
[ -x "${PLANE_BIN}" ] || fail "未找到可执行文件 ${PLANE_BIN}"

RUN_DIR="$(make_run_dir "${PLANE_FIXTURES_DIR}/mipi_webrtc.conf")"
LOG="${RUN_DIR}/plane.log"
trap 'stop_plane; cleanup_run_dir "${RUN_DIR}"' EXIT

start_plane "${RUN_DIR}" "${LOG}"

assert_log() {
  if ! wait_for_log "${LOG}" "$1" "${2:-40}"; then
    echo "--- 日志尾部 ---" >&2
    tail -n 80 "${LOG}" >&2
    fail "$3 (pattern: $1)"
  fi
}

# 插件加载与 provider 计数
assert_log '插件加载目录=' 30 "缺少插件加载日志"
assert_log '插件 provider: image_input=' 30 "缺少 provider 计数日志"
# 取图/检测/推流插件初始化参数（FR-018）
assert_log '\[plugin:image_input\].*type=' 30 "缺少取图插件初始化参数日志"
assert_log '\[plugin:det_mot\].*model_path=' 30 "缺少检测插件初始化参数日志"
assert_log '\[plugin:videostream_output\].*type=' 30 "缺少推流插件初始化参数日志"
# 取图与检测
assert_log '首帧 size=' 60 "缺少首帧日志"
assert_log '检测 targets=' 60 "缺少检测结果日志"
# 推流地址与状态（FR-006/FR-013）
assert_log '推流地址=' 30 "缺少推流地址日志"
assert_log '推流状态 state=.*fps=.*viewers=' 60 "缺少推流状态日志"
# 无观众时应为 Ready（暂停编码）
assert_log '推流状态 state=Ready' 60 "无观众时未进入 Ready 状态"

echo "日志验证通过: 取图→检测→推流链路日志完整"
