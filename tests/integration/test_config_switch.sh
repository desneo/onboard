#!/usr/bin/env bash
# US2 配置化切换验证：仅改配置即可切换图像源与推流协议，且参数生效。
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../harness/lib.sh
source "${SCRIPT_DIR}/../harness/lib.sh"

if [ "${SKIP_HW}" = "1" ]; then
  echo "SKIP_HW=1，跳过配置切换验证"
  exit 0
fi
[ -x "${PLANE_BIN}" ] || fail "未找到可执行文件 ${PLANE_BIN}"

run_case() {
  local fixture="$1" expect_image="$2" expect_stream="$3"
  local run_dir log
  run_dir="$(make_run_dir "${fixture}")"
  log="${run_dir}/plane.log"
  start_plane "${run_dir}" "${log}"
  if ! wait_for_log "${log}" '\[plugin:videostream_output\]' 30; then
    tail -n 60 "${log}" >&2
    stop_plane; cleanup_run_dir "${run_dir}"
    fail "配置 ${fixture} 未完成插件初始化"
  fi
  grep -Eq "\[plugin:image_input\].*type=${expect_image}" "${log}" || {
    stop_plane; cleanup_run_dir "${run_dir}"; fail "取图类型应为 ${expect_image}"; }
  grep -Eq "\[plugin:videostream_output\].*type=${expect_stream}" "${log}" || {
    stop_plane; cleanup_run_dir "${run_dir}"; fail "推流类型应为 ${expect_stream}"; }
  stop_plane
  cleanup_run_dir "${run_dir}"
  echo "配置生效: $(basename "${fixture}") image=${expect_image} stream=${expect_stream}"
}

run_case "${PLANE_FIXTURES_DIR}/rtsp_webrtc.conf" Rtsp WebRtc
run_case "${PLANE_FIXTURES_DIR}/rtsp_rtsp.conf" Rtsp Rtsp

echo "配置化切换验证通过"
