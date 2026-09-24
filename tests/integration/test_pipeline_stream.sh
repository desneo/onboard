#!/usr/bin/env bash
# US1/US2 视频流验证：从输出流断言推流可拉取、可解码、参数正确。
# - RTSP 模式：ffprobe 校验编码/分辨率，ffmpeg 解码若干秒确认有帧。
# - WebRTC 模式：curl 校验 WHEP 观看页可达（ffprobe 无法直接拉 WHEP）。
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../harness/lib.sh
source "${SCRIPT_DIR}/../harness/lib.sh"

if [ "${SKIP_HW}" = "1" ]; then
  echo "SKIP_HW=1，跳过视频流验证"
  exit 0
fi
[ -x "${PLANE_BIN}" ] || fail "未找到可执行文件 ${PLANE_BIN}"
command -v ffprobe >/dev/null || fail "缺少 ffprobe"
command -v curl >/dev/null || fail "缺少 curl"

check_rtsp_stream() {
  local fixture="$1" app stream
  app="live"; stream="stream"
  local run_dir log url
  run_dir="$(make_run_dir "${fixture}")"
  log="${run_dir}/plane.log"
  start_plane "${run_dir}" "${log}"
  if ! wait_for_log "${log}" '推流地址=' 30; then
    tail -n 60 "${log}" >&2
    stop_plane; cleanup_run_dir "${run_dir}"
    fail "未出现推流地址日志"
  fi
  url="$(extract_stream_url "${log}")"
  echo "RTSP 推流地址: ${url}"
  local probe
  probe="$(timeout 20 ffprobe -rtsp_transport tcp -v error \
      -show_entries stream=codec_name,width,height -of default=noprint_wrappers=1 "${url}" 2>&1)" || {
    stop_plane; cleanup_run_dir "${run_dir}"; fail "ffprobe 拉流失败: ${probe}"
  }
  echo "${probe}" | grep -q 'codec_name=h264' || { stop_plane; cleanup_run_dir "${run_dir}"; fail "非 h264 流: ${probe}"; }
  echo "${probe}" | grep -q 'width=1280' || { stop_plane; cleanup_run_dir "${run_dir}"; fail "分辨率异常: ${probe}"; }
  echo "${probe}" | grep -q 'height=720' || { stop_plane; cleanup_run_dir "${run_dir}"; fail "分辨率异常: ${probe}"; }
  timeout 25 ffmpeg -rtsp_transport tcp -v error -i "${url}" -t 3 -f null - >/dev/null 2>&1 || {
    stop_plane; cleanup_run_dir "${run_dir}"; fail "ffmpeg 解码失败"
  }
  stop_plane
  cleanup_run_dir "${run_dir}"
}

check_webrtc_stream() {
  local fixture="$1"
  local run_dir log url code
  run_dir="$(make_run_dir "${fixture}")"
  log="${run_dir}/plane.log"
  start_plane "${run_dir}" "${log}"
  if ! wait_for_log "${log}" '推流地址=' 30; then
    tail -n 60 "${log}" >&2
    stop_plane; cleanup_run_dir "${run_dir}"
    fail "未出现推流地址日志"
  fi
  url="$(extract_stream_url "${log}")"
  echo "WebRTC 推流地址: ${url}"
  case "${url}" in
    http*) ;;
    *) stop_plane; cleanup_run_dir "${run_dir}"; fail "WebRTC 地址非 http: ${url}" ;;
  esac
  code="$(curl -s -o /dev/null -w '%{http_code}' --max-time 10 "${url}" || true)"
  stop_plane
  cleanup_run_dir "${run_dir}"
  case "${code}" in
    200|301|302) echo "WHEP 观看页可达 (HTTP ${code})" ;;
    *) fail "WHEP 观看页不可达 (HTTP ${code})" ;;
  esac
}

check_rtsp_stream "${PLANE_FIXTURES_DIR}/rtsp_rtsp.conf"
check_webrtc_stream "${PLANE_FIXTURES_DIR}/mipi_webrtc.conf"

echo "视频流验证通过: RTSP 可拉取可解码，WebRTC 观看页可达"
