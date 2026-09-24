#!/usr/bin/env bash
# 插件加载契约测试：断言 build/plugins 中三类 provider 各 >= 1。
# 用法: test_plugin_load.sh [build_dir] [plugins_dir]
set -euo pipefail

BUILD_DIR="${1:-build}"
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
CHECK="${ROOT}/${BUILD_DIR}/plugin_load_check"
PLUGINS="${2:-${ROOT}/${BUILD_DIR}/plugins}"

if [ ! -x "${CHECK}" ]; then
  echo "错误: 未找到插件加载检查程序 ${CHECK}（请先构建 plugin_load_check）" >&2
  exit 1
fi
if [ ! -d "${PLUGINS}" ]; then
  echo "错误: 插件目录不存在 ${PLUGINS}" >&2
  exit 1
fi

"${CHECK}" "${PLUGINS}"
