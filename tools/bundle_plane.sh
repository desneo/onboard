#!/usr/bin/env bash
# 组装 build/plane 独立运行目录（供 CMake 构建时自动调用）。
# 用法: bundle_plane.sh <build_dir> <project_root> <midcourse_alg_library> [extra_lib_dirs]
set -euo pipefail

BUILD_DIR="${1:?需要 build 目录}"
ROOT="${2:?需要工程根目录}"
MIDCOURSE_ALG_LIB="${3:?需要构建使用的中制导算法库路径}"
EXTRA_LIB_DIRS="${4:-}"
PLANE_DIR="${BUILD_DIR}/plane"

if [ ! -x "${PLANE_DIR}/plane" ]; then
  echo "未找到可执行文件: ${PLANE_DIR}/plane（请先构建 plane 目标）" >&2
  exit 1
fi

mkdir -p "${PLANE_DIR}/config/profiles" "${PLANE_DIR}/lib"

# 部署说明：随独立运行目录分发，部署端无需依赖源码树即可完成配置和启停。
install -m 0644 "${ROOT}/README.md" "${PLANE_DIR}/README.md"

# 启停和自启部署脚本随独立运行目录分发。
for script in runplane.sh stopplane.sh install-plane-service.sh; do
  install -m 0755 "${ROOT}/scripts/${script}" "${PLANE_DIR}/${script}"
done
rm -f -- "${PLANE_DIR}/plane-service.sh"

# 清理旧版本遗留的图像插件、模型及配置，当前版本不再分发。
rm -rf -- "${PLANE_DIR}/plugins" "${PLANE_DIR}/models" "${PLANE_DIR}/configs" "${PLANE_DIR}/rknn"

# 配置文件：每次从工程生成。
install -m 0644 "${ROOT}/config/plane.ini" "${PLANE_DIR}/config/plane.ini"
rm -f -- "${PLANE_DIR}/config/plane.conf"
install -m 0644 "${ROOT}/config/zenoh.json5" "${PLANE_DIR}/config/zenoh.json5"
install -m 0644 "${ROOT}/config/profiles/"*.ini "${PLANE_DIR}/config/profiles/"

READELF_BIN=""
for candidate in readelf aarch64-linux-gnu-readelf; do
  if command -v "${candidate}" >/dev/null 2>&1; then
    READELF_BIN="${candidate}"
    break
  fi
done

# 收集构建期三方库搜索路径，支持本机和交叉编译两种打包路径。
declare -a SEARCH_DIRS=()
append_search_dir() {
  local dir="${1}"
  [ -n "${dir}" ] || return 0
  case "${dir}" in
    /lib|/lib/*|/usr/lib|/usr/lib/*|\$ORIGIN*) return 0 ;;
  esac
  [ -d "${dir}" ] || return 0
  local existing
  for existing in "${SEARCH_DIRS[@]+"${SEARCH_DIRS[@]}"}"; do
    if [ "${existing}" = "${dir}" ]; then
      return 0
    fi
  done
  SEARCH_DIRS+=("${dir}")
}

if [ -n "${EXTRA_LIB_DIRS}" ]; then
  IFS=':' read -r -a _extra_dirs <<< "${EXTRA_LIB_DIRS}"
  for dir in "${_extra_dirs[@]}"; do
    append_search_dir "${dir}"
  done
fi
append_search_dir "$(dirname "${MIDCOURSE_ALG_LIB}")"

if [ -n "${READELF_BIN}" ]; then
  while IFS= read -r rpath_entry; do
    IFS=':' read -r -a _rpath_dirs <<< "${rpath_entry}"
    for dir in "${_rpath_dirs[@]}"; do
      append_search_dir "${dir}"
    done
  done < <(
    LC_ALL=C "${READELF_BIN}" -d "${PLANE_DIR}/plane" 2>/dev/null \
      | sed -n 's/.*(\(RPATH\|RUNPATH\)).*\[\(.*\)\].*/\2/p' || true
  )
fi

if [ -z "${EXTRA_LIB_DIRS}" ] && [ -f "${BUILD_DIR}/CMakeCache.txt" ]; then
  cached_3rdparty="$(sed -n 's/^PLANE_3RDPARTY_DIR:[^=]*=//p' "${BUILD_DIR}/CMakeCache.txt" | head -n 1)"
  cached_platform="$(sed -n 's/^PLANE_PLATFORM_DIR:[^=]*=//p' "${BUILD_DIR}/CMakeCache.txt" | head -n 1)"
  if [ -n "${cached_3rdparty}" ] && [ -n "${cached_platform}" ]; then
    for pkg in fmt spdlog geographiclib mavsdk zenohc behaviortree; do
      for dir in "${cached_3rdparty}/${pkg}"/*/"${cached_platform}"/lib; do
        append_search_dir "${dir}"
      done
    done
  fi
fi

# 先快照依赖解析结果，再清除旧包残留。主程序打包后使用 $ORIGIN/lib；如果先清理，
# 增量打包时会暂时丢失原包中的私有库路径，导致无法重新收集。
LDD_SNAPSHOT="$(mktemp)"
LIB_STAGE="$(mktemp -d)"
mkdir -p "${LIB_STAGE}/private"
trap 'rm -f "${LDD_SNAPSHOT}"; rm -rf -- "${LIB_STAGE}"' EXIT

if [ -n "${EXTRA_LIB_DIRS}" ]; then
  LC_ALL=C LD_LIBRARY_PATH="${EXTRA_LIB_DIRS}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
    ldd "${PLANE_DIR}/plane" >"${LDD_SNAPSHOT}" 2>/dev/null || true
else
  LC_ALL=C ldd "${PLANE_DIR}/plane" >"${LDD_SNAPSHOT}" 2>/dev/null || true
fi

# 普通系统运行库由目标板通过 APT 安装；这里只暂存 /lib、/usr/lib 之外的项目/三方库。
awk '/=>/ && $3 !~ /^(\/lib\/|\/usr\/lib\/|not$|$)/ {print $3}' "${LDD_SNAPSHOT}" \
  | sort -u \
  | while IFS= read -r lib; do
  [ -f "${lib}" ] || continue
  cp -L "${lib}" "${LIB_STAGE}/private/$(basename "${lib}")"
done

# 交叉编译场景下宿主机 ldd 无法解析目标架构 ELF，补充按 readelf DT_NEEDED 收集私有库。
if [ -n "${READELF_BIN}" ] && [ "${#SEARCH_DIRS[@]}" -gt 0 ]; then
  declare -a ELF_QUEUE=("${PLANE_DIR}/plane" "${MIDCOURSE_ALG_LIB}")
  declare -A SCANNED_ELFS=()
  idx=0
  while [ "${idx}" -lt "${#ELF_QUEUE[@]}" ]; do
    elf="${ELF_QUEUE[${idx}]}"
    idx=$((idx + 1))
    [ -f "${elf}" ] || continue
    if [ -n "${SCANNED_ELFS["${elf}"]:-}" ]; then
      continue
    fi
    SCANNED_ELFS["${elf}"]=1

    while IFS= read -r soname; do
      [ -n "${soname}" ] || continue
      if [ -f "${LIB_STAGE}/private/${soname}" ]; then
        continue
      fi
      for dir in "${SEARCH_DIRS[@]}"; do
        if [ -f "${dir}/${soname}" ]; then
          cp -L "${dir}/${soname}" "${LIB_STAGE}/private/${soname}"
          ELF_QUEUE+=("${LIB_STAGE}/private/${soname}")
          break
        fi
      done
    done < <(
      LC_ALL=C "${READELF_BIN}" -d "${elf}" 2>/dev/null \
        | sed -n 's/.*(NEEDED).*Shared library: \[\(.*\)\].*/\1/p' || true
    )
  done
fi

rm -f "${PLANE_DIR}"/lib/*
cp -a "${LIB_STAGE}/private/." "${PLANE_DIR}/lib/"

# 按 SONAME 分发构建时链接的算法库，保证与公共接口匹配。
install -m 0644 "${MIDCOURSE_ALG_LIB}" "${PLANE_DIR}/lib/libmidcourse_alg.so"

rm -f "${LDD_SNAPSHOT}"
rm -rf -- "${LIB_STAGE}"
trap - EXIT

# 运行期库搜索路径：主程序使用相对 $ORIGIN。
CURRENT_RPATH=""
if [ -n "${READELF_BIN}" ]; then
  CURRENT_RPATH="$(LC_ALL=C "${READELF_BIN}" -d "${PLANE_DIR}/plane" 2>/dev/null \
    | sed -n 's/.*(\(RPATH\|RUNPATH\)).*\[\(.*\)\].*/\2/p' | head -n 1 || true)"
fi
if [ "${CURRENT_RPATH}" != '$ORIGIN/lib' ]; then
  if command -v patchelf >/dev/null 2>&1; then
    patchelf --set-rpath '$ORIGIN/lib' "${PLANE_DIR}/plane"
  else
    echo "错误: ${PLANE_DIR}/plane 的 RPATH/RUNPATH 为 '${CURRENT_RPATH}'（期望 '\$ORIGIN/lib'），且未安装 patchelf" >&2
    exit 1
  fi
fi

echo "已组装独立运行目录: ${PLANE_DIR}"
