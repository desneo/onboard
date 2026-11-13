#!/usr/bin/env bash
# 直接启动机载程序，或通过 --service 启动已安装的 systemd 服务。

plane_service_unit=plane.service
plane_service_file=/etc/systemd/system/plane.service
plane_service_marker='# 由 install-plane-service.sh 管理'

plane_service_error() {
    printf '错误: %s\n' "$*" >&2
    return 1
}

plane_service_read_state() {
    local properties property value
    plane_service_load=
    plane_service_fragment=
    plane_service_dir=
    plane_service_enabled=
    plane_service_active=
    plane_service_job=
    plane_service_user=
    properties=$(systemctl show "$plane_service_unit" \
        -p LoadState -p FragmentPath -p WorkingDirectory -p UnitFileState -p ActiveState -p Job -p User) || return
    while IFS='=' read -r property value; do
        case "$property" in
            LoadState) plane_service_load=$value ;;
            FragmentPath) plane_service_fragment=$value ;;
            WorkingDirectory) plane_service_dir=$value ;;
            UnitFileState) plane_service_enabled=$value ;;
            ActiveState) plane_service_active=$value ;;
            Job) plane_service_job=$value ;;
            User) plane_service_user=$value ;;
        esac
    done <<< "$properties"
}

plane_service_owns_file() {
    [[ -f "$plane_service_file" && ! -L "$plane_service_file" ]] &&
        grep -Fxq -- "$plane_service_marker" "$plane_service_file"
}

plane_service_is_running() {
    case "$plane_service_active" in
        active|activating|deactivating|reloading) return 0 ;;
    esac
    [[ -n "$plane_service_job" && "$plane_service_job" != 0 ]]
}

plane_service_status() {
    # 直接运行的环境（例如容器）可以没有 systemd。
    [[ -e "$plane_service_file" || -L "$plane_service_file" ]] || return 1
    if ! plane_service_read_state; then
        plane_service_error "无法读取 ${plane_service_unit} 状态"
        return 2
    fi
    if [[ "$plane_service_load" != loaded || "$plane_service_fragment" != "$plane_service_file" ]] ||
        ! plane_service_owns_file; then
        plane_service_error "${plane_service_unit} 的注册与当前安装脚本不匹配"
        return 2
    fi
}

plane_service_control() {
    if (( EUID == 0 )); then
        systemctl --no-ask-password "$1" "$plane_service_unit"
    else
        sudo -- systemctl --no-ask-password "$1" "$plane_service_unit"
    fi
}

runplane_usage() {
    printf '用法: %s [plane_dir]\n' "$0"
    printf '      %s --service [plane_dir]\n' "$0"
}

runplane_main() {
    local mode=foreground script_dir source_root runtime_dir requested_dir uptime running_pids
    local -a plane_args=()
    case "${1:-}" in
        -h|--help) runplane_usage; return 0 ;;
        --service) mode=service; shift ;;
        --systemd) mode=systemd; shift ;;
        -*) plane_service_error "未知选项 $1"; return 1 ;;
    esac
    if (( $# > 1 )); then
        runplane_usage >&2
        return 1
    fi

    if [[ "$mode" != systemd ]]; then
        if plane_service_status; then
            if [[ "$mode" == service ]]; then
                if [[ -n "${1:-}" ]]; then
                    requested_dir=$(realpath -e -- "$1")
                    if [[ "$requested_dir" != "$plane_service_dir" ]]; then
                        plane_service_error "指定目录 ${requested_dir} 与服务目录 ${plane_service_dir} 不一致"
                        return 1
                    fi
                fi
                if ! plane_service_is_running && pgrep -x plane >/dev/null; then
                    plane_service_error '已有手动运行的 plane，请先执行 stopplane.sh'
                    return 1
                fi
                plane_service_control start
                return
            fi
            if plane_service_is_running; then
                plane_service_error 'plane 服务正在运行或准备启动，请先执行 stopplane.sh'
                return 1
            fi
        elif [[ "$?" != 1 ]]; then
            return 1
        elif [[ "$mode" == service ]]; then
            plane_service_error '尚未安装 plane.service，请执行 install-plane-service.sh install'
            return 1
        fi
    else
        if [[ -z "${INVOCATION_ID:-}" ]]; then
            plane_service_error '--systemd 由 plane.service 使用'
            return 1
        fi
        # 使用系统运行时间，异常恢复时无需再次等待开机延迟。
        read -r uptime _ < /proc/uptime
        uptime=${uptime%%.*}
        if (( uptime < 10 )); then
            sleep "$((10 - uptime))"
        fi
        plane_args+=(--no-console)
    fi

    script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
    source_root=$(cd -- "$script_dir/.." && pwd)
    if [[ -n "${1:-}" ]]; then
        runtime_dir=$1
    elif [[ "$mode" == systemd || -f "$script_dir/plane" || -f "$script_dir/config/plane.ini" ]]; then
        runtime_dir=$script_dir
    elif [[ -d "$source_root/build/plane" ]]; then
        runtime_dir="$source_root/build/plane"
    else
        plane_service_error '未找到 plane 运行目录，可用参数指定'
        return 1
    fi

    cd -- "$runtime_dir"
    [[ -r config/plane.ini ]] || { plane_service_error '无法读取 config/plane.ini'; return 1; }
    [[ -f plane ]] || { plane_service_error '未找到可执行文件 plane'; return 1; }
    mkdir -p -- logs
    # 锁随 exec 交给 plane，避免同一运行目录并发启动。
    exec 9>>logs/.plane.lock
    if ! flock -n 9; then
        if [[ "$mode" == systemd ]]; then
            plane_service_error '已有 plane 正在运行或启动'
            return 1
        fi
        echo '已有 plane 正在运行或启动。'
        return 0
    fi
    running_pids=$(pgrep -x plane || true)
    if [[ -n "$running_pids" ]]; then
        if [[ "$mode" == systemd ]]; then
            plane_service_error '已有 plane 进程，请先停止后再启动服务'
            return 1
        fi
        printf 'plane 已在运行 (PID=%s)。\n' "${running_pids//$'\n'/ }"
        return 0
    fi

    # 上传工具可能丢失执行权限，启动前补齐。
    [[ -x plane ]] || chmod +x -- plane
    if [[ -f stopplane.sh && ! -x stopplane.sh ]]; then
        chmod +x -- stopplane.sh
    fi
    exec ./plane "${plane_args[@]}"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    set -euo pipefail
    runplane_main "$@"
fi
