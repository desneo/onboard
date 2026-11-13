#!/usr/bin/env bash
# 在独立运行目录安装或卸载开机自启服务。
set -euo pipefail

usage() {
    printf '用法: sudo %s {install|uninstall}\n' "$0"
}

if [[ "$#" == 1 && ( "$1" == -h || "$1" == --help ) ]]; then
    usage
    exit 0
fi
if [[ "$#" != 1 || ( "$1" != install && "$1" != uninstall ) ]]; then
    usage >&2
    exit 1
fi
if (( EUID != 0 )); then
    echo '错误: 请使用 sudo 执行安装或卸载' >&2
    exit 1
fi

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
runtime_dir=$(pwd -P)
source ./runplane.sh

core_dir="$runtime_dir/logs/core"
core_config=/etc/sysctl.d/99-zz-plane-core.conf
core_pattern="${core_dir//%/%%}/core.%e.%p.%t"
core_assignment="kernel.core_pattern = $core_pattern"
previous_prefix='# 原 kernel.core_pattern = '

exec 9>/run/plane-service-install.lock
flock -x 9
plane_service_read_state

managed=0
if plane_service_owns_file &&
    [[ "$plane_service_load" == loaded && "$plane_service_fragment" == "$plane_service_file" ]]; then
    managed=1
    if [[ "$plane_service_dir" != "$runtime_dir" ]]; then
        plane_service_error "服务已注册到 ${plane_service_dir}，请在该目录操作"
    fi
elif [[ "$plane_service_load" != not-found || -e "$plane_service_file" || -L "$plane_service_file" ]]; then
    plane_service_error "存在同名服务 ${plane_service_unit}，请检查系统注册"
fi

IFS= read -r current_core < /proc/sys/kernel/core_pattern
previous_core=$current_core
core_managed=0
if [[ -e "$core_config" || -L "$core_config" ]]; then
    if [[ ! -f "$core_config" || -L "$core_config" ]] ||
        ! grep -Fxq -- "$plane_service_marker" "$core_config" ||
        ! grep -Fxq -- "$core_assignment" "$core_config" ||
        ! grep -Fq -- "$previous_prefix" "$core_config"; then
        plane_service_error "${core_config} 由外部管理或指向其他运行目录"
    fi
    previous_core=$(sed -n "s/^${previous_prefix}//p" "$core_config")
    core_managed=1
fi

if [[ "$1" == uninstall ]]; then
    if (( managed )); then
        systemctl disable --now "$plane_service_unit"
    fi
    if (( core_managed )); then
        if [[ "$current_core" == "$core_pattern" ]]; then
            sysctl -q -w "kernel.core_pattern=$previous_core"
        fi
        rm -- "$core_config"
    fi
    if (( managed )); then
        rm -- "$plane_service_file"
        systemctl daemon-reload
    fi
    printf '%s 已卸载\n' "$plane_service_unit"
    exit 0
fi

if ! plane_service_is_running && pgrep -x plane >/dev/null; then
    plane_service_error '已有手动运行的 plane，请先执行 stopplane.sh'
fi
if [[ "$runtime_dir" == *[[:cntrl:]]* || "$runtime_dir" == *[[:space:]] ]]; then
    plane_service_error '运行目录路径包含控制字符或以空白结尾'
fi
core_bytes=$(printf '%s' "$core_pattern" | wc -c)
if (( core_bytes > 127 )); then
    plane_service_error '运行目录过长，kernel.core_pattern 最多支持 127 字节'
fi
if [[ "$current_core" == '|'* ]]; then
    plane_service_error "当前板端配置了 core 管道处理器，kernel.core_pattern=${current_core}"
fi
if (( core_managed )) && [[ "$current_core" != "$core_pattern" && "$current_core" != "$previous_core" ]]; then
    plane_service_error "kernel.core_pattern 已由外部修改，当前值为 ${current_core}"
fi

# 按 systemd 实际读取的文件顺序识别冲突，忽略已被同名文件覆盖的配置。
sysctl_config=$(/lib/systemd/systemd-sysctl --cat-config)
conflict=$(awk -v managed_name="${core_config##*/}" '
    /^# \/.*\.conf$/ { file = substr($0, 3); name = file; sub(/^.*\//, "", name) }
    /^[[:space:]]*-?kernel[.\/]core_pattern[[:space:]]*=/ && name != managed_name {
        if (name > managed_name || $0 ~ /=[[:space:]]*\|/) print file
    }
' <<< "$sysctl_config")
# procps 也支持直接加载 /etc/sysctl.conf，该文件可能未链接到 sysctl.d。
if [[ -f /etc/sysctl.conf && ! /etc/sysctl.conf -ef /etc/sysctl.d/99-sysctl.conf ]] &&
    grep -Eq '^[[:space:]]*-?kernel[./]core_pattern[[:space:]]*=' /etc/sysctl.conf; then
    conflict="${conflict:+$conflict$'\n'}/etc/sysctl.conf"
fi
if [[ -n "$conflict" ]]; then
    plane_service_error "以下系统配置会接管或覆盖 kernel.core_pattern: ${conflict}"
fi

service_user=${plane_service_user:-${SUDO_USER:-root}}
if [[ "$service_user" == root ]]; then
    service_user=$(stat -c %U -- .)
fi
if ! service_uid=$(id -u -- "$service_user") || (( service_uid == 0 )); then
    plane_service_error '请从非 root 部署账号使用 sudo 安装，或将运行目录交给部署账号'
fi
if ! runuser -u "$service_user" -- /bin/bash -c '
    [[ -r config/plane.ini && -r runplane.sh && -f plane && ( -x plane || -O plane ) ]] &&
        mkdir -p -- logs && [[ -w logs && ! -L logs/core ]] &&
        mkdir -p -m 0700 -- logs/core && [[ -O logs/core ]] && chmod 0700 -- logs/core
'; then
    plane_service_error "账号 ${service_user} 无法使用当前运行目录、plane、config/plane.ini 或 logs/core"
fi

staging=$(mktemp -d)
unit_changed=0
config_created=0
core_applied=0
service_started=0
rollback() {
    local rc=$?
    if (( rc != 0 )); then
        set +e
        if (( service_started )); then
            systemctl stop "$plane_service_unit"
            if [[ "$plane_service_enabled" != enabled ]]; then
                systemctl disable "$plane_service_unit"
            fi
        fi
        if (( core_applied )); then
            if ! sysctl -q -w "kernel.core_pattern=$current_core"; then
                plane_service_error '恢复安装前的 core 路由失败'
            fi
        fi
        if (( config_created )); then
            rm -f -- "$core_config"
        fi
        if (( unit_changed )); then
            if (( managed )); then
                install -m 0644 -- "$staging/previous.service" "$plane_service_file"
            else
                rm -f -- "$plane_service_file"
            fi
            systemctl daemon-reload
        fi
    fi
    rm -rf -- "$staging"
}
trap rollback EXIT

unit_dir=${runtime_dir//%/%%}
cat > "$staging/$plane_service_unit" <<EOF
${plane_service_marker}
[Unit]
Description=机载程序
Wants=network-online.target
After=network-online.target
StartLimitIntervalSec=60s
StartLimitBurst=5

[Service]
Type=exec
User=${service_user}
WorkingDirectory=${unit_dir}
ExecStart=/bin/bash ./runplane.sh --systemd
Restart=on-failure
RestartSec=5s
KillSignal=SIGINT
TimeoutStopSec=10s
LimitCORE=infinity
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF
if ! cmp -s -- "$staging/$plane_service_unit" "$plane_service_file"; then
    if plane_service_is_running; then
        plane_service_error '升级服务前请先执行 stopplane.sh'
    fi
    systemd-analyze verify "$staging/$plane_service_unit"
    if (( managed )); then
        cp -- "$plane_service_file" "$staging/previous.service"
    fi
    unit_changed=1
    install -m 0644 -- "$staging/$plane_service_unit" "$plane_service_file"
fi

if (( ! core_managed )); then
    printf '%s\n%s%s\n%s\n' "$plane_service_marker" "$previous_prefix" "$previous_core" "$core_assignment" > "$staging/core.conf"
    config_created=1
    install -m 0644 -- "$staging/core.conf" "$core_config"
fi
if [[ "$current_core" != "$core_pattern" ]]; then
    core_applied=1
    sysctl -q -w "kernel.core_pattern=$core_pattern"
fi
if ! plane_service_is_running; then
    service_started=1
fi
if ! systemctl enable --now "$plane_service_unit"; then
    plane_service_error "启用或启动失败，请查看 journalctl -u ${plane_service_unit}"
fi
printf '%s 已启用开机自启并启动\n' "$plane_service_unit"
