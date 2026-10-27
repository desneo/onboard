#!/usr/bin/env python3
"""PX4 SITL 与多块机载板的可重复测试编排工具。"""

from __future__ import annotations

import argparse
import configparser
import dataclasses
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import selectors
import shlex
import shutil
import stat
import subprocess
import sys
import tempfile
import time
from typing import Iterable, Sequence
from zoneinfo import ZoneInfo


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config" / "fleet.ini"
PEER_LINK_RE = re.compile(r"他机(上线|下线):\s*([^\s]+)")
LOG_TIME_RE = re.compile(r"^\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\]")
ANSI_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
URI_RE = re.compile(r"(?:udp(?:out|in)?|tcp)://[^\s\"'<>\[\]{}(),;]+", re.IGNORECASE)
PORT_RE = re.compile(r":(\d+)(?:[/?:#]|$)")


class FleetError(RuntimeError):
    """可向用户直接展示的编排错误。"""


@dataclasses.dataclass(frozen=True)
class RemoteHost:
    host: str
    user: str
    password: str

    @property
    def target(self) -> str:
        return f"{self.user}@{self.host}"


@dataclasses.dataclass(frozen=True)
class Simulator(RemoteHost):
    scenario_dir: str
    start_command: str
    stop_command: str
    flight_port_base: int | None


@dataclasses.dataclass(frozen=True)
class Plane(RemoteHost):
    plane_id: str
    runtime_dir: str


@dataclasses.dataclass(frozen=True)
class FleetConfig:
    simulator: Simulator
    planes: tuple[Plane, ...]
    startup_timeout_s: int
    verify_timeout_s: int
    board_arch: str
    board_os_version: str
    apt_proxy: str
    apt_bootstrap_packages: tuple[str, ...]
    apt_packages: tuple[str, ...]


@dataclasses.dataclass(frozen=True)
class EndpointSet:
    flight_urls: tuple[str, ...]
    all_uris: tuple[str, ...]


@dataclasses.dataclass
class PlaneResult:
    plane_id: str
    host: str
    process_running: bool = False
    flight_connected: bool = False
    peer_online: dict[str, bool] = dataclasses.field(default_factory=dict)
    errors: list[str] = dataclasses.field(default_factory=list)

def now_shanghai() -> dt.datetime:
    return dt.datetime.now(ZoneInfo("Asia/Shanghai"))


def new_run_id() -> str:
    return now_shanghai().strftime("%Y%m%d-%H%M%S")


def load_config(path: Path) -> FleetConfig:
    parser = configparser.ConfigParser(interpolation=None)
    if not path.is_file():
        raise FleetError(f"环境清单不存在: {path}")
    parser.read(path, encoding="utf-8")

    try:
        fleet = parser["fleet"]
        sim = parser["simulator"]
        runtime_dir = fleet.get("runtime_dir", "/home/firefly/plane")
        simulator = Simulator(
            host=sim["host"].strip(),
            user=sim["user"].strip(),
            password=sim["password"],
            scenario_dir=sim["scenario_dir"].strip(),
            start_command=sim.get("start_command", "./start").strip(),
            stop_command=sim.get("stop_command", "./stop.sh").strip(),
            flight_port_base=sim.getint("flight_port_base", fallback=None),
        )
    except (KeyError, ValueError) as exc:
        raise FleetError(f"环境清单缺少 simulator/fleet 字段: {exc}") from exc

    planes: list[Plane] = []
    for section in parser.sections():
        if not section.startswith("plane."):
            continue
        plane_id = section.removeprefix("plane.").strip()
        item = parser[section]
        if not plane_id:
            raise FleetError(f"非法飞机 section: {section}")
        try:
            planes.append(
                Plane(
                    plane_id=plane_id,
                    host=item["host"].strip(),
                    user=item["user"].strip(),
                    password=item["password"],
                    runtime_dir=item.get("runtime_dir", runtime_dir).strip(),
                )
            )
        except KeyError as exc:
            raise FleetError(f"{section} 缺少字段: {exc}") from exc
    if not planes:
        raise FleetError("环境清单未声明任何 plane.<id>")
    if len({plane.plane_id for plane in planes}) != len(planes):
        raise FleetError("环境清单存在重复 plane_id")
    if len({plane.host for plane in planes}) != len(planes):
        raise FleetError("环境清单存在重复板卡地址；当前约束是一架飞机对应一块板")

    return FleetConfig(
        simulator=simulator,
        planes=tuple(planes),
        startup_timeout_s=fleet.getint("startup_timeout_s", 60),
        verify_timeout_s=fleet.getint("verify_timeout_s", 45),
        board_arch=fleet.get("board_arch", "aarch64").strip(),
        board_os_version=fleet.get("board_os_version", "22.04").strip(),
        apt_proxy=fleet.get("apt_proxy", "").strip(),
        apt_bootstrap_packages=tuple(
            item
            for item in re.split(r"[\s,]+", fleet.get("apt_bootstrap_packages", ""))
            if item
        ),
        apt_packages=tuple(
            item for item in re.split(r"[\s,]+", fleet.get("apt_packages", "")) if item
        ),
    )


class RemoteExecutor:
    def __init__(self, state_dir: Path):
        state_dir.mkdir(parents=True, exist_ok=True)
        self.known_hosts = state_dir / "known_hosts"
        self.askpass = state_dir / "askpass.sh"
        self.askpass.write_text('#!/bin/sh\nprintf "%s\\n" "$FLEET_SSH_PASSWORD"\n', encoding="utf-8")
        self.askpass.chmod(stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR)

    def _env(self, password: str) -> dict[str, str]:
        env = os.environ.copy()
        env.update(
            {
                "DISPLAY": "fleetctl:0",
                "SSH_ASKPASS": str(self.askpass),
                "SSH_ASKPASS_REQUIRE": "force",
                "FLEET_SSH_PASSWORD": password,
            }
        )
        return env

    def _ssh_options(self) -> list[str]:
        return [
            "-T",
            "-o",
            "BatchMode=no",
            "-o",
            "NumberOfPasswordPrompts=1",
            "-o",
            "StrictHostKeyChecking=accept-new",
            "-o",
            f"UserKnownHostsFile={self.known_hosts}",
            "-o",
            "ConnectTimeout=8",
            "-o",
            "ServerAliveInterval=5",
            "-o",
            "ServerAliveCountMax=3",
        ]

    def run(
        self,
        remote: RemoteHost,
        script: str,
        *,
        timeout: int = 30,
        check: bool = True,
        input_text: str | None = None,
    ) -> subprocess.CompletedProcess[str]:
        command = ["ssh", *self._ssh_options(), remote.target, "bash -lc " + shlex.quote(script)]
        try:
            result = subprocess.run(
                command,
                stdin=subprocess.DEVNULL if input_text is None else None,
                input=input_text,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=timeout,
                env=self._env(remote.password),
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise FleetError(f"SSH 命令超时: {remote.target} ({timeout}s)") from exc
        if check and result.returncode != 0:
            detail = sanitize_output(result.stdout).strip()
            raise FleetError(f"SSH 命令失败: {remote.target}, rc={result.returncode}\n{detail}")
        return result

    def run_sudo(
        self,
        remote: RemoteHost,
        script: str,
        *,
        timeout: int = 30,
        check: bool = True,
    ) -> subprocess.CompletedProcess[str]:
        command = "sudo -S -p '' -- bash -lc " + shlex.quote(script)
        return self.run(
            remote,
            command,
            timeout=timeout,
            check=check,
            input_text=remote.password + "\n",
        )

    def stream(
        self,
        remote: RemoteHost,
        script: str,
        *,
        label: str,
        timeout: int,
    ) -> tuple[int, str]:
        command = ["ssh", *self._ssh_options(), remote.target, "bash -lc " + shlex.quote(script)]
        process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
            env=self._env(remote.password),
        )
        assert process.stdout is not None
        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ)
        output: list[str] = []
        deadline = time.monotonic() + timeout
        try:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    process.kill()
                    process.wait(timeout=5)
                    raise FleetError(f"{label} 命令超时 ({timeout}s)，已终止本地 SSH 会话")
                events = selector.select(timeout=min(1.0, remaining))
                for key, _ in events:
                    line = key.fileobj.readline()
                    if line:
                        clean = sanitize_output(line).rstrip()
                        output.append(clean + "\n")
                        print(f"[{label}] {clean}", flush=True)
                if process.poll() is not None:
                    remainder = process.stdout.read()
                    if remainder:
                        for line in sanitize_output(remainder).splitlines():
                            output.append(line + "\n")
                            print(f"[{label}] {line}", flush=True)
                    break
        finally:
            selector.close()
        return process.returncode or 0, "".join(output)

    def rsync_to(
        self, remote: RemoteHost, source: str | Path, destination: str, extra: Sequence[str] = ()
    ) -> None:
        ssh_command = " ".join(shlex.quote(item) for item in ["ssh", *self._ssh_options()])
        command = ["rsync", "-a", *extra, "-e", ssh_command, str(source), f"{remote.target}:{destination}"]
        result = subprocess.run(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            env=self._env(remote.password),
            check=False,
        )
        if result.returncode != 0:
            raise FleetError(
                f"rsync 失败: {remote.target}:{destination}, rc={result.returncode}\n"
                f"{sanitize_output(result.stdout).strip()}"
            )


def sanitize_output(text: str) -> str:
    return ANSI_RE.sub("", text or "")


def selected_planes(config: FleetConfig, count: int) -> tuple[Plane, ...]:
    if count <= 0:
        raise FleetError("飞机数量必须大于 0")
    if count > len(config.planes):
        declared = ", ".join(f"{plane.plane_id}({plane.host})" for plane in config.planes)
        raise FleetError(
            f"环境不满足：请求 {count} 架飞机，但 config/fleet.ini 仅声明 {len(config.planes)} 块机载板。\n"
            f"已声明: {declared}\n本次不会启动 SITL 或机载程序。"
        )
    return config.planes[:count]


def require_local_commands(commands: Iterable[str]) -> None:
    missing = [command for command in commands if shutil.which(command) is None]
    if missing:
        raise FleetError("代码机缺少命令: " + ", ".join(missing))


def bundle_problems(bundle: Path) -> list[str]:
    required_files = ["plane", "runplane.sh", "stopplane.sh", "config/plane.conf"]
    required_dirs = ["lib"]
    problems = [name for name in required_files if not (bundle / name).is_file()]
    problems.extend(name + "/" for name in required_dirs if not (bundle / name).is_dir())
    return problems


def ensure_bundle() -> Path:
    bundle = ROOT / "build" / "plane"
    problems = bundle_problems(bundle)
    if not problems:
        print(f"[build] 使用现有完整运行包: {bundle}")
        return bundle

    print("[build] 运行包不完整，开始配置、编译并打包: " + ", ".join(problems), flush=True)
    commands = [
        ["cmake", "-S", str(ROOT), "-B", str(ROOT / "build"), "-DCMAKE_BUILD_TYPE=Release"],
        ["cmake", "--build", str(ROOT / "build"), "-j"],
    ]
    for command in commands:
        result = subprocess.run(command, cwd=ROOT, check=False)
        if result.returncode != 0:
            raise FleetError(f"编译失败: {shlex.join(command)}")
    problems = bundle_problems(bundle)
    if problems:
        raise FleetError("编译完成后运行包仍不完整: " + ", ".join(problems))
    return bundle


def preflight(
    config: FleetConfig, planes: Sequence[Plane], remote: RemoteExecutor, *, include_simulator: bool = True
) -> None:
    require_local_commands(["python3", "ssh", "rsync", "cmake"])
    sim = config.simulator
    sim_script = (
        f"if [ ! -d {shlex.quote(sim.scenario_dir)} ]; then "
        f"echo '缺少 SITL 目录: {shlex.quote(sim.scenario_dir)}'; exit 11; fi; "
        f"cd {shlex.quote(sim.scenario_dir)}; "
        f"if [ ! -x {shlex.quote(sim.start_command)} ]; then "
        f"echo '启动脚本不存在或不可执行: {shlex.quote(sim.start_command)}'; ls -la; exit 12; fi; "
        f"if [ ! -x {shlex.quote(sim.stop_command)} ]; then "
        f"echo '停止脚本不存在或不可执行: {shlex.quote(sim.stop_command)}'; ls -la; exit 13; fi; "
        "printf 'ok arch=%s dir=%s\\n' \"$(uname -m)\" \"$(pwd)\""
    )
    if include_simulator:
        print(f"[preflight] 检查 SITL 主机 {sim.target}", flush=True)
        print("[preflight] " + remote.run(sim, sim_script).stdout.strip(), flush=True)
    for plane in planes:
        print(f"[preflight] 检查 {plane.plane_id} {plane.target}")
        expected_arch = shlex.quote(config.board_arch)
        expected_os = shlex.quote(config.board_os_version)
        script = (
            "command -v rsync >/dev/null && "
            "command -v apt-get >/dev/null && "
            "command -v sudo >/dev/null && "
            "actual_arch=$(uname -m) && "
            "actual_os=$(. /etc/os-release && printf '%s' \"$VERSION_ID\") && "
            f"if [ \"$actual_arch\" != {expected_arch} ]; then "
            f"echo '环境不满足: 期望架构 {config.board_arch}，实际架构' \"$actual_arch\"; exit 14; fi; "
            f"if [ \"$actual_os\" != {expected_os} ]; then "
            f"echo '环境不满足: 期望 Ubuntu {config.board_os_version}，实际版本' \"$actual_os\"; exit 15; fi; "
            "printf 'ok arch=%s os=%s free_kb=%s\\n' "
            "\"$actual_arch\" \"$actual_os\" \"$(df -Pk /home | awk 'NR==2 {print $4}')\"; "
            f"if [ -x {shlex.quote(plane.runtime_dir + '/plane')} ]; then "
            f"cd {shlex.quote(plane.runtime_dir)} && "
            "for target in ./plane; do "
            "[ -e \"$target\" ] || continue; "
            "ldd \"$target\" 2>/dev/null | awk -v target=\"$target\" '/not found/ {print target \": \" $1 \": not found\"}'; "
            "done; fi"
        )
        print("[preflight] " + remote.run(plane, script).stdout.strip())


def missing_apt_packages(
    plane: Plane, packages: Sequence[str], remote: RemoteExecutor
) -> tuple[str, ...]:
    if not packages:
        return ()
    package_args = " ".join(shlex.quote(package) for package in packages)
    script = (
        f"for package in {package_args}; do "
        "status=$(dpkg-query -W -f='${Status}' \"$package\" 2>/dev/null || true); "
        "[ \"$status\" = 'install ok installed' ] || printf '%s\\n' \"$package\"; "
        "done"
    )
    result = remote.run(plane, script, timeout=20)
    return tuple(line.strip() for line in result.stdout.splitlines() if line.strip())


def configure_apt_proxy(config: FleetConfig, plane: Plane, remote: RemoteExecutor) -> None:
    proxy = config.apt_proxy
    if not proxy:
        return
    if not re.fullmatch(r"https?://[^\s\";]+", proxy):
        raise FleetError(f"apt_proxy 格式不安全或不受支持: {proxy!r}")
    content = (
        f'Acquire::http::Proxy "{proxy}";\n'
        f'Acquire::https::Proxy "{proxy}";\n'
    )
    script = (
        "umask 022; "
        f"printf '%s' {shlex.quote(content)} > /etc/apt/apt.conf.d/99zz-fleet-proxy; "
        "rm -f /etc/apt/apt.conf.d/99fleet-proxy"
    )
    remote.run_sudo(plane, script, timeout=20)
    print(f"[deps] {plane.plane_id}: APT 全局代理={proxy}")


def apt_update(plane: Plane, remote: RemoteExecutor) -> None:
    print(f"[deps] {plane.plane_id}: apt-get update", flush=True)
    result = remote.run_sudo(
        plane,
        "timeout --signal=TERM 840s apt-get update",
        timeout=900,
        check=False,
    )
    if result.returncode != 0:
        raise FleetError(
            f"{plane.plane_id} apt-get update 失败, rc={result.returncode}:\n"
            f"{sanitize_output(result.stdout).strip()}"
        )


def apt_install(
    plane: Plane, packages: Sequence[str], remote: RemoteExecutor, *, label: str
) -> None:
    if not packages:
        return
    package_args = " ".join(shlex.quote(package) for package in packages)
    print(f"[deps] {plane.plane_id}: {label}: {' '.join(packages)}", flush=True)
    script = (
        "export DEBIAN_FRONTEND=noninteractive; "
        f"timeout --signal=TERM 840s apt-get install -y --no-install-recommends {package_args}"
    )
    result = remote.run_sudo(plane, script, timeout=900, check=False)
    if result.returncode != 0:
        raise FleetError(
            f"{plane.plane_id} APT 安装失败, rc={result.returncode}:\n"
            f"{sanitize_output(result.stdout).strip()}"
        )


def install_system_dependencies(
    config: FleetConfig, plane: Plane, remote: RemoteExecutor
) -> None:
    configure_apt_proxy(config, plane, remote)
    bootstrap_missing = missing_apt_packages(plane, config.apt_bootstrap_packages, remote)
    regular_missing = missing_apt_packages(plane, config.apt_packages, remote)
    if not bootstrap_missing and not regular_missing:
        print(f"[deps] {plane.plane_id}: APT 依赖已完整")
        return

    apt_update(plane, remote)
    # 先安装基础包，再补装其他运行依赖。
    apt_install(plane, bootstrap_missing, remote, label="安装基础依赖")
    regular_missing = missing_apt_packages(plane, config.apt_packages, remote)
    apt_install(plane, regular_missing, remote, label="补装运行依赖")

    unresolved = missing_apt_packages(
        plane, (*config.apt_bootstrap_packages, *config.apt_packages), remote
    )
    if unresolved:
        raise FleetError(f"{plane.plane_id} APT 安装后仍缺少: {', '.join(unresolved)}")
    print(f"[deps] {plane.plane_id}: APT 依赖安装完成")


def stop_planes(planes: Sequence[Plane], remote: RemoteExecutor) -> None:
    for plane in planes:
        runtime = shlex.quote(plane.runtime_dir)
        script = (
            f"if [ -x {runtime}/stopplane.sh ]; then cd {runtime} && ./stopplane.sh 10; "
            "elif pgrep -x plane >/dev/null 2>&1; then "
            "echo '存在 plane 进程但缺少 stopplane.sh' >&2; exit 2; "
            "else echo 'plane 未在运行，且运行目录尚不存在'; fi"
        )
        print(f"[stop] {plane.plane_id}")
        result = remote.run(plane, script, timeout=20, check=False)
        if result.stdout.strip():
            print(f"[{plane.plane_id}] {result.stdout.strip()}")
        if result.returncode != 0:
            raise FleetError(f"停止 {plane.plane_id} 失败，拒绝覆盖运行目录")


def stop_simulator(config: FleetConfig, remote: RemoteExecutor) -> None:
    sim = config.simulator
    script = f"cd {shlex.quote(sim.scenario_dir)} && {sim.stop_command}"
    print("[sitl] 停止旧仿真环境")
    result = remote.run(sim, script, timeout=30, check=False)
    if result.stdout.strip():
        for line in sanitize_output(result.stdout).splitlines():
            print(f"[sitl] {line}")
    if result.returncode != 0:
        raise FleetError(f"SITL stop 失败: rc={result.returncode}")


def endpoint_port(uri: str) -> int | None:
    match = PORT_RE.search(uri)
    return int(match.group(1)) if match else None


def validate_flight_listeners(base_port: int, count: int, socket_text: str) -> tuple[int, ...]:
    if base_port <= 0 or base_port + count - 1 > 65535:
        raise FleetError(f"非法飞控端口范围: {base_port}～{base_port + count - 1}")
    expected = tuple(range(base_port, base_port + count))
    listening = {int(port) for port in re.findall(r":(\d+)\s", socket_text)}
    missing = [port for port in expected if port not in listening]
    if missing:
        raise FleetError(
            f"SITL 未监听声明的飞控端口: {', '.join(map(str, missing))}。"
            "不会修改板端配置。"
        )
    return expected


def parse_endpoints(
    text: str,
    simulator_host: str,
    count: int,
    flight_ports: Sequence[int] | None = None,
) -> EndpointSet:
    clean = sanitize_output(text)
    all_uris = tuple(dict.fromkeys(uri.rstrip(".，。") for uri in URI_RE.findall(clean)))
    if flight_ports is None:
        parsed_flight_ports = sorted(
            {
                port
                for uri in all_uris
                if uri.lower().startswith(("udp://", "udpin://", "udpout://"))
                for port in [endpoint_port(uri)]
                if port is not None and 14200 <= port <= 14299
            }
        )
        if len(parsed_flight_ports) != count:
            raise FleetError(
                f"无法唯一解析 {count} 个飞控端点：发现端口 {parsed_flight_ports or '无'}。"
                "已保存 SITL 原始输出，不会修改板端配置。"
            )
    else:
        parsed_flight_ports = list(flight_ports)
        if len(parsed_flight_ports) != count or len(set(parsed_flight_ports)) != count:
            raise FleetError(f"飞控端口数量或唯一性不满足 {count} 架飞机要求")
    flight_urls = tuple(f"udpout://{simulator_host}:{port}" for port in parsed_flight_ports)

    return EndpointSet(flight_urls=flight_urls, all_uris=all_uris)


def start_simulator(
    config: FleetConfig,
    count: int,
    remote: RemoteExecutor,
    run_dir: Path,
) -> EndpointSet:
    sim = config.simulator
    command = f"cd {shlex.quote(sim.scenario_dir)} && {sim.start_command} -n {count}"
    print(f"[sitl] 启动 {count} 架仿真飞机")
    rc, output = remote.stream(sim, command, label="sitl", timeout=config.startup_timeout_s)
    (run_dir / "sitl-start.log").write_text(output, encoding="utf-8")
    if rc != 0:
        raise FleetError(f"SITL start 失败: rc={rc}，输出见 {run_dir / 'sitl-start.log'}")
    flight_ports = None
    if sim.flight_port_base is not None:
        listeners = remote.run(sim, "ss -H -lun", timeout=15).stdout
        (run_dir / "sitl-udp-listeners.log").write_text(listeners, encoding="utf-8")
        flight_ports = validate_flight_listeners(sim.flight_port_base, count, listeners)
    try:
        endpoints = parse_endpoints(output, sim.host, count, flight_ports)
    except FleetError:
        probe = remote.run(
            sim,
            "ps -eo args | grep -E 'udp(out|in)?://|rtsp://|142[0-9][0-9]' | grep -v grep || true",
            timeout=15,
            check=False,
        ).stdout
        (run_dir / "sitl-processes.log").write_text(probe, encoding="utf-8")
        endpoints = parse_endpoints(output + "\n" + probe, sim.host, count, flight_ports)
    (run_dir / "endpoints.json").write_text(
        json.dumps(dataclasses.asdict(endpoints), ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    for index, url in enumerate(endpoints.flight_urls, start=1):
        print(f"[sitl] #{index}: flight={url}")
    return endpoints


def set_ini_value(text: str, section: str, key: str, value: str) -> str:
    lines = text.splitlines(keepends=True)
    section_re = re.compile(r"^\s*\[([^]]+)]\s*$")
    key_re = re.compile(rf"^\s*{re.escape(key)}\s*=")
    start = None
    end = len(lines)
    for index, line in enumerate(lines):
        match = section_re.match(line.strip("\r\n"))
        if not match:
            continue
        if start is not None:
            end = index
            break
        if match.group(1).strip() == section:
            start = index + 1
    replacement = f"{key} = {value}\n"
    if start is None:
        suffix = "" if not text or text.endswith("\n") else "\n"
        return text + suffix + f"\n[{section}]\n" + replacement
    for index in range(start, end):
        if key_re.match(lines[index]):
            lines[index] = replacement
            return "".join(lines)
    lines.insert(end, replacement)
    return "".join(lines)


def configure_plane(base: str, plane: Plane, flight_url: str) -> str:
    # 旧板端配置可能仍包含已移除的图像模块，生成配置时清除这些 section。
    lines = []
    retired = False
    for line in base.splitlines(keepends=True):
        section = re.match(r"^\s*\[([^]]+)\]", line)
        if section:
            retired = section.group(1).strip() in {"plugins", "image_input", "detection", "video_output", "debug"}
        if not retired:
            lines.append(line)
    result = "".join(lines)
    updates = [
        ("system", "log_dir", "logs"),
        ("communication", "plane_id", plane.plane_id),
        ("communication", "station_id", "GCS"),
        ("communication", "zenoh_config_file", ""),
        ("communication", "zenoh_connect", ""),
        ("communication", "zenoh_listen", ""),
        ("flight_control", "link_type", "udp"),
        ("flight_control", "udp_url", flight_url),
    ]
    for section, key, value in updates:
        result = set_ini_value(result, section, key, value)
    return result


def remote_config_or_default(
    plane: Plane, remote: RemoteExecutor, default_config: Path
) -> str:
    path = f"{plane.runtime_dir}/config/plane.conf"
    result = remote.run(plane, f"test -f {shlex.quote(path)} && cat {shlex.quote(path)} || true", check=False)
    if result.returncode == 0 and result.stdout.strip():
        return result.stdout
    return default_config.read_text(encoding="utf-8")


def sync_plane(
    plane: Plane,
    bundle: Path,
    flight_url: str,
    run_id: str,
    remote: RemoteExecutor,
    run_dir: Path,
) -> None:
    runtime = shlex.quote(plane.runtime_dir)
    config_path = f"{plane.runtime_dir}/config/plane.conf"
    base = remote_config_or_default(plane, remote, bundle / "config" / "plane.conf")
    rendered = configure_plane(base, plane, flight_url)
    local_config = run_dir / f"{plane.plane_id}.plane.conf"
    local_config.write_text(rendered, encoding="utf-8")

    prepare = (
        f"mkdir -p {runtime}/config {runtime}/logs && "
        f"if [ -f {shlex.quote(config_path)} ]; then "
        f"cp -a {shlex.quote(config_path)} {shlex.quote(config_path + '.fleet-backup-' + run_id)}; fi"
    )
    remote.run(plane, prepare)
    print(f"[sync] {plane.plane_id} -> {plane.host}:{plane.runtime_dir}")
    remote.rsync_to(
        plane,
        str(bundle) + "/",
        plane.runtime_dir.rstrip("/") + "/",
        extra=(
            "--delete",
            "--exclude=/logs/",
            "--exclude=/config/plane.conf",
            "--exclude=/config/plane.conf.fleet-backup-*",
        ),
    )
    remote.rsync_to(plane, local_config, config_path)


def check_remote_dependencies(plane: Plane, remote: RemoteExecutor) -> None:
    runtime = shlex.quote(plane.runtime_dir)
    script = (
        f"cd {runtime} && missing=0; "
        "for target in ./plane; do "
        "[ -e \"$target\" ] || continue; "
        "unresolved=$(ldd \"$target\" 2>/dev/null | awk '/not found/ {print $1}'); "
        "if [ -n \"$unresolved\" ]; then "
        "printf '%s: 缺少 %s\\n' \"$target\" \"$(printf '%s' \"$unresolved\" | tr '\\n' ' ')\"; missing=1; fi; "
        "done; exit \"$missing\""
    )
    result = remote.run(plane, script, timeout=20, check=False)
    if result.returncode != 0:
        raise FleetError(
            f"{plane.plane_id} 运行库检查失败，未启动程序:\n{sanitize_output(result.stdout).strip()}"
        )
    print(f"[deps] {plane.plane_id}: 可执行文件与插件依赖完整")


def start_plane(plane: Plane, remote: RemoteExecutor) -> None:
    runtime = shlex.quote(plane.runtime_dir)
    log_dir = "logs"
    script = (
        f"cd {runtime} && mkdir -p {shlex.quote(log_dir)} && "
        "command -v setsid >/dev/null && "
        f"setsid -f ./runplane.sh . >{shlex.quote(log_dir + '/plane.stdout.log')} 2>&1 </dev/null && "
        "printf 'launcher detached\\n'"
    )
    print(f"[start] {plane.plane_id}")
    result = remote.run(plane, script, timeout=10)
    print(f"[{plane.plane_id}] {result.stdout.strip()}")


def read_plane_logs(plane: Plane, remote: RemoteExecutor) -> str:
    base = f"{plane.runtime_dir}/logs"
    script = (
        f"for file in {shlex.quote(base + '/plane.stdout.log')} {shlex.quote(base + '/plane.log')}; do "
        "if [ -f \"$file\" ]; then printf '\\n===== %s =====\\n' \"$file\"; tail -n 5000 \"$file\"; fi; done"
    )
    return sanitize_output(remote.run(plane, script, timeout=10, check=False).stdout)


def analyze_plane(plane: Plane, peers: Sequence[Plane], text: str) -> PlaneResult:
    result = PlaneResult(plane_id=plane.plane_id, host=plane.host)
    # 两个日志文件可能重复或跨运行保留历史；只采信最新启动之后的状态变更。
    records = [(match.group(1), line) for line in text.splitlines() if (match := LOG_TIME_RE.match(line))]
    latest_start = max((timestamp for timestamp, line in records if "plane 启动:" in line), default="")
    current = sorted(
        ((timestamp, line) for timestamp, line in records if timestamp >= latest_start),
        key=lambda record: record[0],
    )
    current_text = "\n".join(line for _, line in current) if latest_start else text
    result.flight_connected = "飞控系统已接入" in current_text or "飞控连接建立" in current_text
    result.peer_online = {peer.plane_id: False for peer in peers if peer != plane}
    for _, line in current:
        if match := PEER_LINK_RE.search(line):
            status, peer_id = match.groups()
            if peer_id in result.peer_online:
                result.peer_online[peer_id] = status == "上线"
    result.errors = [
        line.strip()
        for line in current_text.splitlines()
        if "[error]" in line.lower()
        or "错误:" in line
        or "error while loading shared libraries" in line
        or "检测到致命错误" in line
        or "[fatal]" in line.lower()
    ][-30:]
    result.process_running = False
    return result


def verify_planes(
    config: FleetConfig,
    planes: Sequence[Plane],
    run_id: str,
    remote: RemoteExecutor,
    run_dir: Path,
) -> list[PlaneResult]:
    deadline = time.monotonic() + config.verify_timeout_s
    last_events: dict[str, set[str]] = {plane.plane_id: set() for plane in planes}
    latest_logs: dict[str, str] = {}
    results: list[PlaneResult] = []
    while time.monotonic() < deadline:
        results = []
        all_passed = True
        for plane in planes:
            text = read_plane_logs(plane, remote)
            latest_logs[plane.plane_id] = text
            result = analyze_plane(plane, planes, text)
            running = remote.run(plane, "pgrep -x plane >/dev/null 2>&1", timeout=10, check=False)
            result.process_running = running.returncode == 0
            results.append(result)

            events = set()
            if result.process_running:
                events.add("process")
            if "Zenoh 已上线" in text:
                events.add("zenoh")
            if result.flight_connected:
                events.add("flight")
            for peer_id, online in result.peer_online.items():
                if online:
                    events.add("peer:" + peer_id)
            for event in sorted(events - last_events[plane.plane_id]):
                print(f"[verify] {plane.plane_id}: {event}")
            last_events[plane.plane_id] = events

            peers_ok = all(result.peer_online.values())
            all_passed = all_passed and result.process_running and result.flight_connected and peers_ok
        if all_passed:
            break
        time.sleep(1)

    for plane in planes:
        (run_dir / f"{plane.plane_id}.log").write_text(latest_logs.get(plane.plane_id, ""), encoding="utf-8")
    return results


def write_report(
    run_dir: Path,
    run_id: str,
    planes: Sequence[Plane],
    endpoints: EndpointSet | None,
    results: Sequence[PlaneResult],
) -> None:
    report = {
        "run_id": run_id,
        "time": now_shanghai().isoformat(),
        "planes": [
            dataclasses.asdict(result)
            | {
                "passed": result.process_running
                and result.flight_connected
                and all(result.peer_online.values())
            }
            for result in results
        ],
        "endpoints": dataclasses.asdict(endpoints) if endpoints else None,
    }
    (run_dir / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    lines = [f"# 多机测试报告 {run_id}", "", "| 接收方 | 进程 | 飞控 | " + " | ".join(p.plane_id for p in planes) + " |", "| --- | --- | --- | " + " | ".join("---" for _ in planes) + " |"]
    by_id = {result.plane_id: result for result in results}
    for plane in planes:
        result = by_id[plane.plane_id]
        cells = []
        for source in planes:
            if source == plane:
                cells.append("—")
            else:
                online = result.peer_online.get(source.plane_id, False)
                cells.append("在线" if online else "离线或未发现")
        lines.append(
            f"| {plane.plane_id} | {'通过' if result.process_running else '失败'} | "
            f"{'通过' if result.flight_connected else '失败'} | " + " | ".join(cells) + " |"
        )
    lines.extend(["", "## 日志错误摘要", ""])
    for result in results:
        lines.append(f"### {result.plane_id}")
        lines.append("")
        if result.errors:
            lines.extend(f"- `{line}`" for line in result.errors)
        else:
            lines.append("- 未发现 error/fatal 日志。")
        lines.append("")
    (run_dir / "report.md").write_text("\n".join(lines), encoding="utf-8")


def command_preflight(args: argparse.Namespace, config: FleetConfig, remote: RemoteExecutor) -> int:
    planes = selected_planes(config, args.count)
    preflight(config, planes, remote)
    problems = bundle_problems(ROOT / "build" / "plane")
    if problems:
        print("[preflight] 本地运行包不完整，执行 test 时将自动编译: " + ", ".join(problems))
    else:
        print("[preflight] 本地运行包完整")
    print(f"[preflight] 通过：可使用 {len(planes)} 架飞机")
    return 0


def render_presence_config(base: str, template_text: str, plane: Plane) -> str:
    """保留板端现有连接参数，只分发当前版本支持的配置项。"""
    current = configparser.ConfigParser(interpolation=None)
    current.read_string(base)
    template = configparser.ConfigParser(interpolation=None)
    template.read_string(template_text)
    result = template_text
    for section in template.sections():
        for key in template[section]:
            if current.has_option(section, key):
                result = set_ini_value(result, section, key, current[section][key])
    result = set_ini_value(result, "system", "log_dir", "logs")
    return set_ini_value(result, "communication", "plane_id", plane.plane_id)


def presence_event_counts(text: str, peer_id: str) -> tuple[int, int]:
    events = [status for status, peer in PEER_LINK_RE.findall(text) if peer == peer_id]
    return events.count("上线"), events.count("下线")


def command_test_presence(args: argparse.Namespace, config: FleetConfig, remote: RemoteExecutor) -> int:
    planes = selected_planes(config, args.count)
    if len(planes) != 2:
        raise FleetError("test-presence 当前要求 --count 2")
    run_id = new_run_id()
    run_dir = ROOT / "logs" / "fleet" / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    report = {"run_id": run_id, "status": "running", "checks": [], "backups": {}, "flight_or_task_commands_sent": False}
    print(f"[run] 上下线验证 run_id={run_id}, 证据目录={run_dir}", flush=True)

    def save_report() -> None:
        (run_dir / "presence-report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )

    def passed(name: str) -> None:
        report["checks"].append({"name": name, "status": "passed"})
        save_report()
        print(f"[presence] 通过: {name}", flush=True)

    def snapshot(plane: Plane, stage: str) -> str:
        path = f"{plane.runtime_dir}/logs/plane.stdout.log"
        result = remote.run(plane, f"cat {shlex.quote(path)}", timeout=15, check=False)
        text = sanitize_output(result.stdout)
        (run_dir / f"{stage}-{plane.plane_id}.log").write_text(text, encoding="utf-8")
        return text

    def running(plane: Plane) -> bool:
        return remote.run(plane, "pgrep -x plane >/dev/null", timeout=10, check=False).returncode == 0

    def wait_for_counts(observer: Plane, peer: Plane, expected: tuple[int, int], stage: str) -> None:
        deadline = time.monotonic() + config.verify_timeout_s
        while time.monotonic() < deadline:
            text = snapshot(observer, stage)
            if not running(observer):
                raise FleetError(f"{stage}: {observer.plane_id} 意外退出")
            counts = presence_event_counts(text, peer.plane_id)
            if counts == expected:
                passed(f"{stage}: {observer.plane_id} 观察 {peer.plane_id}, 上线/下线次数={counts}")
                return
            if any(actual > wanted for actual, wanted in zip(counts, expected)):
                raise FleetError(f"{stage}: 出现重复上下线日志: {counts}, 期望 {expected}")
            time.sleep(0.5)
        raise FleetError(f"{stage}: {observer.plane_id} 未观察到 {peer.plane_id} 的预期变化 {expected}")

    def quiet_window(stage: str, seconds: int) -> None:
        before = {plane.plane_id: snapshot(plane, stage + "-before") for plane in planes}
        time.sleep(seconds)
        for observer in planes:
            peer = next(plane for plane in planes if plane != observer)
            text = snapshot(observer, stage)
            if not running(observer) or "系统就绪" not in text:
                raise FleetError(f"{stage}: {observer.plane_id} 未保持运行")
            if "他机状态更新:" in text:
                raise FleetError(f"{stage}: {observer.plane_id} 仍逐条打印位置更新")
            if presence_event_counts(text, peer.plane_id) != presence_event_counts(before[observer.plane_id], peer.plane_id):
                raise FleetError(f"{stage}: {observer.plane_id} 稳定运行期间出现额外上下线日志")
            if PEER_LINK_RE.search(text) is None:
                raise FleetError(f"{stage}: {observer.plane_id} 缺少友机发现日志")
            if re.search(r"他机(?:上线|下线):\s*" + re.escape(observer.plane_id) + r"(?:\s|$)", text):
                raise FleetError(f"{stage}: {observer.plane_id} 把自身打印为友机")
            result = analyze_plane(observer, planes, text)
            if result.errors or not all(result.peer_online.values()):
                raise FleetError(f"{stage}: {observer.plane_id} 存在错误或未感知友机在线: {result.errors}")
        passed(f"{stage}: 双机稳定运行 {seconds}s，仅打印上下线，无位置更新日志")

    def restart_cycle(victim: Plane, observer: Plane, *, abrupt: bool) -> None:
        stage = ("crash" if abrupt else "stop") + "-" + victim.plane_id
        online, offline = presence_event_counts(snapshot(observer, stage + "-baseline"), victim.plane_id)
        try:
            if abrupt:
                # 只终止本次已部署运行目录中的进程，不操作其他板卡或仿真环境。
                runtime = shlex.quote(victim.runtime_dir)
                remote.run(victim, f'for pid in $(pgrep -x plane); do '
                           f'[ "$(readlink /proc/$pid/exe)" = {runtime}/plane ] || exit 2; '
                           'kill -KILL "$pid"; done', timeout=10)
            else:
                stop_planes((victim,), remote)
            stopped_deadline = time.monotonic() + 5
            while running(victim):
                if time.monotonic() >= stopped_deadline:
                    raise FleetError(f"{stage}: 进程未停止")
                time.sleep(0.2)
            wait_for_counts(observer, victim, (online, offline + 1), stage + "-offline")
            time.sleep(2)
            if presence_event_counts(snapshot(observer, stage + "-offline-held"), victim.plane_id) != (online, offline + 1):
                raise FleetError(f"{stage}: 离线期间重复打印日志")
        finally:
            if not running(victim):
                start_plane(victim, remote)
        wait_for_counts(observer, victim, (online + 1, offline + 1), stage + "-recovered")
        # 重启节点应通过 history 发现先上线的观察节点。
        wait_for_counts(victim, observer, (1, 0), stage + "-history")

    try:
        preflight(config, planes, remote, include_simulator=False)
        bundle = ensure_bundle()
        template = (bundle / "config/plane.conf").read_text(encoding="utf-8")
        rendered = {}
        for plane in planes:
            existing = remote.run(plane, f"cat {shlex.quote(plane.runtime_dir + '/config/plane.conf')}").stdout
            (run_dir / f"{plane.plane_id}.previous.conf").write_text(existing, encoding="utf-8")
            local_config = run_dir / f"{plane.plane_id}.plane.conf"
            local_config.write_text(render_presence_config(existing, template, plane), encoding="utf-8")
            rendered[plane.plane_id] = local_config
        for plane in planes:
            backup = plane.runtime_dir + ".backup-presence-" + run_id
            remote.run(plane, f"mkdir -p {shlex.quote(backup)} && rsync -a --exclude=/logs/ "
                       f"{shlex.quote(plane.runtime_dir + '/')} {shlex.quote(backup + '/')}", timeout=60)
            report["backups"][plane.plane_id] = backup
        save_report()
        stop_planes(planes, remote)
        expected_sha = hashlib.sha256((bundle / "plane").read_bytes()).hexdigest()
        for plane in planes:
            remote.rsync_to(plane, str(bundle) + "/", plane.runtime_dir + "/",
                            extra=("--delete", "--exclude=/logs/", "--exclude=/config/plane.conf"))
            remote.rsync_to(plane, rendered[plane.plane_id], plane.runtime_dir + "/config/plane.conf")
            check_remote_dependencies(plane, remote)
            deps = remote.run(plane, f"cd {shlex.quote(plane.runtime_dir)} && sha256sum plane && ./plane --deps-check")
            (run_dir / f"{plane.plane_id}.deps.log").write_text(deps.stdout, encoding="utf-8")
            if expected_sha not in deps.stdout:
                raise FleetError(f"{plane.plane_id} 程序校验值不匹配")
        passed("双机部署校验及依赖自检")
        for plane in planes:
            start_plane(plane, remote)
        wait_for_counts(planes[0], planes[1], (1, 0), "initial-uav1")
        wait_for_counts(planes[1], planes[0], (1, 0), "initial-uav2")
        quiet_window("initial-quiet", 10)
        restart_cycle(planes[1], planes[0], abrupt=False)
        restart_cycle(planes[0], planes[1], abrupt=False)
        restart_cycle(planes[1], planes[0], abrupt=True)
        quiet_window("final-quiet", 10)
        report["status"] = "passed"
        report["binary_sha256"] = expected_sha
        save_report()
        print(f"[result] 上下线及安静日志验证通过；双机保持运行。报告: {run_dir / 'presence-report.json'}", flush=True)
        return 0
    except (FleetError, OSError, configparser.Error) as exc:
        report["status"] = "failed"
        report["error"] = str(exc)
        save_report()
        for plane in planes:
            snapshot(plane, "failure")
        raise FleetError(f"上下线验证失败: {exc}；证据目录: {run_dir}") from exc


def command_test(args: argparse.Namespace, config: FleetConfig, remote: RemoteExecutor) -> int:
    planes = selected_planes(config, args.count)
    run_id = new_run_id()
    run_dir = ROOT / "logs" / "fleet" / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    print(f"[run] run_id={run_id}, 证据目录={run_dir}")
    preflight(config, planes, remote)
    bundle = ensure_bundle()
    stop_planes(planes, remote)
    for plane in planes:
        install_system_dependencies(config, plane, remote)
    stop_simulator(config, remote)
    endpoints = start_simulator(config, len(planes), remote, run_dir)
    for index, plane in enumerate(planes):
        sync_plane(plane, bundle, endpoints.flight_urls[index], run_id, remote, run_dir)
        check_remote_dependencies(plane, remote)
    for plane in planes:
        start_plane(plane, remote)
    results = verify_planes(config, planes, run_id, remote, run_dir)
    write_report(run_dir, run_id, planes, endpoints, results)
    passed = all(
        result.process_running
        and result.flight_connected
        and all(result.peer_online.values())
        for result in results
    )
    print(f"[report] {run_dir / 'report.md'}")
    if passed:
        print("[result] 多机启动与友机上线感知验证通过；环境保持运行。")
        return 0
    print("[result] 验证失败；环境保持现场，使用 down 命令停止。", file=sys.stderr)
    return 1


def command_down(args: argparse.Namespace, config: FleetConfig, remote: RemoteExecutor) -> int:
    planes = selected_planes(config, args.count)
    stop_planes(planes, remote)
    stop_simulator(config, remote)
    print("[down] 机载程序和 SITL 已停止")
    return 0


def command_verify(args: argparse.Namespace, config: FleetConfig, remote: RemoteExecutor) -> int:
    planes = selected_planes(config, args.count)
    run_id = args.run_id
    run_dir = ROOT / "logs" / "fleet" / run_id
    if not run_dir.is_dir():
        raise FleetError(f"本地不存在待复核的运行记录: {run_dir}")

    endpoints = None
    endpoint_file = run_dir / "endpoints.json"
    if endpoint_file.is_file():
        raw = json.loads(endpoint_file.read_text(encoding="utf-8"))
        endpoints = EndpointSet(
            flight_urls=tuple(raw.get("flight_urls", ())),
            all_uris=tuple(raw.get("all_uris", ())),
        )

    print(f"[verify] 复核运行现场 run_id={run_id}")
    results = verify_planes(config, planes, run_id, remote, run_dir)
    write_report(run_dir, run_id, planes, endpoints, results)
    passed = all(
        result.process_running
        and result.flight_connected
        and all(result.peer_online.values())
        for result in results
    )
    print(f"[report] {run_dir / 'report.md'}")
    if passed:
        print("[result] 当前运行现场的友机上线感知验证通过；环境保持运行。")
        return 0
    print("[result] 当前运行现场验证失败；环境保持运行。", file=sys.stderr)
    return 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="环境清单（默认 config/fleet.ini）")
    subparsers = parser.add_subparsers(dest="command", required=True)
    for name, help_text in (
        ("preflight", "只执行环境和连接预检"),
        ("test", "启动、同步并验证多机状态互通"),
        ("test-presence", "部署双机并验证上线、退出、崩溃和恢复，不重启 SITL"),
        ("verify", "复核一个已启动运行现场的多机状态互通"),
        ("down", "停止机载程序和 SITL"),
    ):
        command = subparsers.add_parser(name, help=help_text)
        command.add_argument("--count", type=int, required=True, help="请求的飞机数量")
        if name == "verify":
            command.add_argument("--run-id", required=True, help="待复核的 fleet 运行编号")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        config = load_config(args.config.resolve())
        remote = RemoteExecutor(ROOT / ".fleet")
        if args.command == "preflight":
            return command_preflight(args, config, remote)
        if args.command == "test-presence":
            return command_test_presence(args, config, remote)
        if args.command == "test":
            return command_test(args, config, remote)
        if args.command == "verify":
            return command_verify(args, config, remote)
        if args.command == "down":
            return command_down(args, config, remote)
        raise FleetError(f"未知命令: {args.command}")
    except FleetError as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\n已中断；远端环境可能仍在运行，可执行 down 停止。", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
