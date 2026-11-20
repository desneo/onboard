#!/usr/bin/env python3
"""PX4 SITL 与多块机载板的可重复测试编排工具。"""

from __future__ import annotations

import argparse
import configparser
import dataclasses
import datetime as dt
import hashlib
import ipaddress
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
from typing import Callable, Iterable, Sequence
from zoneinfo import ZoneInfo


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "tests" / "fleet.ini"
PLANE_CONFIG = "config/plane.ini"
PROFILE_DIR = "config/profiles"
SITL_REMOTE_PORT_RANGES = ((14030, 14039), (14280, 14289), (14580, 14589))
PEER_LINK_RE = re.compile(r"他机(上线|下线):\s*([^\s]+)")
GCS_TARGET_HISTORY_RE = re.compile(
    r"GCS 目标更新(?:汇总)?: trackId=([^\s]+) timestamp_ms=\d+ history_size=(\d+)(?=\s|$)"
)
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


@dataclasses.dataclass(frozen=True)
class PlaneConfiguration:
    main: str
    profile_path: str
    profile_text: str
    settings: tuple[tuple[str, str, str], ...]

    def get(self, section: str, key: str, fallback: str = "") -> str:
        for setting_section, setting_key, value in self.settings:
            if (section, key) == (setting_section, setting_key):
                return value
        parser = configparser.ConfigParser(interpolation=None)
        parser.read_string(self.main)
        return parser.get(section, key, fallback=fallback)


@dataclasses.dataclass(frozen=True)
class PlaneDeployment:
    directory: Path
    configuration: PlaneConfiguration


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
            f"环境不满足：请求 {count} 架飞机，但所选测试清单仅声明 {len(config.planes)} 块机载板。\n"
            f"已声明: {declared}\n本次不会启动 SITL 或机载程序。"
        )
    return config.planes[:count]


def require_local_commands(commands: Iterable[str]) -> None:
    missing = [command for command in commands if shutil.which(command) is None]
    if missing:
        raise FleetError("代码机缺少命令: " + ", ".join(missing))


def bundle_problems(bundle: Path) -> list[str]:
    required_files = [
        "plane", "runplane.sh", "stopplane.sh", "install-plane-service.sh", PLANE_CONFIG, "config/zenoh.json5",
    ]
    required_files.extend(f"{PROFILE_DIR}/{path.name}" for path in sorted((ROOT / PROFILE_DIR).glob("*.ini")))
    required_dirs = ["lib"]
    problems = [name for name in required_files if not (bundle / name).is_file()]
    problems.extend(name + "/" for name in required_dirs if not (bundle / name).is_dir())
    if (bundle / "lib").is_dir() and not any(path.is_file() for path in (bundle / "lib").rglob("*")):
        problems.append("lib/（为空）")
    return problems


def check_bundle_dependencies(bundle: Path) -> None:
    try:
        result = subprocess.run(
            [str((bundle / "plane").resolve()), "--deps-check"], cwd=bundle,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=30, check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise FleetError(f"运行包依赖自检无法执行: {exc}") from exc
    if result.returncode != 0:
        raise FleetError(f"运行包依赖自检失败: {sanitize_output(result.stdout).strip()}")


def ensure_bundle() -> Path:
    bundle = ROOT / "build" / "plane"
    problems = bundle_problems(bundle)
    if not problems:
        check_bundle_dependencies(bundle)
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
    check_bundle_dependencies(bundle)
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


def sitl_remote_port(uri: str) -> int | None:
    if not uri.startswith(("udp://", "udpout://")):
        return None
    port = endpoint_port(uri)
    if port is not None and any(first <= port <= last for first, last in SITL_REMOTE_PORT_RANGES):
        return port
    return None


def validate_flight_ports(flight_ports: Sequence[int]) -> tuple[int, ...]:
    ports = tuple(flight_ports)
    if not ports or len(ports) != len(set(ports)) or any(port <= 0 or port > 65535 for port in ports):
        raise FleetError("运行包飞控端口为空、重复或超出有效范围")
    return ports


def validate_flight_listeners(flight_ports: Sequence[int], socket_text: str) -> tuple[int, ...]:
    expected = validate_flight_ports(flight_ports)
    listening = {int(port) for port in re.findall(r":(\d+)\s", socket_text)}
    missing = [port for port in expected if port not in listening]
    if missing:
        raise FleetError(f"SITL 未监听运行包预置的飞控端口: {', '.join(map(str, missing))}")
    return expected


def parse_endpoints(text: str, simulator_host: str, flight_ports: Sequence[int]) -> EndpointSet:
    clean = sanitize_output(text)
    all_uris = tuple(dict.fromkeys(uri.rstrip(".，。") for uri in URI_RE.findall(clean)))
    ports = validate_flight_ports(flight_ports)
    flight_urls = tuple(f"udpout://{simulator_host}:{port}" for port in ports)
    return EndpointSet(flight_urls=flight_urls, all_uris=all_uris)


def start_simulator(
    config: FleetConfig,
    flight_ports: Sequence[int],
    remote: RemoteExecutor,
    run_dir: Path,
    *,
    start_count: int | None = None,
) -> EndpointSet:
    sim = config.simulator
    ports = validate_flight_ports(flight_ports)
    count = len(ports) if start_count is None else start_count
    if count < len(ports):
        raise FleetError("SITL 启动实例数不得少于运行包中的飞控端口数")
    command = f"cd {shlex.quote(sim.scenario_dir)} && {sim.start_command} -n {count}"
    print(f"[sitl] 启动 {count} 架仿真飞机")
    rc, output = remote.stream(sim, command, label="sitl", timeout=config.startup_timeout_s)
    (run_dir / "sitl-start.log").write_text(output, encoding="utf-8")
    if rc != 0:
        raise FleetError(f"SITL start 失败: rc={rc}，输出见 {run_dir / 'sitl-start.log'}")
    listeners = remote.run(sim, "ss -H -lun", timeout=15).stdout
    (run_dir / "sitl-udp-listeners.log").write_text(listeners, encoding="utf-8")
    validate_flight_listeners(ports, listeners)
    endpoints = parse_endpoints(output, sim.host, ports)
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
    if not lines[end - 1].endswith("\n"):
        lines[end - 1] += "\n"
    lines.insert(end, replacement)
    return "".join(lines)


def validate_deployment_identity(plane: Plane, configuration: PlaneConfiguration) -> None:
    plane_id = configuration.get("communication", "plane_id")
    if plane_id != plane.plane_id:
        raise FleetError(
            f"{plane.plane_id} 的预置飞机 ID 为 {plane_id}，与测试清单不一致；"
            "请先修正清单或预置，本次不停止或覆盖板端现场"
        )


def configure_plane(base: PlaneConfiguration, plane: Plane, simulator_host: str | None = None) -> PlaneConfiguration:
    validate_deployment_identity(plane, base)
    profile_text = base.profile_text
    if simulator_host and base.get("flight_control", "link_type") == "udp":
        url = base.get("flight_control", "udp_url")
        port = sitl_remote_port(url)
        if port is not None:
            profile_text = set_ini_value(profile_text, plane.plane_id, "fc_url", f"udpout://{simulator_host}:{port}")
    addresses = {base.get("system", "board_ip")}
    settings = tuple(deployment_settings(base.profile_path, profile_text, addresses))
    return dataclasses.replace(base, profile_text=profile_text, settings=settings)


def configured_sitl_ports(configurations: Sequence[PlaneConfiguration]) -> list[int]:
    ports = [
        port
        for configuration in configurations
        if configuration.get("flight_control", "link_type") == "udp"
        and (port := sitl_remote_port(configuration.get("flight_control", "udp_url"))) is not None
    ]
    if len(ports) != len(set(ports)):
        raise FleetError("所选飞机预置的 SITL 飞控 UDP 端口重复")
    return ports


def check_sitl_profile_listeners(
    config: FleetConfig, configurations: Sequence[PlaneConfiguration], remote: RemoteExecutor
) -> None:
    ports = configured_sitl_ports(configurations)
    if not ports:
        return
    sockets = remote.run(config.simulator, "ss -uln").stdout
    listening = {int(port) for port in re.findall(r":(\d+)\s", sockets)}
    missing = sorted(set(ports) - listening)
    if missing:
        raise FleetError(f"SITL 主机 {config.simulator.host} 未监听预置飞控 UDP 端口: {missing}")


def remote_ipv4_addresses(plane: Plane, remote: RemoteExecutor) -> set[str]:
    result = remote.run(plane, "ip -j -4 address show up")
    try:
        interfaces = json.loads(result.stdout)
        return {
            address["local"]
            for interface in interfaces
            if "UP" in interface["flags"] and "LOOPBACK" not in interface["flags"]
            for address in interface.get("addr_info", ())
            if address["family"] == "inet"
        }
    except (json.JSONDecodeError, KeyError, TypeError) as exc:
        raise FleetError(f"{plane.plane_id} 无法解析本机 IPv4 地址") from exc


def deployment_settings(path: str, text: str, local_addresses: set[str]) -> list[tuple[str, str, str]]:
    parser = configparser.ConfigParser(interpolation=None, delimiters=("=",), default_section="")
    parser.optionxform = str
    parser.SECTCRE = re.compile(r"\[(?P<header>[^\[\]]+)\]$")
    parser.read_string("\n".join(line.strip() for line in text.splitlines()), source=path)
    selected: list[tuple[str, str, str]] = []
    selected_ip: ipaddress.IPv4Address | None = None
    section_names = set()
    board_ips: set[ipaddress.IPv4Address] = set()
    groundstation_ip = ""
    for section in parser.sections():
        plane_id = section.strip()
        if not plane_id or re.search(r"[/\*?#$\s]", plane_id) or plane_id in section_names:
            raise FleetError(f"部署预置 {path} 的飞机 ID 无效或重复: [{section}]")
        section_names.add(plane_id)
        entry = parser[section]
        if plane_id == "groundstation":
            groundstation_ip = entry.get("ip", "")
            continue
        try:
            board_ip = ipaddress.IPv4Address(entry.get("board_ip", ""))
        except ipaddress.AddressValueError as exc:
            raise FleetError(f"部署预置 {path} 的 [{plane_id}] 缺少有效 board_ip") from exc
        if board_ip in board_ips:
            raise FleetError(f"部署预置 {path} 的 board_ip 重复: {board_ip}")
        board_ips.add(board_ip)
        fc_url = entry.get("fc_url", "")
        scheme, _, address = fc_url.partition("://")
        host_or_device, separator, port_or_baud = address.rpartition(":")
        if not separator or re.search(r"\s", fc_url) or not re.fullmatch(r"[0-9]+", port_or_baud):
            raise FleetError(f"部署预置 {path} 的 [{plane_id}] 缺少有效 fc_url")
        digits = port_or_baud.lstrip("0")
        number = int(digits) if 0 < len(digits) <= 10 else 0
        if scheme == "serial" and host_or_device.startswith("/") and 0 < number <= 2**31 - 1:
            flight_settings = [
                ("flight_control", "link_type", "serial"),
                ("flight_control", "serial_device", host_or_device),
                ("flight_control", "serial_baud", str(number)),
            ]
        elif (
            scheme in {"udp", "udpin", "udpout"}
            and (scheme != "udpout" or host_or_device)
            and not any(character in host_or_device for character in "/\\?#@")
            and 0 < number <= 65535
        ):
            flight_settings = [
                ("flight_control", "link_type", "udp"),
                ("flight_control", "udp_url", fc_url),
            ]
        else:
            raise FleetError(f"部署预置 {path} 的 [{plane_id}] 缺少有效 fc_url")
        if str(board_ip) not in local_addresses:
            continue
        if selected_ip is not None:
            raise FleetError(f"部署预置 {path} 中有多个飞机 ID 匹配本机 IPv4")
        selected_ip = board_ip
        selected = [
            ("communication", "plane_id", plane_id),
            ("system", "board_ip", str(board_ip)),
            *flight_settings,
            *(("camera", key, entry.get(key, "")) for key in ("pod_model", "pod_video_url", "pod_control_url")),
        ]
    connect_endpoints = []
    if groundstation_ip:
        try:
            station_ip = ipaddress.IPv4Address(groundstation_ip)
        except ipaddress.AddressValueError as exc:
            raise FleetError(f"部署预置 {path} 的 groundstation.ip 无效: {groundstation_ip}") from exc
        if station_ip.packed[0] in (0, 127) or station_ip.packed[0] >= 224:
            raise FleetError(f"部署预置 {path} 的 groundstation.ip 须为单播 IPv4: {groundstation_ip}")
        if station_ip in board_ips:
            raise FleetError(f"部署预置 {path} 的 groundstation.ip 与 board_ip 重复: {groundstation_ip}")
        connect_endpoints.append(f"tcp/{station_ip}:7447")

    if selected_ip is None:
        raise FleetError(f"部署预置 {path} 未匹配本机 IPv4")
    connect_endpoints.extend(f"tcp/{peer_ip}:7447" for peer_ip in sorted(board_ips - {selected_ip}))
    selected.extend([
        ("communication", "zenoh_listen", f"tcp/{selected_ip}:7447"),
        ("communication", "zenoh_connect", ",".join(connect_endpoints)),
    ])
    return selected


def bundle_config_file(bundle: Path, reference: str, key: str) -> Path:
    normalized = Path(os.path.normpath(reference))
    if Path(reference).is_absolute():
        raise FleetError(f"运行包 {key} 必须使用包内相对路径")
    target = (bundle / reference).resolve()
    if normalized.parts[:1] == ("..",) or not target.is_relative_to(bundle.resolve()):
        raise FleetError(f"运行包 {key} 解析后超出运行包目录")
    if normalized.parts[:1] == ("logs",) or target.relative_to(bundle.resolve()).parts[:1] == ("logs",):
        raise FleetError(f"运行包 {key} 不得引用保留的 logs 目录")
    if not target.is_file():
        raise FleetError(f"运行包 {key} 所引用的包内文件不存在: {reference}")
    return target


def read_bundle_config(plane: Plane, remote: RemoteExecutor, bundle: Path) -> PlaneConfiguration:
    """仅从完整运行包读取配置，用板端实际网卡匹配预置。"""
    parser = configparser.ConfigParser(interpolation=None)
    try:
        main = (bundle / PLANE_CONFIG).read_text(encoding="utf-8")
        parser.read_string(main, source=str(bundle / PLANE_CONFIG))
        profile = parser.get("deployment", "profile", fallback="").strip()
        if not profile:
            raise FleetError(f"运行包 {PLANE_CONFIG} 的 deployment.profile 缺失或为空")
        profile_file = bundle_config_file(bundle, profile, "deployment.profile")
        native_config = parser.get("communication", "zenoh_config_file", fallback="").strip()
        if native_config:
            bundle_config_file(bundle, native_config, "communication.zenoh_config_file")
        profile_text = profile_file.read_text(encoding="utf-8")
        addresses = remote_ipv4_addresses(plane, remote)
        settings = tuple(deployment_settings(str(profile_file), profile_text, addresses))
    except (configparser.Error, OSError, UnicodeError) as exc:
        raise FleetError(f"{plane.plane_id} 读取运行包配置失败: {exc}") from exc
    configuration = PlaneConfiguration(main, profile, profile_text, settings)
    validate_deployment_identity(plane, configuration)
    return configuration


def prepare_plane_deployment(
    plane: Plane, bundle: Path, remote: RemoteExecutor, run_dir: Path, simulator_host: str
) -> PlaneDeployment:
    problems = bundle_problems(bundle)
    if problems:
        raise FleetError("运行包不完整: " + ", ".join(problems))
    configuration = configure_plane(read_bundle_config(plane, remote, bundle), plane, simulator_host)
    directory = run_dir / (plane.plane_id + ".deployment")
    shutil.copytree(bundle, directory)
    profile_file = bundle_config_file(directory, configuration.profile_path, "deployment.profile")
    native_config = configuration.get("communication", "zenoh_config_file").strip()
    if native_config:
        bundle_config_file(directory, native_config, "communication.zenoh_config_file")
    profile_file.write_text(configuration.profile_text, encoding="utf-8")
    return PlaneDeployment(directory, configuration)


def backup_plane_runtime(plane: Plane, remote: RemoteExecutor, run_id: str) -> str:
    backup = plane.runtime_dir + ".backup-" + run_id
    runtime = shlex.quote(plane.runtime_dir)
    remote.run(
        plane,
        f"mkdir -p {shlex.quote(backup)} && "
        f"if [ -d {runtime} ]; then rsync -a --exclude=/logs/ "
        f"{shlex.quote(plane.runtime_dir + '/')} {shlex.quote(backup + '/')}; fi",
        timeout=60,
    )
    return backup


def deploy_plane_bundle(plane: Plane, deployment: PlaneDeployment, remote: RemoteExecutor) -> None:
    """全量替换完整运行目录，仅保留既有日志及 core 文件。"""
    validate_deployment_identity(plane, deployment.configuration)
    remote.run(plane, f"mkdir -p {shlex.quote(plane.runtime_dir)}")
    print(f"[sync] {plane.plane_id} -> {plane.host}:{plane.runtime_dir}")
    remote.rsync_to(
        plane, str(deployment.directory) + "/", plane.runtime_dir.rstrip("/") + "/",
        extra=("--delete", "--exclude=/logs/"),
    )


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
    latest_start = max(
        (
            timestamp
            for timestamp, line in records
            if "plane 程序启动:" in line or "plane 启动:" in line
        ),
        default="",
    )
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
        if "[错误]" in line
        or "[严重]" in line
        or "[error]" in line.lower()
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
            if not result.flight_connected:
                raise FleetError(f"{stage}: {observer.plane_id} 尚未建立飞控连接")
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
        deployments = {
            plane.plane_id: prepare_plane_deployment(plane, bundle, remote, run_dir, config.simulator.host)
            for plane in planes
        }
        check_sitl_profile_listeners(config, tuple(item.configuration for item in deployments.values()), remote)
        for plane in planes:
            report["backups"][plane.plane_id] = backup_plane_runtime(plane, remote, run_id)
        save_report()
        stop_planes(planes, remote)
        expected_sha = hashlib.sha256((bundle / "plane").read_bytes()).hexdigest()
        for plane in planes:
            deploy_plane_bundle(plane, deployments[plane.plane_id], remote)
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


def agent_info_log_payloads(text: str, marker: str) -> list[dict[str, object]]:
    return [json.loads(item) for item in re.findall(re.escape(marker) + r": (\{[^\n]*\})", text)]


def select_gcs_loopback_endpoint(plane: Plane, remote: RemoteExecutor) -> str:
    script = (
        "import socket; "
        "s=socket.socket(socket.AF_INET,socket.SOCK_STREAM); "
        "s.bind(('127.0.0.1',0)); print(s.getsockname()[1]); s.close()"
    )
    result = remote.run(plane, "python3 -c " + shlex.quote(script))
    try:
        port = int(result.stdout.strip())
    except ValueError as exc:
        raise FleetError("无法为 AgentInfo 测试选择本地 TCP 监听端口") from exc
    if not 0 < port <= 65535:
        raise FleetError("AgentInfo 测试的本地 TCP 监听端口无效")
    return f"tcp/127.0.0.1:{port}"


def agent_info_transport_options(
    stage: str, plane_endpoint: str, station_endpoint: str, *, second_station: bool = False
) -> list[str]:
    if stage == "startup":
        # startup 先于机载启动，保留其自动发现流程，避免拨号尚未启动的机载。
        return ["--listen", station_endpoint]
    if second_station:
        return ["--connect", f"{station_endpoint},{plane_endpoint}"]
    return ["--listen", station_endpoint, "--connect", plane_endpoint]


def command_test_agent_info(args: argparse.Namespace, config: FleetConfig, remote: RemoteExecutor) -> int:
    planes = selected_planes(config, args.count)
    if len(planes) != 1:
        raise FleetError("test-agent-info 当前要求 --count 1")
    plane = planes[0]
    preflight(config, planes, remote, include_simulator=False)
    bundle = ensure_bundle()
    gcs_binary = ROOT / "build" / "gcs_sim"
    if not gcs_binary.is_file():
        raise FleetError("缺少 build/gcs_sim，请先完成构建")
    run_id = new_run_id()
    run_dir = ROOT / "logs" / "fleet" / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    runtime = shlex.quote(plane.runtime_dir)
    test_dir = f"/tmp/plane-agent-info-{run_id}"
    report_path = run_dir / "agent-info-report.json"
    station_a = f"fleet-gcs-a-{run_id}"
    station_b = f"fleet-gcs-b-{run_id}"
    station_endpoint = ""
    report = {"run_id": run_id, "status": "running", "checks": [], "plane": plane.plane_id,
              "host": plane.host, "flight_or_task_commands_sent": False}

    def save() -> None:
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    def passed(name: str) -> None:
        report["checks"].append({"name": name, "status": "passed"})
        save()
        print(f"[agent-info] 通过: {name}", flush=True)

    def gcs_text(stage: str) -> str:
        text = sanitize_output(remote.run(plane, f"cat {shlex.quote(test_dir + '/' + stage + '.log')}",
                                          check=False, timeout=10).stdout)
        (run_dir / f"gcs-{stage}.log").write_text(text, encoding="utf-8")
        return text

    def wait_gcs(stage: str) -> tuple[int, str]:
        deadline = time.monotonic() + 20
        status_path = shlex.quote(test_dir + "/" + stage + ".status")
        while time.monotonic() < deadline:
            result = remote.run(plane, f"cat {status_path} 2>/dev/null", check=False, timeout=10)
            if result.returncode == 0 and result.stdout.strip():
                return int(result.stdout.strip()), gcs_text(stage)
            time.sleep(0.2)
        raise FleetError(f"gcs_sim {stage} 未按时退出")

    def wait_gcs_log(stage: str, ready: Callable[[str], bool]) -> None:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if ready(gcs_text(stage)):
                return
            time.sleep(0.2)
        raise FleetError(f"gcs_sim {stage} 未就绪，详见 {run_dir / ('gcs-' + stage + '.log')}")

    def launch_gcs(stage: str, *, query: str = "", seconds: int = 2,
                   presence: bool = True, sender_id: str = station_a, agent_id: str | None = None) -> None:
        options = ["--plane-id", plane.plane_id, "--sender-id", sender_id, "--board-ip", plane.host,
                   "--listen-seconds", str(seconds), "--query-timeout-ms", "1500",
                   "--no-radar-udp"]
        options += agent_info_transport_options(
            stage, f"tcp/{plane.host}:7447", station_endpoint, second_station=sender_id == station_b
        )
        if query:
            options += ["--query", query]
        if agent_id is not None:
            options += ["--agent-id", agent_id]
        if not presence:
            options += ["--no-presence"]
        log = shlex.quote(test_dir + "/" + stage + ".log")
        status = shlex.quote(test_dir + "/" + stage + ".status")
        command = (f"LD_LIBRARY_PATH={shlex.quote(plane.runtime_dir + '/lib')} "
                   f"{shlex.quote(test_dir + '/gcs_sim')} {shlex.join(options)} >{log} 2>&1; "
                   f"printf '%s\\n' \"$?\" >{status}")
        remote.run(plane, f"setsid -f bash -c {shlex.quote(command)} </dev/null >/dev/null 2>&1", timeout=10)

    def verify_gcs(stage: str, *, push_count: int, reply: dict[str, object] | None = None,
                   peer: dict[str, object] | None = None) -> str:
        code, text = wait_gcs(stage)
        log_path = run_dir / ("gcs-" + stage + ".log")
        if code != 0:
            raise FleetError(f"{stage}: gcs_sim 返回 {code}，详见 {log_path}")
        pushes = agent_info_log_payloads(text, "收到 AgentInfo")
        plane_pushes = [item for item in pushes if item.get("agentId") == plane.plane_id]
        if len(plane_pushes) != push_count or any(item != expected for item in plane_pushes):
            raise FleetError(f"{stage}: 机载 AgentInfo 上报数量或内容错误，期望 {push_count} 次，详见 {log_path}")
        replies = agent_info_log_payloads(text, "AgentInfo 查询成功")
        if replies != ([reply] if reply is not None else []):
            raise FleetError(f"{stage}: AgentInfo 查询结果不匹配，详见 {log_path}")
        if peer is not None and peer not in pushes:
            raise FleetError(f"{stage}: 未收到对端地面站的 AgentInfo，详见 {log_path}")
        passed(f"{stage}: 机载上报 {push_count} 次，AgentInfo 检查通过")
        return text

    try:
        deployment = prepare_plane_deployment(plane, bundle, remote, run_dir, config.simulator.host)
        rendered = deployment.configuration
        check_sitl_profile_listeners(config, (rendered,), remote)
        station_endpoint = select_gcs_loopback_endpoint(plane, remote)
        report["地面站本地端点"] = station_endpoint
        port = endpoint_port(rendered.get("flight_control", "udp_url"))
        is_sim = (
            rendered.get("flight_control", "link_type") == "udp"
            and port is not None
            and any(first <= port <= last for first, last in ((14030, 14039), (14280, 14289), (14580, 14589)))
        )
        expected = {"agentId": plane.plane_id, "agentType": "Vehicle", "boardIp": rendered.get("system", "board_ip"),
                    "videoUrl": rendered.get("camera", "pod_video_url"), "isSim": is_sim}
        stations = {
            agent_id: {"agentId": agent_id, "agentType": "GroundStation", "boardIp": plane.host,
                       "videoUrl": "", "isSim": True}
            for agent_id in (station_a, station_b)
        }
        report["expected"] = expected
        report["stations"] = list(stations.values())
        report["backup"] = backup_plane_runtime(plane, remote, run_id)
        save()
        stop_planes(planes, remote)
        deploy_plane_bundle(plane, deployment, remote)
        remote.run(plane, f"mkdir -p {shlex.quote(test_dir)}")
        remote.rsync_to(plane, gcs_binary, test_dir + "/gcs_sim")
        check_remote_dependencies(plane, remote)
        expected_sha = hashlib.sha256((bundle / "plane").read_bytes()).hexdigest()
        deps = remote.run(plane, f"cd {runtime} && sha256sum plane && ./plane --deps-check")
        (run_dir / "deps.log").write_text(deps.stdout, encoding="utf-8")
        if expected_sha not in deps.stdout:
            raise FleetError("板端二进制校验值不匹配")
        report["binary_sha256"] = expected_sha
        passed("一号机部署校验及依赖自检")
        launch_gcs("startup", seconds=7)
        wait_gcs_log("startup", lambda text: "地面站订阅已就绪" in text)
        start_plane(plane, remote)
        verify_gcs("startup", push_count=1)
        time.sleep(1)
        launch_gcs("station-online", query="AgentInfo", seconds=8)
        wait_gcs_log("station-online", lambda text: expected in agent_info_log_payloads(text, "收到 AgentInfo"))
        launch_gcs("second-station", query="AgentInfo", sender_id=station_b, agent_id=station_a)
        verify_gcs("second-station", push_count=1, reply=stations[station_a], peer=stations[station_a])
        verify_gcs("station-online", push_count=2, reply=expected, peer=stations[station_b])
        time.sleep(1)
        launch_gcs("station-reonline", query="AgentInfo")
        reconnect_log = verify_gcs("station-reonline", push_count=1, reply=expected)
        time.sleep(1)
        launch_gcs("query-only", query="AgentInfo", seconds=1, presence=False)
        verify_gcs("query-only", push_count=0, reply=expected)
        launch_gcs("missing-agent", query="AgentInfo", seconds=0, presence=False,
                   agent_id="agent-info-nonexistent-" + run_id)
        code, text = wait_gcs("missing-agent")
        if code == 0 or "查询失败" not in text or "查询成功" in text:
            raise FleetError("不存在的 Agent 未明确返回查询失败")
        passed("不存在的 Agent 明确返回无响应或超时")
        launch_gcs("unsupported-type", query="VehicleState", seconds=0, presence=False)
        code, text = wait_gcs("unsupported-type")
        if code == 0 or "不支持的查询类型" not in text:
            raise FleetError("不支持的类型未明确拒绝")
        passed("不支持的查询类型明确拒绝")
        text = sanitize_output(remote.run(plane, f"cat {runtime}/logs/plane.stdout.log").stdout)
        (run_dir / "plane.stdout.log").write_text(text, encoding="utf-8")
        remote.run(plane, "pgrep -x plane >/dev/null")
        if "系统就绪" not in text:
            raise FleetError("机载未保持就绪")
        for station in stations:
            if f"收到 AgentInfo: agentId={station} agentType=GroundStation" not in text:
                raise FleetError(f"机载未收到地面站 AgentInfo: agentId={station}")
        passed("机载收到两套地面站身份并保持运行")
        if is_sim:
            telemetry = f"收到飞机状态: planeId={plane.plane_id} fcLinkState=connected"
            if "飞控系统已接入" not in text or telemetry not in reconnect_log:
                raise FleetError("SITL 飞控连接或遥测未验证")
            passed("SITL 飞控连接与遥测验证通过")
        report["status"] = "passed"
        save()
        print(f"[result] AgentInfo 板端验证通过，一号机保持运行。报告: {report_path}")
        return 0
    except (FleetError, OSError, ValueError, configparser.Error) as exc:
        report["status"] = "failed"
        report["error"] = str(exc)
        save()
        raise FleetError(f"AgentInfo 验证失败: {exc}，证据目录: {run_dir}") from exc


def radar_history_ready(text: str, target_id: str = "") -> bool:
    """依据当前运行日志中的缓存帧数判断就绪，不将降频日志条数当作雷达帧数。"""
    return any(
        (not target_id or observed_track_id == target_id) and int(history_size) >= 20
        for observed_track_id, history_size in GCS_TARGET_HISTORY_RE.findall(text)
    )


def command_test_midcourse(args: argparse.Namespace, config: FleetConfig, remote: RemoteExecutor) -> int:
    """部署一号板，连接用户手动启动的 SITL，并等待外部雷达数据完成验收。"""
    planes = selected_planes(config, args.count)
    if len(planes) != 1:
        raise FleetError("test-midcourse 当前要求 --count 1")
    plane = planes[0]
    run_id = new_run_id()
    task_id = "midcourse-" + run_id
    target_id = args.target_track_id.strip()
    run_dir = ROOT / "logs" / "fleet" / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    test_dir = f"/tmp/plane-midcourse-{run_id}"
    report = {
        "run_id": run_id,
        "status": "running",
        "plane": plane.plane_id,
        "host": plane.host,
        "task_id": task_id,
        "target_track_id": target_id,
        "checks": [],
        "flight_or_task_commands_sent": False,
    }

    def save() -> None:
        (run_dir / "midcourse-report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )

    def passed(name: str) -> None:
        report["checks"].append({"name": name, "status": "passed"})
        save()
        print(f"[midcourse] 通过: {name}", flush=True)

    def plane_log(stage: str) -> str:
        path = plane.runtime_dir + "/logs/plane.stdout.log"
        text = sanitize_output(remote.run(plane, f"cat {shlex.quote(path)} 2>/dev/null", check=False).stdout)
        (run_dir / f"{stage}-plane.log").write_text(text, encoding="utf-8")
        return text

    def gcs_log(stage: str) -> str:
        path = test_dir + "/gcs.log"
        text = sanitize_output(remote.run(plane, f"cat {shlex.quote(path)} 2>/dev/null", check=False).stdout)
        (run_dir / f"{stage}-gcs.log").write_text(text, encoding="utf-8")
        return text

    def wait_until(description: str, predicate, timeout_s: int) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.5)
        raise FleetError(f"等待超时: {description}")

    def stop_background_gcs() -> None:
        script = (
            f"pid_file={shlex.quote(test_dir + '/gcs.pid')}; "
            "if [ -s \"$pid_file\" ]; then pid=$(cat \"$pid_file\"); "
            "kill -TERM \"$pid\" 2>/dev/null || true; "
            "for i in $(seq 1 20); do kill -0 \"$pid\" 2>/dev/null || exit 0; sleep 0.1; done; fi"
        )
        remote.run(plane, script, timeout=10, check=False)

    save()
    try:
        preflight(config, planes, remote, include_simulator=False)
        bundle = ensure_bundle()
        gcs_binary = ROOT / "build" / "gcs_sim"
        if not gcs_binary.is_file():
            raise FleetError("缺少 build/gcs_sim，请先完成构建")

        deployment = prepare_plane_deployment(plane, bundle, remote, run_dir, config.simulator.host)
        configuration = deployment.configuration
        check_sitl_profile_listeners(config, (configuration,), remote)
        flight_endpoint = configuration.get("flight_control", "udp_url")
        if configuration.get("flight_control", "link_type") == "serial":
            flight_endpoint = f"serial://{configuration.get('flight_control', 'serial_device')}:{configuration.get('flight_control', 'serial_baud')}"
        mode = configuration.get("midcourse_algorithm", "control_output_mode", "VelocityAcceleration")
        offboard_started = {
            "VelocityAcceleration": "Offboard 速度及加速度启动",
            "Attitude": "Offboard 姿态启动",
        }.get(mode)
        if offboard_started is None:
            raise FleetError(f"control_output_mode 无效: {mode}")
        use_px4_limits = configuration.get("midcourse_algorithm", "use_px4_limits", "false").lower() in {"1", "true", "on"}
        report["flight_endpoint"] = flight_endpoint
        report["control_output_mode"] = mode
        report["backup"] = backup_plane_runtime(plane, remote, run_id)
        save()
        stop_planes(planes, remote)
        deploy_plane_bundle(plane, deployment, remote)
        check_remote_dependencies(plane, remote)
        remote.run(plane, f"mkdir -p {shlex.quote(test_dir)}")
        remote.rsync_to(plane, gcs_binary, test_dir + "/gcs_sim")
        deps = remote.run(
            plane,
            f"cd {shlex.quote(plane.runtime_dir)} && ./plane --deps-check && "
            "ldd ./plane | grep 'libmidcourse_alg.so' && ! ldd ./plane | grep -i pluma",
        )
        (run_dir / "deps.log").write_text(deps.stdout, encoding="utf-8")
        passed("部署包依赖自检通过，直连 libmidcourse_alg.so 且不含 Pluma")

        start_plane(plane, remote)
        wait_until(
            f"机载接入用户手动启动的仿真飞控 {config.simulator.host}",
            lambda: "飞控系统已接入" in plane_log("flight-wait")
            and "飞控连接建立" in plane_log("flight-wait")
            and (not use_px4_limits or "飞控系统参数已缓存" in plane_log("flight-wait"))
            and remote.run(plane, "pgrep -x plane >/dev/null", check=False).returncode == 0,
            config.verify_timeout_s,
        )
        passed(f"确认机载进程已接入手动 SITL {flight_endpoint}")

        # 先只启动 GCS 雷达桥并等待本机状态，确认过滤输入已就绪后再下发任务。
        gcs_command = shlex.join(
            [
                test_dir + "/gcs_sim",
                "--plane-id",
                plane.plane_id,
                "--sender-id",
                "GCS",
                "--listen-seconds",
                "0",
            ]
        )
        inner = (
            f"echo $$ >{shlex.quote(test_dir + '/gcs.pid')}; "
            f"exec env LD_LIBRARY_PATH={shlex.quote(plane.runtime_dir + '/lib')} {gcs_command} "
            f">{shlex.quote(test_dir + '/gcs.log')} 2>&1"
        )
        remote.run(plane, f"setsid -f bash -c {shlex.quote(inner)}")
        wait_until(
            "GCS 雷达桥收到本机状态",
            lambda: f"planeId={plane.plane_id}" in gcs_log("state-wait"),
            20,
        )
        state_text = gcs_log("state-ready")
        state_matches = re.findall(
            rf"planeId={re.escape(plane.plane_id)}[^\n]*geoPos.latitude=([-+0-9.eE]+) "
            rf"geoPos.longitude=([-+0-9.eE]+) geoPos.height=[-+0-9.eE]+ "
            rf"geoPos.altitude=([-+0-9.eE]+)", state_text)
        if not state_matches or abs(float(state_matches[-1][0])) < 1.0 or abs(float(state_matches[-1][1])) < 1.0:
            raise FleetError("仿真飞控尚未给出有效经纬度，禁止发送控制验收数据")
        passed("GCS 雷达桥已监听 0.0.0.0:7790，友机过滤已有有效本机状态")

        report["radar_destination"] = f"{plane.host}:7790"
        save()
        print(
            "\n[midcourse] 板端自检通过，请现在向 " + plane.host + ":7790 持续发送雷达 UDP 数据。\n"
            "[midcourse] 将先确认任务启动前已缓存至少 20 个严格递增扫描，再下发反无任务。\n"
            "[midcourse] 7790 只需发送雷达航迹，防守机状态由 GCS 从 Zenoh 获取。\n"
            + (f"[midcourse] 本次锁定靶机 track_id={target_id}。\n" if target_id else
               "[midcourse] 本次未指定 track_id，雷达数据必须只有一个可用靶机候选。\n"),
            flush=True,
        )

        wait_until(
            "任务启动前持续缓存 20 帧雷达历史",
            lambda: radar_history_ready(plane_log("radar-cache-wait"), target_id),
            args.radar_timeout,
        )
        passed("任务启动前已持续缓存至少 20 帧雷达历史")

        task_args = [
            test_dir + "/gcs_sim", "--plane-id", plane.plane_id, "--sender-id", "GCS",
            "--anti-uav", "--task-id", task_id,
        ]
        if target_id:
            task_args.extend(["--target-track-id", target_id])
        task_args.extend(["--listen-seconds", "1", "--no-radar-udp", "--no-presence", "--control-settle-ms", "500"])
        task_command = shlex.join(task_args)
        task_result = remote.run(
            plane, f"env LD_LIBRARY_PATH={shlex.quote(plane.runtime_dir + '/lib')} {task_command}", timeout=15)
        (run_dir / "start-gcs.log").write_text(task_result.stdout, encoding="utf-8")
        report["flight_or_task_commands_sent"] = True
        save()
        print(
            "\n[midcourse] 已下发反无任务，请保持雷达持续发送，以验证第 21 帧增量更新。\n"
            "[midcourse] GCS 会过滤 50 米内匹配到友机的航迹；友机未出现在雷达帧中不会阻塞靶机转发。\n",
            flush=True,
        )

        wait_until(
            f"外部雷达 20+1 帧到达并启动 {mode} Offboard",
            lambda: "中制导初始化完成" in plane_log("midcourse-wait")
            and "中制导目标估计器首次增量更新" in plane_log("midcourse-wait")
            and offboard_started in plane_log("midcourse-wait")
            and "value=Offboard" in plane_log("midcourse-wait"),
            args.radar_timeout,
        )
        text_before_stop = plane_log("midcourse-running")
        if target_id and f"trackId={target_id}" not in text_before_stop:
            raise FleetError(f"机载日志未收到指定靶机 {target_id}")
        time.sleep(4)
        if remote.run(plane, "pgrep -x plane >/dev/null", check=False).returncode != 0:
            raise FleetError("中制导启动后机载进程退出")
        running_text = plane_log("midcourse-sustained")
        if "行为树结束: currentBehaviorTree=AntiUav" in running_text:
            raise FleetError("AntiUav 在外部停止前自行结束")
        cache_index = running_text.find("飞控系统参数已缓存")
        session_index = running_text.find("中制导会话启动")
        set_config_index = running_text.find("中制导规划器配置完成")
        estimator_init_index = running_text.find("中制导初始化完成")
        planner_update_index = running_text.find(offboard_started)
        estimator_update_index = running_text.find("中制导目标估计器首次增量更新")
        if use_px4_limits and not (0 <= cache_index < session_index):
            raise FleetError("飞控系统参数未在中制导会话启动前完成缓存")
        if not (0 <= set_config_index < estimator_init_index < planner_update_index < estimator_update_index):
            raise FleetError("中制导接口调用顺序不符合 setConfig → init → planner.update → estimator.update")
        passed(f"启动前 20 帧缓存、接口调用顺序、第 21 帧更新、{mode} Offboard 与持续运行验证通过")

        stop_command = shlex.join(
            [
                test_dir + "/gcs_sim",
                "--plane-id",
                plane.plane_id,
                "--sender-id",
                "GCS",
                "--stop-task",
                "--task-id",
                task_id,
                "--listen-seconds",
                "1",
                "--no-radar-udp",
                "--no-presence",
                "--control-settle-ms",
                "500",
            ]
        )
        stop_result = remote.run(
            plane,
            f"env LD_LIBRARY_PATH={shlex.quote(plane.runtime_dir + '/lib')} {stop_command}",
            timeout=15,
        )
        (run_dir / "stop-gcs.log").write_text(stop_result.stdout, encoding="utf-8")
        wait_until(
            "任务停止并退出 Offboard",
            lambda: "任务中止: taskId=" + task_id in plane_log("stop-wait")
            and "Offboard 停止" in plane_log("stop-wait"),
            15,
        )
        passed("匹配 taskId 的 Task Stop 已停止 Offboard 并清理会话")
        stop_background_gcs()
        report["status"] = "passed"
        save()
        print(f"[result] 中制导板端验证通过；机载与手动 SITL 保持运行。报告: {run_dir / 'midcourse-report.json'}")
        return 0
    except (FleetError, OSError, ValueError, configparser.Error) as exc:
        stop_background_gcs()
        report["status"] = "failed"
        report["error"] = str(exc)
        save()
        plane_log("failure")
        gcs_log("failure")
        raise FleetError(f"中制导板端验证失败: {exc}；证据目录: {run_dir}") from exc


def command_test(args: argparse.Namespace, config: FleetConfig, remote: RemoteExecutor) -> int:
    planes = selected_planes(config, args.count)
    run_id = new_run_id()
    run_dir = ROOT / "logs" / "fleet" / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    print(f"[run] run_id={run_id}, 证据目录={run_dir}")
    preflight(config, planes, remote)
    bundle = ensure_bundle()
    deployments = [prepare_plane_deployment(plane, bundle, remote, run_dir, config.simulator.host) for plane in planes]
    flight_ports = configured_sitl_ports(tuple(item.configuration for item in deployments))
    for plane in planes:
        backup_plane_runtime(plane, remote, run_id)
    stop_planes(planes, remote)
    for plane in planes:
        install_system_dependencies(config, plane, remote)
    endpoints = EndpointSet((), ())
    if flight_ports:
        stop_simulator(config, remote)
        endpoints = start_simulator(config, flight_ports, remote, run_dir, start_count=len(planes))
    for plane, deployment in zip(planes, deployments):
        deploy_plane_bundle(plane, deployment, remote)
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
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="测试环境清单（默认 tests/fleet.ini）")
    subparsers = parser.add_subparsers(dest="command", required=True)
    for name, help_text in (
        ("preflight", "只执行环境和连接预检"),
        ("test", "启动、同步并验证多机状态互通"),
        ("test-midcourse", "部署一号机，连接手动 SITL 并等待外部雷达验证中制导"),
        ("test-agent-info", "部署一号机并验证 AgentInfo 上报、重连和原生查询，不重启 SITL"),
        ("test-presence", "部署双机并验证上线、退出、崩溃和恢复，不重启 SITL"),
        ("verify", "复核一个已启动运行现场的多机状态互通"),
        ("down", "停止机载程序和 SITL"),
    ):
        command = subparsers.add_parser(name, help=help_text)
        command.add_argument("--count", type=int, required=True, help="请求的飞机数量")
        if name == "test-midcourse":
            command.add_argument("--radar-timeout", type=int, default=300, help="等待外部雷达数据的秒数")
            command.add_argument("--target-track-id", default="", help="锁定的雷达 track_id；留空时要求唯一靶机")
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
        if args.command == "test-agent-info":
            return command_test_agent_info(args, config, remote)
        if args.command == "test-midcourse":
            return command_test_midcourse(args, config, remote)
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
