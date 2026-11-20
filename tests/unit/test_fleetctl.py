#!/usr/bin/env python3

import argparse
import dataclasses
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("fleetctl", ROOT / "tools" / "fleetctl.py")
assert SPEC and SPEC.loader
fleetctl = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = fleetctl
SPEC.loader.exec_module(fleetctl)


class FleetCtlTest(unittest.TestCase):
    def configuration(self, plane, main="", profile=None, fc_url="udpout://192.168.1.40:14280"):
        if profile is None:
            profile = (
                f"[{plane.plane_id}]\nboard_ip = {plane.host}\nfc_url = {fc_url}\n"
                "pod_model = jielian_a2416a\npod_video_url = rtsp://pod/live?a=1&b=2\n"
                "pod_control_url = udp://pod:8001\n"
            )
        main = "[deployment]\nprofile = config/profiles/site.ini\n" + main
        settings = fleetctl.deployment_settings("config/profiles/site.ini", profile, {plane.host})
        return fleetctl.PlaneConfiguration(main, "config/profiles/site.ini", profile, tuple(settings))

    def test_log_keys_keep_original_code_and_px4_names(self):
        sources = "\n".join(
            path.read_text(encoding="utf-8")
            for root in (ROOT / "src", ROOT / "tools" / "gcs_sim")
            for path in root.rglob("*")
            if path.suffix in {".cpp", ".h"}
        )
        for translated_key in (
            "最大水平速度米每秒=",
            "最大爬升速度米每秒=",
            "最大下降速度米每秒=",
            "最大水平加速度米每二次方秒=",
            "飞机ID=",
            "地面站ID=",
            "发送方ID=",
            "任务ID=",
            "航迹ID=",
            "时间戳毫秒=",
            "安全高度米=",
            "滚转角度=",
        ):
            self.assertNotIn(translated_key, sources)
        self.assertIn("MPC_XY_VEL_MAX={:.2f}", sources)
        self.assertIn("MPC_Z_VEL_MAX_UP={:.2f}", sources)
        self.assertIn("MPC_Z_VEL_MAX_DN={:.2f}", sources)
        self.assertIn("MPC_ACC_HOR_MAX={:.2f}", sources)
        self.assertNotIn("MidcourseLimits", sources)
        self.assertNotIn("midcourse_limits", sources)
        self.assertIn("FlightControlParams", sources)
        self.assertIn("飞控系统参数已缓存", sources)
        self.assertIn("[%^%l%$] [%t] %s:%# %v", sources)
        self.assertNotIn("[线程%t]", sources)

    def test_parse_dynamic_endpoints_by_uri_and_port(self):
        output = """
        vehicle B video: rtsp://192.168.1.56:8554/two
        任意文本 udpout://127.0.0.1:14281
        vehicle A udp://0.0.0.0:14280 extra
        video rtsp://192.168.1.56:8554/one
        """
        result = fleetctl.parse_endpoints(output, "192.168.1.56", (14280, 14281))
        self.assertEqual(
            result.flight_urls,
            ("udpout://192.168.1.56:14280", "udpout://192.168.1.56:14281"),
        )

    def test_parse_rejects_empty_duplicate_or_invalid_package_ports(self):
        for ports in ((), (14280, 14280), (0,), (65536,)):
            with self.subTest(ports=ports), self.assertRaises(fleetctl.FleetError):
                fleetctl.parse_endpoints("udpout://192.168.1.56:14280", "192.168.1.56", ports)

    def test_package_flight_ports_are_authoritative_over_sitl_output(self):
        listeners = """
        UNCONN 0 0 0.0.0.0:14280 0.0.0.0:*
        UNCONN 0 0 0.0.0.0:14281 0.0.0.0:*
        """
        ports = fleetctl.validate_flight_listeners((14280, 14281), listeners)
        result = fleetctl.parse_endpoints(
            "udpout://192.168.1.56:14580 udpout://192.168.1.56:14581",
            "192.168.1.56",
            ports,
        )
        self.assertEqual(
            result.flight_urls,
            ("udpout://192.168.1.56:14280", "udpout://192.168.1.56:14281"),
        )
        self.assertIn("udpout://192.168.1.56:14580", result.all_uris)

    def test_configured_flight_ports_must_be_listening(self):
        with self.assertRaises(fleetctl.FleetError):
            fleetctl.validate_flight_listeners((14280, 14281), "0.0.0.0:14280 ")

    def test_start_simulator_uses_package_ports_without_requiring_old_base_port(self):
        config = fleetctl.load_config(fleetctl.DEFAULT_CONFIG)
        for port, start_count, expected_count in ((14280, None, 1), (14281, 2, 2)):
            with self.subTest(port=port), tempfile.TemporaryDirectory(prefix="fleet package ports ") as directory:
                remote = Mock()
                remote.stream.return_value = (0, "旧端点 udpout://192.168.1.56:14580\n吊舱 udp://192.168.1.56:8001\n")
                remote.run.return_value.stdout = f"UNCONN 0 0 0.0.0.0:{port} 0.0.0.0:*\n"
                endpoints = fleetctl.start_simulator(config, (port,), remote, Path(directory), start_count=start_count)
                self.assertEqual(endpoints.flight_urls, (f"udpout://192.168.1.56:{port}",))
                self.assertIn("udpout://192.168.1.56:14580", endpoints.all_uris)
                self.assertIn(f"-n {expected_count}", remote.stream.call_args.args[1])
                self.assertEqual(json.loads((Path(directory) / "endpoints.json").read_text())["flight_urls"], [f"udpout://192.168.1.56:{port}"])
                remote.run.assert_called_once_with(config.simulator, "ss -H -lun", timeout=15)

    def test_ini_update_preserves_other_sections(self):
        source = "[communication]\nplane_id = old\n\n[flight_control]\nudp_url = old\n"
        updated = fleetctl.set_ini_value(source, "communication", "plane_id", "uav2")
        self.assertIn("plane_id = uav2", updated)
        self.assertIn("[flight_control]\nudp_url = old", updated)

    def test_inventory_rejects_more_planes_than_declared(self):
        config = fleetctl.load_config(fleetctl.DEFAULT_CONFIG)
        with self.assertRaises(fleetctl.FleetError):
            fleetctl.selected_planes(config, len(config.planes) + 1)

    def test_test_inventory_path_defaults_and_can_be_overridden(self):
        parser = fleetctl.build_parser()
        command = ["preflight", "--count", "1"]
        self.assertEqual(parser.parse_args(command).config, ROOT / "tests" / "fleet.ini")
        self.assertEqual(parser.parse_args(["--config", "site.ini", *command]).config, Path("site.ini"))

    def write_bundle(self, directory, plane, *, main=None, profile=None, fc_url="udpout://192.168.1.40:14280"):
        common = "[system]\nsafe_height_m = 20\nlog_dir = logs\n[camera]\npod_state_report_hz = 7\n"
        configuration = self.configuration(plane, common if main is None else main, profile=profile, fc_url=fc_url)
        bundle = Path(directory) / "bundle"
        (bundle / fleetctl.PROFILE_DIR).mkdir(parents=True)
        (bundle / "lib").mkdir()
        for filename in ("plane", "runplane.sh", "stopplane.sh", "install-plane-service.sh", "README.md"):
            (bundle / filename).write_text("包内文件 " + filename, encoding="utf-8")
        for preset in (ROOT / fleetctl.PROFILE_DIR).glob("*.ini"):
            (bundle / fleetctl.PROFILE_DIR / preset.name).write_text("包内未选中预置 " + preset.name, encoding="utf-8")
        (bundle / "config/profiles/unused.ini").write_text("包内未选中预置", encoding="utf-8")
        (bundle / "config/zenoh.json5").write_text("{}", encoding="utf-8")
        (bundle / fleetctl.PLANE_CONFIG).write_text(configuration.main, encoding="utf-8")
        (bundle / configuration.profile_path).write_text(configuration.profile_text, encoding="utf-8")
        (bundle / "lib/test.so").write_text("包内动态库", encoding="utf-8")
        return bundle

    def local_remote(self, addresses):
        remote = Mock()
        interfaces = [{"flags": ["UP"], "addr_info": [{"family": "inet", "local": address} for address in addresses]}]
        def run_local(_plane, script, **kwargs):
            if script == "ip -j -4 address show up":
                return subprocess.CompletedProcess([], 0, stdout=json.dumps(interfaces))
            return subprocess.run(["bash", "-c", script], text=True, capture_output=True, check=kwargs.get("check", True))
        def sync(_plane, source, destination, extra=()):
            subprocess.run(["rsync", "-a", *extra, str(source), destination], check=True, capture_output=True, text=True)
        remote.run.side_effect = run_local
        remote.rsync_to.side_effect = sync
        return remote

    def test_package_common_parameters_are_preserved_verbatim(self):
        plane = fleetctl.load_config(fleetctl.DEFAULT_CONFIG).planes[0]
        main = "# 通用参数\n[system]\nsafe_height_m = 20\nlog_dir = logs\n[communication]\nack_timeout_ms = 800\n"
        configuration = self.configuration(plane, main)
        rendered = fleetctl.configure_plane(configuration, plane, "192.168.1.56")
        self.assertEqual(rendered.main, configuration.main)
        self.assertIn("fc_url = udpout://192.168.1.56:14280", rendered.profile_text)

    def test_fleet_environment_and_dependency_order(self):
        config = fleetctl.load_config(fleetctl.DEFAULT_CONFIG)
        self.assertEqual(config.board_arch, "aarch64")
        self.assertEqual(config.board_os_version, "22.04")
        self.assertEqual(config.apt_bootstrap_packages, ())
        self.assertEqual(config.apt_packages, ("libzmq5",))

    def test_verify_command_requires_run_id(self):
        parser = fleetctl.build_parser()
        with self.assertRaises(SystemExit):
            parser.parse_args(["verify", "--count", "2"])
        args = parser.parse_args(["verify", "--count", "2", "--run-id", "20261016-000000"])
        self.assertEqual(args.run_id, "20261016-000000")

    def test_peer_presence_uses_latest_event_across_duplicate_logs(self):
        planes = fleetctl.load_config(fleetctl.DEFAULT_CONFIG).planes[:2]
        online = "[2026-10-27 18:00:01.000] [info] 他机上线: uav2\n"
        offline = "[2026-10-27 18:00:02.000] [info] 他机下线: uav2\n"
        result = fleetctl.analyze_plane(planes[0], planes, online + offline + online)
        self.assertFalse(result.peer_online["uav2"])
        recovered = "[2026-10-27 18:00:03.000] [info] 他机上线: uav2\n"
        result = fleetctl.analyze_plane(planes[0], planes, recovered + online + offline)
        self.assertTrue(result.peer_online["uav2"])

    def test_old_run_cannot_satisfy_current_presence_check(self):
        planes = fleetctl.load_config(fleetctl.DEFAULT_CONFIG).planes[:2]
        text = (
            "[2026-10-27 17:00:00.000] [info] 他机上线: uav2\n"
            "[2026-10-27 17:00:01.000] [info] 飞控连接建立\n"
            "[2026-10-27 18:00:00.000] [info] plane 程序启动: version=0.1.0\n"
        )
        result = fleetctl.analyze_plane(planes[0], planes, text)
        self.assertFalse(result.peer_online["uav2"])
        self.assertFalse(result.flight_connected)

    def test_spdlog_error_level_is_detected(self):
        planes = fleetctl.load_config(fleetctl.DEFAULT_CONFIG).planes[:1]
        text = (
            "[2026-10-30 18:00:00.000] [info] plane 程序启动: version=0.1.0\n"
            "[2026-10-30 18:00:01.000] [error] 飞控连接失败\n"
        )
        result = fleetctl.analyze_plane(planes[0], planes, text)
        self.assertEqual(result.errors, ["[2026-10-30 18:00:01.000] [error] 飞控连接失败"])

    def test_presence_deployment_preserves_common_parameters_and_serial_profile(self):
        plane = fleetctl.load_config(fleetctl.DEFAULT_CONFIG).planes[1]
        source = self.configuration(
            plane, "[system]\nsafe_height_m = 25\n[camera]\npod_state_report_hz = 7\n"
            "[midcourse_algorithm]\nv_cruise_m_s = 12\n",
            fc_url="serial:///dev/test:230400",
        )
        result = fleetctl.configure_plane(source, plane, "192.168.1.56")
        self.assertEqual(result.profile_text, source.profile_text)
        self.assertIn("safe_height_m = 25", result.main)
        self.assertIn("pod_state_report_hz = 7", result.main)
        self.assertIn("v_cruise_m_s = 12", result.main)
        self.assertIn("profile = config/profiles/site.ini", result.main)
        self.assertEqual(result.get("flight_control", "serial_device"), "/dev/test")
        self.assertEqual(result.get("flight_control", "serial_baud"), "230400")
        self.assertNotIn("serial_device", result.main)

    def test_presence_sitl_address_uses_inventory_host_and_preserves_port(self):
        config = fleetctl.load_config(fleetctl.DEFAULT_CONFIG)
        plane = config.planes[1]
        for port in (14031, 14281, 14581):
            with self.subTest(port=port):
                base = self.configuration(plane, fc_url=f"udpout://192.168.1.40:{port}")
                result = fleetctl.configure_plane(base, plane, config.simulator.host)
                self.assertIn(f"fc_url = udpout://192.168.1.56:{port}", result.profile_text)
                self.assertNotIn("192.168.1.40", result.profile_text)
                self.assertNotIn("udp_url", result.main)
        listener = self.configuration(plane, fc_url="udpin://0.0.0.0:14281")
        result = fleetctl.configure_plane(listener, plane, config.simulator.host)
        self.assertIn("fc_url = udpin://0.0.0.0:14281", result.profile_text)

    def test_bundle_config_ignores_missing_empty_broken_and_legacy_board_configuration(self):
        inventory_plane = fleetctl.load_config(fleetctl.DEFAULT_CONFIG).planes[0]
        for old_config in (None, "", "broken old ini", "[deployment]\nprofile = /nonexistent/site.ini\n"):
            with self.subTest(old_config=old_config), tempfile.TemporaryDirectory(prefix="fleet source ") as directory:
                root = Path(directory)
                plane = dataclasses.replace(inventory_plane, runtime_dir=str(root / "runtime"))
                (Path(plane.runtime_dir) / "config").mkdir(parents=True)
                if old_config is not None:
                    (Path(plane.runtime_dir) / fleetctl.PLANE_CONFIG).write_text(old_config, encoding="utf-8")
                (Path(plane.runtime_dir) / "config/plane.conf").write_text("不应读取旧文件", encoding="utf-8")
                bundle = self.write_bundle(root, plane)
                remote = self.local_remote({plane.host})
                configuration = fleetctl.read_bundle_config(plane, remote, bundle)
                self.assertEqual(configuration.main, (bundle / fleetctl.PLANE_CONFIG).read_text())
                self.assertEqual(configuration.get("system", "safe_height_m"), "20")
                self.assertEqual(configuration.get("system", "board_ip"), plane.host)
                remote.run.assert_called_once_with(plane, "ip -j -4 address show up")
                remote.rsync_to.assert_not_called()

    def test_bundle_config_requires_actual_unique_board_match_and_inventory_identity(self):
        plane = fleetctl.load_config(fleetctl.DEFAULT_CONFIG).planes[0]
        profile = (
            f"[uav1]\nboard_ip = {plane.host}\nfc_url = serial:///dev/ttyS0:115200\n"
            "[uav2]\nboard_ip = 192.168.1.101\nfc_url = udpout://192.168.1.56:14281\n"
        )
        with tempfile.TemporaryDirectory(prefix="fleet identity ") as directory:
            bundle = self.write_bundle(directory, plane, profile=profile)
            for addresses, expected in (({"192.168.1.200"}, "未匹配"), ({plane.host, "192.168.1.101"}, "多个飞机 ID"), ({"192.168.1.101"}, "与测试清单不一致")):
                with self.subTest(addresses=addresses), self.assertRaisesRegex(fleetctl.FleetError, expected):
                    fleetctl.read_bundle_config(plane, self.local_remote(addresses), bundle)
            dns_plane = dataclasses.replace(plane, host="uav1.local")
            configuration = fleetctl.read_bundle_config(dns_plane, self.local_remote({plane.host}), bundle)
            self.assertEqual(configuration.get("system", "board_ip"), plane.host)

    def test_bundle_profile_must_be_present_relative_and_within_package(self):
        plane = fleetctl.load_config(fleetctl.DEFAULT_CONFIG).planes[0]
        with tempfile.TemporaryDirectory(prefix="fleet profile ") as directory:
            bundle = self.write_bundle(directory, plane)
            outside = Path(directory) / "outside.ini"
            outside.write_text((bundle / "config/profiles/site.ini").read_text(), encoding="utf-8")
            (bundle / "config/profiles/link.ini").symlink_to(outside)
            for value, expected in (("", "缺失或为空"), (str(outside), "包内相对路径"), ("../outside.ini", "超出运行包"), ("config/profiles/link.ini", "超出运行包"), ("missing.ini", "包内文件不存在"), (".", "包内文件不存在")):
                with self.subTest(value=value):
                    (bundle / fleetctl.PLANE_CONFIG).write_text(f"[deployment]\nprofile = {value}\n", encoding="utf-8")
                    remote = self.local_remote({plane.host})
                    with self.assertRaisesRegex(fleetctl.FleetError, expected):
                        fleetctl.read_bundle_config(plane, remote, bundle)
                    remote.run.assert_not_called()

    def test_profile_and_native_configuration_cannot_reference_preserved_logs(self):
        plane = fleetctl.load_config(fleetctl.DEFAULT_CONFIG).planes[0]
        with tempfile.TemporaryDirectory(prefix="fleet reserved ") as directory:
            bundle = self.write_bundle(directory, plane)
            main = (bundle / fleetctl.PLANE_CONFIG).read_text()
            (bundle / "logs").mkdir()
            (bundle / "logs/site.ini").write_text((bundle / "config/profiles/site.ini").read_text(), encoding="utf-8")
            (bundle / "logs/zenoh.json5").write_text("{}", encoding="utf-8")
            (bundle / "config/profiles/loglink.ini").symlink_to(bundle / "logs/site.ini")
            for section, key, reference in (
                ("deployment", "profile", "logs/site.ini"),
                ("deployment", "profile", "config/../logs/site.ini"),
                ("deployment", "profile", "config/profiles/loglink.ini"),
                ("communication", "zenoh_config_file", "logs/zenoh.json5"),
                ("communication", "zenoh_config_file", "config/../logs/zenoh.json5"),
            ):
                with self.subTest(reference=reference):
                    (bundle / fleetctl.PLANE_CONFIG).write_text(fleetctl.set_ini_value(main, section, key, reference), encoding="utf-8")
                    remote = self.local_remote({plane.host})
                    with self.assertRaisesRegex(fleetctl.FleetError, "不得引用保留的 logs 目录"):
                        fleetctl.prepare_plane_deployment(plane, bundle, remote, Path(directory) / "run", "192.168.1.56")
                    remote.run.assert_not_called()
                    remote.rsync_to.assert_not_called()

    def test_native_configuration_reference_must_be_existing_package_file(self):
        plane = fleetctl.load_config(fleetctl.DEFAULT_CONFIG).planes[0]
        with tempfile.TemporaryDirectory(prefix="fleet native ") as directory:
            bundle = self.write_bundle(directory, plane)
            main = (bundle / fleetctl.PLANE_CONFIG).read_text()
            outside = Path(directory) / "zenoh.json5"
            outside.write_text("{}", encoding="utf-8")
            for reference, expected in (("missing.json5", "包内文件不存在"), (str(outside), "包内相对路径"), ("../zenoh.json5", "超出运行包")):
                with self.subTest(reference=reference):
                    (bundle / fleetctl.PLANE_CONFIG).write_text(fleetctl.set_ini_value(main, "communication", "zenoh_config_file", reference), encoding="utf-8")
                    remote = self.local_remote({plane.host})
                    with self.assertRaisesRegex(fleetctl.FleetError, expected):
                        fleetctl.prepare_plane_deployment(plane, bundle, remote, Path(directory) / "run", "192.168.1.56")
                    remote.run.assert_not_called()
            (bundle / "config/zenoh.json5").unlink()
            self.assertIn("config/zenoh.json5", fleetctl.bundle_problems(bundle))
            (bundle / "lib/test.so").unlink()
            self.assertIn("lib/（为空）", fleetctl.bundle_problems(bundle))

    def test_ensure_bundle_runs_package_dependency_check_before_returning(self):
        plane = fleetctl.load_config(fleetctl.DEFAULT_CONFIG).planes[0]
        with tempfile.TemporaryDirectory(prefix="fleet deps ") as directory:
            root = Path(directory)
            source = self.write_bundle(root, plane)
            (root / "build").mkdir()
            bundle = root / "build/plane"
            source.rename(bundle)
            binary = bundle / "plane"
            binary.write_text("#!/bin/sh\nprintf '缺少测试运行库\\n' >&2\nexit 1\n", encoding="utf-8")
            binary.chmod(0o755)
            with patch.object(fleetctl, "ROOT", root):
                with self.assertRaisesRegex(fleetctl.FleetError, "依赖自检失败: 缺少测试运行库"):
                    fleetctl.ensure_bundle()
                binary.write_text("#!/bin/sh\n[ \"$1\" = '--deps-check' ]\n", encoding="utf-8")
                self.assertEqual(fleetctl.ensure_bundle(), bundle)

    def test_profile_zenoh_endpoints_sort_peers_and_allow_optional_groundstation(self):
        local_ip = "192.168.1.100"
        single_aircraft = f"[uav1]\nboard_ip = {local_ip}\nfc_url = serial:///dev/ttyS0:115200\n"
        aircraft = single_aircraft + (
            "[uav2]\nboard_ip = 192.168.1.11\nfc_url = serial:///dev/ttyS0:115200\n"
            "[uav3]\nboard_ip = 192.168.1.2\nfc_url = serial:///dev/ttyS0:115200\n"
        )
        peers = "tcp/192.168.1.2:7447,tcp/192.168.1.11:7447"
        for groundstation, expected in (
            ("", peers),
            ("[groundstation]\nip =\n", peers),
            ("[groundstation]\nip = 192.168.1.200\n", f"tcp/192.168.1.200:7447,{peers}"),
        ):
            with self.subTest(groundstation=groundstation):
                settings = fleetctl.deployment_settings(
                    "profile.ini", groundstation + aircraft, {local_ip}
                )
                self.assertIn(("communication", "zenoh_listen", f"tcp/{local_ip}:7447"), settings)
                self.assertIn(("communication", "zenoh_connect", expected), settings)

        only_one = fleetctl.deployment_settings(
            "profile.ini", single_aircraft, {local_ip}
        )
        self.assertIn(("communication", "zenoh_connect", ""), only_one)

    def test_profile_zenoh_rejects_invalid_or_conflicting_groundstation(self):
        local_ip = "192.168.1.100"
        peer_ip = "192.168.1.101"
        aircraft = (
            f"[uav1]\nboard_ip = {local_ip}\nfc_url = serial:///dev/ttyS0:115200\n"
            f"[uav2]\nboard_ip = {peer_ip}\nfc_url = serial:///dev/ttyS0:115200\n"
        )
        for groundstation_ip in (
            "groundstation", "::1", "0.1.2.3", "127.0.0.1", "224.0.0.1", "240.0.0.1",
            "255.255.255.255", local_ip, peer_ip,
        ):
            with self.subTest(groundstation_ip=groundstation_ip):
                text = aircraft + f"[groundstation]\nip = {groundstation_ip}\n"
                with self.assertRaisesRegex(fleetctl.FleetError, r"groundstation\.ip"):
                    fleetctl.deployment_settings("profile.ini", text, {local_ip})

    def test_metadata_configuration_uses_profile_and_preserves_other_aircraft(self):
        plane = fleetctl.load_config(fleetctl.DEFAULT_CONFIG).planes[0]
        profile = (
            "# 现场机群\n[groundstation]\nip = 192.168.1.41\n"
            f"[uav1]\nboard_ip = {plane.host}\nfc_url = udpout://192.168.1.40:14280\n"
            "pod_video_url = rtsp://pod/live?a=1&b=2\npod_control_url = udp://pod:8001\n"
            "[uav2]\nboard_ip = 192.168.1.101\nfc_url = serial:///dev/test:230400\n"
            "pod_model = zhongke_d80\npod_video_url = rtsp://second/live\n"
        )
        source = self.configuration(plane, profile=profile)
        result = fleetctl.configure_plane(source, plane, "192.168.1.56")
        self.assertEqual(result.profile_text, profile.replace("udpout://192.168.1.40:14280", "udpout://192.168.1.56:14280"))
        self.assertEqual(result.get("camera", "pod_video_url"), "rtsp://pod/live?a=1&b=2")
        self.assertEqual(result.get("system", "board_ip"), plane.host)
        self.assertEqual(result.get("communication", "zenoh_connect"), "tcp/192.168.1.41:7447,tcp/192.168.1.101:7447")
        self.assertNotIn("pod_video_url", result.main)

    def test_full_deployment_overwrites_all_configuration_and_removes_old_files_preserving_logs(self):
        inventory_plane = fleetctl.load_config(fleetctl.DEFAULT_CONFIG).planes[0]
        with tempfile.TemporaryDirectory(prefix="fleet full sync ") as directory:
            root = Path(directory)
            plane = dataclasses.replace(inventory_plane, runtime_dir=str(root / "runtime"))
            runtime = Path(plane.runtime_dir)
            (runtime / "config/profiles").mkdir(parents=True)
            (runtime / "logs/core").mkdir(parents=True)
            (runtime / fleetctl.PLANE_CONFIG).write_text("[system]\nsafe_height_m = 30\n", encoding="utf-8")
            (runtime / "config/profiles/site.ini").write_text("旧当前预置", encoding="utf-8")
            (runtime / "config/profiles/unused.ini").write_text("旧未选中预置", encoding="utf-8")
            old_files = ("config/plane.conf", "config/profiles/obsolete.ini", "old.tmp", "lib/obsolete.so")
            for name in old_files:
                (runtime / name).parent.mkdir(parents=True, exist_ok=True)
                (runtime / name).write_text("旧文件", encoding="utf-8")
            (runtime / "logs/plane.log").write_text("既有日志", encoding="utf-8")
            (runtime / "logs/core/core.keep").write_bytes(b"existing core")
            bundle = self.write_bundle(root, plane)
            (bundle / "logs/core").mkdir(parents=True)
            (bundle / "logs/plane.log").write_text("包内日志不可覆盖板端日志", encoding="utf-8")
            (bundle / "logs/core/from-package").write_bytes(b"must not upload")
            remote = self.local_remote({plane.host})
            run_dir = root / "run"
            run_dir.mkdir()
            deployment = fleetctl.prepare_plane_deployment(plane, bundle, remote, run_dir, "192.168.1.56")
            backup = Path(fleetctl.backup_plane_runtime(plane, remote, "test"))
            fleetctl.deploy_plane_bundle(plane, deployment, remote)
            self.assertEqual((runtime / fleetctl.PLANE_CONFIG).read_text(), (bundle / fleetctl.PLANE_CONFIG).read_text())
            self.assertIn("safe_height_m = 20", (runtime / fleetctl.PLANE_CONFIG).read_text())
            self.assertIn("fc_url = udpout://192.168.1.56:14280", (runtime / "config/profiles/site.ini").read_text())
            self.assertEqual((runtime / "config/profiles/unused.ini").read_text(), "包内未选中预置")
            for preset in (bundle / fleetctl.PROFILE_DIR).glob("*.ini"):
                if preset.name != "site.ini":
                    self.assertEqual((runtime / fleetctl.PROFILE_DIR / preset.name).read_bytes(), preset.read_bytes())
            for name in old_files:
                self.assertFalse((runtime / name).exists(), name)
            self.assertEqual((runtime / "logs/plane.log").read_text(), "既有日志")
            self.assertEqual((runtime / "logs/core/core.keep").read_bytes(), b"existing core")
            self.assertFalse((runtime / "logs/core/from-package").exists())
            self.assertIn("safe_height_m = 30", (backup / fleetctl.PLANE_CONFIG).read_text())
            self.assertEqual((bundle / "config/profiles/site.ini").read_text().count("192.168.1.40"), 1)
            self.assertEqual(remote.rsync_to.call_count, 1)
            self.assertEqual(remote.rsync_to.call_args.kwargs["extra"], ("--delete", "--exclude=/logs/"))

    def test_invalid_bundle_fails_preparation_before_remote_mutation(self):
        plane = fleetctl.load_config(fleetctl.DEFAULT_CONFIG).planes[0]
        with tempfile.TemporaryDirectory(prefix="fleet invalid ") as directory:
            root = Path(directory)
            profile = f"[another-id]\nboard_ip = {plane.host}\nfc_url = serial:///dev/test:230400\n"
            bundle = self.write_bundle(root, plane, profile=profile)
            remote = self.local_remote({plane.host})
            with self.assertRaisesRegex(fleetctl.FleetError, "与测试清单不一致"):
                fleetctl.prepare_plane_deployment(plane, bundle, remote, root / "run", "192.168.1.56")
            remote.rsync_to.assert_not_called()
            self.assertFalse((root / "run").exists())
            (bundle / "plane").unlink()
            remote.reset_mock()
            with self.assertRaisesRegex(fleetctl.FleetError, "运行包不完整"):
                fleetctl.prepare_plane_deployment(plane, bundle, remote, root / "run", "192.168.1.56")
            remote.run.assert_not_called()

    def test_each_aircraft_gets_complete_isolated_package_directory(self):
        config = fleetctl.load_config(fleetctl.DEFAULT_CONFIG)
        profile = (
            "[uav1]\nboard_ip = 192.168.1.100\nfc_url = udpout://192.168.1.40:14280\n"
            "[uav2]\nboard_ip = 192.168.1.101\nfc_url = udpout://192.168.1.40:14581\n"
        )
        with tempfile.TemporaryDirectory(prefix="fleet staged ") as directory:
            root = Path(directory)
            bundle = self.write_bundle(root, config.planes[0], profile=profile)
            run_dir = root / "run"
            run_dir.mkdir()
            deployments = [fleetctl.prepare_plane_deployment(plane, bundle, self.local_remote({plane.host}), run_dir, config.simulator.host) for plane in config.planes[:2]]
            self.assertNotEqual(deployments[0].directory, deployments[1].directory)
            for plane, deployment in zip(config.planes, deployments):
                self.assertEqual((deployment.directory / "config/profiles/unused.ini").read_text(), "包内未选中预置")
                self.assertEqual((deployment.directory / fleetctl.PLANE_CONFIG).read_bytes(), (bundle / fleetctl.PLANE_CONFIG).read_bytes())
                self.assertEqual(deployment.configuration.get("communication", "plane_id"), plane.plane_id)
                self.assertTrue((deployment.directory / "lib/test.so").is_file())
            self.assertEqual(deployments[0].configuration.get("flight_control", "udp_url"), "udpout://192.168.1.56:14280")
            self.assertEqual(deployments[1].configuration.get("flight_control", "udp_url"), "udpout://192.168.1.56:14581")
            self.assertEqual((bundle / "config/profiles/site.ini").read_text(), profile)

    def test_all_deployment_commands_reject_invalid_package_before_stopping_or_uploading(self):
        config = fleetctl.load_config(fleetctl.DEFAULT_CONFIG)
        for command, count in (("test", 1), ("test-midcourse", 1), ("test-agent-info", 1), ("test-presence", 2)):
            with self.subTest(command=command), tempfile.TemporaryDirectory(prefix="fleet rejected ") as directory:
                temporary_root = Path(directory)
                (temporary_root / "build").mkdir()
                (temporary_root / "build/gcs_sim").touch()
                bundle = self.write_bundle(temporary_root, config.planes[0])
                (bundle / fleetctl.PLANE_CONFIG).write_text("[deployment]\nprofile = \n", encoding="utf-8")
                remote = self.local_remote({config.planes[0].host})
                args = fleetctl.build_parser().parse_args([command, "--count", str(count)])
                with patch.object(fleetctl, "ROOT", temporary_root), \
                     patch.object(fleetctl, "preflight"), \
                     patch.object(fleetctl, "ensure_bundle", return_value=bundle), \
                     patch.object(fleetctl, "stop_planes") as stop, \
                     patch.object(fleetctl, "backup_plane_runtime") as backup, \
                     patch.object(fleetctl, "deploy_plane_bundle") as upload:
                    function = getattr(fleetctl, "command_" + command.replace("-", "_"))
                    with self.assertRaisesRegex(fleetctl.FleetError, r"deployment\.profile"):
                        function(args, config, remote)
                stop.assert_not_called()
                backup.assert_not_called()
                upload.assert_not_called()
                remote.rsync_to.assert_not_called()

    def test_all_deployment_commands_use_same_complete_prepared_package(self):
        config = fleetctl.load_config(fleetctl.DEFAULT_CONFIG)
        profile = (
            "[uav1]\nboard_ip = 192.168.1.100\nfc_url = udpout://192.168.1.40:14280\n"
            "[uav2]\nboard_ip = 192.168.1.101\nfc_url = udpout://192.168.1.40:14281\n"
        )
        for command, count in (("test", 1), ("test-midcourse", 1), ("test-agent-info", 1), ("test-presence", 2)):
            with self.subTest(command=command), tempfile.TemporaryDirectory(prefix="fleet path ") as directory:
                temporary_root = Path(directory)
                (temporary_root / "build").mkdir()
                (temporary_root / "build/gcs_sim").touch()
                bundle = self.write_bundle(temporary_root, config.planes[0], profile=profile)
                remote = Mock()
                def network_information(plane, script, **kwargs):
                    if script == "ip -j -4 address show up":
                        output = json.dumps([{"flags": ["UP"], "addr_info": [{"family": "inet", "local": plane.host}]}])
                    else:
                        output = ""
                    return subprocess.CompletedProcess([], 0, stdout=output)
                remote.run.side_effect = network_information
                args = fleetctl.build_parser().parse_args([command, "--count", str(count)])
                endpoints = fleetctl.EndpointSet(("udpout://192.168.1.56:14280",), ())
                failure = fleetctl.FleetError("测试终止：已到达统一全量上传")
                with patch.object(fleetctl, "ROOT", temporary_root), \
                     patch.object(fleetctl, "preflight"), \
                     patch.object(fleetctl, "ensure_bundle", return_value=bundle), \
                     patch.object(fleetctl, "check_sitl_profile_listeners"), \
                     patch.object(fleetctl, "select_gcs_loopback_endpoint", return_value="tcp/127.0.0.1:35001"), \
                     patch.object(fleetctl, "backup_plane_runtime", return_value="/tmp/runtime.backup"), \
                     patch.object(fleetctl, "stop_planes") as stop, \
                     patch.object(fleetctl, "install_system_dependencies"), \
                     patch.object(fleetctl, "stop_simulator"), \
                     patch.object(fleetctl, "start_simulator", return_value=endpoints), \
                     patch.object(fleetctl, "deploy_plane_bundle", side_effect=failure) as upload:
                    function = getattr(fleetctl, "command_" + command.replace("-", "_"))
                    with self.assertRaisesRegex(fleetctl.FleetError, "已到达统一全量上传"):
                        function(args, config, remote)
                stop.assert_called_once()
                upload.assert_called_once()
                deployment = upload.call_args.args[1]
                self.assertIsInstance(deployment, fleetctl.PlaneDeployment)
                self.assertEqual((deployment.directory / fleetctl.PLANE_CONFIG).read_bytes(), (bundle / fleetctl.PLANE_CONFIG).read_bytes())
                self.assertIn("safe_height_m = 20", deployment.configuration.main)
                self.assertEqual((deployment.directory / "config/profiles/unused.ini").read_text(), "包内未选中预置")
                self.assertTrue((deployment.directory / "lib/test.so").is_file())
                self.assertEqual(len(list((temporary_root / "logs/fleet").glob("*/*.deployment"))), count)

    def test_multi_aircraft_duplicate_package_ports_fail_before_backup_or_stop(self):
        config = fleetctl.load_config(fleetctl.DEFAULT_CONFIG)
        profile = (
            "[uav1]\nboard_ip = 192.168.1.100\nfc_url = udpout://192.168.1.40:14280\n"
            "[uav2]\nboard_ip = 192.168.1.101\nfc_url = udpout://192.168.1.40:14280\n"
        )
        with tempfile.TemporaryDirectory(prefix="fleet duplicate ports ") as directory:
            root = Path(directory)
            bundle = self.write_bundle(root, config.planes[0], profile=profile)
            remote = Mock()
            def network_information(plane, script, **kwargs):
                self.assertEqual(script, "ip -j -4 address show up")
                interfaces = [{"flags": ["UP"], "addr_info": [{"family": "inet", "local": plane.host}]}]
                return subprocess.CompletedProcess([], 0, stdout=json.dumps(interfaces))
            remote.run.side_effect = network_information
            args = fleetctl.build_parser().parse_args(["test", "--count", "2"])
            with patch.object(fleetctl, "ROOT", root), \
                 patch.object(fleetctl, "preflight"), \
                 patch.object(fleetctl, "ensure_bundle", return_value=bundle), \
                 patch.object(fleetctl, "backup_plane_runtime") as backup, \
                 patch.object(fleetctl, "stop_planes") as stop, \
                 patch.object(fleetctl, "start_simulator") as simulator, \
                 patch.object(fleetctl, "deploy_plane_bundle") as upload:
                with self.assertRaisesRegex(fleetctl.FleetError, "端口重复"):
                    fleetctl.command_test(args, config, remote)
            backup.assert_not_called()
            stop.assert_not_called()
            simulator.assert_not_called()
            upload.assert_not_called()
            remote.rsync_to.assert_not_called()

    def test_test_command_only_starts_simulator_for_selected_package_sitl_connections(self):
        config = fleetctl.load_config(fleetctl.DEFAULT_CONFIG)
        for use_sitl in (False, True):
            with self.subTest(use_sitl=use_sitl), tempfile.TemporaryDirectory(prefix="fleet simulator scope ") as directory:
                root = Path(directory)
                second_url = "udpout://192.168.1.40:14281" if use_sitl else "serial:///dev/ttyS0:230400"
                profile = (
                    "[uav1]\nboard_ip = 192.168.1.100\nfc_url = serial:///dev/ttyS0:230400\n"
                    f"[uav2]\nboard_ip = 192.168.1.101\nfc_url = {second_url}\n"
                )
                bundle = self.write_bundle(root, config.planes[0], profile=profile)
                remote = Mock()
                def network_information(plane, script, **kwargs):
                    self.assertEqual(script, "ip -j -4 address show up")
                    interfaces = [{"flags": ["UP"], "addr_info": [{"family": "inet", "local": plane.host}]}]
                    return subprocess.CompletedProcess([], 0, stdout=json.dumps(interfaces))
                remote.run.side_effect = network_information
                args = fleetctl.build_parser().parse_args(["test", "--count", "2"])
                with patch.object(fleetctl, "ROOT", root), \
                     patch.object(fleetctl, "preflight"), \
                     patch.object(fleetctl, "ensure_bundle", return_value=bundle), \
                     patch.object(fleetctl, "backup_plane_runtime"), \
                     patch.object(fleetctl, "stop_planes"), \
                     patch.object(fleetctl, "install_system_dependencies"), \
                     patch.object(fleetctl, "stop_simulator") as stop_simulator, \
                     patch.object(fleetctl, "start_simulator") as start_simulator, \
                     patch.object(fleetctl, "deploy_plane_bundle", side_effect=fleetctl.FleetError("结束测试")):
                    with self.assertRaisesRegex(fleetctl.FleetError, "结束测试"):
                        fleetctl.command_test(args, config, remote)
                if use_sitl:
                    stop_simulator.assert_called_once()
                    self.assertEqual(start_simulator.call_args.args[1], [14281])
                    self.assertEqual(start_simulator.call_args.kwargs["start_count"], 2)
                else:
                    stop_simulator.assert_not_called()
                    start_simulator.assert_not_called()

    def test_multi_plane_rendering_preserves_each_selected_profile_and_ports(self):
        config = fleetctl.load_config(fleetctl.DEFAULT_CONFIG)
        profile = (
            "[groundstation]\nip = 192.168.1.41\n"
            "[uav1]\nboard_ip = 192.168.1.100\nfc_url = udpout://192.168.1.40:14280\n"
            "[uav2]\nboard_ip = 192.168.1.101\nfc_url = udpout://192.168.1.40:14581\n"
        )
        for plane, selected, untouched in (
            (config.planes[0], "udpout://192.168.1.40:14280", "udpout://192.168.1.40:14581"),
            (config.planes[1], "udpout://192.168.1.40:14581", "udpout://192.168.1.40:14280"),
        ):
            with self.subTest(plane=plane.plane_id):
                base = self.configuration(plane, "[system]\nsafe_height_m = 23\n", profile=profile)
                result = fleetctl.configure_plane(base, plane, config.simulator.host)
                self.assertEqual(result.profile_text, profile.replace(selected, selected.replace("192.168.1.40", "192.168.1.56")))
                self.assertIn(untouched, result.profile_text)
                self.assertIn("safe_height_m = 23", result.main)
                self.assertIn("profile = config/profiles/site.ini", result.main)
                self.assertEqual(result.get("communication", "plane_id"), plane.plane_id)

    def test_fresh_deployment_reads_bundle_using_actual_network_interfaces(self):
        plane = fleetctl.load_config(fleetctl.DEFAULT_CONFIG).planes[0]
        remote = self.local_remote({plane.host})
        configuration = fleetctl.read_bundle_config(plane, remote, ROOT)
        self.assertEqual(configuration.get("communication", "plane_id"), "uav1")
        self.assertEqual(configuration.get("system", "board_ip"), plane.host)
        self.assertTrue(configuration.profile_path)
        self.assertIn(f"profile = {configuration.profile_path}", configuration.main)
        remote.run.assert_called_once_with(plane, "ip -j -4 address show up")

    def test_sitl_listener_check_uses_profile_ports_and_skips_serial(self):
        config = fleetctl.load_config(fleetctl.DEFAULT_CONFIG)
        configurations = (
            self.configuration(config.planes[0], fc_url="udpout://192.168.1.56:14280"),
            self.configuration(config.planes[1], fc_url="udpout://192.168.1.56:14581"),
        )
        remote = Mock()
        remote.run.return_value.stdout = "UNCONN 0 0 0.0.0.0:14280 0.0.0.0:*\nUNCONN 0 0 0.0.0.0:14581 0.0.0.0:*\n"
        fleetctl.check_sitl_profile_listeners(config, configurations, remote)
        remote.run.assert_called_once_with(config.simulator, "ss -uln")
        remote.run.return_value.stdout = "UNCONN 0 0 0.0.0.0:14580 0.0.0.0:*\n"
        with self.assertRaisesRegex(fleetctl.FleetError, "未监听预置飞控 UDP 端口"):
            fleetctl.check_sitl_profile_listeners(config, configurations, remote)
        remote.reset_mock()
        serial = self.configuration(config.planes[0], fc_url="serial:///dev/test:230400")
        fleetctl.check_sitl_profile_listeners(config, (serial,), remote)
        remote.run.assert_not_called()
        duplicate = self.configuration(config.planes[1], fc_url="udpout://192.168.1.56:14280")
        with self.assertRaisesRegex(fleetctl.FleetError, "端口重复"):
            fleetctl.check_sitl_profile_listeners(config, (configurations[0], duplicate), remote)
        remote.run.assert_not_called()

    def test_auto_discovery_configuration_bypass_is_removed(self):
        with self.assertRaises(SystemExit):
            fleetctl.build_parser().parse_args(["test-presence", "--count", "2", "--auto-discovery"])

    def test_agent_info_rejects_multiple_boards_before_remote_actions(self):
        config = fleetctl.load_config(fleetctl.DEFAULT_CONFIG)
        remote = Mock()
        with self.assertRaises(fleetctl.FleetError):
            fleetctl.command_test_agent_info(argparse.Namespace(count=2), config, remote)
        remote.run.assert_not_called()
        remote.rsync_to.assert_not_called()

    def test_agent_info_command_and_log_payloads(self):
        args = fleetctl.build_parser().parse_args(["test-agent-info", "--count", "1"])
        self.assertEqual(args.command, "test-agent-info")
        text = (
            '[info] 收到 AgentInfo: {"agentId":"uav1","agentType":"Vehicle"}\n'
            '[info] 收到 AgentInfo: {"agentId":"station-a","agentType":"GroundStation"}\n'
            '[info] AgentInfo 查询成功: {"agentId":"station-b","agentType":"GroundStation"}\n'
        )
        self.assertEqual(fleetctl.agent_info_log_payloads(text, "收到 AgentInfo"), [
            {"agentId": "uav1", "agentType": "Vehicle"},
            {"agentId": "station-a", "agentType": "GroundStation"},
        ])
        self.assertEqual(fleetctl.agent_info_log_payloads(text, "AgentInfo 查询成功"), [
            {"agentId": "station-b", "agentType": "GroundStation"},
        ])

    def test_agent_info_station_connections_use_explicit_ready_station_listener(self):
        plane_endpoint = "tcp/192.168.1.100:7447"
        station_endpoint = "tcp/127.0.0.1:35001"
        self.assertEqual(
            fleetctl.agent_info_transport_options("startup", plane_endpoint, station_endpoint),
            ["--listen", station_endpoint],
        )
        self.assertEqual(
            fleetctl.agent_info_transport_options("station-online", plane_endpoint, station_endpoint),
            ["--listen", station_endpoint, "--connect", plane_endpoint],
        )
        self.assertEqual(
            fleetctl.agent_info_transport_options("second-station", plane_endpoint, station_endpoint, second_station=True),
            ["--connect", station_endpoint + "," + plane_endpoint],
        )
        self.assertEqual(
            fleetctl.agent_info_transport_options("station-reonline", plane_endpoint, station_endpoint),
            ["--listen", station_endpoint, "--connect", plane_endpoint],
        )

    def test_agent_info_loopback_endpoint_is_chosen_on_selected_board(self):
        plane = fleetctl.load_config(fleetctl.DEFAULT_CONFIG).planes[0]
        remote = Mock()
        remote.run.return_value.stdout = "35001\n"
        self.assertEqual(fleetctl.select_gcs_loopback_endpoint(plane, remote), "tcp/127.0.0.1:35001")
        self.assertEqual(remote.run.call_args.args[0], plane)
        self.assertIn("python3 -c", remote.run.call_args.args[1])
        for output in ("", "invalid", "0", "65536"):
            with self.subTest(output=output):
                remote.run.return_value.stdout = output
                with self.assertRaises(fleetctl.FleetError):
                    fleetctl.select_gcs_loopback_endpoint(plane, remote)

    def test_midcourse_rejects_multiple_boards_before_remote_actions(self):
        config = fleetctl.load_config(fleetctl.DEFAULT_CONFIG)
        remote = Mock()
        with self.assertRaises(fleetctl.FleetError):
            fleetctl.command_test_midcourse(argparse.Namespace(count=2), config, remote)
        remote.run.assert_not_called()
        remote.rsync_to.assert_not_called()

    def test_midcourse_command_is_registered(self):
        args = fleetctl.build_parser().parse_args(["test-midcourse", "--count", "1"])
        self.assertEqual(args.command, "test-midcourse")
        self.assertEqual(args.count, 1)
        self.assertEqual(args.target_track_id, "")

    def test_midcourse_verification_uses_aggregate_algorithm_logs(self):
        source = (ROOT / "tools" / "fleetctl.py").read_text(encoding="utf-8")
        self.assertIn("中制导算法 MidcourseAlgorithm::init 输入", source)
        self.assertIn("中制导算法 MidcourseAlgorithm::update 输入", source)
        self.assertIn("中制导算法首次增量量测更新", source)
        self.assertIn("init → update → updateTarget", source)
        self.assertNotIn("中制导目标估计器首次增量更新", source)
        self.assertNotIn("setConfig → init → planner.update → estimator.update", source)

    def test_radar_history_uses_reported_size_for_initial_and_summary_logs(self):
        for prefix in ("GCS 目标更新", "GCS 目标更新汇总"):
            for history_size in (1, 2, 19, 20, 21):
                with self.subTest(prefix=prefix, history_size=history_size):
                    text = (
                        "[2026-11-20 12:00:00.000] [info] [1490] target_store.cpp:97 "
                        f"{prefix}: trackId=mavlink_rc:1 timestamp_ms=1795076655339 "
                        f"history_size={history_size} 更新数量=20\n"
                    )
                    self.assertEqual(fleetctl.radar_history_ready(text), history_size >= 20)
                    self.assertEqual(fleetctl.radar_history_ready(text, "mavlink_rc:1"), history_size >= 20)

    def test_radar_history_requires_exact_requested_track(self):
        text = (
            "GCS 目标更新汇总: trackId=target.1 timestamp_ms=1000 history_size=19 更新数量=20\n"
            "GCS 目标更新汇总: trackId=targetX1 timestamp_ms=2000 history_size=20 更新数量=20\n"
            "GCS 目标更新: trackId=target.10 timestamp_ms=3000 history_size=20\n"
        )
        self.assertTrue(fleetctl.radar_history_ready(text))
        self.assertTrue(fleetctl.radar_history_ready(text, "targetX1"))
        self.assertFalse(fleetctl.radar_history_ready(text, "target.1"))
        self.assertFalse(fleetctl.radar_history_ready(text, "target"))
        self.assertFalse(fleetctl.radar_history_ready(text, "missing"))

    def test_radar_history_does_not_accumulate_log_lines_or_tracks(self):
        text = "".join(
            f"GCS 目标更新: trackId=target-{index % 2} timestamp_ms={1000 + index} history_size=10\n"
            for index in range(40)
        )
        self.assertFalse(fleetctl.radar_history_ready(text))
        self.assertFalse(fleetctl.radar_history_ready(text, "target-0"))

    def test_radar_history_ignores_unrelated_and_incomplete_logs(self):
        for text in (
            "",
            "中制导初始化完成: trackId=target timestamp_ms=1000 history_size=20\n",
            "等待所需目标更新: trackId=target 上次接收时间=从未收到 timestamp_ms=<无>\n",
            "GCS 目标更新: trackId=target timestamp_ms=1000\n其他日志 history_size=20\n",
            "GCS 目标更新汇总: trackId=target timestamp_ms=1000 history_size=20x\n",
            "GCS 目标更新汇总: trackId=target timestamp_ms=1000 history_size=-20\n",
        ):
            with self.subTest(text=text):
                self.assertFalse(fleetctl.radar_history_ready(text))
                self.assertFalse(fleetctl.radar_history_ready(text, "target"))

    def test_presence_preflight_only_contacts_selected_boards(self):
        config = fleetctl.load_config(fleetctl.DEFAULT_CONFIG)
        planes = fleetctl.selected_planes(config, 2)
        remote = Mock()
        remote.run.return_value.stdout = "ok"
        with patch.object(fleetctl, "require_local_commands"):
            fleetctl.preflight(config, planes, remote, include_simulator=False)
        self.assertEqual([call.args[0] for call in remote.run.call_args_list], list(planes))

    def test_presence_counts_match_exact_plane_id(self):
        text = "他机上线: uav2\n他机上线: uav20\n他机下线: uav2\n他机上线: uav2\n"
        self.assertEqual(fleetctl.presence_event_counts(text, "uav2"), (2, 1))


if __name__ == "__main__":
    unittest.main()
