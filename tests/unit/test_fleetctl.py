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
        result = fleetctl.parse_endpoints(output, "192.168.1.56", 2)
        self.assertEqual(
            result.flight_urls,
            ("udpout://192.168.1.56:14280", "udpout://192.168.1.56:14281"),
        )

    def test_parse_rejects_ambiguous_flight_endpoints(self):
        with self.assertRaises(fleetctl.FleetError):
            fleetctl.parse_endpoints("udpout://192.168.1.56:14280", "192.168.1.56", 2)

    def test_configured_flight_ports_override_sitl_output(self):
        listeners = """
        UNCONN 0 0 0.0.0.0:14580 0.0.0.0:*
        UNCONN 0 0 0.0.0.0:14581 0.0.0.0:*
        """
        ports = fleetctl.validate_flight_listeners(14580, 2, listeners)
        result = fleetctl.parse_endpoints(
            "udpout://192.168.1.56:14280 udpout://192.168.1.56:14281",
            "192.168.1.56",
            2,
            ports,
        )
        self.assertEqual(
            result.flight_urls,
            ("udpout://192.168.1.56:14580", "udpout://192.168.1.56:14581"),
        )

    def test_configured_flight_ports_must_be_listening(self):
        with self.assertRaises(fleetctl.FleetError):
            fleetctl.validate_flight_listeners(14580, 2, "0.0.0.0:14580 ")

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

    def test_configure_plane_removes_retired_settings(self):
        config = fleetctl.load_config(fleetctl.DEFAULT_CONFIG)
        source = (
            "[system]\nlog_dir = logs/old-run\n"
            "[image_input]\ntype = Rtsp\n[detection]\nmodel_path = old-model\n"
            "[plugins]\ndirectory = plugins\n[video_output]\ntype = Rtsp\n"
            "[debug]\ntarget_log = true\n[suicide_mission]\nphase_timeout_ms = 1000\n"
            "[safety]\nalert_voltage_threshold_v = 21\n"
            "[communication]\nstation_id = old-station\n"
        )
        result = fleetctl.configure_plane(source, config.planes[0], "udpout://127.0.0.1:14580")
        for section in ("image_input", "detection", "plugins", "video_output", "debug", "suicide_mission"):
            self.assertNotIn(f"[{section}]", result)
        self.assertIn("[safety]\nalert_voltage_threshold_v = 21", result)
        self.assertIn("log_dir = logs\n", result)
        self.assertIn("udp_url = udpout://127.0.0.1:14580", result)
        self.assertNotIn("station_id", result)

    def test_fleet_environment_and_dependency_order(self):
        config = fleetctl.load_config(fleetctl.DEFAULT_CONFIG)
        self.assertEqual(config.board_arch, "aarch64")
        self.assertEqual(config.board_os_version, "22.04")
        self.assertEqual(config.simulator.flight_port_base, 14580)
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

    def test_presence_deployment_preserves_board_connections(self):
        plane = fleetctl.load_config(fleetctl.DEFAULT_CONFIG).planes[1]
        base = (
            "[communication]\nplane_id = old\nzenoh_connect = tcp/127.0.0.1:7447\n"
            "[flight_control]\nlink_type = serial\nserial_device = /dev/test\n"
            "[image_input]\ntype = Rtsp\n"
        )
        template = (ROOT / fleetctl.PLANE_CONFIG).read_text()
        result = fleetctl.render_presence_config(base, template, plane, "192.168.1.56")
        self.assertIn("plane_id = uav2", result)
        self.assertIn("zenoh_connect = tcp/127.0.0.1:7447", result)
        self.assertIn("link_type = serial", result)
        self.assertIn("serial_device = /dev/test", result)
        self.assertNotIn("[image_input]", result)

    def test_presence_sitl_address_uses_inventory_host_and_preserves_port(self):
        config = fleetctl.load_config(fleetctl.DEFAULT_CONFIG)
        template = (ROOT / fleetctl.PLANE_CONFIG).read_text()
        for port in (14281, 14581):
            with self.subTest(port=port):
                base = f"[flight_control]\nlink_type = udp\nudp_url = udpout://192.168.1.40:{port}\n"
                result = fleetctl.render_presence_config(base, template, config.planes[1], config.simulator.host)
                self.assertIn(f"udp_url = udpout://192.168.1.56:{port}", result)
                self.assertNotIn("192.168.1.40", result)
        listener = "[flight_control]\nlink_type = udp\nudp_url = udpin://0.0.0.0:14281\n"
        result = fleetctl.render_presence_config(listener, template, config.planes[1], config.simulator.host)
        self.assertIn("udp_url = udpin://0.0.0.0:14281", result)

    def test_test_config_migration_prefers_ini_and_flattens_selected_profile(self):
        plane = fleetctl.load_config(fleetctl.DEFAULT_CONFIG).planes[0]
        board_ip = plane.host
        other_board_ip = "192.168.1.201"
        active_addresses = [{"family": "inet", "local": board_ip}]
        interfaces = [
            {"flags": ["UP"], "addr_info": active_addresses},
            {"flags": ["UP", "LOOPBACK"], "addr_info": [{"family": "inet", "local": other_board_ip}]},
        ]
        remote = Mock()

        def run_local(_plane, script, *, check=True):
            if script == "ip -j -4 address show up":
                return subprocess.CompletedProcess([], 0, stdout=json.dumps(interfaces))
            return subprocess.run(["bash", "-c", script], text=True, capture_output=True, check=check)

        remote.run.side_effect = run_local
        with tempfile.TemporaryDirectory(prefix="fleet config ") as directory:
            runtime = Path(directory)
            plane = dataclasses.replace(plane, host="uav1.local", runtime_dir=directory)
            config_dir = runtime / "config"
            config_dir.mkdir()
            legacy = (
                "[system]\nsafe_height_m = 25\n"
                "[communication]\nzenoh_listen = tcp/0.0.0.0:7448\nzenoh_connect = tcp/10.0.0.7:7448\n"
            )
            (runtime / fleetctl.LEGACY_PLANE_CONFIG).write_text(legacy, encoding="utf-8")
            self.assertEqual(fleetctl.read_plane_config(plane, remote, ROOT), (legacy, legacy))
            direct = fleetctl.configure_plane(legacy, plane, "udpout://sitl:14580")
            self.assertIn("zenoh_listen = tcp/0.0.0.0:7448", direct)
            self.assertIn("zenoh_connect = tcp/10.0.0.7:7448", direct)

            profile_name = "config/site's preset.ini"
            original = (
                f"[deployment]\nprofile = {profile_name}\n"
                "[system]\nsafe_height_m = 30\n"
                "[communication]\nzenoh_listen = tcp/0.0.0.0:7448\nzenoh_connect = tcp/10.0.0.7:7448\n"
                "[camera]\npod_video_url = rtsp://ignored"
            )
            (runtime / fleetctl.PLANE_CONFIG).write_text(original, encoding="utf-8")
            (runtime / profile_name).write_text(
                f"[aircraft-1]\nboard_ip = {board_ip}\nfc_url = serial:///dev/ttyS0:115200\n"
                "pod_model = jielian_a2416a\npod_video_url = rtsp://camera/live?a=1&b=2\n"
                "pod_control_url = udp://camera:8001\n"
                f"[aircraft-2]\nboard_ip = {other_board_ip}\nfc_url = udpout://sitl:14281\n"
                "[groundstation]\nip = 192.168.1.41\n",
                encoding="utf-8",
            )
            actual, effective = fleetctl.read_plane_config(plane, remote, ROOT)
            self.assertEqual(actual, original)
            zenoh_endpoints = (
                f"zenoh_listen = tcp/{board_ip}:7447",
                f"zenoh_connect = tcp/192.168.1.41:7447,tcp/{other_board_ip}:7447",
            )
            for setting in (
                "safe_height_m = 30", "profile = \n", f"board_ip = {board_ip}", "plane_id = aircraft-1",
                "link_type = serial", "serial_device = /dev/ttyS0", "serial_baud = 115200",
                "pod_model = jielian_a2416a", "pod_video_url = rtsp://camera/live?a=1&b=2",
                "pod_control_url = udp://camera:8001", *zenoh_endpoints,
            ):
                self.assertIn(setting, effective)
            template = (ROOT / fleetctl.PLANE_CONFIG).read_text(encoding="utf-8")
            presence = fleetctl.render_presence_config(effective, template, plane)
            self.assertIn(f"plane_id = {plane.plane_id}", presence)
            self.assertIn(f"board_ip = {board_ip}", presence)
            self.assertIn("link_type = serial", presence)
            self.assertIn("pod_control_url = udp://camera:8001", presence)
            self.assertIn("profile = \n", presence)
            rendered = fleetctl.configure_plane(effective, plane, "udpout://sitl:14580")
            self.assertIn("udp_url = udpout://sitl:14580", rendered)
            self.assertIn("link_type = udp", rendered)
            self.assertIn("profile = \n", rendered)
            for endpoint in zenoh_endpoints:
                self.assertIn(endpoint, presence)
                self.assertIn(endpoint, rendered)

            active_addresses.append({"family": "inet", "local": other_board_ip})
            with self.assertRaisesRegex(fleetctl.FleetError, "多个飞机 ID"):
                fleetctl.read_plane_config(plane, remote, ROOT)
            active_addresses.clear()
            with self.assertRaisesRegex(fleetctl.FleetError, "未匹配"):
                fleetctl.read_plane_config(plane, remote, ROOT)

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
                    "profile.ini", groundstation + aircraft, {local_ip}, allow_unmatched=False
                )
                self.assertIn(("communication", "zenoh_listen", f"tcp/{local_ip}:7447"), settings)
                self.assertIn(("communication", "zenoh_connect", expected), settings)

        only_one = fleetctl.deployment_settings(
            "profile.ini", single_aircraft, {local_ip}, allow_unmatched=False
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
                    fleetctl.deployment_settings("profile.ini", text, {local_ip}, allow_unmatched=False)

    def test_metadata_config_preserves_custom_addresses(self):
        plane = fleetctl.load_config(fleetctl.DEFAULT_CONFIG).planes[0]
        base = "[system]\nboard_ip = 10.0.0.7\n[camera]\npod_video_url = rtsp://pod/live?a=1&b=2\n"
        template = (ROOT / fleetctl.PLANE_CONFIG).read_text()
        result = fleetctl.render_presence_config(base, template, plane)
        self.assertIn("board_ip = 10.0.0.7", result)
        self.assertIn("pod_video_url = rtsp://pod/live?a=1&b=2", result)
        default_result = fleetctl.render_presence_config("[system]\nboard_ip =\n", template, plane)
        self.assertIn("board_ip = " + plane.host, default_result)
        self.assertIn("pod_video_url =\n", default_result)

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
