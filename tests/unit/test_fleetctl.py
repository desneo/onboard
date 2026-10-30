#!/usr/bin/env python3

import argparse
import importlib.util
from pathlib import Path
import sys
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
            "udpout://192.168.1.40:14280 udpout://192.168.1.40:14281",
            "192.168.1.40",
            2,
            ports,
        )
        self.assertEqual(
            result.flight_urls,
            ("udpout://192.168.1.40:14580", "udpout://192.168.1.40:14581"),
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
        config = fleetctl.load_config(ROOT / "config" / "fleet.ini")
        with self.assertRaises(fleetctl.FleetError):
            fleetctl.selected_planes(config, len(config.planes) + 1)

    def test_configure_plane_removes_retired_image_sections(self):
        config = fleetctl.load_config(ROOT / "config" / "fleet.ini")
        source = (
            "[system]\nlog_dir = logs/old-run\n"
            "[image_input]\ntype = Rtsp\n[detection]\nmodel_path = old-model\n"
            "[plugins]\ndirectory = plugins\n[video_output]\ntype = Rtsp\n"
            "[debug]\ntarget_log = true\n[suicide_mission]\nphase_timeout_ms = 1000\n"
            "[safety]\nalert_voltage_threshold_v = 21\n"
        )
        result = fleetctl.configure_plane(source, config.planes[0], "udpout://127.0.0.1:14580")
        for section in ("image_input", "detection", "plugins", "video_output", "debug", "suicide_mission"):
            self.assertNotIn(f"[{section}]", result)
        self.assertIn("[safety]\nalert_voltage_threshold_v = 21", result)
        self.assertIn("log_dir = logs\n", result)
        self.assertIn("udp_url = udpout://127.0.0.1:14580", result)

    def test_fleet_environment_and_dependency_order(self):
        config = fleetctl.load_config(ROOT / "config" / "fleet.ini")
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
        planes = fleetctl.load_config(ROOT / "config" / "fleet.ini").planes[:2]
        online = "[2026-10-27 18:00:01.000] [info] 他机上线: uav2\n"
        offline = "[2026-10-27 18:00:02.000] [info] 他机下线: uav2\n"
        result = fleetctl.analyze_plane(planes[0], planes, online + offline + online)
        self.assertFalse(result.peer_online["uav2"])
        recovered = "[2026-10-27 18:00:03.000] [info] 他机上线: uav2\n"
        result = fleetctl.analyze_plane(planes[0], planes, recovered + online + offline)
        self.assertTrue(result.peer_online["uav2"])

    def test_old_run_cannot_satisfy_current_presence_check(self):
        planes = fleetctl.load_config(ROOT / "config" / "fleet.ini").planes[:2]
        text = (
            "[2026-10-27 17:00:00.000] [info] 他机上线: uav2\n"
            "[2026-10-27 17:00:01.000] [info] 飞控连接建立\n"
            "[2026-10-27 18:00:00.000] [info] plane 程序启动: version=0.1.0\n"
        )
        result = fleetctl.analyze_plane(planes[0], planes, text)
        self.assertFalse(result.peer_online["uav2"])
        self.assertFalse(result.flight_connected)

    def test_spdlog_error_level_is_detected(self):
        planes = fleetctl.load_config(ROOT / "config" / "fleet.ini").planes[:1]
        text = (
            "[2026-10-30 18:00:00.000] [info] plane 程序启动: version=0.1.0\n"
            "[2026-10-30 18:00:01.000] [error] 飞控连接失败\n"
        )
        result = fleetctl.analyze_plane(planes[0], planes, text)
        self.assertEqual(result.errors, ["[2026-10-30 18:00:01.000] [error] 飞控连接失败"])

    def test_presence_deployment_preserves_board_connections(self):
        plane = fleetctl.load_config(ROOT / "config" / "fleet.ini").planes[1]
        base = (
            "[communication]\nplane_id = old\nzenoh_connect = tcp/127.0.0.1:7447\n"
            "[flight_control]\nlink_type = serial\nserial_device = /dev/test\n"
            "[image_input]\ntype = Rtsp\n"
        )
        template = (ROOT / "config" / "plane.conf").read_text()
        result = fleetctl.render_presence_config(base, template, plane)
        self.assertIn("plane_id = uav2", result)
        self.assertIn("zenoh_connect = tcp/127.0.0.1:7447", result)
        self.assertIn("link_type = serial", result)
        self.assertIn("serial_device = /dev/test", result)
        self.assertNotIn("[image_input]", result)

    def test_metadata_config_preserves_custom_addresses(self):
        plane = fleetctl.load_config(ROOT / "config" / "fleet.ini").planes[0]
        base = "[system]\nboard_ip = 10.0.0.7\n[camera]\npod_video_url = rtsp://pod/live?a=1&b=2\n"
        template = (ROOT / "config" / "plane.conf").read_text()
        result = fleetctl.render_presence_config(base, template, plane)
        self.assertIn("board_ip = 10.0.0.7", result)
        self.assertIn("pod_video_url = rtsp://pod/live?a=1&b=2", result)
        default_result = fleetctl.render_presence_config("[system]\nboard_ip =\n", template, plane)
        self.assertIn("board_ip = " + plane.host, default_result)
        self.assertIn("pod_video_url =\n", default_result)

    def test_vehicle_info_rejects_multiple_boards_before_remote_actions(self):
        config = fleetctl.load_config(ROOT / "config" / "fleet.ini")
        remote = Mock()
        with self.assertRaises(fleetctl.FleetError):
            fleetctl.command_test_vehicle_info(argparse.Namespace(count=2), config, remote)
        remote.run.assert_not_called()
        remote.rsync_to.assert_not_called()

    def test_midcourse_rejects_multiple_boards_before_remote_actions(self):
        config = fleetctl.load_config(ROOT / "config" / "fleet.ini")
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
        config = fleetctl.load_config(ROOT / "config" / "fleet.ini")
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
