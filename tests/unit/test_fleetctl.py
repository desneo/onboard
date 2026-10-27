#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("fleetctl", ROOT / "tools" / "fleetctl.py")
assert SPEC and SPEC.loader
fleetctl = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = fleetctl
SPEC.loader.exec_module(fleetctl)


class FleetCtlTest(unittest.TestCase):
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
            "[debug]\ntarget_log = true\n[safety]\nalert_voltage_threshold_v = 21\n"
        )
        result = fleetctl.configure_plane(source, config.planes[0], "udpout://127.0.0.1:14580")
        for section in ("image_input", "detection", "plugins", "video_output", "debug"):
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


if __name__ == "__main__":
    unittest.main()
