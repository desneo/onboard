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
        self.assertEqual(len(result.rtsp_urls), 2)

    def test_parse_rejects_ambiguous_flight_endpoints(self):
        with self.assertRaises(fleetctl.FleetError):
            fleetctl.parse_endpoints("udpout://192.168.1.56:14280", "192.168.1.56", 2)

    def test_ini_update_preserves_other_sections(self):
        source = "[communication]\nplane_id = old\n\n[flight_control]\nudp_url = old\n"
        updated = fleetctl.set_ini_value(source, "communication", "plane_id", "uav2")
        self.assertIn("plane_id = uav2", updated)
        self.assertIn("[flight_control]\nudp_url = old", updated)

    def test_inventory_rejects_more_planes_than_declared(self):
        config = fleetctl.load_config(ROOT / "config" / "fleet.ini")
        with self.assertRaises(fleetctl.FleetError):
            fleetctl.selected_planes(config, 3)

    def test_fleet_environment_and_dependency_order(self):
        config = fleetctl.load_config(ROOT / "config" / "fleet.ini")
        self.assertEqual(config.board_arch, "aarch64")
        self.assertEqual(config.board_os_version, "22.04")
        self.assertEqual(config.apt_bootstrap_packages, ("libopencv-dev",))
        self.assertNotIn("libzmq5", config.apt_packages)

    def test_verify_command_requires_run_id(self):
        parser = fleetctl.build_parser()
        with self.assertRaises(SystemExit):
            parser.parse_args(["verify", "--count", "2"])
        args = parser.parse_args(["verify", "--count", "2", "--run-id", "20261016-000000"])
        self.assertEqual(args.run_id, "20261016-000000")


if __name__ == "__main__":
    unittest.main()
