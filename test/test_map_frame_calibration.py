import importlib.util
import math
import os
from pathlib import Path
import tempfile
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SCRIPT = os.path.join(
    ROOT, "src", "terrain_map_builder", "scripts", "calibrate_map_frame.py")
SPEC = importlib.util.spec_from_file_location("map_frame_calibration", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MapFrameCalibrationTests(unittest.TestCase):
    @staticmethod
    def point_from_enu(origin, east, north, up=0.0):
        # Adequate inverse for a short synthetic baseline used by this test.
        latitude_scale = 111132.0
        longitude_scale = 111320.0 * math.cos(math.radians(origin[0]))
        return (origin[0] + north / latitude_scale,
                origin[1] + east / longitude_scale,
                origin[2] + up)

    def test_east_is_zero_yaw(self):
        origin = (22.3, 114.1, 8.0)
        result = MODULE.estimate_frame(
            [origin] * 20,
            [self.point_from_enu(origin, 20.0, 0.0)] * 20)
        self.assertAlmostEqual(result["yaw_enu_to_oxyz_deg"], 0.0, delta=0.01)
        self.assertAlmostEqual(result["baseline_horizontal_m"], 20.0, delta=0.05)

    def test_north_is_positive_ninety_yaw(self):
        origin = (22.3, 114.1, 8.0)
        result = MODULE.estimate_frame(
            [origin] * 20,
            [self.point_from_enu(origin, 0.0, 20.0)] * 20)
        self.assertAlmostEqual(
            result["yaw_enu_to_oxyz_deg"], 90.0, delta=0.01)

    def test_median_rejects_one_large_outlier(self):
        origin = (22.3, 114.1, 8.0)
        samples = [origin] * 19 + [(23.0, 115.0, 1000.0)]
        self.assertEqual(MODULE.median_geodetic(samples), origin)

    def test_write_updates_only_named_map_module(self):
        result = {
            "origin_lat": 22.31,
            "origin_lon": 114.12,
            "origin_alt": 8.5,
            "yaw_enu_to_oxyz_deg": 17.0,
        }
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name in ("site_a", "site_b"):
                directory = root / "maps" / name
                directory.mkdir(parents=True)
                (directory / "module.yaml").write_text(
                    "map_module: {}\norigin_lat: \"\"\norigin_lon: \"\"\n"
                    "origin_alt: 0.0\nyaw_enu_to_oxyz_deg: 0.0\n".format(name),
                    encoding="utf-8")
            site_a_before = (root / "maps" / "site_a" / "module.yaml").read_text(
                encoding="utf-8")
            written = MODULE.write_module("site_b", result, root)
            self.assertEqual(written, root / "maps" / "site_b" / "module.yaml")
            self.assertEqual(
                (root / "maps" / "site_a" / "module.yaml").read_text(
                    encoding="utf-8"),
                site_a_before)
            site_b = (root / "maps" / "site_b" / "module.yaml").read_text(
                encoding="utf-8")
            self.assertIn("origin_lat: 22.31", site_b)
            self.assertIn("yaw_enu_to_oxyz_deg: 17", site_b)


if __name__ == "__main__":
    unittest.main()
