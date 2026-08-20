import importlib.util
import math
import os
import sys
import types
import unittest

import numpy as np


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SCRIPT = os.path.join(
    ROOT, "src", "terrain_map_builder", "scripts", "align_qianxun_lio.py")

# The geometric core is ROS-independent; provide only the import surface that
# the command-line bag reader needs so this contract test also runs on Windows.
sys.modules.setdefault("rosbag", types.SimpleNamespace(
    Bag=None, bag=types.SimpleNamespace(ROSBagException=Exception)))
spec = importlib.util.spec_from_file_location("align_qianxun_lio", SCRIPT)
alignment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(alignment)


class AlignmentMathTests(unittest.TestCase):
    def test_robust_se2_fit_rejects_outliers(self):
        source = np.array([
            [0.0, 0.0], [2.0, 0.0], [4.0, 1.0], [5.0, 3.0],
            [4.0, 5.0], [2.0, 6.0], [0.0, 5.0], [-1.0, 3.0],
            [1.0, 2.0], [3.0, 4.0], [6.0, 2.0], [2.0, -2.0],
        ])
        yaw = math.radians(28.0)
        rotation = np.array([
            [math.cos(yaw), -math.sin(yaw)],
            [math.sin(yaw), math.cos(yaw)],
        ])
        translation = np.array([13.5, -7.25])
        target = (rotation @ source.T).T + translation
        target[-2:] += np.array([[8.0, -6.0], [-7.0, 9.0]])

        fitted_r, fitted_t, residual, _, mask = alignment.robust_fit(
            source, target, None, 1.0)
        fitted_yaw = math.atan2(fitted_r[1, 0], fitted_r[0, 0])
        self.assertAlmostEqual(fitted_yaw, yaw, places=6)
        np.testing.assert_allclose(fitted_t, translation, atol=1e-6)
        self.assertEqual(int(mask.sum()), 10)
        self.assertLess(float(np.max(residual)), 1e-6)


if __name__ == "__main__":
    unittest.main()
