import math
import datetime
import os
import sys
import struct
import time
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PKG_SRC = os.path.join(
    ROOT, "src", "vehicle_interface", "src"
)
sys.path.insert(0, PKG_SRC)

from vehicle_interface.contracts import (  # noqa: E402
    clamp,
    oxyz_heading_to_ros_yaw,
    quaternion_from_rpy,
    resolve_gnss_measurement_timestamp,
    rmc_measurement_epoch,
    state_is_healthy,
    time_alignment_is_valid,
)
from vehicle_interface import can_bridge  # noqa: E402


class ContractTests(unittest.TestCase):
    def test_rmc_measurement_timestamp_is_utc(self):
        actual = rmc_measurement_epoch("123456.250", "200826")
        expected = datetime.datetime(
            2026, 8, 20, 12, 34, 56, 250000,
            tzinfo=datetime.timezone.utc).timestamp()
        self.assertAlmostEqual(actual, expected, places=6)
        self.assertIsNone(rmc_measurement_epoch("", ""))

    def test_gnss_timestamp_prefers_consistent_monotonic_rmc(self):
        result = resolve_gnss_measurement_timestamp(
            100.0, 100.05, fixed_latency_s=0.05,
            previous_epoch=99.9, max_clock_residual_s=0.10,
        )
        self.assertEqual(result["source"], "rmc_utc")
        self.assertAlmostEqual(result["measurement_epoch"], 100.0)
        self.assertTrue(result["clock_consistent"])
        self.assertAlmostEqual(result["clock_offset_s"], -0.05)
        self.assertAlmostEqual(result["clock_residual_s"], 0.0)

    def test_gnss_timestamp_clock_mismatch_falls_back_and_fails_gate(self):
        result = resolve_gnss_measurement_timestamp(
            98.0, 100.0, fixed_latency_s=0.05,
            previous_epoch=97.0, max_clock_residual_s=0.30,
        )
        self.assertEqual(
            result["source"], "ipc_receive_latency_clock_mismatch"
        )
        self.assertAlmostEqual(result["measurement_epoch"], 99.95)
        self.assertFalse(result["clock_consistent"])

    def test_gnss_timestamp_missing_or_repeated_rmc_uses_receive_clock(self):
        missing = resolve_gnss_measurement_timestamp(None, 100.0)
        self.assertEqual(
            missing["source"], "ipc_receive_latency_missing_rmc"
        )
        self.assertTrue(missing["clock_consistent"])
        self.assertFalse(missing["rmc_available"])

        repeated = resolve_gnss_measurement_timestamp(
            100.0, 100.05, previous_epoch=100.0
        )
        self.assertEqual(
            repeated["source"], "ipc_receive_latency_nonmonotonic_rmc"
        )
        self.assertTrue(repeated["clock_consistent"])
        self.assertAlmostEqual(repeated["measurement_epoch"], 100.05)

    def test_time_alignment_gate(self):
        self.assertTrue(time_alignment_is_valid(0.060, 0.060))
        self.assertTrue(time_alignment_is_valid(-0.059, 0.060))
        self.assertFalse(time_alignment_is_valid(0.061, 0.060))
        self.assertFalse(time_alignment_is_valid(float("nan"), 0.060))

    def test_heading_conversion(self):
        self.assertAlmostEqual(oxyz_heading_to_ros_yaw(0.0), math.pi / 2.0)
        self.assertAlmostEqual(abs(oxyz_heading_to_ros_yaw(90.0)), math.pi)
        self.assertAlmostEqual(oxyz_heading_to_ros_yaw(-90.0), 0.0)

    def test_quaternion_identity(self):
        self.assertEqual(quaternion_from_rpy(0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    def test_health_gate(self):
        state = {
            "UTM_x": 1.0,
            "UTM_y": 2.0,
            "Head": 3.0,
            "FusionValid": 1,
            "PositionValid": 1,
            "ControlFrameReady": 1,
            "HeadingMode": "AHRS",
        }
        self.assertTrue(state_is_healthy(state, True, True, True, True))
        state["HeadingMode"] = "UNKNOWN"
        self.assertFalse(state_is_healthy(state, True, True, True, True))

    def test_malformed_state_fails_closed(self):
        state = {
            "UTM_x": "not-a-number",
            "UTM_y": 2.0,
            "Head": 3.0,
            "FusionValid": "bad-flag",
            "PositionValid": 1,
            "ControlFrameReady": 1,
        }
        self.assertFalse(state_is_healthy(state))

    def test_command_clamp(self):
        self.assertEqual(clamp(2.0, 0.8), 0.8)
        self.assertEqual(clamp(-2.0, 0.8), -0.8)

    def test_tracked_vehicle_keeps_pivot_command(self):
        cfg = can_bridge.Cfg()
        cfg.zero_steer_when_stopped = False
        now = time.monotonic()
        with can_bridge.cmd_lock:
            can_bridge.cmd_state.update({
                "v_cmd": 0.0,
                "w_cmd": 0.20,
                "brake": 0.0,
                "seen_topics": set(can_bridge.REQUIRED_POLICY_TOPICS),
                "topic_times": {
                    topic: now for topic in can_bridge.REQUIRED_POLICY_TOPICS
                },
            })
        v_cmd, w_cmd, brake, _, diagnostics = can_bridge.current_command(cfg)
        self.assertEqual(v_cmd, 0.0)
        self.assertAlmostEqual(w_cmd, 0.20)
        self.assertEqual(brake, 0.0)
        self.assertTrue(diagnostics["healthy"])

    def test_brake_request_zeros_motion_only(self):
        cfg = can_bridge.Cfg()
        now = time.monotonic()
        with can_bridge.cmd_lock:
            can_bridge.cmd_state.update({
                "v_cmd": 0.3,
                "w_cmd": 0.2,
                "brake": 1.0,
                "seen_topics": set(can_bridge.REQUIRED_POLICY_TOPICS),
                "topic_times": {
                    topic: now for topic in can_bridge.REQUIRED_POLICY_TOPICS
                },
            })
        v_cmd, w_cmd, brake, _, diagnostics = can_bridge.current_command(cfg)
        payload, speed_raw, steer_raw = can_bridge.pack_motion_payload(
            v_cmd, w_cmd, cfg
        )
        self.assertEqual((v_cmd, w_cmd, brake), (0.0, 0.0, 1.0))
        self.assertTrue(diagnostics["healthy"])
        self.assertEqual((speed_raw, steer_raw), (0, 0))
        self.assertEqual(struct.unpack("<hh4x", payload), (0, 0))

    def test_can_command_requires_all_policy_channels(self):
        cfg = can_bridge.Cfg()
        now = time.monotonic()
        with can_bridge.cmd_lock:
            can_bridge.cmd_state.update({
                "v_cmd": 0.3,
                "w_cmd": 0.2,
                "brake": 0.0,
                "seen_topics": {"PoliAcc", "PoliSteer"},
                "topic_times": {
                    "PoliAcc": now,
                    "PoliSteer": now,
                    "PoliBrake": 0.0,
                },
            })
        v_cmd, w_cmd, brake, age, diagnostics = \
            can_bridge.current_command(cfg)
        self.assertEqual((v_cmd, w_cmd, brake), (0.0, 0.0, 1.0))
        self.assertFalse(diagnostics["healthy"])
        self.assertEqual(diagnostics["missing_topics"], ["PoliBrake"])
        self.assertEqual(diagnostics["stale_topics"], [])
        self.assertTrue(math.isinf(age))

    def test_can_command_rejects_one_stale_policy_channel(self):
        cfg = can_bridge.Cfg()
        now = time.monotonic()
        with can_bridge.cmd_lock:
            can_bridge.cmd_state.update({
                "v_cmd": 0.3,
                "w_cmd": 0.2,
                "brake": 0.0,
                "seen_topics": set(can_bridge.REQUIRED_POLICY_TOPICS),
                "topic_times": {
                    "PoliAcc": now,
                    "PoliSteer": now - cfg.stale_timeout - 0.1,
                    "PoliBrake": now,
                },
            })
        v_cmd, w_cmd, brake, _, diagnostics = \
            can_bridge.current_command(cfg)
        self.assertEqual((v_cmd, w_cmd, brake), (0.0, 0.0, 1.0))
        self.assertFalse(diagnostics["healthy"])
        self.assertEqual(diagnostics["missing_topics"], [])
        self.assertEqual(diagnostics["stale_topics"], ["PoliSteer"])


if __name__ == "__main__":
    unittest.main()
