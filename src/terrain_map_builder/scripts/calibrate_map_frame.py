#!/usr/bin/env python3
"""Calibrate a map's fixed geodetic origin and ENU-to-O-XYZ yaw.

The operator samples point A while stationary, then moves the GNSS antenna
along the intended O-XYZ +X axis and samples point B.  A is used as the fixed
geodetic origin and the A->B ENU bearing defines the O-XYZ axis rotation.
"""

import argparse
import json
import math
from pathlib import Path
import statistics
import sys
import threading
import time


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3
FUSION_LOCATION_TOPIC = "/fusion_location"


def wrap_degrees(angle):
    return (float(angle) + 180.0) % 360.0 - 180.0


def geodetic_to_ecef(lat_deg, lon_deg, altitude):
    lat = math.radians(float(lat_deg))
    lon = math.radians(float(lon_deg))
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return (
        (radius + altitude) * cos_lat * math.cos(lon),
        (radius + altitude) * cos_lat * math.sin(lon),
        (radius * (1.0 - WGS84_E2) + altitude) * sin_lat,
    )


def geodetic_delta_enu(origin, point):
    origin_ecef = geodetic_to_ecef(*origin)
    point_ecef = geodetic_to_ecef(*point)
    dx = point_ecef[0] - origin_ecef[0]
    dy = point_ecef[1] - origin_ecef[1]
    dz = point_ecef[2] - origin_ecef[2]
    lat = math.radians(origin[0])
    lon = math.radians(origin[1])
    east = -math.sin(lon) * dx + math.cos(lon) * dy
    north = (-math.sin(lat) * math.cos(lon) * dx
             - math.sin(lat) * math.sin(lon) * dy
             + math.cos(lat) * dz)
    up = (math.cos(lat) * math.cos(lon) * dx
          + math.cos(lat) * math.sin(lon) * dy
          + math.sin(lat) * dz)
    return east, north, up


def median_geodetic(samples):
    return tuple(statistics.median(sample[index] for sample in samples)
                 for index in range(3))


def estimate_frame(point_a_samples, point_b_samples):
    origin = median_geodetic(point_a_samples)
    point_b = median_geodetic(point_b_samples)
    east, north, up = geodetic_delta_enu(origin, point_b)
    baseline = math.hypot(east, north)
    # qianxun_position.py uses [x,y] = R(-psi)[east,north].  Therefore the
    # desired +X direction has psi=atan2(north,east).
    yaw = wrap_degrees(math.degrees(math.atan2(north, east)))
    return {
        "origin_lat": origin[0],
        "origin_lon": origin[1],
        "origin_alt": origin[2],
        "yaw_enu_to_oxyz_deg": yaw,
        "point_b_lat": point_b[0],
        "point_b_lon": point_b[1],
        "point_b_alt": point_b[2],
        "baseline_east_m": east,
        "baseline_north_m": north,
        "baseline_up_m": up,
        "baseline_horizontal_m": baseline,
    }


def horizontal_scatter(samples, center):
    distances = []
    for sample in samples:
        east, north, _ = geodetic_delta_enu(center, sample)
        distances.append(math.hypot(east, north))
    rms = math.sqrt(sum(value * value for value in distances) / len(distances))
    return rms, max(distances)


class StateSampler:
    def __init__(self, topic, sample_count, timeout, required_quality,
                 max_speed, max_hdop):
        import rospy
        from std_msgs.msg import String

        self.rospy = rospy
        self.sample_count = sample_count
        self.timeout = timeout
        self.required_quality = required_quality
        self.max_speed = max_speed
        self.max_hdop = max_hdop
        self.condition = threading.Condition()
        self.collecting = False
        self.samples = []
        self.rejected = {}
        self.subscriber = rospy.Subscriber(
            topic, String, self._callback, queue_size=100)

    def _reject(self, reason):
        self.rejected[reason] = self.rejected.get(reason, 0) + 1

    def _decode(self, message):
        try:
            state = json.loads(message.data)
            quality = int(state.get("GGA_quality", -1))
            if quality != self.required_quality:
                return None, "quality"
            if "PositionValid" in state and not bool(int(state["PositionValid"])):
                return None, "position_invalid"
            if "GNSSValid" in state and not bool(int(state["GNSSValid"])):
                return None, "gnss_invalid"
            if abs(float(state.get("Speed", 0.0))) > self.max_speed:
                return None, "moving"
            if float(state.get("GGA_hdop", 0.0)) > self.max_hdop:
                return None, "hdop"
            sample = (
                float(state.get("AntennaLat", state["Lat"])),
                float(state.get("AntennaLon", state["Lon"])),
                float(state.get("AntennaAlt", state["Alt"])),
            )
            if not all(math.isfinite(value) for value in sample):
                return None, "non_finite"
            return sample, None
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return None, "malformed"

    def _callback(self, message):
        with self.condition:
            if not self.collecting:
                return
            sample, reason = self._decode(message)
            if reason is not None:
                self._reject(reason)
                return
            self.samples.append(sample)
            self.condition.notify_all()

    def collect(self, label):
        with self.condition:
            self.samples = []
            self.rejected = {}
            self.collecting = True
            deadline = time.monotonic() + self.timeout
            while (len(self.samples) < self.sample_count
                   and not self.rospy.is_shutdown()):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self.condition.wait(timeout=min(remaining, 0.5))
            self.collecting = False
            samples = list(self.samples)
            rejected = dict(self.rejected)
        if len(samples) < self.sample_count:
            raise RuntimeError(
                "{} only collected {}/{} valid samples in {:.1f}s; rejected={}".format(
                    label, len(samples), self.sample_count, self.timeout, rejected))
        return samples, rejected


def map_module_api():
    script_dir = str(Path(__file__).resolve().parent)
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)
    import map_module

    return map_module


def resolve_module(module, project_root=None):
    map_module = map_module_api()
    root = map_module.find_project_root(project_root or "")
    map_module.validate_name(module)
    # Read now so a misspelled/nonexistent module fails before field sampling.
    map_module.read_config(root, module)
    return root, map_module.module_config_path(root, module)


def write_module(module, result, project_root=None):
    map_module = map_module_api()
    root, _path = resolve_module(module, project_root)

    values = map_module.read_config(root, module)
    for key in ("origin_lat", "origin_lon", "origin_alt",
                "yaw_enu_to_oxyz_deg"):
        values[key] = "{:.12g}".format(result[key])
    return map_module.write_config(root, module, values)


def print_result(result, scatter_a, scatter_b):
    print("\n========== 标定结果 ==========")
    print("origin_lat: {:.10f}".format(result["origin_lat"]))
    print("origin_lon: {:.10f}".format(result["origin_lon"]))
    print("origin_alt: {:.3f}".format(result["origin_alt"]))
    print("yaw_enu_to_oxyz_deg: {:.6f}".format(
        result["yaw_enu_to_oxyz_deg"]))
    print("A->B ENU: E={:+.3f}m N={:+.3f}m U={:+.3f}m, baseline={:.3f}m".format(
        result["baseline_east_m"], result["baseline_north_m"],
        result["baseline_up_m"], result["baseline_horizontal_m"]))
    print("A点水平散布: RMS={:.3f}m MAX={:.3f}m".format(*scatter_a))
    print("B点水平散布: RMS={:.3f}m MAX={:.3f}m".format(*scatter_b))


def main():
    import rospy

    parser = argparse.ArgumentParser(
        description="Use two stationary RTK points to calibrate a map frame")
    parser.add_argument("map_module", help="map module, for example site_a")
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--required-quality", type=int, default=4,
                        help="required NMEA GGA quality (4=RTK fixed)")
    parser.add_argument("--max-speed", type=float, default=0.10)
    parser.add_argument("--max-hdop", type=float, default=2.0)
    parser.add_argument("--min-baseline", type=float, default=10.0)
    parser.add_argument("--write", action="store_true",
                        help="write the result to maps/<module>/module.yaml")
    args = parser.parse_args(rospy.myargv()[1:])
    if args.samples < 10 or args.timeout <= 0.0 or args.min_baseline <= 0.0:
        parser.error("samples must be >=10 and timeout/min-baseline must be positive")

    try:
        project_root, config_path = resolve_module(args.map_module)
    except (OSError, RuntimeError, ValueError) as exc:
        print("ERROR: invalid map module: {}".format(exc), file=sys.stderr)
        return 1

    rospy.init_node("calibrate_map_frame", anonymous=True, disable_signals=True)
    sampler = StateSampler(
        FUSION_LOCATION_TOPIC, args.samples, args.timeout, args.required_quality,
        args.max_speed, args.max_hdop)
    print("\n两点法地图坐标标定（只使用RTK固定解和静止样本）")
    print("目标地图模块: {}".format(args.map_module))
    print("目标配置文件: {}".format(config_path))
    print("运行模式: {}".format(
        "标定成功后自动写入上述文件" if args.write else "仅显示，不写文件"))
    print("A点将成为固定地理原点；A到B必须严格指向地图 O-XYZ 的 +X。")
    print("注意：采集的是GNSS天线坐标，不要在采样期间推动车辆或遮挡天线。")
    try:
        input("\n将车辆停在A点，确认RTK固定后按 Enter 开始采样...")
        point_a, rejected_a = sampler.collect("point A")
        center_a = median_geodetic(point_a)
        print("A点完成: LAT={:.10f} LON={:.10f} ALT={:.3f}, rejected={}".format(
            center_a[0], center_a[1], center_a[2], rejected_a))
        input("\n沿地图 +X 直行至少 {:.1f}m，在B点停稳且保持同向，按 Enter 采样...".format(
            args.min_baseline))
        point_b, rejected_b = sampler.collect("point B")
    except (EOFError, KeyboardInterrupt, RuntimeError) as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        return 2

    result = estimate_frame(point_a, point_b)
    if result["baseline_horizontal_m"] < args.min_baseline:
        print("ERROR: A-B baseline {:.3f}m is below required {:.3f}m".format(
            result["baseline_horizontal_m"], args.min_baseline), file=sys.stderr)
        return 3
    scatter_a = horizontal_scatter(point_a, median_geodetic(point_a))
    scatter_b = horizontal_scatter(point_b, median_geodetic(point_b))
    print_result(result, scatter_a, scatter_b)
    if max(scatter_a[1], scatter_b[1]) > 0.20:
        print("WARNING: 静止点最大散布超过0.20m，建议检查RTK状态后重测。")
    if args.write:
        try:
            print("准备写入目标配置: {}".format(config_path))
            path = write_module(args.map_module, result, project_root)
        except (OSError, RuntimeError, ValueError) as exc:
            print("ERROR: cannot write map module: {}".format(exc), file=sys.stderr)
            return 4
        print("已写入 {}".format(path))
    else:
        print("未修改配置；确认结果后加 --write 再执行可写入地图模块。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
