#!/usr/bin/env python3
"""Validate recorded LiDAR, IMU and Qianxun topics before LIO-SAM mapping."""

import argparse
import json
import math
import sys

import rosbag


FIELD_TYPES = {
    1: "INT8", 2: "UINT8", 3: "INT16", 4: "UINT16",
    5: "INT32", 6: "UINT32", 7: "FLOAT32", 8: "FLOAT64",
}
FUSION_LOCATION_TOPIC = "/fusion_location"


def stamp_of(message, bag_stamp):
    header = getattr(message, "header", None)
    if header is not None and header.stamp.to_sec() > 0.0:
        return header.stamp.to_sec()
    return bag_stamp.to_sec()


def quaternion_norm(q):
    return math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)


def collect_topic(bags, topic, limit=None):
    rows = []
    for path in bags:
        with rosbag.Bag(path, "r") as bag:
            for _, message, bag_stamp in bag.read_messages(topics=[topic]):
                rows.append((stamp_of(message, bag_stamp), message, bag_stamp.to_sec()))
                if limit and len(rows) >= limit:
                    return rows
    rows.sort(key=lambda row: row[0])
    return rows


def frequency(rows):
    if len(rows) < 2 or rows[-1][0] <= rows[0][0]:
        return 0.0
    return (len(rows) - 1) / (rows[-1][0] - rows[0][0])


def main():
    parser = argparse.ArgumentParser(
        description="Preflight one or more synchronized ROS1 mapping bags")
    parser.add_argument("bags", nargs="+", help="bag files recorded in one run")
    parser.add_argument("--points", default="/points_raw")
    parser.add_argument("--imu", default="/imu/data")
    parser.add_argument("--rings", type=int, default=32)
    parser.add_argument("--max-start-offset", type=float, default=1.0)
    args = parser.parse_args()

    try:
        points = collect_topic(args.bags, args.points)
        imu = collect_topic(args.bags, args.imu)
        qianxun = collect_topic(args.bags, FUSION_LOCATION_TOPIC)
    except (OSError, rosbag.bag.ROSBagException) as exc:
        print("ERROR: cannot read bag: {}".format(exc), file=sys.stderr)
        return 2

    missing = [name for name, rows in (
        (args.points, points), (args.imu, imu),
        (FUSION_LOCATION_TOPIC, qianxun)) if not rows]
    if missing:
        print("ERROR: missing topic(s): " + ", ".join(missing))
        return 3

    for name, rows in ((args.points, points), (args.imu, imu),
                       (FUSION_LOCATION_TOPIC, qianxun)):
        non_monotonic = sum(b[0] <= a[0] for a, b in zip(rows, rows[1:]))
        print("{}: count={} start={:.6f} end={:.6f} rate={:.2f}Hz non_monotonic={}".format(
            name, len(rows), rows[0][0], rows[-1][0], frequency(rows), non_monotonic))
        if non_monotonic:
            print("ERROR: {} header timestamps are not strictly increasing".format(name))
            return 4

    overlap_start = max(points[0][0], imu[0][0], qianxun[0][0])
    overlap_end = min(points[-1][0], imu[-1][0], qianxun[-1][0])
    print("Common overlap: {:.3f}s".format(max(0.0, overlap_end - overlap_start)))
    if overlap_end <= overlap_start:
        print("ERROR: LiDAR, IMU and Qianxun time ranges do not overlap")
        return 5
    start_spread = max(points[0][0], imu[0][0]) - min(points[0][0], imu[0][0])
    if start_spread > args.max_start_offset:
        print("WARNING: LiDAR/IMU start times differ by {:.3f}s".format(start_spread))

    cloud = points[0][1]
    fields = {field.name: FIELD_TYPES.get(field.datatype, str(field.datatype))
              for field in cloud.fields}
    print("PointCloud2 frame={!r} fields={}".format(
        cloud.header.frame_id, ", ".join("{}:{}".format(*item)
                                        for item in fields.items())))
    required = {"x", "y", "z", "intensity", "ring", "time"}
    absent = sorted(required - set(fields))
    if absent:
        print("ERROR: /points_raw missing fields: " + ", ".join(absent))
        return 6
    if fields["ring"] not in ("UINT8", "UINT16", "UINT32"):
        print("ERROR: unsupported ring datatype {}".format(fields["ring"]))
        return 7
    if fields["time"] not in ("FLOAT32", "FLOAT64"):
        print("ERROR: time must be FLOAT32 or FLOAT64")
        return 8

    imu_samples = imu[:min(2000, len(imu))]
    valid_q = 0
    acceleration_norms = []
    for _, message, _ in imu_samples:
        qnorm = quaternion_norm(message.orientation)
        if math.isfinite(qnorm) and qnorm >= 0.5:
            valid_q += 1
        a = message.linear_acceleration
        acceleration_norms.append(math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z))
    mean_acc = sum(acceleration_norms) / len(acceleration_norms)
    print("IMU frame={!r} valid_orientation={}/{} mean_acc_norm={:.4f}m/s^2".format(
        imu_samples[0][1].header.frame_id, valid_q, len(imu_samples), mean_acc))
    if valid_q == 0:
        print("ERROR: /imu/data has no valid orientation; real Yesense data must not use dataset fallback")
        return 9
    if not math.isfinite(mean_acc) or not 5.0 <= mean_acc <= 15.0:
        print("ERROR: IMU acceleration units or values are invalid")
        return 10

    valid_location = 0
    for _, message, _ in qianxun[:min(500, len(qianxun))]:
        try:
            state = json.loads(message.data)
            values = (float(state["UTM_x"]), float(state["UTM_y"]),
                      float(state["Head"]))
            if all(math.isfinite(value) for value in values):
                valid_location += 1
        except (AttributeError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            pass
    print("Qianxun valid JSON samples: {}/{}".format(
        valid_location, min(500, len(qianxun))))
    if valid_location == 0:
        print("ERROR: /fusion_location has no valid UTM_x/UTM_y/Head state")
        return 11

    print("PASS: bag satisfies the /points_raw + /imu/data + /fusion_location mapping contract")
    return 0


if __name__ == "__main__":
    sys.exit(main())
