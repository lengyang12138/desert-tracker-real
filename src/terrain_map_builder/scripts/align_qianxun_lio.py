#!/usr/bin/env python3
"""Estimate the planar rigid transform from a LIO-SAM map to Qianxun O-XYZ."""

import argparse
import bisect
import json
import math
import sys

import numpy as np
import rosbag


FUSION_LOCATION_TOPIC = "/fusion_location"


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def healthy(state):
    for key in ("FusionValid", "PositionValid", "ControlFrameReady"):
        if key in state and not bool(int(state[key])):
            return False
    return True


def read_samples(bags, qianxun_topic, lio_topic, time_offset):
    qianxun, lio = [], []
    latest_path = None
    frame_metadata = {}
    for path in bags:
        with rosbag.Bag(path, "r") as bag:
            for topic, message, bag_stamp in bag.read_messages(
                    topics=[qianxun_topic, lio_topic]):
                if topic == qianxun_topic:
                    try:
                        state = json.loads(message.data)
                        if not healthy(state):
                            continue
                        x, y = float(state["UTM_x"]), float(state["UTM_y"])
                        # O-XYZ heading: zero at +Y, CCW positive.
                        yaw = wrap(math.radians(float(state["Head"])) + math.pi / 2.0)
                        if all(math.isfinite(value) for value in (x, y, yaw)):
                            qianxun.append(
                                (bag_stamp.to_sec() + time_offset, x, y, yaw))
                            if not frame_metadata:
                                frame_metadata = {
                                    key: state.get(key) for key in (
                                        "OriginLat", "OriginLon", "OriginAlt",
                                        "OriginMode", "YawEnuToOxyzDeg")
                                }
                    except (AttributeError, KeyError, TypeError, ValueError,
                            json.JSONDecodeError):
                        continue
                else:
                    if hasattr(message, "poses"):
                        if message.poses:
                            latest_path = message
                        continue
                    stamp = message.header.stamp.to_sec() or bag_stamp.to_sec()
                    pose = message.pose.pose
                    values = (pose.position.x, pose.position.y,
                              yaw_from_quaternion(pose.orientation))
                    if all(math.isfinite(value) for value in values):
                        lio.append((stamp,) + values)
    if latest_path is not None:
        lio = []
        for stamped_pose in latest_path.poses:
            stamp = stamped_pose.header.stamp.to_sec()
            pose = stamped_pose.pose
            values = (pose.position.x, pose.position.y,
                      yaw_from_quaternion(pose.orientation))
            if stamp > 0.0 and all(math.isfinite(value) for value in values):
                lio.append((stamp,) + values)
    qianxun.sort()
    lio.sort()
    return qianxun, lio, frame_metadata


def associate(qianxun, lio, max_dt, min_step):
    stamps = [sample[0] for sample in qianxun]
    q_points, l_points, heading_delta, time_delta = [], [], [], []
    last_lio = None
    for l_sample in lio:
        index = bisect.bisect_left(stamps, l_sample[0])
        candidates = [i for i in (index - 1, index) if 0 <= i < len(stamps)]
        if not candidates:
            continue
        nearest = min(candidates, key=lambda i: abs(stamps[i] - l_sample[0]))
        q_sample = qianxun[nearest]
        dt = q_sample[0] - l_sample[0]
        if abs(dt) > max_dt:
            continue
        if last_lio is not None and math.hypot(
                l_sample[1] - last_lio[0], l_sample[2] - last_lio[1]) < min_step:
            continue
        last_lio = (l_sample[1], l_sample[2])
        q_points.append((q_sample[1], q_sample[2]))
        l_points.append((l_sample[1], l_sample[2]))
        heading_delta.append(wrap(q_sample[3] - l_sample[3]))
        time_delta.append(dt)
    return (np.asarray(l_points), np.asarray(q_points),
            np.asarray(heading_delta), np.asarray(time_delta))


def fit_se2(source, target, yaw=None):
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    if yaw is None:
        covariance = (source - source_center).T @ (target - target_center)
        u, singular, vt = np.linalg.svd(covariance)
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0.0:
            vt[-1, :] *= -1.0
            rotation = vt.T @ u.T
    else:
        c, s = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[c, -s], [s, c]])
        singular = np.linalg.svd(source - source_center, compute_uv=False)
    translation = target_center - rotation @ source_center
    residual = np.linalg.norm(
        (rotation @ source.T).T + translation - target, axis=1)
    return rotation, translation, residual, singular


def robust_fit(source, target, fixed_yaw, max_residual):
    # Seed with deterministic two-point RANSAC so a few GNSS jumps cannot
    # rotate the initial least-squares solution and hide the true inliers.
    mask = np.ones(len(source), dtype=bool)
    if len(source) >= 5:
        rng = np.random.default_rng(0)
        best_mask = mask
        best_count = 0
        threshold = min(max_residual, 0.75)
        iterations = min(1000, max(100, len(source) * 4))
        for _ in range(iterations):
            first, second = rng.choice(len(source), size=2, replace=False)
            source_delta = source[second] - source[first]
            target_delta = target[second] - target[first]
            if np.linalg.norm(source_delta) < 0.5 or np.linalg.norm(target_delta) < 0.5:
                continue
            yaw = fixed_yaw
            if yaw is None:
                yaw = wrap(math.atan2(target_delta[1], target_delta[0]) -
                           math.atan2(source_delta[1], source_delta[0]))
            c, s = math.cos(yaw), math.sin(yaw)
            rotation = np.array([[c, -s], [s, c]])
            translation = target[first] - rotation @ source[first]
            residual = np.linalg.norm(
                (rotation @ source.T).T + translation - target, axis=1)
            candidate = residual <= threshold
            count = int(candidate.sum())
            if count > best_count:
                best_count = count
                best_mask = candidate
        if best_count >= 5:
            mask = best_mask
    for _ in range(10):
        rotation, translation, _, singular = fit_se2(
            source[mask], target[mask], fixed_yaw)
        all_residual = np.linalg.norm(
            (rotation @ source.T).T + translation - target, axis=1)
        selected = all_residual[mask]
        median = np.median(selected)
        mad = np.median(np.abs(selected - median))
        threshold = min(
            max_residual, max(0.20, median + 3.0 * 1.4826 * mad))
        new_mask = all_residual <= threshold
        if new_mask.sum() < 5 or np.array_equal(new_mask, mask):
            break
        mask = new_mask
    rotation, translation, residual, singular = fit_se2(
        source[mask], target[mask], fixed_yaw)
    return rotation, translation, residual, singular, mask


def write_yaml(path, rotation, translation, metrics, frame_metadata):
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    matrix = [rotation[0, 0], rotation[0, 1], 0.0, translation[0],
              rotation[1, 0], rotation[1, 1], 0.0, translation[1],
              0.0, 0.0, 1.0, 0.0,
              0.0, 0.0, 0.0, 1.0]
    with open(path, "w", encoding="utf-8", newline="\n") as stream:
        stream.write("source_frame: lio_sam_map\n")
        stream.write("target_frame: qianxun_map\n")
        stream.write("qianxun_frame:\n")
        for key, yaml_key in (
                ("OriginLat", "origin_lat"),
                ("OriginLon", "origin_lon"),
                ("OriginAlt", "origin_alt"),
                ("OriginMode", "origin_mode"),
                ("YawEnuToOxyzDeg", "yaw_enu_to_oxyz_deg")):
            stream.write("  {}: {}\n".format(
                yaml_key, frame_metadata.get(key)))
        stream.write("translation_x: {:.12g}\n".format(translation[0]))
        stream.write("translation_y: {:.12g}\n".format(translation[1]))
        stream.write("translation_z: 0.0\nroll_deg: 0.0\npitch_deg: 0.0\n")
        stream.write("yaw_deg: {:.12g}\n".format(math.degrees(yaw)))
        stream.write("matrix_4x4_row_major: [{}]\n".format(
            ", ".join("{:.12g}".format(value) for value in matrix)))
        stream.write("metrics:\n")
        for key, value in metrics.items():
            stream.write("  {}: {}\n".format(key, value))


def main():
    parser = argparse.ArgumentParser(
        description="Fit LIO-SAM map coordinates to Qianxun O-XYZ coordinates")
    parser.add_argument("bags", nargs="+")
    parser.add_argument("--output", required=True)
    parser.add_argument("--lio-topic", default="/lio_sam/mapping/path")
    parser.add_argument("--qianxun-time-offset", type=float, default=0.0)
    parser.add_argument("--max-time-diff", type=float, default=0.05)
    parser.add_argument("--min-step", type=float, default=0.20)
    parser.add_argument("--max-residual", type=float, default=2.0)
    parser.add_argument("--yaw-source", choices=("trajectory", "heading"),
                        default="trajectory")
    args = parser.parse_args()

    try:
        qianxun, lio, frame_metadata = read_samples(
            args.bags, FUSION_LOCATION_TOPIC, args.lio_topic,
            args.qianxun_time_offset)
    except (OSError, rosbag.bag.ROSBagException) as exc:
        print("ERROR: cannot read bag: {}".format(exc), file=sys.stderr)
        return 2
    if not qianxun or not lio:
        print("ERROR: missing Qianxun or LIO-SAM trajectory", file=sys.stderr)
        return 3
    source, target, heading_delta, time_delta = associate(
        qianxun, lio, args.max_time_diff, args.min_step)
    if len(source) < 10:
        print("ERROR: only {} synchronized pairs; need at least 10".format(
            len(source)), file=sys.stderr)
        return 4
    extent = np.linalg.norm(np.ptp(source, axis=0))
    if extent < 5.0:
        print("ERROR: trajectory extent {:.2f}m is too small".format(extent),
              file=sys.stderr)
        return 5

    heading_yaw = math.atan2(np.sin(heading_delta).sum(),
                             np.cos(heading_delta).sum())
    fixed_yaw = heading_yaw if args.yaw_source == "heading" else None
    rotation, translation, residual, singular, mask = robust_fit(
        source, target, fixed_yaw, args.max_residual)
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    heading_disagreement = math.degrees(wrap(yaw - heading_yaw))
    excitation_ratio = (singular[-1] / singular[0]
                        if len(singular) > 1 and singular[0] > 0.0 else 0.0)
    metrics = {
        "synchronized_pairs": len(source),
        "inliers": int(mask.sum()),
        "trajectory_extent_m": "{:.6g}".format(extent),
        "rmse_m": "{:.6g}".format(math.sqrt(np.mean(residual ** 2))),
        "median_m": "{:.6g}".format(np.median(residual)),
        "max_m": "{:.6g}".format(np.max(residual)),
        "mean_time_delta_s": "{:.6g}".format(np.mean(time_delta)),
        "trajectory_excitation_ratio": "{:.6g}".format(excitation_ratio),
        "heading_yaw_deg": "{:.6g}".format(math.degrees(heading_yaw)),
        "heading_disagreement_deg": "{:.6g}".format(heading_disagreement),
    }
    write_yaml(args.output, rotation, translation, metrics, frame_metadata)
    print("Wrote {}".format(args.output))
    print("LIO -> Qianxun: x={:.6f} y={:.6f} yaw={:.6f}deg RMSE={}m inliers={}/{}".format(
        translation[0], translation[1], math.degrees(yaw), metrics["rmse_m"],
        metrics["inliers"], len(source)))
    print("Apply with: roslaunch terrain_map_builder align_lio_map.launch "
          "translation_x:={:.12g} translation_y:={:.12g} yaw_deg:={:.12g}".format(
              translation[0], translation[1], math.degrees(yaw)))
    if excitation_ratio < 0.02:
        print("WARNING: trajectory is nearly straight; record turns or a loop before trusting yaw")
    if abs(heading_disagreement) > 5.0:
        print("WARNING: position-fit and heading yaw differ by {:.2f}deg".format(
            heading_disagreement))
    if frame_metadata.get("OriginMode") != "fixed":
        print("WARNING: Qianxun origin was not fixed; reuse the recorded OriginLat/OriginLon before navigation")
    return 0


if __name__ == "__main__":
    sys.exit(main())
