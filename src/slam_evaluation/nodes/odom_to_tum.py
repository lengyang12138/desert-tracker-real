#!/usr/bin/env python3
"""Export ROS1 Odometry or PoseStamped messages to TUM trajectory format."""

import argparse
import math
import sys

import rosbag


def pose_from_message(message):
    pose = message.pose.pose if hasattr(message.pose, "pose") else message.pose
    return pose.position, pose.orientation


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("output")
    parser.add_argument("--topic", default="/lio_sam/mapping/odometry")
    args = parser.parse_args()

    rows = []
    last_stamp = -math.inf
    try:
        with rosbag.Bag(args.bag, "r") as bag:
            for _, message, bag_stamp in bag.read_messages(topics=[args.topic]):
                stamp = message.header.stamp.to_sec() or bag_stamp.to_sec()
                if stamp <= last_stamp:
                    continue
                position, orientation = pose_from_message(message)
                values = (stamp, position.x, position.y, position.z,
                          orientation.x, orientation.y, orientation.z,
                          orientation.w)
                if not all(math.isfinite(value) for value in values):
                    continue
                qnorm = math.sqrt(sum(value * value for value in values[4:]))
                if qnorm < 1e-9:
                    continue
                rows.append(values[:4] + tuple(
                    value / qnorm for value in values[4:]))
                last_stamp = stamp
    except (OSError, rosbag.bag.ROSBagException) as exc:
        print("ERROR: cannot read bag: {}".format(exc), file=sys.stderr)
        return 2
    if not rows:
        print("ERROR: no valid poses on {}".format(args.topic), file=sys.stderr)
        return 3
    with open(args.output, "w", encoding="utf-8", newline="\n") as stream:
        for row in rows:
            stream.write(" ".join("{:.9f}".format(value) for value in row) + "\n")
    print("Exported {} poses ({:.3f}s) to {}".format(
        len(rows), rows[-1][0] - rows[0][0], args.output))
    return 0


if __name__ == "__main__":
    sys.exit(main())
