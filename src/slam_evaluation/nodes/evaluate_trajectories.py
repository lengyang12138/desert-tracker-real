#!/usr/bin/env python3
"""Run reproducible EVO SE(3) APE/RPE evaluation without scale correction."""

import argparse
import os
import shutil
import subprocess
import sys


def run(command):
    print("+ " + " ".join(command))
    subprocess.run(command, check=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("ground_truth")
    parser.add_argument("estimate")
    parser.add_argument("output_dir")
    parser.add_argument("--max-time-diff", type=float, default=0.02)
    parser.add_argument("--rpe-distance", type=float, default=10.0)
    args = parser.parse_args()

    required = ("evo_ape", "evo_rpe", "evo_res", "evo_traj")
    missing = [command for command in required if shutil.which(command) is None]
    if missing:
        print("ERROR: missing EVO commands: " + ", ".join(missing),
              file=sys.stderr)
        return 2
    os.makedirs(args.output_dir, exist_ok=True)
    dt = str(args.max_time_diff)
    distance = str(args.rpe_distance)
    common = ["--align", "--t_max_diff", dt]
    outputs = {
        "ape_translation": ["evo_ape", "tum", args.ground_truth, args.estimate]
            + common + ["--pose_relation", "trans_part"],
        "ape_rotation": ["evo_ape", "tum", args.ground_truth, args.estimate]
            + common + ["--pose_relation", "angle_deg"],
        "ape_xy": ["evo_ape", "tum", args.ground_truth, args.estimate]
            + common + ["--project_to_plane", "xy", "--pose_relation", "trans_part"],
        "rpe_translation": ["evo_rpe", "tum", args.ground_truth, args.estimate]
            + common + ["--delta", distance, "--delta_unit", "m", "--all_pairs",
                        "--pose_relation", "trans_part"],
        "rpe_rotation": ["evo_rpe", "tum", args.ground_truth, args.estimate]
            + common + ["--delta", distance, "--delta_unit", "m", "--all_pairs",
                        "--pose_relation", "angle_deg"],
    }
    archives = []
    try:
        for name, command in outputs.items():
            archive = os.path.join(args.output_dir, name + ".zip")
            run(command + ["--save_results", archive])
            archives.append(archive)
        run(["evo_res"] + archives + ["--use_filenames", "--save_table",
                                      os.path.join(args.output_dir, "summary.csv")])
        run(["evo_traj", "tum", args.ground_truth, args.estimate,
             "--ref", args.ground_truth] + common + ["--plot_mode", "xy",
             "--save_plot", os.path.join(args.output_dir, "trajectory.png")])
    except subprocess.CalledProcessError as exc:
        print("ERROR: EVO failed with code {}".format(exc.returncode),
              file=sys.stderr)
        return exc.returncode or 3
    print("Evaluation written to {}".format(args.output_dir))
    return 0


if __name__ == "__main__":
    sys.exit(main())
