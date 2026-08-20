# desert-tracker-real

中文的工程边界、接口表、算法替换点和核心命令见
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)。

Real-vehicle deployment project for the terrain-aware Hybrid A* and tracked
vehicle MPC developed in `desert-tracker`.

The project keeps the proven Qianxun/YIS525/ZMQ/SocketCAN contract from
`plan_try_genzong_xieposhadi`, while replacing the old path generator and
controller with the `desert-tracker` navigation algorithms.

## Architecture

```text
Qianxun GNSS + YIS525
        |
        v
vehicle_interface
  CurGNSS + /bus/location
        |
        +--> ROS Odometry + map/odom/base_link TF
                         |
                         v
              terrain Hybrid A* (move_base global plugin)
                         |
                         v
              tracked-vehicle MPC (move_base local plugin)
                         |
                     /cmd_vel
                         |
                         v
              PoliAcc / PoliSteer / PoliBrake (software stop request)
                         |
                         v
                 SocketCAN 0x200 / 0x203
```

## Catkin packages

| Package | Responsibility |
|---|---|
| `lio_sam` (`src/LIO-SAM`) | Modified LIO-SAM frontend with switchable map-to-odom TF ownership |
| `terrain_map_builder` | Align LIO-SAM PCD maps and build occupancy/DEM/TCM1 planner maps |
| `navigation` | Terrain-aware Hybrid A* and tracked-vehicle MPC plugins |
| `vehicle_interface` | Qianxun + YIS525 fusion, ROS navigation adapter and ZMQ-CAN bridge |
| `pointcloud_preprocess` | Convert real RSHELIOS `/rslidar_points` to the LIO-SAM `/points_raw` contract |
| `fixed_map_localization` | Optional fixed-map NDT correction for LIO-SAM runtime localization |
| `navigation_evaluation` | Per-run CSV/JSON/plots using real-vehicle topics instead of Gazebo truth |
| `slam_evaluation` | Offline TUM export and reproducible EVO APE/RPE evaluation |
| `vehicle_bringup` | Minimal real-vehicle launch and safety configuration |

Gazebo, vehicle meshes, RViz-only packages, historical plots and simulation
logs are deliberately not copied. The modified LIO-SAM core is retained
because localization switching requires deterministic TF ownership; its map
output still connects to planning through the documented PCD contract.

## Build on Ubuntu 20.04 / ROS Noetic

```bash
cd <desert-tracker-real所在目录>
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
python3 -m pip install pyserial pyzmq
```

Required ROS packages include `move_base`, `map_server`, PCL ROS and Grid Map.

## LIO-SAM map interface

The navigation stack consumes one directory, not a live SLAM topic. This keeps
mapping and autonomous driving separated and reproducible.

Expected pipeline:

```text
LIO-SAM save_map -> GlobalMap.pcd
                 -> optional rigid alignment to O-XYZ/map
                 -> traversability_map.yaml/.pgm
                 -> elevation_map.dem
                 -> terrain_cost.bin
                 -> terrain_cost_coarse.bin
```

Create a named map module.  The name is the only switch needed by later
mapping, navigation and evaluation launches:

```bash
PROJECT_ROOT="$(realpath "$(rospack find vehicle_bringup)/../..")"
MAP_MODULE="dune_a"
rosrun terrain_map_builder map_module.py create "${MAP_MODULE}" --origin-lat 22.000000 --origin-lon 114.000000 --origin-alt 0.0 --yaw-enu-to-oxyz-deg 0.0
```

Each module owns `maps/<name>/module.yaml`, `raw/`, `aligned/` and `planner/`.
Its bags are written to `bags/<name>/`, while evaluations are written to
`results/<name>/<localization_mode>/`.  See
[`docs/MULTI_MAP_MODULES.md`](docs/MULTI_MAP_MODULES.md).

After using the LIO-SAM `save_map` service, place `GlobalMap.pcd` in `raw/`.
If the saved map is already expressed in the same initial vehicle/O-XYZ frame,
the following alignment can be identity. Otherwise provide the measured rigid
transform, or pass LIO-SAM's `transformations.pcd` as `reference_pose_file`:

Write the measured alignment into this launch's defaults once, then run:

```bash
roslaunch terrain_map_builder align_lio_map.launch map_module:="${MAP_MODULE}"
```

Generate all planner layers:

```bash
roslaunch terrain_map_builder build_planner_map.launch map_module:="${MAP_MODULE}"
```

Before driving, verify that the PGM origin, the TCM1 origins and the Qianxun
control-point coordinates overlap in RViz. A visually plausible map with a
different origin or yaw is unsafe.

## Interface-only test with the existing plan_try sensor processes

Keep the original sensor/fusion processes running and start this project's
bridge without duplicating serial readers:

```bash
roslaunch vehicle_interface io.launch start_sensors:=false start_can:=false
```

Then start navigation with CAN still disabled:

```bash
roslaunch vehicle_bringup navigation.launch
```

Check `/odometry/imu_incremental`, `/imu/data`, TF,
`/vehicle/state`（实测时间戳、`base_link`速度约定）,
`/hybrid_astar/plan` and `/cmd_vel`.

## LiDAR use: offline mapping and optional online localization

The normal `qianxun` mode does not start LiDAR or LIO-SAM.  A separate mapping
session is used to build the fixed map, while the optional `lio_sam` mode starts
the same sensor frontend against that already-built map.  Sensor topics remain
the same as simulation:

```text
RSHELIOS driver (XYZIRT) -> /rslidar_points -> native ring/time adapter -> /points_raw
YIS525 -> /imu/data
LIO-SAM -> saved GlobalMap.pcd
```

After setting the serial ports, calibration and LiDAR extrinsic defaults once
in the launch/config files, the mapping commands stay short:

```bash
roslaunch vehicle_bringup mapping.launch map_module:="${MAP_MODULE}"
```

For reproducible Qianxun/LIO-SAM alignment, record the processed point cloud,
the one shared IMU stream and the fused Qianxun state together:

```bash
roslaunch vehicle_bringup record_mapping_bag.launch map_module:="${MAP_MODULE}"
```

For a reusable map, store the fixed Qianxun origin and axis rotation in that
module's `module.yaml`. If the first mapping run intentionally uses the first
GNSS fix as origin, write the reported `OriginLat`, `OriginLon`, `OriginAlt`
and `YawEnuToOxyzDeg` back with `map_module.py configure`; later launches load
them automatically when `map_module` is selected.

The recorder starts the real RSHELIOS driver and converter before recording
`/points_raw`, `/imu/data`, `/bus/location`, `/tf` and `/tf_static`. The real
driver's unconverted topic is `/rslidar_points`; LIO-SAM never subscribes to it
directly. It waits three seconds for initialization and then requires fresh
messages on both `/points_raw` and `/imu/data`; a timeout stops the launch
without creating a bag.

The vehicle IPC system clock is the sole absolute timestamp domain. RSHELIOS
runs with `use_lidar_clock: false`, Yesense stamps `/imu/data` from ROS system
time on the same IPC, and point `time` remains only a relative in-scan offset.
Replay a recorded run with sensors and CAN disabled:

```bash
roslaunch vehicle_bringup bag_mapping.launch
rosbag play --clock "bags/${MAP_MODULE}"/mapping_run_*.bag
```

Before mapping, validate all split bag parts:

```bash
rosrun terrain_map_builder check_mapping_bag.py "bags/${MAP_MODULE}"/mapping_run_*.bag
```

Save and stop the mapping stack:

```bash
rosrun terrain_map_builder map_module.py save "${MAP_MODULE}" --resolution 0.2
```

Then run `align_lio_map.launch` and `build_planner_map.launch`. In `qianxun`
mode navigation uses only the generated static map and Qianxun/YIS525
positioning. In `lio_sam` mode LIO-SAM may run online for localization only.
Neither mode adds a PointCloud2 obstacle layer, and MPC does not receive live
LiDAR obstacles.

## Full real-vehicle launch

Initialize SocketCAN first:

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
```

Write the measured calibration values into the launch/config defaults once.
`UNKNOWN` heading mode is blocked by the command bridge. Normal experiment
commands are then one line:

```bash
roslaunch vehicle_bringup full_system.launch map_module:="${MAP_MODULE}" localization_mode:=qianxun
```

To compare against LiDAR localization, use the same processed map and the
aligned PCD from which it was generated:

```bash
roslaunch vehicle_bringup full_system.launch map_module:="${MAP_MODULE}" localization_mode:=lio_sam
```

In `qianxun` mode the bridge owns `/odometry/imu_incremental` and TF. In
`lio_sam` mode those bridge outputs are automatically disabled; LIO-SAM owns
`odom -> base_link` and the fixed-map localizer owns `map -> odom`. The live
PointCloud2 obstacle layer remains absent in both modes, so MPC is still a
path-tracking controller.

Both modes share one physical YIS525. Its driver opens the serial port once and
is the sole publisher of `/imu/data`; Qianxun fusion and LIO-SAM subscribe to
that exact message and header timestamp. The bridge no longer republishes IMU.
The driver checks ROS-master ownership and stops on a duplicate publisher.

The RSHELIOS SDK is configured for `POINT_TYPE=XYZIRT` and publishes `/rslidar_points`.
The adapter preserves its native `ring` and converts absolute `timestamp` to
LIO-SAM relative `time` on `/points_raw`; it does not infer rings from angles.
LiDAR-to-IMU extrinsics and `imuTimeOffset` remain zero identity placeholders
until measured on the complete vehicle; development-board values are not used.

During bag mapping, record `/lio_sam/mapping/odometry` to a compact result bag.
The alignment tool accepts the original split sensor bags and that result bag
together, so their original timestamps remain the synchronization key:

```bash
rosbag record -O "bags/${MAP_MODULE}/lio_result.bag" /lio_sam/mapping/path /lio_sam/mapping/odometry
rosrun terrain_map_builder align_qianxun_lio.py "bags/${MAP_MODULE}"/mapping_run_*.bag "bags/${MAP_MODULE}/lio_result.bag" --output "maps/${MAP_MODULE}/aligned/qianxun_lio_alignment.yaml"
```

The command reports fit RMSE and prints the `translation_x`, `translation_y`
and `yaw_deg` arguments for `align_lio_map.launch`. Use a trajectory containing
turns or a loop; a straight line does not adequately constrain map yaw.

## Per-run real-vehicle evaluation

The evaluator starts by default with `full_system.launch`. It begins on every
`/move_base/goal` and saves a numbered run when the action succeeds, aborts,
is preempted, or the launch is stopped:

```text
${PROJECT_ROOT}/results/${MAP_MODULE}/qianxun/nav_N/
${PROJECT_ROOT}/results/${MAP_MODULE}/lio_sam/nav_N/
```

Every run manifest, metrics JSON and comparison CSV records `map_module`,
`localization_mode` and a fingerprint of the exact module/config/map layers,
so regenerated maps cannot be silently mixed with older evaluations.

Each run contains raw CSV streams, JSON metrics and plots for the global path,
Qianxun reference trajectory, selected localization, controller odometry,
cross-track/goal error, commands, MPC diagnostics, prediction horizons,
terrain attitude and planner Pareto statistics. It also records `/imu/data`,
the complete Qianxun/GNSS state and raw CAN motor feedback directly during the
run, producing `imu_raw.csv`, `motor_feedback_raw.csv` and IMU/GPS/motor
diagnostic plots. LiDAR remains outside the evaluator; use a mapping bag only
when point clouds are actually required. Disable the evaluator with
`start_evaluator:=false`.

SLAM accuracy is kept separate from navigation tracking evaluation. Export a
trajectory and run rigid SE(3) EVO metrics without scale correction with:

```bash
rosrun slam_evaluation odom_to_tum.py "bags/${MAP_MODULE}/lio_result.bag" "results/${MAP_MODULE}/slam/lio.tum" --topic /lio_sam/mapping/odometry
rosrun slam_evaluation evaluate_trajectories.py "results/${MAP_MODULE}/slam/ground_truth.tum" "results/${MAP_MODULE}/slam/lio.tum" "results/${MAP_MODULE}/slam/evo"
```

Additional roll, pitch, gyro and lever-arm calibration arguments are exposed by
`vehicle_interface/launch/io.launch` and should be filled there or passed
through a site-specific launch file before slope experiments.

## Coordinate contract

The retained plan_try state uses O-XYZ heading measured from global `+Y`, with
counterclockwise positive. ROS uses yaw from global `+X`. The adapter performs:

```text
yaw_ros = wrap(head_oxyz + 90 degrees)
```

It does not alter X/Y positions. Therefore the LIO-SAM map and Qianxun origin
must already describe the same physical O-XYZ axes.

The vehicle's tail-first speed sign remains isolated in the SocketCAN bridge.
The planner and MPC always use standard logical commands: positive `v` is
forward and positive `omega` is counterclockwise.

## Safety gates retained

The real interface bridge sends zero speed/yaw and asserts `PoliBrake=1` when:

- fused state is stale;
- `FusionValid`, `PositionValid` or `ControlFrameReady` is false;
- the declared YIS525 heading mode is `UNKNOWN`;
- `/cmd_vel` is stale.

The CAN bridge records independent receive times for `PoliAcc`, `PoliSteer`
and `PoliBrake`. Motion remains inhibited until all three channels have been
seen, and any one missing or stale channel independently forces zero speed and
yaw rate. These software checks do not replace the physical emergency stop.
The inherited CAN `0x200` payload
contains only signed speed and yaw-rate `int16` fields; `PoliBrake` is not a
verified physical-brake bit and currently means "force both motion fields to
zero". Active braking must not be claimed until the vehicle's VCU protocol
document and a stationary hardware test identify a separate brake field.

The tracked chassis is allowed to pivot: a valid `v=0, omega!=0` command is
forwarded to CAN. The optional `--zero-steer-when-stopped` bridge switch is not
used by the real-vehicle launch.

The MPC additionally rejects stale `/vehicle/state`, stale TF and excessive
pose covariance. In `lio_sam` mode it also requires the fixed-map localizer's
latched `/localization/quality_ok`, which expires when trusted NDT matches stop.
At 10 Hz, a control-cycle gap first emits a zero command. Candidate search is
limited to 80 ms: one timeout reuses the previous complete feasible command,
while three consecutive timeouts emit a zero command. The selected path
segment is cached and subsequent projections use a bounded local segment
window, with global reacquisition only after a large path-distance mismatch.

## Validation

Pure-Python contract tests can run without ROS:

```bash
python3 -m unittest discover -s test -v
```

The included `maps/sample_desert` is only for software/interface checks and is
not aligned to any real GNSS origin.
