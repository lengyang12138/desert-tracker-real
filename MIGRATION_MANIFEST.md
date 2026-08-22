# Migration manifest

## From desert-tracker

- `my_Global_path_planning.cpp/.h`: complete terrain-aware Hybrid A* and basic
  comparison plugin;
- `mpc_local_planner.cpp/.h`: tracked-vehicle predictive local controller;
- planner/MPC plugin descriptors and current tuned YAML parameters;
- `pcd_to_costmap.cpp`: PCD to occupancy, DEM and fine/coarse TCM1 processing;
- `pcd_frame_transform.cpp`: non-destructive LIO-SAM PCD frame alignment;
- one compact simulation map set for non-driving integration checks.

The files were copied from the supplied worktree's current state, including
the user's uncommitted algorithm changes.

## From plan_try_genzong_xieposhadi

- YIS525 UART decoder and sole ROS `/imu/data` publisher;
- Qianxun RMC/GGA position processing;
- Qianxun + YIS525 control-point fusion;
- shared O-XYZ frame conventions and JSON ZMQ context;
- `PoliAcc/PoliSteer/PoliBrake` to SocketCAN bridge.

The original planning and MPC/LQR/STSMC Python scripts were not copied because
their roles are replaced by the ROS navigation plugins.

## New integration code

- `/fusion_location` JSON to ROS Odometry/IMU/TF conversion;
- `/cmd_vel` to the retained ZMQ policy contract;
- state/command health gates;
- Catkin package boundaries, launch files, map contract and unit tests.
- RSHELIOS native `ring/timestamp` conversion with the fixed real-vehicle
  `/rslidar_points -> /points_raw` contract;
- fixed-map localization and the navigation evaluator, with Gazebo inputs
  replaced by Qianxun/real-vehicle ROS topics.
- deterministic mapping-bag recording/replay, bag preflight checks,
  Qianxun-to-LIO trajectory alignment and an isolated SLAM APE/RPE evaluator;
- the simulation project's modified LIO-SAM core needed for deterministic TF
  ownership switching (`publish_tf`) between Qianxun and fixed-map modes;
  upstream license retained in `src/LIO-SAM/LICENSE`.

## Runtime boundary

- Qianxun is the default runtime localization source. LIO-SAM/fixed-map NDT is
  an explicit comparison mode selected by `localization_mode:=lio_sam`.
- Navigation consumes the processed static map; no live PointCloud2 obstacle
  layer is loaded.
- MPC receives the planned path and `/odometry/imu_incremental`; it does not
  receive real-time LiDAR obstacles in the current phase.
