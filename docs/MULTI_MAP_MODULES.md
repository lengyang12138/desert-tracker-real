# 多地图模块操作说明

## 1. 目录与参数契约

项目现在使用统一参数 `map_module` 切换地图。地图名只能包含英文字母、数字、
下划线和短横线，且必须以字母或数字开头。

```text
maps/<map_module>/
  module.yaml                         # 本地图固定千寻原点和O-XYZ旋转角
  raw/GlobalMap.pcd                   # LIO-SAM原始地图
  aligned/GlobalMap.pcd               # 对齐到千寻O-XYZ后的地图
  planner/traversability_map.yaml
  planner/traversability_map.pgm
  planner/elevation_map.dem
  planner/obstacle_map.pgm
  planner/terrain_cost.bin
  planner/terrain_cost_coarse.bin

bags/<map_module>/                    # 本地图建图与LIO结果bag
results/<map_module>/qianxun/nav_N/   # 本地图千寻导航评估
results/<map_module>/lio_sam/nav_N/   # 本地图激光定位导航评估
results/<map_module>/slam/            # 本地图SLAM/EVO评估
```

启动守卫会检查地图名、`module.yaml`、规划图层、固定原点以及 LIO 模式所需的
对齐 PCD，并拒绝 `map_module` 与 `map_dir` 指向不同模块。检查失败时整套导航
launch 会退出。

## 2. 创建地图模块

下面以 `dune_a` 为例。所有终端命令均是一条命令一行。

```bash
export MAP_MODULE=dune_a
rosrun terrain_map_builder map_module.py create "$MAP_MODULE" --origin-lat 22.000000 --origin-lon 114.000000 --origin-alt 0.0 --yaw-enu-to-oxyz-deg 0.0
rosrun terrain_map_builder map_module.py list
rosrun terrain_map_builder map_module.py path "$MAP_MODULE"
```

如果首次建图时还不知道固定原点，可以先创建空原点模块：

```bash
rosrun terrain_map_builder map_module.py create dune_b
```

采集后把输出状态中的 `OriginLat`、`OriginLon`、`OriginAlt` 和
`YawEnuToOxyzDeg` 固化回模块：

```bash
rosrun terrain_map_builder map_module.py configure dune_b --origin-lat 22.000000 --origin-lon 114.000000 --origin-alt 0.0 --yaw-enu-to-oxyz-deg 0.0
```

`module.yaml` 的坐标参数优先于 `full_system.launch` 的同名默认参数，因而切换
模块时不会误用上一张地图的原点。

## 3. 采集与离线建图

```bash
roslaunch vehicle_bringup record_mapping_bag.launch map_module:="$MAP_MODULE"
rosrun terrain_map_builder check_mapping_bag.py "bags/${MAP_MODULE}"/mapping_run_*.bag
roslaunch vehicle_bringup bag_mapping.launch
rosbag record -O "bags/${MAP_MODULE}/lio_result.bag" /lio_sam/mapping/path /lio_sam/mapping/odometry
rosbag play --clock "bags/${MAP_MODULE}"/mapping_run_*.bag
rosrun terrain_map_builder map_module.py save "$MAP_MODULE" --resolution 0.2
```

`save` 子命令调用 `/lio_sam/save_map`，自动把 PCD 写入当前模块的 `raw/`，避免
手工路径写错而覆盖另一张地图。

## 4. 地图对齐与规划图层

```bash
rosrun terrain_map_builder align_qianxun_lio.py "bags/${MAP_MODULE}"/mapping_run_*.bag "bags/${MAP_MODULE}/lio_result.bag" --output "maps/${MAP_MODULE}/aligned/qianxun_lio_alignment.yaml"
roslaunch terrain_map_builder align_lio_map.launch map_module:="$MAP_MODULE" translation_x:=【脚本输出】 translation_y:=【脚本输出】 yaw_deg:=【脚本输出】
roslaunch terrain_map_builder build_planner_map.launch map_module:="$MAP_MODULE"
rosrun terrain_map_builder map_module.py validate "$MAP_MODULE" --stage qianxun
```

若要使用固定地图 LIO-SAM/NDT 定位，额外执行：

```bash
rosrun terrain_map_builder map_module.py validate "$MAP_MODULE" --stage lio_sam
```

## 5. 直接切换地图导航

千寻模式：

```bash
roslaunch vehicle_bringup full_system.launch map_module:=dune_a localization_mode:=qianxun
```

换到另一张地图只修改一个参数：

```bash
roslaunch vehicle_bringup full_system.launch map_module:=dune_b localization_mode:=qianxun
```

LIO-SAM/NDT 模式：

```bash
roslaunch vehicle_bringup full_system.launch map_module:=dune_a localization_mode:=lio_sam
```

当前激活模块与文件指纹会锁存发布到 `/map_module/active`：

```bash
rostopic echo -n 1 /map_module/active
```

## 6. 评估隔离与地图指纹

`full_system.launch` 默认启动评估器，输出路径自动按地图和定位模式隔离：

```text
results/dune_a/qianxun/nav_1/
results/dune_a/lio_sam/nav_1/
results/dune_b/qianxun/nav_1/
```

单独启动评估器时同样只需指定地图模块；地图目录和结果目录会自动推导：

```bash
roslaunch navigation_evaluation evaluation.launch map_module:=dune_a localization_mode:=qianxun
```

每轮运行的 `csv_manifest.json`、`evaluation_metrics.json`、
`pareto_decision.json` 和 `run_summary.csv` 都会记录：

- `map_module`；
- `localization_mode`；
- 规划图层和模块配置的 SHA-256 地图指纹；
- 实际读取的地图文件路径、大小和摘要算法。

因此同名地图重新生成后会产生不同指纹，历史评估不会被静默混入新地图结果。
查询当前模块指纹：

```bash
rosrun terrain_map_builder map_module.py fingerprint "$MAP_MODULE"
```

SLAM/EVO 评估也应写入对应模块：

```bash
rosrun slam_evaluation odom_to_tum.py "bags/${MAP_MODULE}/lio_result.bag" "results/${MAP_MODULE}/slam/lio.tum" --topic /lio_sam/mapping/odometry
rosrun slam_evaluation evaluate_trajectories.py "results/${MAP_MODULE}/slam/ground_truth.tum" "results/${MAP_MODULE}/slam/lio.tum" "results/${MAP_MODULE}/slam/evo"
