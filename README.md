# desert-tracker-real

面向真实履带车辆的地形感知自主导航工程，包含千寻 GNSS 与 YIS525 IMU
融合定位、LIO-SAM 建图/可选定位、地形 Hybrid A* 全局规划、履带车 MPC
轨迹跟踪以及 ZMQ–SocketCAN 实车接口。

更详细的工程边界、接口表、TF 所有权和算法替换点见
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)，多地图管理见
[`docs/MULTI_MAP_MODULES.md`](docs/MULTI_MAP_MODULES.md)。

> 当前阶段以低速实车试验和问题逐项验证为目标。软件安全门控不能替代实体急停，
> `PoliBrake` 当前只会强制速度与角速度归零，尚未被确认是车辆的物理制动协议。

## 系统架构

```text
千寻 GNSS + YIS525 IMU
          |
          v
vehicle_interface（定位融合与实车接口）
          |
          +--> /vehicle/state + map/odom/base_link TF
                                  |
                                  v
                       地形 Hybrid A* 全局规划
                                  |
                                  v
                         履带车 MPC 轨迹跟踪
                                  |
                              /cmd_vel
                                  |
                                  v
                 PoliAcc / PoliSteer / PoliBrake
                                  |
                                  v
                        SocketCAN 0x200 / 0x203
```

系统支持两种互斥定位模式：

- `qianxun`：千寻/YIS525 融合定位负责 `map -> odom -> base_link`；
- `lio_sam`：固定地图定位器负责 `map -> odom`，LIO-SAM 负责
  `odom -> base_link`。

两种模式共享同一个 YIS525 驱动和 `/imu/data`，不会重复打开 IMU 串口。

## Catkin 软件包

| 软件包 | 作用 |
|---|---|
| `lio_sam`（`src/LIO-SAM`） | 修改后的 LIO-SAM，支持切换 TF 所有权 |
| `terrain_map_builder` | 对齐 LIO-SAM PCD，并生成占据、DEM 和 TCM1 地图 |
| `navigation` | 地形 Hybrid A* 和履带车 MPC 插件 |
| `vehicle_interface` | 千寻/YIS525 融合、ROS 状态适配和 ZMQ-CAN 桥 |
| `pointcloud_preprocess` | 将 RSHELIOS 点云转换为 LIO-SAM 接口 |
| `fixed_map_localization` | 基于固定 PCD 的 NDT 定位修正 |
| `navigation_evaluation` | 实车导航 CSV、JSON 和图表评估 |
| `slam_evaluation` | TUM 轨迹导出和 EVO APE/RPE 评估 |
| `vehicle_bringup` | 实车启动入口和安全配置 |

项目不包含 Gazebo、车辆网格、仅用于仿真的 RViz 包及历史仿真日志。

## 运行环境与编译

目标环境：Ubuntu 20.04、ROS Noetic。

```bash
cd <desert-tracker-real所在目录>
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
python3 -m pip install pyserial pyzmq
```

还需要安装 `move_base`、`map_server`、PCL ROS 和 Grid Map 等 ROS 软件包。

运行不依赖 ROS 的合同测试：

```bash
python3 -m unittest discover -s test -v
```

## 多地图目录

每张地图都是一个相互独立的模块：

```text
maps/<map_module>/
  module.yaml                    # 固定GNSS原点和地图轴旋转
  raw/GlobalMap.pcd              # LIO-SAM原始地图
  aligned/GlobalMap.pcd          # 对齐到导航坐标系的地图
  planner/traversability_map.yaml
  planner/traversability_map.pgm
  planner/elevation_map.dem
  planner/obstacle_map.pgm
  planner/terrain_cost.bin
  planner/terrain_cost_coarse.bin

bags/<map_module>/               # 建图与定位bag
results/<map_module>/            # 导航和SLAM评估结果
```

仓库已预留 `site_a` 和 `site_b`。默认使用 `site_a`，切换地图只需传递：

```bash
map_module:=site_b
```

也可以新建地图模块：

```bash
rosrun terrain_map_builder map_module.py create site_c \
  --origin-lat 22.000000 \
  --origin-lon 114.000000 \
  --origin-alt 0.0 \
  --yaw-enu-to-oxyz-deg 0.0
```

真实原点和轴旋转必须分别写入对应模块的 `module.yaml`，不能每次启动重新使用
第一帧定位作为原点。

## 建图流程

规划系统读取固定地图目录，不直接读取在线 SLAM 话题。建图与自主驾驶分离，便于
复现和审查：

```text
LIO-SAM save_map -> raw/GlobalMap.pcd
                 -> 刚体对齐到O-XYZ/map
                 -> aligned/GlobalMap.pcd
                 -> traversability_map.yaml/.pgm
                 -> elevation_map.dem
                 -> terrain_cost.bin
                 -> terrain_cost_coarse.bin
```

### 在线采集建图数据

完成串口、雷达模式和初始外参配置后：

```bash
MAP_MODULE="site_a"
roslaunch vehicle_bringup mapping.launch map_module:="${MAP_MODULE}"
```

需要保留可复现数据时，统一录制处理后点云、共享 IMU、千寻状态和 TF：

```bash
roslaunch vehicle_bringup record_mapping_bag.launch map_module:="${MAP_MODULE}"
```

录包节点等待 `/points_raw` 和 `/imu/data` 都有新鲜数据后才启动 rosbag，记录：

```text
/points_raw
/imu/data
/bus/location
/tf
/tf_static
```

`/tf_static` 用于保存采集时的 `base_link -> imu_link` 和
`base_link -> velodyne` 安装关系。离线回放默认使用包内静态 TF，避免重复发布。

### 离线回放与检查

```bash
rosrun terrain_map_builder check_mapping_bag.py \
  "bags/${MAP_MODULE}"/mapping_run_*.bag

roslaunch vehicle_bringup bag_mapping.launch
rosbag play --clock "bags/${MAP_MODULE}"/mapping_run_*.bag
```

由本项目录制的 bag 包含 `/tf_static`。回放不包含静态 TF 的旧包时使用：

```bash
roslaunch vehicle_bringup bag_mapping.launch bag_has_static_tf:=false
```

### 保存、对齐和生成规划地图

```bash
rosrun terrain_map_builder map_module.py save "${MAP_MODULE}" --resolution 0.2

roslaunch terrain_map_builder align_lio_map.launch \
  map_module:="${MAP_MODULE}"

roslaunch terrain_map_builder build_planner_map.launch \
  map_module:="${MAP_MODULE}"
```

如果首次建图有意使用第一帧 GNSS 作为原点，应将输出的 `OriginLat`、
`OriginLon`、`OriginAlt` 和 `YawEnuToOxyzDeg` 固化回模块配置。

自主驾驶前必须在 RViz 中确认：PGM 原点、TCM1 原点、对齐后的 PCD 与千寻控制点
坐标一致。地图看起来合理但原点或 yaw 不同仍然是不安全的。

## 激光雷达与 IMU 接口

```text
RSHELIOS（XYZIRT）
  -> /rslidar_points
  -> ring/time适配器
  -> /points_raw

YIS525 -> /imu/data
LIO-SAM -> GlobalMap.pcd 或定位里程计
```

RSHELIOS 使用 `use_lidar_clock: false`，点云帧时间来自工控机系统时钟，点内
`time` 仅保留扫描内相对时间。YIS525 在同一工控机上使用 ROS 系统时间戳。

安装外参通过启动参数传入：

- `lidar_xyz`、`lidar_ypr`：`base_link -> velodyne`；
- `imu_xyz`、`imu_ypr`：`base_link -> imu_link`；
- `imu_ypr` 和 `lidar_ypr` 的顺序均为 `yaw pitch roll`，单位为弧度。

LIO-SAM 的 `extrinsicTrans`、`extrinsicRot`、`extrinsicRPY` 和
`imuTimeOffset` 仍需根据整车最终安装结果填写，不能直接使用开发板标定值。

## 接口联调（不发送 CAN）

如果原千寻/IMU 进程已经运行，不要重复打开串口：

```bash
roslaunch vehicle_interface io.launch \
  start_sensors:=false \
  start_can:=false

roslaunch vehicle_bringup navigation.launch map_module:=site_a
```

检查：

```text
/odometry/imu_incremental
/imu/data
/vehicle/state
/hybrid_astar/plan
/cmd_vel
map -> odom -> base_link TF
base_link -> imu_link TF
```

## 实车启动

先初始化 SocketCAN：

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
```

千寻定位模式：

```bash
roslaunch vehicle_bringup full_system.launch \
  map_module:=site_a \
  localization_mode:=qianxun
```

LIO-SAM 固定地图定位模式：

```bash
roslaunch vehicle_bringup full_system.launch \
  map_module:=site_a \
  localization_mode:=lio_sam
```

默认实车最高速度为 `0.15 m/s`。正式启动前应完成：

- 实体急停测试；
- CAN 速度、转向方向和三路独立超时测试；
- `/vehicle/state`、TF、定位质量和控制周期 watchdog 测试；
- 地图与定位坐标重合检查；
- MPC 求解超时和上一可行控制回退测试。

普通 `qianxun` 导航模式不会启动 LiDAR，也没有动态 PointCloud2 障碍层。
`lio_sam` 模式使用 LiDAR 做固定地图定位，但同样不向 MPC 提供实时点云避障。

## 千寻与 LIO-SAM 地图对齐

建图期间可以额外记录紧凑的 LIO-SAM 结果：

```bash
rosbag record -O "bags/${MAP_MODULE}/lio_result.bag" \
  /lio_sam/mapping/path \
  /lio_sam/mapping/odometry
```

使用原始时间戳拟合千寻轨迹与 LIO-SAM 轨迹：

```bash
rosrun terrain_map_builder align_qianxun_lio.py \
  "bags/${MAP_MODULE}"/mapping_run_*.bag \
  "bags/${MAP_MODULE}/lio_result.bag" \
  --output "maps/${MAP_MODULE}/aligned/qianxun_lio_alignment.yaml"
```

工具会输出拟合 RMSE 以及 `translation_x`、`translation_y`、`yaw_deg`。
采集轨迹必须包含转弯或闭环，单一直线不足以约束地图 yaw。

## 实车评估

`full_system.launch` 默认启动评估器。每次 `/move_base/goal` 都会创建独立目录：

```text
results/${MAP_MODULE}/qianxun/nav_N/
results/${MAP_MODULE}/lio_sam/nav_N/
```

评估结果包含：

- 全局路径、定位轨迹、控制器里程计；
- CTE、终点误差、速度和角速度指令；
- MPC 诊断、预测时域和求解时间；
- IMU、GNSS 和 CAN 电机反馈；
- 地形姿态和规划器 Pareto 统计；
- 地图模块名、定位模式以及地图文件指纹。

不需要评估时：

```bash
start_evaluator:=false
```

SLAM 精度评估与导航跟踪评估分开执行：

```bash
rosrun slam_evaluation odom_to_tum.py \
  "bags/${MAP_MODULE}/lio_result.bag" \
  "results/${MAP_MODULE}/slam/lio.tum" \
  --topic /lio_sam/mapping/odometry

rosrun slam_evaluation evaluate_trajectories.py \
  "results/${MAP_MODULE}/slam/ground_truth.tum" \
  "results/${MAP_MODULE}/slam/lio.tum" \
  "results/${MAP_MODULE}/slam/evo"
```

## 坐标约定

千寻保留的 O-XYZ 航向角以全局 `+Y` 为零、逆时针为正；ROS yaw 以全局
`+X` 为零。适配器执行：

```text
yaw_ros = wrap(head_oxyz + 90 degrees)
```

适配器不交换 X/Y，因此千寻原点、LIO-SAM 地图和规划地图必须描述同一套
O-XYZ 物理坐标轴。

车辆采用尾部作为逻辑前进方向，符号反转只存在于 SocketCAN 边界。规划器与
MPC 始终使用标准逻辑：`v > 0` 表示前进，`omega > 0` 表示逆时针旋转。

## 安全门控

出现以下任一情况，实车接口会发送零速度、零角速度，并置
`PoliBrake=1` 软件停车请求：

- 融合状态超时；
- `FusionValid`、`PositionValid` 或 `ControlFrameReady` 无效；
- YIS525 航向模式为 `UNKNOWN`；
- `/cmd_vel` 超时。

CAN 桥分别记录 `PoliAcc`、`PoliSteer`、`PoliBrake` 的接收时间。只有三路都
已收到且新鲜时才允许运动；任意一路缺失或超时都会独立触发零速度和零角速度。

当前 `0x200` 报文只确认包含有符号速度和角速度两个 `int16` 字段。
`PoliBrake` 还不是经过验证的物理制动位，只表示将两个运动字段强制清零。
在获得 VCU 协议并完成静止硬件试验前，不能把它描述为主动刹车。

履带底盘允许原地转向，合法的 `v=0, omega!=0` 会继续发送到 CAN。实车启动
不会启用 `--zero-steer-when-stopped`。

MPC 还会检查 `/vehicle/state`、TF、定位质量、位姿协方差和控制周期。候选搜索
限制为 80 ms：单次超时复用上一条完整可行控制，连续三次超时发送零指令。
路径投影使用局部窗口和缓存，只有路径距离失配过大时才进行全局重捕获。

## 注意事项

- 本项目包含真实车辆接口，建议 GitHub 仓库保持 Private；
- 不要提交真实 rosbag、PCD、导航结果、账号令牌或私钥；
- `maps/sample_desert` 仅用于软件和接口检查，不对应任何真实 GNSS 原点；
- 第一次实车运行必须安排安全员并保持实体急停可用。
