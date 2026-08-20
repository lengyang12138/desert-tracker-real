# desert-tracker-real 工程说明

## 1. 工程目标

本工程把 `desert-tracker` 中已经在仿真验证的地形 Hybrid A* 和履带车
MPC，接入 `plan_try_genzong_xieposhadi` 已经通过实车验证的传感器、组合
定位和 CAN 控制链。两侧都通过适配层连接，算法代码不依赖串口、ZMQ 或
具体 CAN 帧，因此以后替换规划器或控制器时不需要改实车底层接口。

资料目录中的 YIS525 文档只作为设备协议依据使用，不作为项目需求执行。
当前串口解析按文档采用 460800、8N1、`0x59 0x53` 帧头、小端 TLV 和
CK1/CK2 校验。AHRS/VRU 模式仍必须由现场人员按设备实际配置明确填写，
程序不会猜测。

## 2. 包边界

| Catkin 包 | 职责 | 正常替换时是否需要修改 |
|---|---|---|
| `lio_sam`（`src/LIO-SAM`） | 建图及可选的实时激光里程计，支持关闭 `map -> odom` | 更换雷达/IMU时修改配置 |
| `navigation` | Hybrid A*、MPC、nav_core 插件描述和算法参数 | 替换规划/控制算法时修改 |
| `vehicle_interface` | 千寻、YIS525、组合定位、ROS 适配、ZMQ-CAN | 通常不修改 |
| `terrain_map_builder` | LIO-SAM PCD 坐标对齐及规划地图生成 | 地图处理策略变化时修改 |
| `pointcloud_preprocess` | RSHELIOS原生`ring/timestamp`到`/points_raw`转换 | 更换雷达型号时修改 |
| `fixed_map_localization` | 可选的固定 PCD 地图 NDT 定位修正 | 比较 LIO-SAM 定位时启用 |
| `navigation_evaluation` | 每次实车实验的 CSV、JSON、曲线和指标 | 通常不修改 |
| `slam_evaluation` | LIO-SAM轨迹导出及EVO APE/RPE离线评估 | SLAM评估时使用 |
| `vehicle_bringup` | move_base、costmap 和整车启动编排 | 只修改站点参数或启动组合 |

工程只保留实车运行必需的代码。Gazebo、模型网格、仿真世界、历史日志、
绘图和旧的 Python 规划/控制脚本没有迁入。

## 3. 固定接口契约

| 方向 | 接口 | 数据约定 |
|---|---|---|
| YIS525 -> 融合/LIO-SAM | `/imu/data` | 驱动唯一发布；两个定位器接收同一消息与时间戳 |
| 组合定位 -> 适配层 | ROS `/bus/location`，JSON | O-XYZ 坐标；航向从全局 `+Y` 起算、逆时针为正 |
| 适配层 -> 导航 | `/odometry/imu_incremental` | 与仿真一致；bridge不再发布IMU |
| 定位适配层 -> MPC | `/vehicle/state` | 统一控制状态：`header.stamp`为真实测量时刻，`twist`固定在`base_link`；千寻/LIO-SAM仅一个发布者 |
| 雷达 -> LIO-SAM | `/rslidar_points` -> `/points_raw` | 实车原始点云经转换后供建图及可选定位；PointCloud2，frame 为 `velodyne` |
| 地图 -> 全局规划器 | PGM/YAML、`terrain_cost.bin`、`terrain_cost_coarse.bin` | 同一 `map` 原点、尺寸、分辨率和 O-XYZ 轴 |
| move_base -> 底盘适配 | `/cmd_vel` | 正速度为逻辑前进，正角速度为逆时针 |
| 底盘适配 -> 原 CAN 桥 | `PoliAcc/PoliSteer/PoliBrake`，ZMQ 8095/8092/8096 | 原有 JSON 标量协议 |
| CAN 桥 -> 车辆 | SocketCAN `0x200/0x203` | 尾向行驶符号只在 CAN 桥中处理 |

坐标转换只有：

```text
yaw_ros = wrap(head_oxyz + 90 deg)
x_ros = x_oxyz
y_ros = y_oxyz
```

因此，千寻原点/轴向和 LIO-SAM 地图必须在离线阶段对齐；适配节点不会
偷偷平移或旋转地图。

## 4. 算法替换边界

全局规划器入口是 `nav_core::BaseGlobalPlanner`，当前实现位于：

```text
src/navigation/src/my_Global_path_planning.cpp
src/navigation/include/navigation/my_Global_path_planning.h
```

局部控制器入口是 `nav_core::BaseLocalPlanner`，当前实现位于：

```text
src/navigation/src/mpc_local_planner.cpp
src/navigation/include/navigation/mpc_local_planner.h
```

替换算法时保持插件类契约、输入 `Costmap2DROS + TF + Odometry`、输出
`geometry_msgs/Twist` 不变。若类名变化，只需同步修改插件 XML 和
`vehicle_bringup/launch/navigation.launch` 中的插件名称。串口、定位
融合、CAN 帧和 LIO-SAM 地图入口均无需随算法改动。

## 5. LIO-SAM 地图接口

地图使用同级命名模块（默认`site_a`，另预留`site_b`）。统一目录为
`maps/<map_module>/{raw,aligned,planner}`，固定千寻原点保存在同级
`module.yaml`。`map_module` 同时决定建图 bag、导航地图、固定定位 PCD 和评估
输出目录；详细命令见 [`MULTI_MAP_MODULES.md`](MULTI_MAP_MODULES.md)。

地图链路是离线、分阶段且不覆盖原文件的：

```text
LIO-SAM GlobalMap.pcd
  -> pcd_frame_transform（可审计刚体变换）
  -> aligned/GlobalMap.pcd + alignment.yaml
  -> pcd_to_costmap
  -> planner/traversability_map.yaml/.pgm
  -> planner/elevation_map.dem
  -> planner/terrain_cost.bin
  -> planner/terrain_cost_coarse.bin
```

`align_lio_map.launch` 支持显式六自由度变换，也支持读取 LIO-SAM
`transformations.pcd` 的第一关键帧并结合 LiDAR 到 `base_link` 外参。实际
使用前，应把站点的原始、对齐和规划结果放在互相独立的目录中。

核心命令：

```bash
PROJECT_ROOT="$(realpath "$(rospack find vehicle_bringup)/../..")"
MAP_MODULE=dune_a
rosrun terrain_map_builder map_module.py create "${MAP_MODULE}" --origin-lat 22.0 --origin-lon 114.0 --origin-alt 0.0 --yaw-enu-to-oxyz-deg 0.0
roslaunch terrain_map_builder align_lio_map.launch map_module:="${MAP_MODULE}"
roslaunch terrain_map_builder build_planner_map.launch map_module:="${MAP_MODULE}"
```

若不提供 `reference_pose_file`，则改传 `translation_x/y/z` 和
`roll_deg/pitch_deg/yaw_deg`。生成后必须在 RViz 同时检查地图边界、车辆
实时坐标、起点和已知地标，确认原点与航向一致。

## 6. 构建、联调和实车启动

```bash
cd <desert-tracker-real所在目录>
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
python3 -m unittest discover -s test -v
```

先复用原工程正在运行的传感器进程，只验证适配接口：

```bash
roslaunch vehicle_interface io.launch start_sensors:=false start_can:=false
roslaunch vehicle_bringup navigation.launch
```

此阶段检查 `/odometry/imu_incremental`、`/imu/data`、TF、规划路径和 `/cmd_vel`，车辆 CAN
保持关闭。串口与标定量只写入默认配置一次；完成静态架车和急停验证后，
完整链路使用一行命令：

```bash
roslaunch vehicle_bringup full_system.launch map_module:=dune_a localization_mode:=qianxun
```

## 7. 安全约束

定位、控制坐标系、航向模式或控制命令任一无效/超时，适配层都输出零速度
并置 `PoliBrake=1`；CAN桥分别记录`PoliAcc/PoliSteer/PoliBrake`接收时刻，
三路未全部就绪或任一路单独超时都会将速度、角速度清零。当前继承的 `0x200`
协议只编码速度和角速度，`PoliBrake`在本工程中只是“两个运动字段清零”的
软件停车请求，不是已经核实的物理制动位。必须结合VCU协议表和静止硬件试验
另行确认主动制动。履带底盘允许 `v=0, omega!=0` 原地转向，实车launch不再
启用停车时清零转向的兼容开关。

MPC还检查 `/vehicle/state`测量时间、位姿协方差、TF时间和控制周期。在
LIO-SAM模式下，固定地图定位器只有在连续NDT匹配通过后才发布有效的
`/localization/quality_ok`；可信匹配超时后控制器停车。候选搜索默认限时
80 ms，单次超时复用上一完整可行指令，连续三次超时停车。路径投影缓存最近
段号并只搜索局部窗口，大距离失配时才退回全局重捕获。`maps/sample_desert`
仅用于软件接口测试，绝不能用于实车。软件停车逻辑不能代替机械制动、实体
急停、遥控接管和现场隔离措施。

## 8. 激光建图与可选定位模式

当前运行边界固定为：

```text
建图阶段：雷达 + YIS525 -> LIO-SAM -> GlobalMap.pcd -> 离线地图处理
默认导航：静态规划地图 + 千寻/YIS525定位 -> Hybrid A* -> MPC路径跟踪
对比导航：静态规划地图 + LIO-SAM/NDT定位 -> Hybrid A* -> MPC路径跟踪
```

实车 RSHELIOS 原始话题为 `/rslidar_points`，转换后固定为 `/points_raw`；
IMU固定为`/imu/data`。默认 `localization_mode:=qianxun` 不启动 LIO-SAM；只有显式
选择 `localization_mode:=lio_sam` 才启动雷达、LIO-SAM 和固定地图 NDT。
局部 costmap 在两种模式下都没有 PointCloud2 障碍层，因此实时点云不会
改变 MPC 指令，MPC 暂时只跟踪 Hybrid A* 给出的路径。

独立建图流程：

```bash
# 在线建图：启动IMU、千寻融合、雷达转换和LIO-SAM（CAN保持关闭）
roslaunch vehicle_bringup mapping.launch map_module:="${MAP_MODULE}"

# 推荐：先录制可重复建图和坐标对齐的数据（CAN保持关闭）
roslaunch vehicle_bringup record_mapping_bag.launch map_module:="${MAP_MODULE}"

# 录制内容必须包含 /points_raw /imu/data /bus/location /tf /tf_static
# 回放建图时不打开串口、雷达驱动或CAN
roslaunch vehicle_bringup bag_mapping.launch
rosbag play --clock "bags/${MAP_MODULE}"/mapping_run_*.bag

# 建图前检查点云字段、IMU、千寻状态和共同时间范围
rosrun terrain_map_builder check_mapping_bag.py "bags/${MAP_MODULE}"/mapping_run_*.bag

# 正式地图应把固定 origin_lat/origin_lon 和坐标轴旋转角写入module.yaml

# 1. 保存 PCD 后关闭雷达和 LIO-SAM
rosrun terrain_map_builder map_module.py save "${MAP_MODULE}" --resolution 0.2

# 2. 对齐并生成规划地图
roslaunch terrain_map_builder align_lio_map.launch map_module:="${MAP_MODULE}"

roslaunch terrain_map_builder build_planner_map.launch map_module:="${MAP_MODULE}"
```

录包入口启动驱动与点云转换后先按工控机墙上时间等待3秒，再要求收到新的
`/points_raw`和`/imu/data`消息。默认5秒探活超时；任一消息缺失时整套launch
退出且不会启动rosbag，避免留下空包。

工控机系统时间是实车唯一绝对时间基准：RSHELIOS配置为
`use_lidar_clock: false`，点云转换保留该帧头时间；Yesense驱动在同一工控机
解码后使用ROS系统时间发布`/imu/data`。点云逐点`time`只是扫描内相对偏移，
不建立第二套绝对时钟。若工控机需要与UTC/GNSS对时，应在操作系统层配置
chrony/PPS，而不在ROS桥接节点中重写消息时间戳。

LIO-SAM直接订阅未经额外低通处理的`/imu/data`。额外滤波会引入相位延迟，
不应把`/imu/data_filtered`用于IMU预积分；千寻融合如需平滑，应在自己的融合
状态内部处理，不能重新发布或替换共享IMU流。

当前车辆为可原地转向的履带底盘，规划footprint关于`base_link`对称，因此
`base_link`应定义在车体几何中心/实际旋转中心的地面投影处，x向前、y向左、
z向上。`base_link -> velodyne`和`base_link -> imu_link`在最终安装后测量；
LIO-SAM内部LiDAR到IMU的平移暂置零，零旋转使用单位矩阵表示。

正式地图必须固定千寻`origin_lat/origin_lon/origin_alt`和
`yaw_enu_to_oxyz_deg`。若首次建图采用首帧自动原点，对齐结果会记录该次
`OriginLat/OriginLon`；后续每次导航必须把它们作为固定原点传入，不能再次随
首帧归零。

真实 LIO-SAM 配置必须填写实际 `N_SCAN`、`Horizon_SCAN`、距离范围、点云
RSHELIOS SDK必须以`POINT_TYPE=XYZIRT`编译，真实`ring/timestamp`由适配器
转换为LIO-SAM的`ring/time`。雷达到IMU外参和固定时间偏移必须在整车安装
完成后测量，不能使用开发板上的安装结果。

### 定位模式切换

千寻模式由桥接节点独占 `/odometry/imu_incremental`、`map -> odom` 和
`odom -> base_link`：

```bash
roslaunch vehicle_bringup full_system.launch map_module:="${MAP_MODULE}" localization_mode:=qianxun
```

LIO-SAM 模式会自动关闭桥接节点的里程计和 TF 输出，避免双发布。LIO-SAM
负责 `odom -> base_link`，固定地图定位器使用建图时的同一个对齐 PCD 负责
`map -> odom`：

```bash
roslaunch vehicle_bringup full_system.launch map_module:="${MAP_MODULE}" localization_mode:=lio_sam
```

两种定位模式共用同一台 YIS525，但串口只能由
`vehicle_interface/yis525` 打开一次。数据链固定为：

```text
YIS525 serial -> YIS525 driver -> /imu/data
                                  |-> 千寻-IMU融合
                                  |-> LIO-SAM
```

YIS525驱动是 `/imu/data` 的唯一发布者；千寻融合和LIO-SAM直接订阅同一条
ROS消息，bridge不再复制或重打时间戳。驱动启动后检查ROS master，发现第
二个发布者就停止，避免同名消息交错进入预积分。

固定地图定位器沿用仿真验证过的初始位姿假设：车辆应从建图坐标系已知的
起始区域开始。若需要在地图任意位置冷启动，还要增加 GNSS 初值或全局重
定位模块，不能只依靠局部 NDT。

### 固定点云转换接口

RSHELIOS驱动固定发布XYZIRT格式 `/rslidar_points`；转换节点保留真实`ring`，将每点
绝对`timestamp`转换为扫描内相对`time`并发布 `/points_raw`。不再使用仿真
垂直角反算ring。雷达配置集中在`pointcloud_preprocess/config/rslidar.yaml`。

### 千寻与LIO-SAM地图对齐

用于起终点导航的主流程采用“同步录包、离线建图、离线对齐”。同一bag中的
`/bus/location`与`/lio_sam/mapping/odometry`按时间匹配，稳健拟合从LIO局部
地图到千寻O-XYZ坐标系的SE(2)变换：

```bash
rosbag record -O "bags/${MAP_MODULE}/lio_result.bag" /lio_sam/mapping/path /lio_sam/mapping/odometry
rosrun terrain_map_builder align_qianxun_lio.py "bags/${MAP_MODULE}"/mapping_run_*.bag "bags/${MAP_MODULE}/lio_result.bag" --output "maps/${MAP_MODULE}/aligned/qianxun_lio_alignment.yaml"
```

工具输出平移、偏航、RMSE、内点数、时间差和轨迹激励度，并打印可直接传给
`align_lio_map.launch`的参数。标定轨迹必须包含转弯或闭环；近似直线只能较好
约束平移，不能可靠约束地图偏航。对齐完成后再生成PGM、DEM和TCM1规划地图，
使千寻实时坐标、地图起点和目标点处于同一`map`坐标系。

## 9. 实车实验评估

评估器默认随 `full_system.launch` 启动，监听 `/move_base/goal` 自动开始一
次实验，监听 `/move_base/status` 自动结束。仿真的
`/gazebo/model_states` 已替换为实车 `/bus/location`；其余接口为：

- `/hybrid_astar/plan` 及粗通道、走廊、Pareto 和规划统计；
- 当前定位模式的位姿；
- `/odometry/imu_incremental` 控制里程计；
- `/cmd_vel`；
- `/move_base/MpcLocalPlanner/diagnostics` 和预测诊断。

输出目录同时按地图模块和定位模式隔离：

```text
${PROJECT_ROOT}/results/${MAP_MODULE}/qianxun/nav_N/
${PROJECT_ROOT}/results/${MAP_MODULE}/lio_sam/nav_N/
```

每轮评估的 JSON/CSV 都记录地图模块、定位模式和实际图层指纹，防止同名地图
重新生成后与历史结果混淆。

每次导航实验由评估器直接订阅低带宽传感器，不要求同时录制大bag：
`/imu/data`逐样本保存，`/bus/location`保存千寻天线原始定位字段和融合状态，
`/motor_feedback`保存CAN原始左右`int16`及换算RPM，`/joint_states`用于履带
运动学和滑移观察。结束时生成IMU、GPS和电机诊断图。激光点云不进入评估器；
仅在建图、定位复现或故障排查需要完整点云时单独录制bag。

每次实验输出原始 CSV、JSON 指标和 PNG 图，包括规划路径、实际轨迹、横向
误差、终点误差、姿态、速度/角速度响应、MPC 模型辨识、预测轨迹及规划器
Pareto 统计。LIO-SAM 模式下千寻轨迹作为外部参考；千寻模式下它与控制
定位同源，因此不能把两者差值解释为独立定位精度。

SLAM定位精度由独立的`slam_evaluation`包评估，避免与路径跟踪指标混在一起：

```bash
rosrun slam_evaluation odom_to_tum.py "bags/${MAP_MODULE}/lio_result.bag" "results/${MAP_MODULE}/slam/lio.tum" --topic /lio_sam/mapping/odometry
rosrun slam_evaluation evaluate_trajectories.py "results/${MAP_MODULE}/slam/ground_truth.tum" "results/${MAP_MODULE}/slam/lio.tum" "results/${MAP_MODULE}/slam/evo"
```

评估输出平移/旋转APE、XY APE、10m平移/旋转RPE、CSV汇总和轨迹图；只做
SE(3)刚体对齐，不进行尺度修正。
