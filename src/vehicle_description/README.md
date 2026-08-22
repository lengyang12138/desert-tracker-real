# 实车RViz模型

该包只负责显示车辆外形，不参与定位、规划、控制或碰撞检测。

- 模型来自原仿真工程 `car_rviz` 的 `base-link-mash.stl`；
- 网格使用5 mm顶点聚类简化，降低RViz加载和Git仓库存储开销；
- URDF仅含现有的`base_link`，不会发布TF；
- 模型已按实车约定重新居中到旋转中心的地面投影，x向前、y向左、z向上；
- 平面尺寸缩放为实测规划footprint的约2.00 m x 1.20 m。

独立启动：

```bash
roslaunch vehicle_description vehicle_visualization.launch
```

随完整导航启动：

```bash
roslaunch vehicle_bringup real_navigation.launch map_module:=site_a localization_mode:=qianxun
```

RViz默认自动启动。使用工具栏中的`2D Nav Goal`在地图上按住并拖动：按下位置
是目标点，箭头方向是目标航向，消息发布到`/move_base_simple/goal`。车辆模型
和`base_link`坐标轴仅跟随真实`map -> odom -> base_link` TF。
