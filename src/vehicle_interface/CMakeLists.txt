cmake_minimum_required(VERSION 3.0.2)
project(vehicle_interface)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  nav_msgs
  rospy
  rosgraph
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  geometry_msgs message_filters nav_msgs rospy rosgraph sensor_msgs std_msgs tf2_ros
)

catkin_install_python(PROGRAMS
  nodes/yis525_node.py
  nodes/qianxun_imu_fusion_node.py
  nodes/real_interface_bridge_node.py
  nodes/lio_vehicle_state_node.py
  nodes/zmq_can_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
