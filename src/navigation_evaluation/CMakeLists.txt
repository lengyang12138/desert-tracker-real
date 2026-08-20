cmake_minimum_required(VERSION 3.0.2)
project(navigation_evaluation)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs geometry_msgs move_base_msgs nav_msgs rospy sensor_msgs std_msgs tf
  visualization_msgs
)
catkin_package()

catkin_install_python(PROGRAMS nodes/real_nav_evaluator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
