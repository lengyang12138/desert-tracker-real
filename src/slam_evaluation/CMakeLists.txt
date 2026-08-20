cmake_minimum_required(VERSION 3.0.2)
project(slam_evaluation)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  nodes/odom_to_tum.py
  nodes/evaluate_trajectories.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
