cmake_minimum_required(VERSION 3.0.2)
project(Project)
set(CATKIN_TOPLEVEL TRUE)
find_package(catkin QUIET
  NO_POLICY_SCOPE
  PATHS /opt/ros/noetic/share/catkin/cmake
  NO_DEFAULT_PATH
)
if(NOT catkin_FOUND)
  message(FATAL_ERROR "catkin not found; source /opt/ros/noetic/setup.bash")
endif()
include(${catkin_EXTRAS_DIR}/toplevel.cmake)

