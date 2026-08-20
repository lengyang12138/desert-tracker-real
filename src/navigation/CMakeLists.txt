cmake_minimum_required(VERSION 3.0.2)
project(navigation)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_global_path_planning mpc_local_planner
  CATKIN_DEPENDS costmap_2d geometry_msgs nav_core nav_msgs pluginlib roscpp
                 sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
                 visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(my_global_path_planning src/my_Global_path_planning.cpp)
target_link_libraries(my_global_path_planning ${catkin_LIBRARIES})

add_library(mpc_local_planner src/mpc_local_planner.cpp)
target_link_libraries(mpc_local_planner ${catkin_LIBRARIES})

install(TARGETS my_global_path_planning mpc_local_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/navigation/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/navigation
)
install(FILES my_global_planner_plugin.xml mpc_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
