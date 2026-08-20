cmake_minimum_required(VERSION 3.0.2)
project(terrain_map_builder)

add_compile_options(-std=c++14 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS
  grid_map_core
  grid_map_msgs
  grid_map_ros
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
)
find_package(PCL REQUIRED COMPONENTS common filters io search segmentation)

catkin_package(CATKIN_DEPENDS
  grid_map_core grid_map_msgs grid_map_ros nav_msgs pcl_conversions pcl_ros roscpp
)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(pcd_to_costmap src/pcd_to_costmap.cpp)
target_link_libraries(pcd_to_costmap ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcd_frame_transform src/pcd_frame_transform.cpp)
target_link_libraries(pcd_frame_transform ${catkin_LIBRARIES} ${PCL_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/check_mapping_bag.py
  scripts/align_qianxun_lio.py
  scripts/map_module.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS pcd_to_costmap pcd_frame_transform
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
