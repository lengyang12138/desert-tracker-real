cmake_minimum_required(VERSION 3.0.2)
project(fixed_map_localization)

add_compile_options(-std=c++14 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs tf2 tf2_ros
)
find_package(PCL REQUIRED COMPONENTS common filters io registration search)
catkin_package(CATKIN_DEPENDS
  geometry_msgs nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs tf2 tf2_ros
)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(fixed_map_localizer src/fixed_map_localizer.cpp)
target_link_libraries(fixed_map_localizer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS fixed_map_localizer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
