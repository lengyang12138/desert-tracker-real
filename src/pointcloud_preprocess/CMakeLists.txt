cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_preprocess)

add_compile_options(-std=c++14 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
catkin_package(CATKIN_DEPENDS roscpp sensor_msgs)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(rs_to_lio_sam src/rs_to_lio_sam.cpp)
target_link_libraries(rs_to_lio_sam ${catkin_LIBRARIES})

install(TARGETS rs_to_lio_sam
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
