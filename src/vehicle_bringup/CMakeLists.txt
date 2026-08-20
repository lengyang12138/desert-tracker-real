cmake_minimum_required(VERSION 3.0.2)
project(vehicle_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(mapping_bag_recorder src/mapping_bag_recorder.cpp)
target_link_libraries(mapping_bag_recorder ${catkin_LIBRARIES})

install(TARGETS mapping_bag_recorder
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
