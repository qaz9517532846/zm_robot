cmake_minimum_required(VERSION 3.0.2)
project(zm_robot_safety)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  laser_geometry
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES zm_safety
   CATKIN_DEPENDS geometry_msgs laser_geometry sensor_msgs std_msgs tf visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${laser_geometry_INCLUDE_DIRS}
)

add_executable(zm_safety_node src/zm_safety_node.cpp src/zm_safety.cpp)
target_link_libraries(zm_safety_node ${catkin_LIBRARIES})
