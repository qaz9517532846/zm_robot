cmake_minimum_required(VERSION 3.0.2)
project(zm_robot_move_relative)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  dynamic_reconfigure
  geometry_msgs
  move_base_msgs
  roscpp
  tf
  tf2_ros
)

generate_dynamic_reconfigure_options(
  cfg/zm_robot_move_relative.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    actionlib
    dynamic_reconfigure
    geometry_msgs
    move_base_msgs
    roscpp
    tf
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(zm_robot_move_relative src/zm_robot_move_relative_main.cpp src/zm_robot_move_relative.cpp)
target_link_libraries(zm_robot_move_relative ${catkin_LIBRARIES})
add_dependencies(zm_robot_move_relative zm_robot_move_relative_gencfg)

add_executable(zm_robot_move_relative_client src/zm_robot_move_relative_client.cpp)
target_link_libraries(zm_robot_move_relative_client ${catkin_LIBRARIES})