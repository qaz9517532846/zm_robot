cmake_minimum_required(VERSION 3.0.2)
project(zm_robot_pid_control)

find_package(catkin REQUIRED COMPONENTS
  gazebo
  geometry_msgs
  joint_state_controller
  nav_msgs
  robot_state_publisher
  roscpp
  std_msgs
  tf
  message_generation
)

add_message_files(
  FILES
  agv_pid_position.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(zm_robot_pid_control SHARED src/zm_robot_gazebo_pid.cc)
target_link_libraries(zm_robot_pid_control ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
