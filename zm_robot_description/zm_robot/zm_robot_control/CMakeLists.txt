cmake_minimum_required(VERSION 3.0.2)
project(zm_robot_control)

find_package(catkin REQUIRED COMPONENTS
  joint_state_controller
  nav_msgs
  robot_state_publisher
  roscpp
  tf
  std_msgs
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(zm_robot_controller_odom src/zm_robot_controller_odom.cpp)
target_link_libraries(zm_robot_controller_odom ${catkin_LIBRARIES})
add_dependencies(zm_robot_controller_odom zm_robot_control_generate_messages_cpp)
