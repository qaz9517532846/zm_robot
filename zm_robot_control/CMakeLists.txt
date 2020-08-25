cmake_minimum_required(VERSION 3.0.2)
project(zm_robot_control)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_msgs
  geometry_msgs
  joint_state_controller
  nav_msgs
  robot_state_publisher
  roscpp
  tf
  std_msgs
)

generate_messages(DEPENDENCIES std_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  gazebo_ros_diff_mecanum_drive)

add_executable(zm_robot_controller_odom src/zm_robot_controller_odom.cpp)
target_link_libraries(zm_robot_controller_odom ${catkin_LIBRARIES})
add_dependencies(zm_robot_controller_odom zm_robot_control_generate_messages_cpp)

add_executable(zm_robot_controller_wheel_vel src/zm_robot_controller_wheel_vel.cpp)
target_link_libraries(zm_robot_controller_wheel_vel ${catkin_LIBRARIES})
add_dependencies(zm_robot_controller_wheel_vel zm_robot_control_generate_messages_cpp)

add_library(gazebo_ros_diff_mecanum_drive src/gazebo_ros_diff_mecanum_drive.cpp)
target_link_libraries(gazebo_ros_diff_mecanum_drive ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
   gazebo_ros_diff_mecanum_drive
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
