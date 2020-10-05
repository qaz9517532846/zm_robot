cmake_minimum_required(VERSION 3.0.2)
project(zm_robot_control)

find_package(catkin REQUIRED COMPONENTS
  gazebo
  geometry_msgs
  joint_state_controller
  nav_msgs
  robot_state_publisher
  roscpp
  tf
  std_msgs
)

add_message_files(
  FILES
  agv_position.msg
)

generate_messages(DEPENDENCIES std_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
add_compile_options(-std=c++11)

add_executable(zm_robot_controller_odom src/zm_robot_controller_odom.cpp)
target_link_libraries(zm_robot_controller_odom ${catkin_LIBRARIES})
add_dependencies(zm_robot_controller_odom zm_robot_control_generate_messages_cpp)

add_executable(zm_robot_controller_wheel_vel src/zm_robot_controller_wheel_vel.cpp)
target_link_libraries(zm_robot_controller_wheel_vel ${catkin_LIBRARIES})
add_dependencies(zm_robot_controller_wheel_vel zm_robot_control_generate_messages_cpp)

add_library(zm_robot_control SHARED src/zm_robot_gazebo_controller.cc)
target_link_libraries(zm_robot_control ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
