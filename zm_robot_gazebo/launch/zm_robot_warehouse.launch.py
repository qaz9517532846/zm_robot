# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Get the launch directory
    zm_robot_gazebo_dir = get_package_share_directory('zm_robot_gazebo')
    
    # Launch configuration variables specific to simulation

    world = LaunchConfiguration('world')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(zm_robot_gazebo_dir, 'worlds', 'zm_robot_warehouse.sdf'),
        description='Full path to world model file to load')

    model_path = os.path.join(zm_robot_gazebo_dir, 'models')

    gazebo_server_cmd_line = [
        'gz', 'sim', '-r', '-v4', world]

    gazebo = ExecuteProcess(
        cmd=gazebo_server_cmd_line, output='screen')

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                   '/sick_lidar0@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/sick_lidar1@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/kinect/depth@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/kinect/color@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/kinect/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/model/zm_robot/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                   '/model/zm_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/model/zm_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/world/default/model/zm_robot/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model'],
        remappings=[('/model/zm_robot/cmd_vel', '/cmd_vel'),
                    ('/model/zm_robot/odometry', '/odometry'),
                    ('/world/default/model/zm_robot/joint_state', '/joint_state'),
                    ('/model/zm_robot/pose', '/tf')],
        output='screen')

    
    node_joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path))
    # Add any conditioned actions
    ld.add_action(gazebo)
    ld.add_action(bridge)
    ##ld.add_action(node_joint_state_publisher)

    return ld