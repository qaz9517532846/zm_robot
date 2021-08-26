import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rviz_config_dir = os.path.join(
        get_package_share_directory('zm_robot_description'),
        'config',
        'zm_robot_demo.rviz'
    )

    xacro_file = os.path.join(get_package_share_directory('zm_robot_description'), 'urdf/', 'zm_robot.urdf.xacro')    
    assert os.path.exists(xacro_file), "The box_bot.xacro doesnt exist in "+ str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    print(robot_desc)
    
    # with open(urdf, 'r') as infp:
    #     robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
           
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_desc}],
            output="screen"),

        Node(package='rviz2', executable='rviz2',
                        name='rviz2',
                        arguments=['-d', rviz_config_dir],
                        output='screen')
    ])