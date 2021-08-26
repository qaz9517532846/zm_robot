import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rviz_config_dir = os.path.join(
        get_package_share_directory('zm_robot_description'),
        'config',
        'zm_robot_demo.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(package='rviz2', executable='rviz2',
                        name='rviz2',
                        arguments=['-d', rviz_config_dir],
                        output='screen')
    ])