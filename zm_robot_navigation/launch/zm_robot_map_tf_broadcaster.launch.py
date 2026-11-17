from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package='zm_robot_navigation',
            executable='map_tf_broadcaster_node',
            name='map_tf_broadcaster',
            output='screen',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                'map_frame': LaunchConfiguration('map_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
