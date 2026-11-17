import os

import launch
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lc = LaunchConfiguration
    actions = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('model_dir', default_value='',
                              description='Directory with xmobility.engine and sac_residual_actor_<actor>.engine '
                                          '(empty: $ZM_ROBOT_OMNI_MODEL_DIR or the source tree models/)'),
        DeclareLaunchArgument('actor', default_value='hybrid',
                              description='Residual actor to run: hybrid | penalty | progress'),
        DeclareLaunchArgument('sync_mode', default_value='timer',
                              description='timer: run at control_rate on the latest messages (real robot). '
                                          'stamp: run once per sensor time stamp (simulation / replay)'),
        DeclareLaunchArgument('control_rate', default_value='10.0',
                              description='Hz. The policy was trained with a 0.1 s control period'),
        DeclareLaunchArgument('sensor_qos', default_value='best_effort',
                              description='QoS of the sensor subscriptions: best_effort | reliable'),
        DeclareLaunchArgument('stop_at_goal', default_value='true',
                              description='On reaching the goal (within tolerances) send one zero-velocity command, '
                                          'then stop publishing cmd_vel until a new goal arrives'),
        DeclareLaunchArgument('max_linear_x', default_value='0.6', description='|cmd_vel.linear.x| limit (m/s)'),
        DeclareLaunchArgument('max_linear_y', default_value='0.6', description='|cmd_vel.linear.y| limit (m/s)'),
        DeclareLaunchArgument('max_angular_z', default_value='1.57', description='|cmd_vel.angular.z| limit (rad/s)'),
        DeclareLaunchArgument('max_linear_accel', default_value='2.0',
                              description='Acceleration limit on /cmd_vel, m/s^2 (same as training; 0 = off)'),
        DeclareLaunchArgument('max_angular_accel', default_value='3.14',
                              description='Angular acceleration limit on /cmd_vel, rad/s^2 (same as training)'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('enable_map_tf_broadcaster', default_value='true',
                              description='Broadcast map -> base_link from /odom (fake localization, no SLAM). '
                                          'Turn off when a real map->odom source is running.'),
        DeclareLaunchArgument('rgb_topic', default_value='/kinect_v2/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/kinect_v2/depth/image_raw'),
        DeclareLaunchArgument('scan0_topic', default_value='/sick_lidar0/scan'),
        DeclareLaunchArgument('scan1_topic', default_value='/sick_lidar1/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('goal_pose_topic', default_value='/goal_pose'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        Node(
            package='zm_robot_omni_policy',
            executable='omni_policy_node',
            name='zm_robot_omni_policy',
            output='screen',
            parameters=[{
                'use_sim_time': lc('use_sim_time'),
                'model_dir': lc('model_dir'),
                'actor': lc('actor'),
                'sync_mode': lc('sync_mode'),
                'control_rate': ParameterValue(lc('control_rate'), value_type=float),
                'sensor_qos': lc('sensor_qos'),
                'stop_at_goal': ParameterValue(lc('stop_at_goal'), value_type=bool),
                'max_linear_x': ParameterValue(lc('max_linear_x'), value_type=float),
                'max_linear_y': ParameterValue(lc('max_linear_y'), value_type=float),
                'max_angular_z': ParameterValue(lc('max_angular_z'), value_type=float),
                'max_linear_accel': ParameterValue(lc('max_linear_accel'), value_type=float),
                'max_angular_accel': ParameterValue(lc('max_angular_accel'), value_type=float),
                'base_frame': lc('base_frame'),
            }],
            remappings=[
                ('/kinect_v2/color/image_raw', lc('rgb_topic')),
                ('/kinect_v2/depth/image_raw', lc('depth_topic')),
                ('/sick_lidar0/scan', lc('scan0_topic')),
                ('/sick_lidar1/scan', lc('scan1_topic')),
                ('/odom', lc('odom_topic')),
                ('/goal_pose', lc('goal_pose_topic')),
                ('/cmd_vel', lc('cmd_vel_topic')),
            ],
        ),
    ]
    try:
        nav_share = get_package_share_directory('zm_robot_navigation')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'zm_robot_map_tf_broadcaster.launch.py')),
            launch_arguments={
                'odom_topic': lc('odom_topic'),
                'map_frame': lc('map_frame'),
                'base_frame': lc('base_frame'),
                'use_sim_time': lc('use_sim_time'),
            }.items(),
            condition=IfCondition(lc('enable_map_tf_broadcaster')),
        ))
    except PackageNotFoundError:
        actions.append(LogInfo(msg='zm_robot_navigation not found: map_tf_broadcaster not started; '
                                   'provide TF from the goal frame to base_link yourself.'))
    return launch.LaunchDescription(actions)
