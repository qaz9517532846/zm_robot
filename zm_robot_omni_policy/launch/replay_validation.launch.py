import launch
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Open-loop deployment check: replay a rollout recorded in the training simulator as ROS2
    topics and compare the policy node's commands with the PyTorch policy's (see README)."""
    lc = LaunchConfiguration
    policy = Node(
        package='zm_robot_omni_policy',
        executable='omni_policy_node',
        name='zm_robot_omni_policy',
        output='screen',
        parameters=[{
            'model_dir': lc('model_dir'),
            'actor': lc('actor'),
            # lockstep replay -> exact per-stamp inference; realtime replay -> the 10 Hz timer path
            'sync_mode': PythonExpression(["'stamp' if '", lc('mode'), "' == 'lockstep' else 'timer'"]),
            'sensor_qos': 'reliable',
            'stop_at_goal': False,    # compare every recorded step, including the last ones
        }],
    )
    replay = Node(
        package='zm_robot_omni_policy',
        executable='replay_validation_node',
        name='zm_robot_omni_replay_validation',
        output='screen',
        parameters=[{
            'ref_path': lc('ref_path'),
            'report_path': lc('report_path'),
            'tolerance': ParameterValue(lc('tolerance'), value_type=float),
            'mode': lc('mode'),
        }],
    )
    return launch.LaunchDescription([
        DeclareLaunchArgument('model_dir', default_value=''),
        DeclareLaunchArgument('actor', default_value='hybrid'),
        DeclareLaunchArgument('ref_path', default_value='',
                              description='Reference rollout .npz (empty: models/validation/ref_hybrid.npz)'),
        DeclareLaunchArgument('report_path', default_value=''),
        DeclareLaunchArgument('mode', default_value='lockstep',
                              description='lockstep: exact step-by-step comparison (policy sync_mode=stamp). '
                                          'realtime: publish at the recorded 10 Hz and smoke-test the timer path'),
        DeclareLaunchArgument('tolerance', default_value='0.02',
                              description='Pass threshold on the max |cmd error| (m/s, rad/s)'),
        policy,
        replay,
        RegisterEventHandler(OnProcessExit(target_action=replay, on_exit=[EmitEvent(event=Shutdown())])),
    ])
