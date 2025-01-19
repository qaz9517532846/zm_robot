import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import xacro
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # moveit_cpp.yaml is passed by filename for now since it's node specific

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'),  
                    '/gz_sim.launch.py']),
                    launch_arguments={'gz_args': 'empty.sdf'}.items(),
             )

    zm_robot_description_path = os.path.join(
        get_package_share_directory('zm_robot_description'))

    rviz_config_dir = os.path.join(
        get_package_share_directory('zm_robot_description'),
        'config',
        'zm_robot_demo.rviz'
    )

    xacro_file = os.path.join(zm_robot_description_path,
                              'urdf',
                              'zm_robot.urdf.xacro')

    urdf_file = os.path.join(zm_robot_description_path,
                              'urdf',
                              'zm_robot.urdf')

    xacro_to_urdf = ExecuteProcess(
        cmd=['xacro', xacro_file, '-o', urdf_file],
        output='screen',
        name='xacro_to_urdf'
    )

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    node_joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
        )

    spawn_entity = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([os.path.join(
                            get_package_share_directory('ros_gz_sim'), 'launch'),  
                            '/gz_spawn_model.launch.py']),
                            launch_arguments={'world': 'empty',
                                              'file': urdf_file,
                                              'entity_name':'zm_robot',
                                              'x': '0.0',
                                              'y': '0.0',
                                              'z': '0.0'}.items(),
                    )

    display_rviz = Node(package='rviz2', executable='rviz2',
                        name='rviz2',
                        arguments=['-d', rviz_config_dir],
                        output='screen')

    return LaunchDescription([
      xacro_to_urdf,
      gazebo,
      ###node_joint_state_publisher,
      #node_robot_state_publisher,
      spawn_entity,
      ###display_rviz
    ])
