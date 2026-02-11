import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """
    Launch file which brings up visual slam node configured for RealSense RGBD.

    This configuration uses aligned depth for pixel-wise correspondence with color images.
    Hardware sync is enabled to improve synchronization between color and depth streams.
    """

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'use_sim_time': True,
            'tracking_mode': 2,  # RGBD mode
            'depth_scale_factor': 1000.0,
            'enable_image_denoising': False,
            'rectified_images': False,
            'image_jitter_threshold_ms': 20.00,  # Increased for aligned depth latency
            'sync_matching_threshold_ms': 10.0,  # Allow some sync tolerance
            'base_frame': 'base_link',
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'enable_ground_constraint_in_odometry': False,
            'enable_ground_constraint_in_slam': False,
            'enable_localization_n_mapping': True,
            'min_num_images': 1,
            'num_cameras': 1,
            'depth_camera_id': 0,
            'camera_optical_frames': [
                'kinect_v2'
            ],
        }],
        remappings=[
            ('visual_slam/image_0', '/kinect_v2/color/image_raw'),
            ('visual_slam/camera_info_0', '/kinect_v2/color/camera_info'),
            ('visual_slam/depth_0', '/kinect_v2/depth/image_data'),
        ],
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return launch.LaunchDescription([
        visual_slam_launch_container,
        Node(
            package='zm_robot_detect',
            executable='Image_float2uint',
            name='Image_float2uint'
        )
        ])