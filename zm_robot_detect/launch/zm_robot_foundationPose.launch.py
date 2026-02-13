import os

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory

# RT-DETR models expect 640x640 encoded image size
RT_DETR_MODEL_INPUT_SIZE = 640
# RT-DETR models expect 3 image channels
RT_DETR_MODEL_NUM_CHANNELS = 3

input_width = 1280
input_height = 720
input_to_RT_DETR_ratio = input_width / RT_DETR_MODEL_INPUT_SIZE

def generate_launch_description():

    pkg_share = get_package_share_directory('zm_robot_detect')

    refine_engine_file_path = os.path.join(
        pkg_share, 'model', 'foundationPose', 'refine_trt_engine.plan')

    score_engine_file_path = os.path.join(
        pkg_share, 'model', 'foundationPose', 'score_trt_engine.plan')

    mesh_file_path = os.path.join(
        pkg_share, 'mesh', 'Mustard', 'textured_simple.obj')

    rt_detr_engine_file_path = os.path.join(
        pkg_share, 'model', 'foundationPose', 'sdetr_grasp.plan')

    refine_model_file_path = os.path.join(
        pkg_share, 'model', 'foundationPose', 'refine_trt_engine.onnx')

    score_model_file_path = os.path.join(
        pkg_share, 'model', 'foundationPose', 'score_trt_engine.onnx')

    mesh_text_path = os.path.join(
        pkg_share, 'mesh', 'Mustard', 'texture_map.png')

    rt_detr_model_file_path = os.path.join(
        pkg_share, 'model', 'foundationPose', 'sdetr_grasp.onnx')

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='foundationpose_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[

            # Resize and pad input images to RT-DETR model input image size
            # Resize from IMAGE_WIDTH x IMAGE_HEIGHT to
            # IMAGE_WIDTH/input_TO_RT_DETR_RATIO x IMAGE_HEIGHT/input_TO_RT_DETR_RATIO
            # output height constraint is not used since keep_aspect_ratio is True
            ComposableNode(
                name='resize_left_rt_detr_node',
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                parameters=[{
                    'input_width': input_width,
                    'input_height': input_height,
                    'output_width': RT_DETR_MODEL_INPUT_SIZE,
                    'output_height': RT_DETR_MODEL_INPUT_SIZE,
                    'keep_aspect_ratio': False,
                    'encoding_desired': 'rgb8',
                    'disable_padding': True
                }],
                remappings=[
                    ('image', '/kinect_v2/color/image_raw'),
                    ('camera_info', '/kinect_v2/color/camera_info'),
                    ('resize/image', 'color_image_resized'),
                    ('resize/camera_info', 'camera_info_resized')
                ]
            ),

            # Convert image to tensor and reshape
            ComposableNode(
                name='image_to_tensor_node',
                package='isaac_ros_tensor_proc',
                plugin='nvidia::isaac_ros::dnn_inference::ImageToTensorNode',
                parameters=[{
                    'scale': False,
                    'tensor_name': 'image',
                }],
                remappings=[
                    ('image', 'color_image_resized'),
                    ('tensor', 'normalized_tensor'),
                ]
            ),

            ComposableNode(
                name='interleaved_to_planar_node',
                package='isaac_ros_tensor_proc',
                plugin='nvidia::isaac_ros::dnn_inference::InterleavedToPlanarNode',
                parameters=[{
                    'input_tensor_shape': [RT_DETR_MODEL_INPUT_SIZE,
                                           RT_DETR_MODEL_INPUT_SIZE,
                                           RT_DETR_MODEL_NUM_CHANNELS]
                }],
                remappings=[
                    ('interleaved_tensor', 'normalized_tensor')
                ]
            ),

            ComposableNode(
                name='reshape_node',
                package='isaac_ros_tensor_proc',
                plugin='nvidia::isaac_ros::dnn_inference::ReshapeNode',
                parameters=[{
                    'output_tensor_name': 'input_tensor',
                    'input_tensor_shape': [RT_DETR_MODEL_NUM_CHANNELS,
                                           RT_DETR_MODEL_INPUT_SIZE,
                                           RT_DETR_MODEL_INPUT_SIZE],
                    'output_tensor_shape': [1, RT_DETR_MODEL_NUM_CHANNELS,
                                            RT_DETR_MODEL_INPUT_SIZE,
                                            RT_DETR_MODEL_INPUT_SIZE]
                }],
                remappings=[
                    ('tensor', 'planar_tensor')
                ],
            ),

            ComposableNode(
                name='rtdetr_preprocessor',
                package='isaac_ros_rtdetr',
                plugin='nvidia::isaac_ros::rtdetr::RtDetrPreprocessorNode',
                remappings=[
                    ('encoded_tensor', 'reshaped_tensor')
                ]
            ),

            # RT-DETR TensorRT
            ComposableNode(
                package='isaac_ros_tensor_rt',
                plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
                name='tensor_rt',
                parameters=[{
                    'engine_file_path': rt_detr_engine_file_path,
                    'model_file_path': rt_detr_model_file_path,
                    'output_binding_names': ['labels', 'boxes', 'scores'],
                    'output_tensor_names': ['labels', 'boxes', 'scores'],
                    'input_tensor_names': ['images', 'orig_target_sizes'],
                    'input_binding_names': ['images', 'orig_target_sizes'],
                    'force_engine_update': False
                }]
            ),

            ComposableNode(
                name='rtdetr_decoder',
                package='isaac_ros_rtdetr',
                plugin='nvidia::isaac_ros::rtdetr::RtDetrDecoderNode',
                parameters=[{
                    'confidence_threshold': 0.5,
                }],
            ),

            # Create a binary segmentation mask from a Detection2DArray published by RT-DETR.
            # The segmentation mask is of size
            # int(IMAGE_WIDTH/input_to_RT_DETR_ratio) x int(IMAGE_HEIGHT/input_to_RT_DETR_ratio)
            ComposableNode(
                name='detection2_d_array_filter',
                package='isaac_ros_foundationpose',
                plugin='nvidia::isaac_ros::foundationpose::Detection2DArrayFilter',
                remappings=[('detection2_d_array', 'detections_output')]
            ),

            ComposableNode(
                name='detection2_d_to_mask',
                package='isaac_ros_foundationpose',
                plugin='nvidia::isaac_ros::foundationpose::Detection2DToMask',
                parameters=[{
                    'mask_width': RT_DETR_MODEL_INPUT_SIZE,
                    'mask_height': RT_DETR_MODEL_INPUT_SIZE,
                }],
                remappings=[('segmentation', 'rt_detr_segmentation')]
            ),

            # Resize segmentation mask to ESS model image size so it can be used by FoundationPose
            # FoundationPose requires depth, rgb image and segmentation mask to be of the same size
            # Resize from int(IMAGE_WIDTH/input_to_RT_DETR_ratio) x
            # int(IMAGE_HEIGHT/input_to_RT_DETR_ratio)
            # to ESS_MODEL_IMAGE_WIDTH x ESS_MODEL_IMAGE_HEIGHT
            # output height constraint is used since keep_aspect_ratio is False
            # and the image is padded
            ComposableNode(
                name='resize_mask_node',
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                parameters=[{
                    'input_width': int(input_width/input_to_RT_DETR_ratio),
                    'input_height': int(input_height/input_to_RT_DETR_ratio),
                    'output_width': input_width,
                    'output_height': input_height,
                    'keep_aspect_ratio': False,
                    'disable_padding': False,
                }],
                remappings=[
                    ('image', 'rt_detr_segmentation'),
                    ('camera_info', 'camera_info_resized'),
                    ('resize/image', 'segmentation'),
                    ('resize/camera_info', 'camera_info_segmentation')
                ]
            ),

            ComposableNode(
                name='foundationpose_node',
                package='isaac_ros_foundationpose',
                plugin='nvidia::isaac_ros::foundationpose::FoundationPoseNode',
                parameters=[{
                    'mesh_file_path': mesh_file_path,

                    'refine_engine_file_path': refine_engine_file_path,
                    'refine_input_tensor_names': ['input_tensor1', 'input_tensor2'],
                    'refine_input_binding_names': ['input1', 'input2'],
                    'refine_output_tensor_names': ['output_tensor1', 'output_tensor2'],
                    'refine_output_binding_names': ['output1', 'output2'],

                    'score_engine_file_path': score_engine_file_path,
                    'score_input_tensor_names': ['input_tensor1', 'input_tensor2'],
                    'score_input_binding_names': ['input1', 'input2'],
                    'score_output_tensor_names': ['output_tensor'],
                    'score_output_binding_names': ['output1'],
                }],
                remappings=[
                    ('pose_estimation/depth_image', '/kinect_v2/depth/image_raw'),
                    ('pose_estimation/segmentation', '/segmentation'),
                    ('pose_estimation/image', '/kinect_v2/color/image_raw'),
                    ('pose_estimation/camera_info', '/kinect_v2/color/camera_info'),
                ]
            ),

            ComposableNode(
                name='selector_node',
                package='isaac_ros_foundationpose',
                plugin='nvidia::isaac_ros::foundationpose::Selector',
                parameters=[{
                    # Expect to reset after the rosbag play complete
                    'reset_period': 65000
                }],
                remappings=[
                    ('image', '/kinect_v2/color/image_raw'),
                    ('camera_info', '/kinect_v2/color/camera_info'),
                    ('depth_image', '/kinect_v2/depth/image_raw'),
                ]
            ),
        ],
        output='screen'
    )

    return launch.LaunchDescription([container])