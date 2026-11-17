# SPDX-License-Identifier: Apache-2.0
#
# Topic names (all remappable from the launch files) and the layout of the debug topic.

from zm_robot_omni_policy import preprocess as pp

RGB_TOPIC = '/kinect_v2/color/image_raw'
DEPTH_TOPIC = '/kinect_v2/depth/image_raw'
SCAN_TOPICS = ('/sick_lidar0/scan', '/sick_lidar1/scan')
ODOM_TOPIC = '/odom'
GOAL_TOPIC = '/goal_pose'
CMD_TOPIC = '/cmd_vel'
ESTOP_TOPIC = '/emergency_stop'

# Layout of the ~/debug Float32MultiArray (one message per inference).
DEBUG_FIELDS = (['step_in_episode'] + [f'final_{a}' for a in ('vx', 'vy', 'wz')] +
                [f'base_{a}' for a in ('vx', 'vy', 'wz')] + [f'residual_{a}' for a in ('vx', 'vy', 'wz')] +
                ['base_speed', 'base_yaw_rate', 'goal_x', 'goal_y', 'goal_heading'] +
                [f'lidar_{i}' for i in range(pp.NUM_SECTORS)] + ['depth_obs_mean', 'infer_ms'])
