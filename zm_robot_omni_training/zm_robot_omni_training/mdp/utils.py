# SPDX-License-Identifier: Apache-2.0
#
# Ported (logic-equivalent, no COMPASS import) from
# mobility_es/mdp/utils.py:goal_pose_in_robot_frame.

from __future__ import annotations

import torch
import warp as wp
from typing import TYPE_CHECKING

import isaaclab.utils.math as math_utils
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def goal_pose_in_robot_frame(
    env: ManagerBasedRLEnv,
    command_name: str = "goal_pose",
    offset: tuple[float, float] = (-0, 0),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Get the goal pose in the robot frame with offset.

    env.command_manager.get_command(command_name)[:, :2] is already in the robot's
    local frame (isaaclab's built-in UniformPose2dCommand does that transform); this
    only applies an additional constant offset, rotated into the robot's current yaw.
    """
    goal_pose = env.command_manager.get_command(command_name)[:, :2]

    robot = env.scene[robot_cfg.name]
    robot_yaw = math_utils.euler_xyz_from_quat(wp.to_torch(robot.data.root_quat_w))[2].reshape(
        (goal_pose.shape[0], 1))
    offset_vec = torch.cat([
        offset[0] * torch.cos(robot_yaw) - offset[1] * torch.sin(robot_yaw),
        offset[0] * torch.sin(robot_yaw) + offset[1] * torch.cos(robot_yaw)
    ],
                           dim=1)
    return goal_pose - offset_vec
