# SPDX-License-Identifier: Apache-2.0
#
# Ported (logic-equivalent, no COMPASS import) from mobility_es/mdp/termination.py.

from __future__ import annotations

import torch
import warp as wp

from isaaclab.assets import RigidObject
from isaaclab.envs import mdp
from isaaclab.managers import SceneEntityCfg

from zm_robot_omni_training.mdp.utils import goal_pose_in_robot_frame
from zm_robot_omni_training.wrapper.env_wrapper import ZmRobotEnvWrapper


def goal_reached(
    env: ZmRobotEnvWrapper,
    dist_threshold: float = 0.5,
    linear_vel_threshold: float = 0.1,
    angular_vel_threshold: float = 0.1,
    heading_threshold: float = 0.5,    # was 0.1 rad (~6 deg): random exploration almost never hit it
    command_name: str = 'goal_pose',
    offset: tuple[float, float] = (-0.0, 0),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    robot: RigidObject = env.scene[robot_cfg.name]

    root_linear_vel = torch.norm(wp.to_torch(robot.data.root_lin_vel_w), dim=1)
    root_angular_vel = wp.to_torch(robot.data.root_ang_vel_w)[:, 2].abs()
    vel_threshold = torch.logical_and(root_linear_vel < linear_vel_threshold,
                                      root_angular_vel < angular_vel_threshold)

    goal_pose = goal_pose_in_robot_frame(env, command_name, offset, robot_cfg)
    goal_pose_abs = torch.norm(goal_pose, p=2, dim=1)
    pose_vel_threshold = torch.logical_and(goal_pose_abs < dist_threshold, vel_threshold)

    goal_heading = env.command_manager.get_command(command_name)[:, 3].abs()
    return torch.logical_and(goal_heading < heading_threshold, pose_vel_threshold)


def nan_pose(env: ZmRobotEnvWrapper, robot_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """Terminate if the robot pose is NaN (can happen if the robot explodes in sim)."""
    nan_pos = torch.isnan(wp.to_torch(env.scene[robot_cfg.name].data.root_pos_w))
    nan_quat = torch.isnan(wp.to_torch(env.scene[robot_cfg.name].data.root_quat_w))
    return torch.logical_or(torch.any(nan_pos, dim=1), torch.any(nan_quat, dim=1))


def illegal_contact_after_grace(env: ZmRobotEnvWrapper,
                                threshold: float,
                                sensor_cfg: SceneEntityCfg,
                                grace_period_steps: int = 5) -> torch.Tensor:
    """Same as isaaclab.envs.mdp.illegal_contact, but ignores contact for the first
    few steps after a reset.

    HolonomicPerfectControlAction teleports the root pose/velocity kinematically
    instead of driving the wheel joints through physics, so PhysX sees a
    discontinuous jump right after reset and resolves it as a transient
    multi-hundred-thousand-newton "penetration" impulse on base_link for 1-2 steps -
    nothing actually collided. Without this grace period, nearly every episode was
    terminating as "base_contact" within its first couple of steps (confirmed via a
    direct force-magnitude trace: ~1e6 N at step 1, ~1.3e5 N at step 2, 0 N from
    step 3 onward), making the reward curve meaningless.
    """
    illegal = mdp.illegal_contact(env, threshold, sensor_cfg)
    within_grace = env.episode_length_buf < grace_period_steps
    return illegal & ~within_grace
