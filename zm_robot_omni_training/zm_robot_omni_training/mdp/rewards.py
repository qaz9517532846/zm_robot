# SPDX-License-Identifier: Apache-2.0
#
# Ported (logic-equivalent, no COMPASS import) from mobility_es/mdp/rewards.py.

from __future__ import annotations

import torch
import warp as wp
from typing import Optional, List

from isaaclab.managers import SceneEntityCfg

from zm_robot_omni_training.mdp.utils import goal_pose_in_robot_frame
from zm_robot_omni_training.wrapper.env_wrapper import ZmRobotEnvWrapper


def goal_reaching(
    env: ZmRobotEnvWrapper,
    command_name: str = "goal_pose",
    offset: tuple[float, float] = (-0, 0),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Reward the agent for moving towards the goal pose."""
    goal_pose = goal_pose_in_robot_frame(env, command_name, offset, robot_cfg)
    goal_pose_abs = -torch.norm(goal_pose, p=2, dim=1).reshape((goal_pose.shape[0], 1))
    return torch.flatten(goal_pose_abs)


def goal_progress(
    env: ZmRobotEnvWrapper,
    command_name: str = "goal_pose",
    offset: tuple[float, float] = (-0, 0),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Potential-based shaping: metres of progress towards the goal made in this step
    (previous distance - current distance; positive = got closer).

    Replaces the old `-distance` reward (goal_reaching): that one only scaled a per-step
    cost by how far away the robot was, so standing still cost little and SAC converged
    to "do nothing" (DESIGN.md section 10 step 8, SAC baseline comparison). Here standing
    still earns exactly 0 and every metre gained earns reward.

    The first step of an episode returns 0 (the stored previous distance still belongs to
    the previous episode of that env).
    """
    goal_pose = goal_pose_in_robot_frame(env, command_name, offset, robot_cfg)
    dist = torch.norm(goal_pose[:, :2], p=2, dim=1)
    prev = getattr(env, "_goal_progress_prev", None)
    if prev is None or prev.shape != dist.shape:
        prev = dist.clone()
    progress = prev - dist
    progress = torch.where(env.episode_length_buf <= 1, torch.zeros_like(progress), progress)
    env._goal_progress_prev = dist.clone()
    # Per-episode cumulative progress (metres), for logging only: reset on an episode's
    # first step, so right after an episode ends (before the next reward call) it still
    # holds that episode's total - train_sac.py's TermStatsCallback reads it there.
    acc = getattr(env, "_ep_progress", None)
    if acc is None or acc.shape != dist.shape:
        acc = torch.zeros_like(dist)
    env._ep_progress = torch.where(env.episode_length_buf <= 1, torch.zeros_like(acc), acc) + progress
    return progress


def goal_heading_alignment(
    env: ZmRobotEnvWrapper,
    command_name: str = "goal_pose",
    offset: tuple[float, float] = (-0.0, 0),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    distance_threshold: float = 1.0
) -> torch.Tensor:
    """Reward aligning with the goal heading as the robot nears the destination.

    DESIGN.md section 6.1 item 2: this is the "final docking heading" reward - only
    matters near the goal (distance_weight decays with distance from it). Distinct
    from, and kept on an independent weight from, the "in-transit heading lock"
    reward (route_heading_lock, section 6.1 item 1).
    """
    goal_pose = goal_pose_in_robot_frame(env, command_name, offset, robot_cfg)
    distance_to_goal = torch.norm(goal_pose[:, :2], p=2, dim=1)

    goal_heading = env.command_manager.get_command(command_name)[:, 3]
    heading_error = goal_heading.abs()

    distance_weight = torch.exp(-distance_to_goal / distance_threshold)
    return -(heading_error * distance_weight)


def goal_heading_progress(
    env: ZmRobotEnvWrapper,
    command_name: str = "goal_pose",
    offset: tuple[float, float] = (-0.0, 0),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    gate_distance: float = 1.0,
) -> torch.Tensor:
    """Radians of goal-heading error removed in this step (previous |error| - current
    |error|; positive = turned towards the goal heading), scaled by a gate that fades
    linearly from 1 at the goal to 0 at `gate_distance`.

    Replaces goal_heading_alignment's per-step penalty -|error| * exp(-d). Measured on
    the exp 7 checkpoints (n=134 / 107 episodes): that penalty summed to -190..-270 per
    timed-out episode vs ~+13 for distance progress, and it shrinks with distance, so the
    150k-step policy had a HIGHER mean episode reward than the 60k one (-114 vs -175)
    while its success rate was lower (10% vs 33%, 95% CIs [6, 18] and [26, 41]): it
    avoided the penalty by finishing farther from the goal. A progress form gives 0 for
    standing still and cannot be earned by staying away (gate is 0 beyond 1 m).

    The first step of an episode returns 0 (the stored previous error belongs to the
    previous episode of that env).
    """
    goal_pose = goal_pose_in_robot_frame(env, command_name, offset, robot_cfg)
    dist = torch.norm(goal_pose[:, :2], p=2, dim=1)
    err = env.command_manager.get_command(command_name)[:, 3].abs()
    prev = getattr(env, "_heading_progress_prev", None)
    if prev is None or prev.shape != err.shape:
        prev = err.clone()
    progress = prev - err
    progress = torch.where(env.episode_length_buf <= 1, torch.zeros_like(progress), progress)
    env._heading_progress_prev = err.clone()
    gate = (1.0 - dist / gate_distance).clamp(0.0, 1.0)
    return progress * gate


def route_heading_lock(
    env: ZmRobotEnvWrapper,
    command_name: str = "goal_pose",
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    distance_threshold: float = 1.0,
) -> torch.Tensor:
    """DESIGN.md section 6.1 item 1 - the key mechanism meant to make vy useful.

    Faded out near the goal (scaled by 1 - exp(-distance/distance_threshold)): it used to
    fight goal_heading_alignment at the goal. Rotating by delta towards the goal heading
    nets ~0.74*delta - 0.5*delta^2 per step, negative for delta > ~1.5 rad, so the
    policy parked at the goal without turning (exp 5, 100k steps: 11/25 episodes ended
    within 0.5 m of the goal, but only 1/25 had heading error < 0.5 rad).

    Rewards keeping the robot's *current* heading close to the heading it had at the
    moment the goal was assigned (UniformCollisionFreePoseCommand.locked_heading_w),
    NOT close to the instantaneous bearing to the goal. A non-holonomic robot would
    satisfy "face the goal" by rotating; this reward specifically does not reward
    that - any cross-track position error has to be corrected with lateral (vy)
    motion instead, since rotating away from the locked heading is penalized.
    """
    robot = env.scene[robot_cfg.name]
    quat = wp.to_torch(robot.data.root_quat_w)
    current_yaw = 2 * torch.atan2(quat[:, 2], quat[:, 3])

    locked_heading = env.command_manager.get_term(command_name).locked_heading_w
    # Wrap the difference into [-pi, pi] before penalizing (angles, not plain floats).
    heading_error = torch.atan2(torch.sin(current_yaw - locked_heading),
                                torch.cos(current_yaw - locked_heading))
    goal_pose = goal_pose_in_robot_frame(env, command_name, (0.0, 0.0), robot_cfg)
    distance = torch.norm(goal_pose[:, :2], p=2, dim=1)
    in_transit = 1.0 - torch.exp(-distance / distance_threshold)
    return -heading_error.pow(2) * in_transit


def masked_action_rate_l2(env: ZmRobotEnvWrapper,
                          idx_masks: Optional[List[int]] = None) -> torch.Tensor:
    """Penalize the rate of change of the actions using an L2 squared kernel."""
    current_action = env.action_manager.action
    prev_action = env.action_manager.prev_action
    if idx_masks is not None:
        current_action = current_action[:, idx_masks]
        prev_action = prev_action[:, idx_masks]
    action_rate = current_action - prev_action
    return torch.sum(action_rate.pow(2), dim=1)


def masked_action_l2(env: ZmRobotEnvWrapper, idx_masks: Optional[List[int]] = None) -> torch.Tensor:
    """Penalize the actions using an L2 squared kernel."""
    current_action = env.action_manager.action
    if idx_masks is not None:
        current_action = current_action[:, idx_masks]
    return torch.sum(current_action.pow(2), dim=1)
