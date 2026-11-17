# SPDX-License-Identifier: Apache-2.0
#
# Ported (logic-equivalent, no COMPASS import) from
# mobility_es/mdp/command/{commands_cfg.py,uniform_collision_free_pose_command.py}.
# Collision avoidance degrades to plain uniform sampling since env.collision_checker is
# always "not initialized" here (see wrapper/env_wrapper.py).

from __future__ import annotations

import random
from collections.abc import Sequence

import torch
import warp as wp

from isaaclab.envs.mdp import commands
from isaaclab.markers.config import GREEN_ARROW_X_MARKER_CFG
from isaaclab.utils import configclass

from zm_robot_omni_training.wrapper.env_wrapper import ZmRobotEnvWrapper


class UniformCollisionFreePoseCommand(commands.UniformPose2dCommand):
    """Command generator for a 2D pose (position + heading), with optional reject-sampling
    away from obstacles (see class docstring note above re: collision_checker)."""

    def __init__(self, cfg: "UniformCollisionFreePose2dCommandCfg", env: ZmRobotEnvWrapper):
        super().__init__(cfg, env)
        self.collision_checker = env.collision_checker
        # DESIGN.md section 6.1 item 1: the robot's world-frame heading at the moment
        # each goal is assigned, locked for the rest of that episode. The
        # route_heading_lock reward (mdp/rewards.py) rewards keeping the *current*
        # heading close to this locked value - not the instantaneous bearing to the
        # goal - so any cross-track position error can only be corrected with lateral
        # (vy) motion, not by freely rotating to re-aim at the goal.
        self.locked_heading_w = torch.zeros(self.num_envs, device=self.device)

    def __str__(self) -> str:
        return f"UniformCollisionFreePoseCommand:\n\tCommand dimension: {tuple(self.command.shape[1:])}\n"

    def _resample_command(self, env_ids: Sequence[int]):
        quat = wp.to_torch(self.robot.data.root_quat_w)[env_ids]    # xyzw
        self.locked_heading_w[env_ids] = 2 * torch.atan2(quat[:, 2], quat[:, 3])

        resample_env_ids = env_ids
        num_resample_trials = 0
        while num_resample_trials < self.cfg.max_resample_trial:
            self.pos_command_w[resample_env_ids] = wp.to_torch(
                self.robot.data.root_pos_w)[resample_env_ids]
            r = torch.empty(len(resample_env_ids), device=self.device)
            self.pos_command_w[resample_env_ids,
                               0] += r.uniform_(*self._get_sampling_range(self.cfg.ranges.pos_x))
            self.pos_command_w[resample_env_ids,
                               1] += r.uniform_(*self._get_sampling_range(self.cfg.ranges.pos_y))
            self.pos_command_w[resample_env_ids, 2] = 0.0
            self.heading_command_w[resample_env_ids] = r.uniform_(*self.cfg.ranges.heading)

            resample_env_ids = self._check_collision(resample_env_ids)
            num_resample_trials += 1
            if len(resample_env_ids) < 1:
                break

        if num_resample_trials >= self.cfg.max_resample_trial:
            print(f"Envs with id: {resample_env_ids} failed to sample collision free "
                  f"poses within the maximum iteration: {self.cfg.max_resample_trial}")

    def _get_sampling_range(self, input_range: tuple[float, float]) -> tuple[float, float]:
        if not self.cfg.minimum_distance or random.random() > self.cfg.minimum_distance_prob:
            return input_range
        low, high = input_range
        if low > -self.cfg.minimum_distance and high < self.cfg.minimum_distance:
            raise ValueError('Sampling range is too small for minimum distance.')

        sampling_ranges = []
        if low < -self.cfg.minimum_distance:
            sampling_ranges.append((low, -self.cfg.minimum_distance))
        if high > self.cfg.minimum_distance:
            sampling_ranges.append((self.cfg.minimum_distance, high))
        return random.choice(sampling_ranges)

    def _check_collision(self, env_ids: Sequence[int]):
        """Return the env_ids whose sampled goal must be re-sampled: goal inside/too close
        to an obstacle, or closer than cfg.min_goal_distance to the robot (otherwise a
        stationary robot can 'reach' the goal at spawn - see scripts/diag_spawn.py)."""
        bad = torch.zeros(len(env_ids), dtype=torch.bool, device=self.device)
        if self.cfg.min_goal_distance:
            robot_xy = wp.to_torch(self.robot.data.root_pos_w)[env_ids, :2]
            bad |= torch.norm(self.pos_command_w[env_ids, :2] - robot_xy,
                              dim=1) < self.cfg.min_goal_distance
        if self.collision_checker.is_initialized():
            sampled_points_r = self.pos_command_w[env_ids, :2]
            if self._env.scene.cfg.environment.prim_path.startswith('/World/envs/'):
                sampled_points_r = sampled_points_r - self._env.scene.env_origins[env_ids, :2]
            bad |= self.collision_checker.is_in_collision(sampled_points_r, distance=0.5) == 1
        return env_ids[torch.where(bad)[0]]


@configclass
class UniformCollisionFreePose2dCommandCfg(commands.UniformPose2dCommandCfg):
    """Config for UniformCollisionFreePoseCommand."""
    class_type: type = UniformCollisionFreePoseCommand

    max_resample_trial: int = 100
    minimum_distance: float = None
    minimum_distance_prob: float = None
    min_goal_distance: float = 1.0    # hard floor on robot->goal distance (metres)

    goal_pose_visualizer_cfg = GREEN_ARROW_X_MARKER_CFG.replace(
        prim_path="/Visuals/Command/pose_goal")
    goal_pose_visualizer_cfg.markers["arrow"].scale = (0.1, 0.1, 0.2)
