# SPDX-License-Identifier: Apache-2.0
#
# Holonomic (mecanum) "perfect control" action term: bypasses wheel/roller joint physics
# entirely and writes the integrated (vx, vy, wz) command straight to the root
# pose/velocity. PhysX has no anisotropic-friction model for mecanum rollers, so driving
# the real 4 wheels + 36 passive roller joints during RL training is not worth the
# instability risk (see DESIGN.md section 9).

from __future__ import annotations

from dataclasses import MISSING

import torch
import warp as wp
from collections.abc import Sequence

from isaaclab.assets.articulation import Articulation
from isaaclab.managers import ActionTerm, ActionTermCfg
from isaaclab.utils import configclass

from zm_robot_omni_training.wrapper.env_wrapper import ZmRobotEnvWrapper


@configclass
class HolonomicPerfectControlActionCfg(ActionTermCfg):
    """Configuration for the holonomic perfect-control action term.

    See :class:`HolonomicPerfectControlAction` for more details.
    """

    class_type: type[ActionTerm] = MISSING
    # Per-step change limits, so a discontinuous policy output doesn't teleport the root
    # instantly - keeps the kinematic shortcut closer to what a real mecanum drivetrain
    # (with actual acceleration limits) can track, and avoids jitter in the smoothness
    # reward terms.
    # 2.0 m/s^2 linear, 3.14 rad/s^2 angular (the dynamics experiments 2-7 were trained
    # under). A tighter 0.8 / 1.57 was tried in experiment 8 and reverted at the user's
    # request: its 150k-step checkpoint was worse than its 60k one (success 0/27 vs 2/27,
    # ending 3.91 m from the goal on average), so checkpoints from experiment 8 are not
    # comparable with 2-7.
    max_linear_accel: float = 2.0
    max_angular_accel: float = 3.14


class HolonomicPerfectControlAction(ActionTerm):
    """Action item for holonomic (mecanum) robot with perfect controller. 3 DOF: vx, vy
    (both in the robot's body frame), wz."""

    cfg: HolonomicPerfectControlActionCfg
    """The articulation asset on which the action term is applied."""
    _asset: Articulation

    def __init__(self, cfg: HolonomicPerfectControlActionCfg, env: ZmRobotEnvWrapper):
        super().__init__(cfg, env)

        self._raw_actions = torch.zeros(self.num_envs, self.action_dim, device=self.device)
        self._processed_actions = torch.zeros_like(self.raw_actions)
        # Rate-limiter state: last (vx, vy, wz) actually applied.
        self._prev_command = torch.zeros(self.num_envs, 3, device=self.device)

    @property
    def action_dim(self) -> int:
        return 3

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions):
        self._raw_actions[:] = actions
        self._processed_actions = self.raw_actions

    def apply_actions(self):
        dt = self._env.physics_dt

        # Rate-limit the commanded (vx, vy, wz) against the last applied command.
        max_dv = self.cfg.max_linear_accel * dt
        max_dw = self.cfg.max_angular_accel * dt
        delta = self.processed_actions - self._prev_command
        delta[:, 0:2] = torch.clamp(delta[:, 0:2], -max_dv, max_dv)
        delta[:, 2] = torch.clamp(delta[:, 2], -max_dw, max_dw)
        command = self._prev_command + delta
        self._prev_command = command

        linear_x = command[:, 0]
        linear_y = command[:, 1]
        angular_z = command[:, 2]

        # Get current position and orientation quaternion (xyzw: x=0, y=1, z=2, w=3)
        pos = wp.to_torch(self._asset.data.root_link_pos_w).clone()
        quat = wp.to_torch(self._asset.data.root_link_quat_w)

        yaw = 2 * torch.atan2(quat[:, 2], quat[:, 3])

        # Update yaw with angular velocity first, then move using the new heading.
        new_yaw = yaw + angular_z * dt

        # Rotate the body-frame (vx, vy) into world frame using the new heading.
        cos_yaw = torch.cos(new_yaw)
        sin_yaw = torch.sin(new_yaw)
        world_vx = linear_x * cos_yaw - linear_y * sin_yaw
        world_vy = linear_x * sin_yaw + linear_y * cos_yaw

        pos[:, 0] += world_vx * dt
        pos[:, 1] += world_vy * dt

        new_quat = torch.zeros_like(quat)
        new_quat[:, 2] = torch.sin(new_yaw / 2)
        new_quat[:, 3] = torch.cos(new_yaw / 2)

        new_pose = torch.cat([pos, new_quat], dim=-1)
        new_vel = torch.stack([
            world_vx,
            world_vy,
            torch.zeros_like(angular_z),
            torch.zeros_like(angular_z),
            torch.zeros_like(angular_z),
            angular_z,
        ],
                              dim=1)
        self._asset.write_root_pose_to_sim_index(root_pose=new_pose)
        self._asset.write_root_velocity_to_sim_index(root_velocity=new_vel)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._raw_actions[env_ids] = 0.0
        self._prev_command[env_ids] = 0.0
