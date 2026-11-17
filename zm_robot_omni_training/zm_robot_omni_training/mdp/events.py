# SPDX-License-Identifier: Apache-2.0
#
# Ported (logic-equivalent, no COMPASS import) from mobility_es/mdp/events.py. Collision
# avoidance degrades to plain uniform sampling since env.collision_checker is always
# "not initialized" here (see wrapper/env_wrapper.py) - same fallback COMPASS itself
# uses for any scene without a registered occupancy map.

from __future__ import annotations

import torch
import warp as wp

import isaaclab.utils.math as math_utils
from isaaclab.managers import SceneEntityCfg

from zm_robot_omni_training.wrapper.env_wrapper import ZmRobotEnvWrapper


def sample_root_state_uniform(env: ZmRobotEnvWrapper,
                              env_ids: torch.Tensor,
                              pose_range: dict[str, tuple[float, float]],
                              velocity_range: dict[str, tuple[float, float]],
                              asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    asset = env.scene[asset_cfg.name]
    default_root_pose = wp.to_torch(asset.data.default_root_pose)[env_ids].clone()
    default_root_vel = wp.to_torch(asset.data.default_root_vel)[env_ids].clone()

    pose_range_list = [
        pose_range.get(key, (0.0, 0.0)) for key in ["x", "y", "z", "roll", "pitch", "yaw"]
    ]
    pose_ranges = torch.tensor(pose_range_list, device=asset.device)

    rand_samples = math_utils.sample_uniform(pose_ranges[:, 0],
                                             pose_ranges[:, 1], (len(env_ids), 6),
                                             device=asset.device)
    positions = default_root_pose[:, 0:3] + env.scene.env_origins[env_ids] + rand_samples[:, 0:3]
    orientations_delta = math_utils.quat_from_euler_xyz(rand_samples[:, 3], rand_samples[:, 4],
                                                        rand_samples[:, 5])
    orientations = math_utils.quat_mul(default_root_pose[:, 3:7], orientations_delta)

    velocity_range_list = [
        velocity_range.get(key, (0.0, 0.0)) for key in ["x", "y", "z", "roll", "pitch", "yaw"]
    ]
    velocity_ranges = torch.tensor(velocity_range_list, device=asset.device)
    rand_samples = math_utils.sample_uniform(velocity_ranges[:, 0],
                                             velocity_ranges[:, 1], (len(env_ids), 6),
                                             device=asset.device)
    velocities = default_root_vel + rand_samples

    return positions, orientations, velocities


def reset_root_state_uniform_collision_free(env: ZmRobotEnvWrapper,
                                            env_ids: torch.Tensor,
                                            pose_range: dict[str, tuple[float, float]],
                                            velocity_range: dict[str, tuple[float, float]],
                                            asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
                                            max_resample_trial=10):
    asset = env.scene[asset_cfg.name]

    positions, orientations, velocities = sample_root_state_uniform(env, env_ids, pose_range,
                                                                    velocity_range, asset_cfg)
    if env.collision_checker.is_initialized():
        env_ids_index_map = {int(val.item()): idx for idx, val in enumerate(env_ids)}
        resample_env_ids = env_ids
        resample_indices = torch.tensor(
            [env_ids_index_map[int(env_id.item())] for env_id in resample_env_ids])
        num_resample_trials = 0
        while num_resample_trials < max_resample_trial:
            positions_2d = positions[resample_indices, :2]
            if env.scene.cfg.environment.prim_path.startswith('/World/envs/'):
                positions_2d = positions_2d - env.scene.env_origins[resample_env_ids, :2]
            in_collisions = env.collision_checker.is_in_collision(positions_2d)
            resample_env_ids = resample_env_ids[torch.where(in_collisions == 1)[0]]
            if len(resample_env_ids) < 1:
                break
            resample_indices = torch.tensor(
                [env_ids_index_map[int(env_id.item())] for env_id in resample_env_ids])
            positions[resample_indices], orientations[resample_indices], velocities[
                resample_indices] = sample_root_state_uniform(env, resample_env_ids, pose_range,
                                                              velocity_range, asset_cfg)
            num_resample_trials += 1

    asset.write_root_pose_to_sim(torch.cat([positions, orientations], dim=-1), env_ids=env_ids)
    asset.write_root_velocity_to_sim(velocities, env_ids=env_ids)
