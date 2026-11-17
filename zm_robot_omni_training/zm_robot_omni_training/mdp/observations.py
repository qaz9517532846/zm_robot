# SPDX-License-Identifier: Apache-2.0
#
# zm_robot's observation terms. `lidar_occupancy_grid` is zm_robot-specific (new); the
# rest (root_speed/root_yaw_rate/camera_img/camera_to_world/camera_intrinsics/
# routing_to_goal_simplified/relative_goal_heading/upsample_segments) are ported
# (logic-equivalent, no COMPASS import) from mobility_es/mdp/observations.py.

from __future__ import annotations

import torch
import warp as wp

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import convert_camera_frame_orientation_convention

from zm_robot_omni_training.mdp.utils import goal_pose_in_robot_frame
from zm_robot_omni_training.wrapper.env_wrapper import ZmRobotEnvWrapper

# Local occupancy grid extent and resolution (DESIGN.md section 5: +-5m, 0.1m/cell).
# Tunable - these are first-pass placeholder values, not yet validated against training.
GRID_RANGE_M = 5.0
GRID_RESOLUTION_M = 0.1
GRID_SIZE = int(2 * GRID_RANGE_M / GRID_RESOLUTION_M)    # 100x100


def root_speed(env: ZmRobotEnvWrapper,
               asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """The speed of the root, i.e. the norm of the linear velocity."""
    asset: RigidObject = env.scene[asset_cfg.name]
    return torch.norm(wp.to_torch(asset.data.root_lin_vel_w), dim=1).unsqueeze(1)


def root_yaw_rate(env: ZmRobotEnvWrapper,
                  asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """The yaw rate of the root."""
    asset: RigidObject = env.scene[asset_cfg.name]
    return wp.to_torch(asset.data.root_ang_vel_w)[:, 2].unsqueeze(1)


def camera_img(env: ZmRobotEnvWrapper, data_type: str = "rgb"):
    supported_data_types = ["rgb", "depth"]
    if data_type not in supported_data_types:
        raise ValueError(f"data type {data_type} is not in the support list {supported_data_types}")
    camera = env.scene["camera"]
    return camera.data.output[data_type].reshape((camera.data.output[data_type].shape[0], -1))


def camera_img_chw(env: ZmRobotEnvWrapper, data_type: str = "rgb", normalize: bool = True) -> torch.Tensor:
    """Camera image as [B, C, H, W] (channels-first), for RSL-RL's CNN model - which
    needs the native image shape, not camera_img()'s flattened [B, H*W*C]. TiledCamera's
    own output is channels-last uint8 [B, H, W, C] (isaaclab.sensors.camera.tiled_camera).

    normalize=True (RSL-RL/PPO path, the default): float in [0, 1] - Conv2d requires
    float input and RSL-RL's CNNModel does no normalization of its own.
    normalize=False (SB3/SAC path): raw [0, 255] values, left as-is - SB3's own CNN
    feature extractor expects uint8-range images and does the /255 scaling itself, and
    keeping this as uint8 (via the ObsTerm's clip=(0, 255), which Isaac Lab's
    Sb3VecEnvWrapper reads to cast dtype) cuts the SAC replay buffer's memory 4x versus
    storing pre-normalized float32 images.
    """
    camera = env.scene["camera"]
    # Always cast to float: Isaac Lab's ObservationManager applies the ObsTerm's
    # `clip` with float bounds via an in-place obs.clip_(min=.., max=..), which errors
    # on a uint8 tensor (result-type promotion can't write back in place) - the actual
    # uint8 cast for the SAC path happens later, in Sb3VecEnvWrapper's own observation
    # processor, not here.
    img = camera.data.output[data_type].permute(0, 3, 1, 2).float()
    if normalize and data_type == "rgb":
        img = img / 255.0
    return img


def camera_to_world(env: ZmRobotEnvWrapper):
    """Camera-to-world 4x4 transform matrices, batched: [B, 4, 4]."""
    camera = env.scene["camera"]
    pos_w = camera.data.pos_w
    quat_w_world = camera.data.quat_w_world
    quat_w_world = convert_camera_frame_orientation_convention(quat_w_world,
                                                               origin="world",
                                                               target="ros")

    batch_size = pos_w.shape[0]
    R_w_c = torch.zeros((batch_size, 3, 3), device=pos_w.device)
    x, y, z, w = quat_w_world[:, 0], quat_w_world[:, 1], quat_w_world[:, 2], quat_w_world[:, 3]
    R_w_c[:, 0, 0] = 1 - 2 * (y**2 + z**2)
    R_w_c[:, 0, 1] = 2 * (x * y - w * z)
    R_w_c[:, 0, 2] = 2 * (x * z + w * y)
    R_w_c[:, 1, 0] = 2 * (x * y + w * z)
    R_w_c[:, 1, 1] = 1 - 2 * (x**2 + z**2)
    R_w_c[:, 1, 2] = 2 * (y * z - w * x)
    R_w_c[:, 2, 0] = 2 * (x * z - w * y)
    R_w_c[:, 2, 1] = 2 * (y * z + w * x)
    R_w_c[:, 2, 2] = 1 - 2 * (x**2 + y**2)

    transform = torch.zeros((batch_size, 4, 4), device=pos_w.device)
    transform[:, :3, :3] = R_w_c
    transform[:, :3, 3] = pos_w
    transform[:, 3, 3] = 1.0
    return transform


def camera_intrinsics(env: ZmRobotEnvWrapper):
    camera = env.scene["camera"]
    return camera.data.intrinsic_matrices


def upsample_segments(start_pose, end_pose, d_max=1.0, n_segments=10):
    """Discretize the straight line from start_pose to end_pose into up to n_segments
    [x1,y1,x2,y2] chunks of length d_max (padded/truncated to exactly n_segments)."""
    num_env = start_pose.size(0)
    all_segments = torch.zeros((num_env, n_segments, 4)).to(start_pose.device)

    for i in range(num_env):
        s_x, s_y = start_pose[i]
        e_x, e_y = end_pose[i]
        distance = torch.sqrt((e_x - s_x)**2 + (e_y - s_y)**2)
        num_segments = torch.ceil(distance / d_max).int().item()

        t = torch.linspace(0, 1, num_segments + 1).unsqueeze(1)
        points = (1 - t) * torch.tensor([s_x, s_y]) + t * torch.tensor([e_x, e_y])
        segments = torch.cat([points[:-1], points[1:]], dim=1)

        if num_segments < n_segments:
            last_point = torch.tensor([e_x, e_y])
            last_segment = torch.cat([last_point, last_point], dim=0)
            padding_segments = last_segment.repeat(n_segments - num_segments, 1)
            segments = torch.cat([segments, padding_segments], dim=0)
        else:
            segments = segments[:n_segments]
        all_segments[i] = segments

    return all_segments


def routing_to_goal_simplified(
    env: ZmRobotEnvWrapper,
    command_name: str = "goal_pose",
    offset: tuple[float, float] = (-0.0, 0),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    goal_pose = goal_pose_in_robot_frame(env, command_name, offset, robot_cfg)
    start_pose = goal_pose.new_zeros(goal_pose.shape[0], 2)
    route = upsample_segments(start_pose, goal_pose)    # [B, n_segments, 4]
    # Flatten to [B, n_segments*4] so this term can concatenate with the other (rank-1)
    # terms in the "policy" observation group.
    return route.reshape(route.shape[0], -1)


def relative_goal_heading(env: ZmRobotEnvWrapper, command_name: str = "goal_pose") -> torch.Tensor:
    goal_heading = env.command_manager.get_command(command_name)[:, 3].unsqueeze(1)
    return torch.cat([torch.cos(goal_heading), torch.sin(goal_heading)], dim=1)


def lidar_occupancy_grid(
    env: ZmRobotEnvWrapper,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    lidar_names: tuple[str, ...] = ("lidar_0", "lidar_1"),
    normalize: bool = True,
) -> torch.Tensor:
    """Merge the two SICK LiDAR scans into one robot-centric local occupancy grid.

    Both LiDARs' ray hit points are projected into the robot's base_link frame and
    rasterized into a single [num_envs, 1, GRID_SIZE, GRID_SIZE] binary occupancy grid
    (channels-first, matching RSL-RL's CNNModel convention - see camera_img_chw). Rays
    that didn't hit anything within range are dropped rather than rasterized as
    obstacles.

    normalize=True (RSL-RL/PPO path, the default): binary 0.0/1.0 float.
    normalize=False (SB3/SAC path): binary 0/255, same "raw image range" convention as
    camera_img_chw(normalize=False) - SB3 divides by 255 internally, which recovers the
    original 0/1 occupancy semantics after its own CNN feature extractor.
    """
    robot = env.scene[robot_cfg.name]
    pos = wp.to_torch(robot.data.root_link_pos_w)    # [B, 3]
    quat = wp.to_torch(robot.data.root_link_quat_w)    # [B, 4], xyzw
    yaw = 2 * torch.atan2(quat[:, 2], quat[:, 3])    # [B]

    # Unlike root_link_pos_w/quat_w (warp arrays), RayCasterData.ray_hits_w is already
    # a plain torch.Tensor in this Isaac Lab version - wp.to_torch() on it errors out.
    hits_w = torch.cat([env.scene[name].data.ray_hits_w for name in lidar_names],
                       dim=1)    # [B, num_rays, 3]

    # Translate to robot-centric, then rotate into the robot's body frame (inverse yaw).
    rel = hits_w[..., :2] - pos[:, None, :2]    # [B, num_rays, 2]
    x = rel[..., 0]
    y = rel[..., 1]
    cos_yaw = torch.cos(-yaw).unsqueeze(-1)
    sin_yaw = torch.sin(-yaw).unsqueeze(-1)
    body_x = x * cos_yaw - y * sin_yaw
    body_y = x * sin_yaw + y * cos_yaw

    valid = torch.isfinite(hits_w).all(dim=-1)
    valid = valid & (body_x.abs() < GRID_RANGE_M) & (body_y.abs() < GRID_RANGE_M)

    ix = ((body_x + GRID_RANGE_M) / GRID_RESOLUTION_M).long().clamp(0, GRID_SIZE - 1)
    iy = ((body_y + GRID_RANGE_M) / GRID_RESOLUTION_M).long().clamp(0, GRID_SIZE - 1)
    flat_idx = (iy * GRID_SIZE + ix) * valid.long()    # invalid points dumped at index 0

    grid = torch.zeros(hits_w.shape[0], GRID_SIZE * GRID_SIZE, device=hits_w.device)
    grid.scatter_add_(1, flat_idx, valid.float())
    occupancy = (grid > 0).float().reshape(hits_w.shape[0], 1, GRID_SIZE, GRID_SIZE)
    return occupancy * 255.0 if not normalize else occupancy


def lidar_polar_ranges(
    env: ZmRobotEnvWrapper,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    lidar_names: tuple[str, ...] = ("lidar_0", "lidar_1"),
    num_sectors: int = 36,
    max_range: float = GRID_RANGE_M,
) -> torch.Tensor:
    """Compact LiDAR observation for MLP policies: both scans merged in the robot frame
    and reduced to the nearest obstacle distance in each of `num_sectors` equal bearing
    sectors (sector 0 starts at -pi, body x axis = bearing 0). Normalized to [0, 1],
    1.0 = nothing within max_range. [B, num_sectors].

    (lidar_occupancy_grid is the image-style version of the same information, for CNNs.)
    """
    robot = env.scene[robot_cfg.name]
    pos = wp.to_torch(robot.data.root_link_pos_w)
    quat = wp.to_torch(robot.data.root_link_quat_w)
    yaw = 2 * torch.atan2(quat[:, 2], quat[:, 3])

    hits_w = torch.cat([env.scene[name].data.ray_hits_w for name in lidar_names], dim=1)
    rel = hits_w[..., :2] - pos[:, None, :2]
    cos_yaw = torch.cos(-yaw).unsqueeze(-1)
    sin_yaw = torch.sin(-yaw).unsqueeze(-1)
    body_x = rel[..., 0] * cos_yaw - rel[..., 1] * sin_yaw
    body_y = rel[..., 0] * sin_yaw + rel[..., 1] * cos_yaw

    valid = torch.isfinite(hits_w).all(dim=-1)
    rng = torch.hypot(body_x, body_y)
    rng = torch.where(valid, rng, torch.full_like(rng, max_range)).clamp(max=max_range)
    ang = torch.atan2(torch.where(valid, body_y, torch.zeros_like(body_y)),
                      torch.where(valid, body_x, torch.ones_like(body_x)))
    idx = ((ang + torch.pi) / (2 * torch.pi) * num_sectors).long().clamp(0, num_sectors - 1)

    out = torch.full((hits_w.shape[0], num_sectors), max_range, device=hits_w.device)
    out.scatter_reduce_(1, idx, rng, reduce="amin")
    return out / max_range


# --------------------------------------------------------------------------------------
# X-Mobility (frozen) as a feature/prior source, plus a depth branch for the policy.
# X-Mobility itself only sees RGB; depth reaches the policy through camera_depth_u8.
# --------------------------------------------------------------------------------------
import torch.nn.functional as F    # noqa: E402

from isaaclab.managers import ManagerTermBase    # noqa: E402

XMOB_OBS_DIM = 2048 + 3    # policy_state (2048) + base command [vx, vy, wz]


class XMobilityObs(ManagerTermBase):
    """Per-step output of the frozen X-Mobility: its 2048-d fused policy state and its own
    suggested command mapped to this robot's action space ([vx, 0, wz]; X-Mobility was
    trained on differential-drive robots, so it never commands vy).

    Stateful: the RSSM history is kept per env and zeroed in reset(env_ids) (called by the
    observation manager when an episode ends), like COMPASS does.
    """

    def __init__(self, cfg, env: ZmRobotEnvWrapper):
        super().__init__(cfg, env)
        from zm_robot_omni_training.models.xmobility import XMobilityRunner
        self._runner = XMobilityRunner(env.num_envs, str(env.device))
        self._last = None
        self._last_key = None

    def reset(self, env_ids=None):
        self._runner.reset(env_ids)
        self._last_key = None

    def __call__(self, env: ZmRobotEnvWrapper, max_lin: float = 0.6, max_ang: float = 1.57,
                 camera_name: str = "camera") -> torch.Tensor:
        # ZmRobotEnvWrapper.reset() recomputes observations once more right after
        # super().reset(); do not advance the RSSM twice for the same simulation state.
        key = (env.common_step_counter, int(env.episode_length_buf.sum().item()))
        if key == self._last_key and self._last is not None:
            return self._last
        act = env.action_manager.action    # the command applied on the previous step
        self._runner.set_executed_action(act[:, 0], act[:, 1], act[:, 2])
        rgb = env.scene[camera_name].data.output["rgb"][..., :3]
        speed = root_speed(env).reshape(env.num_envs, 1)
        route = routing_to_goal_simplified(env)
        policy_state, a6 = self._runner.step(rgb, speed, route)
        base = torch.stack([a6[:, 0].clamp(-max_lin, max_lin), torch.zeros_like(a6[:, 0]),
                            a6[:, 5].clamp(-max_ang, max_ang)], dim=1)
        self._last = torch.cat([policy_state, base], dim=1)
        self._last_key = key
        return self._last


def camera_depth_u8(env: ZmRobotEnvWrapper, camera_name: str = "camera", out_hw=(64, 104),
                    max_depth: float = 5.0) -> torch.Tensor:
    """Depth as a small channels-first image on [0, 255] (0 = at the camera, 255 = >= max_depth
    or no return), so SB3 treats it as an image (CNN) and stores it as uint8 in the replay
    buffer. Area-averaged down from the 320x512 render."""
    d = env.scene[camera_name].data.output["depth"].float()    # [B,H,W,1]
    d = torch.nan_to_num(d, nan=max_depth, posinf=max_depth, neginf=0.0).clamp(0.0, max_depth)
    d = d.permute(0, 3, 1, 2)
    d = F.adaptive_avg_pool2d(d, out_hw)
    return d / max_depth * 255.0
