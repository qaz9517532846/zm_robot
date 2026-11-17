#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Roll out a residual-form checkpoint (state_lidar_xmob) with ONE environment and write a
# composite mp4 + key-frame PNGs: overhead view (robot, goal), the robot's front RGB camera,
# its depth image, and the 36-sector LiDAR polar plot - i.e. what the policy sees.
#
# Usage (inside the container):
#   python3 scripts/record_resid_demo.py --checkpoint logs/sb3/.../model_150000_steps.zip \
#       --reward hybrid --out logs/videos/resid_hybrid_demo.mp4 --episodes 4 --max_steps 900 \
#       --headless --enable_cameras

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--reward", choices=["progress", "penalty", "hybrid"], default="hybrid")
parser.add_argument("--residual_scale", type=float, default=0.5)
parser.add_argument("--out", type=str, default="logs/videos/resid_demo.mp4")
parser.add_argument("--episodes", type=int, default=4)
parser.add_argument("--max_steps", type=int, default=900)
parser.add_argument("--cam_height", type=float, default=6.5)
parser.add_argument("--size", type=int, default=540)
parser.add_argument("--seed", type=int, default=7)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
# Same RTX black-frame workaround as eval_sac.py.
args_cli.kit_args = (args_cli.kit_args + " " if getattr(args_cli, "kit_args", "") else "") + "--/persistent/UJITSO/geometry=false"
simulation_app = AppLauncher(args_cli).app

import os    # noqa: E402

import cv2    # noqa: E402
import gymnasium as gym    # noqa: E402
import imageio    # noqa: E402
import numpy as np    # noqa: E402
import torch    # noqa: E402
import warp as wp    # noqa: E402
from stable_baselines3 import SAC    # noqa: E402

import isaaclab.sim as sim_utils    # noqa: E402
from isaaclab.sensors import CameraCfg, TiledCameraCfg    # noqa: E402
from isaaclab_rl.sb3 import Sb3VecEnvWrapper    # noqa: E402

from zm_robot_omni_training.config.zm_robot_env_cfg import (    # noqa: E402
    ZmRobotGoalReachingSACStateLidarXMobEnvCfg, ZmRobotGoalReachingSACStateLidarXMobHybridEnvCfg,
    ZmRobotGoalReachingSACStateLidarXMobPenaltyEnvCfg)
from zm_robot_omni_training.mdp.utils import goal_pose_in_robot_frame    # noqa: E402
from zm_robot_omni_training.wrapper.env_wrapper import ZmRobotEnvWrapper    # noqa: E402
from zm_robot_omni_training.wrapper.residual_wrapper import ResidualActionVecEnv    # noqa: E402

ACTION_LOW = np.array([-0.6, -0.6, -1.57], dtype=np.float32)
ACTION_HIGH = np.array([0.6, 0.6, 1.57], dtype=np.float32)
FOCAL, APERTURE = 10.0, 20.955
LIDAR_MAX_M = 5.0


def lidar_panel(ranges, size=230):
    """Polar plot of the 36 normalised sector ranges; robot heading (bearing 0) points up."""
    img = np.full((size, size, 3), 250, np.uint8)
    c, r = size // 2, size // 2 - 14
    for f in (0.25, 0.5, 0.75, 1.0):
        cv2.circle(img, (c, c), int(r * f), (200, 200, 200), 1, cv2.LINE_AA)
    pts = []
    n = len(ranges)
    for i, v in enumerate(ranges):
        b = -np.pi + (i + 0.5) * 2 * np.pi / n    # bearing of sector centre, 0 = body +x
        rr = r * float(v)
        pts.append((int(c - rr * np.sin(b)), int(c - rr * np.cos(b))))    # up = forward, left = +y
    cv2.fillPoly(img, [np.array(pts, np.int32)], (210, 235, 214))
    cv2.polylines(img, [np.array(pts, np.int32)], True, (40, 130, 70), 1, cv2.LINE_AA)
    cv2.arrowedLine(img, (c, c), (c, c - 22), (0, 0, 200), 2, cv2.LINE_AA, tipLength=0.35)
    cv2.circle(img, (c, c), 4, (0, 0, 200), -1)
    cv2.putText(img, f"LiDAR (rings 1.25/2.5/3.75/{LIDAR_MAX_M:.0f} m)", (4, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (60, 60, 60), 1, cv2.LINE_AA)
    return img


def compose(over, rgb, depth, lidar, text):
    W, Hh = args_cli.size + 6 + 480, args_cli.size
    canvas = np.full((Hh, W, 3), 255, np.uint8)
    canvas[:args_cli.size, :args_cli.size] = over
    x0 = args_cli.size + 6
    rgb_s = cv2.resize(rgb, (480, 300), interpolation=cv2.INTER_AREA)
    canvas[0:300, x0:x0 + 480] = rgb_s
    cv2.putText(canvas, "robot RGB (X-Mobility input)", (x0 + 6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)
    d = np.clip(depth / 5.0, 0, 1)
    dcol = cv2.applyColorMap((255 - d * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)[..., ::-1]
    dcol = cv2.resize(dcol, (240, 150), interpolation=cv2.INTER_AREA)
    canvas[306:456, x0 + 236:x0 + 476] = dcol
    cv2.putText(canvas, "depth (0-5 m)", (x0 + 242, 322), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    canvas[306:536, x0:x0 + 230] = lidar_panel(lidar)
    cv2.putText(canvas, "action (m/s, m/s, rad/s)", (x0 + 236, 482), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (60, 60, 60), 1, cv2.LINE_AA)
    cv2.putText(canvas, text, (x0 + 236, 504), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30, 30, 30), 1, cv2.LINE_AA)
    return canvas


def main():
    cfg = {"progress": ZmRobotGoalReachingSACStateLidarXMobEnvCfg,
           "penalty": ZmRobotGoalReachingSACStateLidarXMobPenaltyEnvCfg,
           "hybrid": ZmRobotGoalReachingSACStateLidarXMobHybridEnvCfg}[args_cli.reward]()
    cfg.scene.num_envs = 1
    cfg.seed = args_cli.seed
    cfg.scene.overhead_cam = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/OverheadCam", update_period=0.0, height=args_cli.size, width=args_cli.size,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(focal_length=FOCAL, focus_distance=400.0, horizontal_aperture=APERTURE,
                                         clipping_range=(0.1, 50.0)),
        offset=CameraCfg.OffsetCfg(pos=(0.0, 0.0, args_cli.cam_height), rot=(0.70710678, 0.0, -0.70710678, 0.0),
                                   convention="world"))
    # The residual env removes cfg.scene.camera? No: the X-Mobility obs needs it, so it stays.
    raw = ZmRobotEnvWrapper(cfg=cfg)
    env = Sb3VecEnvWrapper(raw)
    env.action_space = gym.spaces.Box(low=ACTION_LOW, high=ACTION_HIGH, dtype=np.float32)
    env = ResidualActionVecEnv(env, ACTION_LOW, ACTION_HIGH, args_cli.residual_scale)
    model = SAC.load(args_cli.checkpoint, env=env)
    u = raw.unwrapped
    tm = u.termination_manager
    ocam, rcam = u.scene["overhead_cam"], u.scene["camera"]
    cmd = u.command_manager.get_term("goal_pose")
    s = args_cli.size * FOCAL / (APERTURE * args_cli.cam_height)

    def to_px(x, y):    # pitch_sign = -1 mapping from record_video.py (image up = -X, right = +Y)
        return int(args_cli.size / 2 + s * y), int(args_cli.size / 2 + s * x)

    base = os.path.splitext(os.path.abspath(args_cli.out))[0]
    os.makedirs(os.path.dirname(base), exist_ok=True)
    writer = imageio.get_writer(args_cli.out, fps=10, codec="libx264", quality=7)
    obs = env.reset()
    ep, ep_step, frames, black = 0, 0, 0, 0
    ep_frames = []
    for _ in range(args_cli.max_steps * args_cli.episodes):
        action, _ = model.predict(obs, deterministic=True)
        over = ocam.data.output["rgb"][0][..., :3].cpu().numpy().astype(np.uint8).copy()
        rgb = rcam.data.output["rgb"][0][..., :3].cpu().numpy().astype(np.uint8).copy()
        depth = rcam.data.output["depth"][0, ..., 0].float().cpu().numpy()
        depth = np.nan_to_num(depth, nan=5.0, posinf=5.0, neginf=0.0)
        if rgb.max() == 0:
            black += 1
        origin = u.scene.env_origins[0, :2]
        rpos = (wp.to_torch(u.scene["robot"].data.root_pos_w)[0, :2] - origin).cpu().numpy()
        gpos = (cmd.pos_command_w[0, :2] - origin).cpu().numpy()
        dist = float(torch.norm(goal_pose_in_robot_frame(u, "goal_pose", (0.0, 0.0))[0, :2]))
        quat = wp.to_torch(u.scene["robot"].data.root_quat_w)[0]
        yaw = float(2 * torch.atan2(quat[2], quat[3]))
        gp, rp = to_px(*gpos), to_px(*rpos)
        cv2.circle(over, gp, max(int(0.5 * s), 3), (255, 0, 0), 2)
        cv2.circle(over, gp, 3, (255, 0, 0), -1)
        cv2.circle(over, rp, 5, (0, 255, 0), -1)
        cv2.arrowedLine(over, rp, to_px(rpos[0] + 0.7 * np.cos(yaw), rpos[1] + 0.7 * np.sin(yaw)), (0, 255, 0), 2)
        cv2.putText(over, f"ep {ep + 1}  t={ep_step * 0.1:5.1f}s  dist={dist:4.2f}m", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1, cv2.LINE_AA)
        lidar = np.asarray(obs["lidar_ranges"][0], dtype=np.float32)
        a = action[0]
        fr = compose(over, rgb, depth, lidar, f"{a[0]:+.2f} {a[1]:+.2f} {a[2]:+.2f}")
        writer.append_data(fr)
        ep_frames.append(fr)
        frames += 1
        obs, _, dones, _ = env.step(action)
        ep_step += 1
        if dones[0]:
            cause = next((k for k in ("goal_reached", "base_contact", "time_out") if bool(tm.get_term(k)[0])), "other")
            print(f"[demo] episode {ep + 1} ended after {ep_step} steps: {cause}", flush=True)
            n = len(ep_frames)
            for tag, idx in (("start", 0), ("mid", n // 2), ("end", n - 1)):
                imageio.imwrite(f"{base}_ep{ep + 1}_{cause}_{tag}.png", ep_frames[idx])
            ep_frames, ep, ep_step = [], ep + 1, 0
            if ep >= args_cli.episodes:
                break
    writer.close()
    print(f"[demo] wrote {frames} frames -> {args_cli.out}; black robot-camera frames: {black}", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
