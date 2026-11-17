#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Roll out a trained SB3 SAC checkpoint with an overhead camera and write an annotated
# mp4 (robot, goal, distance, step). Uses its own camera + imageio instead of Isaac
# Lab's VideoRecorderCfg so the overlay can be drawn per frame.
#
# Usage:
#   python3 scripts/record_video.py --checkpoint <model.zip> --obs state_lidar \
#       --out logs/videos/demo.mp4 --episodes 3 --max_steps 1200 --headless --enable_cameras

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--obs", choices=["state", "state_lidar"], default="state_lidar")
parser.add_argument("--out", type=str, default="logs/videos/demo.mp4")
parser.add_argument("--episodes", type=int, default=3)
parser.add_argument("--max_steps", type=int, default=1200)
parser.add_argument("--cam_height", type=float, default=6.5, help="camera height [m]")
parser.add_argument("--size", type=int, default=480)
parser.add_argument("--pitch_sign", type=float, default=-1.0,
                    help="-1 (default): rotate -90 deg about Y, camera looks DOWN. +1 looks "
                    "up at the sky and renders black (verified).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
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
    ZmRobotGoalReachingSACStateEnvCfg, ZmRobotGoalReachingSACStateLidarEnvCfg)
from zm_robot_omni_training.mdp.utils import goal_pose_in_robot_frame    # noqa: E402
from zm_robot_omni_training.wrapper.env_wrapper import ZmRobotEnvWrapper    # noqa: E402

ACTION_LOW = np.array([-0.6, -0.6, -1.57], dtype=np.float32)
ACTION_HIGH = np.array([0.6, 0.6, 1.57], dtype=np.float32)
FOCAL, APERTURE = 10.0, 20.955    # same pinhole as the robot camera in scene_assets.py


def main():
    cfg = (ZmRobotGoalReachingSACStateLidarEnvCfg()
           if args_cli.obs == "state_lidar" else ZmRobotGoalReachingSACStateEnvCfg())
    cfg.scene.num_envs = 1
    # Overhead camera straight down. "world" convention looks along +X; rotating +90 deg
    # about Y sends +X to -Z. Resulting image: up = world +X, right = world -Y.
    cfg.scene.overhead_cam = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/OverheadCam",
        update_period=0.0,
        height=args_cli.size,
        width=args_cli.size,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(focal_length=FOCAL,
                                         focus_distance=400.0,
                                         horizontal_aperture=APERTURE,
                                         clipping_range=(0.1, 50.0)),
        offset=CameraCfg.OffsetCfg(pos=(0.0, 0.0, args_cli.cam_height),
                                   rot=(0.70710678, 0.0, args_cli.pitch_sign * 0.70710678, 0.0),
                                   convention="world"),
    )
    raw = ZmRobotEnvWrapper(cfg=cfg)
    env = Sb3VecEnvWrapper(raw)
    env.action_space = gym.spaces.Box(low=ACTION_LOW, high=ACTION_HIGH, dtype=np.float32)
    model = SAC.load(args_cli.checkpoint, env=env)
    u = raw.unwrapped
    tm = u.termination_manager
    cam = u.scene["overhead_cam"]
    cmd = u.command_manager.get_term("goal_pose")
    s = args_cli.size * FOCAL / (APERTURE * args_cli.cam_height)    # pixels per metre

    def to_px(x, y):
        # pitch_sign=+1: image up = world +X, right = world -Y. pitch_sign=-1 (verified
        # on a rendered frame: the real robot sat at the point mirrored through the
        # image centre from where the +1 mapping drew it): image up = -X, right = +Y.
        k = args_cli.pitch_sign
        return int(args_cli.size / 2 - k * s * y), int(args_cli.size / 2 - k * s * x)

    os.makedirs(os.path.dirname(os.path.abspath(args_cli.out)), exist_ok=True)
    writer = imageio.get_writer(args_cli.out, fps=10, codec="libx264", quality=7)
    obs = env.reset()
    ep, ep_step, frames = 0, 0, 0
    for step in range(args_cli.max_steps):
        action, _ = model.predict(obs, deterministic=True)
        # Frame for the state the policy just acted on (before the step).
        rgb = cam.data.output["rgb"][0][..., :3].cpu().numpy().astype(np.uint8).copy()
        origin = u.scene.env_origins[0, :2]
        rpos = (wp.to_torch(u.scene["robot"].data.root_pos_w)[0, :2] - origin).cpu().numpy()
        gpos = (cmd.pos_command_w[0, :2] - origin).cpu().numpy()
        dist = float(torch.norm(goal_pose_in_robot_frame(u, "goal_pose", (0.0, 0.0))[0, :2]))
        quat = wp.to_torch(u.scene["robot"].data.root_quat_w)[0]
        yaw = float(2 * torch.atan2(quat[2], quat[3]))
        gp, rp = to_px(*gpos), to_px(*rpos)
        cv2.circle(rgb, gp, max(int(0.5 * s), 3), (255, 0, 0), 2)    # goal tolerance 0.5 m
        cv2.circle(rgb, gp, 3, (255, 0, 0), -1)
        cv2.circle(rgb, rp, 5, (0, 255, 0), -1)
        tip = to_px(rpos[0] + 0.7 * np.cos(yaw), rpos[1] + 0.7 * np.sin(yaw))
        cv2.arrowedLine(rgb, rp, tip, (0, 255, 0), 2)
        cv2.putText(rgb, f"ep {ep + 1}  t={ep_step * 0.1:5.1f}s  dist={dist:4.2f}m", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1, cv2.LINE_AA)
        writer.append_data(rgb)
        if frames == 0:
            imageio.imwrite(os.path.splitext(args_cli.out)[0] + "_frame0.png", rgb)
        frames += 1

        obs, _, dones, _ = env.step(action)
        ep_step += 1
        if dones[0]:
            cause = next((k for k in ("goal_reached", "base_contact", "time_out")
                          if bool(tm.get_term(k)[0])), "other")
            print(f"[video] episode {ep + 1} ended after {ep_step} steps: {cause}", flush=True)
            ep += 1
            ep_step = 0
            if ep >= args_cli.episodes:
                break
    writer.close()
    print(f"[video] wrote {frames} frames -> {args_cli.out}", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
