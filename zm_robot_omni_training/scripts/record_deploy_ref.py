#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Record a reference rollout for validating the ROS2 deployment (zm_robot_omni_policy):
# one environment, deterministic residual policy, and for every step the RAW sensor data
# (RGB, depth, per-LiDAR ray hits + sensor pose, robot pose/twist, goal pose) together with
# the observation dict the PyTorch policy saw and the commands it produced (X-Mobility base,
# SAC residual, final). The ROS2 replay node publishes the raw data as topics and compares
# the deployed node's /cmd_vel with `final`.
#
# Usage (inside the container):
#   python3 scripts/record_deploy_ref.py --checkpoint logs/sb3/.../model_150000_steps.zip \
#       --reward hybrid --out deploy_export/ref_hybrid.npz --episodes 3 --max_steps 220 \
#       --headless --enable_cameras

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--reward", choices=["progress", "penalty", "hybrid"], default="hybrid")
parser.add_argument("--residual_scale", type=float, default=0.5)
parser.add_argument("--out", type=str, default="deploy_export/ref.npz")
parser.add_argument("--episodes", type=int, default=3)
parser.add_argument("--max_steps", type=int, default=220, help="total step cap")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.kit_args = (args_cli.kit_args + " " if getattr(args_cli, "kit_args", "") else "") + "--/persistent/UJITSO/geometry=false"
simulation_app = AppLauncher(args_cli).app

import os    # noqa: E402

import gymnasium as gym    # noqa: E402
import numpy as np    # noqa: E402
import warp as wp    # noqa: E402
from stable_baselines3 import SAC    # noqa: E402

from isaaclab_rl.sb3 import Sb3VecEnvWrapper    # noqa: E402

from zm_robot_omni_training.config.zm_robot_env_cfg import (    # noqa: E402
    ZmRobotGoalReachingSACStateLidarXMobEnvCfg, ZmRobotGoalReachingSACStateLidarXMobHybridEnvCfg,
    ZmRobotGoalReachingSACStateLidarXMobPenaltyEnvCfg)
from zm_robot_omni_training.wrapper.env_wrapper import ZmRobotEnvWrapper    # noqa: E402
from zm_robot_omni_training.wrapper.residual_wrapper import ResidualActionVecEnv    # noqa: E402

ACTION_LOW = np.array([-0.6, -0.6, -1.57], dtype=np.float32)
ACTION_HIGH = np.array([0.6, 0.6, 1.57], dtype=np.float32)


def t2n(x):
    return (wp.to_torch(x) if not hasattr(x, "detach") else x).detach().cpu().numpy().copy()


def main():
    cfg = {"progress": ZmRobotGoalReachingSACStateLidarXMobEnvCfg,
           "penalty": ZmRobotGoalReachingSACStateLidarXMobPenaltyEnvCfg,
           "hybrid": ZmRobotGoalReachingSACStateLidarXMobHybridEnvCfg}[args_cli.reward]()
    cfg.scene.num_envs = 1
    raw = ZmRobotEnvWrapper(cfg=cfg)
    env = Sb3VecEnvWrapper(raw)
    env.action_space = gym.spaces.Box(low=ACTION_LOW, high=ACTION_HIGH, dtype=np.float32)
    env = ResidualActionVecEnv(env, ACTION_LOW, ACTION_HIGH, args_cli.residual_scale)
    model = SAC.load(args_cli.checkpoint, env=env)
    u = raw.unwrapped
    tm = u.termination_manager
    cam, robot = u.scene["camera"], u.scene["robot"]
    lidars = [u.scene["lidar_0"], u.scene["lidar_1"]]
    cmd = u.command_manager.get_term("goal_pose")

    rec = {k: [] for k in ("rgb", "depth", "root_pos", "root_quat", "lin_vel_w", "ang_vel_w", "goal_pos_w",
                           "goal_heading_w", "base", "residual", "final", "episode", "ep_step", "done",
                           "cause")}
    for i in range(2):
        for k in ("hits", "pos", "quat"):
            rec[f"lidar{i}_{k}"] = []
    obs_rec = {}
    obs = env.reset()
    ep, ep_step = 0, 0
    for _ in range(args_cli.max_steps):
        residual, _ = model.predict(obs, deterministic=True)
        base = np.asarray(obs["xmobility"][0, 2048:2051], np.float32)
        final = np.clip(base + residual[0], ACTION_LOW, ACTION_HIGH)
        rec["rgb"].append(t2n(cam.data.output["rgb"][0][..., :3]).astype(np.uint8))
        rec["depth"].append(t2n(cam.data.output["depth"][0, ..., 0]).astype(np.float32))
        rec["root_pos"].append(t2n(robot.data.root_link_pos_w)[0])
        rec["root_quat"].append(t2n(robot.data.root_link_quat_w)[0])
        rec["lin_vel_w"].append(t2n(robot.data.root_lin_vel_w)[0])
        rec["ang_vel_w"].append(t2n(robot.data.root_ang_vel_w)[0])
        rec["goal_pos_w"].append(t2n(cmd.pos_command_w)[0])
        rec["goal_heading_w"].append(float(t2n(cmd.heading_command_w)[0]))
        for i, ld in enumerate(lidars):
            rec[f"lidar{i}_hits"].append(t2n(ld.data.ray_hits_w)[0].astype(np.float32))
            rec[f"lidar{i}_pos"].append(t2n(ld.data.pos_w)[0])
            rec[f"lidar{i}_quat"].append(t2n(ld.data.quat_w)[0])
        for k, v in obs.items():
            obs_rec.setdefault(k, []).append(np.asarray(v[0], np.float32).copy())
        rec["base"].append(base)
        rec["residual"].append(np.asarray(residual[0], np.float32))
        rec["final"].append(final)
        rec["episode"].append(ep)
        rec["ep_step"].append(ep_step)
        obs, _, dones, _ = env.step(residual)
        ep_step += 1
        cause = ""
        if dones[0]:
            cause = next((k for k in ("goal_reached", "base_contact", "time_out") if bool(tm.get_term(k)[0])), "other")
            print(f"[ref] episode {ep + 1} ended after {ep_step} steps: {cause}", flush=True)
            ep, ep_step = ep + 1, 0
        rec["done"].append(bool(dones[0]))
        rec["cause"].append(cause)
        if ep >= args_cli.episodes:
            break
    out = {k: np.asarray(v) for k, v in rec.items()}
    out.update({f"obs_{k}": np.asarray(v) for k, v in obs_rec.items()})
    for i, ld in enumerate(lidars):    # ray pattern in the sensor frame, if the sensor exposes it
        for attr in ("ray_directions", "ray_starts"):
            if hasattr(ld, attr):
                out[f"lidar{i}_{attr}"] = t2n(getattr(ld, attr))[0]
    out["step_dt"] = np.float32(u.step_dt)
    os.makedirs(os.path.dirname(os.path.abspath(args_cli.out)), exist_ok=True)
    np.savez_compressed(args_cli.out, **out)
    print(f"[ref] wrote {len(rec['final'])} steps -> {args_cli.out} "
          f"({os.path.getsize(args_cli.out) / 1e6:.1f} MB)", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
