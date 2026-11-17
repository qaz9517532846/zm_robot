#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Sanity check + baseline: drive the robot with X-Mobility's OWN command only ([vx, 0, wz]
# from the frozen model, RGB + route + speed in, no learning) and report how episodes end.
# If the integration is right this should clearly beat a zero/random policy; it is also the
# "X-Mobility zero-shot" baseline row for the paper-style comparison.
#
# Usage: python3 scripts/eval_xmobility_base.py --num_envs 4 --steps 1500 --headless --enable_cameras

import argparse
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--steps", type=int, default=1500)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
# RTX geometry streaming + "read transforms from Fabric" (both on by default) intermittently
# makes the camera render pure black (rgb 0, depth all-inf) from the 3rd step on. Measured with
# the real X-Mobility term: 2/2 black without this flag, 0/3 black with it.
args_cli.kit_args = (args_cli.kit_args + " " if getattr(args_cli, "kit_args", "") else "") + "--/persistent/UJITSO/geometry=false"
simulation_app = AppLauncher(args_cli).app

import gymnasium as gym    # noqa: E402
import numpy as np    # noqa: E402
import torch    # noqa: E402
from isaaclab_rl.sb3 import Sb3VecEnvWrapper    # noqa: E402

from zm_robot_omni_training.config.zm_robot_env_cfg import ZmRobotGoalReachingSACStateLidarXMobEnvCfg    # noqa: E402
from zm_robot_omni_training.mdp.observations import XMOB_OBS_DIM    # noqa: E402
from zm_robot_omni_training.mdp.utils import goal_pose_in_robot_frame    # noqa: E402
from zm_robot_omni_training.wrapper.env_wrapper import ZmRobotEnvWrapper    # noqa: E402


def main():
    cfg = ZmRobotGoalReachingSACStateLidarXMobEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    raw = ZmRobotEnvWrapper(cfg=cfg)
    env = Sb3VecEnvWrapper(raw)
    env.action_space = gym.spaces.Box(low=np.array([-0.6, -0.6, -1.57], np.float32),
                                      high=np.array([0.6, 0.6, 1.57], np.float32), dtype=np.float32)
    u = raw.unwrapped
    tm = u.termination_manager
    print("[obs spaces]", {k: (v.shape, v.dtype) for k, v in env.observation_space.spaces.items()}, flush=True)

    obs = env.reset()
    counts = {"goal_reached": 0, "base_contact": 0, "time_out": 0}
    end_dist, start_dist, prog = [], [], []
    d0 = torch.norm(goal_pose_in_robot_frame(u, "goal_pose", (0.0, 0.0))[:, :2], dim=1)
    t0 = time.time()
    base_abs = []
    black = 0
    frames = 0
    for step in range(args_cli.steps):
        action = obs["xmobility"][:, 2048:XMOB_OBS_DIM].astype(np.float32)
        base_abs.append(np.abs(action).mean(0))
        obs, _, dones, _ = env.step(action)
        rgbs = u.scene["camera"].data.output["rgb"]
        black += int((rgbs.flatten(1).amax(1) == 0).sum())    # env-frames that rendered pure black
        frames += rgbs.shape[0]
        if step == 0:
            print(f"[first step done, GPU peak {torch.cuda.max_memory_allocated() / 1e9:.1f} GB]", flush=True)
        for i in np.nonzero(dones)[0]:
            for k in counts:
                if bool(tm.get_term(k)[i]):
                    counts[k] += 1
                    break
    dt = time.time() - t0
    n = max(sum(counts.values()), 1)
    print("=" * 60)
    print(f"X-Mobility base command only: {args_cli.num_envs} envs x {args_cli.steps} steps "
          f"({dt:.0f} s, {1000 * dt / args_cli.steps:.0f} ms/step)")
    print(f"episodes finished: {sum(counts.values())}  {counts}  success rate {counts['goal_reached'] / n:.1%}")
    print(f"black camera frames: {black}/{frames} ({100.0 * black / frames:.1f}%)")
    print("mean |base cmd| vx,vy,wz:", np.mean(base_abs, 0).round(3).tolist())
    dnow = torch.norm(goal_pose_in_robot_frame(u, "goal_pose", (0.0, 0.0))[:, :2], dim=1)
    print("dist to goal now:", dnow.cpu().numpy().round(2).tolist())
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
