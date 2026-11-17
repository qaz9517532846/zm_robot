#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Standalone smoke test for the zm_robot env - no COMPASS import, no RL training loop
# yet. Just: launch Isaac Sim, spawn the env, step it with random actions for a bit,
# print observation shapes once. Replaces relying on COMPASS's run.py for this check.
#
# Usage (run inside the Isaac Lab python environment):
#   python scripts/smoke_test.py --num_envs 1 --headless --enable_cameras --num_steps 200

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="zm_robot standalone env smoke test.")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--num_steps", type=int, default=200)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch    # noqa: E402  (must come after AppLauncher)

from zm_robot_omni_training.config.zm_robot_env_cfg import ZmRobotGoalReachingEnvCfg    # noqa: E402
from zm_robot_omni_training.wrapper.env_wrapper import ZmRobotEnvWrapper    # noqa: E402


def main():
    cfg = ZmRobotGoalReachingEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs

    env = ZmRobotEnvWrapper(cfg=cfg)
    obs, _ = env.reset()

    print("=== Observation group shapes ===")
    for group_name in ("policy", "camera", "lidar"):
        print(f"  {group_name}: {tuple(obs[group_name].shape)}")

    action_dim = env.action_manager.total_action_dim
    print(f"=== action_dim: {action_dim} ===")

    for step in range(args_cli.num_steps):
        actions = torch.zeros(env.num_envs, action_dim, device=env.device)
        actions[:, 0] = 0.3    # small constant forward vx, just to see it move
        obs, reward, terminated, truncated, info = env.step(actions)
        if step % 50 == 0:
            print(f"step {step}: reward={reward.mean().item():.3f} "
                 f"terminated={terminated.any().item()}")

    print("Smoke test finished without crashing.")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
