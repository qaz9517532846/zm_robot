#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Train zm_robot's holonomic navigation policy with RSL-RL PPO. No COMPASS import, no
# gym.make/task-registry/hydra machinery (that exists in Isaac Lab's own generic
# train.py to support hundreds of registered tasks via a --task string; we only have
# one task, so this constructs the env directly instead).
#
# Usage:
#   python train.py --num_envs 16 --headless --enable_cameras [--max_iterations N]

import argparse
import os
import time
from datetime import datetime

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Train zm_robot with RSL-RL PPO.")
parser.add_argument("--num_envs", type=int, default=None)
parser.add_argument("--max_iterations", type=int, default=None)
parser.add_argument("--resume_checkpoint", type=str, default=None,
                    help="Path to an rsl_rl .pt checkpoint to resume from.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import importlib.metadata    # noqa: E402

from rsl_rl.runners import OnPolicyRunner    # noqa: E402  (must come after AppLauncher)

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg    # noqa: E402

from zm_robot_omni_training.config.zm_robot_env_cfg import ZmRobotGoalReachingEnvCfg    # noqa: E402
from zm_robot_omni_training.config.agents.rsl_rl_ppo_cfg import ZM_ROBOT_PPO_RUNNER_CFG    # noqa: E402
from zm_robot_omni_training.wrapper.env_wrapper import ZmRobotEnvWrapper    # noqa: E402


def main():
    agent_cfg = ZM_ROBOT_PPO_RUNNER_CFG
    if args_cli.max_iterations is not None:
        agent_cfg.max_iterations = args_cli.max_iterations
    # Migrates/cleans up deprecated model-cfg fields (e.g. the legacy "stochastic" flag)
    # for whatever rsl-rl-lib version is actually installed - skipping this makes the
    # model constructor choke on stale fields the cfg dataclasses still carry.
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, importlib.metadata.version("rsl-rl-lib"))

    env_cfg = ZmRobotGoalReachingEnvCfg()
    if args_cli.num_envs is not None:
        env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = agent_cfg.seed

    log_root_path = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))
    log_dir = os.path.join(log_root_path, datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
    print(f"[INFO] Logging experiment in directory: {log_dir}")

    env = ZmRobotEnvWrapper(cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
    if args_cli.resume_checkpoint:
        print(f"[INFO] Resuming from checkpoint: {args_cli.resume_checkpoint}")
        runner.load(args_cli.resume_checkpoint)

    start_time = time.time()
    try:
        runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)
        print(f"Training time: {round(time.time() - start_time, 2)} seconds")
    except KeyboardInterrupt:
        pass
    finally:
        env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
