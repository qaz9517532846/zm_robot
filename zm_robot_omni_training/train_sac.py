#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Train zm_robot's holonomic navigation policy with Stable-Baselines3 SAC - the
# off-policy path (see config/agents/sb3_sac_cfg.py for why). Mirrors train.py's
# structure (direct env construction, no --task/gym.make registry).
#
# Usage:
#   python train_sac.py --num_envs 6 --headless --enable_cameras [--total_timesteps N]

import argparse
import os
from datetime import datetime

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Train zm_robot with Stable-Baselines3 SAC.")
parser.add_argument("--num_envs", type=int, default=None)
parser.add_argument("--total_timesteps", type=int, default=None)
parser.add_argument("--obs", choices=["full", "state", "state_lidar", "state_lidar_xmob"], default="full",
                    help="full: camera+LiDAR+vectors (CNN); state: vectors only (MLP, diagnostic).")
parser.add_argument("--reward", choices=["progress", "penalty", "hybrid"], default="progress",
                    help="progress: goal_heading_progress (exp 9, default). penalty: the old "
                    "goal_heading_alignment penalty (exp 7) - state_lidar only.")
parser.add_argument("--seed", type=int, default=None,
                    help="Seed for the env (cfg.seed) and for SB3. None = unseeded (the "
                    "original exp 2-9 runs were unseeded).")
parser.add_argument("--run_name", type=str, default=None,
                    help="Log sub-directory name instead of a timestamp, so later scripts "
                    "know the checkpoint path in advance.")
parser.add_argument("--residual_scale", type=float, default=0.0,
                    help="COMPASS-style residual: >0 makes SAC output a residual (this fraction of the "
                    "full command range) added to X-Mobility's own command. 0 = off. "
                    "state_lidar_xmob only.")
parser.add_argument("--resume_checkpoint", type=str, default=None,
                    help="Path to an SB3 .zip checkpoint to resume from.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if args_cli.residual_scale > 0 and args_cli.obs != "state_lidar_xmob":
    parser.error("--residual_scale needs --obs state_lidar_xmob")
if args_cli.reward == "hybrid" and args_cli.obs not in ("state_lidar", "state_lidar_xmob"):
    parser.error("--reward hybrid is only defined for --obs state_lidar / state_lidar_xmob")
if args_cli.reward == "penalty" and args_cli.obs not in ("state_lidar", "state_lidar_xmob"):
    parser.error("--reward penalty is only defined for --obs state_lidar / state_lidar_xmob")

# RTX geometry streaming + "read transforms from Fabric" (both on by default) intermittently
# makes the camera render pure black (rgb 0, depth all-inf) from the 3rd step on. Measured with
# the real X-Mobility term: 2/2 black without this flag, 0/3 black with it.
args_cli.kit_args = (args_cli.kit_args + " " if getattr(args_cli, "kit_args", "") else "") + "--/persistent/UJITSO/geometry=false"
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np    # noqa: E402
import gymnasium as gym    # noqa: E402

from stable_baselines3 import SAC    # noqa: E402  (must come after AppLauncher)
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback    # noqa: E402

from isaaclab_rl.sb3 import Sb3VecEnvWrapper    # noqa: E402

from zm_robot_omni_training.config.zm_robot_env_cfg import (ZmRobotGoalReachingSACEnvCfg,
                                                             ZmRobotGoalReachingSACStateEnvCfg,
                                                             ZmRobotGoalReachingSACStateLidarEnvCfg,
                                                             ZmRobotGoalReachingSACStateLidarPenaltyEnvCfg,
                                                             ZmRobotGoalReachingSACStateLidarHybridEnvCfg,
                                                             ZmRobotGoalReachingSACStateLidarXMobEnvCfg,
                                                             ZmRobotGoalReachingSACStateLidarXMobPenaltyEnvCfg,
                                                             ZmRobotGoalReachingSACStateLidarXMobHybridEnvCfg)    # noqa: E402
from zm_robot_omni_training.config.agents.sb3_sac_cfg import (SAC_HYPERPARAMS, TOTAL_TIMESTEPS,
                                                               SAVE_FREQ, EXPERIMENT_NAME,
                                                               SAC_STATE_HYPERPARAMS,
                                                               SAC_XMOB_HYPERPARAMS,
                                                               STATE_TOTAL_TIMESTEPS)    # noqa: E402
from zm_robot_omni_training.wrapper.env_wrapper import ZmRobotEnvWrapper    # noqa: E402
from zm_robot_omni_training.wrapper.residual_wrapper import ResidualActionVecEnv    # noqa: E402

# zm_robot's real safety-rated velocity limits (same vx/wz bounds the deployed
# non-holonomic COMPASS policy node clips to - see
# zm_robot_compass_policy/launch/zm_robot_compass_policy.launch.py's max_linear_x/
# max_angular_z defaults; vy mirrors vx since it's the same mecanum drivetrain in the
# other body-frame axis). SAC's policy is a tanh-squashed Gaussian that ALWAYS maps its
# output onto exactly [action_space.low, action_space.high] - unlike PPO, it requires a
# bounded action space to even be mathematically well-defined. Isaac Lab always declares
# single_action_space as Box(-inf, inf) (manager_based_rl_env.py's
# _configure_gym_env_spaces imposes no limits by design), and Sb3VecEnvWrapper's own
# fallback for that (clip to +-100) is only a generic placeholder - with
# HolonomicPerfectControlAction's process_actions() applying the raw action directly as
# (vx, vy, wz) in m/s and rad/s (no internal scaling), +-100 would let the actor's
# untrained output command speeds ~100x too fast, so this has to be overridden with the
# robot's real range before handing the env to SAC.
ACTION_LOW = np.array([-0.6, -0.6, -1.57], dtype=np.float32)
ACTION_HIGH = np.array([0.6, 0.6, 1.57], dtype=np.float32)


class TermStatsCallback(BaseCallback):
    """Every `every` env timesteps print + log how episodes ended since the last report
    (goal_reached / base_contact / time_out counts and mean distance to goal when they
    ended). SB3's own ep_rew/ep_len cannot tell a success from a crash."""

    def __init__(self, raw_env, every=1000):
        super().__init__()
        self.u = raw_env.unwrapped
        self.every = every
        self.next_report = every
        self.reset_window()

    def _on_training_start(self) -> None:
        # When resuming, num_timesteps starts at the checkpoint's step count; align the
        # first report to the next multiple of `every` instead of printing a backlog.
        self.next_report = (self.num_timesteps // self.every + 1) * self.every

    def reset_window(self):
        self.counts = {"goal_reached": 0, "base_contact": 0, "time_out": 0}
        self.progress = []

    def _on_step(self) -> bool:
        dones = self.locals["dones"]
        if dones.any():
            tm = self.u.termination_manager
            acc = getattr(self.u, "_ep_progress", None)
            for i in np.nonzero(dones)[0]:
                if acc is not None:
                    self.progress.append(float(acc[i]))
                for k in self.counts:
                    if bool(tm.get_term(k)[i]):
                        self.counts[k] += 1
                        break
        if self.num_timesteps >= self.next_report:
            n = max(sum(self.counts.values()), 1)
            line = (f"[TERM] steps={self.num_timesteps} episodes={sum(self.counts.values())} "
                    f"goal={self.counts['goal_reached']} contact={self.counts['base_contact']} "
                    f"timeout={self.counts['time_out']} goal_rate={self.counts['goal_reached'] / n:.2f} "
                    f"progress_m={np.mean(self.progress) if self.progress else float('nan'):+.2f}")
            print(line, flush=True)
            for k, v in self.counts.items():
                self.logger.record(f"term/{k}", v)
            self.reset_window()
            self.next_report += self.every
        return True


def main():
    state_only = args_cli.obs in ("state", "state_lidar")    # vector observations -> MlpPolicy
    xmob = args_cli.obs == "state_lidar_xmob"    # dict obs (vectors + X-Mobility + depth image)
    env_cfg = {"full": ZmRobotGoalReachingSACEnvCfg,
               "state": ZmRobotGoalReachingSACStateEnvCfg,
               "state_lidar": {"penalty": ZmRobotGoalReachingSACStateLidarPenaltyEnvCfg,
                           "hybrid": ZmRobotGoalReachingSACStateLidarHybridEnvCfg,
                           "progress": ZmRobotGoalReachingSACStateLidarEnvCfg}[args_cli.reward],
               "state_lidar_xmob": {"penalty": ZmRobotGoalReachingSACStateLidarXMobPenaltyEnvCfg,
                                "hybrid": ZmRobotGoalReachingSACStateLidarXMobHybridEnvCfg,
                                "progress": ZmRobotGoalReachingSACStateLidarXMobEnvCfg}[args_cli.reward]}[args_cli.obs]()
    if args_cli.num_envs is not None:
        env_cfg.scene.num_envs = args_cli.num_envs

    log_root_path = os.path.abspath(
        os.path.join("logs", "sb3", EXPERIMENT_NAME + ("" if args_cli.obs == "full" else "_" + args_cli.obs)))
    log_dir = os.path.join(log_root_path,
                           args_cli.run_name or datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
    if args_cli.seed is not None:
        env_cfg.seed = args_cli.seed
    print(f"[INFO] seed={args_cli.seed}", flush=True)
    print(f"[INFO] Logging experiment in directory: {log_dir}")

    raw_env = ZmRobotEnvWrapper(cfg=env_cfg)
    env = Sb3VecEnvWrapper(raw_env)
    env.action_space = gym.spaces.Box(low=ACTION_LOW, high=ACTION_HIGH, dtype=np.float32)
    if args_cli.residual_scale > 0:
        env = ResidualActionVecEnv(env, ACTION_LOW, ACTION_HIGH, args_cli.residual_scale)
        print(f"[INFO] residual action: final = clip(X-Mobility cmd + SAC), SAC range = "
              f"{args_cli.residual_scale} x full range", flush=True)

    total_timesteps = args_cli.total_timesteps or (STATE_TOTAL_TIMESTEPS if (state_only or xmob) else TOTAL_TIMESTEPS)

    if args_cli.resume_checkpoint:
        print(f"[INFO] Resuming from checkpoint: {args_cli.resume_checkpoint}")
        model = SAC.load(args_cli.resume_checkpoint, env=env, tensorboard_log=log_dir)
        # The replay buffer is not part of the checkpoint, so it is empty now. Without a
        # warm-up SB3 would start gradient updates at once on a handful of transitions
        # and could degrade the loaded policy. Collect RESUME_WARMUP steps first (SB3
        # acts uniformly at random while num_timesteps < learning_starts, and does no
        # updates), then continue training.
        RESUME_WARMUP = 2000
        model.learning_starts = model.num_timesteps + RESUME_WARMUP
        print(f"[INFO] Resumed at num_timesteps={model.num_timesteps}; "
              f"no updates until {model.learning_starts} (replay buffer warm-up).")
    else:
        if state_only:
            model = SAC("MlpPolicy", env, tensorboard_log=log_dir, verbose=1, seed=args_cli.seed,
                        **SAC_STATE_HYPERPARAMS)
        elif xmob:
            model = SAC("MultiInputPolicy", env, tensorboard_log=log_dir, verbose=1,
                        seed=args_cli.seed, **SAC_XMOB_HYPERPARAMS)
        else:
            model = SAC("MultiInputPolicy", env, tensorboard_log=log_dir, verbose=1,
                        seed=args_cli.seed, **SAC_HYPERPARAMS)

    checkpoint_callback = CheckpointCallback(save_freq=max(SAVE_FREQ // env.num_envs, 1),
                                             save_path=log_dir,
                                             name_prefix="model")

    try:
        model.learn(total_timesteps=total_timesteps, callback=[checkpoint_callback, TermStatsCallback(raw_env)],
                   reset_num_timesteps=args_cli.resume_checkpoint is None)
        model.save(os.path.join(log_dir, "model_final"))
    except KeyboardInterrupt:
        model.save(os.path.join(log_dir, "model_interrupted"))
    finally:
        env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
