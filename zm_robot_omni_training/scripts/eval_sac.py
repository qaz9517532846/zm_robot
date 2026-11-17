#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Roll out a trained SB3 SAC checkpoint and report what SB3's own logging does not:
# per-episode termination cause (goal_reached / base_contact / time_out), final distance
# to goal, and action-axis usage (vx vs vy vs wz).
#
# Usage:
#   python3 scripts/eval_sac.py --checkpoint logs/sb3/.../model_final.zip --num_envs 6 \
#       --steps 1000 --headless --enable_cameras [--random]

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", type=str, default=None)
parser.add_argument("--num_envs", type=int, default=6)
parser.add_argument("--steps", type=int, default=1000, help="vectorized env steps")
parser.add_argument("--random", action="store_true", help="uniform-random actions baseline")
parser.add_argument("--obs", choices=["full", "state", "state_lidar", "state_lidar_xmob"], default="full")
parser.add_argument("--reward", choices=["progress", "penalty", "hybrid"], default="progress",
                    help="progress: goal_heading_progress (exp 9, default). penalty: the old "
                    "goal_heading_alignment penalty (exp 7) - state_lidar only.")
parser.add_argument("--residual_scale", type=float, default=0.0,
                    help="evaluate a residual-form checkpoint (see train_sac.py)")
parser.add_argument("--zero", action="store_true", help="all-zero actions baseline")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
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
import torch    # noqa: E402
import gymnasium as gym    # noqa: E402
from stable_baselines3 import SAC    # noqa: E402
from isaaclab_rl.sb3 import Sb3VecEnvWrapper    # noqa: E402

from zm_robot_omni_training.config.zm_robot_env_cfg import (ZmRobotGoalReachingSACEnvCfg,
                                                             ZmRobotGoalReachingSACStateEnvCfg,
                                                             ZmRobotGoalReachingSACStateLidarEnvCfg,
                                                             ZmRobotGoalReachingSACStateLidarPenaltyEnvCfg,
                                                             ZmRobotGoalReachingSACStateLidarHybridEnvCfg,
                                                             ZmRobotGoalReachingSACStateLidarXMobEnvCfg,
                                                             ZmRobotGoalReachingSACStateLidarXMobPenaltyEnvCfg,
                                                             ZmRobotGoalReachingSACStateLidarXMobHybridEnvCfg)    # noqa: E402
from zm_robot_omni_training.mdp.utils import goal_pose_in_robot_frame    # noqa: E402
from zm_robot_omni_training.wrapper.env_wrapper import ZmRobotEnvWrapper    # noqa: E402
from zm_robot_omni_training.wrapper.residual_wrapper import ResidualActionVecEnv    # noqa: E402

ACTION_LOW = np.array([-0.6, -0.6, -1.57], dtype=np.float32)
ACTION_HIGH = np.array([0.6, 0.6, 1.57], dtype=np.float32)


def main():
    cfg = {"full": ZmRobotGoalReachingSACEnvCfg, "state": ZmRobotGoalReachingSACStateEnvCfg,
           "state_lidar": {"penalty": ZmRobotGoalReachingSACStateLidarPenaltyEnvCfg,
                           "hybrid": ZmRobotGoalReachingSACStateLidarHybridEnvCfg,
                           "progress": ZmRobotGoalReachingSACStateLidarEnvCfg}[args_cli.reward],
           "state_lidar_xmob": {"penalty": ZmRobotGoalReachingSACStateLidarXMobPenaltyEnvCfg,
                                "hybrid": ZmRobotGoalReachingSACStateLidarXMobHybridEnvCfg,
                                "progress": ZmRobotGoalReachingSACStateLidarXMobEnvCfg}[args_cli.reward]}[args_cli.obs]()
    cfg.scene.num_envs = args_cli.num_envs
    raw = ZmRobotEnvWrapper(cfg=cfg)
    env = Sb3VecEnvWrapper(raw)
    env.action_space = gym.spaces.Box(low=ACTION_LOW, high=ACTION_HIGH, dtype=np.float32)
    if args_cli.residual_scale > 0:
        env = ResidualActionVecEnv(env, ACTION_LOW, ACTION_HIGH, args_cli.residual_scale)
    model = None if (args_cli.random or args_cli.zero) else SAC.load(args_cli.checkpoint, env=env)

    u = raw.unwrapped
    cmd = u.command_manager.get_term("goal_pose")
    tm = u.termination_manager
    counts = {"goal_reached": 0, "base_contact": 0, "time_out": 0, "other": 0}
    final_head, final_dist, ep_len_list, start_dist, start_cur = [], [], [], [], np.zeros(args_cli.num_envs)
    act_abs = []
    ep_len = np.zeros(args_cli.num_envs)
    rm = u.reward_manager
    term_names = list(rm._term_names)
    ep_terms = np.zeros((args_cli.num_envs, len(term_names)))    # running per-episode sums
    term_rows, term_causes = [], []    # finished episodes: per-term sums + outcome

    obs = env.reset()
    for _ in range(args_cli.steps):
        if args_cli.zero:
            action = np.zeros((args_cli.num_envs, 3), dtype=np.float32)
        elif model is None:
            action = np.random.uniform(env.action_space.low, env.action_space.high, size=(args_cli.num_envs, 3))
        else:
            action, _ = model.predict(obs, deterministic=True)
        act_abs.append(np.abs(action).mean(axis=0))
        # distance BEFORE the step's auto-reset overwrites it is not available, so
        # record per-env distance each step and use the last pre-done value.
        pre_dist = torch.norm(goal_pose_in_robot_frame(u, "goal_pose", (0.0, 0.0))[:, :2], dim=1).detach().cpu().numpy().copy()
        pre_head = u.command_manager.get_command("goal_pose")[:, 3].abs().cpu().numpy().copy()
        for i in np.nonzero(ep_len == 0)[0]:
            start_cur[i] = pre_dist[i]
        obs, _, dones, _ = env.step(action)
        ep_len += 1
        # _step_reward holds the weighted per-SECOND value; * step_dt = actual reward added.
        ep_terms += (rm._step_reward * u.step_dt).cpu().numpy()
        for i in np.nonzero(dones)[0]:
            if tm.get_term("goal_reached")[i]:
                counts["goal_reached"] += 1
            elif tm.get_term("base_contact")[i]:
                counts["base_contact"] += 1
            elif tm.get_term("time_out")[i]:
                counts["time_out"] += 1
            else:
                counts["other"] += 1
            cause = ("goal_reached" if tm.get_term("goal_reached")[i] else
                     "base_contact" if tm.get_term("base_contact")[i] else
                     "time_out" if tm.get_term("time_out")[i] else "other")
            term_rows.append(ep_terms[i].copy())
            term_causes.append(cause)
            ep_terms[i] = 0.0
            final_dist.append(float(pre_dist[i]))
            start_dist.append(float(start_cur[i]))
            final_head.append(float(pre_head[i]))
            ep_len_list.append(ep_len[i])
            ep_len[i] = 0

    n = max(sum(counts.values()), 1)
    a = np.mean(act_abs, axis=0)
    print("=" * 60)
    print(f"EVAL ({'zero' if args_cli.zero else 'random' if model is None else args_cli.checkpoint})")
    print(f"episodes finished: {n}")
    for k, v in counts.items():
        print(f"  {k:13s}: {v:3d} ({100.0 * v / n:.0f}%)")
    def wilson(k, n, z=1.96):
        if n == 0:
            return float("nan"), float("nan")
        ph = k / n
        d = 1 + z * z / n
        c = (ph + z * z / (2 * n)) / d
        h = z * np.sqrt(ph * (1 - ph) / n + z * z / (4 * n * n)) / d
        return c - h, c + h

    lo, hi = wilson(counts["goal_reached"], n)
    print(f"success rate: {100.0 * counts['goal_reached'] / n:.1f}%   95% CI [{100 * lo:.1f}%, {100 * hi:.1f}%]   (n={n})")
    print(f"mean episode length : {np.mean(ep_len_list):.1f}")
    print(f"mean dist-to-goal at episode end: {np.mean(final_dist):.2f} m (min {np.min(final_dist):.2f})")
    print(f"mean dist-to-goal at episode START: {np.mean(start_dist):.2f} m  "
          f"(mean progress = {np.mean(start_dist) - np.mean(final_dist):+.2f} m)")
    fd, fh = np.array(final_dist), np.array(final_head)
    print(f"episodes ending within 0.5 m of goal: {(fd < 0.5).sum()}/{len(fd)}   "
          f"within 0.7 m: {(fd < 0.7).sum()}/{len(fd)}")
    print(f"heading error at end: mean {fh.mean():.2f} rad, within 0.5 rad: {(fh < 0.5).sum()}/{len(fh)}")
    print(f"within 0.5 m AND heading<0.5: {((fd < 0.5) & (fh < 0.5)).sum()}/{len(fd)}")
    print(f"sorted final dist: {np.round(np.sort(fd), 2).tolist()}")
    T = np.array(term_rows)
    causes = np.array(term_causes)
    print("per-term reward, mean SUM per episode (reward units, already weight*dt):")
    groups = [("ALL", np.ones(len(causes), bool))] + [
        (c, causes == c) for c in ("goal_reached", "time_out", "base_contact") if (causes == c).any()]
    print(f"  {'term':24s}" + "".join(f"{g + '(' + str(int(m.sum())) + ')':>18s}" for g, m in groups))
    for j, name in enumerate(term_names):
        print(f"  {name:24s}" + "".join(f"{T[m, j].mean():18.2f}" for _, m in groups))
    print(f"  {'TOTAL':24s}" + "".join(f"{T[m].sum(axis=1).mean():18.2f}" for _, m in groups))
    print(f"mean |action|  vx={a[0]:.3f}  vy={a[1]:.3f}  wz={a[2]:.3f}")
    print("=" * 60)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
