#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Record fixed-length overhead clips of several trained policies in ONE simulator run: one
# parallel environment per model, each driven deterministically by its own checkpoint. Used
# for the side-by-side comparison video (paper/make_grid_video.py).
#
#   --group lidar   SAC on state + LiDAR:          --model <label>=<checkpoint.zip> ...
#   --group xmob    X-Mobility base + SAC residual: --model <label>=<checkpoint.zip> ...
#                   and <label>=none for the frozen X-Mobility alone (zero residual)
#
# Before recording, every environment is given the SAME start pose and goal as environment 0
# (first episode only; later episodes are sampled independently). Whether that worked is
# written to each clip's JSON ("synced").
#
# Output per model: <outdir>/<label>.mp4 (20 fps = 2x real time) and <outdir>/<label>.json.
#
# Runs on the Isaac Lab of the IsaacLab-Arena virtual environment (the container the policies
# were trained in no longer exists):
#   cd ~/catkin_ws/src/zm_robot/zm_robot_omni_training
#   PYTHONPATH=$PWD:$PWD/.pydeps HF_HUB_OFFLINE=1 OMNI_KIT_ACCEPT_EULA=YES \
#     ~/nvidia/IsaacLab-Arena/.venv/bin/python scripts/record_clip.py --group lidar --model ...

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--group", choices=["lidar", "xmob"], required=True)
parser.add_argument("--model", action="append", required=True, help="<label>=<checkpoint.zip | none>")
parser.add_argument("--residual_scale", type=float, default=0.5)
parser.add_argument("--outdir", type=str, required=True)
parser.add_argument("--steps", type=int, default=600)
parser.add_argument("--seed", type=int, default=7)
parser.add_argument("--cam_height", type=float, default=6.5)
parser.add_argument("--size", type=int, default=384)
parser.add_argument("--start", type=str, default="3.696,-2.822,1.51",
                    help="first-episode start pose x,y,yaw in the environment frame ('' = copy environment 0)")
parser.add_argument("--goal", type=str, default="1.591,-4.471,2.799",
                    help="first-episode goal x,y,heading in the environment frame")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.kit_args = (args_cli.kit_args + " " if getattr(args_cli, "kit_args", "") else "") + "--/persistent/UJITSO/geometry=false"
args_cli.enable_cameras = True    # not a CLI flag on this Isaac Lab version; still a launcher setting
simulation_app = AppLauncher(args_cli).app

import json    # noqa: E402
import os    # noqa: E402
import traceback    # noqa: E402
import types as _types    # noqa: E402

import gymnasium as gym    # noqa: E402
import imageio    # noqa: E402
import numpy as np    # noqa: E402
import torch    # noqa: E402
import warp as wp    # noqa: E402
from PIL import Image, ImageDraw    # noqa: E402
from stable_baselines3 import SAC    # noqa: E402

import isaaclab.sim as sim_utils    # noqa: E402
import isaaclab.utils as _il_utils    # noqa: E402
from isaaclab.sensors import CameraCfg, TiledCameraCfg    # noqa: E402
from isaaclab.sim import SimulationCfg as _SimulationCfg    # noqa: E402
from isaaclab.utils.configclass import configclass as _configclass    # noqa: E402
from isaaclab_rl.sb3 import Sb3VecEnvWrapper    # noqa: E402

# Compatibility with this (newer) Isaac Lab, without touching the environment package:
#  * `from isaaclab.utils import configclass` used to return the decorator; here the lazily
#    loaded attribute is shadowed by the submodule of the same name.
#  * `SimulationCfg.render` no longer exists; the environment only sets two cosmetic renderer
#    options on it.
_il_utils.configclass = _configclass
if not hasattr(_SimulationCfg, "render"):
    _SimulationCfg.render = _types.SimpleNamespace()

from zm_robot_omni_training.config.zm_robot_env_cfg import (    # noqa: E402
    ZmRobotGoalReachingSACStateLidarEnvCfg, ZmRobotGoalReachingSACStateLidarXMobEnvCfg)
from zm_robot_omni_training.mdp.utils import goal_pose_in_robot_frame    # noqa: E402
from zm_robot_omni_training.wrapper.env_wrapper import ZmRobotEnvWrapper    # noqa: E402
from zm_robot_omni_training.wrapper.residual_wrapper import ResidualActionVecEnv    # noqa: E402

ACTION_LOW = np.array([-0.6, -0.6, -1.57], dtype=np.float32)
ACTION_HIGH = np.array([0.6, 0.6, 1.57], dtype=np.float32)
FOCAL, APERTURE = 10.0, 20.955


def t(x):
    return x if isinstance(x, torch.Tensor) else wp.to_torch(x)


def sync_first_episode(u, cmd):
    """Give every environment the same first start pose and goal: the fixed --start / --goal
    (so that separate simulator runs match too), or environment 0's if --start is empty."""
    robot = u.scene["robot"]
    n = u.num_envs
    origins = u.scene.env_origins
    pos, quat = t(robot.data.root_pos_w).clone(), t(robot.data.root_quat_w).clone()
    if args_cli.start:
        sx, sy, syaw = (float(v) for v in args_cli.start.split(","))
        gx, gy, gh = (float(v) for v in args_cli.goal.split(","))
        new_pos = origins.clone()
        new_pos[:, 0] += sx
        new_pos[:, 1] += sy
        new_pos[:, 2] = pos[:, 2]
        new_quat = torch.zeros(n, 4, device=pos.device)    # xyzw
        new_quat[:, 2], new_quat[:, 3] = float(np.sin(syaw / 2)), float(np.cos(syaw / 2))
        cmd.pos_command_w[:, 0] = origins[:, 0] + gx
        cmd.pos_command_w[:, 1] = origins[:, 1] + gy
        cmd.heading_command_w[:] = gh
    else:
        new_pos = origins + (pos[0] - origins[0])
        new_quat = quat[0].unsqueeze(0).repeat(n, 1)
        cmd.pos_command_w[:] = origins + (cmd.pos_command_w[0] - origins[0])
        cmd.heading_command_w[:] = cmd.heading_command_w[0]
    robot.write_root_pose_to_sim(torch.cat([new_pos, new_quat], dim=-1))
    robot.write_root_velocity_to_sim(torch.zeros(n, 6, device=pos.device))
    cmd.locked_heading_w[:] = 2 * torch.atan2(new_quat[:, 2], new_quat[:, 3])


def main():
    specs = [m.split("=", 1) for m in args_cli.model]
    n = len(specs)
    cfg = ZmRobotGoalReachingSACStateLidarEnvCfg() if args_cli.group == "lidar" else ZmRobotGoalReachingSACStateLidarXMobEnvCfg()
    cfg.scene.num_envs = n
    cfg.seed = args_cli.seed
    size = args_cli.size
    cfg.scene.overhead_cam = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/OverheadCam", update_period=0.0, height=size, width=size, data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(focal_length=FOCAL, focus_distance=400.0, horizontal_aperture=APERTURE,
                                         clipping_range=(0.1, 50.0)),
        offset=CameraCfg.OffsetCfg(pos=(0.0, 0.0, args_cli.cam_height), rot=(0.70710678, 0.0, -0.70710678, 0.0),
                                   convention="world"))
    raw = ZmRobotEnvWrapper(cfg=cfg)
    env = Sb3VecEnvWrapper(raw)
    env.action_space = gym.spaces.Box(low=ACTION_LOW, high=ACTION_HIGH, dtype=np.float32)
    if args_cli.group == "xmob":
        env = ResidualActionVecEnv(env, ACTION_LOW, ACTION_HIGH, args_cli.residual_scale)
    models = [None if path == "none" else SAC.load(path, device="cuda") for _, path in specs]
    u = raw.unwrapped
    tm = u.termination_manager
    cam = u.scene["overhead_cam"]
    cmd = u.command_manager.get_term("goal_pose")
    robot = u.scene["robot"]
    s = size * FOCAL / (APERTURE * args_cli.cam_height)    # pixels per metre
    print(f"[clip] environment up: {n} envs, group {args_cli.group}", flush=True)

    def to_px(x, y):    # camera looks straight down: image up = -X, right = +Y (see record_video.py)
        return (size / 2 + s * y, size / 2 + s * x)

    def act(obs):
        out = np.zeros((n, 3), np.float32)
        for i, m in enumerate(models):
            if m is None:
                continue
            o = {k: v[i:i + 1] for k, v in obs.items()} if isinstance(obs, dict) else obs[i:i + 1]
            out[i] = m.predict(o, deterministic=True)[0][0]
        return out

    obs = env.reset()
    synced = False
    try:
        sync_first_episode(u, cmd)
        obs, _, _, _ = env.step(np.zeros((n, 3), np.float32))    # one idle step: sensors + observations at the new pose
        synced = True
    except Exception:    # pylint: disable=broad-except
        print("[clip] could not synchronise the first episode; starts and goals are independent", flush=True)
        traceback.print_exc()
        obs = env.reset()

    os.makedirs(args_cli.outdir, exist_ok=True)
    writers = [imageio.get_writer(os.path.join(args_cli.outdir, f"{label}.mp4"), fps=20, codec="libx264",
                                  quality=7, macro_block_size=None) for label, _ in specs]
    metas = [{"label": label, "group": args_cli.group, "checkpoint": path, "seed": args_cli.seed, "synced": synced,
              "steps": [], "episodes": []} for label, path in specs]
    ep = [0] * n
    ep_step = [0] * n
    trails = [[] for _ in range(n)]
    for k in range(args_cli.steps):
        action = act(obs)
        frames = cam.data.output["rgb"][..., :3].cpu().numpy().astype(np.uint8)
        origins = u.scene.env_origins[:, :2]
        rpos_all = (t(robot.data.root_pos_w)[:, :2] - origins).cpu().numpy()
        gpos_all = (cmd.pos_command_w[:, :2] - origins).cpu().numpy()
        dist_all = torch.norm(goal_pose_in_robot_frame(u, "goal_pose", (0.0, 0.0))[:, :2], dim=1).cpu().numpy()
        quat = t(robot.data.root_quat_w)
        yaw_all = (2 * torch.atan2(quat[:, 2], quat[:, 3])).cpu().numpy()
        for i in range(n):
            rpos, gpos, yaw = rpos_all[i], gpos_all[i], float(yaw_all[i])
            if ep_step[i] == 0:
                metas[i]["episodes"].append({"start": rpos.round(3).tolist(), "goal": gpos.round(3).tolist(),
                                             "goal_heading": float(cmd.heading_command_w[i]), "start_yaw": round(yaw, 3)})
                trails[i] = []
            trails[i].append(to_px(*rpos))
            img = Image.fromarray(frames[i])
            d = ImageDraw.Draw(img)
            gp, rp = to_px(*gpos), to_px(*rpos)
            r = 0.5 * s
            d.ellipse([gp[0] - r, gp[1] - r, gp[0] + r, gp[1] + r], outline=(255, 0, 0), width=2)
            d.ellipse([gp[0] - 2, gp[1] - 2, gp[0] + 2, gp[1] + 2], fill=(255, 0, 0))
            if len(trails[i]) > 1:
                d.line(trails[i], fill=(255, 210, 0), width=2)
            tip = to_px(rpos[0] + 0.6 * np.cos(yaw), rpos[1] + 0.6 * np.sin(yaw))
            d.line([rp, tip], fill=(0, 230, 0), width=3)
            d.ellipse([rp[0] - 4, rp[1] - 4, rp[0] + 4, rp[1] + 4], fill=(0, 230, 0))
            writers[i].append_data(np.asarray(img))

        obs, _, dones, _ = env.step(action)
        for i in range(n):
            ep_step[i] += 1
            cause = ""
            if dones[i]:
                cause = next((c for c in ("goal_reached", "base_contact", "time_out") if bool(tm.get_term(c)[i])), "other")
                metas[i]["episodes"][-1].update({"length": ep_step[i], "cause": cause, "final_dist": round(float(dist_all[i]), 3)})
                ep[i] += 1
                ep_step[i] = 0
            metas[i]["steps"].append({"ep": ep[i] - (1 if dones[i] else 0), "dist": round(float(dist_all[i]), 3), "cause": cause})
        if k % 100 == 0:
            print(f"[clip] step {k}/{args_cli.steps}  episodes finished per env: {ep}", flush=True)
    for w, m, (label, _) in zip(writers, metas, specs):
        w.close()
        json.dump(m, open(os.path.join(args_cli.outdir, f"{label}.json"), "w"))
    print(f"[clip] done: {n} clips x {args_cli.steps} frames -> {args_cli.outdir} (synced={synced})", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
