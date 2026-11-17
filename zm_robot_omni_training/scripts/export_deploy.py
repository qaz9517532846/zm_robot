#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Export the residual policy for deployment as two ONNX graphs (pure PyTorch, no simulator):
#
#   xmobility.onnx                 frozen X-Mobility, one recurrent step
#       in : image [1,3,320,512] float 0..1, speed [1,1], route [1,10,4], action_input [1,6],
#            history_input [1,1024], sample_input [1,512]
#       out: policy_state [1,2048], action_output [1,6], history_output, sample_output
#
#   sac_residual_actor_<reward>.onnx   SB3 SAC actor, deterministic, already un-scaled
#       in : actions [1,3], base_speed [1,1], base_yaw_rate [1,1], goal_heading [1,2],
#            lidar_ranges [1,36], route [1,40], xmobility [1,2051], depth_img [1,1,64,104] (0..255)
#       out: residual [1,3]   (m/s, m/s, rad/s; inside the residual range)
#
# The command applied to the robot is clip(base + residual), base = [vx, 0, wz] clamped,
# exactly as wrapper/residual_wrapper.py does in training/evaluation.
#
# Usage (inside the container, from the repo root):
#   python3 scripts/export_deploy.py --out deploy_export \
#       --checkpoint hybrid=logs/sb3/.../resid_hybrid_seed101/model_150000_steps.zip [...]

import argparse
import json
import os

import numpy as np
import torch
from torch import nn
from stable_baselines3 import SAC

from zm_robot_omni_training.models.xmobility import (ACTION_DIM, INPUT_IMAGE_SIZE, ROUTE_VEC_SIZE,
                                                      build_xmobility)

OBS_KEYS = ["actions", "base_speed", "base_yaw_rate", "goal_heading", "lidar_ranges", "route",
            "xmobility", "depth_img"]


class XMobStep(nn.Module):
    """One X-Mobility step; mirrors XMobilityRunner._step_chunk for batch size 1."""

    def __init__(self, model):
        super().__init__()
        self.model = model

    def forward(self, image, speed, route, action_input, history_input, sample_input):
        b = image.shape[0]
        batch = {
            "image": image.unsqueeze(1),
            "speed": speed.reshape(b, 1, 1),
            "route": route.reshape(b, 1, -1, ROUTE_VEC_SIZE),
            "history": history_input,
            "sample": sample_input,
            "action": action_input,
        }
        action, _, history, sample, _, _, _ = self.model.inference(batch, False, False, False)
        latent = torch.cat([history, sample], dim=-1)
        route_feat = self.model.action_policy.route_encoder(batch["route"].reshape(b, -1, ROUTE_VEC_SIZE))
        policy_state = self.model.action_policy.policy_state_fusion(latent, route_feat)
        return policy_state, action.reshape(b, ACTION_DIM), history, sample


class ActorStep(nn.Module):
    """SB3 SAC actor, deterministic (tanh(mean)), un-scaled to the residual range like predict()."""

    def __init__(self, sac):
        super().__init__()
        self.actor = sac.policy.actor
        self.register_buffer("low", torch.as_tensor(sac.action_space.low, dtype=torch.float32))
        self.register_buffer("high", torch.as_tensor(sac.action_space.high, dtype=torch.float32))

    def forward(self, actions, base_speed, base_yaw_rate, goal_heading, lidar_ranges, route,
                xmobility, depth_img):
        obs = dict(zip(OBS_KEYS, (actions, base_speed, base_yaw_rate, goal_heading, lidar_ranges,
                                  route, xmobility, depth_img)))
        a = self.actor(obs, deterministic=True)    # in [-1, 1]
        return self.low + 0.5 * (a + 1.0) * (self.high - self.low)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", default="deploy_export")
    p.add_argument("--checkpoint", action="append", default=[], help="name=path/to/model.zip")
    p.add_argument("--device", default="cuda")
    args = p.parse_args()
    os.makedirs(args.out, exist_ok=True)
    dev = args.device
    torch.manual_seed(0)
    meta = {"image_hw": list(INPUT_IMAGE_SIZE), "obs_keys": OBS_KEYS, "actors": {}}

    # ---- X-Mobility
    xm = XMobStep(build_xmobility(dev)).eval()
    h, w = INPUT_IMAGE_SIZE
    xin = (torch.rand(1, 3, h, w, device=dev), torch.rand(1, 1, device=dev),
           torch.rand(1, 10, 4, device=dev), torch.zeros(1, 6, device=dev),
           torch.zeros(1, 1024, device=dev), torch.zeros(1, 512, device=dev))
    xnames = ["image", "speed", "route", "action_input", "history_input", "sample_input"]
    xout = ["policy_state", "action_output", "history_output", "sample_output"]
    xpath = os.path.join(args.out, "xmobility.onnx")
    with torch.no_grad():
        ref = xm(*xin)
        torch.onnx.export(xm, xin, xpath, input_names=xnames, output_names=xout, dynamo=False,
                          opset_version=17)
    print(f"[export] {xpath}  {os.path.getsize(xpath) / 1e6:.1f} MB", flush=True)

    import onnxruntime as ort
    sess = ort.InferenceSession(xpath, providers=["CPUExecutionProvider"])
    got = sess.run(xout, {n: t.cpu().numpy() for n, t in zip(xnames, xin)})
    meta["xmobility_onnx_vs_torch_max_abs"] = {
        n: float(np.abs(g - r.cpu().numpy()).max()) for n, g, r in zip(xout, got, ref)}
    print("[parity] xmobility onnxruntime(CPU) vs torch:", meta["xmobility_onnx_vs_torch_max_abs"], flush=True)

    # ---- SAC actors
    for item in args.checkpoint:
        name, path = item.split("=", 1)
        sac = SAC.load(path, device=dev)
        act = ActorStep(sac).to(dev).eval()
        space = sac.observation_space
        ain = []
        for k in OBS_KEYS:
            shp = (1, *space[k].shape)
            ain.append(torch.rand(shp, device=dev) * (255.0 if k == "depth_img" else 1.0))
        apath = os.path.join(args.out, f"sac_residual_actor_{name}.onnx")
        with torch.no_grad():
            aref = act(*ain)
            # reference through SB3's own predict() (numpy in, un-scaled action out)
            pred, _ = sac.predict({k: t.cpu().numpy() for k, t in zip(OBS_KEYS, ain)}, deterministic=True)
            torch.onnx.export(act, tuple(ain), apath, input_names=OBS_KEYS, output_names=["residual"],
                              dynamo=False, opset_version=17)
        s2 = ort.InferenceSession(apath, providers=["CPUExecutionProvider"])
        agot = s2.run(["residual"], {k: t.cpu().numpy() for k, t in zip(OBS_KEYS, ain)})[0]
        meta["actors"][name] = {
            "checkpoint": path,
            "residual_low": sac.action_space.low.tolist(),
            "residual_high": sac.action_space.high.tolist(),
            "obs_shapes": {k: list(space[k].shape) for k in OBS_KEYS},
            "onnx_vs_torch_max_abs": float(np.abs(agot - aref.cpu().numpy()).max()),
            "wrapper_vs_sb3_predict_max_abs": float(np.abs(aref.cpu().numpy() - pred).max()),
        }
        print(f"[export] {apath}  {os.path.getsize(apath) / 1e6:.2f} MB  {meta['actors'][name]}", flush=True)

    json.dump(meta, open(os.path.join(args.out, "export_meta.json"), "w"), indent=1)
    print("[export] done", flush=True)


if __name__ == "__main__":
    main()
