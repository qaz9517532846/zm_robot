#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Engine-level parity check without ROS: feed the TensorRT engines the observations the
# PyTorch policy saw in the recorded rollout (teacher-forced inputs; the RSSM state is the
# engine's own) and compare the outputs.
#
#   python3 scripts/check_engines_offline.py [--actor hybrid] [--ref models/validation/ref_hybrid.npz]
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from zm_robot_omni_policy import preprocess as pp    # noqa: E402
from zm_robot_omni_policy.trt_runner import TrtEngine    # noqa: E402

ap = argparse.ArgumentParser()
root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models')
ap.add_argument('--model_dir', default=root)
ap.add_argument('--actor', default='hybrid')
ap.add_argument('--ref', default=os.path.join(root, 'validation', 'ref_hybrid.npz'))
ap.add_argument('--zero_route_first_step', action='store_true',
                help='feed X-Mobility an all-zero route at the very first step of the run '
                     '(tests the hypothesis that the simulator did so)')
args = ap.parse_args()

d = np.load(args.ref)
n = len(d['final'])
xm = TrtEngine(os.path.join(args.model_dir, 'xmobility.engine'))
ac = TrtEngine(os.path.join(args.model_dir, f'sac_residual_actor_{args.actor}.engine'))
# PyTorch residuals of every actor on this rollout's observations (models/validation/ref_actor_residuals.npz);
# the rollout itself was driven by the hybrid actor, so only its recorded residual is available otherwise.
actor_ref_path = os.path.join(os.path.dirname(args.ref), 'ref_actor_residuals.npz')
ref_res = np.load(actor_ref_path)[args.actor] if os.path.exists(actor_ref_path) else d['residual']
e_ps, e_base, e_res_tf, e_res_own = (np.zeros(n) for _ in range(4))
hist = samp = None
for k in range(n):
    if d['ep_step'][k] == 0:
        hist, samp = np.zeros((1, 1024), np.float32), np.zeros((1, 512), np.float32)
    prev = d['obs_actions'][k]
    route = d['obs_route'][k].reshape(1, 10, 4)
    if args.zero_route_first_step and k == 0:
        route = np.zeros_like(route)
    xo = xm({'image': pp.rgb_to_input(d['rgb'][k]), 'speed': d['obs_base_speed'][k][None],
             'route': route, 'action_input': np.array([[prev[0], prev[1], 0, 0, 0, prev[2]]], np.float32),
             'history_input': hist, 'sample_input': samp})
    hist, samp = xo['history_output'], xo['sample_output']
    a6 = xo['action_output'].reshape(6)
    base = np.array([np.clip(a6[0], -0.6, 0.6), 0.0, np.clip(a6[5], -1.57, 1.57)], np.float32)
    e_ps[k] = np.abs(xo['policy_state'].reshape(-1) - d['obs_xmobility'][k][:2048]).max()
    e_base[k] = np.abs(base - d['obs_xmobility'][k][2048:]).max()
    feeds = {key: d[f'obs_{key}'][k][None] for key in ('actions', 'base_speed', 'base_yaw_rate', 'goal_heading',
                                                        'lidar_ranges', 'route', 'depth_img')}
    # (a) actor alone: every input from the recording, including X-Mobility's output
    e_res_tf[k] = np.abs(ac({**feeds, 'xmobility': d['obs_xmobility'][k][None]})['residual'].reshape(3) - ref_res[k]).max()
    # (b) actor fed by the X-Mobility ENGINE's output
    own = np.concatenate([xo['policy_state'].reshape(-1), base])[None]
    e_res_own[k] = np.abs(ac({**feeds, 'xmobility': own})['residual'].reshape(3) - ref_res[k]).max()

first = np.asarray(d['ep_step']) == 0
rep = {
    'actor': args.actor,
    'steps': n,
    'actor_engine_vs_pytorch_residual_max_abs': float(e_res_tf.max()),
    'xmobility_engine_policy_state_max_abs': {'all': float(e_ps.max()), 'excluding_run_step_0_to_2': float(e_ps[3:].max())},
    'xmobility_engine_base_cmd_max_abs': {'all': float(e_base.max()), 'excluding_run_step_0_to_2': float(e_base[3:].max())},
    'both_engines_residual_max_abs': {'all': float(e_res_own.max()), 'excluding_run_step_0_to_2': float(e_res_own[3:].max())},
    'worst_steps_base_cmd': np.argsort(-e_base)[:5].tolist(),
}
print(json.dumps(rep, indent=1))
