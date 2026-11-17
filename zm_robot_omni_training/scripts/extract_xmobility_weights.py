#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Extract the inference weights (state_dict only, no optimizer/loop state) from the
# X-Mobility Lightning checkpoint into this repo, so training no longer reads files from
# the COMPASS checkout.
#
# Usage (inside the container, plain python is enough):
#   python3 scripts/extract_xmobility_weights.py \
#       --src /workspace/COMPASS/assets/x_mobility.ckpt

import argparse
import os

import torch

DEFAULT_DST = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                           "zm_robot_omni_training", "third_party", "x_mobility", "weights",
                           "x_mobility_policy.pt")

parser = argparse.ArgumentParser()
parser.add_argument("--src", required=True, help="x_mobility Lightning .ckpt (HF nvidia/X-Mobility)")
parser.add_argument("--dst", default=DEFAULT_DST)
args = parser.parse_args()

# weights_only=False: Lightning checkpoints carry non-tensor objects. Only run this on the
# checkpoint you downloaded yourself (same trust assumption as COMPASS's own loader).
ckpt = torch.load(args.src, map_location="cpu", weights_only=False)
state = {k.removeprefix("model."): v for k, v in ckpt["state_dict"].items()}
os.makedirs(os.path.dirname(os.path.abspath(args.dst)), exist_ok=True)
torch.save({"state_dict": state, "source": os.path.basename(args.src)}, args.dst)
n = sum(v.numel() for v in state.values())
print(f"saved {len(state)} tensors ({n / 1e6:.1f}M params) -> {os.path.abspath(args.dst)} "
      f"({os.path.getsize(args.dst) / 1e6:.0f} MB)")
