# SPDX-License-Identifier: Apache-2.0
#
# Frozen X-Mobility as a per-step feature/prior extractor for the zm_robot SAC pipeline.
# Everything it needs lives inside this repo (third_party/x_mobility: vendored model code,
# gin config, state_dict-only weights) - nothing is imported from the COMPASS checkout.
#
# X-Mobility facts this wrapper relies on (verified in the vendored code / checkpoint):
#   * It sees RGB ONLY (DINOv2-small encoder, 320x512 input) + scalar speed + a 10x4 route
#     vector. It has NO depth input; depth must be fed to the policy by a separate branch.
#   * It is recurrent (RSSM, history 1024 + sample 512) and must be reset per env.
#   * Its 6-d action is a twist [vx, vy, vz, wx, wy, wz]; trained on differential-drive
#     robots, so only index 0 (vx) and 5 (wz) are meaningful.

from __future__ import annotations

import os
from contextlib import contextmanager

import gin
import torch
from transformers import Dinov2Config, Dinov2Model

THIRD_PARTY = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "third_party",
                           "x_mobility")
GIN_FILE = os.path.join(THIRD_PARTY, "x_mobility_config.gin")
WEIGHTS = os.path.join(THIRD_PARTY, "weights", "x_mobility_policy.pt")

INPUT_IMAGE_SIZE = (320, 512)    # (H, W) the checkpoint was trained with
POLICY_STATE_DIM = 2048
ACTION_DIM = 6
ROUTE_VEC_SIZE = 4


@contextmanager
def _offline_dinov2():
    """ImageDINOEncoder calls Dinov2Model.from_pretrained('facebook/dinov2-small'), which
    would download weights. The checkpoint already contains every DINOv2 tensor, so build
    the same architecture from its config instead and let load_state_dict fill it."""
    original = Dinov2Model.from_pretrained

    def build(_name, *args, **kwargs):
        cfg = Dinov2Config(hidden_size=384,
                           num_hidden_layers=12,
                           num_attention_heads=6,
                           mlp_ratio=4,
                           image_size=518,
                           patch_size=14,
                           layer_norm_eps=1e-6,
                           qkv_bias=True,
                           layerscale_value=1.0,
                           use_swiglu_ffn=False,
                           drop_path_rate=0.0,
                           output_attentions=kwargs.get("output_attentions", False))
        # ImageDINOEncoder reads outputs.attentions[-1]; the default SDPA kernel does not
        # return attention maps, so force the eager implementation.
        cfg._attn_implementation = "eager"
        return Dinov2Model(cfg)

    Dinov2Model.from_pretrained = build
    try:
        yield
    finally:
        Dinov2Model.from_pretrained = original


def build_xmobility(device: str = "cuda"):
    # Imported here so that importing this module does not register gin configurables twice.
    from zm_robot_omni_training.third_party.x_mobility.x_mobility import XMobility

    gin.parse_config_file(GIN_FILE, skip_unknown=True)
    with _offline_dinov2():
        model = XMobility()
    ckpt = torch.load(WEIGHTS, map_location="cpu", weights_only=False)
    model.load_state_dict(ckpt["state_dict"], strict=True)    # strict: a mismatch is a bug
    model.eval().requires_grad_(False)
    return model.to(device)


class XMobilityRunner:
    """Stateful, batched, frozen X-Mobility. One RSSM state per parallel env."""

    def __init__(self, num_envs: int, device: str = "cuda", model=None,
                 chunk: int = int(os.environ.get("XMOB_CHUNK", 4))):
        self.model = model if model is not None else build_xmobility(device)
        self.device = device
        self.num_envs = num_envs
        self.chunk = chunk
        self.history = torch.zeros(num_envs, self.model.rssm.hidden_state_dim, device=device)
        self.sample = torch.zeros(num_envs, self.model.rssm.state_dim, device=device)
        self.prev_action = torch.zeros(num_envs, ACTION_DIM, device=device)

    def reset(self, env_ids=None):
        """Zero the recurrent state of finished envs (same as COMPASS: history/sample/action=0)."""
        if env_ids is None:
            env_ids = slice(None)
        # step() runs under inference_mode, so the stored states are inference tensors that
        # cannot be edited in place outside it; clone (a normal tensor) before zeroing.
        self.history = self.history.clone()
        self.sample = self.sample.clone()
        self.prev_action = self.prev_action.clone()
        self.history[env_ids] = 0.0
        self.sample[env_ids] = 0.0
        self.prev_action[env_ids] = 0.0

    @torch.inference_mode()
    def _step_chunk(self, rgb, speed, route_flat, history, sample, prev_action):
        b = rgb.shape[0]
        batch = {
            "image": (rgb.permute(0, 3, 1, 2).float() / 255.0).unsqueeze(1),
            "speed": speed.reshape(b, 1, 1).float(),
            "route": route_flat.reshape(b, 1, -1, ROUTE_VEC_SIZE).float(),
            "history": history,
            "sample": sample,
            "action": prev_action,
        }
        action, _, history, sample, _, _, _ = self.model.inference(batch, False, False, False)
        action = action.reshape(b, ACTION_DIM)
        latent = torch.cat([history, sample], dim=-1)
        route_feat = self.model.action_policy.route_encoder(
            batch["route"].reshape(b, -1, ROUTE_VEC_SIZE))
        policy_state = self.model.action_policy.policy_state_fusion(latent, route_feat)
        return policy_state, action, history, sample

    @torch.inference_mode()
    def step(self, rgb_bhwc_uint8: torch.Tensor, speed: torch.Tensor, route_flat: torch.Tensor):
        """rgb [B,H,W,3] uint8 (H,W = INPUT_IMAGE_SIZE); speed [B,1] m/s; route_flat [B,40].

        Returns policy_state [B,2048] and X-Mobility's own 6-d command [B,6].

        Runs in chunks of `chunk` envs: the DINOv2 encoder keeps all 12 layers' attention
        maps (1370x1370 per head), ~8 GB for 16 envs at once, which together with the
        renderer does not fit in a 16 GB GPU (measured OOM)."""
        outs = []
        for i in range(0, rgb_bhwc_uint8.shape[0], self.chunk):
            sl = slice(i, i + self.chunk)
            outs.append(self._step_chunk(rgb_bhwc_uint8[sl], speed[sl], route_flat[sl],
                                         self.history[sl], self.sample[sl], self.prev_action[sl]))
        policy_state, action, history, sample = (torch.cat(x, dim=0) for x in zip(*outs))
        self.history, self.sample = history, sample
        if os.environ.get("XMOB_EMPTY_CACHE"):
            torch.cuda.empty_cache()
        return policy_state, action

    def set_executed_action(self, vx: torch.Tensor, vy: torch.Tensor, wz: torch.Tensor):
        """Feed the command that was actually applied back into the RSSM as the next
        step's previous action (COMPASS feeds `final_actions`)."""
        a = torch.zeros(vx.shape[0], ACTION_DIM, device=vx.device)
        a[:, 0], a[:, 1], a[:, 5] = vx, vy, wz
        self.prev_action = a
