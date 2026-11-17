# Vendored X-Mobility (inference-side model code)

Source: NVlabs X-MOBILITY, `x_mobility-0.1.0-py3-none-any.whl` as shipped in the COMPASS
repo (`x_mobility/`), Apache-2.0 (see LICENSE). Only the model package files needed to
build and run the network are copied; the only edit is the import path
(`model.x_mobility.*` -> `zm_robot_omni_training.third_party.x_mobility.*`).

- `x_mobility_config.gin`: copy of COMPASS `configs/x_mobility_config.gin` (the gin
  bindings the checkpoint was trained with: DINOv2 image encoder, RSSM 1024+512,
  policy_state_dim 2048, self-attention fusion, MLP policy head, 6-d action).
- `weights/x_mobility_policy.pt`: `state_dict` only (optimizer states dropped) extracted from
  `x_mobility-nav2-semantic_action_path.ckpt` (HF nvidia/X-Mobility). Not committed anywhere
  upstream; regenerate with `scripts/extract_xmobility_weights.py`.
