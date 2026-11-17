# SPDX-License-Identifier: Apache-2.0
#
# Stable-Baselines3 SAC config for zm_robot - the off-policy fallback after 5 PPO
# hyperparameter experiments (fixed LR bracketed 1e-4..1e-3, adaptive schedule with
# both tight and loose desired_kl) all failed to find a stable, improving configuration
# at the num_envs=6-forced batch size of 144 samples/update (DESIGN.md section 10
# step 8). SAC's replay buffer reuses past transitions across many gradient steps
# instead of throwing the batch away after one on-policy update, which is the specific
# property PPO can't offer under this GPU's num_envs ceiling.
#
# Numbers below are a first-pass starting point (SB3 SAC defaults where sensible),
# not yet validated against training.

SAC_HYPERPARAMS = dict(
    learning_rate=3.0e-4,
    # Replay buffer memory is the binding constraint here, not GPU: each transition
    # stores both obs and next_obs (SB3's DictReplayBuffer doesn't support
    # optimize_memory_usage), dominated by the uint8 camera image (3*320*512 bytes =
    # ~0.49 MiB). 5000 transitions * ~1.06 MiB (camera+lidar+vector, obs+next_obs)
    # ~= 5.3 GiB; 8000 -> ~8.5 GiB, still safe against this machine's ~26 GiB free host
    # RAM alongside Isaac Sim's own footprint.
    buffer_size=8_000,
    learning_starts=1_000,
    batch_size=64,
    tau=0.005,
    gamma=0.99,
    train_freq=1,
    # 6 envs add 6 transitions per env step; 1 gradient step per env step means only
    # 1 update per 6 new samples. 2 uses the buffer harder (still cheap vs the sim).
    gradient_steps=2,
    # "auto": SAC tunes its own entropy coefficient against a target entropy
    # (-action_dim by default) instead of a fixed entropy_coef - this is the exact
    # hand-tuning PPO's entropy_coef forced on us (DESIGN.md experiments 1-5) that SAC
    # does not need.
    ent_coef="auto",
    # Default target is -action_dim (-3): the auto-tuned coefficient kept falling
    # (0.93 -> 0.35 in exp 1) until the policy collapsed to ~zero actions. A higher target
    # entropy keeps the policy exploring for longer.
    target_entropy=-1.0,
    policy_kwargs=dict(net_arch=[256, 128]),
)

TOTAL_TIMESTEPS = 120_000
SAVE_FREQ = 10_000
EXPERIMENT_NAME = "zm_robot_omni_sac"

# State-only diagnostic variant (train_sac.py --obs state): tiny observations, so the
# replay buffer can be huge and batches large - none of the image-driven limits above.
SAC_STATE_HYPERPARAMS = dict(
    learning_rate=3.0e-4,
    buffer_size=300_000,
    learning_starts=5_000,
    batch_size=256,
    tau=0.005,
    gamma=0.99,
    train_freq=1,
    gradient_steps=1,
    ent_coef="auto",
    policy_kwargs=dict(net_arch=[256, 256]),
)
STATE_TOTAL_TIMESTEPS = 300_000


# State + LiDAR + frozen X-Mobility features + depth image (train_sac.py --obs
# state_lidar_xmob). Each stored observation is ~2.1k floats + a 64x104 uint8 image
# (~15 KB), obs and next_obs both kept, so the buffer is capped at 100k (~3 GB RAM).
SAC_XMOB_HYPERPARAMS = dict(SAC_STATE_HYPERPARAMS, buffer_size=100_000)
