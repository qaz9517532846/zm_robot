# SPDX-License-Identifier: Apache-2.0
#
# RSL-RL PPO config for zm_robot. First pass (DESIGN.md section 10 step 4/5): symmetric
# actor-critic (both see policy+camera+lidar, no asymmetric "privileged" critic yet),
# CNN branches for the two image-shaped groups (camera, lidar), MLP for the vector group
# (policy). Numbers below are untuned starting points, not validated against training.

from isaaclab_rl.rsl_rl import (
    RslRlCNNModelCfg,
    RslRlOnPolicyRunnerCfg,
    RslRlPpoAlgorithmCfg,
)

# One CNN per 2D obs group (camera: 3ch RGB 320x512, lidar: 1ch 100x100) - same
# hyperparameters broadcast to both groups, input_channels is inferred per-group from
# the actual observation tensor shape.
_CNN_CFG = RslRlCNNModelCfg.CNNCfg(
    output_channels=[16, 32],
    kernel_size=[3, 3],
    activation="elu",
    max_pool=[True, True],
    norm="batch",
    global_pool="avg",
)

ACTOR_CFG = RslRlCNNModelCfg(
    obs_normalization=True,
    hidden_dims=[256, 128],
    # init_std lowered 1.0 -> 0.5: a 500-iteration run showed Mean action std pinned
    # at 1.00-1.01 for the entire run (never shrinking), which washed out a 43-
    # iteration streak of genuine goal_reached successes (DESIGN.md section 10 step
    # 8). Starting smaller gives the learnable std room to move and reduces the
    # chance new good behavior gets drowned by exploration noise immediately.
    distribution_cfg=RslRlCNNModelCfg.GaussianDistributionCfg(init_std=0.5),
    cnn_cfg=_CNN_CFG,
    activation="elu",
)

CRITIC_CFG = RslRlCNNModelCfg(
    obs_normalization=True,
    hidden_dims=[256, 128],
    cnn_cfg=_CNN_CFG,
    activation="elu",
)

ALGORITHM_CFG = RslRlPpoAlgorithmCfg(
    value_loss_coef=1.0,
    use_clipped_value_loss=True,
    clip_param=0.2,
    # Bracketing experiment: lr=1e-3 (unthrottled) diverged (reward -101->-923,
    # std climbed the wrong way); lr=1e-4 + entropy=0.0 went the other way - 201
    # iterations completely flat (reward stuck in a -161..-167 band, std frozen at
    # 0.50, zero goal_reached) - stable but not learning at all, "too cold." The
    # midpoint lr=3e-4 + entropy=0.0005 ALSO diverged (confirmed at iter 76: reward
    # -155->-247->-313->-368 over 25 iterations, no recovery) - just slower than
    # lr=1e-3. Conclusion: no learning_rate value in this bracket is both stable and
    # learning, which points at the 144-sample batch (num_envs=6 * num_steps_per_env=24)
    # itself being too noisy for PPO, not just a bad LR pick.
    #
    # Tried shrinking num_mini_batches (4->2->1) to cut gradient noise directly
    # instead of moving the LR again - both OOM immediately. At this point the
    # rollout buffer already has the GPU at ~14.4/15.46 GiB, and the learning
    # phase's CNN forward+backward over a bigger per-step shard (72 or 144 images
    # instead of 36) doesn't fit. So num_mini_batches is stuck at the known-safe 4;
    # there is no memory headroom left to de-noise the gradient that way.
    #
    # Fifth pass: stop hand-bracketing a fixed LR and go back to schedule="adaptive",
    # but fix why it collapsed the first time instead of abandoning it. Read RSL-RL's
    # own implementation (algorithms/ppo.py): it halves the LR whenever
    # kl_mean > desired_kl*2, floor 1e-5, ceiling 1e-2 - with desired_kl=0.01 (the
    # original value) that threshold is 0.02, which this tiny noisy 144-sample batch
    # blows past almost every update, so LR ratchets straight down to the 1e-5 floor
    # and never recovers (the exact behavior measured in the first attempt). Loosening
    # desired_kl to 0.05 raises that threshold to 0.10, giving the schedule enough
    # slack to tolerate this batch's inherent noise instead of reading it as runaway
    # policy change every step. Starting lr=1e-3 (ceiling of the earlier bracket) so
    # the adaptive mechanism actually gets to try the higher end and throttle down
    # on its own only if updates are genuinely too aggressive, rather than us presetting
    # a fixed value that's either too hot or too cold for the whole run.
    entropy_coef=0.0003,
    num_learning_epochs=5,
    num_mini_batches=4,
    learning_rate=1.0e-3,
    schedule="adaptive",
    gamma=0.99,
    lam=0.95,
    desired_kl=0.05,
    max_grad_norm=1.0,
)

ZM_ROBOT_PPO_RUNNER_CFG = RslRlOnPolicyRunnerCfg(
    seed=42,
    # Tried doubling this to 48 (hoping for a less-noisy batch without touching
    # num_envs) but it OOMs: PPO's learning phase re-runs the CNN forward+backward
    # over the *stored* rollout images each epoch, so memory scales with
    # num_envs*num_steps_per_env in a much steeper way than just raw observation
    # storage would suggest. Staying at the known-safe 24 for now; revisit only
    # alongside a smaller CNN/lower camera resolution if a bigger batch is needed.
    num_steps_per_env=24,
    max_iterations=1000,
    save_interval=50,
    experiment_name="zm_robot_omni",
    empirical_normalization=False,    # deprecated in rsl_rl >= 4.0 - obs_normalization above is used instead
    obs_groups={
        "actor": ["policy", "camera", "lidar"],
        "critic": ["policy", "camera", "lidar"],
    },
    actor=ACTOR_CFG,
    critic=CRITIC_CFG,
    algorithm=ALGORITHM_CFG,
)
