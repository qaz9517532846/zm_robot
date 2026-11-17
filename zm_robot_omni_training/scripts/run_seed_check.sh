#!/usr/bin/env bash
# Re-run the exp 9 configuration with fixed seeds and evaluate each 150k checkpoint
# with the same large-sample protocol as before. Runs sequentially (one sim at a time).
cd "$(dirname "${BASH_SOURCE[0]}")/.."
for SEED in 101 202; do
  python3 -u train_sac.py --obs state_lidar --num_envs 16 --total_timesteps 150000 \
      --seed $SEED --run_name exp9_seed$SEED --headless > logs/sac_exp9_seed$SEED.log 2>&1
  python3 -u scripts/eval_sac.py --obs state_lidar --num_envs 16 --steps 6200 --headless \
      --checkpoint logs/sb3/zm_robot_omni_sac_state_lidar/exp9_seed$SEED/model_150000_steps.zip \
      > logs/eval_exp9_seed$SEED.log 2>&1
done
echo SEEDCHECK_DONE > logs/seed_check_done.txt
