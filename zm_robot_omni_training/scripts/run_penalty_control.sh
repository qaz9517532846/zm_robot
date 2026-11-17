#!/usr/bin/env bash
# Control for the exp 9 seed check: exp 7's goal-heading PENALTY reward, trained FROM
# SCRATCH for 150k steps with the same fixed seeds, evaluated with the same protocol.
cd "$(dirname "${BASH_SOURCE[0]}")/.."
for SEED in 101 202; do
  python3 -u train_sac.py --obs state_lidar --reward penalty --num_envs 16 \
      --total_timesteps 150000 --seed $SEED --run_name exp7ctl_seed$SEED --headless \
      > logs/sac_exp7ctl_seed$SEED.log 2>&1
  python3 -u scripts/eval_sac.py --obs state_lidar --reward penalty --num_envs 16 \
      --steps 6200 --headless \
      --checkpoint logs/sb3/zm_robot_omni_sac_state_lidar/exp7ctl_seed$SEED/model_150000_steps.zip \
      > logs/eval_exp7ctl_seed$SEED.log 2>&1
done
echo PENALTY_CONTROL_DONE > logs/penalty_control_done.txt
