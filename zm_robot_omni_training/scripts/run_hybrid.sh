#!/usr/bin/env bash
# Hybrid goal-heading reward (progress + 1/4 penalty), state_lidar obs, seeds 101..505,
# 150k steps each + the standard large-sample eval. Waits for the X-Mobility baseline eval
# (one simulation at a time).
cd "$(dirname "${BASH_SOURCE[0]}")/.."
until grep -qE "dist to goal now|Traceback" logs/eval_xmob_base.log 2>/dev/null; do sleep 20; done
D=logs/sb3/zm_robot_omni_sac_state_lidar
for SEED in 101 202 303 404 505; do
  NAME=hybrid_seed$SEED
  python3 -u train_sac.py --obs state_lidar --reward hybrid --num_envs 16 --total_timesteps 150000 \
      --seed $SEED --run_name $NAME --headless > logs/sac_$NAME.log 2>&1
  python3 -u scripts/eval_sac.py --obs state_lidar --reward hybrid --num_envs 16 --steps 6200 --headless \
      --checkpoint $D/$NAME/model_150000_steps.zip > logs/eval_$NAME.log 2>&1
  echo "$NAME done" >> logs/hybrid_progress.txt
done
echo HYBRID_DONE > logs/hybrid_done.txt
