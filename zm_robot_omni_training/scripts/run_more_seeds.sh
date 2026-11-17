#!/usr/bin/env bash
# More seeds for the 2-arm comparison (progress = exp9 reward, penalty = exp7 reward).
# Interleaved so a partial run is still balanced. Sequential: one sim at a time.
cd "$(dirname "${BASH_SOURCE[0]}")/.."
D=logs/sb3/zm_robot_omni_sac_state_lidar
for SEED in 303 404 505; do
  for ARM in progress penalty; do
    if [ "$ARM" = progress ]; then NAME=exp9_seed$SEED; else NAME=exp7ctl_seed$SEED; fi
    python3 -u train_sac.py --obs state_lidar --reward $ARM --num_envs 16 --total_timesteps 150000 \
        --seed $SEED --run_name $NAME --headless > logs/sac_$NAME.log 2>&1
    python3 -u scripts/eval_sac.py --obs state_lidar --reward $ARM --num_envs 16 --steps 6200 --headless \
        --checkpoint $D/$NAME/model_150000_steps.zip > logs/eval_$NAME.log 2>&1
    echo "$NAME done" >> logs/more_seeds_progress.txt
  done
done
echo MORESEEDS_DONE > logs/more_seeds_done.txt
