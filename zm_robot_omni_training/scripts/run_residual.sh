#!/usr/bin/env bash
# COMPASS-style residual SAC (final = clip(X-Mobility cmd + SAC residual)), seed 101, three rewards.
# Each: train 150k steps (16 envs) -> deterministic eval 16 envs x 6200 steps.
# Progress lines -> logs/residual_progress.txt; flag file logs/residual_done.txt when all finished.
cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1
export HF_HUB_OFFLINE=1
SEED=101
for REW in progress penalty hybrid; do
  NAME=resid_${REW}_seed${SEED}
  python3 -u train_sac.py --obs state_lidar_xmob --reward $REW --residual_scale 0.5 \
      --num_envs 16 --total_timesteps 150000 --seed $SEED --run_name $NAME \
      --headless --enable_cameras > logs/sac_${NAME}.log 2>&1
  CK=logs/sb3/zm_robot_omni_sac_state_lidar_xmob/$NAME/model_150000_steps.zip
  if [ -f "$CK" ]; then
    python3 -u scripts/eval_sac.py --obs state_lidar_xmob --reward $REW --residual_scale 0.5 \
        --num_envs 16 --steps 6200 --checkpoint $CK --headless --enable_cameras > logs/eval_${NAME}.log 2>&1
    echo "$NAME $(grep 'success rate' logs/eval_${NAME}.log)" >> logs/residual_progress.txt
  else
    echo "$NAME TRAIN_FAILED (no $CK)" >> logs/residual_progress.txt
  fi
done
echo ALL_DONE > logs/residual_done.txt
