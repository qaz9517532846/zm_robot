#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/.." && HF_HUB_OFFLINE=1 python3 -u train_sac.py --obs state_lidar_xmob --reward progress --residual_scale 0.5 --num_envs 4 --total_timesteps 400 --headless --enable_cameras --run_name smoke_residual > logs/smoke_residual.log 2>&1
