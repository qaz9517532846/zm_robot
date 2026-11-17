#!/usr/bin/env bash
# After the hybrid queue finishes: X-Mobility zero-shot baseline (its own command only),
# 16 envs x 6200 steps, same protocol as the SAC evals. X-Mobility runs in chunks of 4 envs.
cd "$(dirname "${BASH_SOURCE[0]}")/.."
until [ -f logs/hybrid_done.txt ]; do sleep 30; done
HF_HUB_OFFLINE=1 python3 -u scripts/eval_xmobility_base.py --num_envs 16 --steps 6200 --headless \
    --enable_cameras > logs/eval_xmob_base.log 2>&1
echo XMOB_BASELINE_DONE > logs/xmob_baseline_done.txt
