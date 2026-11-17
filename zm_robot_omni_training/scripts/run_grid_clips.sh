#!/usr/bin/env bash
# Record the comparison clips in small batches (one simulation at a time). On this Isaac Lab
# version every environment loads its own copy of the office mesh for the LiDAR ray caster,
# so 15 environments at once run the 16 GB GPU out of memory; 3 per run fit.
# Batches whose clips already exist are skipped, so the script can simply be re-run.
#   scripts/run_grid_clips.sh <batch>      batch = l1..l5 (LiDAR), x1, x2 (X-Mobility group) or all
cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1
PY=/home/zmtech/nvidia/IsaacLab-Arena/.venv/bin/python
OUT=logs/videos/grid
L=logs/sb3/zm_robot_omni_sac_state_lidar
X=logs/sb3/zm_robot_omni_sac_state_lidar_xmob
mkdir -p "$OUT"
export PYTHONPATH=$PWD:$PWD/.pydeps HF_HUB_OFFLINE=1 OMNI_KIT_ACCEPT_EULA=YES

run() {    # run <batch name> <group> <label=checkpoint>...
  local name=$1 group=$2; shift 2
  local args=() missing=0
  for m in "$@"; do
    args+=(--model "$m")
    [ -f "$OUT/${m%%=*}.json" ] || missing=1
  done
  if [ "$missing" -eq 0 ]; then echo "[$name] already recorded"; return 0; fi
  timeout 1500 "$PY" -u scripts/record_clip.py --group "$group" "${args[@]}" --outdir "$OUT" > "$OUT/run_$name.log" 2>&1
  echo "[$name] exit $?: $(grep -E '\[clip\] (done|could not)' "$OUT/run_$name.log" | tr '\n' ' ')"
}
lidar() { run "$1" lidar "lidar_$2_$3=$L/$5_seed$3/model_150000_steps.zip" "lidar_$2_$4=$L/$5_seed$4/model_150000_steps.zip" ${6:+"lidar_$2_$6=$L/$5_seed$6/model_150000_steps.zip"}; }

case "${1:-all}" in
  l1|all) lidar l1 progress 101 202 exp9 303 ;;&
  l2|all) lidar l2 progress 404 505 exp9 ;;&
  l3|all) lidar l3 penalty 101 202 exp7ctl 303 ;;&
  l4|all) lidar l4 penalty 404 505 exp7ctl ;;&
  l5|all) lidar l5 hybrid 101 202 hybrid 303 ;;&
  l6|all) lidar l6 hybrid 404 505 hybrid ;;&
  x1|all) run x1 xmob "resid_progress_101=$X/resid_progress_seed101/model_150000_steps.zip" "resid_penalty_101=$X/resid_penalty_seed101/model_150000_steps.zip" ;;&
  x2|all) run x2 xmob "resid_hybrid_101=$X/resid_hybrid_seed101/model_150000_steps.zip" "xmob_alone=none" ;;
esac
echo "clips so far: $(ls "$OUT"/*.mp4 2>/dev/null | wc -l)/19"
