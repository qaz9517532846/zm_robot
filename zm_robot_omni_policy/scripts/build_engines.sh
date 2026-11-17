#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Build the TensorRT engines from the ONNX files in models/ (run once per machine / TensorRT
# version / GPU). FP32 by default so the output matches the PyTorch policy closely;
# pass --fp16 for a smaller, faster engine (re-run the replay validation afterwards).
#
#   scripts/build_engines.sh [--fp16] [--force] [model_dir]
set -euo pipefail
FP16=""; FORCE=0; DIR=""
for a in "$@"; do
  case "$a" in
    --fp16) FP16="--fp16" ;;
    --force) FORCE=1 ;;
    *) DIR="$a" ;;
  esac
done
DIR="${DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../models" && pwd)}"
command -v trtexec >/dev/null || { echo "trtexec not found (TensorRT)"; exit 1; }
for onnx in "$DIR"/*.onnx; do
  engine="${onnx%.onnx}.engine"
  if [ -f "$engine" ] && [ "$FORCE" -eq 0 ] && [ "$engine" -nt "$onnx" ]; then
    echo "[skip] $(basename "$engine") is up to date"; continue
  fi
  echo "[build] $(basename "$onnx") -> $(basename "$engine") ${FP16}"
  trtexec --onnx="$onnx" --saveEngine="$engine" $FP16 --skipInference > "$engine.log" 2>&1 \
    || { echo "trtexec failed, see $engine.log"; tail -5 "$engine.log"; exit 1; }
  ls -la "$engine"
done
