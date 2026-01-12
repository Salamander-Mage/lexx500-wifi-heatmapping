#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$HOME/Desktop/Heatmapping Tool Project"

docker run -it --rm \
  --net=host \
  --privileged \
  -v "${PROJECT_DIR}:/workspace" \
  -w /workspace \
  lexx-heatmap:noetic \
  bash
