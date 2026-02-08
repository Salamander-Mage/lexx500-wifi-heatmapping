#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="/home/salamander/Desktop/lexx500-wifi-heatmapping"

docker run -it --rm \
  --net=host \
  --privileged \
  -v "${PROJECT_DIR}:/workspace" \
  -w /workspace \
  lexx-heatmap:noetic \
  bash
