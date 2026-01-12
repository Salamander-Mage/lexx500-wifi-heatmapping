#!/usr/bin/env bash
set -euo pipefail

DB="${1:?usage: collect_on_robot.sh path/to/output.sqlite}"
IFACE="${2:-wlan0}"
HZ="${3:-2}"

source /opt/ros/noetic/setup.bash

python -m src.app.collect \
  --db "$DB" \
  --interface "$IFACE" \
  --hz "$HZ" \
  --pose_source ros_tf
