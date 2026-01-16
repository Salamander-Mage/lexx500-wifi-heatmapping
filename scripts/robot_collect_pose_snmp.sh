#!/usr/bin/env bash
# scripts/robot_collect_pose_snmp.sh
#
# One-command robot survey runner:
# - Reads pose from ROS1 (/amcl_pose by default)
# - Reads RSSI from MOXA via SNMP (no Wi-Fi NIC needed on the NUC)
# - Writes a .sqlite DB into ./data/
#
# Usage:
#   ./scripts/robot_collect_pose_snmp.sh klab_trial01
#
# Optional overrides (env vars):
#   IMAGE=ghcr.io/lexxpluss/hybrid-amr-release:v1.14.1
#   ROS_MASTER_URI=http://127.0.0.1:11311
#   ROS_IP=192.168.1.77
#   POSE_TOPIC=/amcl_pose
#   HZ=2
#   SNMP_HOST=192.168.1.254
#   SNMP_COMMUNITY=public
#   SNMP_RSSI_OID=.1.3.6.1.4.1.8691.15.35.1.11.17.1.12.1.1   (example; set to your actual RSSI OID)
#
# NOTE:
#   You MUST set SNMP_RSSI_OID to the correct OID for your device model/firmware.
#   (Your earlier diff suggests there are changing wifi-related fields under enterprises.8691.*)

set -euo pipefail

# -------- args --------
NAME="${1:-}"
if [[ -z "$NAME" ]]; then
  echo "Usage: $0 <run_name>"
  echo "Example: $0 klab_trial01"
  exit 1
fi

# -------- config (defaults) --------
IMAGE="${IMAGE:-ghcr.io/lexxpluss/hybrid-amr-release:v1.14.1}"

ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
# Try to auto-pick ROS_IP from primary interface if user didn't set it.
if [[ -z "${ROS_IP:-}" ]]; then
  ROS_IP="$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' | head -n1)"
  export ROS_IP
fi
POSE_TOPIC="${POSE_TOPIC:-/amcl_pose}"
HZ="${HZ:-2}"

# SNMP target: default to gateway if not set
if [[ -z "${SNMP_HOST:-}" ]]; then
  SNMP_HOST="$(ip route | awk '/default/ {print $3; exit}')"
  export SNMP_HOST
fi
SNMP_COMMUNITY="${SNMP_COMMUNITY:-public}"

# IMPORTANT: set this to the real RSSI OID for MOXA (or FXE-5000 later)
SNMP_RSSI_OID="${SNMP_RSSI_OID:-}"

# Output location
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DB_HOST="${ROOT_DIR}/data/${NAME}.sqlite"
mkdir -p "${ROOT_DIR}/data"

echo "=== lexx500-wifi-heatmapping: robot_collect_pose_snmp ==="
echo "Run name:         ${NAME}"
echo "Output DB:        ${OUT_DB_HOST}"
echo "Docker image:     ${IMAGE}"
echo "ROS_MASTER_URI:   ${ROS_MASTER_URI}"
echo "ROS_IP:           ${ROS_IP}"
echo "POSE_TOPIC:       ${POSE_TOPIC}"
echo "HZ:               ${HZ}"
echo "SNMP_HOST:        ${SNMP_HOST}"
echo "SNMP_COMMUNITY:   ${SNMP_COMMUNITY}"
echo "SNMP_RSSI_OID:    ${SNMP_RSSI_OID:-<NOT SET>}"
echo

if [[ -z "$SNMP_RSSI_OID" ]]; then
  echo "ERROR: SNMP_RSSI_OID is not set."
  echo "Set it like:"
  echo "  export SNMP_RSSI_OID=.1.3.6.1.4.1.8691...."
  echo "Then re-run."
  exit 2
fi

# Quick connectivity sanity (non-fatal if it fails)
echo "Sanity: ping SNMP host..."
ping -c 1 -W 1 "${SNMP_HOST}" >/dev/null 2>&1 && echo "  OK" || echo "  (ping failed, continuing)"

# -------- run collector inside container --------
# Assumes your Python code supports:
#   --pose_source ros1
#   --pose_topic /amcl_pose
#   --rssi_source snmp
#   --snmp_host <ip>
#   --snmp_community <community>
#   --snmp_rssi_oid <oid>
#
# If your flags are named slightly differently, just edit the python3 -m line.
docker run --rm -it \
  --network host \
  -v "${ROOT_DIR}:/workspace" \
  -w /workspace \
  "${IMAGE}" \
  bash -lc "
    set -e
    source /opt/ros/noetic/setup.bash

    export ROS_MASTER_URI='${ROS_MASTER_URI}'
    export ROS_IP='${ROS_IP}'
    unset ROS_HOSTNAME

    # ROS Python pkgs + your app
    export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/workspace/src

    # optional sanity
    python3 -c 'import rospy; print(\"rospy OK\")'
    python3 -c 'import app; print(\"app OK\")'

    # run
    python3 -m app.collect \
      --db '/workspace/data/${NAME}.sqlite' \
      --hz '${HZ}' \
      --pose_source ros1 \
      --pose_topic '${POSE_TOPIC}' \
      --rssi_source snmp \
      --snmp_host '${SNMP_HOST}' \
      --snmp_community '${SNMP_COMMUNITY}' \
      --snmp_rssi_oid '${SNMP_RSSI_OID}' \
      --interface lo
  "

echo
echo "✅ Done. DB written to: ${OUT_DB_HOST}"
echo "Next (on your laptop or on the NUC):"
echo "  python3 -u src/render_from_sqlite.py --db data/${NAME}.sqlite --out_dir output/${NAME} --cell_m 0.75 --min_samples 2 --bad_p10_threshold -75"
