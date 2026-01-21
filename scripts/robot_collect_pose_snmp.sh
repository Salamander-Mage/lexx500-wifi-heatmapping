#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./scripts/robot_collect_pose_snmp.sh data/amr_run01.sqlite
#
# Optional env overrides:
#   ROS_IP=192.168.1.77
#   ROS_MASTER_URI=http://127.0.0.1:11311
#   POSE_TOPIC=/amcl_pose
#   HZ=2
#   SNMP_HOST=192.168.1.254
#   SNMP_COMMUNITY=public
#   RSSI_OID=.1.3.6.1.4.1.8691.15.35.1.11.17.1.4.1.1
#   SSID_OID=.1.3.6.1.4.1.8691.15.35.1.11.17.1.6.1.1
#   BSSID_OID=.1.3.6.1.4.1.8691.15.35.1.11.17.1.3.1.1

DB_PATH="${1:-data/amr_run01.sqlite}"

ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
ROS_IP="${ROS_IP:-192.168.1.77}"
POSE_TOPIC="${POSE_TOPIC:-/amcl_pose}"
HZ="${HZ:-2}"

SNMP_HOST="${SNMP_HOST:-192.168.1.254}"
SNMP_COMMUNITY="${SNMP_COMMUNITY:-public}"
RSSI_OID="${RSSI_OID:-.1.3.6.1.4.1.8691.15.35.1.11.17.1.4.1.1}"
SSID_OID="${SSID_OID:-.1.3.6.1.4.1.8691.15.35.1.11.17.1.6.1.1}"
BSSID_OID="${BSSID_OID:-.1.3.6.1.4.1.8691.15.35.1.11.17.1.3.1.1}"

mkdir -p "$(dirname "$DB_PATH")"

docker run --rm -it \
  --user root \
  --network host \
  -v "$PWD:/workspace" \
  -w /workspace \
  ghcr.io/lexxpluss/hybrid-amr-release:v1.14.1 \
  bash -lc "
    set -e
    mkdir -p /var/lib/apt/lists/partial
    apt-get update -y >/dev/null
    apt-get install -y snmp >/dev/null

    source /opt/ros/noetic/setup.bash
    export ROS_MASTER_URI='${ROS_MASTER_URI}'
    export ROS_IP='${ROS_IP}'
    unset ROS_HOSTNAME

    export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/workspace/src

    python3 -m app.collect \
      --db /workspace/${DB_PATH} \
      --hz ${HZ} \
      --pose_source ros1 \
      --pose_topic ${POSE_TOPIC} \
      --snmp_host ${SNMP_HOST} \
      --snmp_community ${SNMP_COMMUNITY} \
      --snmp_rssi_oid ${RSSI_OID} \
      --snmp_ssid_oid ${SSID_OID} \
      --snmp_bssid_oid ${BSSID_OID}
  "
