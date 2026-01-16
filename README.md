# Lexx500 Wi-Fi Heatmapping — Field Engineer Quick Guide

PURPOSE
- Collect robot pose + Wi-Fi signal strength during AMR operation
- Produce Wi-Fi coverage heatmaps
- Safe, read-only, stop anytime

================================================================
ROBOT MODE (RECOMMENDED)
================================================================

Uses:
- Real antenna position
- Real navigation path
- Production-accurate results

----------------------------------------------------------------
ONE-TIME SETUP (PER SITE)
----------------------------------------------------------------

export SNMP_RSSI_OID=.1.3.6.1.4.1.8691.15.35.1.11.17.1.12.1.1

(Optional)

echo 'export SNMP_RSSI_OID=.1.3.6.1.4.1.8691.15.35.1.11.17.1.12.1.1' >> ~/.bashrc

----------------------------------------------------------------
COLLECT DATA (ON ROBOT)
----------------------------------------------------------------

./scripts/robot_collect_pose_snmp.sh klab_trial01

Output:
data/klab_trial01.sqlite

Stop anytime with Ctrl+C.

----------------------------------------------------------------
OPTIONAL: LAPTOP WALK SURVEY (BASELINE / DEBUG)
----------------------------------------------------------------

Uses:
- Human walking path
- Laptop Wi-Fi interface
- No ROS required

From laptop:

python3 -m src.app.collect \
  --db data/laptop_walk01.sqlite \
  --hz 2 \
  --pose_source click \
  --interface wlp0s20f3

Notes:
- Click on map to approximate position
- Results are qualitative, not robot-accurate

----------------------------------------------------------------
HEATMAP GENERATION (ANY MACHINE)
----------------------------------------------------------------

python3 src/render_from_sqlite.py \
  --db data/klab_trial01.sqlite \
  --out_dir output/klab_trial01

----------------------------------------------------------------
SANITY CHECKS
----------------------------------------------------------------

Robot pose:
rostopic echo /amcl_pose

SNMP:
snmpget -v2c -c public <MOXA_IP> sysDescr.0
snmpget -v2c -c public <MOXA_IP> $SNMP_RSSI_OID

----------------------------------------------------------------
COMMON ISSUES
----------------------------------------------------------------

RSSI None:
- Wrong OID
- SNMP disabled

Pose frozen:
- Robot not moving
- Localization paused

----------------------------------------------------------------
FILES YOU CARE ABOUT
----------------------------------------------------------------

scripts/robot_collect_pose_snmp.sh

src/app/collect.py

src/app/wifi_snmp.py

data/*.sqlite

output/*/

----------------------------------------------------------------
DO NOT DO
----------------------------------------------------------------

- Do not modify ROS configs
- Do not install packages on robot
- Do not run alongside mapping

================================================================
END
