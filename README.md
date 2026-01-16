# Lexx500 Wi-Fi Heatmapping — Field Engineer Quick Guide

PURPOSE
- Collect robot pose + Wi-Fi signal strength during normal AMR operation
- Produce Wi-Fi coverage heatmaps
- Read-only, no ROS modification, safe to stop anytime

REQUIREMENTS (already on robot)
- Docker
- ROS running
- Network access to MOXA / FXE device

----------------------------------------------------------------
ONE-TIME SETUP (PER SITE)
----------------------------------------------------------------

1) Identify SNMP RSSI OID (MOXA / FXE)

Example (AWK-1137C):
export SNMP_RSSI_OID=.1.3.6.1.4.1.8691.15.35.1.11.17.1.12.1.1

(Optional but recommended)
echo 'export SNMP_RSSI_OID=.1.3.6.1.4.1.8691.15.35.1.11.17.1.12.1.1' >> ~/.bashrc

----------------------------------------------------------------
DATA COLLECTION (ON ROBOT)
----------------------------------------------------------------

From repo root:

./scripts/robot_collect_pose_snmp.sh klab_trial01

What this does:
- Runs inside existing hybrid-amr container
- Subscribes to /amcl_pose
- Polls SNMP RSSI
- Writes SQLite database continuously

Output file:
data/klab_trial01.sqlite

Stop anytime with Ctrl+C (data is preserved).

----------------------------------------------------------------
EXPECTED OUTPUT
----------------------------------------------------------------

Using interface: snmp

Logging to: data/klab_trial01.sqlite

Sample rate: 2.0 Hz

Pose source: ros1

Collected 10 samples... pose=(x,y) rssi=-62

Collected 20 samples...

If sample count increases → system is working.

----------------------------------------------------------------
HEATMAP GENERATION (OFFLINE OR ON ROBOT)
----------------------------------------------------------------

python3 src/render_from_sqlite.py \
  --db data/klab_trial01.sqlite \
  --out_dir output/klab_trial01

Results saved to:
output/klab_trial01/

----------------------------------------------------------------
SANITY CHECK COMMANDS
----------------------------------------------------------------

Check pose:
rostopic echo /amcl_pose

Check SNMP connectivity:
snmpget -v2c -c public <MOXA_IP> sysDescr.0

Check RSSI OID manually:
snmpget -v2c -c public <MOXA_IP> $SNMP_RSSI_OID

----------------------------------------------------------------
COMMON ISSUES
----------------------------------------------------------------

RSSI always None:
- Verify SNMP_RSSI_OID
- Confirm MOXA firmware matches OID

Pose frozen:
- Robot not moving OR localization paused
- /amcl_pose publisher down

Script fails immediately:
- ROS not running
- Container not healthy

----------------------------------------------------------------
DO NOT DO
----------------------------------------------------------------

- Do NOT modify ROS launch files
- Do NOT install Python packages on host
- Do NOT add ROS nodes

This tool is read-only by design.

----------------------------------------------------------------
FILES YOU CARE ABOUT
----------------------------------------------------------------

scripts/robot_collect_pose_snmp.sh

src/app/collect.py

src/app/wifi_snmp.py

data/*.sqlite

output/*/

----------------------------------------------------------------
ENGINEER CHECKLIST
----------------------------------------------------------------

[ ] /amcl_pose publishing

[ ] SNMP reachable

[ ] SNMP_RSSI_OID set

[ ] SQLite file created

[ ] Samples increasing
