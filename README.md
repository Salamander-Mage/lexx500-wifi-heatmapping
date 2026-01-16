Lexx500 Wi-Fi Heatmapping (Field Engineer Guide)

Purpose:
Record robot position + Wi-Fi signal strength while the AMR operates, then generate a Wi-Fi coverage heatmap.

✅ No ROS changes
✅ Read-only pose subscription
✅ One command to collect
✅ Safe to stop anytime

What This Tool Does

While the robot is operating:

Reads robot pose from ROS (/amcl_pose)

Reads Wi-Fi signal via SNMP (MOXA / FXE)

Logs synchronized samples to SQLite

Heatmap is generated offline

Requirements (Already on Robot)

Docker

ROS already running

Network access to MOXA / FXE device

One-Time Setup (Per Site)
1️⃣ Identify the SNMP RSSI OID

This is the only required configuration.

Example:

export SNMP_RSSI_OID=.1.3.6.1.4.1.8691.15.35.1.11.17.1.12.1.1


(Optional, recommended)

echo 'export SNMP_RSSI_OID=...' >> ~/.bashrc

Collect Wi-Fi + Pose Data

From the repo root:

./scripts/robot_collect_pose_snmp.sh klab_trial01


What happens:

Starts Docker

Connects to ROS

Polls SNMP

Writes to:

data/klab_trial01.sqlite


Stop anytime with Ctrl+C
✔ Data is saved continuously

Expected Console Output
Using interface: snmp
Logging to: data/klab_trial01.sqlite
Sample rate: 2.0 Hz
Pose source: ros1
Collected 10 samples... pose=(x,y) rssi=-62
Collected 20 samples...


If you see samples increasing → it’s working.

Generate the Heatmap (After Collection)
python3 src/render_from_sqlite.py \
  --db data/klab_trial01.sqlite \
  --out_dir output/klab_trial01


Results saved to:

output/klab_trial01/

Common Checks (If Something Fails)
Pose not updating?
rostopic echo /amcl_pose

SNMP reachable?
snmpget -v2c -c public <MOXA_IP> sysDescr.0

RSSI always None?

Check SNMP_RSSI_OID

Confirm device firmware matches expected OID

What NOT to Do

❌ Do not modify ROS launch files
❌ Do not add ROS nodes
❌ Do not install Python packages on host

This tool is read-only and external by design.

Engineer Checklist

 /amcl_pose exists

 SNMP device reachable

 SNMP_RSSI_OID set

 Script runs

 SQLite file created

Files You Care About

scripts/robot_collect_pose_snmp.sh → run this

data/*.sqlite → raw samples

output/*/ → heatmaps