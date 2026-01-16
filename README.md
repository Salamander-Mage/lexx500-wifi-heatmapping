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


================================================================
HOW TO READ THE HTML REPORT
================================================================

Open:
output/<run_name>/report.html

The report contains multiple images. Each answers a different question.

----------------------------------------------------------------
KEY VOCABULARY
----------------------------------------------------------------

RSSI
- Received Signal Strength Indicator
- Units: dBm (negative numbers)
- Higher (closer to 0) = stronger signal

P10 (10th Percentile)
- Worst 10% of samples at a location
- Represents reliability, not average
- Used to detect intermittent dropouts

Median RSSI
- Typical signal level at a location
- Hides short dropouts

Bad Zone
- Area where P10 RSSI is below threshold
- Indicates high risk for packet loss

----------------------------------------------------------------
IMAGE GUIDE
----------------------------------------------------------------

1) rssi_median.png
------------------
Question it answers:
- "What signal do I usually get here?"

How to read:
- Smooth, stable view of coverage
- Good for AP placement validation

DO NOT use alone to judge reliability.

----------------------------------------------------------------
2) rssi_p10.png
---------------
Question it answers:
- "How bad does it get at its worst?"

This is the MOST IMPORTANT image.

How to read:
- Highlights weak or unstable areas
- Correlates with navigation dropouts

Target values:
- Ideal:       ≥ -70 dBm
- Acceptable:  ≥ -75 dBm
- Risky:       < -75 dBm
- Severe:      < -80 dBm

----------------------------------------------------------------
3) bad_zones.png
----------------
Question it answers:
- "Where will the robot likely fail?"

How it is generated:
- Any cell where P10 < threshold (default -75 dBm)

How to use:
- Prioritize these zones first
- Strong evidence for:
  - AP relocation
  - Channel changes
  - Additional APs

----------------------------------------------------------------
4) path_overlay.png
-------------------
Question it answers:
- "Where did the robot actually go?"

What it shows:
- Robot path overlaid on signal map
- Useful for:
  - Verifying coverage along routes
  - Explaining why issues only happen on certain paths

----------------------------------------------------------------
5) rssi_over_time.png
---------------------
Question it answers:
- "Is the signal stable over time?"

How to read:
- Spikes and drops indicate roaming or interference
- Flat line = stable link

Useful for:
- Detecting transient issues
- Correlating with logs

----------------------------------------------------------------
6) rssi_hist.png
----------------
Question it answers:
- "How consistent is the signal overall?"

How to read:
- Narrow peak = stable network
- Wide spread = unreliable link

----------------------------------------------------------------
FINAL INTERPRETATION RULES
----------------------------------------------------------------

If Median is good but P10 is bad:
- Network looks fine but is unreliable

If Bad Zones align with robot path:
- Expect real navigation issues

If Laptop looks good but Robot looks bad:
- Antenna placement or shielding issue

----------------------------------------------------------------
WHAT THIS TOOL IS FOR
----------------------------------------------------------------

- Diagnosing Wi-Fi reliability issues
- Comparing MOXA vs FXE-5000 performance
- Validating AP changes

NOT for:
- Absolute throughput measurement
- Certification testing
- Replacing RF surveys

================================================================
END REPORT GUIDE
================================================================


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
