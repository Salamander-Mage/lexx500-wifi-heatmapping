# Lexx500 Wi-Fi Heatmapping — Field Engineer Quick Guide

PURPOSE
- Collect robot pose + Wi-Fi signal strength during AMR operation
- Produce Wi-Fi coverage heatmaps
- Safe, read-only, stop anytime

=====
ROBOT MODE (RECOMMENDED)
=====

Uses:
- Real antenna position
- Real navigation path
- Production-accurate results

-----------------------------------------
Wi-Fi RSSI via SNMP (Hardware-Dependent)
-----------------------------------------
IMPORTANT: SNMP OIDs for Wi-Fi metrics (RSSI, SSID, BSSID, noise, etc.) are vendor- and model-specific. Do NOT assume the same OIDs work across different access points or industrial routers.


On a fresh clone export the following at the root directory of the app for applicable hardware:

export SNMP_HOST= #use default moxa or fxe-5000 ip address

export SNMP_COMMUNITY=public

# RSSI (dBm)
export SNMP_RSSI_OID=.1.3.6.1.4.1.672.69.3.3.2.1.9.0 #FXE5000

export SNMP_RSSI_OID=.1.3.6.1.4.1.8691.15.35.1.11.17.1.4.1.1  #MOXA

# SSID
export SNMP_SSID_OID=.1.3.6.1.4.1.672.69.3.3.2.1.6.0 #FXE5000

export SNMP_SSID_OID=.1.3.6.1.4.1.8691.15.35.1.11.17.1.6.1.1  #MOXA

# BSSID (AP MAC)
export SNMP_BSSID_OID=.1.3.6.1.4.1.672.69.3.3.2.1.8.0 #FXE5000

export SNMP_BSSID_OID=.1.3.6.1.4.1.8691.15.35.1.11.17.1.3.1.1 #MOXA


Notes:
- Some devices expose noise floor, SNR, or TX power instead of RSSI
- Using the wrong OID can silently produce misleading heatmaps
- This tool assumes RSSI values are in dBm (negative integers)

------------------------------
COLLECT DATA (ON ROBOT)
------------------------------
At the directory root run the following:

First make the script executable:

chmod +x scripts/robot_collect_pose_snmp.sh

./scripts/robot_collect_pose_snmp.sh data/amr_run01.sqlite

Output:
data/amr_run01.sqlite

Stop scanning anytime with Ctrl+C.

Render this sqlite into a HTML report on the AMR:

docker run --rm -it \
  -v "$PWD:/workspace" \
  -w /workspace \
  python:3.10-slim \
  bash -lc '
    pip install --no-cache-dir numpy pandas matplotlib >/dev/null
    python src/render_from_sqlite.py \
      --db data/amr_run01.sqlite \
      --out out/amr_run01.html
  '


From your laptop run the following example:

scp -r \
  lexxauto@192.168.0.117:~/Tools/lexx500-wifi-heatmapping/out/amr_run01.html \
  ~/Desktop/

xdg-open ~/Desktop/amr_run01.html/report.html

--------------------------------------
LAPTOP WALK SURVEY (BASELINE / DEBUG)
--------------------------------------

Uses:
- Human walking path
- Laptop Wi-Fi interface
- No ROS required

What this mode does:
  You walk with a laptop and manually enter the pose points (x y yaw) while the tool logs:
  - your entered pose
  - Wi-Fi info from the laptop's Wi-Fi interface (RSSI, link state, etc.)
  - optional ping probe (if enabled)

  It outputs:

  - data/<run>.sqlite (raw log)
  - out/<run>.html/report.html (heatmap report)

Prerequisites:
- linux laptop
- python 3 installed
- docker installed
- sqlite3 installed

1. Clone repo
     git clone <YOUR_GITHUB_REPO_URL>

     cd lexx500-wifi-heatmapping

     mkdir -p data out (create output folders)

2. Quick sanity check

   PYTHONPATH="$PWD/src" python3 -c "import app; print('✅ app import OK')"

3. Start manual walk

   PYTHONPATH="$PWD/src" python3 -m app.collect \
        --db data/laptop_walk.sqlite \
        --hz 1 \
        --pose_source manual \
        --manual_pose \
        --commit_every 1

   You should see a prompt like
   Enter pose: x y yaw...

4. Enter poses while you walk

   x y yaw
   0 0 0
   1.0 0 0
   1.0 1.0 90
   2.0 1.0 90
   End the run with Ctrl+C

5. Verify the SQLite database has data

   sqlite3 data/laptop_walk.sqlite '.tables'
   sqlite3 data/laptop_walk.sqlite 'select count(*) from samples;'
   sqlite3 data/laptop_walk.sqlite 'select * from samples limit 3;'

6. Render the HTML report (recommended via Docker)

   docker run --rm -it \
  -v "$PWD:/workspace" \
  -w /workspace \
  python:3.10-slim \
  bash -lc '
    set -e
    pip install --no-cache-dir numpy pandas matplotlib
    python src/render_from_sqlite.py \
      --db data/laptop_walk.sqlite \
      --out out/laptop_walk.html
  '

7. Open the report
   sudo chown -R "$USER:$USER" out
   xdg-open out/laptop_walk.html/report.html
 

=====
HOW TO READ THE HTML REPORT
=====

Open:
output/<run_name>/report.html

The report contains multiple images. Each answers a different question.

-------------------------
KEY VOCABULARY
-------------------------

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

-----------------------------------------------
IMAGE GUIDE
-----------------------------------------------


----------------
1) rssi_median.png
------------------
Question it answers:
- "What signal do I usually get here?"

How to read:
- Smooth, stable view of coverage
- Good for AP placement validation

DO NOT use alone to judge reliability.

----------------
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

----------------
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

-------------------
4) path_overlay.png
-------------------
Question it answers:
- "Where did the robot actually go?"

What it shows:
- Robot path overlaid on signal map
- Useful for:
  - Verifying coverage along routes
  - Explaining why issues only happen on certain paths

---------------------
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

----------------
6) rssi_hist.png
----------------
Question it answers:
- "How consistent is the signal overall?"

How to read:
- Narrow peak = stable network
- Wide spread = unreliable link

--------------------------
FINAL INTERPRETATION RULES
--------------------------

If Median is good but P10 is bad:
- Network looks fine but is unreliable

If Bad Zones align with robot path:
- Expect real navigation issues

If Laptop looks good but Robot looks bad:
- Antenna placement or shielding issue

---------------------
WHAT THIS TOOL IS FOR
---------------------

- Diagnosing Wi-Fi reliability issues
- Comparing MOXA vs FXE-5000 performance
- Validating AP changes

NOT for:
- Absolute throughput measurement
- Certification testing
- Replacing RF surveys

=====
END REPORT GUIDE
=====


----------------------
SANITY CHECKS
----------------------

Robot pose:
rostopic echo /amcl_pose

SNMP:
snmpget -v2c -c public <MOXA_IP> sysDescr.0
snmpget -v2c -c public <MOXA_IP> $SNMP_RSSI_OID

-----------------------
COMMON ISSUES
-----------------------

RSSI None:
- Wrong OID
- SNMP disabled

Pose frozen:
- Robot not moving
- Localization paused

----------------------
DO NOT DO
----------------------

- Do not modify ROS configs
- Do not install packages on robot
- Do not run alongside mapping

=====
END
