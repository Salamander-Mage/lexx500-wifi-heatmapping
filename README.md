# Lexx500 Wi-Fi Heatmapping — Field Engineer Quick Guide

Collect Wi-Fi signal data (ROS1 pose + SNMP on AMR, or manual laptop walk), store it in SQLite, and render heatmaps/HTML reports. Copy/paste the flow that matches your situation.

## SNMP OID examples (pick the right ones for your device)
- FXE5000: RSSI `.1.3.6.1.4.1.672.69.3.3.2.1.9.0`, SSID `.1.3.6.1.4.1.672.69.3.3.2.1.6.0`, BSSID `.1.3.6.1.4.1.672.69.3.3.2.1.8.0`
- MOXA:    RSSI `.1.3.6.1.4.1.8691.15.35.1.11.17.1.4.1.1`, SSID `.1.3.6.1.4.1.8691.15.35.1.11.17.1.6.1.1`, BSSID `.1.3.6.1.4.1.8691.15.35.1.11.17.1.3.1.1`
- Unknown? Discover once per site: `snmpwalk -v2c -c <comm> <host> 1.3.6.1.4.1 | head -n 200` and test candidates with `snmpget` while moving (RSSI should vary).

## Quick Start Recipes (copy/paste)

### A) AMR with ROS1 + SNMP (online or air-gapped)
```
source /opt/ros/noetic/setup.bash            # if available on host; skip inside our Docker image
export PYTHONPATH="$PWD/src:$PYTHONPATH"
python3 src/app/collect.py \
  --pose_source ros1 --pose_topic /amcl_pose \
  --snmp_host "$SNMP_HOST" --snmp_community "${SNMP_COMMUNITY:-public}" \
  --snmp_rssi_oid "$SNMP_RSSI_OID" \
  --snmp_ssid_oid "${SNMP_SSID_OID:-}" \
  --snmp_bssid_oid "${SNMP_BSSID_OID:-}" \
  --db data/amr_run01.sqlite

python3 src/render_from_sqlite.py --db data/amr_run01.sqlite --out_dir output/amr_run01
```
Notes: ROS1 env must provide `rospy` and `tf`; our Docker image includes `ros-noetic-tf`.

### B) Laptop/manual (keyboard pose)
```
export PYTHONPATH="$PWD/src:$PYTHONPATH"
python3 src/app/collect.py \
  --pose_source manual_shortcuts \
  --manual_step_m 0.5 --manual_turn_deg 90 \
  --hz 1 --db data/laptop_walk.sqlite --commit_every 1

python3 src/render_from_sqlite.py --db data/laptop_walk.sqlite --out_dir output/laptop_walk
```
Shortcuts: arrow/WASD/QE/RF for movement; Enter reuses last pose.

<<<<<<< HEAD
### C) Simulation sanity check
```
python3 src/simulate_walk.py --db data/sim_samples.sqlite --width_m 30 --height_m 20 --step_m 0.5 --hz 2 --seed 7
=======
## Simulation (no hardware)
Use this to sanity-check the render pipeline without a robot or Wi‑Fi:

What it does:
- Generates a lawnmower path over a rectangular area (default 30 m × 20 m, 0.5 m step).
- Synthesizes RSSI using a simple path-loss model against two virtual APs, adds Gaussian noise.
- Logs into SQLite with the same schema as real runs (roam markers included when BSSID changes).

Run and render:
```
python3 src/simulate_walk.py --db data/sim_samples.sqlite \
  --width_m 30 --height_m 20 --step_m 0.5 --hz 2 --seed 7

>>>>>>> f81c974 (Clarify simulation instructions)
python3 src/render_from_sqlite.py --db data/sim_samples.sqlite --out_dir output/sim
```
Notes:
- Tweak APs/area by editing `src/simulate_walk.py` (AP coords/tx power/path-loss exponent/noise).
- Outputs land in `data/sim_samples.sqlite` and `output/sim/report.html`.

### D) Air-gapped with Docker (recommended)
On an Internet host:
```
./scripts/offline_pack.sh --image-tag lexx500-wifi:offline --deb snmp
```
Carry `offline_bundle/` to site, then on the AMR:
```
docker load -i offline_bundle/lexx500-wifi-offline.tar
docker run -it --rm --net=host --privileged \
  -v "$PWD:/workspace" -w /workspace \
  lexx500-wifi:offline bash
# inside container
source /opt/ros/noetic/setup.bash
export PYTHONPATH=/workspace/src:$PYTHONPATH
python3 src/app/collect.py ...same as flow A...
```

### E) Air-gapped without Docker (bare metal)
On an Internet host:
```
./scripts/offline_pack.sh --image-tag lexx500-wifi:offline --deb snmp
```
On the AMR:
```
sudo dpkg -i offline_bundle/debs/snmp*.deb
python3 -m venv .venv && source .venv/bin/activate
pip install --no-index --find-links offline_bundle/wheelhouse -r requirements.txt
export PYTHONPATH="$PWD/src:$PYTHONPATH"
python3 src/app/collect.py ...same as flow A or B...
```

## Minimal Command Cheat Sheet
- ROS1 + SNMP: use flow A above.
- Manual keyboard: use flow B above.
- Export DB to CSV: `python3 src/export_sqlite_to_csv.py --db data/run.sqlite --out data/run.csv`
- Render existing DB: `python3 src/render_from_sqlite.py --db data/run.sqlite --out_dir output/run`

## Report Reading (fast)
- `rssi_median.png`: typical signal per cell (median)
- `rssi_p10.png`: worst-tendency per cell (10th percentile)
- `bad_zones.png`: cells where P10 < threshold (default -75 dBm)
- `path_overlay.png`: path with roam/disconnect markers
- `rssi_over_time.png`: stability over time
- `rssi_hist.png`: distribution spread

Rules of thumb: aim for P10 ≥ -75 dBm on routes; < -80 dBm is risky.

## Troubleshooting
- RSSI is None: wrong OID or SNMP disabled.
- Missing ROS deps: `import rospy`/`import tf` fails → use the Docker image or install `ros-noetic-tf` and source `/opt/ros/noetic/setup.bash`.
- Laptop mode can’t find Wi-Fi: pass `--interface wlpXsY`.
- Stop safely with Ctrl+C; commits are periodic.
