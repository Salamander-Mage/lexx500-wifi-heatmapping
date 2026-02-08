# Lexx500 Wi-Fi Heatmapping

Collect Wi-Fi signal data along a path (robot or laptop), store it in SQLite/CSV, and render heatmaps plus an HTML report that highlights weak zones, roam/disconnect events, and basic stats.

## What’s Included
- Data collection (ROS1 pose + SNMP Wi-Fi on AMR, or manual laptop walk)
- Simulation generator (synthetic walk + RSSI model)
- Rendering to PNG heatmaps and HTML report
- Offline pack script for air-gapped sites

## Quick Start (Robot, ROS1 + SNMP)
0) Prereq: find the correct OIDs for your hardware/site (do this once):
  - Check vendor docs for RSSI/SSID/BSSID OIDs (preferred).
  - If unknown, run a targeted walk (from a laptop on the same VLAN):
    - `snmpwalk -v2c -c <community> <host> 1.3.6.1.4.1 | head -n 200` (scan the enterprise tree).
    - Or narrower guesses (replace OIDs with likely enterprise roots):
     - FXE5000 root: `snmpwalk -v2c -c <community> <host> 1.3.6.1.4.1.672`
     - MOXA root:    `snmpwalk -v2c -c <community> <host> 1.3.6.1.4.1.8691`
  - Locate values that change with signal (RSSI dBm, negative numbers) and strings for SSID/BSSID.
  - Verify with `snmpget -v2c -c <community> <host> <candidate_oid>` while moving the device; RSSI should vary.

> AMR note: HybridAMR on the robot must match the version actually installed on your AMR; fielded units often lag the latest release. Mismatched images can miss deps or have ROS/network stack differences.

1) Provide SNMP details (env file recommended):
  - Copy `snmp_oids.example.env` to `snmp_oids.env`, edit the IP/community, and choose the correct OID block.
  - Load it: `source snmp_oids.env`
  Env vars needed: SNMP_HOST, SNMP_COMMUNITY (default public), SNMP_RSSI_OID (required), optional SNMP_SSID_OID, SNMP_BSSID_OID.

2) Collect (uses ROS1 `/amcl_pose` and SNMP Wi-Fi):
```
PYTHONPATH="$PWD/src" python3 -m app.collect \
  --pose_source ros1 --pose_topic /amcl_pose \
  --snmp_host "$SNMP_HOST" --snmp_community "${SNMP_COMMUNITY:-public}" \
  --snmp_rssi_oid "$SNMP_RSSI_OID" \
  --snmp_ssid_oid "${SNMP_SSID_OID:-}" \
  --snmp_bssid_oid "${SNMP_BSSID_OID:-}" \
  --db data/amr_run01.sqlite
```
Stop with Ctrl+C. Samples land in `data/amr_run01.sqlite`.

3) Render report:
```
python3 src/render_from_sqlite.py --db data/amr_run01.sqlite --out_dir output/amr_run01
```
Open `output/amr_run01/report.html`.

## Quick Start (Laptop Walk, Manual Pose)
Prereqs: Linux, `iw`, Python 3.

1) Collect manual walk (prompts for x y yaw):
```
PYTHONPATH="$PWD/src" python3 -m app.collect \
  --pose_source manual --manual_pose \
  --manual_step_m 0.5 --manual_turn_deg 90 \
  --hz 1 --db data/laptop_walk.sqlite --commit_every 1
```
Shortcuts during entry: Enter=reuse last, `f`/`b` move forward/back by step_m, `l`/`r` rotate by turn_deg.

2) Render:
```
python3 src/render_from_sqlite.py --db data/laptop_walk.sqlite --out_dir output/laptop_walk
```
Open `output/laptop_walk/report.html`.

## Simulation (no hardware)
Generate synthetic samples into SQLite and render with the main pipeline:
```
python3 src/simulate_walk.py --db data/sim_samples.sqlite
python3 src/render_from_sqlite.py --db data/sim_samples.sqlite --out_dir output/sim
```

## Offline/air-gapped sites
- If the AMR VLAN has no Internet (common for secured worksites), build an offline bundle on a connected machine:
  - `./scripts/offline_pack.sh --image-tag lexx500-wifi:offline --deb snmp`
  - Artifacts go to `offline_bundle/`: Docker image tar, wheelhouse, SNMP debs, and repo archive.
- On site: `docker load -i offline_bundle/lexx500-wifi-offline.tar` (or `pip install --no-index --find-links offline_bundle/wheelhouse -r requirements.txt` if running baremetal) and `sudo dpkg -i offline_bundle/debs/snmp*.deb` for SNMP tools.
- Container helper: `scripts/run_noetic_container.sh` mounts the repo and uses the prebuilt image (includes numpy/pandas/matplotlib/pillow + SNMP tools baked into Dockerfile.noetic).

## Offline Bundle (no Internet on site)
Build offline assets on a connected machine:
```
./scripts/offline_pack.sh --image-tag lexx500-wifi:offline --deb snmp
```
Artifacts: `offline_bundle/` containing wheelhouse, SNMP debs, Docker image tar, and repo archive.

Use on site:
- If Docker allowed: `docker load -i offline_bundle/lexx500-wifi-offline.tar`
- Without Docker: `pip install --no-index --find-links offline_bundle/wheelhouse -r requirements.txt`
- Install SNMP tools: `sudo dpkg -i offline_bundle/debs/snmp*.deb`

## Minimal Command Cheat Sheet
- ROS1 + SNMP: see Quick Start (Robot)
- Manual laptop: see Quick Start (Laptop Walk)
- Export DB to CSV: `python3 src/export_sqlite_to_csv.py --db data/run.sqlite --out data/run.csv`
- Render existing DB: `python3 src/render_from_sqlite.py --db data/run.sqlite --out_dir output/run`

## Report Reading (fast)
- `rssi_median.png`: typical signal per cell (median)
- `rssi_p10.png`: worst-tendency per cell (10th percentile) — most important
- `bad_zones.png`: cells where P10 < threshold (default -75 dBm)
- `path_overlay.png`: path with roam/disconnect markers
- `rssi_over_time.png`: stability over time
- `rssi_hist.png`: distribution spread

Rules of thumb:
- Aim for P10 ≥ -75 dBm on routes; < -80 dBm is risky
- If median is fine but P10 is poor, expect intermittents
- Align bad zones with robot routes to prioritize fixes

## Troubleshooting
- RSSI is None: wrong OID or SNMP disabled
- No pose in ROS mode: check `/amcl_pose` and ROS networking; `pose_provider.has_pose()` must become true
- Laptop mode no Wi-Fi: ensure `iw` interface auto-detected or pass `--interface wlpXsY`
- Probes: `ping` parsing is Linux-centric; on macOS/BusyBox results may be missing

## Notes
- Wi-Fi OIDs are vendor-specific; validate once per site/hardware.
- Values assumed to be RSSI in dBm (negative integers).
- Stop collection safely with Ctrl+C; commits are periodic.