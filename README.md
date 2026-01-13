# Lexx500 Wi-Fi Heatmapping Tool

Wi-Fi survey + heatmapping tool for Lexx500 AMRs.  
Generates an HTML report with RSSI heatmaps (median + P10), bad-zone overlay, path overlay, and basic stability plots.

Supports workflows for:
- **Laptop walk survey** (quick baseline)
- **Robot NUC survey (SSH)** (best for V6 MOXA vs V7 FXE-5000 comparisons)

---

## What this produces

For a given survey DB (`.sqlite`), the renderer outputs:

- `rssi_median.png` — typical signal level per cell
- `rssi_p10.png` — worst-case tendency per cell (10th percentile)
- `bad_zones.png` — cells where P10 is below a threshold (e.g. -75 dBm)
- `path_overlay.png` — route/path with roam + disconnect markers
- `rssi_over_time.png` — signal over time
- `rssi_hist.png` — RSSI distribution
- `report.html` — single report embedding everything above

---
Glossary:

RSSI (Received Signal Strength Indicator)
A measurement of Wi-Fi signal strength at the client device, expressed in dBm.
Higher (closer to 0) is better.

Example: −55 dBm = strong signal

Example: −80 dBm = weak signal

Median RSSI (P50)
The middle value of all measured RSSI samples.
Half the samples are stronger, half are weaker.

Good for understanding typical coverage

Can hide short drops or unstable areas

P10 RSSI (10th Percentile RSSI)
The RSSI value below which 10% of samples fall.

In simple terms:

“In the worst ~10% of moments, the signal was this weak.”

Why it matters:

Captures brief drops, fades, and interference

Much better indicator of reliability than averages

Very important for mobile robots that cannot pause or retry easily

Bad Zone
A grid cell where the P10 RSSI falls below a defined threshold (e.g. −75 dBm) and enough samples were collected.

Interpretation:

Indicates areas where connectivity may be unreliable

Strong evidence for AP placement or channel plan issues

More meaningful than a single weak measurement

Roam Event
A detected change of Wi-Fi access point (BSSID) while remaining connected.

Expected in well-designed multi-AP environments

Too many → aggressive roaming

None → potential “sticky client” behavior

Disconnect Event
A moment when the Wi-Fi client loses connection entirely.

Usually more severe than roaming

Often correlated with very low RSSI, interference, or AP issues

Worst-Case Tendency
A practical way to describe P10 behavior:

“What the signal looks like when things are briefly bad.”

Used instead of “average” when reliability matters more than comfort.

Suggested Thresholds (Practical Guidance)

P10 ≥ −70 dBm → Very reliable

P10 ≥ −75 dBm → Acceptable for AMR routes

P10 < −75 dBm → Risk of instability

P10 < −80 dBm → High likelihood of disconnects

# ============================================================
# Lexx500 Wi-Fi Heatmapping Tool — Quick Start (Ubuntu 24.04)
# ============================================================

# --- 1) Create and activate virtual environment ---
python3 -m venv .venv
source .venv/bin/activate

# --- 2) Install dependencies ---
python3 -m pip install --upgrade pip
python3 -m pip install pandas numpy matplotlib pillow

# (Optional) Freeze dependencies
pip freeze > requirements.txt

# --- 3) Sanity check ---
python3 -m py_compile src/render_from_sqlite.py

# ============================================================
# 4) Collect Wi-Fi survey data
# ============================================================

# A) Laptop walk survey (baseline)
# Replace wlp9s0 with your Wi-Fi interface:
#   ip -br a | grep -E 'wl|wlp'
python3 -m src.app.collect \
  --db data/office_run.sqlite \
  --interface wlp9s0 \
  --hz 2

# ============================================================
# B) Robot AMR survey (recommended)
# ============================================================

# SSH into the AMR
ssh <user>@<AMR_ip>

# Verify Wi-Fi interface and ROS
ip -br a | grep -E 'wl|wlp'
rosnode list

# Run collector on the AMR
python3 -m src.app.collect \
  --db /tmp/klab_v6_moxa_run01.sqlite \
  --interface <robot_wifi_iface> \
  --hz 2

# Copy database back to laptop
scp <user>@<AMR_ip>:/tmp/klab_v6_moxa_run01.sqlite data/

# ============================================================
# 5) Render heatmaps + HTML report
# ============================================================

python3 -u src/render_from_sqlite.py \
  --db data/office_run.sqlite \
  --out_dir output/office_run_report \
  --cell_m 0.75 \
  --min_samples 2 \
  --bad_p10_threshold -75

# Open the report
xdg-open output/office_run_report/report.html
