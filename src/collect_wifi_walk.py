import argparse
import time
from pathlib import Path

import pandas as pd

from wifi_reader_iw import read_wifi_iw, detect_wifi_interface


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="data/real_wifi_walk.csv")
    parser.add_argument("--interface", default="auto")  # allow auto-detect
    parser.add_argument("--hz", type=float, default=1.0)
    parser.add_argument("--step_m", type=float, default=0.5)
    parser.add_argument("--samples", type=int, default=120)
    args = parser.parse_args()   # <-- args is defined HERE

    # Resolve interface
    iface = args.interface
    if iface in ("", "auto"):
        detected = detect_wifi_interface()
        if not detected:
            raise RuntimeError(
                "Could not detect Wi-Fi interface automatically. "
                "Run `iw dev` and pass --interface <name>."
            )
        iface = detected

    print(f"Using Wi-Fi interface: {iface}")

    rows = []
    x, y = 0.0, 0.0
    prev_bssid = None

    for i in range(args.samples):
        rssi, bssid, ssid = read_wifi_iw(iface)

        roam = False
        if bssid and prev_bssid and bssid != prev_bssid:
            roam = True
        prev_bssid = bssid

        rows.append(
            {
                "ts_s": i / args.hz,
                "x_m": x,
                "y_m": y,
                "rssi_dbm": rssi,
                "ssid": ssid,
                "bssid": bssid,
                "roam_event": roam,
            }
        )

        # Fake walking in +X direction
        x += args.step_m

        time.sleep(1.0 / args.hz)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_csv(out, index=False)
    print(f"✅ wrote {len(rows)} samples -> {out}")
