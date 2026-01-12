from __future__ import annotations

import argparse
from pathlib import Path
import math
import random

import numpy as np
import pandas as pd


def rssi_model(x: float, y: float, aps: list[dict]) -> tuple[float, str]:
    """Return RSSI (dBm) and chosen AP BSSID based on simple path-loss-like model.

    We pick the AP with the strongest received power.
    """
    best_rssi = -999.0
    best_bssid = "UNKNOWN"

    for ap in aps:
        ax, ay = ap["x"], ap["y"]
        bssid = ap["bssid"]
        # distance in meters (avoid 0)
        d = max(0.5, math.hypot(x - ax, y - ay))

        # simple path loss model:
        # rssi = tx_power - 10*n*log10(d) + noise
        tx = ap["tx_dbm"]  # "effective"
        n = ap["n"]        # path loss exponent
        noise = random.gauss(0, ap["noise_sigma"])

        rssi = tx - (10.0 * n * math.log10(d)) + noise

        if rssi > best_rssi:
            best_rssi = rssi
            best_bssid = bssid

    return best_rssi, best_bssid


def generate_lawnmower_path(width_m: float, height_m: float, step_m: float) -> tuple[np.ndarray, np.ndarray]:
    """Lawnmower coverage path."""
    ys = np.arange(step_m, height_m, step_m)
    xs_list = []
    ys_list = []

    direction = 1
    for y in ys:
        if direction == 1:
            xs = np.arange(step_m, width_m, step_m)
        else:
            xs = np.arange(width_m - step_m, 0, -step_m)
        xs_list.extend(xs.tolist())
        ys_list.extend([y] * len(xs))
        direction *= -1

    return np.array(xs_list), np.array(ys_list)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=str, default="data/sim_samples.csv")
    parser.add_argument("--width_m", type=float, default=30.0)
    parser.add_argument("--height_m", type=float, default=20.0)
    parser.add_argument("--step_m", type=float, default=0.5)
    parser.add_argument("--hz", type=float, default=2.0)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)

    # Define a couple of "APs" in world coordinates.
    aps = [
        {"x": 5.0, "y": 5.0, "bssid": "AP1", "tx_dbm": -35.0, "n": 2.2, "noise_sigma": 1.5},
        {"x": 25.0, "y": 15.0, "bssid": "AP2", "tx_dbm": -34.0, "n": 2.4, "noise_sigma": 1.8},
    ]

    xs, ys = generate_lawnmower_path(args.width_m, args.height_m, args.step_m)

    ts = np.arange(0, len(xs)) / args.hz

    rssi_vals = []
    bssids = []
    for x, y in zip(xs, ys):
        rssi, bssid = rssi_model(float(x), float(y), aps)
        rssi_vals.append(rssi)
        bssids.append(bssid)

    # roam events = BSSID change
    roam = [False]
    for i in range(1, len(bssids)):
        roam.append(bssids[i] != bssids[i - 1])

    df = pd.DataFrame(
        {
            "ts_s": ts,
            "x_m": xs,
            "y_m": ys,
            "rssi_dbm": np.round(rssi_vals, 1),
            "bssid": bssids,
            "roam_event": roam,
        }
    )

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(out_path, index=False)
    print(f"✅ wrote {len(df)} samples -> {out_path}")


if __name__ == "__main__":
    main()
