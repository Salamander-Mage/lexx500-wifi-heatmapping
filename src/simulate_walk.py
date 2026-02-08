from __future__ import annotations

import argparse
import math
import random
import time
from pathlib import Path

import numpy as np

from app.logger_sqlite import SqliteLogger


def rssi_model(x: float, y: float, aps: list[dict]) -> tuple[float, str]:
    """Return RSSI (dBm) and chosen AP BSSID based on a simple path-loss model."""
    best_rssi = -999.0
    best_bssid = "UNKNOWN"

    for ap in aps:
        ax, ay = ap["x"], ap["y"]
        bssid = ap["bssid"]
        d = max(0.5, math.hypot(x - ax, y - ay))  # avoid log(0)

        tx = ap["tx_dbm"]
        n = ap["n"]
        noise = random.gauss(0, ap["noise_sigma"])

        rssi = tx - (10.0 * n * math.log10(d)) + noise

        if rssi > best_rssi:
            best_rssi = rssi
            best_bssid = bssid

    return best_rssi, best_bssid


def generate_lawnmower_path(width_m: float, height_m: float, step_m: float) -> tuple[np.ndarray, np.ndarray]:
    """Lawnmower coverage path (boustrophedon sweep)."""
    ys = np.arange(step_m, height_m, step_m)
    xs_list: list[float] = []
    ys_list: list[float] = []

    direction = 1
    for y in ys:
        xs = np.arange(step_m, width_m, step_m) if direction == 1 else np.arange(width_m - step_m, 0, -step_m)
        xs_list.extend(xs.tolist())
        ys_list.extend([y] * len(xs))
        direction *= -1

    return np.array(xs_list, dtype=float), np.array(ys_list, dtype=float)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--db", type=str, default="data/sim_samples.sqlite")
    parser.add_argument("--width_m", type=float, default=30.0)
    parser.add_argument("--height_m", type=float, default=20.0)
    parser.add_argument("--step_m", type=float, default=0.5)
    parser.add_argument("--hz", type=float, default=2.0)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)

    # Synthetic APs in world coordinates
    aps = [
        {"x": 5.0, "y": 5.0, "bssid": "AP1", "tx_dbm": -35.0, "n": 2.2, "noise_sigma": 1.5},
        {"x": 25.0, "y": 15.0, "bssid": "AP2", "tx_dbm": -34.0, "n": 2.4, "noise_sigma": 1.8},
    ]

    xs, ys = generate_lawnmower_path(args.width_m, args.height_m, args.step_m)

    db_path = Path(args.db)
    db_path.parent.mkdir(parents=True, exist_ok=True)

    logger = SqliteLogger(db_path)
    logger.open()

    prev_bssid = None
    start = time.time()

    for i, (x, y) in enumerate(zip(xs, ys)):
        rssi, bssid = rssi_model(float(x), float(y), aps)
        roam_event = 1 if (prev_bssid is not None and bssid != prev_bssid) else 0
        prev_bssid = bssid

        ts_unix_ms = int((start + i / args.hz) * 1000.0)

        row = {
            "ts_unix_ms": ts_unix_ms,
            "x_m": float(x),
            "y_m": float(y),
            "theta_rad": 0.0,
            "connected": 1,
            "ssid": None,
            "bssid": bssid,
            "rssi_dbm": round(float(rssi), 1),
            "freq_mhz": None,
            "roam_event": roam_event,
            "disconnect_event": 0,
            "disconnect_streak_s": 0.0,
            "ping_loss_pct": None,
            "ping_avg_ms": None,
        }

        logger.insert_sample(row)

        if (i + 1) % 50 == 0:
            logger.commit()

    logger.commit()
    logger.close()
    print(f"✅ wrote {len(xs)} synthetic samples -> {db_path}")


if __name__ == "__main__":
    main()
