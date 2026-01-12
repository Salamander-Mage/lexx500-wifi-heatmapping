import argparse
import sqlite3
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from map_utils import MapImage, make_blank_map_png, grid_aggregate


def plot_heatmap(map_img, grid, counts, title, out_path, vmin=-90, vmax=-40, min_samples=3):
    """Render a heatmap on top of an optional floorplan image."""
    grid = grid.copy()
    grid[counts < min_samples] = np.nan

    fig = plt.figure(figsize=(10, 7))

    if map_img.img is not None:
        plt.imshow(map_img.img, extent=[0, map_img.width_m, 0, map_img.height_m], origin="lower")

    ny, nx = grid.shape
    x_edges = np.linspace(0, map_img.width_m, nx + 1)
    y_edges = np.linspace(0, map_img.height_m, ny + 1)

    plt.pcolormesh(x_edges, y_edges, grid, shading="auto", vmin=vmin, vmax=vmax)
    plt.colorbar(label="RSSI (dBm)")
    plt.title(title)
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.tight_layout()

    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=160)
    plt.close(fig)


def plot_bad_zones(map_img, grid_p10, counts, threshold_dbm, out_path, min_samples=3):
    """
    Overlay cells where P10 RSSI is below threshold.
    Returns number of 'bad' cells.
    """
    bad = (counts >= min_samples) & np.isfinite(grid_p10) & (grid_p10 < threshold_dbm)

    fig = plt.figure(figsize=(10, 7))

    if map_img.img is not None:
        plt.imshow(map_img.img, extent=[0, map_img.width_m, 0, map_img.height_m], origin="lower")

    ny, nx = bad.shape
    x_edges = np.linspace(0, map_img.width_m, nx + 1)
    y_edges = np.linspace(0, map_img.height_m, ny + 1)

    overlay = bad.astype(float)
    overlay[~bad] = np.nan

    plt.pcolormesh(x_edges, y_edges, overlay, shading="auto")
    plt.title(f"Bad Zones (P10 < {threshold_dbm:.0f} dBm)")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.tight_layout()

    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=160)
    plt.close(fig)

    return int(bad.sum())


def plot_path_overlay(map_img, df_all, out_path):
    """Plot the walked path and mark roam/disconnect events."""
    fig = plt.figure(figsize=(10, 7))

    if map_img.img is not None:
        plt.imshow(map_img.img, extent=[0, map_img.width_m, 0, map_img.height_m], origin="lower")

    plt.plot(df_all["x_m"], df_all["y_m"], linewidth=1, label="path")

    if "roam_event" in df_all.columns:
        roam = df_all[df_all["roam_event"] == 1]
        if len(roam) > 0:
            plt.scatter(roam["x_m"], roam["y_m"], marker="x", s=50, label="roam")

    if "disconnect_event" in df_all.columns:
        disc = df_all[df_all["disconnect_event"] == 1]
        if len(disc) > 0:
            plt.scatter(disc["x_m"], disc["y_m"], marker="o", s=50, label="disconnect")

    plt.title("Path overlay (roam / disconnect)")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.legend(loc="upper right")
    plt.tight_layout()

    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=160)
    plt.close(fig)


def plot_rssi_over_time(df_valid, out_path):
    t = (df_valid["ts_unix_ms"] - df_valid["ts_unix_ms"].iloc[0]) / 1000.0
    r = df_valid["rssi_dbm"].to_numpy(dtype=float)

    fig = plt.figure(figsize=(10, 4))
    plt.plot(t, r, linewidth=1)
    plt.title("RSSI over time (connected samples)")
    plt.xlabel("Time (s)")
    plt.ylabel("RSSI (dBm)")
    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=160)
    plt.close(fig)


def plot_rssi_histogram(df_valid, out_path):
    r = df_valid["rssi_dbm"].to_numpy(dtype=float)

    fig = plt.figure(figsize=(8, 4))
    plt.hist(r, bins=30)
    plt.title("RSSI distribution (connected samples)")
    plt.xlabel("RSSI (dBm)")
    plt.ylabel("Count")
    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=160)
    plt.close(fig)


def recommend_from_data(df_all, df_valid, grid_p10, counts, min_samples, bad_threshold):
    rec = []

    rssi_vals = df_valid["rssi_dbm"].to_numpy(dtype=float)
    rssi_p10 = float(np.percentile(rssi_vals, 10))
    rssi_p50 = float(np.percentile(rssi_vals, 50))
    rssi_p90 = float(np.percentile(rssi_vals, 90))

    valid_cells = (counts >= min_samples) & np.isfinite(grid_p10)
    bad_cells_mask = valid_cells & (grid_p10 < bad_threshold)

    n_valid_cells = int(valid_cells.sum())
    n_bad_cells = int(bad_cells_mask.sum())
    bad_pct = (100.0 * n_bad_cells / n_valid_cells) if n_valid_cells > 0 else 0.0

    roam_cnt = int(df_all["roam_event"].sum()) if "roam_event" in df_all.columns else 0
    disc_cnt = int(df_all["disconnect_event"].sum()) if "disconnect_event" in df_all.columns else 0

    rec.append(f"Observed RSSI (connected samples): P10={rssi_p10:.0f} dBm, median={rssi_p50:.0f} dBm, P90={rssi_p90:.0f} dBm.")
    rec.append(f"Grid cells with usable samples: {n_valid_cells}. Cells with P10 < {bad_threshold:.0f} dBm: {n_bad_cells} ({bad_pct:.1f}%).")
    rec.append(f"Roam events: {roam_cnt}. Disconnect events: {disc_cnt}.")

    if n_bad_cells == 0 and disc_cnt == 0:
        rec.append("Coverage on the surveyed path appears strong; if issues occur, prioritize investigating roaming thresholds, interference/airtime congestion, or upstream network conditions.")
    elif n_bad_cells > 0:
        rec.append("Weak areas detected (low P10). If these overlap with robot routes, prioritize AP placement/channel plan improvements before relying on client-side tuning.")

    rec.append("Suggested field test targets (validate by re-survey after each change):")
    rec.append(" - Keep traversal routes above ~-75 dBm at P10 (worst-case tendency).")
    rec.append(" - In overlap zones where roaming should occur, aim for ~-70 to -75 dBm with stable SNR to avoid sticky-client behavior.")
    if roam_cnt == 0:
        rec.append(" - No roaming observed in this survey. If your deployment requires multi-AP roaming, survey overlap zones between APs and confirm BSSID transitions occur without packet loss.")
    if disc_cnt > 0:
        rec.append(" - Disconnects observed: correlate points on path overlay with RSSI drops; if RSSI is not low, investigate interference, DHCP issues, or AP/client timeout settings.")

    if "ping_loss_pct" in df_all.columns and df_all["ping_loss_pct"].notna().any():
        loss_p50 = float(np.nanpercentile(df_all["ping_loss_pct"].to_numpy(dtype=float), 50))
        rtt_p50 = float(np.nanpercentile(df_all["ping_avg_ms"].to_numpy(dtype=float), 50)) if "ping_avg_ms" in df_all.columns else float("nan")
        rec.append(f"Probe summary: median loss={loss_p50:.0f}%, median RTT={rtt_p50:.1f} ms.")
        rec.append(" - If RSSI is good but RTT/loss spikes, suspect airtime congestion, channel overlap, or upstream network bottlenecks.")

    return rec


def write_html(out_dir: Path, summary: list[str], recommendations: list[str], images: list[dict]):
    css = """
    :root{--bg:#0b1220;--card:#111a2e;--muted:#9aa4b2;--text:#e7eefc;--border:rgba(255,255,255,.08)}
    *{box-sizing:border-box} body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial;
    background:linear-gradient(180deg,#070c16 0%,#0b1220 40%,#0b1220 100%);color:var(--text)}
    .wrap{max-width:1100px;margin:0 auto;padding:28px 18px 60px}
    h1{margin:0;font-size:26px} .sub{color:var(--muted);font-size:13px;margin-top:6px}
    .grid{display:grid;gap:14px} .cards{grid-template-columns:repeat(4,minmax(0,1fr));margin-top:18px}
    .card{background:rgba(17,26,46,.95);border:1px solid var(--border);border-radius:14px;padding:14px;
    box-shadow:0 10px 30px rgba(0,0,0,.22)}
    .k{color:var(--muted);font-size:12px;margin-bottom:6px} .v{font-size:18px;font-weight:700}
    .section{margin-top:18px} .section h2{margin:0 0 10px 0;font-size:16px}
    ul{margin:0;padding-left:18px} li{margin:6px 0;line-height:1.35;color:#d6e2ff}
    .gallery{grid-template-columns:repeat(2,minmax(0,1fr))}
    .imgcard img{width:100%;height:auto;border-radius:12px;border:1px solid var(--border);background:#0a1020}
    .imgtitle{margin:10px 0 4px;font-weight:700} .imgcap{margin:0;color:var(--muted);font-size:13px}
    details{margin-top:10px} summary{cursor:pointer;color:#cfe0ff}
    @media(max-width:980px){.cards{grid-template-columns:repeat(2,minmax(0,1fr))}}
    @media(max-width:640px){.cards,.gallery{grid-template-columns:1fr}}
    """

    def find(prefix: str, default="—"):
        for s in summary:
            if s.startswith(prefix):
                return s.split(":", 1)[1].strip()
        return default

    stat_total = find("Total samples")
    stat_used = find("Samples used for heatmap (connected + RSSI)")
    stat_roam = find("Roam events")
    stat_disc = find("Disconnect events")

    html = []
    html.append("<html><head><meta charset='utf-8'>")
    html.append("<meta name='viewport' content='width=device-width, initial-scale=1'/>")
    html.append("<title>Wi-Fi Heatmap Report</title>")
    html.append(f"<style>{css}</style></head><body><div class='wrap'>")

    html.append("<div>")
    html.append("<h1>Wi-Fi Heatmap Report</h1>")
    html.append("<div class='sub'>Lexx500 Wi-Fi survey output (RSSI + roam/disconnect events)</div>")
    html.append("</div>")

    html.append("<div class='grid cards'>")
    html.append(f"<div class='card'><div class='k'>Total samples</div><div class='v'>{stat_total}</div></div>")
    html.append(f"<div class='card'><div class='k'>Heatmap samples used</div><div class='v'>{stat_used}</div></div>")
    html.append(f"<div class='card'><div class='k'>Roam events</div><div class='v'>{stat_roam}</div></div>")
    html.append(f"<div class='card'><div class='k'>Disconnect events</div><div class='v'>{stat_disc}</div></div>")
    html.append("</div>")

    html.append("<div class='section card'>")
    html.append("<h2>Recommendations</h2><ul>")
    for r in recommendations:
        html.append(f"<li>{r}</li>")
    html.append("</ul></div>")

    html.append("<div class='section card'>")
    html.append("<h2>Outputs</h2>")
    html.append("<div class='grid gallery'>")
    for im in images:
        f = im["file"]
        title = im.get("title", f)
        cap = im.get("caption", "")
        html.append("<div class='imgcard'>")
        html.append(f"<img src='{f}'/>")
        html.append(f"<div class='imgtitle'>{title}</div>")
        if cap:
            html.append(f"<p class='imgcap'>{cap}</p>")
        html.append("</div>")
    html.append("</div></div>")

    html.append("<details class='section'><summary>Run details</summary>")
    html.append("<div class='card' style='margin-top:10px;'><ul>")
    for s in summary:
        html.append(f"<li>{s}</li>")
    html.append("</ul></div></details>")

    html.append("</div></body></html>")
    (out_dir / "report.html").write_text("\n".join(html), encoding="utf-8")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--db", required=True)
    p.add_argument("--out_dir", default="output")
    p.add_argument("--map_png", default="", help="Optional floorplan PNG. Leave blank for blank background.")

    p.add_argument("--auto_fit", action="store_true", default=True)
    p.add_argument("--no_auto_fit", action="store_true", help="Disable auto-fit and use provided width/height.")
    p.add_argument("--margin_m", type=float, default=1.0)

    p.add_argument("--shift_to_origin", action="store_true", default=True)
    p.add_argument("--no_shift_to_origin", action="store_true")

    p.add_argument("--width_m", type=float, default=30.0)
    p.add_argument("--height_m", type=float, default=20.0)

    p.add_argument("--cell_m", type=float, default=1.0)
    p.add_argument("--min_samples", type=int, default=3)
    p.add_argument("--bad_p10_threshold", type=float, default=-75.0)

    args = p.parse_args()
    if args.no_auto_fit:
        args.auto_fit = False
    if args.no_shift_to_origin:
        args.shift_to_origin = False

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    conn = sqlite3.connect(args.db)
    df_all = pd.read_sql_query("SELECT * FROM samples ORDER BY ts_unix_ms ASC", conn)
    conn.close()
    if len(df_all) == 0:
        raise RuntimeError("No samples found in DB.")

    df_valid = df_all.copy()
    if "connected" in df_valid.columns:
        df_valid = df_valid[df_valid["connected"] == 1]
    df_valid = df_valid[df_valid["rssi_dbm"].notna()]
    if len(df_valid) == 0:
        raise RuntimeError("No valid RSSI samples (connected + rssi_dbm not null).")

    # time plots
    img_time = "rssi_over_time.png"
    img_hist = "rssi_hist.png"
    plot_rssi_over_time(df_valid, out_dir / img_time)
    plot_rssi_histogram(df_valid, out_dir / img_hist)

    xmin, xmax = float(df_valid["x_m"].min()), float(df_valid["x_m"].max())
    ymin, ymax = float(df_valid["y_m"].min()), float(df_valid["y_m"].max())

    if args.auto_fit:
        w = (xmax - xmin) + 2 * args.margin_m
        h = (ymax - ymin) + 2 * args.margin_m
        width_m = max(w, args.cell_m * 2)
        height_m = max(h, args.cell_m * 2)
    else:
        width_m = args.width_m
        height_m = args.height_m

    if args.shift_to_origin:
        x0 = xmin - args.margin_m
        y0 = ymin - args.margin_m
        df_all["x_m"] = df_all["x_m"] - x0
        df_all["y_m"] = df_all["y_m"] - y0
        df_valid["x_m"] = df_valid["x_m"] - x0
        df_valid["y_m"] = df_valid["y_m"] - y0

    map_png = Path(args.map_png) if args.map_png else None
    if map_png is None or not map_png.exists():
        blank_path = out_dir / "blank_map.png"
        make_blank_map_png(blank_path)
        map_png = blank_path

    map_img = MapImage(map_png, width_m=width_m, height_m=height_m).load()

    xs = df_valid["x_m"].to_numpy(dtype=float)
    ys = df_valid["y_m"].to_numpy(dtype=float)
    rssi = df_valid["rssi_dbm"].to_numpy(dtype=float)

    grid_med, counts = grid_aggregate(xs, ys, rssi, width_m, height_m, args.cell_m, stat="median")
    grid_p10, _ = grid_aggregate(xs, ys, rssi, width_m, height_m, args.cell_m, stat="p10")

    img_med = "rssi_median.png"
    img_p10 = "rssi_p10.png"
    img_bad = "bad_zones.png"
    img_path = "path_overlay.png"

    plot_heatmap(map_img, grid_med, counts, "RSSI Median Heatmap (dBm)", out_dir / img_med, min_samples=args.min_samples)
    plot_heatmap(map_img, grid_p10, counts, "RSSI P10 Heatmap (dBm) — worst-case tendency", out_dir / img_p10, min_samples=args.min_samples)

    bad_cells = plot_bad_zones(
        map_img, grid_p10, counts,
        threshold_dbm=args.bad_p10_threshold,
        out_path=out_dir / img_bad,
        min_samples=args.min_samples,
    )

    plot_path_overlay(map_img, df_all, out_dir / img_path)

    roam_cnt = int(df_all["roam_event"].sum()) if "roam_event" in df_all.columns else 0
    disc_cnt = int(df_all["disconnect_event"].sum()) if "disconnect_event" in df_all.columns else 0

    recommendations = recommend_from_data(
        df_all=df_all,
        df_valid=df_valid,
        grid_p10=grid_p10,
        counts=counts,
        min_samples=args.min_samples,
        bad_threshold=args.bad_p10_threshold,
    )

    summary = [
        f"DB: {args.db}",
        f"Total samples: {len(df_all)}",
        f"Samples used for heatmap (connected + RSSI): {len(df_valid)}",
        f"Roam events: {roam_cnt}",
        f"Disconnect events: {disc_cnt}",
        f"Grid: cell_m={args.cell_m}, min_samples={args.min_samples}",
        f"Auto-fit: {args.auto_fit}, shift_to_origin: {args.shift_to_origin}, margin_m={args.margin_m}",
        f"Map extent: {width_m:.2f}m × {height_m:.2f}m",
        f"Bad cells (P10 < {args.bad_p10_threshold:.0f} dBm): {bad_cells}",
        f"Original x range: [{xmin:.2f}, {xmax:.2f}], y range: [{ymin:.2f}, {ymax:.2f}]",
    ]

    images = [
        {"file": img_med, "title": "RSSI Median Heatmap", "caption": "Typical signal level per cell (median)."},
        {"file": img_p10, "title": "RSSI P10 Heatmap", "caption": "Worst-case tendency per cell (10th percentile)."},
        {"file": img_bad, "title": "Bad Zones Overlay", "caption": f"Cells where P10 < {args.bad_p10_threshold:.0f} dBm (and enough samples)."},
        {"file": img_path, "title": "Path Overlay", "caption": "Route path + roam/disconnect markers."},
        {"file": img_time, "title": "RSSI Over Time", "caption": "Spot dips/instability even if averages look OK."},
        {"file": img_hist, "title": "RSSI Histogram", "caption": "Overall RSSI distribution (connected samples)."},
    ]

    write_html(out_dir, summary, recommendations, images)
    print(f"✅ Wrote report: {out_dir / 'report.html'}")


if __name__ == "__main__":
    main()
