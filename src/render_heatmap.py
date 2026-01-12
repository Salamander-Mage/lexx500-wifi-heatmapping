from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from map_utils import MapImage, make_blank_map_png, grid_aggregate


def plot_heatmap_on_map(
    map_img: MapImage,
    grid: np.ndarray,
    counts: np.ndarray,
    cell_m: float,
    title: str,
    out_path: Path,
    vmin: float = -90,
    vmax: float = -40,
    min_samples: int = 5,
) -> None:
    """
    grid shape is (ny, nx) where row=0 is y in [0,cell_m).
    We'll display it in world orientation (y increasing upward).
    """
    # Mask low-sample cells
    masked = grid.copy()
    masked[counts < min_samples] = np.nan

    fig = plt.figure(figsize=(10, 7))

    # Background map
    if map_img.img is not None:
        plt.imshow(map_img.img, extent=[0, map_img.width_m, 0, map_img.height_m], origin="lower")

    # Heat layer
    ny, nx = masked.shape
    x_edges = np.linspace(0, map_img.width_m, nx + 1)
    y_edges = np.linspace(0, map_img.height_m, ny + 1)

    plt.pcolormesh(x_edges, y_edges, masked, shading="auto", vmin=vmin, vmax=vmax)
    plt.colorbar(label="RSSI (dBm)")
    plt.title(title)
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")

    # Draw sample density warning
    plt.text(
        0.02,
        0.02,
        f"min_samples_per_cell={min_samples}",
        transform=plt.gca().transAxes,
        fontsize=9,
        bbox=dict(boxstyle="round", alpha=0.2),
    )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(out_path, dpi=160)
    plt.close(fig)


def write_simple_html_report(out_dir: Path, images: list[Path], summary_lines: list[str]) -> Path:
    html = ["<html><head><meta charset='utf-8'><title>Wi-Fi Heatmap Report</title></head><body>"]
    html.append("<h1>Wi-Fi Heatmap Report (Simulation)</h1>")
    html.append("<h2>Summary</h2><ul>")
    for line in summary_lines:
        html.append(f"<li>{line}</li>")
    html.append("</ul>")

    html.append("<h2>Outputs</h2>")
    for img in images:
        rel = img.name
        html.append(f"<h3>{rel}</h3>")
        html.append(f"<img src='{rel}' style='max-width: 100%; border: 1px solid #ccc;' />")

    html.append("</body></html>")

    out_path = out_dir / "report.html"
    out_path.write_text("\n".join(html), encoding="utf-8")
    return out_path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=str, default="data/sim_samples.csv")
    parser.add_argument("--out_dir", type=str, default="output")
    parser.add_argument("--map_png", type=str, default="maps/home_map.png")
    parser.add_argument("--width_m", type=float, default=30.0)
    parser.add_argument("--height_m", type=float, default=20.0)
    parser.add_argument("--cell_m", type=float, default=1.0)
    parser.add_argument("--min_samples", type=int, default=5)
    args = parser.parse_args()

    csv_path = Path(args.csv)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Ensure we have a background map image (optional)
    map_png = Path(args.map_png)
    if not map_png.exists():
        map_png.parent.mkdir(parents=True, exist_ok=True)
        make_blank_map_png(map_png)

    map_img = MapImage(map_png, width_m=args.width_m, height_m=args.height_m).load()

    df = pd.read_csv(csv_path)
    xs = df["x_m"].to_numpy(dtype=float)
    ys = df["y_m"].to_numpy(dtype=float)
    rssi = df["rssi_dbm"].to_numpy(dtype=float)

    grid_med, counts = grid_aggregate(xs, ys, rssi, args.width_m, args.height_m, args.cell_m, stat="median")
    grid_p10, _ = grid_aggregate(xs, ys, rssi, args.width_m, args.height_m, args.cell_m, stat="p10")

    img1 = out_dir / "rssi_median_heatmap.png"
    img2 = out_dir / "rssi_p10_heatmap.png"

    plot_heatmap_on_map(
        map_img=map_img,
        grid=grid_med,
        counts=counts,
        cell_m=args.cell_m,
        title="RSSI Median Heatmap (dBm)",
        out_path=img1,
        min_samples=args.min_samples,
    )

    plot_heatmap_on_map(
        map_img=map_img,
        grid=grid_p10,
        counts=counts,
        cell_m=args.cell_m,
        title="RSSI P10 Heatmap (dBm) — worst-case tendency",
        out_path=img2,
        min_samples=args.min_samples,
    )

    # Simple summary
    roam_count = int(df["roam_event"].sum()) if "roam_event" in df.columns else 0
    summary = [
        f"Samples: {len(df)}",
        f"Roam events: {roam_count}",
        f"Grid cell size: {args.cell_m} m",
        f"Min samples/cell to show value: {args.min_samples}",
        f"Map size: {args.width_m}m × {args.height_m}m",
    ]

    report = write_simple_html_report(out_dir, [img1, img2], summary)
    print(f"✅ wrote images: {img1}, {img2}")
    print(f"✅ wrote report: {report} (open in browser)")


if __name__ == "__main__":
    main()
