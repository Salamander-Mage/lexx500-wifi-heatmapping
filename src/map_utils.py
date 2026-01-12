from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
from PIL import Image


@dataclass
class MapImage:
    """Simple 2D map reference.

    We treat the image as a top-down map where:
      - (0,0) is bottom-left in "world" coordinates
      - image (0,0) is top-left in pixel coordinates

    We define a world coordinate frame in meters:
      x in [0, width_m], y in [0, height_m]
    """
    img_path: Optional[Path]
    width_m: float
    height_m: float
    img: Optional[Image.Image] = None

    def load(self) -> "MapImage":
        if self.img_path is None:
            return self
        self.img = Image.open(self.img_path).convert("RGBA")
        return self

    @property
    def size_px(self) -> Optional[Tuple[int, int]]:
        if self.img is None:
            return None
        return self.img.size  # (w, h)

    def world_to_pixel(self, x: float, y: float) -> Tuple[float, float]:
        """Convert world meters -> image pixels (float)."""
        if self.img is None:
            raise ValueError("MapImage.img is None. Call load() or set img=None and don't use world_to_pixel.")
        w_px, h_px = self.img.size
        px = (x / self.width_m) * w_px
        py = (1.0 - (y / self.height_m)) * h_px  # invert y (world up, image down)
        return px, py

    def pixel_to_world(self, px: float, py: float) -> Tuple[float, float]:
        """Convert image pixels -> world meters."""
        if self.img is None:
            raise ValueError("MapImage.img is None. Call load() first.")
        w_px, h_px = self.img.size
        x = (px / w_px) * self.width_m
        y = (1.0 - (py / h_px)) * self.height_m
        return x, y


def make_blank_map_png(path: Path, width_px: int = 900, height_px: int = 600) -> None:
    """Create a simple blank map background so you can test overlays quickly."""
    img = Image.new("RGBA", (width_px, height_px), (245, 245, 245, 255))
    img.save(path)


def grid_aggregate(
    xs: np.ndarray,
    ys: np.ndarray,
    values: np.ndarray,
    width_m: float,
    height_m: float,
    cell_m: float,
    stat: str = "median",
) -> Tuple[np.ndarray, np.ndarray]:
    """Aggregate values into a grid over [0,width_m]x[0,height_m].

    Returns:
      grid: shape (ny, nx) with aggregated value (np.nan where insufficient samples)
      counts: shape (ny, nx) with sample counts
    """
    if xs.shape != ys.shape or xs.shape != values.shape:
        raise ValueError("xs, ys, values must have same shape")

    nx = int(np.ceil(width_m / cell_m))
    ny = int(np.ceil(height_m / cell_m))

    grid = np.full((ny, nx), np.nan, dtype=float)
    counts = np.zeros((ny, nx), dtype=int)

    # bin indices
    ix = np.clip((xs / cell_m).astype(int), 0, nx - 1)
    iy = np.clip((ys / cell_m).astype(int), 0, ny - 1)

    # collect per-cell
    buckets: dict[tuple[int, int], list[float]] = {}
    for x_i, y_i, v in zip(ix, iy, values):
        key = (y_i, x_i)  # (row, col)
        buckets.setdefault(key, []).append(float(v))

    for (r, c), vals in buckets.items():
        counts[r, c] = len(vals)
        arr = np.array(vals, dtype=float)
        if stat == "median":
            grid[r, c] = float(np.median(arr))
        elif stat == "p10":
            grid[r, c] = float(np.percentile(arr, 10))
        else:
            raise ValueError(f"Unknown stat: {stat}")

    return grid, counts
