from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, List, Tuple
import threading
import time

import matplotlib.pyplot as plt
import matplotlib.image as mpimg


@dataclass
class Pose:
    x_m: float
    y_m: float
    theta_rad: float = 0.0


class ClickMapPoseProvider:
    """
    Shows a map image and lets you click your current (x,y) position in meters.
    Samples use the latest pose.

    Optional: interpolate between the last 2 clicks for smoother motion.
    """

    def __init__(self, map_png: str, width_m: float, height_m: float, interpolate: bool = True):
        self.map_png = map_png
        self.width_m = width_m
        self.height_m = height_m
        self.interpolate = interpolate

        self._lock = threading.Lock()
        self._stop = False

        # Store click history as (t, x, y)
        self._clicks: List[Tuple[float, float, float]] = []
        self._latest_pose = Pose(0.0, 0.0, 0.0)

        self._thread: Optional[threading.Thread] = None

    def start(self):
        def ui_loop():
            img = mpimg.imread(self.map_png)

            fig, ax = plt.subplots(figsize=(10, 7))
            ax.imshow(img, extent=[0, self.width_m, 0, self.height_m], origin="lower")
            ax.set_title("Click your current position. Close window or press 'q' to stop.")
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")

            marker, = ax.plot([], [], "o")  # current point marker

            def on_click(event):
                if event.inaxes != ax:
                    return
                x = float(event.xdata)
                y = float(event.ydata)
                t = time.time()
                with self._lock:
                    self._clicks.append((t, x, y))
                    self._latest_pose = Pose(x, y, self._latest_pose.theta_rad)
                marker.set_data([x], [y])
                fig.canvas.draw_idle()
                print(f"[pose] clicked x={x:.2f}, y={y:.2f}")

            def on_key(event):
                if event.key and event.key.lower() == "q":
                    self._stop = True
                    plt.close(fig)

            fig.canvas.mpl_connect("button_press_event", on_click)
            fig.canvas.mpl_connect("key_press_event", on_key)

            plt.show()

            # if window closed
            self._stop = True

        self._thread = threading.Thread(target=ui_loop, daemon=True)
        self._thread.start()

    def should_stop(self) -> bool:
        return self._stop

    def get_pose(self) -> Pose:
        """
        If interpolate=True and we have >=2 clicks, estimate pose between last two clicks.
        Otherwise, return latest clicked pose.
        """
        with self._lock:
            if not self.interpolate or len(self._clicks) < 2:
                return self._latest_pose

            (t0, x0, y0), (t1, x1, y1) = self._clicks[-2], self._clicks[-1]
            now = time.time()

        # If time ordering weird or too close, just return latest
        if t1 <= t0:
            return Pose(x1, y1, 0.0)

        # clamp interpolation parameter to [0,1]
        alpha = (now - t0) / (t1 - t0)
        if alpha < 0:
            alpha = 0.0
        if alpha > 1:
            alpha = 1.0

        xi = x0 + (x1 - x0) * alpha
        yi = y0 + (y1 - y0) * alpha
        return Pose(xi, yi, 0.0)
