from dataclasses import dataclass
from typing import Optional, Tuple
import threading


@dataclass
class Pose:
    x_m: float
    y_m: float
    theta_rad: float = 0.0


class ManualPoseProvider:
    """
    Pose is updated by user input in a background thread.
    Samples use the latest pose.
    """

    def __init__(self, initial: Pose = Pose(0.0, 0.0, 0.0)):
        self._pose = initial
        self._lock = threading.Lock()
        self._stop = False
        self._thread: Optional[threading.Thread] = None

    def start(self):
        def loop():
            print("\nManual pose mode:")
            print("  Type: x y   (meters) to set pose")
            print("  Type: q     to stop collection\n")
            while not self._stop:
                try:
                    line = input("> ").strip()
                except EOFError:
                    break
                if not line:
                    continue
                if line.lower() == "q":
                    self._stop = True
                    break
                parts = line.split()
                if len(parts) >= 2:
                    try:
                        x = float(parts[0])
                        y = float(parts[1])
                        with self._lock:
                            self._pose = Pose(x, y, self._pose.theta_rad)
                        print(f"Set pose to x={x:.2f}, y={y:.2f}")
                    except ValueError:
                        print("Invalid input. Example: 3.5 1.2")
                else:
                    print("Invalid input. Example: 3.5 1.2")

        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def should_stop(self) -> bool:
        return self._stop

    def get_pose(self) -> Pose:
        with self._lock:
            return self._pose
