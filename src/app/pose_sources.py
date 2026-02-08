# src/app/pose_sources.py
from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class Pose2D:
    """2D pose in meters + yaw in radians."""
    x: float
    y: float
    yaw: float


def _normalize_angle_rad(a: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Return yaw (rotation around Z) from quaternion."""
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PoseProvider:
    """Interface: pose providers must implement get_pose()."""

    def get_pose(self) -> Pose2D:
        raise NotImplementedError


class ROS1PoseProvider(PoseProvider):
    """
    ROS1 pose provider (e.g., /amcl_pose PoseWithCovarianceStamped).
    Lazily imports rospy so laptop-only runs don't require ROS.
    """

    def __init__(self, topic: str = "/amcl_pose", wait_timeout_s: float = 10.0):
        self.topic = topic
        self.wait_timeout_s = float(wait_timeout_s)

        # Lazy imports so this file doesn't break outside ROS environments.
        import rospy  # type: ignore
        from geometry_msgs.msg import PoseWithCovarianceStamped  # type: ignore

        self._rospy = rospy
        self._PoseMsg = PoseWithCovarianceStamped
        self._last: Optional[Pose2D] = None

        self._sub = rospy.Subscriber(self.topic, self._PoseMsg, self._cb, queue_size=1)

    def _cb(self, msg) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
        self._last = Pose2D(float(p.x), float(p.y), float(yaw))

    def get_pose(self) -> Pose2D:
        start = time.time()
        while self._last is None:
            if time.time() - start > self.wait_timeout_s:
                raise TimeoutError(
                    f"Timed out waiting for first pose on topic {self.topic}"
                )
            # Let rospy callbacks run
            self._rospy.sleep(0.05)

        return self._last


class ManualPoseProvider(PoseProvider):
    """
    Terminal-based manual pose entry with small-step shortcuts.

    Inputs:
      - `x y yaw` (deg by default) or `x y yaw rad`
      - ENTER: reuse last pose
      - `f` / `forward`: move forward step_m along current yaw
      - `b` / `back`: move backward step_m along current yaw
      - `l` / `left`: rotate +turn_deg (no translation)
      - `r` / `right`: rotate -turn_deg (no translation)
    """

    def __init__(self, default_yaw_unit: str = "deg", step_m: float = 0.5, turn_deg: float = 90.0):
        self.default_yaw_unit = default_yaw_unit.lower().strip()
        if self.default_yaw_unit not in ("deg", "rad"):
            self.default_yaw_unit = "deg"
        self.step_m = float(step_m)
        self.turn_rad = math.radians(float(turn_deg))
        self._last: Optional[Pose2D] = None

    def _prompt(self) -> Pose2D:
        if self._last is None:
            msg = (
                "Enter pose: x y yaw (deg default), or shortcuts: f/b forward/back, l/r rotate, Enter=repeat\n"
                "> "
            )
        else:
            msg = (
                "Enter pose (or Enter keep, f/b move, l/r rotate): "
                f"{self._last.x:.2f} {self._last.y:.2f} {math.degrees(self._last.yaw):.1f}deg\n> "
            )

        try:
            line = input(msg).strip().lower()
        except EOFError:
            raise KeyboardInterrupt

        # Empty -> reuse
        if line == "":
            if self._last is None:
                print("No previous pose yet. Please enter x y yaw.", file=sys.stderr)
                return self._prompt()
            return self._last

        # Shortcuts require a previous pose
        if line in ("f", "forward"):
            if self._last is None:
                print("Need an initial pose before using forward/back.", file=sys.stderr)
                return self._prompt()
            dx = self.step_m * math.cos(self._last.yaw)
            dy = self.step_m * math.sin(self._last.yaw)
            return Pose2D(self._last.x + dx, self._last.y + dy, self._last.yaw)

        if line in ("b", "back", "backward"):
            if self._last is None:
                print("Need an initial pose before using forward/back.", file=sys.stderr)
                return self._prompt()
            dx = -self.step_m * math.cos(self._last.yaw)
            dy = -self.step_m * math.sin(self._last.yaw)
            return Pose2D(self._last.x + dx, self._last.y + dy, self._last.yaw)

        if line in ("l", "left"):
            if self._last is None:
                print("Need an initial pose before rotating.", file=sys.stderr)
                return self._prompt()
            return Pose2D(self._last.x, self._last.y, _normalize_angle_rad(self._last.yaw + self.turn_rad))

        if line in ("r", "right"):
            if self._last is None:
                print("Need an initial pose before rotating.", file=sys.stderr)
                return self._prompt()
            return Pose2D(self._last.x, self._last.y, _normalize_angle_rad(self._last.yaw - self.turn_rad))

        # Full pose entry
        parts = line.split()
        if len(parts) < 3:
            print("Need at least: x y yaw", file=sys.stderr)
            return self._prompt()

        try:
            x = float(parts[0])
            y = float(parts[1])
            yaw = float(parts[2])
        except ValueError:
            print("Could not parse numbers. Example: 1.0 2.0 90", file=sys.stderr)
            return self._prompt()

        unit = self.default_yaw_unit
        if len(parts) >= 4:
            unit = parts[3].lower()

        if unit in ("deg", "degree", "degrees"):
            yaw_rad = math.radians(yaw)
        elif unit in ("rad", "radian", "radians"):
            yaw_rad = yaw
        else:
            yaw_rad = math.radians(yaw) if self.default_yaw_unit == "deg" else yaw

        return Pose2D(float(x), float(y), _normalize_angle_rad(float(yaw_rad)))

    def get_pose(self) -> Pose2D:
        self._last = self._prompt()
        return self._last


# Backwards-compatible name expected by collect.py / older docs
ClickMapPoseProvider = ManualPoseProvider
