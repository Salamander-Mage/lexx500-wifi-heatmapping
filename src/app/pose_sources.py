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
    Terminal-based manual pose entry.

    Workflow:
      - First call prompts for x y yaw_deg (or x y yaw_rad)
      - Next calls allow pressing Enter to reuse last pose
      - You can also type: "x y yaw" again to update whenever you want

    Accepts yaw in degrees by default unless you add suffix 'rad'.
    Examples:
      1.0 2.0 90
      1.0 2.0 1.57 rad
    """

    def __init__(self, default_yaw_unit: str = "deg"):
        self.default_yaw_unit = default_yaw_unit.lower().strip()
        if self.default_yaw_unit not in ("deg", "rad"):
            self.default_yaw_unit = "deg"
        self._last: Optional[Pose2D] = None

    def _prompt(self) -> Pose2D:
        if self._last is None:
            msg = "Enter pose: x y yaw (deg default)  e.g. 1.0 2.0 90\n> "
        else:
            msg = (
                "Enter pose: x y yaw (or press Enter to keep last: "
                f"{self._last.x:.2f} {self._last.y:.2f} {math.degrees(self._last.yaw):.1f}deg)\n> "
            )

        try:
            line = input(msg).strip()
        except EOFError:
            raise KeyboardInterrupt

        if line == "":
            if self._last is None:
                print("No previous pose yet. Please enter x y yaw.", file=sys.stderr)
                return self._prompt()
            return self._last

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
            # Unknown suffix -> assume default
            yaw_rad = math.radians(yaw) if self.default_yaw_unit == "deg" else yaw

        return Pose2D(float(x), float(y), _normalize_angle_rad(float(yaw_rad)))

    def get_pose(self) -> Pose2D:
        self._last = self._prompt()
        return self._last


# Backwards-compatible name expected by collect.py / older docs
ClickMapPoseProvider = ManualPoseProvider
