# src/app/pose_ros_tf.py
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class Pose2D:
    x_m: float
    y_m: float
    theta_rad: float


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """
    Convert quaternion -> yaw (rotation around Z axis).
    ROS uses ENU; for 2D navigation yaw is what you want.
    """
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class RosTfPoseProvider:
    """
    ROS1 TF pose provider using tf.TransformListener.
    Looks up (map_frame -> base_frame) transform and returns (x,y,yaw).
    """

    def __init__(
        self,
        map_frame: str = "map",
        base_frame: str = "base_link",
        timeout_s: float = 0.25,
        allow_fallback_odom: bool = True,
        odom_frame: str = "odom",
    ):
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.odom_frame = odom_frame
        self.timeout_s = timeout_s
        self.allow_fallback_odom = allow_fallback_odom

        self._started = False
        self._tf = None  # tf.TransformListener
        self._rospy = None

    def start(self) -> None:
        if self._started:
            return

        import rospy
        import tf

        self._rospy = rospy

        # Important: don't reinitialize if running in a ROS node already
        if not rospy.core.is_initialized():
            rospy.init_node("wifi_heatmap_collector", anonymous=True, disable_signals=True)

        self._tf = tf.TransformListener()
        # Give TF buffer time to fill
        time.sleep(0.25)

        self._started = True

    def get_pose(self) -> Pose2D:
        if not self._started:
            raise RuntimeError("RosTfPoseProvider not started. Call start() first.")

        assert self._tf is not None
        assert self._rospy is not None

        frames_to_try = [(self.map_frame, self.base_frame)]
        if self.allow_fallback_odom:
            frames_to_try.append((self.odom_frame, self.base_frame))

        last_err = None
        for parent, child in frames_to_try:
            try:
                # rospy.Time(0) -> latest available transform
                (trans, rot) = self._tf.lookupTransform(parent, child, self._rospy.Time(0))
                x, y = float(trans[0]), float(trans[1])
                yaw = _yaw_from_quaternion(float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3]))
                return Pose2D(x_m=x, y_m=y, theta_rad=yaw)
            except Exception as e:
                last_err = e

        raise RuntimeError(f"TF lookup failed for {frames_to_try}: {last_err}")

    def ok(self) -> bool:
        # If ROS is shutting down, stop loop politely
        if self._rospy is None:
            return True
        return not self._rospy.is_shutdown()
