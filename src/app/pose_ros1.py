import threading
from dataclasses import dataclass

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion


@dataclass
class Pose2D:
    x_m: float = 0.0
    y_m: float = 0.0
    theta_rad: float = 0.0


class Ros1AmclPoseProvider:
    """
    Reads /amcl_pose (geometry_msgs/PoseWithCovarianceStamped) and keeps latest x,y,yaw.
    """
    def __init__(self, topic: str = "/amcl_pose"):
        self.topic = topic
        self._lock = threading.Lock()
        self._pose = Pose2D()
        self._has_pose = False

    def start(self):
        if not rospy.core.is_initialized():
            rospy.init_node("lexx500_wifi_heatmap_collector", anonymous=True, disable_signals=True)

        rospy.Subscriber(self.topic, PoseWithCovarianceStamped, self._cb, queue_size=1)

    def _cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        with self._lock:
            self._pose = Pose2D(float(p.x), float(p.y), float(yaw))
            self._has_pose = True

    def get_pose(self) -> Pose2D:
        with self._lock:
            return self._pose

    def has_pose(self) -> bool:
        with self._lock:
            return self._has_pose

    def should_stop(self) -> bool:
        return False
