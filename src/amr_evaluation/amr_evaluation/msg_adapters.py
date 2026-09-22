"""
ROS 메시지 → 순수 자료형 변환 (덕 타이핑, rclpy/메시지 패키지 import 없음).

노드 콜백이 이 함수들만 거치면 핵심 계산은 ROS 없이 단위 테스트할 수 있다.
"""

import math
from typing import Any, Optional, Tuple

from amr_evaluation.metrics import PoseSample, stamp_to_sec, yaw_from_quaternion
import numpy as np


def stamp_of(header: Any, fallback: Optional[float] = None) -> float:
    """header.stamp → float [s]. 스탬프가 0 이면 fallback (없으면 0.0)."""
    t = stamp_to_sec(header.stamp.sec, header.stamp.nanosec)
    if t == 0.0 and fallback is not None:
        return fallback
    return t


def odom_to_sample(msg: Any, fallback_time: Optional[float] = None) -> PoseSample:
    """nav_msgs/Odometry → PoseSample (v = 평면 속도 크기, w = angular.z)."""
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    lin = msg.twist.twist.linear
    return PoseSample(
        t=stamp_of(msg.header, fallback_time),
        x=float(p.x), y=float(p.y),
        yaw=yaw_from_quaternion(q.x, q.y, q.z, q.w),
        v=math.hypot(float(lin.x), float(lin.y)),
        w=float(msg.twist.twist.angular.z),
    )


def pose_stamped_to_sample(msg: Any, fallback_time: Optional[float] = None) -> PoseSample:
    """geometry_msgs/PoseStamped → PoseSample (속도 0)."""
    p = msg.pose.position
    q = msg.pose.orientation
    return PoseSample(
        t=stamp_of(msg.header, fallback_time),
        x=float(p.x), y=float(p.y),
        yaw=yaw_from_quaternion(q.x, q.y, q.z, q.w),
    )


def twist_to_velocity(msg: Any) -> Tuple[float, float]:
    """geometry_msgs/Twist → (평면 속도 크기 [m/s], yaw 속도 [rad/s])."""
    return math.hypot(float(msg.linear.x), float(msg.linear.y)), float(msg.angular.z)


def path_to_points(msg: Any) -> np.ndarray:
    """nav_msgs/Path → (N, 2) 정점 배열 (빈 경로면 (0, 2))."""
    pts = [(float(ps.pose.position.x), float(ps.pose.position.y)) for ps in msg.poses]
    return np.asarray(pts, dtype=float).reshape(-1, 2)
