"""
TaskSpec ↔ amr_msgs/msg/Task 변환 (ROS 메시지 의존, rclpy 는 쓰지 않는다).

- deadline: None ↔ builtin_interfaces/Time(0, 0)
- created : header.stamp (0 이면 변환 시각 now)
- yaw     : z 축 회전 쿼터니언 (x=y=0)
"""

from __future__ import annotations

import math
from typing import Callable, Dict, Optional, Tuple

from amr_msgs.msg import Task
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped

from amr_fleet.task_schema import (
    DEFAULT_ITEM_MASSES, Pose2D, TaskSpec, generate_task_id,
)

_MAX_TIME_SEC = 2 ** 31 - 1   # builtin_interfaces/Time.sec 는 int32


def float_to_time(t: Optional[float]) -> Time:
    """
    초(float) → builtin_interfaces/Time. None·음수·NaN → (0, 0).

    int32 sec 범위를 넘는 값(+inf 포함)은 최댓값으로 자른다 — 발행 경로에서 예외가 나지 않게 한다
    (접수 검증이 마감 지평선을 막으므로 정상 입력에서는 일어나지 않는다).
    """
    msg = Time()
    if t is None or math.isnan(t) or t <= 0.0:
        return msg
    if t >= _MAX_TIME_SEC:
        msg.sec, msg.nanosec = _MAX_TIME_SEC, 999_999_999
        return msg
    sec = int(t)
    msg.sec = sec
    msg.nanosec = int(round((t - sec) * 1e9))
    if msg.nanosec >= 1_000_000_000:
        msg.sec, msg.nanosec = sec + 1, msg.nanosec - 1_000_000_000
    return msg


def time_to_float(t: Time) -> Optional[float]:
    """builtin_interfaces/Time → epoch 초. (0, 0) 은 None."""
    if t.sec == 0 and t.nanosec == 0:
        return None
    return t.sec + t.nanosec * 1e-9


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """요(yaw) → z 축 회전 쿼터니언 (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """(x, y, z, w) → yaw [rad]."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_to_msg(pose: Pose2D, stamp: Optional[Time] = None) -> PoseStamped:
    """Pose2D → geometry_msgs/PoseStamped."""
    msg = PoseStamped()
    msg.header.frame_id = pose.frame_id
    if stamp is not None:
        msg.header.stamp = stamp
    msg.pose.position.x = float(pose.x)
    msg.pose.position.y = float(pose.y)
    qx, qy, qz, qw = yaw_to_quaternion(pose.yaw)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


def pose_from_msg(msg: PoseStamped) -> Pose2D:
    """geometry_msgs/PoseStamped → Pose2D."""
    q = msg.pose.orientation
    return Pose2D(x=msg.pose.position.x, y=msg.pose.position.y,
                  yaw=quaternion_to_yaw(q.x, q.y, q.z, q.w),
                  frame_id=msg.header.frame_id or 'map')


def spec_to_msg(spec: TaskSpec, stamp: Optional[Time] = None) -> Task:
    """`TaskSpec` → amr_msgs/Task. stamp 가 없으면 created 를 header.stamp 로 쓴다."""
    msg = Task()
    msg.header.stamp = stamp if stamp is not None else float_to_time(spec.created)
    msg.header.frame_id = spec.pickup.frame_id
    msg.task_id = spec.task_id
    msg.robot_id = spec.robot_id
    msg.priority = int(max(0, min(255, spec.priority)))
    msg.deadline = float_to_time(spec.deadline)
    msg.pickup_pose = pose_to_msg(spec.pickup, msg.header.stamp)
    msg.dropoff_pose = pose_to_msg(spec.dropoff, msg.header.stamp)
    msg.item_type = spec.item_type
    msg.item_mass = float(spec.item_mass)
    msg.status = int(spec.status)
    return msg


def spec_from_msg(msg: Task, now: float, id_factory: Callable[[], str] = generate_task_id,
                  item_masses: Optional[Dict[str, float]] = None) -> TaskSpec:
    """
    amr_msgs/Task → TaskSpec.

    task_id 가 비어 있으면 생성하고, item_mass 가 0 이하이면 payload 표에서 채운다.
    """
    masses = item_masses or DEFAULT_ITEM_MASSES
    created = time_to_float(msg.header.stamp)
    mass = float(msg.item_mass)
    if mass <= 0.0:
        mass = float(masses.get(msg.item_type, 0.0))
    return TaskSpec(
        task_id=msg.task_id or id_factory(),
        pickup=pose_from_msg(msg.pickup_pose),
        dropoff=pose_from_msg(msg.dropoff_pose),
        item_type=msg.item_type,
        item_mass=mass,
        priority=int(msg.priority),
        deadline=time_to_float(msg.deadline),
        robot_id=msg.robot_id,
        created=created if created is not None else float(now),
        status=int(msg.status),
    )
