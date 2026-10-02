"""
3D 강체 변환 도우미 (numpy, ROS 비의존).

쿼터니언은 ROS 순서 (x, y, z, w). 회전 행렬은 "부모 ← 자식" (p_parent = R p_child + t).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


def quat_to_matrix(q: Sequence[float]) -> np.ndarray:
    """쿼터니언 (x, y, z, w) → 3×3 회전 행렬 (정규화 포함)."""
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(r: np.ndarray) -> np.ndarray:
    """3×3 회전 행렬 → 쿼터니언 (x, y, z, w), w >= 0 (Shepperd 방법)."""
    m = np.asarray(r, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    out = np.array(q, dtype=float)
    out /= np.linalg.norm(out)
    return -out if out[3] < 0.0 else out


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """고정축 roll-pitch-yaw (URDF 규약, R = Rz(yaw) Ry(pitch) Rx(roll))."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def yaw_of(r: np.ndarray) -> float:
    """회전 행렬의 yaw (ZYX 오일러의 z)."""
    return math.atan2(r[1, 0], r[0, 0])


@dataclass
class Transform:
    """강체 변환 p_parent = rotation · p_child + translation."""

    rotation: np.ndarray
    translation: np.ndarray

    @staticmethod
    def identity() -> 'Transform':
        return Transform(np.eye(3), np.zeros(3))

    @staticmethod
    def from_quat(translation: Sequence[float], quat: Sequence[float]) -> 'Transform':
        return Transform(quat_to_matrix(quat), np.asarray(translation, dtype=float).reshape(3))

    def apply(self, p: Sequence[float]) -> np.ndarray:
        return self.rotation @ np.asarray(p, dtype=float).reshape(3) + self.translation

    def compose(self, other: 'Transform') -> 'Transform':
        """Compose: self ∘ other — other 의 자식 좌표 → self 의 부모 좌표."""
        return Transform(self.rotation @ other.rotation,
                         self.rotation @ other.translation + self.translation)

    def inverse(self) -> 'Transform':
        rt = self.rotation.T
        return Transform(rt, -rt @ self.translation)

    def rotate_covariance(self, cov: np.ndarray) -> np.ndarray:
        """위치 공분산을 부모 프레임으로 회전 (Σ' = R Σ Rᵀ)."""
        return self.rotation @ np.asarray(cov, dtype=float) @ self.rotation.T

    def quaternion(self) -> np.ndarray:
        return matrix_to_quat(self.rotation)


def transform_from_msg(tf) -> Transform:
    """geometry_msgs/Transform(또는 TransformStamped.transform) → Transform."""
    t = tf.translation
    q = tf.rotation
    return Transform.from_quat((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))


def prefixed_frame(prefix: str, frame: str) -> str:
    """
    프레임 접두사 적용 ("amr_01/" + "base_link").

    비었거나 이미 '/' 를 포함한 이름, 전역 map 은 그대로 둔다 (multi_robot.md §2).
    """
    if not prefix or not frame or '/' in frame or frame == 'map':
        return frame
    return prefix + frame


def threaded_tf_listener(buffer):
    """
    전용 스레드(내부 노드)에서 /tf, /tf_static 을 받는 TransformListener.

    Humble 기본(spin_thread=False, 호출 노드 공유)은 노드의 콜백이 도는 동안 /tf 를 처리하지 못해
    lookup_transform 의 timeout 대기가 무의미하다. 영상 스탬프가 최신 TF 보다 조금만 앞서도 조회가 실패한다
    (Gazebo 실측: 깊이 스탬프 − 최신 odom TF 1.1 s, map 조회 전부 실패 — perception.md §8).
    tf2_ros 는 노드 생성 시점에만 import 한다 (이 모듈은 ROS 없이도 import 된다).
    """
    import tf2_ros
    return tf2_ros.TransformListener(buffer, None, spin_thread=True)


def shutdown_tf_listener(listener) -> None:
    """threaded_tf_listener 의 스레드·내부 노드 정리 (destroy_node 에서 호출)."""
    if listener is None:
        return
    executor = getattr(listener, 'executor', None)
    if executor is not None:
        executor.shutdown()
        listener.dedicated_listener_thread.join(timeout=2.0)
    listener.node.destroy_node()
