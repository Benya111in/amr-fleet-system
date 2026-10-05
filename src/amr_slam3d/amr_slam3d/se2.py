"""평면 자세(x, y, yaw) 계산 — 합성·역·각 정규화·쿼터니언 변환 (rclpy 비의존)."""

from __future__ import annotations

import math
from typing import Iterable, Tuple

Pose2 = Tuple[float, float, float]


def wrap(a: float) -> float:
    """각을 [-π, π) 로 접는다."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def compose(a: Pose2, b: Pose2) -> Pose2:
    """합성 a ⊕ b — a 프레임에서 본 b 를 a 의 부모 프레임으로 옮긴다."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a: Pose2) -> Pose2:
    """역변환 a⁻¹ — a 와 합성하면 항등이다."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap(-a[2]))


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    """쿼터니언 → yaw (ZYX 오일러의 z)."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Yaw 만 있는 쿼터니언 (qx, qy, qz, qw)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def parse_pose(text: str) -> Pose2:
    """'x,y,yaw' 문자열 → 자세 (런치·CLI 인자)."""
    parts = [p for p in text.replace(' ', '').split(',') if p]
    if len(parts) != 3:
        raise ValueError(f"자세는 'x,y,yaw' 세 값이어야 한다: {text!r}")
    x, y, yaw = (float(p) for p in parts)
    return (x, y, yaw)


def path_length(points: Iterable[Tuple[float, float]]) -> float:
    """꺾은선 길이 [m]."""
    total, prev = 0.0, None
    for p in points:
        if prev is not None:
            total += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
    return total
