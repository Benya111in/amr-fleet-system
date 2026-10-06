"""
IMU 방위 비교 (rclpy 비의존) — madgwick 재계산 검증과 방위 품질(GT 대비 roll·pitch 오차)에 쓴다.

쿼터니언은 ROS 순서 (x, y, z, w). 오일러는 ZYX (yaw-pitch-roll) — tf2 getRPY 와 같은 규약.
"""

from __future__ import annotations

from typing import Dict, Sequence

import numpy as np


def quat_to_rpy(q: np.ndarray) -> np.ndarray:
    """(N, 4) x y z w → (N, 3) roll pitch yaw [rad]."""
    q = np.asarray(q, dtype=float).reshape(-1, 4)
    x, y, z, w = q.T
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.stack([roll, pitch, yaw], axis=1)


def wrap(a: np.ndarray) -> np.ndarray:
    """각을 (−π, π] 로."""
    return np.arctan2(np.sin(a), np.cos(a))


def rpy_error_deg(q_est: np.ndarray, q_ref: np.ndarray) -> np.ndarray:
    """추정 − 기준 오일러 각 차 (N, 3) [deg], 각마다 감아 둔다."""
    return np.degrees(wrap(quat_to_rpy(q_est) - quat_to_rpy(q_ref)))


def stats(e: Sequence[float]) -> Dict[str, float]:
    """|e| 의 RMSE·평균·p95·최대와 n (빈 입력은 n 0, NaN)."""
    a = np.abs(np.asarray(e, dtype=float))
    if a.size == 0:
        nan = float('nan')
        return {'n': 0, 'rmse': nan, 'mean': nan, 'p95': nan, 'max': nan}
    return {'n': int(a.size), 'rmse': float(np.sqrt(np.mean(a * a))), 'mean': float(a.mean()),
            'p95': float(np.percentile(a, 95)), 'max': float(a.max())}


def quat_diff(q_a: np.ndarray, q_b: np.ndarray) -> np.ndarray:
    """
    두 쿼터니언 열의 성분 최대 차 (부호 맞춤 뒤) — 같은 필터 출력이 비트 단위로 같은지 볼 때 쓴다.

    q 와 −q 는 같은 회전이므로 내적이 음수인 쌍은 한쪽 부호를 뒤집는다.
    """
    a = np.asarray(q_a, dtype=float).reshape(-1, 4)
    b = np.asarray(q_b, dtype=float).reshape(-1, 4)
    if a.shape != b.shape:
        raise ValueError(f'모양이 다르다 {a.shape} vs {b.shape}')
    sign = np.where(np.sum(a * b, axis=1) < 0.0, -1.0, 1.0)
    return np.max(np.abs(a - sign[:, None] * b), axis=1) if len(a) else np.zeros(0)


def quat_angle_deg(q_a: np.ndarray, q_b: np.ndarray) -> np.ndarray:
    """
    두 쿼터니언 열 사이 회전각 [deg] — 상대 회전 r = q_a* ⊗ q_b 의 2·atan2(‖r_xyz‖, |r_w|).

    acos(|⟨q_a, q_b⟩|) 는 같은 회전 근처에서 부동소수 오차가 1e-6° 대로 커진다 — atan2 꼴은 그렇지 않다.
    """
    a = np.asarray(q_a, dtype=float).reshape(-1, 4)
    b = np.asarray(q_b, dtype=float).reshape(-1, 4)
    if a.shape != b.shape:
        raise ValueError(f'모양이 다르다 {a.shape} vs {b.shape}')
    ax, ay, az, aw = -a[:, 0], -a[:, 1], -a[:, 2], a[:, 3]          # 켤레
    bx, by, bz, bw = b.T
    rx = aw * bx + ax * bw + ay * bz - az * by
    ry = aw * by - ax * bz + ay * bw + az * bx
    rz = aw * bz + ax * by - ay * bx + az * bw
    rw = aw * bw - ax * bx - ay * by - az * bz
    return np.degrees(2.0 * np.arctan2(np.sqrt(rx * rx + ry * ry + rz * rz), np.abs(rw)))


def pair_by_stamp(t_a: Sequence[float], t_b: Sequence[float], tol: float = 1e-6):
    """
    두 스탬프 열에서 같은 스탬프(±tol)의 쌍 인덱스 (ia, ib) — 짝 없는 것은 버린다.

    t_b 는 정렬돼 있지 않아도 된다.
    """
    a = np.asarray(t_a, dtype=float)
    b = np.asarray(t_b, dtype=float)
    order = np.argsort(b, kind='stable')
    bs = b[order]
    k = np.clip(np.searchsorted(bs, a), 0, max(len(bs) - 1, 0))
    if len(bs) == 0:
        return np.zeros(0, dtype=int), np.zeros(0, dtype=int)
    k0 = np.clip(k - 1, 0, len(bs) - 1)
    best = np.where(np.abs(bs[k0] - a) < np.abs(bs[k] - a), k0, k)
    ok = np.abs(bs[best] - a) <= tol
    return np.nonzero(ok)[0], order[best[ok]]
