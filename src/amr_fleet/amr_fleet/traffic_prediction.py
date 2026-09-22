"""
경로 충돌 예측 (명세 4.9 "경로 충돌 예측 및 해소 알고리즘").

각 로봇의 plan 을 현재 자세에서부터 공칭 속도로 따라간다고 보고 시각 표본
(t_k, x_k, y_k, ψ_k) 으로 바꾼다(시공간 표본, dt 간격, horizon 까지). 두 로봇의 표본 중

    |t_a - t_b| <= time_window  이고  |p_a(t_a) - p_b(t_b)| < safety_distance

인 쌍이 있으면 충돌로 예측한다 (같은 자리를 time_window 안에 번갈아 지나가는 것까지 포함).
가장 이른 쌍(max(t_a, t_b) 최소)의 진행 방향 차 |Δψ| 로 유형을 가른다.

    |Δψ| >= head_on_angle    → HEAD_ON    정면 대향. 1차선 통로에서는 교착 후보
    |Δψ| <= following_angle  → FOLLOWING  추종. 차두는 지역 계획기·safety_node 몫
    그 외                     → CROSSING   교차. 교차로 밖이면 우선순위 낮은 쪽이 양보

safety_distance = 2 × 로봇 외접 반경 + 여유 (기본 2 × 0.36 + 0.3 = 1.02 m).
경로가 없거나 정지 명령(hold)·도킹 중인 로봇은 현재 자세에 머무는 표본열(진행 방향 = yaw)로 둔다.
"""

from __future__ import annotations

import dataclasses
import itertools
import math
from typing import List, Optional, Sequence

import numpy as np

from amr_fleet.traffic_geometry import angle_diff, cumulative_length, points_at, project

HEAD_ON = 'HEAD_ON'
CROSSING = 'CROSSING'
FOLLOWING = 'FOLLOWING'
CONFLICT_KINDS = (HEAD_ON, CROSSING, FOLLOWING)


@dataclasses.dataclass
class Trajectory:
    """로봇 1대의 예측 시공간 표본."""

    robot_id: str
    t: np.ndarray          # (K,) [s], 현재 시각 기준
    xy: np.ndarray         # (K, 2) [m]
    heading: np.ndarray    # (K,) [rad]
    moving: bool           # False = 제자리 예측


def predict_trajectory(robot_id: str, x: float, y: float, yaw: float,
                       path: Optional[np.ndarray] = None, speed: float = 1.0,
                       horizon_s: float = 10.0, dt_s: float = 0.25,
                       s0: Optional[float] = None) -> Trajectory:
    """
    경로를 공칭 속도로 따라가는 표본열. path 가 없으면 제자리.

    s0 는 로봇의 경로 위 호 길이 (None = 자세를 경로에 투영). 경로 끝에 닿으면 거기 머문다.
    첫 표본은 실제 자세이고 이후는 경로 위 점이다.
    """
    if horizon_s <= 0.0 or dt_s <= 0.0:
        raise ValueError('horizon_s, dt_s 는 양수여야 한다')
    t = np.arange(0.0, horizon_s + 1e-9, dt_s)
    if path is None or len(path) < 2 or speed <= 0.0:
        return _stationary(robot_id, x, y, yaw, t)
    cum = cumulative_length(path)
    if s0 is None:
        s0, _ = project(path, cum, x, y)
    if cum[-1] - s0 <= 1e-3:
        return _stationary(robot_id, x, y, yaw, t)
    xy, hd = points_at(path, cum, s0 + speed * t)
    xy[0] = (x, y)
    return Trajectory(robot_id, t, xy, hd, True)


def _stationary(robot_id: str, x: float, y: float, yaw: float, t: np.ndarray) -> Trajectory:
    xy = np.tile(np.array([[x, y]], dtype=float), (len(t), 1))
    return Trajectory(robot_id, t, xy, np.full(len(t), float(yaw)), False)


def classify_conflict(heading_a: float, heading_b: float, head_on_deg: float = 135.0,
                      following_deg: float = 45.0) -> str:
    """진행 방향 차로 HEAD_ON / FOLLOWING / CROSSING."""
    d = math.degrees(angle_diff(heading_a, heading_b))
    if d >= head_on_deg:
        return HEAD_ON
    if d <= following_deg:
        return FOLLOWING
    return CROSSING


@dataclasses.dataclass(frozen=True)
class Conflict:
    """예측 충돌 1건 (가장 이른 시공간 겹침)."""

    robot_a: str
    robot_b: str
    kind: str
    t_a: float           # a 가 충돌 지점에 있는 예측 시각 [s]
    t_b: float
    x: float             # 충돌 지점 (두 표본의 중점)
    y: float
    distance: float      # 두 표본 사이 거리 [m]

    @property
    def t_first(self) -> float:
        """둘 중 먼저 닿는 시각."""
        return min(self.t_a, self.t_b)

    def time_of(self, robot_id: str) -> float:
        """해당 로봇이 충돌 지점에 닿는 예측 시각."""
        return self.t_a if robot_id == self.robot_a else self.t_b

    def other(self, robot_id: str) -> str:
        """상대 로봇 id."""
        return self.robot_b if robot_id == self.robot_a else self.robot_a


def find_conflict(a: Trajectory, b: Trajectory, safety_distance: float,
                  time_window: float, head_on_deg: float = 135.0,
                  following_deg: float = 45.0) -> Optional[Conflict]:
    """두 궤적의 가장 이른 시공간 겹침. 둘 다 정지면 예측 대상이 아니다(None)."""
    if not (a.moving or b.moving):
        return None
    diff = a.xy[:, None, :] - b.xy[None, :, :]
    dist = np.sqrt(np.sum(diff * diff, axis=2))
    dt = np.abs(a.t[:, None] - b.t[None, :])
    hit = (dist < safety_distance) & (dt <= time_window + 1e-9)
    if not hit.any():
        return None
    ii, jj = np.nonzero(hit)
    t_late = np.maximum(a.t[ii], b.t[jj])
    k = int(np.lexsort((dist[ii, jj], t_late))[0])
    i, j = int(ii[k]), int(jj[k])
    kind = classify_conflict(a.heading[i], b.heading[j], head_on_deg, following_deg)
    mid = 0.5 * (a.xy[i] + b.xy[j])
    return Conflict(a.robot_id, b.robot_id, kind, float(a.t[i]), float(b.t[j]),
                    float(mid[0]), float(mid[1]), float(dist[i, j]))


def predict_conflicts(trajectories: Sequence[Trajectory], safety_distance: float,
                      time_window: float, head_on_deg: float = 135.0,
                      following_deg: float = 45.0) -> List[Conflict]:
    """모든 쌍의 예측 충돌 (robot_id 순 쌍, 이른 순 정렬)."""
    out: List[Conflict] = []
    ordered = sorted(trajectories, key=lambda tr: tr.robot_id)
    for a, b in itertools.combinations(ordered, 2):
        c = find_conflict(a, b, safety_distance, time_window, head_on_deg, following_deg)
        if c is not None:
            out.append(c)
    out.sort(key=lambda c: (c.t_first, c.robot_a, c.robot_b))
    return out
