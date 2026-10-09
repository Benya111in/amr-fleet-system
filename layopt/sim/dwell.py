"""
체류 기록 — contract.md §4 dwell[].k_hist_s.

단계마다 로봇을 체류 지점(layout.json dwell_points) 하나에 붙이거나(귀속) 붙이지 않고, 붙은 시간을 그 지점의
이웃 자세(neighbors)에 서 있던 다른 로봇 수 k 로 나눠 쌓는다.

귀속 (우선순위 순)
 1. 도킹 단계(인식 · 도킹 · 적재/하역 · 이탈 · 충전)인 로봇 → 그 도크 · 충전기 지점 (forced). 그 동안 차체는
    staging 과 도킹 자세 사이에 있다 (robot.py).
 2. 서 있고(선속도 < 0.05 m/s) 교통 hold 가 구역 Z 토큰 대기("zone:Z…")인 로봇 → Z 의 대기열(queue:Z:*) 지점 중
    가장 가까운 것 (queue_radius 안). 생성기의 대기열 자세는 구역 경계에서 요청 거리(2.0 m) 앞이고 실제 정지는
    hold 지연 · 제동 · 비스듬한 접근 때문에 0.5–3 m 어긋난다 (sim.md §체류).
 3. 서 있는 로봇 → at_radius(0.5 m) 안의 가장 가까운 지점.
k (귀속된 로봇 i, 지점 d) = 다른 로봇 j 중 "d 의 이웃 자세에 서 있는" 수:
   (a) j 가 이웃 자세 하나에서 ±0.5 m 안, 또는
   (b) j 가 귀속된 지점의 자세가 d 의 이웃 자세 하나에서 ±0.5 m 안 (위 2 번과 같은 이유로 실제 위치 대신 귀속을 본다),
   (c) d 가 대기열 · 도크이면, 서서 (a)(b) 로봇이나 i 에 막혀 줄 선 로봇 (줄 둘째 이웃 자세 — 몸체 막힘 사슬).
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence

import numpy as np

from layopt.sim.layout import DwellPoint

NEIGHBOR_TOL = 0.5
STATIONARY_V = 0.05


class DwellTracker:
    """체류 지점 × k 시간표."""

    def __init__(self, points: Sequence[DwellPoint], n_robots: int, at_radius: float = 0.5,
                 queue_radius: float = 4.0):
        self.points: List[DwellPoint] = list(points)
        d = len(self.points)
        self.n = n_robots
        self.at_radius = at_radius
        self.queue_radius = queue_radius
        self.index = {p.id: i for i, p in enumerate(self.points)}
        self.P = np.array([[p.x, p.y] for p in self.points]) if d else np.zeros((0, 2))
        self.queues_of: Dict[str, List[int]] = {}
        for i, p in enumerate(self.points):
            if p.kind == 'queue':
                zone = p.id.split(':')[1]
                self.queues_of.setdefault(zone, []).append(i)
        nb, owner = [], []
        for i, p in enumerate(self.points):
            for q in p.neighbors:
                nb.append((q[0], q[1]))
                owner.append(i)
        self.NB = np.array(nb) if nb else np.zeros((0, 2))
        self.owner = np.array(owner, dtype=int)
        # 지점 d 의 이웃 자세 ±0.5 m 안에 자세가 있는 지점들 (k 의 (b))
        self.nb_points: List[set] = [set() for _ in range(d)]
        if len(self.NB) and d:
            dd = np.hypot(self.NB[:, None, 0] - self.P[None, :, 0],
                          self.NB[:, None, 1] - self.P[None, :, 1])
            for m, e in zip(*np.nonzero(dd <= NEIGHBOR_TOL)):
                self.nb_points[int(self.owner[m])].add(int(e))
        self.chain = [p.kind in ('queue', 'dock') for p in self.points]
        self.hist = np.zeros((d, n_robots), dtype=float)       # k = 0 … n−1
        self.unattributed_hold_s = 0.0

    def attribute(self, xy: np.ndarray, v: np.ndarray, forced: Sequence[Optional[str]],
                  hold_zones: Sequence[Optional[List[str]]]) -> List[int]:
        """로봇마다 귀속 지점 번호 (-1 = 없음)."""
        att = [-1] * len(xy)
        if not len(self.P):
            return att
        for i in range(len(xy)):
            if forced[i] is not None:
                att[i] = self.index.get(forced[i], -1)
                continue
            if v[i] >= STATIONARY_V:
                continue
            x, y = xy[i]
            zs = hold_zones[i]
            if zs:
                best, bd = -1, self.queue_radius
                for z in zs:                                   # 사슬의 첫 구역부터
                    for q in self.queues_of.get(z, ()):
                        d = math.hypot(self.P[q, 0] - x, self.P[q, 1] - y)
                        if d <= bd:
                            best, bd = q, d
                    if best >= 0:
                        break
                if best >= 0:
                    att[i] = best
                    continue
            d = np.hypot(self.P[:, 0] - x, self.P[:, 1] - y)
            j = int(d.argmin())
            if d[j] <= self.at_radius:
                att[i] = j
        return att

    def step(self, xy: np.ndarray, v: np.ndarray, dt: float,
             forced: Optional[Sequence[Optional[str]]] = None,
             hold_zones: Optional[Sequence[Optional[List[str]]]] = None,
             blocked_by: Optional[Sequence[int]] = None) -> List[tuple]:
        """한 단계 → [(로봇 번호, 체류 지점 id, k)] (귀속된 로봇만)."""
        n = len(xy)
        forced = forced if forced is not None else [None] * n
        hold_zones = hold_zones if hold_zones is not None else [None] * n
        blocked_by = blocked_by if blocked_by is not None else [-1] * n
        att = self.attribute(xy, v, forced, hold_zones)
        for i in range(n):
            if hold_zones[i] and att[i] < 0 and v[i] < STATIONARY_V:
                self.unattributed_hold_s += dt
        out: List[tuple] = []
        if not any(a >= 0 for a in att):
            return out
        near = None
        if len(self.NB):
            near = np.hypot(xy[:, None, 0] - self.NB[None, :, 0],
                            xy[:, None, 1] - self.NB[None, :, 1]) <= NEIGHBOR_TOL    # (n, M)
        for i in range(n):
            d = att[i]
            if d < 0:
                continue
            s = set()
            if near is not None:
                cols = self.owner == d
                if cols.any():
                    s |= {int(j) for j in np.nonzero(near[:, cols].any(axis=1))[0]}
            nbp = self.nb_points[d]
            if nbp:
                s |= {j for j in range(n) if att[j] in nbp}
            s.discard(i)
            if self.chain[d]:
                grow = True
                while grow:
                    grow = False
                    for j in range(n):
                        if j != i and j not in s and v[j] < STATIONARY_V and \
                                (blocked_by[j] == i or blocked_by[j] in s):
                            s.add(j)
                            grow = True
            k = min(len(s), self.n - 1)
            self.hist[d, k] += dt
            out.append((i, self.points[d].id, k))
        return out

    def rows(self, ndigits: int = 2) -> List[Dict]:
        """계약 §4 dwell 목록 (모든 체류 지점, k = 0 … n−1)."""
        out = []
        for i, p in enumerate(self.points):
            out.append({'id': p.id, 'k_hist_s': {str(k): round(float(self.hist[i, k]), ndigits)
                                                 for k in range(self.n)}})
        return out


__all__ = ['DwellTracker', 'NEIGHBOR_TOL', 'STATIONARY_V']
