"""
체류 기록 — contract.md §4 dwell[].k_hist_s.

체류 지점 d (layout.json dwell_points) 마다, 로봇이 그 자세에 서 있던 시간을 "그때 d 의 이웃 자세(neighbors) 중
어느 하나에서 ±0.5 m 안에 있던 다른 로봇 수 k" 로 나눠 쌓는다.

- "서 있다": 로봇 중심이 d 에서 at_radius 안이고 선속도 < 0.05 m/s (traffic.yaml deadlock.stationary_speed).
  제자리 회전 · 도킹 · 적재 · 충전 단계는 모두 선 것으로 본다 (그 단계에서 차체는 staging 자세에 있다, robot.py).
- at_radius: 기본 0.5 m. 대기열(queue) 지점은 게이트 정지 위치가 경로 · 속도에 따라 흩어지므로 따로 준다
  (sim.md — 묶음의 queue 정의를 본 뒤 정한다).
- k 는 "다른 로봇이 이웃 자세 ±0.5 m 안" 의 수 (한 로봇이 이웃 자세 둘에 걸쳐도 한 번).
같은 단계에 한 로봇은 가장 가까운 체류 지점 하나에만 쌓는다.
"""

from __future__ import annotations

from typing import Dict, List, Sequence

import numpy as np

from layopt.sim.layout import DwellPoint

NEIGHBOR_TOL = 0.5
STATIONARY_V = 0.05


class DwellTracker:
    """체류 지점 × k 시간표."""

    def __init__(self, points: Sequence[DwellPoint], n_robots: int, at_radius: float = 0.5,
                 queue_radius: float = 0.5):
        self.points: List[DwellPoint] = list(points)
        d = len(self.points)
        self.n = n_robots
        self.P = np.array([[p.x, p.y] for p in self.points]) if d else np.zeros((0, 2))
        self.R = np.array([queue_radius if p.kind == 'queue' else at_radius
                           for p in self.points]) if d else np.zeros(0)
        nb, owner = [], []
        for i, p in enumerate(self.points):
            for q in p.neighbors:
                nb.append((q[0], q[1]))
                owner.append(i)
        self.NB = np.array(nb) if nb else np.zeros((0, 2))
        self.owner = np.array(owner, dtype=int)
        self.hist = np.zeros((d, n_robots), dtype=float)       # k = 0 … n−1
        self.total = np.zeros(d)

    def step(self, xy: np.ndarray, v: np.ndarray, dt: float) -> List[tuple]:
        """로봇 위치 xy (n, 2) · 선속도 v (n,) → [(로봇 번호, 체류 지점 id, k)] (선 로봇만)."""
        out: List[tuple] = []
        if not len(self.P):
            return out
        still = v < STATIONARY_V
        if not still.any():
            return out
        dd = np.hypot(xy[:, None, 0] - self.P[None, :, 0], xy[:, None, 1] - self.P[None, :, 1])
        dd[~still] = np.inf
        dd[dd > self.R[None, :]] = np.inf
        best = dd.argmin(axis=1)
        ok = np.isfinite(dd[np.arange(len(xy)), best])
        if not ok.any():
            return out
        if len(self.NB):
            near = np.hypot(xy[:, None, 0] - self.NB[None, :, 0],
                            xy[:, None, 1] - self.NB[None, :, 1]) <= NEIGHBOR_TOL   # (n, M)
        for i in np.nonzero(ok)[0].tolist():
            d = int(best[i])
            k = 0
            if len(self.NB):
                cols = self.owner == d
                if cols.any():
                    hit = near[:, cols].any(axis=1)
                    hit[i] = False
                    k = int(hit.sum())
            self.hist[d, k] += dt
            self.total[d] += dt
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
