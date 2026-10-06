"""
판정 오라클 — 모의 밖에서 보는 "정답" 지표 (로봇·서버의 믿음과 무관하게 실제 자세로 센다).

- 구역 동시 점유: 매 단계 실제 차체(방향 사각형, 분리축 정리)가 용량 1 구역에 둘 이상 → 위반 구간.
  죽은 로봇의 차체도 센다. 같은 방향 추종을 허용한 1차선 통로(same_direction)는 따로 센다.
- 중복 실행: 같은 작업에 둘 이상의 로봇이 load(픽업 시작)를 시작.
- 완료·지연: 하역 끝 시각 − 작업 생성 시각.
- 교착: 0.5 s 마다 대기 그래프(정지선에서 선 로봇 → 그 구역 허가를 믿는 로봇 · 차체가 걸친 로봇)의 크기 ≥ 2
  강연결 성분이 confirm_s(1 s) 이어지면 교착. 사라질 때까지의 시간을 잰다.
"""

from __future__ import annotations

import math
from typing import Dict, FrozenSet, List, Optional, Set, Tuple

import numpy as np

from amr_fleet.deadlock import strongly_connected_components
from amr_fleet.fleetsim.geometry import CIRCUMRADIUS, rect_box_overlap


class Oracle:
    """실행 1 건의 판정 기록."""

    def __init__(self, sim, confirm_s: float = 1.0):
        self.sim = sim
        self.world = sim.world
        self.confirm_s = float(confirm_s)
        ids = list(self.world.boxes)
        self.zone_ids = ids
        b = [self.world.boxes[z] for z in ids]
        self._bx = np.array([[q.xmin, q.ymin, q.xmax, q.ymax] for q in b])
        self.shared = {z for z in ids if self.world.same_direction.get(z)}
        self.assigns: Dict[str, List[Tuple[float, str, str]]] = {}
        self.loads: Dict[str, List[Tuple[float, str]]] = {}
        self.completions: Dict[str, List[Tuple[float, str]]] = {}
        self.viol: List[dict] = []
        self._open: Dict[str, dict] = {}
        self.min_dist = math.inf
        self.cycles: List[dict] = []
        self._cyc_open: Dict[FrozenSet[str], dict] = {}
        self.long_waits: Set[Tuple[str, float]] = set()

    # ------------------------------------------------------------------ 사건
    def assigned(self, task_id: str, robot: str, t: float, via: str) -> None:
        self.assigns.setdefault(task_id, []).append((t, robot, via))

    def load_start(self, task_id: str, robot: str, t: float) -> None:
        self.loads.setdefault(task_id, []).append((t, robot))

    def complete(self, task_id: str, robot: str, t: float) -> None:
        self.completions.setdefault(task_id, []).append((t, robot))

    # ------------------------------------------------------------------ 표본
    def sample(self, now: float) -> None:
        """단계 끝 실제 자세로 동시 점유를 센다."""
        robots = self.sim.robots
        xy = np.array([[r.x, r.y] for r in robots])
        bx = self._bx
        dx = np.maximum(np.maximum(bx[None, :, 0] - xy[:, None, 0], 0.0),
                        xy[:, None, 0] - bx[None, :, 2])
        dy = np.maximum(np.maximum(bx[None, :, 1] - xy[:, None, 1], 0.0),
                        xy[:, None, 1] - bx[None, :, 3])
        near = np.hypot(dx, dy) < CIRCUMRADIUS
        occ: Dict[str, List[str]] = {}
        for i, j in zip(*np.nonzero(near)):
            r = robots[int(i)]
            z = self.zone_ids[int(j)]
            if rect_box_overlap(r.x, r.y, r.th, self.world.boxes[z]):
                occ.setdefault(z, []).append(r.id)
        for z in list(self._open):
            if len(occ.get(z, ())) < 2:
                v = self._open.pop(z)
                v['t1'] = now
        for z, rs in occ.items():
            if len(rs) >= 2:
                if z not in self._open:
                    v = {'zone': z, 't0': now, 't1': None, 'robots': sorted(rs),
                         'shared_corridor': z in self.shared}
                    self._open[z] = v
                    self.viol.append(v)
                else:
                    self._open[z]['robots'] = sorted(set(self._open[z]['robots']) | set(rs))
        if len(xy) > 1:
            d = np.hypot(xy[:, None, 0] - xy[None, :, 0], xy[:, None, 1] - xy[None, :, 1])
            d[np.diag_indices(len(xy))] = np.inf
            self.min_dist = min(self.min_dist, float(d.min()))

    def wfg(self, now: float) -> None:
        """대기 그래프 강연결 성분 (0.5 s 마다)."""
        robots = self.sim.robots
        adj: Dict[str, List[str]] = {r.id: [] for r in robots}
        for r in robots:
            if r.wait_zone is None:
                continue
            if r.wait_since is not None and now - r.wait_since > self.sim.cfg.wait_max_s:
                self.long_waits.add((r.id, round(r.wait_since, 3)))
            z = r.wait_zone
            box = self.world.boxes[z]
            for h in robots:
                if h is r:
                    continue
                if h.permitted(z) or rect_box_overlap(h.x, h.y, h.th, box):
                    adj[r.id].append(h.id)
        comps = [frozenset(c) for c in strongly_connected_components(adj) if len(c) >= 2]
        live = set(comps)
        for c in list(self._cyc_open):
            if c not in live:
                rec = self._cyc_open.pop(c)
                rec['t1'] = now
        for c in comps:
            if c not in self._cyc_open:
                rec = {'members': sorted(c), 't0': now, 't1': None}
                self._cyc_open[c] = rec
                self.cycles.append(rec)

    # ------------------------------------------------------------------ 요약
    def summary(self, horizon: float) -> dict:
        for v in self.viol:
            if v['t1'] is None:
                v['t1'] = horizon
        for c in self.cycles:
            if c['t1'] is None:
                c['t1'] = horizon
        hard = [v for v in self.viol if not v['shared_corridor']]
        dl = [c for c in self.cycles if c['t1'] - c['t0'] >= self.confirm_s]
        dup = sorted(t for t, ls in self.loads.items() if len({r for _, r in ls}) >= 2)
        return {
            'cooccupancy_events': len(hard),
            'cooccupancy_s': round(sum(v['t1'] - v['t0'] for v in hard), 3),
            'cooccupancy_detail': [{'zone': v['zone'], 't0': round(v['t0'], 2),
                                    't1': round(v['t1'], 2), 'robots': v['robots']}
                                   for v in hard[:5]],
            'shared_corridor_events': len(self.viol) - len(hard),
            'dup_tasks': len(dup),
            'dup_detail': dup[:5],
            'deadlocks': len(dl),
            'deadlock_s': [round(c['t1'] - c['t0'], 2) for c in dl],
            'deadlock_unresolved': sum(1 for c in dl if c['t1'] >= horizon - 1e-9),
            'long_waits': len(self.long_waits),
            'min_robot_dist_m': (None if not math.isfinite(self.min_dist)
                                 else round(self.min_dist, 3)),
        }


def first(xs: Optional[List[Tuple[float, str]]]) -> Optional[Tuple[float, str]]:
    return min(xs) if xs else None


__all__ = ['Oracle']
