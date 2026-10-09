"""
측위 사건 훅 — (체류 지점 id, 함께 선 이웃 수 k) → P(LOST / 분), P(오수렴 | LOST).

기본값은 0 이다 (순수 교통 지표). 실험 1 의 표가 나오면 table 로 꽂는다 (prereg.md §6: λ_LOST(d, k) · P_mis(d, k)).
사건 모형:
- 로봇이 체류 지점 d 에 서 있고 이웃 k 대일 때, 단계마다 LOST 확률 1 − exp(−λ·dt), λ = p_lost_per_min / 60.
- LOST 가 나면 제자리 회전 + 재초기화 reinit_s 동안 멈춘다 (기본 30 s = behavior.yaml relocalization_timeout_ms).
- 그 뒤 P(오수렴 | LOST) 로 오수렴 → FAILED: failed_stop_s(120 s) 더 멈춘다 (상태 ERROR, 움직일 수 없음).
- 멈춘 동안 시간 단계(도킹 · 적재 · 충전)는 미뤄지고, IDLE 로봇은 할당 후보가 아니다.
난수는 (seed, 로봇, 'loc') 파생 열 — 배치끼리 공통 난수가 아니다 (사건이 체류에 따라 달라지므로 불가능).
"""

from __future__ import annotations

import dataclasses
import math
import random
from typing import Callable, Dict, Optional, Tuple

from layopt.sim.workload import derive

RateFn = Callable[[str, int], float]


def _zero(point_id: str, k: int) -> float:
    return 0.0


@dataclasses.dataclass
class LocModel:
    """측위 사건 모형 (기본: 사건 없음)."""

    p_lost_per_min: RateFn = _zero
    p_mis_given_lost: RateFn = _zero
    reinit_s: float = 30.0
    failed_stop_s: float = 120.0

    @property
    def enabled(self) -> bool:
        return self.p_lost_per_min is not _zero

    @classmethod
    def from_table(cls, table: Dict[Tuple[str, int], Tuple[float, float]],
                   default: Tuple[float, float] = (0.0, 0.0), **kw) -> 'LocModel':
        """{(지점 id 또는 'kind:<종류>', k): (p_lost_per_min, p_mis)} 표 → 모형."""
        def look(pid: str, k: int, i: int) -> float:
            v = table.get((pid, k))
            if v is None and ':' in pid:
                v = table.get(('kind:' + pid.split(':', 1)[0], k))
            return (v or default)[i]
        return cls(lambda p, k: look(p, k, 0), lambda p, k: look(p, k, 1), **kw)


class LocEvents:
    """실행 1 건의 측위 사건 (로봇별 난수 열)."""

    def __init__(self, model: LocModel, seed: int, robot_ids):
        self.model = model
        self.rng = {r: random.Random(derive(seed, 'loc', r)) for r in robot_ids}
        self.lost = 0
        self.misconverge = 0
        self.recovery_s = 0.0
        self.log = []

    def step(self, now: float, dt: float, robot, point_id: Optional[str], k: int) -> None:
        if point_id is None or now < robot.frozen_until:
            return
        lam = self.model.p_lost_per_min(point_id, k) / 60.0
        if lam <= 0.0:
            return
        rng = self.rng[robot.id]
        if rng.random() >= 1.0 - math.exp(-lam * dt):
            return
        self.lost += 1
        stop = self.model.reinit_s
        mis = rng.random() < self.model.p_mis_given_lost(point_id, k)
        if mis:
            self.misconverge += 1
            stop += self.model.failed_stop_s
            robot.loc_failed = True
        robot.frozen_until = now + stop
        self.recovery_s += stop
        self.log.append((round(now, 2), robot.id, point_id, k, bool(mis)))


__all__ = ['LocEvents', 'LocModel']
