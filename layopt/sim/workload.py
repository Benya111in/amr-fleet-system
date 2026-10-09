"""
작업 열 · 시작 잔량 — 공통 난수 (같은 시드 = 배치와 무관하게 같은 외생 입력).

- 포아송 dock→dock 작업: 도착률 λ = load × base_rate. 부하끼리도 공통 난수가 되게 최고 부하(LOAD_MAX = 1.5)의 열을
  한 번 만들고 도착마다 u ~ U(0,1) 을 뽑아 u < load / LOAD_MAX 인 것만 남긴다 (포아송 솎아내기 — 결과는 정확히
  도착률 load·base_rate 의 포아송이고, 같은 시드에서 0.5 ⊂ 1.0 ⊂ 1.5 로 겹친다). 작업 id 는 최고 부하 열의 번호다.
  출발·도착 도크는 도크 4 곳 중 서로 다른 두 곳 (fleetsim.config.task_stream 과 같은 규칙), 물품 종류와
  우선순위도 같은 규칙 (물품 질량·적재 시간은 config/robot_params.yaml payload 표).
- P5 작업 (실험 0 보정): runs/b24/b4-p5 plan.json 의 5 건 — t_base 60 s 에 동시 게시.
- 시작 잔량: 로봇마다 U(25, 60) % (seed, 로봇) 파생.
"""

from __future__ import annotations

import dataclasses
import random
from typing import Dict, List, Sequence, Tuple

from layopt.sim import ensure_paths

ensure_paths()
from amr_fleet.fleetsim.config import derive, uniform  # noqa: E402

LOAD_MAX = 1.5
PRIORITIES = (50, 100, 150, 200)                   # fleetsim.config.PRIORITIES
ITEMS: Dict[str, Tuple[float, float]] = {          # robot_params.yaml payload: (질량 kg, 적재 시간 s)
    'small': (2.0, 5.0), 'medium': (10.0, 10.0), 'large': (25.0, 15.0)}
SOC_RANGE = (25.0, 60.0)


@dataclasses.dataclass(frozen=True)
class Task:
    """작업 1 건 (도크 id 로 적는다 — 배치가 바뀌어도 같은 작업)."""

    task_id: str
    t: float
    pickup: str
    dropoff: str
    item_type: str
    priority: int

    @property
    def mass(self) -> float:
        return ITEMS[self.item_type][0]

    @property
    def load_time(self) -> float:
        return ITEMS[self.item_type][1]


def poisson_tasks(seed: int, base_rate: float, load: float, horizon_s: float,
                  docks: Sequence[str]) -> List[Task]:
    """포아송 작업 열 (최고 부하 열을 솎아낸 것, 모듈 머리말)."""
    if not 0.0 < load <= LOAD_MAX:
        raise ValueError(f'load 는 (0, {LOAD_MAX}]')
    docks = sorted(docks)
    rng = random.Random(derive(seed, 'e1_tasks'))
    keep_rng = random.Random(derive(seed, 'e1_thin'))
    rate = LOAD_MAX * base_rate
    out: List[Task] = []
    t = 0.0
    k = 0
    while True:
        t += rng.expovariate(rate)
        if t >= horizon_s:
            break
        pick = rng.choice(docks)
        drop = rng.choice([d for d in docks if d != pick])
        item = rng.choice(sorted(ITEMS))
        prio = rng.choice(PRIORITIES)
        u = keep_rng.random()
        k += 1
        if u < load / LOAD_MAX:
            out.append(Task(f'e1_{k:04d}', t, pick, drop, item, prio))
    return out


P5_T_BASE = 60.0
P5_ROUTES = (('dock_1', 'dock_a', 100), ('dock_a', 'dock_1', 99), ('dock_2', 'dock_b', 98),
             ('dock_b', 'dock_2', 97), ('dock_1', 'dock_b', 96))


def p5_tasks() -> List[Task]:
    """P5 작업 5 건 (plan.json: 모두 offset 0, small 2 kg, 우선순위 100 … 96)."""
    return [Task(f'p5.{i:02d}', P5_T_BASE, a, b, 'small', pr)
            for i, (a, b, pr) in enumerate(P5_ROUTES)]


def initial_soc(seed: int, robot: str) -> float:
    """시작 잔량 [%] ~ U(25, 60) (seed, 로봇)."""
    return uniform(SOC_RANGE[0], SOC_RANGE[1], seed, 'e1_soc', robot)


__all__ = ['ITEMS', 'LOAD_MAX', 'P5_ROUTES', 'P5_T_BASE', 'PRIORITIES', 'SOC_RANGE', 'Task',
           'derive', 'initial_soc', 'p5_tasks', 'poisson_tasks', 'uniform']
