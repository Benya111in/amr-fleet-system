"""
오프라인 비교용 사례 — 저장소의 실제 도크·스폰 좌표와 중앙 기준 할당.

좌표 출처 (scripts/allocation_compare.py 와 같은 값)
- 도크 4곳: src/amr_behavior/config/behavior.yaml docks.*.staging
- 스폰 5곳: src/amr_bringup/config/fleet_spawn.yaml robots

make_case 는 연구 계산 mrta_small.py 의 make_case 와 **같은 순서로 난수를 소비**한다 (같은 시드 → 같은 사례).
작업 절반은 입고(도크 → 임의 지점), 절반은 출고(임의 지점 → 도크). 로봇은 스폰 자리('spawn') 또는
창고 56 × 36 m 에 흩어진 자리('scatter', 운용 중 상태).
"""

from __future__ import annotations

import dataclasses
import random
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from amr_fleet.allocation import RobotInfo, solve_assignment
from amr_fleet.task_schema import Pose2D, TaskSpec

DOCKS: Dict[str, Tuple[float, float]] = {
    'dock_1': (-27.78, 17.0), 'dock_2': (-27.78, 13.0),
    'dock_a': (27.78, 17.0), 'dock_b': (27.78, 13.0),
}
SPAWNS: List[Tuple[str, float, float]] = [
    ('amr_01', 18.0, -16.0), ('amr_02', 20.0, -16.0), ('amr_03', 22.0, -16.0),
    ('amr_04', 24.0, -16.0), ('amr_05', 26.0, -16.0),
]
ROBOT_IDS: List[str] = [r for r, _, _ in SPAWNS]
XLIM: Tuple[float, float] = (-28.0, 28.0)
YLIM: Tuple[float, float] = (-18.0, 18.0)
ROBOT_MODES = ('spawn', 'scatter')


@dataclasses.dataclass
class Case:
    """사례 1건: 작업(후보 순서)과 로봇."""

    tasks: List[TaskSpec]
    robots: List[RobotInfo]

    @property
    def task_ids(self) -> List[str]:
        return [t.task_id for t in self.tasks]

    @property
    def robot_ids(self) -> List[str]:
        return [r.robot_id for r in self.robots]


def _rand_pt(rng: random.Random, free_xy: Optional[Callable[[float, float], bool]]):
    while True:
        p = (rng.uniform(*XLIM), rng.uniform(*YLIM))
        if free_xy is None or free_xy(*p):
            return p


def make_case(n_tasks: int, rng: random.Random, robots_mode: str = 'scatter',
              free_xy: Optional[Callable[[float, float], bool]] = None) -> Case:
    """
    사례를 만든다. free_xy 가 없으면 mrta_small.make_case 와 같은 난수 소비 순서다.

    free_xy(x, y) 를 주면 임의 지점을 빈칸에서만 뽑는다 (격자 비용 사례용, 난수 소비가 달라진다).
    """
    if robots_mode not in ROBOT_MODES:
        raise ValueError(f'robots_mode 는 {ROBOT_MODES} 중 하나: {robots_mode!r}')
    docks = list(DOCKS.values())
    pairs = []
    for _ in range(n_tasks):
        if rng.random() < 0.5:
            pairs.append((rng.choice(docks), _rand_pt(rng, free_xy)))
        else:
            pairs.append((_rand_pt(rng, free_xy), rng.choice(docks)))
    if robots_mode == 'spawn':
        pts = [(x, y) for _, x, y in SPAWNS]
    else:
        pts = [_rand_pt(rng, free_xy) for _ in range(len(SPAWNS))]
    tasks = [TaskSpec(task_id=f't{j}', pickup=Pose2D(*p), dropoff=Pose2D(*d),
                      item_type='small', item_mass=2.0)
             for j, (p, d) in enumerate(pairs)]
    robots = [RobotInfo(robot_id=rid, x=x, y=y) for rid, (x, y) in zip(ROBOT_IDS, pts)]
    return Case(tasks=tasks, robots=robots)


def central_greedy(cost: Sequence[Sequence[float]]) -> Dict[int, int]:
    """
    중앙 순차 탐욕(SGA = SSI 단일 할당): 남은 (로봇, 작업) 쌍 중 최소 비용을 반복해 고른다.

    동점은 (로봇 번호, 작업 번호) 가 작은 쪽. 반환 {작업 번호: 로봇 번호}. 비유한 비용 쌍은 고르지 않는다.
    """
    n_r = len(cost)
    n_t = len(cost[0]) if n_r else 0
    free_r, free_t = set(range(n_r)), set(range(n_t))
    out: Dict[int, int] = {}
    while free_r and free_t:
        best = None
        for i in free_r:
            row = cost[i]
            for j in free_t:
                c = row[j]
                if c != c or c == float('inf'):
                    continue
                key = (c, i, j)
                if best is None or key < best:
                    best = key
        if best is None:
            break
        _, i, j = best
        out[j] = i
        free_r.discard(i)
        free_t.discard(j)
    return out


def optimal_assignment(cost: Sequence[Sequence[float]]) -> Dict[int, int]:
    """Hungarian 최적 (allocation.solve_assignment) → {작업 번호: 로봇 번호}."""
    return {c: r for r, c in solve_assignment(cost)}


def total_cost(cost: Sequence[Sequence[float]], assign: Dict[int, int]) -> float:
    """할당의 비용 합 [s]."""
    return sum(cost[i][j] for j, i in assign.items())


__all__ = ['Case', 'DOCKS', 'ROBOT_IDS', 'ROBOT_MODES', 'SPAWNS', 'XLIM', 'YLIM',
           'central_greedy', 'make_case', 'optimal_assignment', 'total_cost']
