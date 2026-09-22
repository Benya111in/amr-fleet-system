"""
중앙 집중식 작업 할당 전략 (명세 4.9 "최소 거리 기반, 부하 균형, 또는 Hungarian").

전략 인터페이스
    strategy.assign(tasks, robots) -> {task_id: robot_id}
- tasks : 스케줄러가 정한 순서의 대기 작업 (앞이 먼저). robot_id 가 비어 있는 것만 온다.
- robots: 이번 라운드에 받을 수 있는 로봇 (available=True). 한 라운드에 로봇당 최대 1건.
- 반환 : 할당된 쌍만. 할당하지 않은 작업은 다음 라운드로 넘어간다.

새 전략은 AllocationStrategy 를 상속하고 @register_strategy('이름') 을 붙이면
fleet_manager 의 allocation_strategy 파라미터로 고를 수 있다 (알고리즘 연구 트랙 확장점).

이동 시간 추정은 travel_time() = 거리 / nominal_speed 하나로 모아 두었다. 사다리꼴 프로파일
등 정밀한 ETA 로 바꾸려면 이 함수만 교체한다.

최적성 지표(명세 "총 이동 거리, 작업 완료 시간")는 allocate() 가 매 라운드 dict 로 돌려준다.

라운드 입력은 select_batch() 로 고른다: 고정 지정(robot_id) 작업은 그 로봇이 이번 라운드에 비어 있을
때만 넣고, 남은 자리는 스케줄러 순서의 미지정 작업으로 채운다. 바쁜 로봇에 고정된 작업이 상위를
차지해도 빈 로봇이 놀지 않는다 (head-of-line blocking 방지).
"""

from __future__ import annotations

import abc
import dataclasses
import itertools
import math
import time
from typing import Dict, List, Sequence, Tuple, Type

from amr_fleet.task_schema import TaskSpec

try:
    # 노드 시작 시 미리 import 한다 (첫 할당 콜백 안에서 import 하면 부하 시 ~1 s 걸렸다)
    from scipy.optimize import linear_sum_assignment as _scipy_lsa
except ImportError:  # 순수 파이썬 Hungarian 으로 대체
    _scipy_lsa = None


@dataclasses.dataclass
class RobotInfo:
    """할당 계산에 필요한 로봇 요약."""

    robot_id: str
    x: float = 0.0
    y: float = 0.0
    load: int = 0            # 누적 할당 작업 수(진행 중 포함) — load_balance 의 부하 지표
    available: bool = True   # IDLE 이고 대기 작업이 없으면 True


@dataclasses.dataclass
class AllocationResult:
    """한 라운드의 할당 결과와 최적성 지표."""

    assignments: Dict[str, str]
    metrics: Dict[str, float]


def euclid(ax: float, ay: float, bx: float, by: float) -> float:
    """평면 유클리드 거리 [m]."""
    return math.hypot(bx - ax, by - ay)


def approach_distance(robot: RobotInfo, task: TaskSpec) -> float:
    """로봇 현재 위치 → 픽업 거리 [m]."""
    return euclid(robot.x, robot.y, task.pickup.x, task.pickup.y)


def delivery_distance(task: TaskSpec) -> float:
    """픽업 → 하역 거리 [m]."""
    return euclid(task.pickup.x, task.pickup.y, task.dropoff.x, task.dropoff.y)


def travel_time(distance_m: float, nominal_speed: float) -> float:
    """이동 시간 추정 [s] = 거리 / 공칭 속도. 속도가 0 이하이면 ValueError."""
    if nominal_speed <= 0.0:
        raise ValueError('nominal_speed 는 양수여야 한다')
    return distance_m / nominal_speed


class AllocationStrategy(abc.ABC):
    """할당 전략 기반 클래스."""

    name: str = 'base'

    def __init__(self, nominal_speed: float = 1.0):
        if nominal_speed <= 0.0:
            raise ValueError('nominal_speed 는 양수여야 한다')
        self.nominal_speed = float(nominal_speed)

    @abc.abstractmethod
    def assign(self, tasks: Sequence[TaskSpec], robots: Sequence[RobotInfo]) -> Dict[str, str]:
        """작업 → 로봇 매핑을 돌려준다 (한 라운드, 로봇당 최대 1건)."""


_REGISTRY: Dict[str, Type[AllocationStrategy]] = {}


def register_strategy(name: str):
    """전략 클래스를 이름으로 등록하는 데코레이터."""
    def _wrap(cls: Type[AllocationStrategy]) -> Type[AllocationStrategy]:
        cls.name = name
        _REGISTRY[name] = cls
        return cls
    return _wrap


def available_strategies() -> List[str]:
    """등록된 전략 이름 (정렬)."""
    return sorted(_REGISTRY)


def create_strategy(name: str, **kwargs) -> AllocationStrategy:
    """이름으로 전략 인스턴스를 만든다. 모르는 이름이면 KeyError."""
    try:
        cls = _REGISTRY[name]
    except KeyError:
        raise KeyError(f'알 수 없는 할당 전략 {name!r}. 사용 가능: {available_strategies()}') from None
    return cls(**kwargs)


@register_strategy('nearest')
class NearestStrategy(AllocationStrategy):
    """작업 순서대로, 픽업에 가장 가까운 로봇을 준다 (탐욕, O(n·m))."""

    def assign(self, tasks: Sequence[TaskSpec], robots: Sequence[RobotInfo]) -> Dict[str, str]:
        pool = list(robots)
        out: Dict[str, str] = {}
        for task in tasks:
            if not pool:
                break
            best = min(pool, key=lambda r: (approach_distance(r, task), r.robot_id))
            out[task.task_id] = best.robot_id
            pool.remove(best)
        return out


@register_strategy('load_balance')
class LoadBalanceStrategy(AllocationStrategy):
    """누적 할당 수(load)가 가장 적은 로봇 우선, 동률이면 가장 가까운 로봇."""

    def assign(self, tasks: Sequence[TaskSpec], robots: Sequence[RobotInfo]) -> Dict[str, str]:
        pool = [dataclasses.replace(r) for r in robots]
        out: Dict[str, str] = {}
        for task in tasks:
            if not pool:
                break
            best = min(pool, key=lambda r: (r.load, approach_distance(r, task), r.robot_id))
            out[task.task_id] = best.robot_id
            pool.remove(best)
        return out


@register_strategy('hungarian')
class HungarianStrategy(AllocationStrategy):
    """
    배치 최적 할당: 비용 = 로봇→픽업 이동 시간, 총합 최소 (Kuhn–Munkres).

    scipy.optimize.linear_sum_assignment 를 쓰고, 없으면 순수 파이썬 구현으로 대체한다.
    직사각 행렬(작업 수 ≠ 로봇 수)은 min(n, m) 쌍을 할당한다.
    """

    def __init__(self, nominal_speed: float = 1.0, use_scipy: bool = True):
        super().__init__(nominal_speed)
        self.use_scipy = use_scipy

    def cost_matrix(self, tasks: Sequence[TaskSpec],
                    robots: Sequence[RobotInfo]) -> List[List[float]]:
        """이동 시간 행렬 [s] (행 = robots, 열 = tasks)."""
        return [[travel_time(approach_distance(r, t), self.nominal_speed) for t in tasks]
                for r in robots]

    def assign(self, tasks: Sequence[TaskSpec], robots: Sequence[RobotInfo]) -> Dict[str, str]:
        if not tasks or not robots:
            return {}
        cost = self.cost_matrix(tasks, robots)
        pairs = solve_assignment(cost, use_scipy=self.use_scipy)
        return {tasks[c].task_id: robots[r].robot_id for r, c in pairs}


def solve_assignment(cost: Sequence[Sequence[float]],
                     use_scipy: bool = True) -> List[Tuple[int, int]]:
    """
    선형 할당 문제: 비용 행렬(행 × 열)에서 총합 최소인 (행, 열) 쌍 min(n, m) 개.

    use_scipy=False 이거나 scipy 를 못 쓰면 순수 파이썬 Hungarian 을 쓴다.
    """
    if use_scipy and _scipy_lsa is not None:
        rows, cols = _scipy_lsa(cost)
        return sorted((int(r), int(c)) for r, c in zip(rows, cols))
    return hungarian(cost)


def hungarian(cost: Sequence[Sequence[float]]) -> List[Tuple[int, int]]:
    """
    순수 파이썬 Kuhn–Munkres (포텐셜 방식, O(n²·m)). 행 수 ≤ 열 수를 요구하며 반대면 전치한다.

    반환은 (행, 열) 쌍을 행 오름차순으로.
    """
    n = len(cost)
    if n == 0 or len(cost[0]) == 0:
        return []
    m = len(cost[0])
    if n > m:
        transposed = [[cost[r][c] for r in range(n)] for c in range(m)]
        return sorted((r, c) for c, r in hungarian(transposed))

    inf = float('inf')
    u = [0.0] * (n + 1)
    v = [0.0] * (m + 1)
    p = [0] * (m + 1)       # p[j] = 열 j 에 배정된 행 (1-based, 0 = 없음)
    way = [0] * (m + 1)
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = [inf] * (m + 1)
        used = [False] * (m + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = inf
            j1 = 0
            for j in range(1, m + 1):
                if used[j]:
                    continue
                cur = cost[i0 - 1][j - 1] - u[i0] - v[j]
                if cur < minv[j]:
                    minv[j] = cur
                    way[j] = j0
                if minv[j] < delta:
                    delta = minv[j]
                    j1 = j
            for j in range(m + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    return sorted((p[j] - 1, j - 1) for j in range(1, m + 1) if p[j] != 0)


def brute_force_assignment(cost: Sequence[Sequence[float]]) -> Tuple[float, List[Tuple[int, int]]]:
    """모든 순열을 뒤지는 오라클 (테스트용, n·m ≤ 7×7). (최소 비용, 쌍 목록)."""
    n, m = len(cost), len(cost[0]) if cost else 0
    best_cost, best = float('inf'), []
    if n <= m:
        for cols in itertools.permutations(range(m), n):
            total = sum(cost[r][c] for r, c in enumerate(cols))
            if total < best_cost:
                best_cost, best = total, list(enumerate(cols))
    else:
        for rows in itertools.permutations(range(n), m):
            total = sum(cost[r][c] for c, r in enumerate(rows))
            if total < best_cost:
                best_cost, best = total, sorted((r, c) for c, r in enumerate(rows))
    return best_cost, best


def select_batch(tasks: Sequence[TaskSpec], robots: Sequence[RobotInfo],
                 k: int = 0) -> List[TaskSpec]:
    """
    이번 라운드에 실제로 배정할 수 있는 작업만 스케줄러 순서대로 최대 k 개 고른다.

    - 고정 작업: 그 로봇이 available 이고 이 배치에서 아직 다른 고정 작업이 차지하지 않았을 때만.
    - 미지정 작업: 순서대로 남은 자리를 채운다.
    k <= 0 이면 available 로봇 수. 로봇이 없으면 빈 목록.
    """
    free = {r.robot_id for r in robots if r.available}
    limit = k if k > 0 else len(free)
    if not free or limit <= 0:
        return []
    batch: List[TaskSpec] = []
    pinned_taken = set()
    for task in tasks:
        if len(batch) >= limit:
            break
        if task.robot_id:
            if task.robot_id in free and task.robot_id not in pinned_taken:
                pinned_taken.add(task.robot_id)
                batch.append(task)
        else:
            batch.append(task)
    return batch


def allocate(strategy: AllocationStrategy, tasks: Sequence[TaskSpec],
             robots: Sequence[RobotInfo]) -> AllocationResult:
    """
    한 라운드 할당: 고정 지정(robot_id) 처리 → 전략 실행 → 최적성 지표 계산.

    - robot_id 가 지정된 작업은 그 로봇이 available 이면 바로 배정하고, 아니면 이번 라운드를 건너뛴다.
    - 전략에는 available 이고 아직 배정되지 않은 로봇만 넘긴다.
    """
    t0 = time.perf_counter()
    by_id: Dict[str, RobotInfo] = {r.robot_id: r for r in robots}
    free = {r.robot_id for r in robots if r.available}
    assignments: Dict[str, str] = {}
    unpinned: List[TaskSpec] = []
    for task in tasks:
        if task.robot_id:
            if task.robot_id in free:
                assignments[task.task_id] = task.robot_id
                free.discard(task.robot_id)
        else:
            unpinned.append(task)
    pool = [by_id[rid] for rid in sorted(free)]
    if unpinned and pool:
        for task_id, robot_id in strategy.assign(unpinned, pool).items():
            if robot_id in free and task_id not in assignments:
                assignments[task_id] = robot_id
                free.discard(robot_id)
    metrics = evaluate_assignment(assignments, tasks, robots, strategy.nominal_speed)
    metrics['strategy'] = strategy.name
    metrics['compute_time_ms'] = (time.perf_counter() - t0) * 1e3
    return AllocationResult(assignments=assignments, metrics=metrics)


def evaluate_assignment(assignments: Dict[str, str], tasks: Sequence[TaskSpec],
                        robots: Sequence[RobotInfo], nominal_speed: float) -> Dict[str, float]:
    """
    최적성 지표 (명세 4.9): 총 이동 거리, 접근 거리, 완료 시간 추정(makespan).

    makespan = 로봇별 (접근 + 배송) 이동 시간 합의 최대값 [s].
    """
    tasks_by_id = {t.task_id: t for t in tasks}
    robots_by_id = {r.robot_id: r for r in robots}
    total = approach = 0.0
    per_robot: Dict[str, float] = {}
    for task_id, robot_id in assignments.items():
        task, robot = tasks_by_id[task_id], robots_by_id[robot_id]
        a = approach_distance(robot, task)
        d = delivery_distance(task)
        approach += a
        total += a + d
        per_robot[robot_id] = per_robot.get(robot_id, 0.0) + travel_time(a + d, nominal_speed)
    n = len(assignments)
    return {
        'n_tasks': float(len(tasks)),
        'n_robots': float(len(robots)),
        'n_assigned': float(n),
        'total_travel_distance_m': total,
        'approach_distance_m': approach,
        'mean_approach_distance_m': approach / n if n else 0.0,
        'makespan_s': max(per_robot.values()) if per_robot else 0.0,
    }


__all__ = [
    'AllocationResult', 'AllocationStrategy', 'HungarianStrategy', 'LoadBalanceStrategy',
    'NearestStrategy', 'RobotInfo', 'allocate', 'available_strategies', 'brute_force_assignment',
    'create_strategy', 'evaluate_assignment', 'hungarian', 'register_strategy', 'select_batch',
    'solve_assignment', 'travel_time',
]
