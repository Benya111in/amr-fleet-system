"""
중앙 할당 · 충전기 배정.

할당 (fleet_manager_node 의미, fleet.yaml): allocation_period_s 0.5 s 마다 후보 = IDLE · 작업 없음 · 실행기가 받을
로봇(잔량 ≥ accept_battery_min_percent 20 %, 측위 복구 중 아님). 대기 작업을 TaskScheduler 순서로 select_batch
(k = 후보 수) → allocate(nearest) → 즉시 시작. 통신 지연(0–100 ms) · 거절 backoff 는 넣지 않는다 (작업 시간에 견주어
작다). 복귀 중 로봇은 후보가 아니다 (accept_while_returning: false).

충전기 (과제 지시): 가장 가까운(직선 거리) 빈 충전기를 점유한다. 없으면 가장 가까운 충전기의 대기열 자세
(layout.json 충전기 체류 지점의 첫 이웃, 없으면 staging 바깥 2.5 m)에서 기다린다. 현행 BT(charge.xml) 는 로봇 번호로
돌린 순서로 고르고 빈 곳이 없으면 그 자리에서 10 s 뒤 다시 시도한다 — 다른 점은 sim.md 에 적었다.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional

from layopt.sim import ensure_paths
from layopt.sim.layout import Layout, Pose
from layopt.sim.workload import Task

ensure_paths()
from amr_fleet.allocation import allocate, create_strategy, RobotInfo, select_batch  # noqa: E402
from amr_fleet.scheduler import TaskScheduler  # noqa: E402
from amr_fleet.task_schema import Pose2D, TaskSpec  # noqa: E402

ALLOC_PERIOD_S = 0.5


class Dispatcher:
    """중앙 할당 (nearest)."""

    def __init__(self, layout: Layout, strategy: str = 'nearest'):
        self.layout = layout
        self.strategy = create_strategy(strategy, nominal_speed=1.0)
        self.sched = TaskScheduler()
        self.tasks: Dict[str, Task] = {}
        self.next_t = 0.0

    def post(self, task: Task) -> None:
        x, y, _ = self.layout.docks[task.pickup]
        dx, dy, _ = self.layout.docks[task.dropoff]
        spec = TaskSpec(task_id=task.task_id, pickup=Pose2D(x, y), dropoff=Pose2D(dx, dy),
                        item_type=task.item_type, item_mass=task.mass, priority=task.priority,
                        created=task.t)
        self.tasks[task.task_id] = task
        self.sched.push(spec)

    def pending(self) -> int:
        return len(self.sched)

    def round(self, now: float, robots) -> List[tuple]:
        """할당 라운드 → [(robot, task)]."""
        if now + 1e-9 < self.next_t:
            return []
        self.next_t = now + ALLOC_PERIOD_S
        if not len(self.sched):
            return []
        cands = [r for r in robots if r.available(now)]
        if not cands:
            return []
        infos = [RobotInfo(r.id, r.x, r.y) for r in cands]
        batch = select_batch(self.sched.ordered(now), infos, 0)
        if not batch:
            return []
        res = allocate(self.strategy, batch, infos)
        by_id = {r.id: r for r in cands}
        out = []
        for tid, rid in sorted(res.assignments.items()):
            self.sched.remove(tid)
            out.append((by_id[rid], self.tasks[tid]))
        return out


class ChargerManager:
    """충전기 점유 (가장 가까운 빈 곳)."""

    def __init__(self, layout: Layout, queue_m: float = 2.5):
        self.layout = layout
        self.owner: Dict[str, Optional[str]] = {c: None for c in sorted(layout.chargers)}
        self.queue: Dict[str, Pose] = {}
        for c, (x, y, th) in layout.chargers.items():
            q = None
            for d in layout.dwell_points:
                if d.kind == 'charger' and d.id == f'charger:{c}' and d.neighbors:
                    q = d.neighbors[0]
            if q is None:
                q = (x - queue_m * math.cos(th), y - queue_m * math.sin(th), th)
            self.queue[c] = q
        self.claims = 0
        self.waits = 0

    def _nearest(self, robot, cands) -> Optional[str]:
        best, bd = None, math.inf
        for c in cands:
            x, y, _ = self.layout.chargers[c]
            d = math.hypot(x - robot.x, y - robot.y)
            if d < bd - 1e-9:
                best, bd = c, d
        return best

    def claim(self, robot) -> Optional[str]:
        free = [c for c, o in self.owner.items() if o is None or o == robot.id]
        c = self._nearest(robot, free)
        if c is not None:
            self.owner[c] = robot.id
            self.claims += 1
        return c

    def release(self, robot) -> None:
        for c, o in self.owner.items():
            if o == robot.id:
                self.owner[c] = None

    def queue_pose(self, robot) -> Pose:
        self.waits += 1
        return self.queue[self._nearest(robot, list(self.owner))]


__all__ = ['ALLOC_PERIOD_S', 'ChargerManager', 'Dispatcher']
