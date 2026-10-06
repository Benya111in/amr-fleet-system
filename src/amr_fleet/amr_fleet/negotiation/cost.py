"""
입찰 비용 — 기존 중앙 할당과 **같은 정의**를 쓴다.

allocation.py 의 HungarianStrategy.cost_matrix 는
travel_time(approach_distance(robot, task), nominal_speed), 즉 "로봇 현재 위치 → 픽업 이동 시간 [s]"
이다. EuclideanCost 는 그 함수를 그대로 불러 같은 값을 낸다 (시험 test_nego_cost.py 가 행렬 일치를
확인한다). GridAstarCost 는 거리만 격자 A* 경로 길이로 바꾼 선택지다 (보고서 54행 "지도에서 미리
만든 격자 A* 거리 캐시").

중앙(nearest·hungarian)과 분산(CBAA)을 비교할 때는 양쪽에 같은 CostModel 을 줘야 공정하다 (보고서 189행).
"""

from __future__ import annotations

import abc
from typing import List, Sequence

from amr_fleet.allocation import approach_distance, RobotInfo, travel_time
from amr_fleet.negotiation.grid import OccupancyGrid
from amr_fleet.task_schema import TaskSpec


class CostModel(abc.ABC):
    """로봇 → 작업 입찰 비용 [s]."""

    name = 'base'

    def __init__(self, nominal_speed: float = 1.0):
        if nominal_speed <= 0.0:
            raise ValueError('nominal_speed 는 양수여야 한다')
        self.nominal_speed = float(nominal_speed)

    @abc.abstractmethod
    def approach_m(self, robot: RobotInfo, task: TaskSpec) -> float:
        """로봇 현재 위치 → 픽업 이동 거리 [m] (도달 불가면 inf)."""

    def cost(self, robot: RobotInfo, task: TaskSpec) -> float:
        """이동 시간 [s] = 거리 / 공칭 속도 (allocation.travel_time)."""
        return travel_time(self.approach_m(robot, task), self.nominal_speed)

    def matrix(self, robots: Sequence[RobotInfo], tasks: Sequence[TaskSpec]) -> List[List[float]]:
        """비용 행렬 (행 = 로봇, 열 = 작업) — HungarianStrategy.cost_matrix 와 같은 모양."""
        return [[self.cost(r, t) for t in tasks] for r in robots]


class EuclideanCost(CostModel):
    """allocation.approach_distance (평면 유클리드) — 중앙 할당과 같은 값."""

    name = 'euclid'

    def approach_m(self, robot: RobotInfo, task: TaskSpec) -> float:
        return approach_distance(robot, task)


class GridAstarCost(CostModel):
    """점유 격자 위 8-이웃 A* 경로 길이 (grid.OccupancyGrid.path_length, 쌍마다 기억)."""

    name = 'grid_astar'

    def __init__(self, grid: OccupancyGrid, nominal_speed: float = 1.0):
        super().__init__(nominal_speed)
        self.grid = grid

    def approach_m(self, robot: RobotInfo, task: TaskSpec) -> float:
        return self.grid.path_length((robot.x, robot.y), (task.pickup.x, task.pickup.y))


__all__ = ['CostModel', 'EuclideanCost', 'GridAstarCost']
