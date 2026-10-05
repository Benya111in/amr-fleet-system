"""
동적 장애물 지면 진실 집합 — actor(사람, SDF 궤적 보간) + 물리 모델(지게차·셔틀, gz 오도메트리).

obstacle_truth_node(발행)와 collision_monitor_node(충돌 판정)가 같이 쓴다. 두 노드 모두 actor 는 같은 월드 SDF 를
읽어 요청 시각에 직접 보간하고(시각 정렬 오차 없음), 모델은 가장 최근 오도메트리를 그 시각까지 속도로 외삽한다
(OdometryPublisher 50 Hz → 최대 20 ms 외삽).
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional

from amr_simulation.actor_trajectory import ActorTrajectory
from amr_simulation.footprint import Rect


@dataclass
class ObstacleState:
    """한 시각의 장애물 상태 (월드 좌표)."""

    name: str
    kind: str            # person | vehicle
    x: float
    y: float
    yaw: float           # 몸체 자세
    vx: float            # 월드 속도
    vy: float
    radius: float = 0.0  # person: 원 반지름
    bounds: tuple = ()   # vehicle: 몸체 좌표 [x_min, x_max, y_min, y_max]
    height: float = 0.0

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    @property
    def heading(self) -> float:
        """이동 방향 (정지 중이면 몸체 자세)."""
        return math.atan2(self.vy, self.vx) if self.speed > 1e-3 else self.yaw

    def rect(self) -> Optional[Rect]:
        if self.kind == "person":
            return None
        return Rect(self.x, self.y, self.yaw, *self.bounds)


@dataclass
class _ModelTrack:
    bounds: tuple
    height: float
    stamp: float = math.nan
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


class ObstacleSet:
    """actor 궤적 + 모델 오도메트리로 임의 시각의 장애물 상태를 만든다."""

    def __init__(self, actors: List[ActorTrajectory], actor_radius: float, actor_height: float,
                 models: Dict[str, tuple], model_heights: Dict[str, float], max_extrapolation=0.5):
        self.actors = actors
        self.actor_radius = actor_radius
        self.actor_height = actor_height
        self.models = {n: _ModelTrack(tuple(b), model_heights.get(n, 1.0))
                       for n, b in models.items()}
        self.max_extrapolation = max_extrapolation

    def names(self) -> List[str]:
        return [a.name for a in self.actors] + list(self.models)

    def update_model(self, name: str, stamp: float, x: float, y: float, yaw: float,
                     v_body: float, v_lat: float, wz: float) -> None:
        """nav_msgs/Odometry 값으로 갱신 (twist 는 몸체 좌표 → 월드로 돌려 저장)."""
        m = self.models[name]
        c, s = math.cos(yaw), math.sin(yaw)
        m.stamp, m.x, m.y, m.yaw = stamp, x, y, yaw
        m.vx, m.vy, m.wz = c * v_body - s * v_lat, s * v_body + c * v_lat, wz

    def states(self, t: float) -> List[ObstacleState]:
        out = []
        for a in self.actors:
            x, y, yaw, vx, vy = a.state(t)
            out.append(ObstacleState(a.name, "person", x, y, yaw, vx, vy,
                                     radius=self.actor_radius, height=self.actor_height))
        for n, m in self.models.items():
            if math.isnan(m.stamp):
                continue                      # 아직 오도메트리를 못 받음
            dt = max(-self.max_extrapolation, min(self.max_extrapolation, t - m.stamp))
            out.append(ObstacleState(n, "vehicle", m.x + m.vx * dt, m.y + m.vy * dt,
                                     m.yaw + m.wz * dt, m.vx, m.vy,
                                     bounds=m.bounds, height=m.height))
        return out


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def declare_obstacle_params(node) -> dict:
    """두 노드 공통 파라미터 (config/dynamic_obstacles.yaml)."""
    p = {
        "world_file": node.declare_parameter("world_file", "").value,
        "actor_radius": node.declare_parameter("actor_radius", 0.30).value,
        "actor_height": node.declare_parameter("actor_height", 1.80).value,
        "models": list(node.declare_parameter("models", ["forklift_main", "shuttle_amr"]).value),
        "model_footprints": list(node.declare_parameter(
            "model_footprints", [-1.2, 2.0, -0.64, 0.64, -0.30, 0.30, -0.20, 0.20]).value),
        "model_heights": list(node.declare_parameter("model_heights", [2.2, 0.45]).value),
        "model_odom_topic": node.declare_parameter("model_odom_topic", "/sim/{name}/odom").value,
    }
    n = len(p["models"])
    if len(p["model_footprints"]) != 4 * n or len(p["model_heights"]) != n:
        raise ValueError("model_footprints 는 모델당 4개, model_heights 는 모델당 1개여야 한다")
    return p


def default_world_file() -> str:
    from ament_index_python.packages import get_package_share_directory
    import os
    return os.path.join(get_package_share_directory("amr_simulation"), "worlds", "warehouse.sdf")


def obstacle_set_from_params(p: dict) -> ObstacleSet:
    from amr_simulation.actor_trajectory import load_actors
    world = p["world_file"] or default_world_file()
    fp = p["model_footprints"]
    models = {name: tuple(fp[4 * i:4 * i + 4]) for i, name in enumerate(p["models"])}
    heights = {name: p["model_heights"][i] for i, name in enumerate(p["models"])}
    return ObstacleSet(load_actors(world), p["actor_radius"], p["actor_height"], models, heights)
