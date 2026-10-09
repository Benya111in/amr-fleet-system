"""
동적 장애물 (작업자 6 · 지게차 · 셔틀) — world.sdf 궤적을 시드별 위상으로 돌린다.

Gazebo 에서 궤적은 결정적이다(사람 actor 웨이포인트, 차량 왕복 플러그인). 실행마다 다른 것은 로봇과의 상대 위상뿐이라
여기서는 actor 마다 위상 φ ~ U(0, 주기)를 (seed, 이름)에서 뽑는다 — 같은 시드면 배치와 무관하게 같은 위상이다
(공통 난수). 위치는 지평 전체를 dt 간격으로 미리 계산해 둔다 (단계마다 표 조회).

로봇과의 상호작용(멈춤 · 감속 · 지나가기)은 robot.py 의 몸체 막힘 규칙이 맡는다 — actor 는 로봇에게 비키지 않는다
(Gazebo actor 는 충돌체가 없고 지게차 · 셔틀은 고정 경로 왕복이다).
"""

from __future__ import annotations

import math
from typing import List

import numpy as np

from layopt.sim.layout import ActorSpec
from layopt.sim.workload import uniform


class ActorField:
    """모든 actor 의 위치 · 속도 표 (단계 k → (N, ...) 배열)."""

    def __init__(self, actors: List[ActorSpec], seed: int, horizon_s: float, dt: float,
                 enabled: bool = True):
        self.actors = list(actors) if enabled else []
        self.names = [a.name for a in self.actors]
        self.kinds = [a.kind for a in self.actors]
        n = len(self.actors)
        steps = int(round(horizon_s / dt)) + 2
        self.dt = dt
        self.n = n
        self.half_len = np.array([a.half_len for a in self.actors])
        self.half_wid = np.array([a.half_wid for a in self.actors])
        self.xy = np.zeros((steps, n, 2))
        self.vel = np.zeros((steps, n, 2))
        self.phase = []
        t = np.arange(steps) * dt
        for i, a in enumerate(self.actors):
            per = a.period
            ph = uniform(0.0, per, seed, 'actor_phase', a.name) if per > 0 else 0.0
            self.phase.append(ph)
            tt = np.mod(t + ph, per) if per > 0 else np.zeros_like(t)
            self.xy[:, i, 0] = np.interp(tt, a.t, a.x)
            self.xy[:, i, 1] = np.interp(tt, a.t, a.y)
        if n:
            self.vel[1:] = (self.xy[1:] - self.xy[:-1]) / dt
            self.vel[0] = self.vel[1]

    def at(self, k: int):
        """단계 k 의 (위치 (N, 2), 속도 (N, 2))."""
        k = min(k, len(self.xy) - 1)
        return self.xy[k], self.vel[k]

    def speed(self, k: int) -> np.ndarray:
        _, v = self.at(k)
        return np.hypot(v[:, 0], v[:, 1])


def heading_of(vx: float, vy: float, default: float = 0.0) -> float:
    return math.atan2(vy, vx) if vx * vx + vy * vy > 1e-8 else default


__all__ = ['ActorField', 'heading_of']
