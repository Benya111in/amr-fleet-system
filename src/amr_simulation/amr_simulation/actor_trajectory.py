"""
Gazebo Fortress actor 궤적의 지면 진실 — SDF <trajectory> 웨이포인트를 Gazebo 와 같은 규칙으로 sim time 에 보간한다.

Fortress 의 actor 는 서버(물리)에서 움직이지 않고 렌더링 쪽이 궤적을 계산하므로 /world/<w>/pose/info 에 actor 포즈가
없다. gz-sim 6 은 궤적마다 ign-common PoseAnimation(키프레임 = 웨이포인트)을 만들고

  - 위치: ign-math Spline — 키프레임 사이 3차 Hermite. 매개변수 s = 구간 안 시간 비율 (t − t_k) / (t_{k+1} − t_k).
          접선 T_i = 0.5·(P_{i+1} − P_{i−1})·(1 − tension) (Catmull-Rom). 첫 점 = 끝 점이면 닫힌 곡선으로 보고
          T_0 = T_N = 0.5·(P_1 − P_{N−1})·(1 − tension),
          아니면 끝점은 한쪽 차분 0.5·(P_1 − P_0)·(1 − tension).
  - 자세: RotationSpline(squad). 여기서는 yaw 를 두 키프레임 사이에서 최단 방향 선형 보간한다 (근사 — 위치는 정확).
  - 반복: loop 이면 (sim 시간 − delay_start) mod 마지막 웨이포인트 시각.

으로 계산한다. 이 모듈은 같은 식을 그대로 구현한다 (worlds/gen_warehouse_world.py 가 등시간 간격 웨이포인트를 쓰므로
접선 = 속도 × 간격 이 되어 구간 경계에서도 속도가 이어진다). 표준 라이브러리만 쓴다 (단위 테스트에서 그대로 import).
"""

import math
import xml.etree.ElementTree as ET
from bisect import bisect_right
from dataclasses import dataclass, field
from typing import List, Tuple


def _hermite(p1, p2, t1, t2, s):
    s2, s3 = s * s, s * s * s
    h00, h01 = 2 * s3 - 3 * s2 + 1, -2 * s3 + 3 * s2
    h10, h11 = s3 - 2 * s2 + s, s3 - s2
    return tuple(h00 * a + h01 * b + h10 * c + h11 * d for a, b, c, d in zip(p1, p2, t1, t2))


def _hermite_ds(p1, p2, t1, t2, s):
    """d(위치)/ds."""
    s2 = s * s
    d00, d01 = 6 * s2 - 6 * s, -6 * s2 + 6 * s
    d10, d11 = 3 * s2 - 4 * s + 1, 3 * s2 - 2 * s
    return tuple(d00 * a + d01 * b + d10 * c + d11 * d for a, b, c, d in zip(p1, p2, t1, t2))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class ActorTrajectory:
    """actor 하나의 궤적. waypoints = [(t, x, y, z, yaw)], 시각은 궤적 시작 기준."""

    name: str
    waypoints: List[Tuple[float, float, float, float, float]]
    tension: float = 0.0
    loop: bool = True
    delay_start: float = 0.0
    _tangents: list = field(default_factory=list, repr=False)
    _times: list = field(default_factory=list, repr=False)

    def __post_init__(self):
        if len(self.waypoints) < 2:
            raise ValueError(f"{self.name}: 웨이포인트가 2개 미만")
        self._times = [w[0] for w in self.waypoints]
        pts = [w[1:4] for w in self.waypoints]
        n = len(pts)
        k = 0.5 * (1.0 - self.tension)
        closed = all(abs(a - b) <= 1e-6 for a, b in zip(pts[0], pts[-1]))
        tans = []
        for i in range(n):
            if i == 0:
                prev, nxt = (pts[n - 2], pts[1]) if closed else (pts[0], pts[1])
            elif i == n - 1:
                prev, nxt = (pts[n - 2], pts[1]) if closed else (pts[n - 2], pts[n - 1])
            else:
                prev, nxt = pts[i - 1], pts[i + 1]
            tans.append(tuple(k * (b - a) for a, b in zip(prev, nxt)))
        self._tangents = tans

    @property
    def duration(self) -> float:
        return self._times[-1]

    def _local_time(self, sim_time: float) -> float:
        t = sim_time - self.delay_start
        if t <= 0.0:
            return 0.0
        if self.loop and self.duration > 0.0:
            return math.fmod(t, self.duration)
        return min(t, self.duration)

    def _segment(self, tl: float):
        k = min(max(bisect_right(self._times, tl) - 1, 0), len(self._times) - 2)
        t0, t1 = self._times[k], self._times[k + 1]
        s = 0.0 if t1 <= t0 else min(max((tl - t0) / (t1 - t0), 0.0), 1.0)
        return k, s, t1 - t0

    def state(self, sim_time: float):
        """(x, y, yaw, vx, vy) — 월드 좌표 위치, 자세 yaw(근사), 속도 [m/s]."""
        tl = self._local_time(sim_time)
        k, s, dt = self._segment(tl)
        p1, p2 = self.waypoints[k][1:4], self.waypoints[k + 1][1:4]
        t1, t2 = self._tangents[k], self._tangents[k + 1]
        x, y, _ = _hermite(p1, p2, t1, t2, s)
        vx, vy, _ = (c / dt for c in _hermite_ds(p1, p2, t1, t2, s)) if dt > 0 else (0.0, 0.0, 0.0)
        y0, y1 = self.waypoints[k][4], self.waypoints[k + 1][4]
        yaw = _wrap(y0 + s * _wrap(y1 - y0))
        return x, y, yaw, vx, vy


def _floats(text: str) -> list:
    return [float(v) for v in text.split()]


def load_actors(sdf_path: str) -> List[ActorTrajectory]:
    """월드 SDF 의 모든 <actor> 궤적 (첫 trajectory 만 — 이 프로젝트 월드는 actor 마다 하나)."""
    root = ET.parse(sdf_path).getroot()
    actors = []
    for a in root.iter("actor"):
        script = a.find("script")
        if script is None:
            continue
        traj = script.find("trajectory")
        if traj is None:
            continue
        wps = []
        for w in traj.findall("waypoint"):
            t = float(w.find("time").text)
            x, y, z, _r, _p, yaw = _floats(w.find("pose").text)
            wps.append((t, x, y, z, yaw))
        wps.sort(key=lambda w: w[0])
        loop_e, delay_e = script.find("loop"), script.find("delay_start")
        actors.append(ActorTrajectory(
            name=a.get("name"), waypoints=wps, tension=float(traj.get("tension", "0")),
            loop=(loop_e is None or loop_e.text.strip().lower() in ("true", "1")),
            delay_start=float(delay_e.text) if delay_e is not None else 0.0))
    return actors
