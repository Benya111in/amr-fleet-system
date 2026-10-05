"""
정답(GT) 되먹임 경로 추종 — 파일럿 bag 기록용 주행 (rclpy 비의존, pytest 로 바로 시험한다).

bag 을 여러 SLAM 행에 똑같이 재생하려면 주행 경로가 추정기와 독립이어야 한다 (보고서 「3D LiDAR SLAM 도입 방안」
'한 번 기록하고 여러 번 재생' — 경로를 정답 되먹임으로 정하므로 재생 비교가 성립한다). 그래서 자세 입력은
`ground_truth/odom`(월드 프레임)이고 출력은 차동구동 명령 (v, ω) 이다.

단계: hold_start(정지 — IMU 바이어스 추정·SLAM 초기화) → 웨이포인트마다 [rotate(제자리 회전) → drive(선분 추종)
→ dwell(정지)] → hold_end → done.

drive 는 선분 a→b 를 따른다: ω = k_h·wrap(θ_seg − yaw) − k_e·e (e = 선분 왼쪽 법선 방향 횡오차),
v = min(v_max, √(2·a·남은 거리), v_prev + a·dt). 선분 진행 거리 s 가 길이 − goal_tol 에 닿으면 도착이다.
목표점 방향(atan2)이 아니라 선분 방향을 쓰는 이유: 목표 근처에서 atan2 가 튀어 제자리 회전이 생기지 않게 한다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import List, Optional, Sequence, Tuple

from amr_slam3d.se2 import wrap

Point = Tuple[float, float]


@dataclass(frozen=True)
class Limits:
    """주행 한계·이득 (단위: m, s, rad)."""

    v_max: float = 0.5           # [m/s] 직진 최고 속도
    w_max: float = 0.5           # [rad/s] 회전 최고 각속도
    a_max: float = 0.5           # [m/s²] 직진 가감속
    w_min: float = 0.05          # [rad/s] 제자리 회전 최소 각속도 (마찰로 멈추지 않게)
    k_heading: float = 1.5       # [1/s] 방향 오차 이득
    k_cross: float = 1.0         # [rad/(s·m)] 횡오차 이득
    align_tol: float = 0.03      # [rad] 이 안이면 회전을 끝내고 직진
    realign_tol: float = 0.5     # [rad] 직진 중 방향 오차가 이보다 크면 멈추고 다시 회전
    goal_tol: float = 0.05       # [m] 선분 끝 도착 판정
    cross_max: float = 0.5       # [m] 횡오차가 이보다 크면 멈추고 목표점을 향해 다시 회전


@dataclass
class RouteSpec:
    """경로 정의 (config/pilot_route.yaml 의 route 블록)."""

    waypoints: List[Point]
    hold_start_s: float = 20.0
    hold_end_s: float = 5.0
    dwell_s: float = 1.0
    limits: Limits = field(default_factory=Limits)


@dataclass
class Command:
    """한 주기 명령."""

    v: float
    w: float
    phase: str
    index: int            # 지금 향하는 웨이포인트 번호 (0부터)
    done: bool = False


def route_from_dict(data: dict) -> RouteSpec:
    """YAML 의 route 블록 → RouteSpec (모르는 키는 ValueError — 오타로 기본값이 조용히 쓰이지 않게)."""
    known_top = {'waypoints', 'hold_start_s', 'hold_end_s', 'dwell_s', 'limits', 'start'}
    unknown = set(data) - known_top
    if unknown:
        raise ValueError(f'route 에 모르는 키: {sorted(unknown)}')
    wps = [(float(p[0]), float(p[1])) for p in data.get('waypoints', [])]
    if not wps:
        raise ValueError('waypoints 가 비었다')
    lim_data = data.get('limits') or {}
    names = set(Limits.__dataclass_fields__)
    bad = set(lim_data) - names
    if bad:
        raise ValueError(f'limits 에 모르는 키: {sorted(bad)}')
    limits = Limits(**{k: float(v) for k, v in lim_data.items()})
    return RouteSpec(waypoints=wps, hold_start_s=float(data.get('hold_start_s', 20.0)),
                     hold_end_s=float(data.get('hold_end_s', 5.0)),
                     dwell_s=float(data.get('dwell_s', 1.0)), limits=limits)


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class RouteFollower:
    """웨이포인트 목록을 정답 자세로 따라간다. step() 을 일정 주기로 부른다."""

    def __init__(self, spec: RouteSpec):
        self.spec = spec
        self.lim = spec.limits
        self.phase = 'hold_start'
        self.index = 0
        self._t0: Optional[float] = None
        self._t_phase = 0.0
        self._t_prev: Optional[float] = None
        self._v_prev = 0.0
        self._seg_a: Optional[Point] = None

    def _enter(self, phase: str, t: float) -> None:
        self.phase = phase
        self._t_phase = t

    def _segment(self) -> Tuple[Point, Point]:
        b = self.spec.waypoints[self.index]
        a = self._seg_a if self._seg_a is not None else b
        return a, b

    def step(self, t: float, x: float, y: float, yaw: float) -> Command:
        """시각 t [s] 의 정답 자세 (x, y, yaw) → 명령."""
        if self._t0 is None:
            self._t0 = t
            self._t_phase = t
        dt = 0.0 if self._t_prev is None else max(0.0, t - self._t_prev)
        self._t_prev = t
        cmd = self._step(t, x, y, yaw, dt)
        self._v_prev = cmd.v
        return cmd

    def _step(self, t: float, x: float, y: float, yaw: float, dt: float) -> Command:
        lim = self.lim
        if self.phase == 'hold_start':
            if t - self._t_phase < self.spec.hold_start_s:
                return Command(0.0, 0.0, self.phase, self.index)
            self._seg_a = (x, y)
            self._enter('rotate', t)
        if self.phase == 'dwell':
            if t - self._t_phase < self.spec.dwell_s:
                return Command(0.0, 0.0, self.phase, self.index)
            self.index += 1
            if self.index >= len(self.spec.waypoints):
                self._enter('hold_end', t)
            else:
                self._enter('rotate', t)
        if self.phase == 'hold_end':
            if t - self._t_phase < self.spec.hold_end_s:
                return Command(0.0, 0.0, self.phase, len(self.spec.waypoints) - 1)
            self._enter('done', t)
        if self.phase == 'done':
            return Command(0.0, 0.0, 'done', len(self.spec.waypoints) - 1, done=True)

        b = self.spec.waypoints[self.index]
        if self.phase == 'rotate':
            target = math.atan2(b[1] - y, b[0] - x)
            if math.hypot(b[0] - x, b[1] - y) < lim.goal_tol:
                self._enter('dwell', t)
                return Command(0.0, 0.0, 'dwell', self.index)
            err = wrap(target - yaw)
            if abs(err) <= lim.align_tol:
                # 지금 자리에서 목표까지를 새 선분으로 삼는다 (회전 중 미끄러짐까지 흡수)
                self._seg_a = (x, y)
                self._enter('drive', t)
            else:
                w = _clamp(lim.k_heading * err, -lim.w_max, lim.w_max)
                if abs(w) < lim.w_min:
                    w = math.copysign(lim.w_min, err)
                return Command(0.0, w, 'rotate', self.index)

        # drive: 선분 a→b 추종
        a, b = self._segment()
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg < 1e-9:
            self._enter('dwell', t)
            return Command(0.0, 0.0, 'dwell', self.index)
        ux, uy = (b[0] - a[0]) / seg, (b[1] - a[1]) / seg
        px, py = x - a[0], y - a[1]
        s = px * ux + py * uy                 # 선분 진행 거리
        e = -px * uy + py * ux                # 왼쪽 법선 방향 횡오차 (양수 = 선 왼쪽)
        remaining = seg - s
        if remaining <= lim.goal_tol:
            self._enter('dwell', t)
            return Command(0.0, 0.0, 'dwell', self.index)
        heading_err = wrap(math.atan2(uy, ux) - yaw)
        if abs(heading_err) > lim.realign_tol or abs(e) > lim.cross_max:
            self._enter('rotate', t)
            return Command(0.0, 0.0, 'rotate', self.index)
        v_cap = math.sqrt(max(0.0, 2.0 * lim.a_max * remaining))
        v = min(lim.v_max, v_cap, self._v_prev + lim.a_max * dt if dt > 0.0 else lim.v_max)
        v = max(0.0, v)
        w = _clamp(lim.k_heading * heading_err - lim.k_cross * e, -lim.w_max, lim.w_max)
        return Command(v, w, 'drive', self.index)


def simulate(spec: RouteSpec, start: Tuple[float, float, float], dt: float = 0.02,
             t_max: float = 2000.0, slip: float = 0.0,
             w_bias: float = 0.0) -> List[Tuple[float, float, float, float, Command]]:
    """
    이상 유니사이클로 경로를 끝까지 돌린다 (시험·경로 길이 추정용).

    slip: 직진 명령의 비율 오차 (실제 v = (1 − slip)·v). w_bias: 실제 각속도에 더하는 치우침 [rad/s]
    (좌우 바퀴 불균형 흉내). 반환: [(t, x, y, yaw, 명령)].
    """
    follower = RouteFollower(spec)
    x, y, yaw = start
    t = 0.0
    log = []
    while t <= t_max:
        cmd = follower.step(t, x, y, yaw)
        log.append((t, x, y, yaw, cmd))
        if cmd.done:
            break
        v = cmd.v * (1.0 - slip)
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        w = cmd.w + (w_bias if cmd.v > 0.0 else 0.0)
        yaw = wrap(yaw + w * dt)
        t += dt
    return log


def segment_points(spec: RouteSpec, start: Sequence[float]) -> List[Point]:
    """출발점 + 웨이포인트 (경로 길이 계산용)."""
    return [(float(start[0]), float(start[1]))] + list(spec.waypoints)
