"""
대역 시뮬레이터용 운동학·기하 (rclpy 비의존 순수 모듈).

kinematic_sim 대역이 Gazebo(DiffDrive + 센서) 자리를 채울 때 쓰는 최소 모델이다.
  - 유니사이클 적분: 명령 (v, ω) 을 robot_params.yaml limits 의 가속 한계로 추종, 원호 정확 적분
  - 차동 구동: 바퀴 각속도 ↔ 차체 속도 (순기구학/역기구학)
  - 2D 레이캐스트: 원/축 정렬 박스 장애물 → LaserScan 거리
  - 풋프린트 여유 거리: 차체 사각형 가장자리 ~ 점 최단 거리 (safety 거리 기준, robot_params.yaml
    safety.distance_reference: footprint_edge)
"""

from dataclasses import dataclass
import math
from typing import Dict, Iterable, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class Limits:
    """동역학 제한 (robot_params.yaml limits)."""

    v_max: float = 2.0
    v_min: float = -0.5
    a_max: float = 1.0
    w_max: float = 1.5
    alpha_max: float = 2.0

    @classmethod
    def from_params(cls, params: Dict) -> 'Limits':
        lim = params.get('limits', {})
        return cls(float(lim.get('max_linear_velocity', 2.0)),
                   float(lim.get('min_linear_velocity', -0.5)),
                   float(lim.get('max_linear_acceleration', 1.0)),
                   float(lim.get('max_angular_velocity', 1.5)),
                   float(lim.get('max_angular_acceleration', 2.0)))


@dataclass
class State:
    """평면 자세 + 차체 속도."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    w: float = 0.0


def wrap(a: float) -> float:
    """(-pi, pi] 래핑."""
    return math.atan2(math.sin(a), math.cos(a))


def approach(current: float, target: float, max_delta: float) -> float:
    """현재값 current 를 target 쪽으로 최대 max_delta 만큼 옮긴다."""
    if target > current:
        return min(target, current + max_delta)
    return max(target, current - max_delta)


def step(state: State, v_cmd: float, w_cmd: float, dt: float,
         limits: Limits = Limits()) -> Tuple[State, float, float]:
    """
    시간 dt 동안 명령을 가속 한계로 추종하며 적분한다.

    반환: (새 상태, 차체 전방 가속도 ax, 측방 가속도 ay = v·ω) [m/s²].
    위치는 구간 평균 속도로 원호 정확 적분한다 (ω → 0 이면 직선).
    """
    if dt <= 0.0:
        return State(state.x, state.y, state.yaw, state.v, state.w), 0.0, 0.0
    v_t = min(max(v_cmd, limits.v_min), limits.v_max)
    w_t = min(max(w_cmd, -limits.w_max), limits.w_max)
    v1 = approach(state.v, v_t, limits.a_max * dt)
    w1 = approach(state.w, w_t, limits.alpha_max * dt)
    v_avg = 0.5 * (state.v + v1)
    w_avg = 0.5 * (state.w + w1)
    dth = w_avg * dt
    if abs(dth) < 1e-9:
        dx = v_avg * dt * math.cos(state.yaw)
        dy = v_avg * dt * math.sin(state.yaw)
    else:
        r = v_avg / w_avg
        dx = r * (math.sin(state.yaw + dth) - math.sin(state.yaw))
        dy = -r * (math.cos(state.yaw + dth) - math.cos(state.yaw))
    new = State(state.x + dx, state.y + dy, wrap(state.yaw + dth), v1, w1)
    ax = (v1 - state.v) / dt
    ay = v_avg * w_avg
    return new, ax, ay


def wheel_rates(v: float, w: float, radius: float, separation: float) -> Tuple[float, float]:
    """차체 (v, ω) → 바퀴 각속도 (좌, 우) [rad/s] (역기구학)."""
    return (v - w * separation / 2.0) / radius, (v + w * separation / 2.0) / radius


def body_increment(dphi_l: float, dphi_r: float, radius: float,
                   separation: float) -> Tuple[float, float]:
    """바퀴 회전 증분 [rad] → (이동 거리 Δs [m], 회전 Δθ [rad]) (순기구학)."""
    sl, sr = dphi_l * radius, dphi_r * radius
    return 0.5 * (sl + sr), (sr - sl) / separation


def integrate_pose(x: float, y: float, yaw: float, ds: float,
                   dth: float) -> Tuple[float, float, float]:
    """증분 (Δs, Δθ) 을 중점 헤딩으로 적분한다 (2차 Runge-Kutta)."""
    mid = yaw + 0.5 * dth
    return x + ds * math.cos(mid), y + ds * math.sin(mid), wrap(yaw + dth)


def quantize(angle: float, ticks_per_rev: int) -> float:
    """바퀴 각도 [rad] 를 엔코더 틱 분해능으로 양자화한다 (내림)."""
    tick = 2.0 * math.pi / ticks_per_rev
    return math.floor(angle / tick) * tick


def beam_angles(angle_min: float, angle_max: float, samples: int) -> np.ndarray:
    """스캔(LaserScan) 빔 각도 (angle_min 부터 samples 개, 증분 = (max − min)/(samples − 1))."""
    if samples < 2:
        return np.array([angle_min])
    return angle_min + np.arange(samples) * (angle_max - angle_min) / (samples - 1)


def raycast(ox: float, oy: float, oyaw: float, angles: np.ndarray,
            circles: Sequence[Tuple[float, float, float]] = (),
            boxes: Sequence[Tuple[float, float, float, float]] = (),
            range_max: float = 25.0) -> np.ndarray:
    """
    센서 원점 (ox, oy, 방위 oyaw) 에서 빔마다 최근접 교차 거리 [m] (없으면 inf).

    circles: (cx, cy, r) 월드 좌표, boxes: (xmin, ymin, xmax, ymax) 월드 축 정렬.
    """
    th = oyaw + np.asarray(angles, dtype=float)
    dx, dy = np.cos(th), np.sin(th)
    best = np.full(th.shape, np.inf)
    for cx, cy, r in circles:
        fx, fy = ox - cx, oy - cy
        b = fx * dx + fy * dy
        c = fx * fx + fy * fy - r * r
        disc = b * b - c
        hit = disc >= 0.0
        sq = np.sqrt(np.where(hit, disc, 0.0))
        t1 = -b - sq
        t2 = -b + sq
        t = np.where(t1 > 1e-9, t1, np.where(t2 > 1e-9, 0.0 if c < 0 else t2, np.inf))
        t = np.where(hit, t, np.inf)
        best = np.minimum(best, t)
    for xmin, ymin, xmax, ymax in boxes:
        with np.errstate(divide='ignore', invalid='ignore'):
            tx1 = np.where(dx != 0.0, (xmin - ox) / dx, -np.inf)
            tx2 = np.where(dx != 0.0, (xmax - ox) / dx, np.inf)
            ty1 = np.where(dy != 0.0, (ymin - oy) / dy, -np.inf)
            ty2 = np.where(dy != 0.0, (ymax - oy) / dy, np.inf)
        inside_x = (xmin <= ox <= xmax)
        inside_y = (ymin <= oy <= ymax)
        tx_lo = np.where(dx != 0.0, np.minimum(tx1, tx2), -np.inf if inside_x else np.inf)
        tx_hi = np.where(dx != 0.0, np.maximum(tx1, tx2), np.inf if inside_x else -np.inf)
        ty_lo = np.where(dy != 0.0, np.minimum(ty1, ty2), -np.inf if inside_y else np.inf)
        ty_hi = np.where(dy != 0.0, np.maximum(ty1, ty2), np.inf if inside_y else -np.inf)
        t_near = np.maximum(tx_lo, ty_lo)
        t_far = np.minimum(tx_hi, ty_hi)
        hit = (t_far >= t_near) & (t_far > 1e-9)
        t = np.where(t_near > 1e-9, t_near, 0.0)
        best = np.minimum(best, np.where(hit, t, np.inf))
    best[best > range_max] = np.inf
    return best


def rect_distance(px: np.ndarray, py: np.ndarray, half_length: float,
                  half_width: float) -> np.ndarray:
    """원점 중심 축 정렬 사각형(차체 풋프린트) 가장자리 ~ 점 거리 [m] (안이면 0)."""
    ddx = np.maximum(np.abs(px) - half_length, 0.0)
    ddy = np.maximum(np.abs(py) - half_width, 0.0)
    return np.hypot(ddx, ddy)


def footprint_clearance(points_base: np.ndarray, length: float, width: float) -> float:
    """base_link 좌표 점들 (N, 2) 중 풋프린트 가장자리까지 최단 거리 [m] (점이 없으면 inf)."""
    pts = np.asarray(points_base, dtype=float).reshape(-1, 2)
    if len(pts) == 0:
        return math.inf
    return float(np.min(rect_distance(pts[:, 0], pts[:, 1], length / 2.0, width / 2.0)))


def scan_to_points(ranges: Sequence[float], angles: np.ndarray,
                   extrinsic: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    """스캔(LaserScan) 거리 → base_link 평면 점 (N, 2). extrinsic = 라이다 (x, y, yaw)."""
    r = np.asarray(ranges, dtype=float)
    a = np.asarray(angles, dtype=float)
    ok = np.isfinite(r) & (r > 0.0)
    ex, ey, eyaw = extrinsic
    th = a[ok] + eyaw
    return np.column_stack([ex + r[ok] * np.cos(th), ey + r[ok] * np.sin(th)])


def box_outline(box: Tuple[float, float, float, float], step: float = 0.02) -> np.ndarray:
    """박스 (xmin, ymin, xmax, ymax) 테두리 표본점 (N, 2) — 여유 거리 계산용."""
    xmin, ymin, xmax, ymax = box
    nx = max(2, int(math.ceil((xmax - xmin) / step)) + 1)
    ny = max(2, int(math.ceil((ymax - ymin) / step)) + 1)
    xs = np.linspace(xmin, xmax, nx)
    ys = np.linspace(ymin, ymax, ny)
    return np.vstack([np.column_stack([xs, np.full(nx, ymin)]),
                      np.column_stack([xs, np.full(nx, ymax)]),
                      np.column_stack([np.full(ny, xmin), ys]),
                      np.column_stack([np.full(ny, xmax), ys])])


def world_to_base(points: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    """월드 점 (N, 2) → 로봇(base) 좌표."""
    p = np.asarray(points, dtype=float).reshape(-1, 2) - np.array([x, y])
    c, s = math.cos(yaw), math.sin(yaw)
    return p @ np.array([[c, -s], [s, c]])


def base_to_world(points: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    """로봇(base) 점 (N, 2) → 월드 좌표."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    c, s = math.cos(yaw), math.sin(yaw)
    return p @ np.array([[c, s], [-s, c]]) + np.array([x, y])


def clearance_to_boxes(state: State, boxes: Iterable[Tuple[float, float, float, float]],
                       length: float, width: float) -> float:
    """로봇 풋프린트 ~ 월드 박스들 최단 여유 거리 [m] (박스가 없으면 inf)."""
    best = math.inf
    for box in boxes:
        pts = world_to_base(box_outline(box), state.x, state.y, state.yaw)
        best = min(best, footprint_clearance(pts, length, width))
    return best


def yaw_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """방위각 yaw → 쿼터니언 (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def gaussian(rng: np.random.Generator, sigma: float, size: Optional[int] = None):
    """σ ≤ 0 이면 0 을 돌려주는 가우시안 표본."""
    if sigma <= 0.0:
        return 0.0 if size is None else np.zeros(size)
    return rng.normal(0.0, sigma, size)
