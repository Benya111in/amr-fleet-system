"""
점유 지도 → Nav2 식 코스트맵 · 팀 A* 경로 · 여유 거리 지도.

- 정적 층 (nav2 static_layer, trinary_costmap): p = (255 − 화소)/255, p > occupied_thresh → LETHAL 254,
  p < free_thresh → 0, 그 사이 → NO_INFORMATION 255 (track_unknown_space).
- 팽창 (전역 코스트맵 inflation_layer): 내접 0.20 m, inflation_radius 1.2 m, cost_scaling_factor 2.0
  (nav2_params.yaml global_costmap). amr_core_py.inflate =
  src/amr_navigation/src/core/grid.cpp inflate().
- 경로: 팀 A* (amr_navigation::AStarPlanner, cost_weight κ = 2.0, allow_unknown · unknown_cost 252,
  tolerance 0.25 m, 숏컷 + 평활) = amr_core_py.plan
  (src/amr_navigation/src/core/astar.cpp + path_smoother.cpp).
- 주차 로봇: 전역 코스트맵 obstacle_layer 는 멈춘 로봇을 장애물로 본다. 로봇마다 "다른 로봇들의 대기 슬롯에 차체
  (0.60 × 0.40 m)가 서 있는" 코스트맵 판을 따로 팽창해 두고 그 로봇의 계획에 쓴다 (sim.md §근사).
- 여유 거리 지도: 셀 중심 → 가장 가까운 장애물(점유 · 미지) 거리 [m]. 지나가기 판정(robot.py)에 쓴다.

같은 프로세스 안에서 구간 경로를 기억한다 (같은 배치의 시드 · 부하끼리 공유).
"""

from __future__ import annotations

import math
import pathlib
from typing import Dict, Iterable, Optional, Sequence, Tuple

import numpy as np
from scipy import ndimage

from layopt.sim import ensure_paths
from layopt.sim.layout import Layout, Pose

ensure_paths()
from amr_fleet.fleetsim.geometry import Path, rect_corners  # noqa: E402
from amr_fleet.negotiation.grid import _read_map_yaml, read_pgm  # noqa: E402
from amr_fleet.traffic_geometry import GridSpec  # noqa: E402

try:
    import amr_core_py as _core          # layopt/sim/native/build.sh
except ImportError as _e:                # pragma: no cover - 빌드 안 했을 때 안내
    _core = None
    _CORE_ERR = _e

LETHAL, INSCRIBED, UNKNOWN = 254, 253, 255
INSCRIBED_R = 0.20           # 풋프린트 내접 반경 (0.60 × 0.40)
INFLATION_R = 1.2            # global_costmap inflation_layer.inflation_radius
COST_SCALING = 2.0           # global_costmap inflation_layer.cost_scaling_factor
COST_WEIGHT = 2.0            # planner_server AStar.cost_weight
GOAL_TOL = 0.25              # AStar.tolerance
V_MAX = 1.0                  # DWA max_vel_x
ACC = 1.0                    # robot_params limits.max_linear_acceleration
OMEGA_MAX = 1.5              # robot_params limits.max_angular_velocity
CURV_WINDOW_M = 0.3          # 곡률 추정 창 (경로 점 간격 0.05 m 의 6 칸)
V_MIN_CURVE = 0.2            # 곡률 제한 하한 [m/s] (급커브에서도 이만큼은 간다)


def core_available() -> bool:
    return _core is not None


class NavMap:
    """한 배치의 지도 · 코스트맵 판 · 경로 기억."""

    def __init__(self, layout: Layout):
        if _core is None:
            raise ImportError('amr_core_py 없음 — bash layopt/sim/native/build.sh 로 빌드 '
                              f'({_CORE_ERR})')
        self.layout = layout
        meta = _read_map_yaml(str(layout.map_yaml))
        image = pathlib.Path(meta['image'])
        if not image.is_absolute():
            image = pathlib.Path(layout.map_yaml).resolve().parent / image
        w, h, pix = read_pgm(str(image))
        img = np.frombuffer(pix, dtype=np.uint8).reshape(h, w)[::-1, :]   # 행 0 = 지도 아래
        negate = int(meta.get('negate', 0))
        p = img / 255.0 if negate else (255.0 - img) / 255.0
        occ_t = float(meta.get('occupied_thresh', 0.65))
        free_t = float(meta.get('free_thresh', 0.196))
        cells = np.zeros((h, w), dtype=np.uint8)
        cells[p > occ_t] = LETHAL
        cells[(p >= free_t) & (p <= occ_t)] = UNKNOWN
        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        self.w, self.h = w, h
        self.static = cells
        self.spec = GridSpec(self.res, self.ox, self.oy, w, h)
        self.occupied = (cells == LETHAL) | (cells == UNKNOWN)          # TrafficManager.set_map 입력
        self.clearance = ndimage.distance_transform_edt(~self.occupied) * self.res
        self.base = _core.inflate(cells, self.res, INSCRIBED_R, INFLATION_R, COST_SCALING)
        self._variants: Dict[str, np.ndarray] = {}
        self._cache: Dict[tuple, Optional[Path]] = {}
        self.plans = 0                           # A* 호출 수 (기억 적중 제외)
        self.plan_fail = 0

    # ------------------------------------------------------------------ 좌표
    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return (int(math.floor((x - self.ox) / self.res)),
                int(math.floor((y - self.oy) / self.res)))

    def clearance_at(self, x: float, y: float) -> float:
        """한 점의 정적 여유 거리 [m] (지도 밖 0)."""
        cx, cy = self.world_to_cell(x, y)
        if 0 <= cx < self.w and 0 <= cy < self.h:
            return float(self.clearance[cy, cx])
        return 0.0

    # ------------------------------------------------------------------ 코스트맵 판
    def variant(self, robot: Optional[str]) -> np.ndarray:
        """로봇 robot 의 계획 코스트맵: 다른 로봇들의 대기 슬롯에 차체를 세워 팽창한 판 (None = 정적만)."""
        key = robot or ''
        g = self._variants.get(key)
        if g is not None:
            return g
        if robot is None:
            g = self.base
        else:
            cells = self.static.copy()
            for r, (x, y, th) in self.layout.slots.items():
                if r != robot:
                    self._paint_rect(cells, x, y, th, LETHAL)
            g = _core.inflate(cells, self.res, INSCRIBED_R, INFLATION_R, COST_SCALING)
        self._variants[key] = g
        return g

    def _paint_rect(self, cells: np.ndarray, x: float, y: float, th: float, val: int,
                    hl: float = 0.30, hw: float = 0.20) -> None:
        corners = np.array(rect_corners(x, y, th, hl, hw))
        x0, y0 = corners.min(axis=0)
        x1, y1 = corners.max(axis=0)
        cx0, cy0 = self.world_to_cell(x0, y0)
        cx1, cy1 = self.world_to_cell(x1, y1)
        cx0, cy0 = max(cx0, 0), max(cy0, 0)
        cx1, cy1 = min(cx1, self.w - 1), min(cy1, self.h - 1)
        if cx1 < cx0 or cy1 < cy0:
            return
        gx = self.ox + (np.arange(cx0, cx1 + 1) + 0.5) * self.res
        gy = self.oy + (np.arange(cy0, cy1 + 1) + 0.5) * self.res
        X, Y = np.meshgrid(gx, gy)
        c, s = math.cos(th), math.sin(th)
        u = (X - x) * c + (Y - y) * s
        v = -(X - x) * s + (Y - y) * c
        inside = (np.abs(u) <= hl) & (np.abs(v) <= hw)
        sub = cells[cy0:cy1 + 1, cx0:cx1 + 1]
        sub[inside] = val

    def with_obstacles(self, grid: np.ndarray, discs: Sequence[Tuple[float, float, float]] = (),
                       lethal_mask: Optional[np.ndarray] = None,
                       keep: Sequence[Tuple[float, float, float]] = ()) -> np.ndarray:
        """
        판 사본에 원(중심, 반경) · keepout 마스크(같은 격자 bool)를 LETHAL 로 칠한다.

        keep 의 원 안은 원래 판 값으로 되돌린다 (계획하는 로봇 자기 몸 — 시작 칸이 LETHAL 이면 계획이 실패한다,
        traffic_resolution.build_keepout_mask 와 같은 규칙).
        """
        g = grid.copy()
        if lethal_mask is not None:
            g[lethal_mask] = LETHAL
        for x, y, r in discs:
            win = self._disc(x, y, r)
            if win is not None:
                sl, inside = win
                g[sl][inside] = LETHAL
        for x, y, r in keep:
            win = self._disc(x, y, r)
            if win is not None:
                sl, inside = win
                g[sl][inside] = grid[sl][inside]
        return g

    def _disc(self, x: float, y: float, r: float):
        cx0, cy0 = self.world_to_cell(x - r, y - r)
        cx1, cy1 = self.world_to_cell(x + r, y + r)
        cx0, cy0 = max(cx0, 0), max(cy0, 0)
        cx1, cy1 = min(cx1, self.w - 1), min(cy1, self.h - 1)
        if cx1 < cx0 or cy1 < cy0:
            return None
        gx = self.ox + (np.arange(cx0, cx1 + 1) + 0.5) * self.res
        gy = self.oy + (np.arange(cy0, cy1 + 1) + 0.5) * self.res
        X, Y = np.meshgrid(gx, gy)
        return (slice(cy0, cy1 + 1), slice(cx0, cx1 + 1)), (X - x) ** 2 + (Y - y) ** 2 <= r * r

    # ------------------------------------------------------------------ 경로
    def plan(self, start: Tuple[float, float], goal: Pose, robot: Optional[str] = None,
             grid: Optional[np.ndarray] = None, cache: bool = True) -> Optional['NavPath']:
        """시작점 → 목표 (x, y, yaw) 팀 A* 경로 (실패 None). grid 를 주면 그 판으로 (기억 안 함)."""
        key = None
        if grid is None and cache:
            key = (robot or '', round(start[0], 2), round(start[1], 2), round(goal[0], 3),
                   round(goal[1], 3), round(goal[2], 3))
            if key in self._cache:
                return self._cache[key]
        g = grid if grid is not None else self.variant(robot)
        self.plans += 1
        st, poses, _cost, _exp = _core.plan(g, self.res, self.ox, self.oy, float(start[0]),
                                            float(start[1]), float(goal[0]), float(goal[1]),
                                            float(goal[2]), COST_WEIGHT, GOAL_TOL, True, 252,
                                            True)
        out: Optional[NavPath] = None
        if st == 'success' and len(poses) >= 1:
            pts = [(float(start[0]), float(start[1]))] + [(float(p[0]), float(p[1]))
                                                          for p in poses]
            out = NavPath(pts, goal[2])
        else:
            self.plan_fail += 1
        if key is not None:
            self._cache[key] = out
        return out


class NavPath(Path):
    """팀 A* 경로 + 곡률 속도 상한 프로파일 + TrafficManager 용 (N, 2) 배열."""

    def __init__(self, points: Sequence[Tuple[float, float]], goal_yaw: float = 0.0):
        super().__init__(points)
        self.goal_yaw = float(goal_yaw)
        self.xy = np.asarray(self.points, dtype=float)
        self.cum_arr = np.asarray(self.cum, dtype=float)
        self._vlim: Dict[float, np.ndarray] = {}

    def vlim(self, omega: float = OMEGA_MAX) -> np.ndarray:
        v = self._vlim.get(omega)
        if v is None:
            v = self._vlim[omega] = speed_profile(self.xy, self.cum_arr, omega)
        return v

    def v_at(self, s: float, omega: float = OMEGA_MAX) -> float:
        """호 길이 s 의 곡률 · 감속 속도 상한 (경로 끝은 0 이 아니다 — 끝 정지는 별도)."""
        return float(np.interp(s, self.cum_arr, self.vlim(omega)))


def speed_profile(xy: np.ndarray, cum: np.ndarray, omega: float = OMEGA_MAX) -> np.ndarray:
    """
    DWA 근사 속도 상한: 곡률 κ 에서 v ≤ ω/κ, 앞쪽 상한까지 감속 a 로 미리 줄인다 (뒤로 훑기).

    κ = 창(CURV_WINDOW_M) 앞뒤 방향 차 / 창 길이. 끝점 정지는 운동학의 정지 목표가 맡는다.
    ω 는 곡선 주행 각속도 상한 (기본 = 회전 한계 1.5 rad/s, 보정값은 SimConfig.omega_curve — sim.md §보정).
    """
    n = len(xy)
    v = np.full(n, V_MAX)
    if n < 3:
        return v
    seg = np.diff(xy, axis=0)
    head = np.arctan2(seg[:, 1], seg[:, 0])
    hs = np.concatenate([[head[0]], head])          # 점 i 로 들어오는 방향
    lo = np.searchsorted(cum, cum - CURV_WINDOW_M / 2.0)
    hi = np.minimum(n - 1, np.searchsorted(cum, cum + CURV_WINDOW_M / 2.0))
    d = np.abs(np.arctan2(np.sin(hs[hi] - hs[lo]), np.cos(hs[hi] - hs[lo])))
    span = np.maximum(cum[hi] - cum[lo], 1e-6)
    k = np.where(hi > lo, d / span, 0.0)
    curved = k > 1e-6
    v[curved] = np.minimum(V_MAX, np.maximum(V_MIN_CURVE, omega / k[curved]))
    for i in range(n - 2, -1, -1):                   # 앞 상한까지 감속 a 로
        ds = cum[i + 1] - cum[i]
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * ACC * ds))
    return v


def path_length(xy: Iterable[Tuple[float, float]]) -> float:
    pts = list(xy)
    return float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])))


__all__ = ['ACC', 'COST_WEIGHT', 'INFLATION_R', 'LETHAL', 'NavMap', 'NavPath', 'OMEGA_MAX',
           'V_MAX', 'core_available', 'path_length', 'speed_profile']
