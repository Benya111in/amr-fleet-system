"""
모의 세계: 저장소 지도 → 격자 경로, 교통 구역, 도크·스폰, 경로 위 구역 방문과 정지선.

- 지도: maps/warehouse.yaml (+ PGM). 점유 화소(p > occupied_thresh)와 미지 화소를 장애물로 보고 거리 변환으로
  여유 거리를 만든다. 경로 격자(기본 0.2 m)는 칸 중심 여유가 plan_clearance_m 이상인 칸만 지난다.
- 경로: 8-이웃 격자 Dijkstra(scipy.sparse.csgraph) → 칸 중심 열 → 시선 당기기(여유 los_clearance_m 이상인
  직선이면 꼭짓점을 건너뛴다). 끝점이 막힌 칸이면 가장 가까운 빈 칸을 거친다. 재계획은 없다 (Nav2 대신).
- 구역: config/traffic_zones.yaml 의 축 정렬 사각형 (traffic_zones.load_layout · ZoneMap 을 그대로 쓴다).
  요청 거리 = traffic_zones.approach_map (traffic.yaml 의 approach_distance_m 2.0 ·
  corridor_approach_m 2.5, 항목의 approach_m 우선), 사슬 간격 chain_gap_m 1.5.
- 도크·스폰: negotiation.scenario 의 DOCKS(behavior.yaml staging) · SPAWNS(fleet_spawn.yaml) 를
  import 한다.

경로 위 구역 방문(LegZones)은 경로마다 한 번 계산해 기억한다:
- 중심 방문 ZoneVisit(s_in, s_out) — 토큰 요청(request_chain)에 쓰는 중앙 교통 관리자와 같은 정의
- 차체 접촉 구간 [s_touch, s_clear] — 차체 외접원(+여유)이 구역 상자에 닿는 호 길이 구간. 정지선 = s_touch − 여유,
  "몸체가 완전히 나왔다" = s > s_clear. 외접원은 어떤 방향의 차체도 덮으므로 정지선에 선 차체는 구역에 닿지 않는다.
"""

from __future__ import annotations

import dataclasses
import math
import pathlib
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

from amr_fleet.fleetsim.geometry import Box, box_of_polygon, CIRCUMRADIUS, Path
from amr_fleet.negotiation.grid import _read_map_yaml, read_pgm
from amr_fleet.negotiation.scenario import DOCKS, SPAWNS
from amr_fleet.traffic_zones import (
    approach_map, load_layout, Zone, zone_visits, ZoneMap, ZoneVisit,
)

_PKG = pathlib.Path(__file__).resolve().parents[2]          # src/amr_fleet
_REPO = pathlib.Path(__file__).resolve().parents[4]         # 저장소 뿌리
DEFAULT_MAP = _REPO / 'maps' / 'warehouse.yaml'
DEFAULT_ZONES = _PKG / 'config' / 'traffic_zones.yaml'

APPROACH_M = 2.0            # traffic.yaml zones.approach_distance_m
CORRIDOR_APPROACH_M = 2.5   # traffic.yaml zones.corridor_approach_m
CHAIN_GAP_M = 1.5           # traffic.yaml zones.chain_gap_m
TOUCH_MARGIN_M = 0.02       # 외접원에 더하는 여유 (접촉 판정)
STOP_MARGIN_M = 0.05        # 정지선 = 접촉 시작 − 이 값
SCAN_STEP_M = 0.05          # 접촉 구간 훑기 간격


@dataclasses.dataclass(frozen=True)
class ZonePass:
    """경로 하나가 구역 하나를 지나는 방문 (중심 방문 + 차체 접촉 구간)."""

    visit: ZoneVisit            # 중심점 기준 (s_in, s_out) — request_chain 용
    s_touch: float              # 차체(외접원)가 처음 닿는 호 길이 (0 이면 시작부터 닿아 있음)
    s_clear: float              # 차체가 마지막으로 닿는 호 길이 (이보다 크면 완전히 나옴)

    @property
    def zone_id(self) -> str:
        return self.visit.zone_id

    @property
    def stop_s(self) -> float:
        """허가가 없을 때 차체 중심이 넘지 않는 호 길이."""
        return max(0.0, self.s_touch - STOP_MARGIN_M)


@dataclasses.dataclass
class Leg:
    """한 구간(출발 이름 → 도착 이름)의 경로와 구역 방문."""

    start: str
    goal: str
    path: Path
    passes: Tuple[ZonePass, ...]

    def visits_from(self, s: float) -> List[ZoneVisit]:
        """현재 호 길이 s 기준으로 옮긴 중심 방문 (request_chain 입력, 지난 방문은 뺀다)."""
        out = []
        for p in self.passes:
            v = p.visit
            if v.s_out - s < -1e-9:
                continue
            out.append(ZoneVisit(v.zone_id, v.s_in - s, v.s_out - s, v.entry, v.exit))
        return out


class World:
    """지도·구역·도크·스폰과 경로 기억 (프로세스 안에서 한 번 만들고 실행끼리 공유한다)."""

    def __init__(self, map_yaml: Optional[str] = None, zones_yaml: Optional[str] = None,
                 cell_m: float = 0.2, plan_clearance_m: float = 0.42,
                 los_clearance_m: float = 0.40):
        self.map_yaml = str(map_yaml or DEFAULT_MAP)
        self.zones_yaml = str(zones_yaml or DEFAULT_ZONES)
        self.cell_m = float(cell_m)
        self.plan_clearance_m = float(plan_clearance_m)
        self.los_clearance_m = float(los_clearance_m)
        self._load_map()
        zones, _ = load_layout(self.zones_yaml, corridor_same_direction=True)
        self.zones: List[Zone] = zones
        self.zone_map = ZoneMap(zones)
        self.boxes: Dict[str, Box] = {z.zone_id: box_of_polygon([tuple(p) for p in z.polygon])
                                      for z in zones}
        self.same_direction: Dict[str, bool] = {z.zone_id: bool(z.same_direction) for z in zones}
        self.approach: Dict[str, float] = approach_map(zones, APPROACH_M, CORRIDOR_APPROACH_M)
        self.chain_gap_m = CHAIN_GAP_M
        self.places: Dict[str, Tuple[float, float]] = dict(DOCKS)
        for rid, x, y in SPAWNS:
            self.places[f'spawn_{rid}'] = (float(x), float(y))
        self.docks: Tuple[str, ...] = tuple(sorted(DOCKS))
        self.robot_ids: Tuple[str, ...] = tuple(r for r, _, _ in SPAWNS)
        self._legs: Dict[Tuple[str, str], Leg] = {}
        self._trees: Dict[int, np.ndarray] = {}

    # ------------------------------------------------------------------ 지도
    def _load_map(self) -> None:
        meta = _read_map_yaml(self.map_yaml)
        image = pathlib.Path(meta['image'])
        if not image.is_absolute():
            image = pathlib.Path(self.map_yaml).resolve().parent / image
        w, h, pix = read_pgm(str(image))
        img = np.frombuffer(pix, dtype=np.uint8).reshape(h, w)
        res = float(meta['resolution'])
        negate = int(meta.get('negate', 0))
        occ_t = float(meta.get('occupied_thresh', 0.65))
        free_t = float(meta.get('free_thresh', 0.196))
        p = img / 255.0 if negate else (255.0 - img) / 255.0
        free = p < free_t                                   # 점유·미지 = 장애물
        del occ_t
        free_up = free[::-1, :]                             # 행 0 = 지도 아래(+y 시작)
        self.res = res
        self.origin = (float(meta['origin'][0]), float(meta['origin'][1]))
        self.pix_w, self.pix_h = w, h
        self.clearance = ndimage.distance_transform_edt(free_up) * res   # [m], 행 = y
        f = max(1, int(round(self.cell_m / res)))
        self._f = f
        self.gw, self.gh = w // f, h // f
        cy = np.arange(self.gh) * f + f // 2
        cx = np.arange(self.gw) * f + f // 2
        cell_clear = self.clearance[np.ix_(cy, cx)]
        self.cell_free = cell_clear >= self.plan_clearance_m       # (gh, gw)
        self._build_graph()

    def _build_graph(self) -> None:
        gw, gh = self.gw, self.gh
        free = self.cell_free
        idx = np.arange(gw * gh).reshape(gh, gw)
        rows, cols, data = [], [], []
        step = self.cell_m
        for dy, dx, c in ((0, 1, step), (1, 0, step), (1, 1, step * math.sqrt(2)),
                          (1, -1, step * math.sqrt(2))):
            ys = slice(0, gh - dy)
            yd = slice(dy, gh)
            if dx >= 0:
                xs, xd = slice(0, gw - dx), slice(dx, gw)
            else:
                xs, xd = slice(-dx, gw), slice(0, gw + dx)
            ok = free[ys, xs] & free[yd, xd]
            if dx != 0 and dy != 0:                     # 모서리 자르기 금지
                ok &= free[ys.start:ys.stop, xd] & free[yd, xs]
            a = idx[ys, xs][ok]
            b = idx[yd, xd][ok]
            rows.append(a)
            cols.append(b)
            data.append(np.full(a.shape, c))
        r = np.concatenate(rows)
        c = np.concatenate(cols)
        d = np.concatenate(data)
        n = gw * gh
        self._graph = coo_matrix((d, (r, c)), shape=(n, n)).tocsr()

    def cell_of(self, x: float, y: float) -> Tuple[int, int]:
        """월드 좌표 → (열, 행)."""
        return (int(math.floor((x - self.origin[0]) / self.cell_m)),
                int(math.floor((y - self.origin[1]) / self.cell_m)))

    def cell_center(self, cx: int, cy: int) -> Tuple[float, float]:
        """칸 중심 월드 좌표 (거리 변환에 쓴 화소와 같은 점)."""
        f = self._f
        return (self.origin[0] + (cx * f + f // 2 + 0.5) * self.res,
                self.origin[1] + (cy * f + f // 2 + 0.5) * self.res)

    def clearance_at(self, xy: np.ndarray) -> np.ndarray:
        """점 배열 (N, 2) 의 여유 거리 [m] (지도 밖은 0)."""
        px = np.floor((xy[:, 0] - self.origin[0]) / self.res).astype(int)
        py = np.floor((xy[:, 1] - self.origin[1]) / self.res).astype(int)
        inside = (px >= 0) & (px < self.pix_w) & (py >= 0) & (py < self.pix_h)
        out = np.zeros(len(xy))
        out[inside] = self.clearance[py[inside], px[inside]]
        return out

    def _nearest_free_cell(self, x: float, y: float, max_r: int = 40) -> Tuple[int, int]:
        cx, cy = self.cell_of(x, y)
        best, best_d = None, math.inf
        for r in range(0, max_r + 1):
            for dx in range(-r, r + 1):
                for dy in ((-r, r) if abs(dx) != r else range(-r, r + 1)):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < self.gw and 0 <= ny < self.gh and self.cell_free[ny, nx]:
                        px, py = self.cell_center(nx, ny)
                        d = math.hypot(px - x, py - y)
                        if d < best_d:
                            best, best_d = (nx, ny), d
            if best is not None and best_d <= r * self.cell_m:
                return best
        if best is None:
            raise ValueError(f'({x:.2f}, {y:.2f}) 근처에 빈 칸이 없다')
        return best

    def _los(self, a: Tuple[float, float], b: Tuple[float, float]) -> bool:
        n = max(2, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / (self.res))) + 1)
        t = np.linspace(0.0, 1.0, n)
        xy = np.stack([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])], axis=1)
        return bool(np.all(self.clearance_at(xy) >= self.los_clearance_m))

    # ------------------------------------------------------------------ 경로
    def plan(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Path:
        """시작점 → 목표점 꺾은선 경로 (격자 Dijkstra + 시선 당기기)."""
        if math.hypot(goal[0] - start[0], goal[1] - start[1]) < 1e-9:
            return Path([start])
        s_cell = self._nearest_free_cell(*start)
        g_cell = self._nearest_free_cell(*goal)
        s_idx = s_cell[1] * self.gw + s_cell[0]
        g_idx = g_cell[1] * self.gw + g_cell[0]
        pred = self._trees.get(s_idx)
        if pred is None:
            _, pred = dijkstra(self._graph, directed=False, indices=s_idx,
                               return_predecessors=True)
            self._trees[s_idx] = pred
        if g_idx != s_idx and pred[g_idx] < 0:
            raise ValueError(f'경로 없음: {start} → {goal}')
        cells = [g_idx]
        while cells[-1] != s_idx:
            cells.append(int(pred[cells[-1]]))
        cells.reverse()
        pts = ([tuple(start)] + [self.cell_center(i % self.gw, i // self.gw) for i in cells]
               + [tuple(goal)])
        # 시선 당기기: 다음 점은 늘 받아들이고(끝점 ↔ 붙은 칸은 장애물 가까이일 수 있다), 그 너머는 여유가
        # los_clearance_m 이상인 직선일 때만 건너뛴다
        pulled = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = i + 1
            while j + 1 < len(pts) and self._los(pts[i], pts[j + 1]):
                j += 1
            pulled.append(pts[j])
            i = j
        return Path(pulled)

    def leg(self, start: str, goal: str) -> Leg:
        """이름(도크 id · 'spawn_<robot>') 사이 구간 (기억)."""
        key = (start, goal)
        leg = self._legs.get(key)
        if leg is None:
            path = self.plan(self.places[start], self.places[goal])
            leg = Leg(start, goal, path, self.zone_passes(path))
            self._legs[key] = leg
        return leg

    def zone_passes(self, path: Path) -> Tuple[ZonePass, ...]:
        """
        경로 위 구역 방문과 차체 접촉 구간 (s_touch 순).

        차체 접촉 구간마다 방문 하나. 그 구간 안에 중심 방문(zone_visits)이 있으면 그것을 요청 기준으로 쓰고,
        중심은 들어가지 않고 차체만 스치는 구간이면 접촉 구간으로 만든 방문을 쓴다 (스침도 토큰 없이는 안 된다).
        """
        if path.length <= 0.0:
            visits: List[ZoneVisit] = []
            ss = np.zeros(1)
        else:
            visits = zone_visits(np.asarray(path.points, dtype=float), self.zone_map, step=0.2)
            n = max(2, int(math.ceil(path.length / SCAN_STEP_M)) + 1)
            ss = np.linspace(0.0, path.length, n)
        xy = np.array([path.point(float(s)) for s in ss])
        r = CIRCUMRADIUS + TOUCH_MARGIN_M
        out: List[ZonePass] = []
        for zid, b in self.boxes.items():
            dx = np.maximum(np.maximum(b.xmin - xy[:, 0], 0.0), xy[:, 0] - b.xmax)
            dy = np.maximum(np.maximum(b.ymin - xy[:, 1], 0.0), xy[:, 1] - b.ymax)
            touch = np.hypot(dx, dy) < r
            if not touch.any():
                continue
            k = 0
            while k < len(ss):
                if not touch[k]:
                    k += 1
                    continue
                lo = k
                while k + 1 < len(ss) and touch[k + 1]:
                    k += 1
                hi = k
                k += 1
                s_touch = 0.0 if lo == 0 else max(0.0, float(ss[lo]) - SCAN_STEP_M)
                s_clear = (path.length if hi == len(ss) - 1
                           else min(path.length, float(ss[hi]) + SCAN_STEP_M))
                inner = [v for v in visits if v.zone_id == zid
                         and s_touch - 1e-9 <= v.s_in <= s_clear + 1e-9]
                if inner:
                    v0, v1 = inner[0], inner[-1]
                    visit = ZoneVisit(zid, v0.s_in, v1.s_out, v0.entry, v1.exit)
                else:
                    p0, p1 = path.point(s_touch), path.point(s_clear)
                    visit = ZoneVisit(zid, s_touch, s_clear, p0, p1)
                out.append(ZonePass(visit, s_touch, s_clear))
        out.sort(key=lambda p: (p.s_touch, p.zone_id))
        return tuple(out)

    # ------------------------------------------------------------------ 조회
    def body_zones(self, x: float, y: float, th: float) -> List[str]:
        """실제 차체(방향 사각형)가 겹치는 구역 (정의 순)."""
        from amr_fleet.fleetsim.geometry import rect_box_overlap
        return [z for z, b in self.boxes.items() if rect_box_overlap(x, y, th, b)]

    def circle_zones(self, x: float, y: float, r: float = CIRCUMRADIUS) -> List[str]:
        """외접원이 겹치는 구역 (중앙 관리자의 몸체 판정 — 방향을 모를 때)."""
        return [z for z, b in self.boxes.items() if b.distance(x, y) < r]

    def directions(self, leg: Leg) -> Mapping[str, Optional[Tuple[float, float]]]:
        """구간의 구역별 통과 방향 (같은 방향 추종 판정용)."""
        return {p.zone_id: p.visit.direction() for p in leg.passes}


_WORLD: Optional[World] = None


def default_world() -> World:
    """프로세스 공유 기본 세계 (처음 부를 때 만든다)."""
    global _WORLD
    if _WORLD is None:
        _WORLD = World()
    return _WORLD


def leg_names(world: World, sources: Sequence[str]) -> List[Tuple[str, str]]:
    """미리 계산할 구간 목록 (출발 × 도크)."""
    return [(s, d) for s in sources for d in world.docks if s != d]


__all__ = ['APPROACH_M', 'CHAIN_GAP_M', 'CORRIDOR_APPROACH_M', 'DEFAULT_MAP', 'DEFAULT_ZONES',
           'Leg', 'STOP_MARGIN_M', 'World', 'ZonePass', 'default_world', 'leg_names']
