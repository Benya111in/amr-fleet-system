"""
교차로 · 병목(1차선 통로) 구역과 진입 토큰 (명세 4.9 "교차로/병목 구간에서의 우선순위 결정 로직").

구역(zone) 출처 — zones.source 파라미터
- yaml : config/traffic_zones.yaml 의 zones 다각형 (창고 배치를 알 때. 기본)
- map  : /map 에서 자동 유도 (derive_zones_from_grid)
    · 좁은 통로: 벽까지 거리(EDT)의 국소 최대(능선) × 2 = 통로 폭 < narrow_width_m 인 통과 가능 셀
      (통과 가능 = 벽까지 ≥ passage_radius, 풋프린트 반폭 + 여유). 연결 성분마다 corridor 구역
    · 교차로  : 네 축 방향 통과 가능 연속 길이 중 intersection_arm_m 이상이 3개 이상인 셀
      (T자·십자). 넓은 광장(면적 > intersection_max_area_m2)은 교차로로 보지 않는다.
      축 정렬 통로를 가정한다 — 기울어진 배치는 yaml 로 준다.
- both : 둘 다 (겹치면 yaml 우선)

진입 토큰 — 모든 구역의 용량은 1대 (corridor 는 같은 방향 추종만 여러 대 허용 가능)
- 로봇이 구역 진입점 approach_distance_m 앞에 오면 토큰을 요청한다. 구역 출구와 다음 구역 입구가
  chain_gap_m 이하로 붙어 있으면(교차로 → 통로 등) 묶어서 한 번에 받는다 ("교차로 안에서 서지 않기").
- 부여 순서 = (기아 상태 먼저) → 작업 우선순위 높은 순 → 마감 이른 순 → 오래 기다린 순 → robot_id.
- 기아 방지: 먼저 요청한 로봇이 (엄격히) 나중 요청 로봇에게 max_bypass 번 추월당하면 "기아" 로
  표시되어 그 구역은 기아 로봇에게 예약된다 (기아 로봇끼리는 요청 순서 → 우선순위). 같은 주기에 함께
  요청한 로봇끼리는 우선순위 순서가 추월이 아니다. 따라서 한 로봇보다 먼저 부여받는 다른 로봇 수는
  max_bypass + (n - 1) 이하 — 대기 상한 = 그 수 × 구역 최대 점유 시간.
- 반납: 구역에 들어갔다가 몸체가 완전히 나오면, 또는 경로가 더는 그 구역을 지나지 않으면.
"""

from __future__ import annotations

import collections
import dataclasses
import math
import re
from typing import Any, Deque, Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

import numpy as np
from scipy import ndimage
import yaml

from amr_fleet.traffic_geometry import (
    GridSpec, cumulative_length, points_at, points_in_polygon, polygon_area,
)

INTERSECTION = 'intersection'
CORRIDOR = 'corridor'
ZONE_KINDS = (INTERSECTION, CORRIDOR)
ZONE_ID_RE = re.compile(r'^[A-Za-z0-9_.-]{1,64}$')


@dataclasses.dataclass
class Zone:
    """구역 1개. polygon 이 있으면 yaml, 없으면 지도 유도(라벨 격자) 구역."""

    zone_id: str
    kind: str
    polygon: Optional[np.ndarray] = None
    same_direction: bool = False      # corridor: 같은 방향 추종 진입 허용
    area_m2: float = 0.0
    centroid: Tuple[float, float] = (0.0, 0.0)
    source: str = 'yaml'


def _finite_pair(p: Any) -> Tuple[float, float]:
    if not isinstance(p, (list, tuple)) or len(p) != 2:
        raise ValueError(f'꼭짓점은 [x, y] 여야 한다: {p!r}')
    x, y = float(p[0]), float(p[1])
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError(f'꼭짓점 좌표가 유한하지 않다: {p!r}')
    return x, y


def zones_from_config(entries: Any, corridor_same_direction: bool = False) -> List[Zone]:
    """
    traffic_zones.yaml 의 zones 목록 → Zone 목록.

    항목 = {id, kind: intersection|corridor, polygon: [[x, y], ...] (꼭짓점 ≥ 3, map 프레임 m),
    same_direction: bool (선택, corridor 만 의미 — 없으면 corridor_same_direction)}.
    형식 오류·중복 id·넓이 0 은 항목 번호와 함께 ValueError.
    """
    if entries is None:
        return []
    if not isinstance(entries, (list, tuple)):
        raise ValueError('zones 는 목록이어야 한다')
    zones: List[Zone] = []
    seen: Set[str] = set()
    for k, e in enumerate(entries):
        where = f'zones[{k}]'
        if not isinstance(e, Mapping):
            raise ValueError(f'{where}: {{id, kind, polygon}} 사전이어야 한다')
        unknown = set(e) - {'id', 'kind', 'polygon', 'same_direction'}
        if unknown:
            raise ValueError(f'{where}: 모르는 키 {sorted(unknown)}')
        zone_id = str(e.get('id', ''))
        if not ZONE_ID_RE.match(zone_id):
            raise ValueError(f'{where}: id 는 [A-Za-z0-9_.-]{{1,64}}: {zone_id!r}')
        if zone_id in seen:
            raise ValueError(f'{where}: 중복 id {zone_id!r}')
        kind = str(e.get('kind', '')).lower()
        if kind not in ZONE_KINDS:
            raise ValueError(f'{where}: kind 는 {ZONE_KINDS} 중 하나: {kind!r}')
        poly = e.get('polygon')
        if not isinstance(poly, (list, tuple)) or len(poly) < 3:
            raise ValueError(f'{where}: polygon 은 꼭짓점 3개 이상')
        try:
            pts = np.array([_finite_pair(p) for p in poly], dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError(f'{where}: {exc}') from exc
        area = polygon_area(pts)
        if area <= 1e-6:
            raise ValueError(f'{where}: 넓이가 0 인 다각형')
        same = e.get('same_direction', corridor_same_direction)
        if not isinstance(same, bool):
            raise ValueError(f'{where}: same_direction 은 true/false')
        seen.add(zone_id)
        zones.append(Zone(zone_id, kind, pts, bool(same) and kind == CORRIDOR, area,
                          (float(pts[:, 0].mean()), float(pts[:, 1].mean())), 'yaml'))
    return zones


def load_layout(path: str, corridor_same_direction: bool = False
                ) -> Tuple[List[Zone], List[Any]]:
    """
    교통 배치 파일(config/traffic_zones.yaml) → (구역, 포켓 항목 원본).

    최상위 키: frame_id (map 만), zones, pockets. 포켓 해석은 traffic_resolution.pockets_from_config.
    파일이 없으면 OSError, 형식 오류는 ValueError.
    """
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if data is None:
        data = {}
    if not isinstance(data, Mapping):
        raise ValueError(f'{path}: 최상위는 사전이어야 한다')
    unknown = set(data) - {'frame_id', 'zones', 'pockets'}
    if unknown:
        raise ValueError(f'{path}: 모르는 최상위 키 {sorted(unknown)}')
    frame = str(data.get('frame_id', 'map'))
    if frame != 'map':
        raise ValueError(f'{path}: frame_id 는 map 만 지원한다 ({frame!r})')
    zones = zones_from_config(data.get('zones'), corridor_same_direction)
    pockets = data.get('pockets') or []
    if not isinstance(pockets, (list, tuple)):
        raise ValueError(f'{path}: pockets 는 목록이어야 한다')
    return zones, list(pockets)


class ZoneMap:
    """점 → 구역 조회 (yaml 다각형 우선, 그다음 지도 유도 라벨 격자)."""

    def __init__(self, polygon_zones: Sequence[Zone] = (), grid_zones: Sequence[Zone] = (),
                 labels: Optional[np.ndarray] = None, spec: Optional[GridSpec] = None):
        self._zones: Dict[str, Zone] = {}
        for z in list(polygon_zones) + list(grid_zones):
            if z.zone_id in self._zones:
                raise ValueError(f'중복 zone id {z.zone_id!r}')
            self._zones[z.zone_id] = z
        self._polys = list(polygon_zones)
        self._labels = labels
        self._spec = spec
        self._label_ids: List[Optional[str]] = [None] + [z.zone_id for z in grid_zones]
        if labels is not None and spec is None:
            raise ValueError('labels 에는 spec 이 필요하다')

    def __len__(self) -> int:
        return len(self._zones)

    def __contains__(self, zone_id: str) -> bool:
        return zone_id in self._zones

    def ids(self) -> List[str]:
        """구역 id 목록 (정의 순)."""
        return list(self._zones)

    def get(self, zone_id: str) -> Zone:
        """구역 조회 (없으면 KeyError)."""
        return self._zones[zone_id]

    def zones(self) -> List[Zone]:
        """구역 목록 (정의 순)."""
        return list(self._zones.values())

    def zones_at(self, xy: np.ndarray) -> List[Optional[str]]:
        """점마다 속한 구역 id (없으면 None)."""
        pts = np.asarray(xy, dtype=float).reshape(-1, 2)
        out: List[Optional[str]] = [None] * len(pts)
        if self._labels is not None and len(self._label_ids) > 1:
            vals = self._spec.lookup(self._labels, pts, 0)
            for k, v in enumerate(vals):
                out[k] = self._label_ids[int(v)]
        for z in reversed(self._polys):        # 먼저 정의한 다각형이 이긴다
            inside = points_in_polygon(pts, z.polygon)
            for k in np.nonzero(inside)[0]:
                out[int(k)] = z.zone_id
        return out

    def zone_at(self, x: float, y: float) -> Optional[str]:
        """한 점의 구역."""
        return self.zones_at(np.array([[x, y]]))[0]

    def mask(self, spec: GridSpec, zone_ids: Optional[Iterable[str]] = None) -> np.ndarray:
        """격자 셀 중심이 (지정한 / 모든) 구역에 드는지 (height, width) bool."""
        gx, gy = spec.centers()
        ids = self.zones_at(np.stack([gx.ravel(), gy.ravel()], axis=1))
        wanted = None if zone_ids is None else set(zone_ids)
        flat = np.array([z is not None and (wanted is None or z in wanted) for z in ids],
                        dtype=bool)
        return flat.reshape(spec.height, spec.width)


# ---------------------------------------------------------------------- 지도 유도
def _axis_runs(trav: np.ndarray) -> List[np.ndarray]:
    """셀마다 +x, -x, +y, -y 방향 연속 주행 가능 셀 수 (자기 포함)."""
    t = trav.astype(np.int32)
    h, w = t.shape
    east, west = np.zeros_like(t), np.zeros_like(t)
    north, south = np.zeros_like(t), np.zeros_like(t)
    east[:, w - 1], west[:, 0] = t[:, w - 1], t[:, 0]
    for j in range(w - 2, -1, -1):
        east[:, j] = (east[:, j + 1] + 1) * t[:, j]
    for j in range(1, w):
        west[:, j] = (west[:, j - 1] + 1) * t[:, j]
    north[h - 1, :], south[0, :] = t[h - 1, :], t[0, :]
    for i in range(h - 2, -1, -1):
        north[i, :] = (north[i + 1, :] + 1) * t[i, :]
    for i in range(1, h):
        south[i, :] = (south[i - 1, :] + 1) * t[i, :]
    return [east, west, north, south]


def clearance_map(occupied: np.ndarray, resolution: float) -> np.ndarray:
    """셀 중심에서 가장 가까운 점유 셀 가장자리까지 거리 [m] (점유 셀은 음수)."""
    return ndimage.distance_transform_edt(~occupied) * resolution - 0.5 * resolution


def derive_zones_from_grid(occupied: np.ndarray, spec: GridSpec, passage_radius: float = 0.22,
                           narrow_width_m: float = 1.5, min_corridor_area_m2: float = 0.2,
                           intersection_arm_m: float = 4.0,
                           intersection_min_area_m2: float = 1.0,
                           intersection_max_area_m2: float = 36.0, margin_m: float = 0.3,
                           corridor_same_direction: bool = False
                           ) -> Tuple[np.ndarray, List[Zone]]:
    """
    점유 격자 → (라벨 격자 int32, 구역 목록). 라벨 k(≥1) = 구역 목록의 k-1 번째.

    occupied: (height, width) bool, 미지(-1) 는 점유로 넘겨야 한다. passage_radius 는 로봇 중심이
    지날 수 있는 벽까지 최소 거리 (풋프린트 반폭 0.2 m + 여유) — 외접 반경(0.36 m)을 쓰면 폭 0.6 m
    통로가 통과 불가로 사라진다.
    """
    res = spec.resolution
    free = ~occupied
    clear = clearance_map(occupied, res)
    trav = free & (clear >= passage_radius)
    labels = np.zeros(occupied.shape, dtype=np.int32)
    zones: List[Zone] = []
    structure = np.ones((3, 3), dtype=bool)
    gx, gy = spec.centers()

    def add(mask: np.ndarray, kind: str, prefix: str, lo: float, hi: float,
            grow_m: float) -> None:
        comp, n = ndimage.label(mask, structure=structure)
        for k in range(1, n + 1):
            cells = comp == k
            area = float(cells.sum()) * res * res
            if area < lo or area > hi:
                continue
            if grow_m > 0.0:
                it = max(1, int(round(grow_m / res)))
                cells = ndimage.binary_dilation(cells, structure=structure, iterations=it) & free
            cells &= labels == 0
            if not cells.any():
                continue
            zones.append(Zone(f'{prefix}_{len(zones) + 1}', kind, None,
                              corridor_same_direction and kind == CORRIDOR,
                              float(cells.sum()) * res * res,
                              (float(gx[cells].mean()), float(gy[cells].mean())), 'map'))
            labels[cells] = len(zones)

    # 교차로 먼저 (통로와 겹치면 교차로가 이긴다)
    arm_cells = intersection_arm_m / res
    long_arms = sum((r >= arm_cells).astype(np.int32) for r in _axis_runs(trav))
    add(trav & (long_arms >= 3), INTERSECTION, 'intersection_auto',
        intersection_min_area_m2, intersection_max_area_m2, margin_m)

    reach = max(1, int(round(0.5 * narrow_width_m / res)))
    ridge = ndimage.maximum_filter(np.where(free, clear, 0.0), size=2 * reach + 1)
    narrow = trav & (2.0 * ridge < narrow_width_m)
    add(narrow, CORRIDOR, 'corridor_auto', min_corridor_area_m2, math.inf, reach * res)
    return labels, zones


# ---------------------------------------------------------------------- 경로 위 구역 방문
@dataclasses.dataclass
class ZoneVisit:
    """경로가 구역 하나를 지나는 구간 (호 길이는 경로 시작점 = 로봇 위치 기준)."""

    zone_id: str
    s_in: float
    s_out: float
    entry: Tuple[float, float]
    exit: Tuple[float, float]

    def direction(self) -> Optional[Tuple[float, float]]:
        """입구 → 출구 단위 벡터 (구간이 짧으면 None)."""
        dx, dy = self.exit[0] - self.entry[0], self.exit[1] - self.entry[1]
        n = math.hypot(dx, dy)
        return None if n < 0.2 else (dx / n, dy / n)


def zone_visits(path: np.ndarray, zone_map: ZoneMap, step: float = 0.2,
                max_s: float = math.inf) -> List[ZoneVisit]:
    """경로(첫 점 = 로봇 위치)를 step 간격으로 훑어 구역 방문 목록을 만든다."""
    if path is None or len(path) == 0 or len(zone_map) == 0:
        return []
    cum = cumulative_length(path)
    end = min(float(cum[-1]), max_s)
    s = np.arange(0.0, end + 0.5 * step, step) if end > 0.0 else np.zeros(1)
    s = np.minimum(s, end)
    xy, _ = points_at(path, cum, s)
    ids = zone_map.zones_at(xy)
    visits: List[ZoneVisit] = []
    start = None
    for k in range(len(ids) + 1):
        cur = ids[k] if k < len(ids) else None
        prev = ids[start] if start is not None else None
        if start is not None and cur != prev:
            visits.append(ZoneVisit(prev, float(s[start]), float(s[k - 1]),
                                    (float(xy[start, 0]), float(xy[start, 1])),
                                    (float(xy[k - 1, 0]), float(xy[k - 1, 1]))))
            start = None
        if start is None and cur is not None:
            start = k
    return visits


def request_chain(visits: Sequence[ZoneVisit], approach_m: float,
                  chain_gap_m: float) -> List[ZoneVisit]:
    """
    지금 요청할 구역 묶음 (같은 구역의 재방문은 한 번만).

    아직 들어가지 않은(s_in > 0) 첫 방문이 approach_m 안이면 그것부터, 다음 방문 입구가 앞 방문
    출구에서 chain_gap_m 이하면 이어 붙인다. 없으면 [].
    """
    upcoming = [v for v in visits if v.s_in > 1e-6]
    if not upcoming or upcoming[0].s_in > approach_m:
        return []
    chain = [upcoming[0]]
    for v in upcoming[1:]:
        if v.s_in - chain[-1].s_out > chain_gap_m:
            break
        chain.append(v)
    out: List[ZoneVisit] = []
    for v in chain:
        if all(v.zone_id != o.zone_id for o in out):
            out.append(v)
    return out


# ---------------------------------------------------------------------- 진입 토큰
Direction = Optional[Tuple[float, float]]


@dataclasses.dataclass
class TokenRequest:
    """로봇 1대의 토큰 요청 (묶음 구역을 한 번에)."""

    robot_id: str
    zones: Tuple[str, ...]
    directions: Dict[str, Direction] = dataclasses.field(default_factory=dict)
    priority: int = -1                 # 작업 우선순위 0~255 (작업 없음 -1)
    deadline: Optional[float] = None   # 마감 [s] (없으면 None = 가장 늦음)


@dataclasses.dataclass
class TokenDecision:
    """요청 1건의 이번 주기 결과."""

    granted: bool
    zones: Tuple[str, ...]
    blockers: Tuple[str, ...] = ()     # 대기 원인 로봇 (보유자 · 기아 예약자)
    waiting_s: float = 0.0
    bypassed: int = 0


@dataclasses.dataclass
class _Wait:
    zones: Tuple[str, ...]
    since: float
    seq: int
    bypassed: int = 0


class ZoneTokenManager:
    """구역 진입 토큰 부여 · 반납 (용량 1, corridor 는 같은 방향 추종 선택)."""

    def __init__(self, same_direction: Optional[Mapping[str, bool]] = None, max_bypass: int = 2,
                 same_direction_cos: float = 0.5):
        if max_bypass < 0:
            raise ValueError('max_bypass 는 0 이상')
        self.max_bypass = int(max_bypass)
        self.same_direction_cos = float(same_direction_cos)
        self._same_dir: Dict[str, bool] = dict(same_direction or {})
        self._holders: Dict[str, Dict[str, Direction]] = {}
        self._entered: Set[Tuple[str, str]] = set()
        self._waits: Dict[str, _Wait] = {}
        self._seq = 0
        self.grant_log: Deque[Tuple[float, str, Tuple[str, ...]]] = \
            collections.deque(maxlen=1000)

    def set_zones(self, same_direction: Mapping[str, bool]) -> None:
        """구역 목록 교체 (지도 갱신). 없어진 구역의 보유 기록은 버린다."""
        self._same_dir = dict(same_direction)
        for z in [z for z in self._holders if z not in self._same_dir]:
            del self._holders[z]
        self._entered = {(r, z) for r, z in self._entered if z in self._same_dir}

    def holders(self, zone_id: str) -> List[str]:
        """구역 토큰 보유 로봇."""
        return sorted(self._holders.get(zone_id, {}))

    def held_by(self, robot_id: str) -> List[str]:
        """로봇이 보유한 구역."""
        return sorted(z for z, h in self._holders.items() if robot_id in h)

    def forget(self, robot_id: str) -> None:
        """로봇의 보유·대기 기록 삭제."""
        for h in self._holders.values():
            h.pop(robot_id, None)
        self._entered = {(r, z) for r, z in self._entered if r != robot_id}
        self._waits.pop(robot_id, None)

    def _compatible(self, zone_id: str, robot_id: str, direction: Direction) -> bool:
        others = {r: d for r, d in self._holders.get(zone_id, {}).items() if r != robot_id}
        if not others:
            return True
        if not self._same_dir.get(zone_id, False) or direction is None:
            return False
        for d in others.values():
            if d is None or d[0] * direction[0] + d[1] * direction[1] < self.same_direction_cos:
                return False
        return True

    def _blockers(self, zone_id: str, robot_id: str) -> List[str]:
        return [r for r in self._holders.get(zone_id, {}) if r != robot_id]

    def update(self, now: float, requests: Mapping[str, TokenRequest],
               occupancy: Mapping[str, Set[str]], upcoming: Mapping[str, Set[str]],
               directions: Optional[Mapping[str, Mapping[str, Direction]]] = None
               ) -> Dict[str, TokenDecision]:
        """
        한 주기 처리: 반납 → 점유 등록 → 요청 갱신 → 부여 → 추월 집계.

        occupancy: 로봇 → 지금 몸체가 걸친 구역, upcoming: 로봇 → 남은 경로가 지나는 구역,
        directions: 로봇 → 구역별 통과 방향 (점유 등록용, 없으면 요청의 방향).
        """
        directions = directions or {}
        # 1) 반납: 들어갔다 나왔거나, 경로가 더는 지나지 않는 구역
        for zone_id, hs in self._holders.items():
            for rid in list(hs):
                occ = zone_id in occupancy.get(rid, ())
                if occ:
                    self._entered.add((rid, zone_id))
                elif (rid, zone_id) in self._entered or zone_id not in upcoming.get(rid, ()):
                    del hs[rid]
                    self._entered.discard((rid, zone_id))
        # 2) 점유 등록: 토큰 없이 구역 안에 있는 로봇 (시작 · 관리 밖 진입). 충돌하는 보유자가
        #    있으면 등록하지 않는다 → 그 로봇은 계속 대기(hold)하고 교착이면 해소 계층이 푼다
        for rid, zones in occupancy.items():
            for zone_id in zones:
                if zone_id not in self._same_dir:
                    continue
                hs = self._holders.setdefault(zone_id, {})
                if rid in hs:
                    continue
                d = directions.get(rid, {}).get(zone_id)
                req = requests.get(rid)
                if d is None and req is not None:
                    d = req.directions.get(zone_id)
                if self._compatible(zone_id, rid, d):
                    hs[rid] = d
                    self._entered.add((rid, zone_id))
        # 3) 요청 갱신
        need: Dict[str, Tuple[str, ...]] = {}
        for rid, req in requests.items():
            missing = tuple(z for z in req.zones
                            if z in self._same_dir and rid not in self._holders.get(z, {}))
            if missing:
                need[rid] = missing
        for rid in list(self._waits):
            # 경로가 조금 바뀌어도(겹치는 구역이 남으면) 대기 시각 · 추월 수를 이어 간다 (기아 방지)
            if rid not in need or not set(self._waits[rid].zones) & set(requests[rid].zones):
                del self._waits[rid]
            else:
                self._waits[rid].zones = requests[rid].zones
        for rid in need:
            if rid not in self._waits:
                self._seq += 1
                self._waits[rid] = _Wait(requests[rid].zones, now, self._seq)

        # 4) 부여 (기아 로봇 먼저 요청 순, 그다음 우선순위 → 마감 → 대기 시간 → id)
        def key(rid: str):
            w, req = self._waits[rid], requests[rid]
            deadline = req.deadline if req.deadline is not None else math.inf
            if w.bypassed >= self.max_bypass:
                return (0, w.since, -req.priority, deadline, w.seq, rid)
            return (1, -req.priority, deadline, w.since, w.seq, rid)

        reserved: Dict[str, str] = {}
        granted_now: Dict[str, Tuple[str, ...]] = {}
        blockers: Dict[str, List[str]] = {}
        for rid in sorted(need, key=key):
            req, w = requests[rid], self._waits[rid]
            ok, why = True, []
            for zone_id in need[rid]:
                owner = reserved.get(zone_id)
                if owner is not None and owner != rid:
                    ok = False
                    why.append(owner)
                if not self._compatible(zone_id, rid, req.directions.get(zone_id)):
                    ok = False
                    why += self._blockers(zone_id, rid)
            if ok:
                for zone_id in need[rid]:
                    self._holders.setdefault(zone_id, {})[rid] = req.directions.get(zone_id)
                granted_now[rid] = need[rid]
                self.grant_log.append((now, rid, need[rid]))
            else:
                blockers[rid] = sorted(set(why))
                if w.bypassed >= self.max_bypass:
                    for zone_id in need[rid]:
                        reserved.setdefault(zone_id, rid)
        # 5) 추월 집계: 먼저 기다리던 로봇보다 엄격히 나중 요청이 같은 구역을 받으면 +1
        for rid, zones in need.items():
            if rid in granted_now:
                continue
            w = self._waits[rid]
            for other, got in granted_now.items():
                ow = self._waits[other]
                if set(got) & set(zones) and ow.since > w.since:
                    w.bypassed += 1
        for rid in granted_now:
            del self._waits[rid]
        # 결과
        out: Dict[str, TokenDecision] = {}
        for rid, req in requests.items():
            if rid in need and rid not in granted_now:
                w = self._waits[rid]
                out[rid] = TokenDecision(False, req.zones, tuple(blockers.get(rid, ())),
                                         now - w.since, w.bypassed)
            else:
                out[rid] = TokenDecision(True, req.zones)
        return out
