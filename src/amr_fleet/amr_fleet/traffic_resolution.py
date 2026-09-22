"""
교착 해소 전략 (명세 4.9 "교착 해소 전략 최소 2가지"; sequences.md §3 전략 1 · 2).

전략 1 YIELD    우선순위 기반 양보: 교착 로봇 중 우선순위가 가장 낮은 로봇(희생 로봇)을 가장 가까운
                빈 대기 포켓으로 보낸다 (traffic/yield_pose + traffic/hold=true). 나머지는 그대로 진행.
                포켓 = traffic_zones.yaml 의 pockets (기본) 또는 auto_pockets 면 자동 탐색 셀: 주행 가능,
                구역(통로·교차로) 밖, 다른 로봇의 남은 경로에서 pocket_clearance_m 이상.
                다른 로봇 몸체와 다른 로봇이 보유·점유한 구역을 지나지 않는 격자 Dijkstra 로 "도달 가능한
                가장 가까운" 곳을 고른다 (그 경로 = Pocket.route — traffic_manager 가 경로 위 빈 구역의
                토큰을 희생 로봇에 미리 부여한다). 스스로 움직일 수 없는 로봇(E-stop · 오류 · 관측 끊김)은
                희생 로봇이 되지 않는다.
전략 2 ALT_PATH 대체 경로 탐색: 희생 로봇의 목표 경로 중 자기 몸 앞쪽 ~ 분쟁 구간 끝(+extend) 을
                keepout_half_width_m 폭으로, 상대 로봇 몸체 주변을 그 로봇 전용 keepout_mask 에
                lethal(100) 로 칠한다. Nav2 KeepoutFilter 가 전역 경로를 다시 찾는다(플래너 코드 불변).
                마스크 가드: 자기 몸 주변은 비우고, 상대가 목표 자리에 있거나 목표가 분쟁 구간 안이면
                마스크를 만들지 않으며, 마스크를 칠한 지도에서 목표가 닿지 않으면(거친 격자 연결 성분)
                플래너 실패를 기다리지 않고 곧바로 희생 로봇을 바꾼다.
                상대가 모두 움직일 수 없는 로봇(E-stop · 오류 · 관측 끊김)이면 그 로봇이 걸친 구역 전체도
                칠한다 (그 구역 토큰은 풀리지 않으니 구역을 돌아가는 경로만 의미가 있다).
에스컬레이션    포켓 없음 · escalate_after_s 동안 상대가 전혀 못 움직임(사이클 지속) · yield_timeout_s 초과
                → 전략 2. 전략 2 에서 replan_grace_s 안에 마스크를 피하는 경로가 없으면(또는
                alt_path_timeout_s 초과) 다음 순위 로봇으로 희생자를 바꿔(우선순위 역전) 전략 1 부터 다시.
                후보 소진 · deadlock_max_s 초과 · 무진전 해소 시도 max_attempts_per_robot 초과 → UNRESOLVED
                (ERROR 알림, 운영자/작업 재할당 몫). cooldown_s 뒤 같은 로봇 집합을 새로 시도한다.
                움직일 수 없는 로봇이 낀 사건의 UNRESOLVED 는 그 로봇들이 다시 움직일 수 있거나 관측에서
                빠질 때까지 다시 열지 않는다 (E-stop 로봇 하나로 cooldown_s 마다 교착 · 실패를 되풀이하지 않게).
해소 판정      (1) 희생 로봇이 아닌 주행 로봇이 모두 분쟁 영역(탐지 시 로봇 위치 반경
                contested_radius_m, 걸쳐 있던 구역)을 벗어났고 남은 경로 pass_check_m 안에 다시 들어오지
                않고, (2) 희생 로봇을 지금 풀어 원래 목표 경로로 보내도 release_horizon_s 안에 그들과
                정면·교차 충돌이 예측되지 않을 때 (1차선 통로에서 희생 로봇이 아직 상대 앞에서 물러나는
                중에 풀어 다시 마주치는 것을 막는다). 그때 hold · yield_pose · keepout_mask 를 모두 거둔다.
"""

from __future__ import annotations

import collections
import dataclasses
import heapq
import math
import re
from typing import (
    Any, Deque, Dict, FrozenSet, Iterable, List, Mapping, Optional, Sequence, Set, Tuple,
)

import numpy as np
from scipy import ndimage

from amr_fleet.traffic_geometry import (
    GridSpec, cumulative_length, disc_mask, distance_to_path, project, sub_path,
)
from amr_fleet.traffic_prediction import FOLLOWING, find_conflict, predict_trajectory
from amr_fleet.traffic_zones import ZoneMap, clearance_map, zone_visits

YIELD = 'YIELD'
ALT_PATH = 'ALT_PATH'
STRATEGIES = (YIELD, ALT_PATH)

EV_DEADLOCK = 'DEADLOCK'
EV_RESOLVED = 'RESOLVED'
EV_UNRESOLVED = 'UNRESOLVED'
EV_ESCALATED = 'ESCALATED'

LETHAL = 100


@dataclasses.dataclass
class ResolutionConfig:
    """해소 파라미터 (traffic.yaml resolution.*)."""

    robot_radius: float = 0.36            # 외접 반경 — 포켓 · 몸체 판정
    passage_radius: float = 0.22          # 통과 가능 = 벽까지 ≥ 이 값 (풋프린트 반폭 0.2 + 여유)
    auto_pockets: bool = True
    pocket_search_radius_m: float = 12.0
    pocket_clearance_m: float = 1.0
    pocket_path_horizon_m: float = 12.0
    search_resolution_m: float = 0.25
    escalate_after_s: float = 10.0
    yield_timeout_s: float = 30.0
    replan_grace_s: float = 5.0
    alt_path_timeout_s: float = 45.0
    deadlock_max_s: float = 120.0
    max_attempts_per_robot: int = 3
    keepout_half_width_m: float = 0.6
    keepout_extend_m: float = 1.5
    keepout_margin_m: float = 0.2
    keepout_resolution_m: float = 0.05
    contested_radius_m: float = 1.2
    pass_check_m: float = 6.0
    move_eps_m: float = 0.5
    cooldown_s: float = 30.0
    progress_reset_m: float = 2.0         # 목표 경로가 이만큼 줄면 무진전 시도 수를 0 으로
    release_check: bool = True            # 풀기 전 희생 로봇 원래 경로 ↔ 상대 경로 충돌 예측
    release_horizon_s: float = 10.0
    # 아래는 TrafficConfig 가 최상위 · prediction.* 값으로 채운다 (파라미터로 따로 두지 않는다)
    nominal_speed: float = 1.0
    safety_distance: float = 1.02
    time_window_s: float = 2.0
    head_on_angle_deg: float = 135.0
    following_angle_deg: float = 45.0


@dataclasses.dataclass
class RobotView:
    """해소·탐지 계층이 보는 로봇 1대 (TrafficManager 가 매 주기 만든다)."""

    robot_id: str
    x: float
    y: float
    yaw: float = 0.0
    speed: float = 0.0
    path: Optional[np.ndarray] = None     # 지금 따르는 남은 경로 (첫 점 ≈ 현재 위치), 없으면 None
    active: bool = False                  # 목표를 향해 주행해야 하는 상태
    idle: bool = False                    # 작업·경로 없음 (주차)
    priority: int = -1                    # 작업 우선순위 (작업 없음 -1)
    deadline: Optional[float] = None
    zones: FrozenSet[str] = frozenset()   # 몸체가 걸친 구역
    intent: Optional[np.ndarray] = None   # 목표(작업) 경로. 양보 중에는 포켓 경로가 아닌 원래 경로
    mobile: bool = True                   # 명령을 받아 움직일 수 있다 (E-stop · 오류 · 관측 끊김이면 False)
    stale: bool = False                   # 관측이 끊겨 마지막 자세에 멈춘 장애물로 본다

    @property
    def xy(self) -> Tuple[float, float]:
        """위치."""
        return (self.x, self.y)

    def rank(self) -> Tuple[int, float, str]:
        """중요도 정렬 키 (작을수록 중요): 우선순위 높은 순 → 마감 이른 순 → id."""
        return (-self.priority, self.deadline if self.deadline is not None else math.inf,
                self.robot_id)

    def remaining_m(self) -> Optional[float]:
        """지금 경로의 남은 길이 (경로 없으면 None)."""
        return _length(self.path)

    def intent_path(self) -> Optional[np.ndarray]:
        """목표 경로 (없으면 지금 경로)."""
        return self.intent if self.intent is not None else self.path

    def intent_remaining_m(self) -> Optional[float]:
        """목표 경로의 남은 길이."""
        return _length(self.intent_path())


def _length(path: Optional[np.ndarray]) -> Optional[float]:
    if path is None or len(path) < 2:
        return None
    return float(cumulative_length(path)[-1])


@dataclasses.dataclass(frozen=True)
class Pocket:
    """대기 포켓 (yaml 또는 자동)."""

    pocket_id: str
    x: float
    y: float
    yaw: float = 0.0
    cost_m: float = 0.0
    route: Tuple[Tuple[float, float], ...] = dataclasses.field(default=(), compare=False)


POCKET_ID_RE = re.compile(r'^[A-Za-z0-9_.-]{1,64}$')


def pockets_from_config(entries: Any) -> List[Pocket]:
    """
    traffic_zones.yaml 의 pockets 목록 → Pocket 목록.

    항목 = {id, x, y, yaw (rad, 선택 — 기본 0)}. 형식 오류·비유한 수·중복 id 는 항목 번호와 함께 ValueError.
    """
    if entries is None:
        return []
    if not isinstance(entries, (list, tuple)):
        raise ValueError('pockets 는 목록이어야 한다')
    out: List[Pocket] = []
    seen: Set[str] = set()
    for k, e in enumerate(entries):
        where = f'pockets[{k}]'
        if not isinstance(e, dict):
            raise ValueError(f'{where}: {{id, x, y[, yaw]}} 사전이어야 한다')
        unknown = set(e) - {'id', 'x', 'y', 'yaw'}
        if unknown:
            raise ValueError(f'{where}: 모르는 키 {sorted(unknown)}')
        pid = str(e.get('id', ''))
        if not POCKET_ID_RE.match(pid):
            raise ValueError(f'{where}: id 는 [A-Za-z0-9_.-]{{1,64}}: {pid!r}')
        if pid in seen:
            raise ValueError(f'{where}: 중복 id {pid!r}')
        try:
            x, y, yaw = float(e['x']), float(e['y']), float(e.get('yaw', 0.0))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'{where}: x, y 는 수여야 한다 ({exc!r})') from exc
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError(f'{where}: 좌표가 유한하지 않다')
        seen.add(pid)
        out.append(Pocket(pid, x, y, yaw))
    return out


# ---------------------------------------------------------------------- 포켓 탐색
class TraversabilityGrid:
    """
    포켓 탐색용 저해상도 격자.

    passable = 로봇 중심이 지날 수 있는 셀 (벽까지 ≥ passage_radius, 블록 안 한 셀이라도 — 좁은 통로 보존),
    trav     = 포켓을 둘 수 있는 셀 (벽까지 ≥ 외접 반경, 블록 안 모든 셀 — 제자리 회전 가능),
    zone_mask = 구역(통로 · 교차로) 셀 — 자동 포켓 제외,
    zone_labels = 셀 → zone_ids 번호 (-1 = 구역 없음) — 보유 구역 우회 · 간격 검사.
    """

    def __init__(self, spec: GridSpec, traversable: np.ndarray,
                 zone_mask: Optional[np.ndarray] = None,
                 passable: Optional[np.ndarray] = None,
                 zone_labels: Optional[np.ndarray] = None,
                 zone_ids: Sequence[str] = ()):
        self.spec = spec
        self.trav = traversable.astype(bool)
        self.passable = self.trav.copy() if passable is None else passable.astype(bool) | self.trav
        self.zone_ids = list(zone_ids)
        self.zone_labels = (np.full(self.trav.shape, -1, dtype=np.int32) if zone_labels is None
                            else zone_labels.astype(np.int32))
        self.zone_mask = (self.zone_labels >= 0 if zone_mask is None
                          else zone_mask.astype(bool))

    def zones_mask(self, zone_ids: Iterable[str]) -> np.ndarray:
        """지정한 구역의 셀 (height, width) bool (모르는 id 는 무시)."""
        idx = [k for k, z in enumerate(self.zone_ids) if z in set(zone_ids)]
        return np.isin(self.zone_labels, idx)

    @classmethod
    def from_occupancy(cls, occupied: np.ndarray, spec: GridSpec, robot_radius: float,
                       search_resolution: float, zone_map: Optional[ZoneMap] = None,
                       passage_radius: Optional[float] = None) -> 'TraversabilityGrid':
        """점유 격자(미지 = 점유) → f×f 블록으로 줄인 격자 (f = search_resolution / resolution)."""
        clear = clearance_map(occupied, spec.resolution)
        pr = robot_radius if passage_radius is None else min(passage_radius, robot_radius)
        f = max(1, int(round(search_resolution / spec.resolution)))
        h, w = spec.height // f, spec.width // f
        ds = GridSpec(spec.resolution * f, spec.origin_x, spec.origin_y, w, h)

        def blocks(mask: np.ndarray) -> np.ndarray:
            return mask[:h * f, :w * f].reshape(h, f, w, f)

        trav = blocks(clear >= robot_radius).all(axis=(1, 3))
        passable = blocks(clear >= pr).any(axis=(1, 3))
        if zone_map is None or not len(zone_map):
            return cls(ds, trav, None, passable)
        ids = zone_map.ids()
        gx, gy = ds.centers()
        at = zone_map.zones_at(np.stack([gx.ravel(), gy.ravel()], axis=1))
        index = {z: k for k, z in enumerate(ids)}
        labels = np.array([-1 if z is None else index[z] for z in at],
                          dtype=np.int32).reshape(h, w)
        return cls(ds, trav, None, passable, labels, ids)


def near_points_mask(spec: GridSpec, points: Iterable[Sequence[float]],
                     radius: float) -> np.ndarray:
    """점들 중 하나라도 radius 안인 셀 (EDT, 셀 중심 기준)."""
    marks = np.zeros((spec.height, spec.width), dtype=bool)
    pts = np.asarray(list(points), dtype=float).reshape(-1, 2)
    if len(pts):
        ix, iy = spec.world_to_cell(pts[:, 0], pts[:, 1])
        ok = spec.in_bounds(ix, iy)
        marks[iy[ok], ix[ok]] = True
    if not marks.any():
        return marks
    return ndimage.distance_transform_edt(~marks) * spec.resolution <= radius


def find_yield_pocket(tgrid: TraversabilityGrid, start: Tuple[float, float],
                      other_robots: Sequence[Tuple[float, float]],
                      avoid_paths: Sequence[np.ndarray], cfg: ResolutionConfig,
                      pockets: Sequence[Pocket] = (),
                      reserved: Sequence[Tuple[float, float]] = (),
                      blocked: Optional[np.ndarray] = None) -> Optional[Pocket]:
    """
    출발점에서 다른 로봇 몸체를 지나지 않고 갈 수 있는 가장 가까운 포켓 (경로 비용 기준).

    pockets 가 있으면 그중에서, 없으면(또는 auto_pockets 이고 지정 포켓이 모두 불가면) 자동 탐색.
    포켓 조건: 주행 가능, 구역 밖(자동만), 다른 로봇·예약 포켓에서 2r + margin 이상,
    avoid_paths 의 점에서 pocket_clearance_m 이상. 반경 pocket_search_radius_m 안에서만 찾는다.
    blocked: 지나지 못하는 셀 (다른 로봇이 보유·점유한 구역). 돌려주는 Pocket.route = 출발점 → 포켓
    격자 경로 (월드 좌표).
    """
    spec = tgrid.spec
    r = cfg.robot_radius
    sx, sy = spec.world_to_cell(start[0], start[1])
    sx, sy = int(sx), int(sy)
    if not bool(spec.in_bounds(sx, sy)):
        return None
    bodies = near_points_mask(spec, other_robots, 2.0 * r)
    own = disc_mask(spec, [start], r + spec.resolution)
    impassable = ~tgrid.passable | (bodies & ~own)
    if blocked is not None:
        impassable |= blocked
    impassable[sy, sx] = False
    step = max(0.5 * spec.resolution, 0.05)
    samples = [p[:, :2] for p in (_resample(q, step) for q in avoid_paths) if p is not None]
    forbid = near_points_mask(spec, np.concatenate(samples) if samples else [],
                              cfg.pocket_clearance_m)
    forbid |= near_points_mask(spec, list(other_robots) + list(reserved),
                               2.0 * r + cfg.keepout_margin_m)
    forbid |= ~tgrid.trav

    targets: Dict[Tuple[int, int], Pocket] = {}
    for p in pockets:
        px, py = spec.world_to_cell(p.x, p.y)
        if bool(spec.in_bounds(px, py)) and not forbid[int(py), int(px)]:
            targets[(int(px), int(py))] = p
    use_auto = cfg.auto_pockets and (not pockets or not targets)
    if not targets and not use_auto:
        return None
    auto_ok = forbid | tgrid.zone_mask

    res = spec.resolution
    dist = {(sx, sy): 0.0}
    parent: Dict[Tuple[int, int], Tuple[int, int]] = {}
    heap = [(0.0, sx, sy)]

    def route(cx: int, cy: int, end: Tuple[float, float]) -> Tuple[Tuple[float, float], ...]:
        cells = [(cx, cy)]
        while cells[-1] in parent:
            cells.append(parent[cells[-1]])
        pts = [(float(start[0]), float(start[1]))]
        for ix, iy in reversed(cells[:-1]):
            wx, wy = spec.cell_to_world(ix, iy)
            pts.append((float(wx), float(wy)))
        pts.append((float(end[0]), float(end[1])))
        return tuple(pts)

    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
             (-1, -1, math.sqrt(2))]
    while heap:
        d, cx, cy = heapq.heappop(heap)
        if d > dist.get((cx, cy), math.inf) + 1e-12:
            continue
        if (cx, cy) in targets:
            p = targets[(cx, cy)]
            return Pocket(p.pocket_id, p.x, p.y, p.yaw, d, route(cx, cy, (p.x, p.y)))
        if use_auto and not auto_ok[cy, cx]:
            wx, wy = spec.cell_to_world(cx, cy)
            yaw = math.atan2(float(wy) - start[1], float(wx) - start[0]) if d > 0 else 0.0
            return Pocket('auto', float(wx), float(wy), yaw, d,
                          route(cx, cy, (float(wx), float(wy))))
        for dx, dy, c in moves:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < spec.width and 0 <= ny < spec.height) or impassable[ny, nx]:
                continue
            nd = d + c * res
            if nd > cfg.pocket_search_radius_m or nd >= dist.get((nx, ny), math.inf):
                continue
            dist[(nx, ny)] = nd
            parent[(nx, ny)] = (cx, cy)
            heapq.heappush(heap, (nd, nx, ny))
    return None


def _resample(path: Optional[np.ndarray], step: float) -> Optional[np.ndarray]:
    if path is None or len(path) == 0:
        return None
    if len(path) == 1:
        return np.asarray(path, dtype=float)
    cum = cumulative_length(path)
    return sub_path(path, cum, 0.0, float(cum[-1]), step)


# ---------------------------------------------------------------------- keepout 마스크
@dataclasses.dataclass
class KeepoutMask:
    """로봇 1대 전용 keepout 마스크 (nav_msgs/OccupancyGrid 로 발행, 100 = lethal)."""

    spec: GridSpec
    data: np.ndarray        # (height, width) int8, 0 / 100

    @property
    def lethal_cells(self) -> int:
        """치명(lethal) 셀 수."""
        return int(np.count_nonzero(self.data >= LETHAL))

    def blocks(self, xy: np.ndarray) -> np.ndarray:
        """점마다 lethal 셀에 드는지."""
        return self.spec.lookup(self.data, np.asarray(xy, dtype=float), 0) >= LETHAL

    def lethal_points(self) -> np.ndarray:
        """치명(lethal) 셀 중심 (K, 2)."""
        iy, ix = np.nonzero(self.data >= LETHAL)
        x, y = self.spec.cell_to_world(ix, iy)
        return np.stack([x, y], axis=1) if len(ix) else np.zeros((0, 2))


def mask_spec_for(map_spec: GridSpec, resolution: float) -> GridSpec:
    """지도 전체를 덮는 keepout 마스크 격자 (해상도만 바꾼다)."""
    # OccupancyGrid 해상도는 float32 (0.1 → 0.10000000149) — 오차로 한 칸 늘지 않게 1e-3 칸을 뺀다
    width = max(1, int(math.ceil(map_spec.width * map_spec.resolution / resolution - 1e-3)))
    height = max(1, int(math.ceil(map_spec.height * map_spec.resolution / resolution - 1e-3)))
    return GridSpec(resolution, map_spec.origin_x, map_spec.origin_y, width, height)


def empty_mask(spec: GridSpec) -> KeepoutMask:
    """치명 셀 없는 마스크 (해소 후 · 시작 시 발행)."""
    return KeepoutMask(spec, np.zeros((spec.height, spec.width), dtype=np.int8))


def _from_position(path: Optional[np.ndarray], x: float, y: float) -> Optional[np.ndarray]:
    """경로를 현재 위치에 투영해 그 뒤만 남기고 현재 위치를 앞에 붙인다."""
    if path is None or len(path) < 2:
        return None
    cum = cumulative_length(path)
    s0, d0 = project(path, cum, x, y)
    rest = sub_path(path, cum, s0, float(cum[-1]), 0.2)
    return np.vstack([[x, y], rest]) if d0 > 0.05 else rest


def build_keepout_mask(victim: RobotView, blockers: Sequence[Tuple[float, float]],
                       cfg: ResolutionConfig, zone_map: Optional[ZoneMap] = None,
                       contested_zones: Iterable[str] = (),
                       spec: Optional[GridSpec] = None,
                       block_zones: Iterable[str] = ()) -> Optional[KeepoutMask]:
    """
    희생 로봇의 분쟁 구간 keepout 마스크. 목표 경로가 없거나 목표가 분쟁 구간 안이면 None.

    - 목표 경로(intent — 양보 중이면 포켓 경로가 아닌 원래 경로)의 자기 몸 앞(r + margin)부터
      분쟁 구간 끝까지를 keepout_half_width_m 폭으로 (분쟁 구간 끝 = 경로 근처 상대 로봇 투영점,
      분쟁 구역 첫 방문 출구 중 가장 먼 곳 + keepout_extend_m, 단 목표 앞 반폭 + r + margin 에서 자른다)
    - 상대 로봇 몸체 주변 (r + margin)
    - block_zones 구역 전체 (움직일 수 없는 상대가 걸친 구역 — 목표가 그 안이면 None)
    - 희생 로봇 자기 몸 주변은 비운다 (시작 셀이 lethal 이면 플래너가 실패한다)
    spec 이 있으면 그 격자(지도 전체), 없으면 분쟁 구간을 감싸는 작은 격자에 그린다.
    """
    path = _from_position(victim.intent_path(), victim.x, victim.y)
    if path is None:
        return None
    r, m = cfg.robot_radius, cfg.keepout_margin_m
    goal = path[-1]
    if any(math.hypot(bx - goal[0], by - goal[1]) < 2.0 * r + m for bx, by in blockers):
        return None       # 상대가 목표 자리에 있다 — 목표를 막는 마스크는 플래너만 실패시킨다
    block_zones = sorted(set(block_zones)) if zone_map is not None else []
    if block_zones and zone_map.zone_at(float(goal[0]), float(goal[1])) in block_zones:
        return None       # 목표가 막힌 구역 안
    cum = cumulative_length(path)
    total = float(cum[-1])
    s0 = min(total, r + m)
    s_need = s0           # 반드시 막을 끝: 경로 근처 상대의 투영점, 분쟁 구역의 (첫 방문) 출구
    for bx, by in blockers:
        sb, db = project(path, cum, bx, by)
        if db <= cfg.contested_radius_m:
            s_need = max(s_need, sb)
    contested = set(contested_zones)
    if zone_map is not None and contested:
        seen = set()
        for v in zone_visits(path, zone_map):
            if v.zone_id in contested and v.zone_id not in seen:
                seen.add(v.zone_id)
                s_need = max(s_need, v.s_out)
    s_max = total - (cfg.keepout_half_width_m + r + m)    # 목표 자리는 몸 크기만큼 비워 둔다
    if s_need > s_max:
        return None       # 목표가 분쟁 구간 안 — 돌아갈 길이 있을 수 없다 (목표 경합은 양보로)
    s1 = min(s_need + cfg.keepout_extend_m, s_max)       # 너머로 더 칠하는 길이는 목표 앞에서 자른다
    if spec is None:
        res = cfg.keepout_resolution_m
        band = sub_path(path, cum, s0, s1, 0.5 * res) if s1 > s0 else np.zeros((0, 2))
        allp = np.concatenate([band.reshape(-1, 2),
                               np.asarray(blockers, dtype=float).reshape(-1, 2),
                               np.array([[victim.x, victim.y]])])
        pad = max(cfg.keepout_half_width_m, r + m) + 2.0 * res
        x0 = math.floor((allp[:, 0].min() - pad) / res) * res
        y0 = math.floor((allp[:, 1].min() - pad) / res) * res
        w = int(math.ceil((allp[:, 0].max() + pad - x0) / res))
        h = int(math.ceil((allp[:, 1].max() + pad - y0) / res))
        spec = GridSpec(res, x0, y0, w, h)
    band = (sub_path(path, cum, s0, s1, 0.5 * spec.resolution) if s1 > s0
            else np.zeros((0, 2)))
    lethal = (near_points_mask(spec, band, cfg.keepout_half_width_m) if len(band)
              else np.zeros((spec.height, spec.width), dtype=bool))
    if blockers:
        lethal |= disc_mask(spec, blockers, r + m)
    if block_zones:
        lethal |= zone_map.mask(spec, block_zones)
    lethal &= ~disc_mask(spec, [victim.xy], r + m)
    return KeepoutMask(spec, np.where(lethal, LETHAL, 0).astype(np.int8))


def route_exists(tgrid: TraversabilityGrid, start: Tuple[float, float],
                 goal: Tuple[float, float], mask: Optional[KeepoutMask] = None) -> bool:
    """
    정적 지도(+ keepout 마스크)에서 start → goal 로 이어지는 통과 가능 영역이 있는지.

    거친 격자의 8-연결 성분으로 본다 (낙관적 — 있다고 해도 플래너가 실패할 수는 있다. 그때는
    replan_grace_s 뒤 희생 로봇 교체). 다른 로봇은 장애물로 보지 않는다 (마스크의 몸체 원만 본다).
    """
    spec = tgrid.spec
    ok = tgrid.passable.copy()
    if mask is not None and mask.lethal_cells:
        gx, gy = spec.centers()
        ok &= ~mask.blocks(np.stack([gx.ravel(), gy.ravel()], axis=1)).reshape(ok.shape)
    sx, sy = (int(v) for v in spec.world_to_cell(start[0], start[1]))
    tx, ty = (int(v) for v in spec.world_to_cell(goal[0], goal[1]))
    if not (bool(spec.in_bounds(sx, sy)) and bool(spec.in_bounds(tx, ty))):
        return False
    ok[sy, sx] = True
    labels, _ = ndimage.label(ok, structure=np.ones((3, 3), dtype=bool))
    home = labels[sy, sx]
    near = labels[max(0, ty - 1):ty + 2, max(0, tx - 1):tx + 2]   # 목표 셀이 거친 격자에서 벽에 붙은 경우
    return bool(np.any(near == home))


# ---------------------------------------------------------------------- 사건 상태 기계
@dataclasses.dataclass
class WorldView:
    """해소 계층 입력 (한 주기)."""

    robots: Dict[str, RobotView]
    zone_map: Optional[ZoneMap] = None
    tgrid: Optional[TraversabilityGrid] = None
    pockets: Sequence[Pocket] = ()
    mask_spec: Optional[GridSpec] = None      # keepout 마스크 격자 (지도 전체). None = 작은 격자
    held_zones: Mapping[str, FrozenSet[str]] = dataclasses.field(default_factory=dict)
    # ↑ 로봇 → 보유(토큰) · 점유(몸체) 구역. 희생 로봇의 포켓 경로가 남의 구역을 지나지 않게 한다


@dataclasses.dataclass
class Incident:
    """확정된 교착 1건과 해소 진행 상태."""

    incident_id: int
    robots: Tuple[str, ...]
    kind: str
    detected_at: float
    victims: List[str]                           # 해소 시도 순서 (덜 중요한 로봇 먼저)
    contested_points: Dict[str, Tuple[float, float]]
    contested_zones: Set[str]
    victim_index: int = 0
    strategy: str = ''
    step_started: float = 0.0
    step_positions: Dict[str, Tuple[float, float]] = dataclasses.field(default_factory=dict)
    pocket: Optional[Pocket] = None
    mask: Optional[KeepoutMask] = None
    attempts: List[str] = dataclasses.field(default_factory=list)
    original_paths: Dict[str, Optional[np.ndarray]] = dataclasses.field(default_factory=dict)
    immobile: Tuple[str, ...] = ()               # 스스로 못 움직이는 로봇 (희생 로봇 후보 아님)

    @property
    def victim(self) -> str:
        """현재 희생 로봇."""
        return self.victims[min(self.victim_index, len(self.victims) - 1)]

    @property
    def signature(self) -> FrozenSet[str]:
        """로봇 집합."""
        return frozenset(self.robots)


@dataclasses.dataclass
class ResolutionEvent:
    """해소 계층 이벤트 (TrafficManager 가 /fleet/traffic_events 로 옮긴다)."""

    kind: str                 # DEADLOCK | ESCALATED | RESOLVED | UNRESOLVED
    incident: Incident
    time: float
    message: str
    values: Dict[str, str]


@dataclasses.dataclass
class IncidentCommand:
    """사건이 희생 로봇에 거는 명령."""

    incident_id: int
    hold: bool
    yield_pose: Optional[Tuple[float, float, float]] = None
    keepout: Optional[KeepoutMask] = None


class IncidentManager:
    """교착 사건 열기 · 전략 선택 · 에스컬레이션 · 해소 판정."""

    HISTORY_MAX = 1000

    def __init__(self, cfg: Optional[ResolutionConfig] = None):
        self.cfg = cfg or ResolutionConfig()
        self._active: Dict[int, Incident] = {}
        self._next_id = 1
        self._robot_attempts: Dict[str, int] = {}
        self._anchor: Dict[str, float] = {}           # 마지막 사건 때 목표 경로 남은 길이
        self._cooldown: Dict[FrozenSet[str], float] = {}
        self._stuck: Dict[FrozenSet[str], FrozenSet[str]] = {}   # 실패한 집합 → 움직일 수 없던 로봇
        self.history: Deque[Dict[str, object]] = collections.deque(maxlen=self.HISTORY_MAX)

    # --- 조회 ---
    def active(self) -> List[Incident]:
        """진행 중 사건 (id 순)."""
        return [self._active[k] for k in sorted(self._active)]

    def members(self) -> Set[str]:
        """진행 중 사건에 묶인 로봇."""
        return {r for inc in self._active.values() for r in inc.robots}

    def in_cooldown(self, robots: Iterable[str], now: float) -> bool:
        """UNRESOLVED 직후 같은 로봇 집합은 cooldown_s 동안 다시 열지 않는다."""
        until = self._cooldown.get(frozenset(robots))
        return until is not None and now < until

    def suppressed(self) -> Set[str]:
        """움직일 수 없는 로봇 때문에 실패해 다시 열지 않는 사건의 로봇."""
        return {r for sig in self._stuck for r in sig}

    def attempts_of(self, robot_id: str) -> int:
        """로봇의 무진전 해소 시도 수."""
        return self._robot_attempts.get(robot_id, 0)

    def reset_robot(self, robot_id: str) -> None:
        """로봇이 목표를 바꾸거나 도착했다 (무진전 카운터 초기화)."""
        self._robot_attempts.pop(robot_id, None)
        self._anchor.pop(robot_id, None)

    def observe_progress(self, world: WorldView) -> None:
        """목표 경로가 progress_reset_m 넘게 줄어든 로봇은 무진전 카운터를 지운다 (라이브락 논증의 m_i)."""
        for rid in list(self._anchor):
            v = world.robots.get(rid)
            rem = None if v is None else v.intent_remaining_m()
            if rem is not None and rem < self._anchor[rid] - self.cfg.progress_reset_m:
                self.reset_robot(rid)

    def commands(self) -> Dict[str, IncidentCommand]:
        """희생 로봇별 명령."""
        out: Dict[str, IncidentCommand] = {}
        for inc in self.active():
            v = inc.victim
            if inc.strategy == YIELD and inc.pocket is not None:
                p = inc.pocket
                out[v] = IncidentCommand(inc.incident_id, True, (p.x, p.y, p.yaw))
            elif inc.strategy == ALT_PATH and inc.mask is not None:
                out[v] = IncidentCommand(inc.incident_id, False, None, inc.mask)
        return out

    # --- 사건 열기 ---
    def open(self, now: float, robots: Sequence[str], kind: str, world: WorldView,
             contested_zones: Iterable[str] = ()) -> List[ResolutionEvent]:
        """확정된 교착으로 사건을 연다 → DEADLOCK (+ 곧바로 전략 시작 또는 UNRESOLVED)."""
        views = world.robots
        members = tuple(sorted(r for r in robots if r in views))
        # 희생 로봇 = 스스로 움직일 수 있는 로봇만 (E-stop · 오류 · 관측 끊김 로봇에 양보 명령은 의미 없다)
        victims = sorted((r for r in members if views[r].mobile), key=lambda r: views[r].rank(),
                         reverse=True)
        inc = Incident(self._next_id, members, kind, now, victims or list(members),
                       {r: views[r].xy for r in members}, set(contested_zones))
        inc.immobile = tuple(r for r in members if not views[r].mobile)
        inc.original_paths = {r: views[r].intent_path() for r in members}
        self._next_id += 1
        for r in members:
            self._robot_attempts[r] = self._robot_attempts.get(r, 0) + 1
            rem = views[r].intent_remaining_m()
            if rem is not None:
                self._anchor[r] = rem
        detected = self._event(EV_DEADLOCK, inc, now, f'교착 {kind}: {" ↔ ".join(members)}',
                               victim=inc.victim)
        if not victims:
            return [detected] + self._unresolved(inc, now, 'no_mobile_victim')
        worst = max(members, key=lambda r: self._robot_attempts[r])
        if self._robot_attempts[worst] > self.cfg.max_attempts_per_robot:
            return [detected] + self._unresolved(inc, now, f'attempts({worst})')
        self._active[inc.incident_id] = inc
        events = self._start_yield(inc, now, world)
        detected.values['strategy'] = inc.strategy
        detected.values['victim'] = inc.victim
        detected.values['attempts'] = '>'.join(inc.attempts)
        if inc.pocket is not None:
            detected.values['pocket'] = inc.pocket.pocket_id
        return [detected] + events

    # --- 주기 처리 ---
    def step(self, now: float, world: WorldView) -> List[ResolutionEvent]:
        """모든 사건을 한 주기 진행."""
        self.observe_progress(world)
        for sig, imm in list(self._stuck.items()):
            if any(r not in world.robots or world.robots[r].mobile for r in imm):
                del self._stuck[sig]              # 다시 움직일 수 있다 → 새로 시도해도 된다
                self._cooldown.pop(sig, None)
        events: List[ResolutionEvent] = []
        for inc in self.active():
            events += self._step_one(inc, now, world)
        return events

    def _step_one(self, inc: Incident, now: float, world: WorldView) -> List[ResolutionEvent]:
        cfg = self.cfg
        if now - inc.detected_at > cfg.deadlock_max_s:
            return self._unresolved(inc, now, 'deadlock_max')
        views = world.robots
        victim = inc.victim
        others = [r for r in inc.robots if r != victim]
        elapsed = now - inc.step_started
        if inc.strategy == YIELD:
            if self._others_passed(inc, others, world) and self._release_ok(inc, others, world):
                return self._resolved(inc, now)
            if elapsed >= cfg.escalate_after_s and not self._any_moved(inc, others, world):
                return self._start_alt(inc, now, world, 'persist')
            if elapsed >= cfg.yield_timeout_s:
                return self._start_alt(inc, now, world, 'yield_timeout')
            return []
        # ALT_PATH: 희생 로봇의 지금 경로가 마스크를 피하면 대체 경로를 찾은 것
        #           (꼭짓점만 보면 긴 직선 구간이 마스크를 건너가도 모른다 → 마스크 해상도로 다시 뽑는다)
        v = views.get(victim)
        plan_ok = (v is None or not v.active or v.path is None
                   or not bool(inc.mask.blocks(
                       _resample(v.path, 0.5 * inc.mask.spec.resolution)).any()))
        if plan_ok:
            active_others = [r for r in others if r in views and views[r].active]
            if active_others:
                done = (self._others_passed(inc, others, world)
                        and self._release_ok(inc, others, world))
            else:
                done = v is None or self._passed(v, inc, world, exclude=victim)
            if done:
                return self._resolved(inc, now)
        if not plan_ok and elapsed >= cfg.replan_grace_s:
            return self._next_victim(inc, now, world, 'no_alt_path')
        if elapsed >= cfg.alt_path_timeout_s:
            return self._next_victim(inc, now, world, 'alt_timeout')
        return []

    # --- 판정 ---
    def _passed(self, view: RobotView, inc: Incident, world: WorldView,
                exclude: Optional[str] = None) -> bool:
        """분쟁 영역을 벗어나 남은 경로도 다시 들어오지 않는지 (주행 중이 아니면 True)."""
        if not view.active or view.path is None:
            return True
        cfg = self.cfg
        pts = np.array([p for r, p in inc.contested_points.items() if r != exclude],
                       dtype=float).reshape(-1, 2)
        if len(pts) and np.any(np.hypot(pts[:, 0] - view.x, pts[:, 1] - view.y)
                               < cfg.contested_radius_m):
            return False
        if view.zones & inc.contested_zones:
            return False
        cum = cumulative_length(view.path)
        ahead = sub_path(view.path, cum, 0.0, min(float(cum[-1]), cfg.pass_check_m), 0.2)
        if len(pts) and np.any(distance_to_path(pts, ahead) < cfg.contested_radius_m):
            return False
        if world.zone_map is not None and inc.contested_zones:
            if set(world.zone_map.zones_at(ahead)) & inc.contested_zones:
                return False
        return True

    def _others_passed(self, inc: Incident, others: Sequence[str], world: WorldView) -> bool:
        views = world.robots
        return all(r not in views or self._passed(views[r], inc, world) for r in others)

    def _release_ok(self, inc: Incident, others: Sequence[str], world: WorldView) -> bool:
        """희생 로봇을 원래 목표 경로로 풀어도 상대와 정면 · 교차 충돌이 예측되지 않는지."""
        cfg = self.cfg
        v = world.robots.get(inc.victim)
        if not cfg.release_check or v is None or not v.active:
            return True
        original = inc.original_paths.get(inc.victim)
        path = _from_position(original if original is not None else v.intent_path(), v.x, v.y)
        if path is None:
            return True
        tv = predict_trajectory(v.robot_id, v.x, v.y, v.yaw, path, cfg.nominal_speed,
                                cfg.release_horizon_s, 0.25, 0.0)
        for r in others:
            o = world.robots.get(r)
            if o is None or not o.active or o.path is None:
                continue
            to = predict_trajectory(r, o.x, o.y, o.yaw, o.path, cfg.nominal_speed,
                                    cfg.release_horizon_s, 0.25, 0.0)
            c = find_conflict(tv, to, cfg.safety_distance, cfg.time_window_s,
                              cfg.head_on_angle_deg, cfg.following_angle_deg)
            if c is not None and c.kind != FOLLOWING:
                return False
        return True

    def _any_moved(self, inc: Incident, others: Sequence[str], world: WorldView) -> bool:
        for r in others:
            v, p0 = world.robots.get(r), inc.step_positions.get(r)
            if v is not None and p0 is not None and \
                    math.hypot(v.x - p0[0], v.y - p0[1]) >= self.cfg.move_eps_m:
                return True
        return False

    # --- 전략 전이 ---
    def _begin(self, inc: Incident, now: float, world: WorldView, strategy: str) -> None:
        inc.strategy = strategy
        inc.step_started = now
        inc.step_positions = {r: world.robots[r].xy for r in inc.robots if r in world.robots}

    def _start_yield(self, inc: Incident, now: float, world: WorldView) -> List[ResolutionEvent]:
        views = world.robots
        victim = inc.victim
        others = [r for r in inc.robots if r != victim]
        if not any(r in views and views[r].active for r in others):
            # 상대가 모두 주차(유휴) — 비킬 로봇이 없으니 희생 로봇이 돌아간다
            return self._start_alt(inc, now, world, 'others_parked', announce=False)
        pocket = None
        if world.tgrid is not None and victim in views:
            avoid = [views[r].intent_path() for r in views
                     if r != victim and views[r].active and views[r].intent_path() is not None]
            avoid = [_clip(p, self.cfg.pocket_path_horizon_m) for p in avoid]
            others_xy = [views[r].xy for r in views if r != victim]
            reserved = [(i.pocket.x, i.pocket.y) for i in self._active.values()
                        if i is not inc and i.pocket is not None]
            held = set().union(*(zs for r, zs in world.held_zones.items() if r != victim)) \
                - set(views[victim].zones)
            blocked = world.tgrid.zones_mask(held) if held else None
            pocket = find_yield_pocket(world.tgrid, views[victim].xy, others_xy, avoid,
                                       self.cfg, world.pockets, reserved, blocked)
        if pocket is None:
            return self._start_alt(inc, now, world, 'no_pocket')
        self._begin(inc, now, world, YIELD)
        inc.pocket, inc.mask = pocket, None
        inc.attempts.append(f'{YIELD}:{victim}')
        return []

    def _start_alt(self, inc: Incident, now: float, world: WorldView, reason: str,
                   announce: bool = True) -> List[ResolutionEvent]:
        views = world.robots
        victim = inc.victim
        v = views.get(victim)
        mask = None
        if v is not None and v.active:
            blockers = [p for r, p in inc.contested_points.items() if r != victim]
            others = [views[r] for r in inc.robots if r != victim and r in views]
            stuck = set()
            if others and not any(o.mobile for o in others):
                stuck = set().union(*(o.zones for o in others))    # 풀리지 않는 구역은 돌아간다
            mask = build_keepout_mask(v, blockers, self.cfg, world.zone_map,
                                      inc.contested_zones, world.mask_spec, stuck)
            intent = v.intent_path()
            if mask is not None and world.tgrid is not None and intent is not None and \
                    not route_exists(world.tgrid, v.xy, (float(intent[-1, 0]),
                                                         float(intent[-1, 1])), mask):
                mask = None       # 마스크를 피해 목표로 가는 길이 지도에 없다 → 기다리지 않고 역전
        if mask is None:
            inc.attempts.append(f'{ALT_PATH}:{victim}({reason},no_route)')
            return self._next_victim(inc, now, world, f'{reason},no_route')
        was_yield = inc.strategy == YIELD
        self._begin(inc, now, world, ALT_PATH)
        inc.pocket, inc.mask = None, mask
        inc.attempts.append(f'{ALT_PATH}:{victim}({reason})')
        if announce and (was_yield or reason == 'no_pocket'):
            return [self._event(EV_ESCALATED, inc, now,
                                f'전략 1 → 2 ({reason}): {victim} keepout', victim=victim,
                                reason=reason, strategy=ALT_PATH)]
        return []

    def _next_victim(self, inc: Incident, now: float, world: WorldView,
                     reason: str) -> List[ResolutionEvent]:
        inc.pocket, inc.mask = None, None
        inc.victim_index += 1
        if inc.victim_index >= len(inc.victims):
            return self._unresolved(inc, now, f'exhausted({reason})')
        victim = inc.victim
        rest = self._start_yield(inc, now, world)
        if inc.incident_id not in self._active or inc.victim != victim:
            return rest               # 새 희생 로봇도 곧바로 실패 → 그쪽 이벤트가 이미 나갔다
        ev = self._event(EV_ESCALATED, inc, now, f'우선순위 역전 ({reason}): 희생 로봇 → {victim}',
                         victim=victim, reason=reason, strategy=inc.strategy,
                         pocket=inc.pocket.pocket_id if inc.pocket is not None else '')
        return [ev] + rest

    # --- 종료 ---
    def _resolved(self, inc: Incident, now: float) -> List[ResolutionEvent]:
        self._active.pop(inc.incident_id, None)
        dt = now - inc.detected_at
        self.history.append(dict(incident=inc.incident_id, robots=inc.robots, kind=inc.kind,
                                 outcome=EV_RESOLVED, strategy=inc.strategy, victim=inc.victim,
                                 resolve_s=dt, attempts=list(inc.attempts)))
        return [self._event(EV_RESOLVED, inc, now,
                            f'해소 {inc.strategy} ({inc.victim}) {dt:.1f} s', victim=inc.victim,
                            strategy=inc.strategy, resolve_time_s=f'{dt:.2f}',
                            pocket=inc.pocket.pocket_id if inc.pocket is not None else '')]

    def _unresolved(self, inc: Incident, now: float, reason: str) -> List[ResolutionEvent]:
        self._active.pop(inc.incident_id, None)
        self._cooldown[inc.signature] = now + self.cfg.cooldown_s
        if inc.immobile:
            self._cooldown[inc.signature] = math.inf
            self._stuck[inc.signature] = frozenset(inc.immobile)
        for r in inc.robots:            # cooldown 뒤 새로 시도할 수 있게 (운영자 알림은 이미 나간다)
            self.reset_robot(r)
        dt = now - inc.detected_at
        self.history.append(dict(incident=inc.incident_id, robots=inc.robots, kind=inc.kind,
                                 outcome=EV_UNRESOLVED, strategy=inc.strategy,
                                 victim=inc.victim, resolve_s=dt, attempts=list(inc.attempts),
                                 reason=reason))
        return [self._event(EV_UNRESOLVED, inc, now, f'해소 실패 ({reason}) {dt:.1f} s',
                            victim=inc.victim, reason=reason, strategy=inc.strategy,
                            resolve_time_s=f'{dt:.2f}')]

    def _event(self, kind: str, inc: Incident, now: float, message: str,
               **extra: object) -> ResolutionEvent:
        values = {'incident': str(inc.incident_id), 'robots': ','.join(inc.robots),
                  'type': inc.kind, 'attempts': '>'.join(inc.attempts),
                  'zones': ','.join(sorted(inc.contested_zones))}
        if inc.immobile:
            values['immobile'] = ','.join(inc.immobile)
        values.update({k: str(v) for k, v in extra.items()})
        return ResolutionEvent(kind, inc, now, message, values)


def _clip(path: np.ndarray, length: float) -> np.ndarray:
    cum = cumulative_length(path)
    if len(path) < 2 or cum[-1] <= length:
        return path
    return sub_path(path, cum, 0.0, length, 0.2)
