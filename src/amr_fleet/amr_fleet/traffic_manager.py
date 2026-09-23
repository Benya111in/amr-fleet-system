"""
교통 관리 본체 — traffic_manager_node 의 ROS 없는 로직 (components.md §3.6 / §5.6, sequences.md §3).

한 주기 update(now, 관측) (노드에서 2 Hz):
 1. 관측 → RobotView: plan 을 현재 위치에 투영한 남은 경로, 목표 경로(양보 중에는 원래 경로),
    정지 지속 시간, 몸체가 걸친 구역.
    관측이 obs_timeout_s 넘게 끊긴 로봇(stale)은 빼지 않는다 (fail closed): 마지막 자세에 멈춘
    장애물 · wait-for 노드로 두고 토큰을 그대로 쥐게 한다 → traffic/STALE (토큰 보유 중이면 알림).
    lost_timeout_s(= fleet_manager robot_state_timeout_s) 를 넘기면(lost) 마지막 몸체가 걸친 구역
    토큰만 남기고 나머지를 반납한다 — 관측이 다시 오면 recovered.
 2. 충돌 예측: 목표 경로를 공칭 속도로 따라간다고 본 시공간 표본 → HEAD_ON / CROSSING / FOLLOWING
    (토큰 · 교차 hold 로 서 있는 로봇도 "풀어 주면 갈 경로" 로 예측한다 — 풀었다 다시 거는 진동을
    막는다. 앞 로봇 몸체에 막혀 stationary_time_s 넘게 서 있는 줄(queue)은 제자리로 예측한다)
 3. 구역 토큰: 교차로 · 1차선 통로 진입 approach 앞에서 요청 → 우선순위 · 마감 · 대기 순 부여,
    기아 방지 (traffic_zones). 못 받으면 traffic/hold. 몸체가 구역 안이면 묶이지 않은 다음 구역은
    빠져나온 뒤에 요청한다. 포켓으로 가는 희생 로봇은 포켓 경로 위 빈 구역 토큰을 미리 받는다
 4. 교차 양보: 구역 밖 CROSSING 예측은 우선순위 낮은 쪽을 충돌 crossing_act_s 전에 hold 하고,
    상대가 지나가 예측 충돌이 사라질 때까지(최대 crossing_max_hold_s) 유지한다. 몸체가 구역 안이거나
    들어간 구역 토큰을 쥔 로봇은 세우지 않는다 (상대가 양보하거나 둘 다 진행)
 5. wait-for 그래프 (token / crossing / block) → Tarjan SCC → confirm_s 유지 시 교착 확정
 6. 사이클 없는 막힘: 주차(유휴) · 움직일 수 없는(E-stop · 오류 · stale) 로봇이 경로·구역을
    막음(BLOCKED), 진전 없는 정체 묶음(LIVELOCK), 토큰 대기 감시(token_wait_stall_s → STALL,
    wait_max_s → UNRESOLVED — 막는 로봇과 그 상태를 적는다)
 7. 해소: 전략 1 YIELD → 전략 2 ALT_PATH → 우선순위 역전 → UNRESOLVED (traffic_resolution)
 8. 명령 합성 (로봇별): 사건 명령 > 토큰 hold > 교차 hold.
mode=observe 면 토큰 · 교차 양보 · 해소를 끄고 탐지 · 보고만 한다 (교통 관리 없는 기준선 측정용).
  보고 이력(히스테리시스): 사이클은 같은(부분/상위) 로봇 집합이 clear_s 동안 연속으로 안 보여야
  RESOLVED(strategy=none) 로 닫고 그 전에는 다시 세지 않는다. 막힘(BLOCKED/LIVELOCK)은 로봇마다
  에피소드(마지막 진전 시각이 바뀔 때까지)에 한 번만 센다.
"""

from __future__ import annotations

import collections
import dataclasses
import math
from typing import Deque, Dict, FrozenSet, List, Mapping, Optional, Sequence, Set, Tuple

import numpy as np

from amr_fleet.alerts import (
    ALERT_DEADLOCK, ALERT_TRAFFIC_STALE, TRAFFIC_DEADLOCK, TRAFFIC_ESCALATED, TRAFFIC_RESOLVED,
    TRAFFIC_STALE, TRAFFIC_STALL, TRAFFIC_UNRESOLVED,
)
from amr_fleet.deadlock import (
    DL_BLOCKED, DL_LIVELOCK, EDGE_BLOCK, EDGE_CROSSING, EDGE_TOKEN, CycleConfirmer,
    StallDetector, WaitForGraph, classify_deadlock, deadlock_components,
)
from amr_fleet.traffic_geometry import (
    GridSpec, as_path, cumulative_length, distance_to_path, project, sub_path,
)
from amr_fleet.traffic_prediction import (
    CROSSING, Conflict, Trajectory, predict_conflicts, predict_trajectory,
)
from amr_fleet.traffic_resolution import (
    ALT_PATH, EV_DEADLOCK, EV_ESCALATED, EV_RESOLVED, EV_UNRESOLVED, YIELD, IncidentManager,
    KeepoutMask, Pocket, ResolutionConfig, ResolutionEvent, RobotView, TraversabilityGrid,
    WorldView, mask_spec_for, pockets_from_config,
)
from amr_fleet.traffic_zones import (
    CORRIDOR, TokenDecision, TokenRequest, Zone, ZoneMap, ZoneTokenManager, derive_zones_from_grid,
    grid_spacing_conflicts, load_layout, request_chain, spacing_conflicts, zone_visits,
)

# amr_msgs/msg/RobotState.STATUS_* (kpi.py 와 같은 값)
ST_IDLE, ST_MOVING, ST_DOCKING, ST_LOADING, ST_CHARGING, ST_ERROR, ST_ESTOP = range(7)
BUSY_STATIONARY = frozenset({ST_DOCKING, ST_LOADING, ST_CHARGING, ST_ERROR, ST_ESTOP})
IMMOBILE = frozenset({ST_ERROR, ST_ESTOP})       # 스스로 비킬 수 없다 (BLOCKED 대상, 희생 로봇 제외)
STATUS_NAMES = {ST_IDLE: 'IDLE', ST_MOVING: 'MOVING', ST_DOCKING: 'DOCKING', ST_LOADING: 'LOADING',
                ST_CHARGING: 'CHARGING', ST_ERROR: 'ERROR', ST_ESTOP: 'ESTOP'}
STALE, LOST = 'stale', 'lost'
SPACING_CHECKS = ('reject', 'warn')

# diagnostic_msgs/DiagnosticStatus 수준 (노드가 bytes 로 바꾼다)
LEVEL_OK, LEVEL_WARN, LEVEL_ERROR = 0, 1, 2
MODES = ('active', 'observe')
ZONE_SOURCES = ('yaml', 'map', 'both', 'none')
YIELD_GOAL_TOL_M = 0.5      # plan 끝점이 명령한 포켓에서 이 안이면 "양보 경로" 로 본다


# ---------------------------------------------------------------------- 설정
@dataclasses.dataclass
class PredictionConfig:
    """prediction.* — 충돌 예측 · 교차 양보."""

    horizon_s: float = 10.0
    dt_s: float = 0.25
    safety_margin_m: float = 0.3          # safety_distance = 2 r + margin
    time_window_s: float = 2.0
    head_on_angle_deg: float = 135.0
    following_angle_deg: float = 45.0
    crossing_hold: bool = True
    crossing_act_s: float = 4.0           # 양보자 도착 예측이 이보다 가까우면 hold
    crossing_min_s: float = 0.8           # 이보다 가까우면 못 세운다 → 상대가 양보
    crossing_max_hold_s: float = 15.0


@dataclasses.dataclass
class ZonesConfig:
    """zones.* — 교차로 · 1차선 통로 구역과 진입 토큰."""

    source: str = 'yaml'                  # yaml | map | both | none
    tokens: bool = True
    approach_distance_m: float = 1.5       # 주기 0.5 s + 지연 0.1 s + 제동 0.5 m (1 m/s) = 1.1 m < 1.5 m
    corridor_approach_m: float = 2.5      # 1차선 통로: 입구에서 더 멀리 기다린다 (나오는 로봇이 비켜 갈 자리)
    chain_gap_m: float = 1.5
    spacing_check: str = 'reject'         # 구역 간격 검사 (spacing_conflicts): reject | warn
    max_bypass: int = 2
    corridor_same_direction: bool = True
    narrow_width_m: float = 1.5
    min_corridor_area_m2: float = 0.2      # 통과 가능 띠 넓이 (폭 0.6 m 통로의 띠 폭은 ~0.2 m)
    intersection_arm_m: float = 4.0
    intersection_min_area_m2: float = 1.0
    intersection_max_area_m2: float = 36.0
    auto_margin_m: float = 0.3
    occupied_threshold: int = 50


@dataclasses.dataclass
class DeadlockConfig:
    """deadlock.* — wait-for 그래프 · 확정 · 정체."""

    block_lookahead_m: float = 2.0
    block_lateral_margin_m: float = 0.3
    stationary_speed: float = 0.05
    stationary_time_s: float = 2.0
    confirm_s: float = 1.0
    clear_s: float = 3.0                  # observe: 보고한 교착이 이만큼 연속으로 안 보여야 풀림
    stall_time_s: float = 15.0
    stall_progress_m: float = 0.5
    detour_reset_m: float = 2.0
    idle_block_s: float = 5.0
    livelock_radius_m: float = 3.0
    token_wait_stall_s: float = 30.0      # 토큰 대기가 이만큼 이어지면 traffic/STALL (막는 로봇 · 상태)
    wait_max_s: float = 120.0             # 토큰 대기 · 1대 정체가 이만큼이면 traffic/UNRESOLVED (운영자 몫)


_GROUPS = ('prediction', 'zones', 'deadlock', 'resolution')
# 최상위 · prediction.* 값을 복사해 쓰는 해소 설정 (파라미터로 따로 선언하지 않는다)
_HIDDEN = {'resolution.robot_radius', 'resolution.passage_radius', 'resolution.nominal_speed',
           'resolution.safety_distance', 'resolution.time_window_s',
           'resolution.head_on_angle_deg', 'resolution.following_angle_deg'}


@dataclasses.dataclass
class TrafficConfig:
    """traffic.yaml 의 ros__parameters 와 1:1 (그룹은 '<group>.<name>')."""

    mode: str = 'active'
    robot_radius: float = 0.36            # 풋프린트 0.6 x 0.4 m 외접 반경
    passage_radius: float = 0.22          # 로봇 중심이 지날 수 있는 벽까지 거리 (반폭 0.2 + 여유)
    nominal_speed: float = 1.0            # [m/s] fleet.yaml nominal_speed 와 같은 값
    goal_tolerance_m: float = 0.3
    obs_timeout_s: float = 3.0            # 넘으면 stale: 마지막 자세 · 토큰 유지 (fail closed)
    lost_timeout_s: float = 5.0           # 넘으면 lost (fleet.yaml robot_state_timeout_s 와 같은 값)
    max_path_m: float = 60.0              # 남은 경로를 이만큼만 본다
    zones_file: str = ''                  # traffic_zones.yaml 경로 ('' = 구역 · 포켓 없음)
    prediction: PredictionConfig = dataclasses.field(default_factory=PredictionConfig)
    zones: ZonesConfig = dataclasses.field(default_factory=ZonesConfig)
    deadlock: DeadlockConfig = dataclasses.field(default_factory=DeadlockConfig)
    resolution: ResolutionConfig = dataclasses.field(default_factory=ResolutionConfig)

    def __post_init__(self):
        self._sync()

    def _sync(self) -> None:
        r, p = self.resolution, self.prediction
        r.robot_radius = self.robot_radius
        r.passage_radius = self.passage_radius
        r.nominal_speed = self.nominal_speed
        r.safety_distance = self.safety_distance
        r.time_window_s = p.time_window_s
        r.head_on_angle_deg = p.head_on_angle_deg
        r.following_angle_deg = p.following_angle_deg

    @property
    def safety_distance(self) -> float:
        """두 로봇 중심 간 충돌 판정 거리 [m]."""
        return 2.0 * self.robot_radius + self.prediction.safety_margin_m

    def to_flat(self) -> Dict[str, object]:
        """'<group>.<name>' → 현재 값 (ROS 파라미터 선언용)."""
        out: Dict[str, object] = {}
        for f in dataclasses.fields(self):
            if f.name in _GROUPS:
                for g in dataclasses.fields(getattr(self, f.name)):
                    key = f'{f.name}.{g.name}'
                    if key not in _HIDDEN:
                        out[key] = getattr(getattr(self, f.name), g.name)
            else:
                out[f.name] = getattr(self, f.name)
        return out

    @classmethod
    def from_flat(cls, flat: Mapping[str, object]) -> 'TrafficConfig':
        """평탄 dict → 설정 (모르는 키는 ValueError, 타입은 기본값 타입으로 변환)."""
        cfg = cls()
        for key, value in flat.items():
            group, _, name = key.rpartition('.')
            if (group and group not in _GROUPS) or key in _HIDDEN or name in _GROUPS:
                raise ValueError(f'알 수 없는 traffic 파라미터: {key}')
            target = getattr(cfg, group) if group else cfg
            if name not in {f.name for f in dataclasses.fields(target)}:
                raise ValueError(f'알 수 없는 traffic 파라미터: {key}')
            default = getattr(target, name)
            if isinstance(default, bool):
                if isinstance(value, bool):
                    pass
                elif str(value).strip().lower() in ('true', 'false'):
                    value = str(value).strip().lower() == 'true'
                else:
                    raise ValueError(f'{key} 는 true/false: {value!r}')
            elif isinstance(default, int):
                value = int(value)
            elif isinstance(default, float):
                value = float(value)
            else:
                value = str(value)
            setattr(target, name, value)
        cfg._sync()
        cfg.validate()
        return cfg

    def validate(self) -> None:
        """값 범위 검사 (ValueError)."""
        if self.mode not in MODES:
            raise ValueError(f'mode 는 {MODES} 중 하나: {self.mode!r}')
        if self.zones.source not in ZONE_SOURCES:
            raise ValueError(f'zones.source 는 {ZONE_SOURCES} 중 하나: {self.zones.source!r}')
        if self.zones.spacing_check not in SPACING_CHECKS:
            raise ValueError(f'zones.spacing_check 는 {SPACING_CHECKS} 중 하나: '
                             f'{self.zones.spacing_check!r}')
        positive = {
            'robot_radius': self.robot_radius, 'passage_radius': self.passage_radius,
            'nominal_speed': self.nominal_speed,
            'prediction.horizon_s': self.prediction.horizon_s,
            'prediction.dt_s': self.prediction.dt_s,
            'deadlock.confirm_s': self.deadlock.confirm_s + 1e-9,
            'deadlock.stall_time_s': self.deadlock.stall_time_s,
            'deadlock.token_wait_stall_s': self.deadlock.token_wait_stall_s,
            'zones.approach_distance_m': self.zones.approach_distance_m,
            'zones.corridor_approach_m': self.zones.corridor_approach_m,
            'obs_timeout_s': self.obs_timeout_s,
            'resolution.search_resolution_m': self.resolution.search_resolution_m,
            'resolution.keepout_resolution_m': self.resolution.keepout_resolution_m,
        }
        for k, v in positive.items():
            if not math.isfinite(v) or v <= 0.0:
                raise ValueError(f'{k} 는 양수여야 한다: {v}')
        if self.zones.max_bypass < 0:
            raise ValueError('zones.max_bypass 는 0 이상')
        if not self.lost_timeout_s >= self.obs_timeout_s:
            raise ValueError('lost_timeout_s 는 obs_timeout_s 이상')
        if not self.deadlock.wait_max_s >= self.deadlock.token_wait_stall_s:
            raise ValueError('deadlock.wait_max_s 는 token_wait_stall_s 이상')
        if self.resolution.max_attempts_per_robot < 1:
            raise ValueError('resolution.max_attempts_per_robot 는 1 이상')


# ---------------------------------------------------------------------- 입출력 타입
@dataclasses.dataclass
class RobotObservation:
    """노드(또는 시뮬레이터)가 주는 로봇 1대 관측."""

    robot_id: str
    x: float
    y: float
    yaw: float = 0.0
    speed: float = 0.0
    plan: Optional[np.ndarray] = None     # /amr_XX/plan (N, 2), 없으면 None
    status: int = ST_MOVING               # RobotState.status
    task_id: str = ''
    priority: int = -1
    deadline: Optional[float] = None
    stamp: Optional[float] = None         # 관측 시각 (None = now)


@dataclasses.dataclass
class RobotCommand:
    """로봇 1대에 내릴 명령 (노드가 바뀐 것만 발행)."""

    hold: bool = False
    reason: str = ''
    yield_pose: Optional[Tuple[float, float, float]] = None
    keepout: Optional[KeepoutMask] = None
    incident_id: int = 0


@dataclasses.dataclass
class TrafficEvent:
    """/fleet/traffic_events 한 건 (+ alert_name 이 있으면 /fleet/alerts 도)."""

    name: str
    level: int
    message: str
    robots: Tuple[str, ...]
    values: Dict[str, str]
    alert_name: str = ''
    hardware_id: str = ''


@dataclasses.dataclass
class TickResult:
    """update() 결과."""

    commands: Dict[str, RobotCommand]
    events: List[TrafficEvent]
    conflicts: List[Conflict]
    wait_for: List[Tuple[str, str, str]]
    token_waits: Dict[str, TokenDecision]


@dataclasses.dataclass
class _Motion:
    last_xy: Tuple[float, float]
    last_t: float
    still_since: float


@dataclasses.dataclass
class _CrossLatch:
    yielder: str
    other: str
    since: float


def occupancy_from_values(values: np.ndarray, threshold: int = 50) -> np.ndarray:
    """점유 격자 값 (height, width) → 점유 bool (미지 -1 도 점유)."""
    v = np.asarray(values)
    return (v < 0) | (v >= threshold)


def _leads_to(chain: Mapping[str, str], start: str, target: str) -> bool:
    """양보 사슬(양보 로봇 → 상대)을 start 부터 따라가면 target 에 닿는지."""
    seen = set()
    node: Optional[str] = start
    while node is not None and node not in seen:
        if node == target:
            return True
        seen.add(node)
        node = chain.get(node)
    return False


def _remaining(plan: np.ndarray, x: float, y: float, max_len: float
               ) -> Tuple[np.ndarray, float]:
    """남은 경로: plan 을 (x, y) 에 투영해 현재 위치부터 max_len 까지 + 전체 남은 길이."""
    cum = cumulative_length(plan)
    s0, d0 = project(plan, cum, x, y)
    end = min(float(cum[-1]), s0 + max_len)
    rem = sub_path(plan, cum, s0, end, 0.2)
    path = np.vstack([[x, y], rem]) if d0 > 0.05 else rem
    return path, float(cum[-1]) - s0 + d0


# ---------------------------------------------------------------------- 본체
class TrafficManager:
    """교통 관리 · 교착 탐지 · 해소 본체."""

    def __init__(self, config: Optional[TrafficConfig] = None,
                 zones: Optional[Sequence[Zone]] = None,
                 pockets: Optional[Sequence[Pocket]] = None):
        self.cfg = config or TrafficConfig()
        self.cfg._sync()          # 생성 뒤 바꾼 최상위 · prediction 값을 해소 설정에 반영
        self.cfg.validate()
        z = self.cfg.zones
        file_zones: List[Zone] = []
        file_pockets: List[Pocket] = []
        if self.cfg.zones_file and (zones is None or pockets is None):
            fz, fp = load_layout(self.cfg.zones_file, z.corridor_same_direction)
            file_zones, file_pockets = fz, pockets_from_config(fp)
        self._yaml_zones = list(zones if zones is not None else file_zones)
        if z.source not in ('yaml', 'both'):
            self._yaml_zones = []
        self.pockets: List[Pocket] = list(pockets if pockets is not None else file_pockets)
        self.layout_warnings: List[str] = []
        if z.tokens and self.active_mode:
            self._check_spacing(spacing_conflicts(self._yaml_zones, self._approach_of(
                self._yaml_zones), z.chain_gap_m, self.cfg.robot_radius),
                z.spacing_check == 'reject')
        self.zone_map = ZoneMap(self._yaml_zones)
        self._approach = self._approach_of(self.zone_map.zones())
        self.tokens = ZoneTokenManager({zz.zone_id: zz.same_direction
                                        for zz in self.zone_map.zones()}, z.max_bypass)
        self.tgrid: Optional[TraversabilityGrid] = None
        self.mask_spec: Optional[GridSpec] = None
        self.incidents = IncidentManager(self.cfg.resolution)
        d = self.cfg.deadlock
        self._confirmer = CycleConfirmer(d.confirm_s)
        self._stall = StallDetector(d.stall_time_s, d.stall_progress_m, d.detour_reset_m)
        self._motion: Dict[str, _Motion] = {}
        self._goals: Dict[str, Tuple[float, float]] = {}
        self._intent_plan: Dict[str, np.ndarray] = {}
        self._yield_goal: Dict[str, Tuple[float, float]] = {}
        self._latches: Dict[FrozenSet[str], _CrossLatch] = {}
        self._stall_reported: Dict[str, int] = {}             # 1대 정체 보고: 1 = STALL, 2 = UNRESOLVED
        self._observed: Dict[FrozenSet[str], float] = {}      # observe 모드: 보고한 교착 → 탐지 시각
        self._last_seen: Dict[FrozenSet[str], float] = {}     # observe 모드: 마지막으로 보인 시각
        # observe 모드: 막힘(BLOCKED/LIVELOCK)을 보고한 로봇 → 그때의 마지막 진전 시각 (에피소드)
        self._stall_episode: Dict[str, Optional[float]] = {}
        self._stall_groups: Dict[FrozenSet[str], float] = {}  # observe 모드: 보고한 막힘 묶음 → 시각
        self._stale: Dict[str, str] = {}                      # 관측이 끊긴 로봇 → stale | lost
        self._stale_alerted: Set[str] = set()                 # 토큰 보유로 알림을 낸 stale 로봇
        self._status: Dict[str, int] = {}
        self._token_watch_level: Dict[str, int] = {}          # 토큰 대기 감시: 1 = STALL, 2 = UNRESOLVED
        self._held_prev: Set[str] = set()                     # 지난 주기 hold 명령을 받은 로봇
        self._yield_tokens: Dict[str, Tuple[Tuple[int, str], Tuple[str, ...]]] = {}
        # ↑ 희생 로봇 → ((사건, 포켓), 포켓 경로에서 미리 받은 구역)
        self.views: Dict[str, RobotView] = {}
        self.stats: Dict[str, object] = {
            'deadlocks': 0, 'resolved': 0, 'unresolved': 0, 'escalations': 0,
            'resolve_times': collections.deque(maxlen=1000),
            'strategies': {YIELD: 0, ALT_PATH: 0}}

    @property
    def active_mode(self) -> bool:
        """해소 명령을 내리는 모드인지 (observe 면 False)."""
        return self.cfg.mode == 'active'

    # ------------------------------------------------------------------ 지도
    def set_map(self, occupied: np.ndarray, spec: GridSpec) -> None:
        """/map 수신: 지도 유도 구역(source=map|both) · 포켓 탐색 격자 · 마스크 격자를 다시 만든다."""
        z, cfg = self.cfg.zones, self.cfg
        labels, grid_zones = None, []
        if z.source in ('map', 'both'):
            labels, grid_zones = derive_zones_from_grid(
                occupied, spec, cfg.passage_radius, z.narrow_width_m, z.min_corridor_area_m2,
                z.intersection_arm_m, z.intersection_min_area_m2, z.intersection_max_area_m2,
                z.auto_margin_m, z.corridor_same_direction)
            taken = {zz.zone_id for zz in self._yaml_zones}
            grid_zones = [g for g in grid_zones if g.zone_id not in taken]
        self.zone_map = ZoneMap(self._yaml_zones, grid_zones,
                                labels if grid_zones else None, spec if grid_zones else None)
        self.tokens.set_zones({zz.zone_id: zz.same_direction for zz in self.zone_map.zones()})
        self._approach = self._approach_of(self.zone_map.zones())
        self.tgrid = TraversabilityGrid.from_occupancy(
            occupied, spec, cfg.robot_radius, cfg.resolution.search_resolution_m, self.zone_map,
            cfg.passage_radius)
        self.mask_spec = mask_spec_for(spec, cfg.resolution.keepout_resolution_m)
        if grid_zones and z.tokens and self.active_mode:     # 지도 유도 구역은 경고만 (고칠 yaml 이 없다)
            self._check_spacing(grid_spacing_conflicts(
                self.tgrid.zone_labels, self.tgrid.zone_ids, self.tgrid.spec.resolution,
                self._approach, z.chain_gap_m, cfg.robot_radius,
                only={g.zone_id for g in grid_zones}), False)

    def _approach_of(self, zones: Sequence[Zone]) -> Dict[str, float]:
        """구역 → 토큰 요청 거리 (1차선 통로는 corridor_approach_m)."""
        z = self.cfg.zones
        return {zz.zone_id: z.corridor_approach_m if zz.kind == CORRIDOR else z.approach_distance_m
                for zz in zones}

    def _check_spacing(self, conflicts: Sequence[Tuple[str, str, float]], reject: bool) -> None:
        """구역 간격 검사 결과: reject 면 ValueError, 아니면 layout_warnings 에 남긴다."""
        if not conflicts:
            return
        z = self.cfg.zones
        pairs = ', '.join(f'{a}-{b} {g:.2f} m' for a, b, g in conflicts)
        msg = (f'구역 간격이 chain_gap_m({z.chain_gap_m:g}) 보다 넓고 요청 거리(approach_distance_m '
               f'{z.approach_distance_m:g}, 통로 corridor_approach_m {z.corridor_approach_m:g}) + '
               f'robot_radius({self.cfg.robot_radius:g}) 보다 좁다 — 묶이지도 않고 사이에 설 자리도 '
               f'없어 구역 안에서 기다린다: {pairs}')
        if reject:
            raise ValueError(msg + ' (zones.spacing_check: reject)')
        self.layout_warnings.append(msg)

    def describe_zones(self) -> str:
        """로그용 구역 요약."""
        parts = [f'{zz.zone_id}({zz.kind},{zz.source},{zz.area_m2:.1f}m2)'
                 for zz in self.zone_map.zones()]
        return ', '.join(parts) if parts else '(없음)'

    # ------------------------------------------------------------------ 관측 → 뷰
    def _make_view(self, now: float, ob: RobotObservation) -> RobotView:
        cfg = self.cfg
        rid = ob.robot_id
        plan = as_path(ob.plan) if ob.plan is not None else None
        if plan is not None and len(plan) < 2:
            plan = None
        yg = self._yield_goal.get(rid)
        if plan is not None and not (
                yg is not None
                and math.hypot(plan[-1, 0] - yg[0], plan[-1, 1] - yg[1]) <= YIELD_GOAL_TOL_M):
            self._intent_plan[rid] = plan          # 포켓으로 가는 경로가 아니면 목표 경로
        intent_plan = self._intent_plan.get(rid) if plan is not None else None
        path = intent = None
        active = False
        if intent_plan is not None:
            intent, remaining = _remaining(intent_plan, ob.x, ob.y, cfg.max_path_m)
            path = intent if plan is intent_plan else _remaining(plan, ob.x, ob.y,
                                                                 cfg.max_path_m)[0]
            goal = (float(intent_plan[-1, 0]), float(intent_plan[-1, 1]))
            active = (remaining > cfg.goal_tolerance_m and ob.status not in BUSY_STATIONARY
                      and (ob.status == ST_MOVING or bool(ob.task_id)))
            prev = self._goals.get(rid)
            if prev is None or math.hypot(goal[0] - prev[0], goal[1] - prev[1]) > 0.5:
                self.incidents.reset_robot(rid)
            self._goals[rid] = goal
        if not active:
            path = intent = None
            if rid in self._goals and ob.status not in BUSY_STATIONARY:
                self._goals.pop(rid)
                self.incidents.reset_robot(rid)
        idle = not active and not ob.task_id and ob.status == ST_IDLE
        zones = self._body_zones(ob.x, ob.y, ob.yaw)
        m = self._motion.get(rid)
        moved = m is None or math.hypot(ob.x - m.last_xy[0], ob.y - m.last_xy[1]) > \
            max(0.05, cfg.deadlock.stationary_speed * (now - m.last_t))
        if m is None:
            m = self._motion[rid] = _Motion((ob.x, ob.y), now, now)
        if moved or abs(ob.speed) > cfg.deadlock.stationary_speed:
            m.still_since = now
        m.last_xy, m.last_t = (ob.x, ob.y), now
        return RobotView(rid, ob.x, ob.y, ob.yaw, abs(ob.speed), path, active, idle,
                         ob.priority, ob.deadline, zones, intent, ob.status not in IMMOBILE)

    def _body_zones(self, x: float, y: float, yaw: float) -> FrozenSet[str]:
        if not len(self.zone_map):
            return frozenset()
        r = self.cfg.robot_radius
        pts = [(x, y)] + [(x + r * math.cos(yaw + k * math.pi / 2),
                           y + r * math.sin(yaw + k * math.pi / 2)) for k in range(4)]
        return frozenset(z for z in self.zone_map.zones_at(np.array(pts)) if z)

    def _stale_view(self, ob: RobotObservation) -> RobotView:
        """관측이 끊긴 로봇: 마지막 자세에 멈춘 장애물 (주행 · 유휴 아님, 움직일 수 없음)."""
        return RobotView(ob.robot_id, ob.x, ob.y, ob.yaw, 0.0, None, False, False, ob.priority,
                         ob.deadline, self._body_zones(ob.x, ob.y, ob.yaw), None, False, True)

    def still_for(self, robot_id: str, now: float) -> float:
        """정지 지속 시간 [s]."""
        m = self._motion.get(robot_id)
        return 0.0 if m is None else now - m.still_since

    # ------------------------------------------------------------------ 주기
    def update(self, now: float, observations: Mapping[str, RobotObservation]) -> TickResult:
        """한 주기: 예측 → 토큰 → 교차 양보 → wait-for → 교착/정체 → 해소 → 명령."""
        cfg = self.cfg
        acting = self.active_mode
        views: Dict[str, RobotView] = {}
        for rid, ob in sorted(observations.items()):
            if ob.stamp is None or now - ob.stamp <= cfg.obs_timeout_s:
                views[rid] = self._make_view(now, ob)
            else:
                views[rid] = self._stale_view(ob)     # 빼지 않는다 (fail closed)
            self._status[rid] = ob.status
        for rid in [r for r in set(self._motion) | set(self._status) if r not in observations]:
            self._motion.pop(rid, None)              # 관측 목록에서 아예 빠진 로봇만 잊는다
            self.tokens.forget(rid)
            self._intent_plan.pop(rid, None)
            self._status.pop(rid, None)
            self._stale.pop(rid, None)
            self._stale_alerted.discard(rid)
        self.views = views
        events: List[TrafficEvent] = self._stale_events(now, observations, views)
        world = WorldView(views, self.zone_map if len(self.zone_map) else None, self.tgrid,
                          self.pockets, self.mask_spec, self._held_zones(views))

        # 사건 진행 (지난 주기 명령의 결과를 먼저 본다)
        events += self._convert(self.incidents.step(now, world))
        inc_cmds = self.incidents.commands()
        yielding = {r for r, c in inc_cmds.items() if c.hold}
        tokens_on = acting and cfg.zones.tokens and len(self.zone_map) > 0
        if tokens_on:
            self._sync_yield_tokens(now, views, yielding)

        # 2) 충돌 예측 — 목표 경로 기준 (양보 중인 로봇 · 앞 로봇에 막힌 줄은 제자리)
        queued = self._queued(now, views, yielding)
        trajs: List[Trajectory] = []
        for rid, v in views.items():
            moving = v.active and rid not in yielding and rid not in queued
            trajs.append(predict_trajectory(
                rid, v.x, v.y, v.yaw, v.intent_path() if moving else None, cfg.nominal_speed,
                cfg.prediction.horizon_s, cfg.prediction.dt_s, 0.0))
        p = cfg.prediction
        conflicts = predict_conflicts(trajs, cfg.safety_distance, p.time_window_s,
                                      p.head_on_angle_deg, p.following_angle_deg)
        moving_ids = {t.robot_id for t in trajs if t.moving}

        # 3) 구역 토큰
        token_waits: Dict[str, TokenDecision] = {}
        request_zones: Dict[str, Tuple[str, ...]] = {}
        if tokens_on:
            token_waits, request_zones = self._tokens(now, views, yielding)
            world.held_zones = self._held_zones(views)

        # 4) 교차 양보
        crossing: Dict[str, str] = {}
        if acting and p.crossing_hold:
            crossing = self._crossing_holds(now, conflicts, views, moving_ids, yielding,
                                            token_waits)

        # 5) wait-for 그래프 → 교착 확정
        graph = self._wait_for(now, views, token_waits, crossing, yielding)
        comps = [c for c in deadlock_components(graph)
                 if not set(c) & self.incidents.members()
                 and not self.incidents.in_cooldown(c, now)]
        for sig in self._confirmer.update(now, comps):
            members = sorted(sig)
            headings = {r: self._heading(views[r]) for r in members}
            kind = classify_deadlock(members, headings, p.head_on_angle_deg)
            zones = set().union(*(views[r].zones for r in members))
            for r in members:
                zones |= set(request_zones.get(r, ()))
            opened = self._open(now, members, kind, world, zones)
            edges = ','.join(f'{s}→{d}:{why}' for s, d, why in graph.edges()
                             if s in sig and d in sig)
            for ev in opened:
                if ev.name == TRAFFIC_DEADLOCK:
                    ev.values['edges'] = edges      # 사이클 간선과 이유 (진단 · 대시보드)
            events += opened
        if not acting:
            events += self._observe_cleared(now)

        # 6) 사이클 없는 막힘 · 정체 · 긴 토큰 대기
        events += self._stalls(now, views, graph, token_waits, crossing, yielding, world)
        events += self._token_watch(now, views, token_waits)

        # 7) 명령 합성
        commands = self._compose(views, token_waits, crossing)
        self._yield_goal = {r: (c.yield_pose[0], c.yield_pose[1])
                            for r, c in commands.items() if c.yield_pose is not None}
        self._held_prev = {r for r, c in commands.items() if c.hold}
        return TickResult(commands, events, conflicts, graph.edges(), token_waits)

    # ------------------------------------------------------------------ 세부 단계
    def _held_zones(self, views: Mapping[str, RobotView]) -> Dict[str, FrozenSet[str]]:
        """로봇 → 보유(토큰) · 점유(몸체) 구역."""
        held = self.tokens.holdings()
        return {rid: frozenset(held.get(rid, ())) | v.zones for rid, v in views.items()
                if held.get(rid) or v.zones}

    def _state_name(self, rid: str) -> str:
        if rid in self._stale:
            return self._stale[rid]
        return STATUS_NAMES.get(self._status.get(rid, -1), '?')

    def _stale_events(self, now: float, observations: Mapping[str, RobotObservation],
                      views: Mapping[str, RobotView]) -> List[TrafficEvent]:
        """관측 끊김 상태 전이: fresh → stale(WARN) → lost(ERROR) → recovered(OK)."""
        out: List[TrafficEvent] = []
        for rid, v in views.items():
            prev = self._stale.get(rid)
            held = self.tokens.held_by(rid)
            if not v.stale:
                if prev is None:
                    continue
                del self._stale[rid]
                alerted = rid in self._stale_alerted
                self._stale_alerted.discard(rid)
                out.append(TrafficEvent(
                    TRAFFIC_STALE, LEVEL_OK, f'{rid} 관측 재수신 (recovered)', (rid,),
                    {'robots': rid, 'state': 'recovered', 'zones': ','.join(held)},
                    ALERT_TRAFFIC_STALE if alerted else '', rid))
                continue
            age = now - float(observations[rid].stamp)
            state = LOST if age > self.cfg.lost_timeout_s else STALE
            if state == prev:
                continue
            self._stale[rid] = state
            if held:
                self._stale_alerted.add(rid)
            values = {'robots': rid, 'state': state, 'age_s': f'{age:.1f}',
                      'zones': ','.join(held), 'body_zones': ','.join(sorted(v.zones)),
                      'x': f'{v.x:.2f}', 'y': f'{v.y:.2f}'}
            if state == LOST:
                kept = [z for z in held if z in v.zones]
                released = [z for z in held if z not in v.zones]
                values['released'] = ','.join(released)
                msg = (f'{rid} 관측 {age:.1f} s 없음 (lost) — 마지막 자세 장애물 유지, '
                       f'몸체 구역 토큰 {",".join(kept) or "없음"} 유지'
                       + (f', {",".join(released)} 반납' if released else ''))
                level = LEVEL_ERROR
            else:
                msg = (f'{rid} 관측 {age:.1f} s 없음 (stale) — 마지막 자세에 멈춘 장애물, '
                       f'토큰 {",".join(held) or "없음"} 유지')
                level = LEVEL_WARN
            out.append(TrafficEvent(TRAFFIC_STALE, level, msg, (rid,), values,
                                    ALERT_TRAFFIC_STALE if rid in self._stale_alerted else '',
                                    rid))
        return out

    def _sync_yield_tokens(self, now: float, views: Mapping[str, RobotView],
                           yielding: Set[str]) -> None:
        """포켓으로 가는 희생 로봇에 포켓 경로 위 (비어 있는) 구역 토큰을 미리 부여한다."""
        for rid in [r for r in self._yield_tokens if r not in yielding]:
            del self._yield_tokens[rid]
        for inc in self.incidents.active():
            rid, pocket = inc.victim, inc.pocket
            if rid not in yielding or pocket is None or rid not in views or len(pocket.route) < 2:
                continue
            key = (inc.incident_id, pocket.pocket_id)
            if self._yield_tokens.get(rid, (None,))[0] == key:
                continue
            visits = [vi for vi in zone_visits(np.array(pocket.route), self.zone_map, 0.2)
                      if vi.zone_id not in views[rid].zones]
            want = list(dict.fromkeys(vi.zone_id for vi in visits))
            dirs = {vi.zone_id: vi.direction() for vi in visits}
            if self.tokens.reserve(now, rid, want, dirs):     # 실패하면 다음 주기에 다시
                self._yield_tokens[rid] = (key, tuple(want))

    def _queued(self, now: float, views: Mapping[str, RobotView], yielding: Set[str]) -> Set[str]:
        """앞 로봇 몸체에 막혀 stationary_time_s 넘게 서 있는 주행 로봇 (traffic hold 로 선 로봇 제외)."""
        d = self.cfg.deadlock
        out: Set[str] = set()
        for rid, v in views.items():
            if not v.active or rid in yielding or rid in self._held_prev \
                    or self.still_for(rid, now) < d.stationary_time_s:
                continue
            if self._blockers(v, views):
                out.add(rid)
        return out

    @staticmethod
    def _heading(v: RobotView) -> float:
        path = v.intent_path()
        if path is not None and len(path) >= 2:
            d = path[min(len(path) - 1, 3)] - path[0]
            if math.hypot(d[0], d[1]) > 1e-3:
                return math.atan2(d[1], d[0])
        return v.yaw

    def _ahead(self, v: RobotView, length: float) -> Optional[np.ndarray]:
        if v.path is None or len(v.path) < 2:
            return None
        cum = cumulative_length(v.path)
        return sub_path(v.path, cum, 0.0, min(float(cum[-1]), length), 0.1)

    def _tokens(self, now: float, views: Mapping[str, RobotView], yielding: Set[str]
                ) -> Tuple[Dict[str, TokenDecision], Dict[str, Tuple[str, ...]]]:
        cfg = self.cfg
        requests, occupancy, upcoming, dirs = {}, {}, {}, {}
        request_zones: Dict[str, Tuple[str, ...]] = {}
        frozen = {rid for rid, v in views.items() if v.stale and self._stale.get(rid) != LOST}
        for rid, v in views.items():
            occupancy[rid] = set(v.zones)
            upcoming[rid] = set(self._yield_tokens[rid][1]) if rid in self._yield_tokens else set()
            if v.path is None or not v.active:
                continue
            visits = zone_visits(v.path, self.zone_map, 0.2)
            upcoming[rid] |= {vi.zone_id for vi in visits}
            dirs[rid] = {vi.zone_id: vi.direction() for vi in visits if vi.s_in <= 1e-6}
            if rid in yielding:
                continue
            chain = request_chain(visits, self._approach, cfg.zones.chain_gap_m, v.zones)
            if chain:
                requests[rid] = TokenRequest(rid, tuple(c.zone_id for c in chain),
                                             {c.zone_id: c.direction() for c in chain},
                                             v.priority, v.deadline)
                request_zones[rid] = requests[rid].zones
        waits = {rid: dec for rid, dec in self.tokens.update(now, requests, occupancy, upcoming,
                                                             dirs, frozen).items()
                 if not dec.granted}
        return waits, request_zones

    def _crossing_holds(self, now: float, conflicts: Sequence[Conflict],
                        views: Mapping[str, RobotView], moving: Set[str], yielding: Set[str],
                        token_waits: Mapping[str, TokenDecision]) -> Dict[str, str]:
        """교차 예측 → 양보 로봇 → 상대. 한 번 건 hold 는 예측 충돌이 사라질 때까지 유지 (래치)."""
        p = self.cfg.prediction
        out: Dict[str, str] = {}
        seen: Set[FrozenSet[str]] = set()
        tokens_on = self.cfg.zones.tokens and len(self.zone_map) > 0
        busy = self.incidents.members() | yielding | set(token_waits)
        # 구역 안에 몸체가 있거나 들어간 구역 토큰을 쥔 로봇은 세우지 않는다 (구역 · 토큰을 쥔 채 서면
        # 그 토큰을 기다리는 로봇과 교착이 된다 — 1차선 통로 한가운데 등)
        pinned = {r for r, v in views.items() if v.zones or self.tokens.entered_by(r)}
        for c in conflicts:
            a, b = c.robot_a, c.robot_b
            if c.kind != CROSSING or a not in moving or b not in moving or {a, b} & busy:
                continue
            if tokens_on and self.zone_map.zone_at(c.x, c.y) is not None:
                continue          # 구역 안 충돌은 토큰이 맡는다
            pair = frozenset((a, b))
            latch = self._latches.get(pair)
            if latch is not None and latch.yielder in pinned:
                latch = None
                del self._latches[pair]
            if latch is None:
                # 덜 중요한 쪽부터, 세울 수 있는(구역 밖 · crossing_min_s 이상) 로봇이 양보
                cands = [r for r in sorted((a, b), key=lambda r: views[r].rank(), reverse=True)
                         if r not in pinned and c.time_of(r) >= p.crossing_min_s]
                if not cands:
                    continue
                y = cands[0]
                o = b if y == a else a
                ty = c.time_of(y)
                if ty > p.crossing_act_s or y in out:
                    continue
                ahead = self._ahead(views[o], 3.0)
                if ahead is not None and distance_to_path(np.array([views[y].xy]), ahead)[0] < \
                        self.cfg.safety_distance:
                    continue      # 양보자가 이미 상대 앞길에 있으면 세우면 막힌다
                if _leads_to(out, o, y):
                    continue      # 교차 양보끼리 순환하면(역전 · 고정으로 순서가 뒤집힌 경우) 모두 선다
                latch = self._latches[pair] = _CrossLatch(y, o, now)
            seen.add(pair)
            if now - latch.since <= p.crossing_max_hold_s and latch.yielder not in out \
                    and not _leads_to(out, latch.other, latch.yielder):
                out[latch.yielder] = latch.other
        for k in [k for k in self._latches if k not in seen]:
            del self._latches[k]
        return out

    def _blockers(self, v: RobotView, views: Mapping[str, RobotView]) -> List[str]:
        """
        로봇 v 의 남은 경로 앞 block_lookahead_m 안, 통로 폭 안에 몸체가 있는 로봇.

        경로 위 투영이 몸 앞쪽 절반(0.5 r) 너머인 로봇 — 경로가 곧바로 꺾여 옆에 선 로봇 쪽으로 가는 경우
        (구역 출구 바로 옆에서 기다리는 로봇 등)도 잡는다. 옆 · 뒤(투영 ≈ 0)는 뺀다.
        """
        d = self.cfg.deadlock
        ahead = self._ahead(v, d.block_lookahead_m)
        if ahead is None:
            return []
        cum = cumulative_length(ahead)
        lateral = 2.0 * self.cfg.robot_radius + d.block_lateral_margin_m
        out = []
        for rid, o in views.items():
            if rid == v.robot_id:
                continue
            s, dist = project(ahead, cum, o.x, o.y)
            if dist < lateral and s > 0.5 * self.cfg.robot_radius:
                out.append(rid)
        return out

    def _wait_for(self, now: float, views: Mapping[str, RobotView],
                  token_waits: Mapping[str, TokenDecision], crossing: Mapping[str, str],
                  yielding: Set[str]) -> WaitForGraph:
        d = self.cfg.deadlock
        g = WaitForGraph()
        for rid in views:
            g.add_node(rid)
        for rid, dec in token_waits.items():
            for b in dec.blockers:
                if b in views:
                    g.add(rid, b, EDGE_TOKEN)
        for y, o in crossing.items():
            g.add(y, o, EDGE_CROSSING)
        for rid, v in views.items():
            if not v.active or rid in yielding or rid in token_waits or rid in crossing:
                continue
            if self.still_for(rid, now) < d.stationary_time_s:
                continue
            for b in self._blockers(v, views):
                if views[b].speed <= d.stationary_speed and b not in yielding:
                    g.add(rid, b, EDGE_BLOCK)
        return g

    def _open(self, now: float, members: Sequence[str], kind: str, world: WorldView,
              zones: Set[str]) -> List[TrafficEvent]:
        """확정된 교착: active 면 해소 사건, observe 면 탐지 보고만."""
        if self.active_mode:
            return self._convert(self.incidents.open(now, members, kind, world, zones))
        sig = frozenset(members)
        # 막힘 에피소드: 보고한 로봇은 진전(마지막 진전 시각이 바뀜)할 때까지 다시 세지 않는다.
        # 막힘 간선은 로봇이 제자리에서 조금만 움직여도 끊겼다 이어지고, 같은 대치가 사이클 ↔ 정체로
        # 모습을 바꾸므로 묶음 단위로 세면 한 대치가 여러 번 잡힌다
        watch = [r for r in members if r in self.views and self.views[r].active]
        new = [r for r in watch if r not in self._stall_episode]
        if kind in (DL_BLOCKED, DL_LIVELOCK):
            for r in watch:
                self._stall_episode.setdefault(r, self._stall.anchor_time(r))
            if not new:
                return []
            self._stall_groups[frozenset(watch)] = now
        else:
            # 같은 교착이 몇 주기 끊겼다 이어지거나 한 대가 붙고 빠져도(부분/상위 집합) 새로 세지 않는다
            same = [s for s in self._observed if s <= sig or sig <= s]
            for s in same:
                self._last_seen[s] = now
            if same or (watch and not new):
                return []
            for r in watch:
                self._stall_episode.setdefault(r, self._stall.anchor_time(r))
            self._observed[sig] = now
            self._last_seen[sig] = now
        self.stats['deadlocks'] += 1
        values = {'robots': ','.join(members), 'type': kind, 'strategy': 'none',
                  'zones': ','.join(sorted(zones))}
        return [TrafficEvent(TRAFFIC_DEADLOCK, LEVEL_ERROR,
                             f'교착 {kind}: {" ↔ ".join(members)} (observe — 해소 안 함)',
                             tuple(members), values, ALERT_DEADLOCK, members[0])]

    def _end_stall_episodes(self, now: float, views: Mapping[str, RobotView]
                            ) -> List[TrafficEvent]:
        """
        관찰(observe) 모드: 진전(마지막 진전 시각이 바뀜)·목표 없음·관측 없음이면 막힘 에피소드 끝.

        보고한 막힘 묶음의 로봇이 모두 끝나면 RESOLVED(strategy=none).
        """
        for rid in list(self._stall_episode):
            v = views.get(rid)
            if v is None or not v.active or \
                    self._stall.anchor_time(rid) != self._stall_episode[rid]:
                del self._stall_episode[rid]
        out = []
        for sig, t0 in list(self._stall_groups.items()):
            if sig & set(self._stall_episode):
                continue
            del self._stall_groups[sig]
            members = sorted(sig)
            dt = now - t0
            self.stats['resolved'] += 1
            self.stats['resolve_times'].append(dt)
            out.append(TrafficEvent(TRAFFIC_RESOLVED, LEVEL_OK, f'막힘 저절로 풀림 {dt:.1f} s',
                                    tuple(members), {'robots': ','.join(members),
                                                     'strategy': 'none',
                                                     'resolve_time_s': f'{dt:.2f}'},
                                    ALERT_DEADLOCK, members[0]))
        return out

    def _observe_cleared(self, now: float) -> List[TrafficEvent]:
        """관찰(observe) 모드: 보고한 교착이 clear_s 동안 안 보이고 모두 진전하면 RESOLVED."""
        present = set(self._confirmer.pending(now)) | self._confirmer.reported()
        out = []
        for sig in list(self._observed):
            if any(s <= sig or sig <= s for s in present):
                self._last_seen[sig] = now
                continue
            if now - self._last_seen.get(sig, now) < self.cfg.deadlock.clear_s:
                continue          # 몇 주기 끊긴 것은 같은 교착으로 본다 (깜박임 재보고 방지)
            if sig & set(self._stall_episode):
                continue          # 사이클 모양은 사라졌어도 아직 진전하지 않은 로봇이 있다
            t0 = self._observed.pop(sig)
            self._last_seen.pop(sig, None)
            members = sorted(sig)
            dt = now - t0         # 탐지 → 풀림 판정 (clear_s 이력 포함)
            self.stats['resolved'] += 1
            self.stats['resolve_times'].append(dt)
            out.append(TrafficEvent(TRAFFIC_RESOLVED, LEVEL_OK,
                                    f'교착 저절로 풀림 {dt:.1f} s', tuple(members),
                                    {'robots': ','.join(members), 'strategy': 'none',
                                     'resolve_time_s': f'{dt:.2f}'},
                                    ALERT_DEADLOCK, members[0]))
        return out

    def _stalls(self, now: float, views: Mapping[str, RobotView], graph: WaitForGraph,
                token_waits: Mapping[str, TokenDecision], crossing: Mapping[str, str],
                yielding: Set[str], world: WorldView) -> List[TrafficEvent]:
        d = self.cfg.deadlock
        events: List[TrafficEvent] = []
        busy = self.incidents.members()
        # 진전 감시 (hold 중인 로봇과, block 간선으로 hold 중인 로봇 뒤에 줄 선 로봇은 감시하지 않는다 —
        # 그 기다림은 토큰 대기 감시 · 교차 양보 상한 · 사건이 맡는다)
        held = set(token_waits) | set(crossing) | yielding
        behind = graph.predecessors_closure(held, EDGE_BLOCK)
        stalled = []
        for rid, v in views.items():
            watch = v.active and rid not in busy and rid not in held and rid not in behind
            if self._stall.update(rid, now, v.intent_remaining_m(),
                                  self._goals.get(rid), watch):
                stalled.append(rid)
            elif rid in self._stall_reported:
                del self._stall_reported[rid]
        events += self._end_stall_episodes(now, views)
        # 유휴(주차) · 움직일 수 없는(E-stop · 오류 · stale) 로봇이 막음 → BLOCKED {대기 로봇, 그 로봇들}
        for rid in sorted(views):
            if rid in busy or not views[rid].active:
                continue
            idle_targets = [b for b in graph.successors(rid)
                            if (views[b].idle or not views[b].mobile) and b not in busy]
            if idle_targets and self.still_for(rid, now) >= d.idle_block_s:
                members = [rid] + idle_targets
                if self.incidents.in_cooldown(members, now):
                    continue
                zones = set(views[rid].zones)
                zones |= set().union(*(views[b].zones for b in idle_targets))
                events += self._open(now, members, DL_BLOCKED, world, zones)
                busy = self.incidents.members()
        stalled = [r for r in stalled if r not in busy]     # 방금 BLOCKED 사건에 묶인 로봇은 뺀다
        # 가까이 모인 정체 로봇 묶음 → LIVELOCK
        groups: List[List[str]] = []
        for rid in sorted(stalled):
            for grp in groups:
                if any(math.hypot(views[rid].x - views[o].x, views[rid].y - views[o].y)
                       <= d.livelock_radius_m for o in grp):
                    grp.append(rid)
                    break
            else:
                groups.append([rid])
        for grp in groups:
            if len(grp) >= 2 and not self.incidents.in_cooldown(grp, now):
                zones = set().union(*(views[r].zones for r in grp))
                events += self._open(now, grp, DL_LIVELOCK, world, zones)
                if self.active_mode:
                    for r in grp:
                        self._stall.reset(r)
            elif len(grp) == 1:
                events += self._single_stall(now, grp[0], graph.successors(grp[0]))
        return events

    def _single_stall(self, now: float, rid: str, blockers: Sequence[str]) -> List[TrafficEvent]:
        """1대 정체: 한 번 traffic/STALL, wait_max_s 넘게 이어지면 (active) traffic/UNRESOLVED."""
        level = self._stall_reported.get(rid, 0)
        stalled = self._stall.stalled_for(rid, now)
        values = {'robots': rid, 'blockers': ','.join(blockers),
                  'states': ','.join(self._state_name(b) for b in blockers)}
        who = ', '.join(f'{b}({self._state_name(b)})' for b in blockers) or '없음'
        out: List[TrafficEvent] = []
        if level < 1:
            self._stall_reported[rid] = level = 1
            out.append(TrafficEvent(TRAFFIC_STALL, LEVEL_WARN,
                                    f'{rid} 진전 없음 {stalled:.0f} s (막는 로봇: {who})', (rid,),
                                    values, '', rid))
        if level < 2 and self.active_mode and stalled >= self.cfg.deadlock.wait_max_s:
            self._stall_reported[rid] = 2
            self.stats['unresolved'] += 1
            values.update(robots=','.join([rid] + list(blockers)), type='STALL',
                          reason='wait_max', waiting_s=f'{stalled:.1f}')
            out.append(TrafficEvent(TRAFFIC_UNRESOLVED, LEVEL_ERROR,
                                    f'{rid} 진전 없음 {stalled:.0f} s — 해소 실패 (막는 로봇: {who})',
                                    tuple([rid] + list(blockers)), values, ALERT_DEADLOCK, rid))
        return out

    def _token_watch(self, now: float, views: Mapping[str, RobotView],
                     token_waits: Mapping[str, TokenDecision]) -> List[TrafficEvent]:
        """
        토큰 대기 감시 (대기 로봇은 정체 감시에서 빠지므로 따로 본다).

        대기가 token_wait_stall_s 이어지면 traffic/STALL, wait_max_s 면 traffic/UNRESOLVED
        (fleet/DEADLOCK ERROR) — 대기마다 한 번씩, 막는 로봇과 그 상태(ESTOP · LOADING · stale 등)를 적는다.
        사건에 묶인 로봇은 사건이 맡는다.
        """
        d = self.cfg.deadlock
        out: List[TrafficEvent] = []
        for rid in [r for r in self._token_watch_level if r not in token_waits]:
            del self._token_watch_level[rid]
        members = self.incidents.members() | self.incidents.suppressed()
        for rid, dec in sorted(token_waits.items()):
            if rid in members:
                continue          # 사건이 맡는다 (움직일 수 없는 로봇 때문에 이미 실패 알림을 낸 경우 포함)
            level = self._token_watch_level.get(rid, 0)
            blockers = ', '.join(f'{b}({self._state_name(b)})' for b in dec.blockers)
            values = {'robots': ','.join([rid] + [b for b in dec.blockers if b != rid]),
                      'blockers': ','.join(dec.blockers),
                      'states': ','.join(self._state_name(b) for b in dec.blockers),
                      'zones': ','.join(dec.zones), 'waiting_s': f'{dec.waiting_s:.1f}',
                      'type': 'TOKEN_WAIT'}
            if level < 1 and dec.waiting_s >= d.token_wait_stall_s:
                self._token_watch_level[rid] = level = 1
                out.append(TrafficEvent(
                    TRAFFIC_STALL, LEVEL_WARN,
                    f'{rid} 구역 {"+".join(dec.zones)} 토큰 {dec.waiting_s:.0f} s 대기 '
                    f'(막는 로봇: {blockers or "기아 예약"})', (rid,), dict(values), '', rid))
            if level < 2 and dec.waiting_s >= d.wait_max_s:
                self._token_watch_level[rid] = 2
                self.stats['unresolved'] += 1
                values['reason'] = 'token_wait_max'
                out.append(TrafficEvent(
                    TRAFFIC_UNRESOLVED, LEVEL_ERROR,
                    f'{rid} 구역 {"+".join(dec.zones)} 토큰 {dec.waiting_s:.0f} s 대기 — 해소 실패 '
                    f'(막는 로봇: {blockers or "기아 예약"})',
                    tuple(values['robots'].split(',')), values, ALERT_DEADLOCK, rid))
        return out

    def _compose(self, views: Mapping[str, RobotView],
                 token_waits: Mapping[str, TokenDecision],
                 crossing: Mapping[str, str]) -> Dict[str, RobotCommand]:
        cmds = {rid: RobotCommand() for rid in views}
        if not self.active_mode:
            return cmds
        inc = self.incidents.commands()
        for rid, cmd in cmds.items():
            ic = inc.get(rid)
            if ic is not None:
                cmd.incident_id = ic.incident_id
                cmd.keepout = ic.keepout
                cmd.yield_pose = ic.yield_pose
                if ic.hold:
                    cmd.hold, cmd.reason = True, f'yield:incident{ic.incident_id}'
                    continue
            if rid in token_waits:
                # 마스크가 이미 막는 구역을 기다리는 중이면 토큰 hold 를 걸지 않는다: 실행기는 hold 동안
                # 주행 goal 을 취소하므로(move_to.xml) 세워 두면 재계획 자체를 못 해 ALT_PATH 가
                # 영원히 성립하지 않는다 (통합 시나리오 12 강제 교착: 마스크를 주고도 no_alt_path).
                # 못 들어가게 막는 일은 마스크가 한다.
                blockers = [views[b] for b in token_waits[rid].blockers if b in views]
                if ic is not None and ic.keepout is not None \
                        and set(token_waits[rid].zones) <= set(ic.masked_zones) \
                        and not any(b.stale for b in blockers):
                    # 관측이 끊긴 보유자(stale)면 hold 를 유지한다 (fail-closed): 그 로봇의 몸체·토큰은
                    # 남아 있지만 실제 위치를 모르므로 마스크만 믿고 풀어 줄 수 없다.
                    continue
                cmd.hold = True
                cmd.reason = 'zone:' + '+'.join(token_waits[rid].zones)
            elif rid in crossing:
                cmd.hold, cmd.reason = True, f'crossing:{crossing[rid]}'
        return cmds

    def _convert(self, evs: Sequence[ResolutionEvent]) -> List[TrafficEvent]:
        out = []
        for ev in evs:
            inc = ev.incident
            if ev.kind == EV_DEADLOCK:
                self.stats['deadlocks'] += 1
                out.append(TrafficEvent(TRAFFIC_DEADLOCK, LEVEL_ERROR, ev.message, inc.robots,
                                        ev.values, ALERT_DEADLOCK, inc.victim))
            elif ev.kind == EV_ESCALATED:
                self.stats['escalations'] += 1
                out.append(TrafficEvent(TRAFFIC_ESCALATED, LEVEL_WARN, ev.message, inc.robots,
                                        ev.values, '', inc.victim))
            elif ev.kind == EV_RESOLVED:
                self.stats['resolved'] += 1
                self.stats['resolve_times'].append(ev.time - inc.detected_at)
                self.stats['strategies'][inc.strategy] += 1
                for r in inc.robots:
                    self._stall.reset(r)
                out.append(TrafficEvent(TRAFFIC_RESOLVED, LEVEL_OK, ev.message, inc.robots,
                                        ev.values, ALERT_DEADLOCK, inc.victim))
            elif ev.kind == EV_UNRESOLVED:
                self.stats['unresolved'] += 1
                for r in inc.robots:
                    self._stall.reset(r)
                out.append(TrafficEvent(TRAFFIC_UNRESOLVED, LEVEL_ERROR, ev.message, inc.robots,
                                        ev.values, ALERT_DEADLOCK, inc.victim))
        return out

    def resolve_time_stats(self) -> Dict[str, float]:
        """해소 시간 요약 (최근 1000건): 건수 · 평균 · 최대 [s]."""
        times: Deque[float] = self.stats['resolve_times']
        if not times:
            return {'count': 0, 'mean_s': 0.0, 'max_s': 0.0}
        return {'count': len(times), 'mean_s': float(sum(times) / len(times)),
                'max_s': float(max(times))}
