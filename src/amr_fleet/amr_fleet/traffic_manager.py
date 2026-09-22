"""
교통 관리 본체 — traffic_manager_node 의 ROS 없는 로직 (components.md §3.6 / §5.6, sequences.md §3).

한 주기 update(now, 관측) (노드에서 2 Hz):
 1. 관측 → RobotView: plan 을 현재 위치에 투영한 남은 경로, 목표 경로(양보 중에는 원래 경로),
    정지 지속 시간, 몸체가 걸친 구역
 2. 충돌 예측: 목표 경로를 공칭 속도로 따라간다고 본 시공간 표본 → HEAD_ON / CROSSING / FOLLOWING
    (hold 로 서 있는 로봇도 "풀어 주면 갈 경로" 로 예측한다 — 풀었다 다시 거는 진동을 막는다)
 3. 구역 토큰: 교차로 · 1차선 통로 진입 approach 앞에서 요청 → 우선순위 · 마감 · 대기 순 부여,
    기아 방지 (traffic_zones). 못 받으면 traffic/hold
 4. 교차 양보: 구역 밖 CROSSING 예측은 우선순위 낮은 쪽을 충돌 crossing_act_s 전에 hold 하고,
    상대가 지나가 예측 충돌이 사라질 때까지(최대 crossing_max_hold_s) 유지한다
 5. wait-for 그래프 (token / crossing / block) → Tarjan SCC → confirm_s 유지 시 교착 확정
 6. 사이클 없는 막힘: 유휴 로봇이 경로·구역을 막음(BLOCKED), 진전 없는 정체 묶음(LIVELOCK)
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
    ALERT_DEADLOCK, TRAFFIC_DEADLOCK, TRAFFIC_ESCALATED, TRAFFIC_RESOLVED, TRAFFIC_STALL,
    TRAFFIC_UNRESOLVED,
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
    TokenDecision, TokenRequest, Zone, ZoneMap, ZoneTokenManager, derive_zones_from_grid,
    load_layout, request_chain, zone_visits,
)

# amr_msgs/msg/RobotState.STATUS_* (kpi.py 와 같은 값)
ST_IDLE, ST_MOVING, ST_DOCKING, ST_LOADING, ST_CHARGING, ST_ERROR, ST_ESTOP = range(7)
BUSY_STATIONARY = frozenset({ST_DOCKING, ST_LOADING, ST_CHARGING, ST_ERROR, ST_ESTOP})

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
    approach_distance_m: float = 2.5
    chain_gap_m: float = 1.5
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
    obs_timeout_s: float = 3.0
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
        positive = {
            'robot_radius': self.robot_radius, 'passage_radius': self.passage_radius,
            'nominal_speed': self.nominal_speed,
            'prediction.horizon_s': self.prediction.horizon_s,
            'prediction.dt_s': self.prediction.dt_s,
            'deadlock.confirm_s': self.deadlock.confirm_s + 1e-9,
            'deadlock.stall_time_s': self.deadlock.stall_time_s,
            'resolution.search_resolution_m': self.resolution.search_resolution_m,
            'resolution.keepout_resolution_m': self.resolution.keepout_resolution_m,
        }
        for k, v in positive.items():
            if not math.isfinite(v) or v <= 0.0:
                raise ValueError(f'{k} 는 양수여야 한다: {v}')
        if self.zones.max_bypass < 0:
            raise ValueError('zones.max_bypass 는 0 이상')
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
        self.zone_map = ZoneMap(self._yaml_zones)
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
        self._stall_reported: Set[str] = set()
        self._observed: Dict[FrozenSet[str], float] = {}      # observe 모드: 보고한 교착 → 탐지 시각
        self._last_seen: Dict[FrozenSet[str], float] = {}     # observe 모드: 마지막으로 보인 시각
        # observe 모드: 막힘(BLOCKED/LIVELOCK)을 보고한 로봇 → 그때의 마지막 진전 시각 (에피소드)
        self._stall_episode: Dict[str, Optional[float]] = {}
        self._stall_groups: Dict[FrozenSet[str], float] = {}  # observe 모드: 보고한 막힘 묶음 → 시각
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
        self.tgrid = TraversabilityGrid.from_occupancy(
            occupied, spec, cfg.robot_radius, cfg.resolution.search_resolution_m, self.zone_map,
            cfg.passage_radius)
        self.mask_spec = mask_spec_for(spec, cfg.resolution.keepout_resolution_m)

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
        zones = frozenset()
        if len(self.zone_map):
            r = cfg.robot_radius
            pts = [(ob.x, ob.y)] + [(ob.x + r * math.cos(ob.yaw + k * math.pi / 2),
                                     ob.y + r * math.sin(ob.yaw + k * math.pi / 2))
                                    for k in range(4)]
            zones = frozenset(z for z in self.zone_map.zones_at(np.array(pts)) if z)
        m = self._motion.get(rid)
        moved = m is None or math.hypot(ob.x - m.last_xy[0], ob.y - m.last_xy[1]) > \
            max(0.05, cfg.deadlock.stationary_speed * (now - m.last_t))
        if m is None:
            m = self._motion[rid] = _Motion((ob.x, ob.y), now, now)
        if moved or abs(ob.speed) > cfg.deadlock.stationary_speed:
            m.still_since = now
        m.last_xy, m.last_t = (ob.x, ob.y), now
        return RobotView(rid, ob.x, ob.y, ob.yaw, abs(ob.speed), path, active, idle,
                         ob.priority, ob.deadline, zones, intent)

    def still_for(self, robot_id: str, now: float) -> float:
        """정지 지속 시간 [s]."""
        m = self._motion.get(robot_id)
        return 0.0 if m is None else now - m.still_since

    # ------------------------------------------------------------------ 주기
    def update(self, now: float, observations: Mapping[str, RobotObservation]) -> TickResult:
        """한 주기: 예측 → 토큰 → 교차 양보 → wait-for → 교착/정체 → 해소 → 명령."""
        cfg = self.cfg
        acting = self.active_mode
        fresh = {rid: ob for rid, ob in observations.items()
                 if ob.stamp is None or now - ob.stamp <= cfg.obs_timeout_s}
        views = {rid: self._make_view(now, ob) for rid, ob in sorted(fresh.items())}
        for rid in [r for r in self._motion if r not in observations]:
            self._motion.pop(rid)
            self.tokens.forget(rid)
            self._intent_plan.pop(rid, None)
        self.views = views
        world = WorldView(views, self.zone_map if len(self.zone_map) else None, self.tgrid,
                          self.pockets, self.mask_spec)
        events: List[TrafficEvent] = []

        # 사건 진행 (지난 주기 명령의 결과를 먼저 본다)
        events += self._convert(self.incidents.step(now, world))
        inc_cmds = self.incidents.commands()
        yielding = {r for r, c in inc_cmds.items() if c.hold}

        # 2) 충돌 예측 — 목표 경로 기준 (양보 중인 로봇은 제자리)
        trajs: List[Trajectory] = []
        for rid, v in views.items():
            moving = v.active and rid not in yielding
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
        if acting and cfg.zones.tokens and len(self.zone_map):
            token_waits, request_zones = self._tokens(now, views, yielding)

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

        # 6) 사이클 없는 막힘 · 정체
        events += self._stalls(now, views, graph, token_waits, crossing, yielding, world)

        # 7) 명령 합성
        commands = self._compose(views, token_waits, crossing)
        self._yield_goal = {r: (c.yield_pose[0], c.yield_pose[1])
                            for r, c in commands.items() if c.yield_pose is not None}
        return TickResult(commands, events, conflicts, graph.edges(), token_waits)

    # ------------------------------------------------------------------ 세부 단계
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
        for rid, v in views.items():
            occupancy[rid] = set(v.zones)
            if v.path is None or not v.active:
                upcoming[rid] = set()
                continue
            visits = zone_visits(v.path, self.zone_map, 0.2)
            upcoming[rid] = {vi.zone_id for vi in visits}
            dirs[rid] = {vi.zone_id: vi.direction() for vi in visits if vi.s_in <= 1e-6}
            if rid in yielding:
                continue
            chain = request_chain(visits, cfg.zones.approach_distance_m, cfg.zones.chain_gap_m)
            if chain:
                requests[rid] = TokenRequest(rid, tuple(c.zone_id for c in chain),
                                             {c.zone_id: c.direction() for c in chain},
                                             v.priority, v.deadline)
                request_zones[rid] = requests[rid].zones
        waits = {rid: dec for rid, dec in self.tokens.update(now, requests, occupancy, upcoming,
                                                             dirs).items() if not dec.granted}
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
        for c in conflicts:
            a, b = c.robot_a, c.robot_b
            if c.kind != CROSSING or a not in moving or b not in moving or {a, b} & busy:
                continue
            if tokens_on and self.zone_map.zone_at(c.x, c.y) is not None:
                continue          # 구역 안 충돌은 토큰이 맡는다
            pair = frozenset((a, b))
            latch = self._latches.get(pair)
            if latch is None:
                y, o = sorted((a, b), key=lambda r: views[r].rank(), reverse=True)
                if c.time_of(y) < p.crossing_min_s:
                    y, o = o, y
                ty = c.time_of(y)
                if ty < p.crossing_min_s or ty > p.crossing_act_s or y in out:
                    continue
                ahead = self._ahead(views[o], 3.0)
                if ahead is not None and distance_to_path(np.array([views[y].xy]), ahead)[0] < \
                        self.cfg.safety_distance:
                    continue      # 양보자가 이미 상대 앞길에 있으면 세우면 막힌다
                latch = self._latches[pair] = _CrossLatch(y, o, now)
            seen.add(pair)
            if now - latch.since <= p.crossing_max_hold_s and latch.yielder not in out:
                out[latch.yielder] = latch.other
        for k in [k for k in self._latches if k not in seen]:
            del self._latches[k]
        return out

    def _blockers(self, v: RobotView, views: Mapping[str, RobotView]) -> List[str]:
        """로봇 v 의 남은 경로 앞 block_lookahead_m 안, 통로 폭 안에 몸체가 있는 로봇."""
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
            if dist < lateral and s >= self.cfg.robot_radius:
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
        # 진전 감시 (hold 중인 로봇은 감시하지 않는다)
        stalled = []
        for rid, v in views.items():
            watch = (v.active and rid not in busy and rid not in token_waits
                     and rid not in crossing and rid not in yielding)
            if self._stall.update(rid, now, v.intent_remaining_m(),
                                  self._goals.get(rid), watch):
                stalled.append(rid)
            elif rid in self._stall_reported:
                self._stall_reported.discard(rid)
        events += self._end_stall_episodes(now, views)
        # 유휴(주차) 로봇이 막음 → BLOCKED {대기 로봇, 유휴 로봇들}
        for rid in sorted(views):
            if rid in busy or not views[rid].active:
                continue
            idle_targets = [b for b in graph.successors(rid)
                            if views[b].idle and b not in busy]
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
            elif len(grp) == 1 and grp[0] not in self._stall_reported:
                rid = grp[0]
                self._stall_reported.add(rid)
                blockers = ','.join(graph.successors(rid))
                events.append(TrafficEvent(
                    TRAFFIC_STALL, LEVEL_WARN,
                    f'{rid} 진전 없음 {self._stall.stalled_for(rid, now):.0f} s '
                    f'(막는 로봇: {blockers or "없음"})', (rid,),
                    {'robots': rid, 'blockers': blockers}, '', rid))
        return events

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
