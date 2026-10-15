"""
경로 게이트 — 로봇 자신의 plan · 자세로 다음 구역 사슬을 정하고, 결정점 안에서 허가가 없으면 멈춤을 알린다.

보너스 4 b4-wire (p3.md §10.1), rclpy 비의존. 보고서: "로봇이 자기 /plan, 자세, 구역 다각형으로 다음 사슬 S와 결정점
P_d를 계산하고, 허가 상태는 같은 프로세스의 투표 모듈이 쥔다 … 진입선까지 거리가 제동거리 + 여유보다 짧은데 허가가
없으면 goal을 취소하고 정지한다 … 결정점에 다가가는 동안 미리 요청하면 정지 없이 통과할 수 있다."
서버 traffic_manager 의 사슬 규칙(traffic_zones.zone_visits · request_chain — 순수 Python)을 그대로 로봇 쪽에서 쓴다.

한 번의 decide() 가 내는 것:
- request  : 지금 투표로 요청할 사슬 ('zone:<id>', 같은 구역 재방문은 한 번) — 입구가 요청 거리 + prerequest_m 안인
             첫 미진입 구역부터 chain_gap_m 이하로 붙은 구역까지. 몸체가 구역 k 에 걸쳐 있으면 k 와 묶이지 않은 다음
             구역은 k 를 나온 뒤에 요청한다 (구역 안에서 기다리지 않기 — request_chain 의 occupied 규칙).
- blocking : 지금 허가 없이는 움직이면 안 되는 미보유 구역 — 사슬 첫 입구가 그 구역의 요청 거리(결정점 거리,
             traffic.yaml approach_distance_m 2.0 · 통로 2.5) 안인데 사슬 전부를 쥐지 못했다. 사슬은 전부-아니면-전무
             (첫 구역만 쥐고 들어가 그 안에서 서지 않게).
- ahead    : 남은 경로가 지나는(지금 안 포함) 구역 — 이것과 사슬 · 몸체 밖의 보유 구역은 해제 후보다.
- body     : 몸체(중심 + 외접 반경 네 점, traffic_manager._body_zones 와 같은 규칙)가 걸친 구역.
움직이는 단계(active)가 아니면 경로를 보지 않는다 (사슬 없음 → 진행 중 요청 취소 · 보유 해제 후보). 자세를 모르는데
움직이는 단계면 blocking = (UNKNOWN_POSE,) — 구역을 판정할 수 없으면 움직이지 않는다 (fail-closed).

몸체 판정 (body_entry, b4-p5b — p5.md §0-0 · §5-1): 위 규칙은 구역 진입을 경로 **중심점**으로 쟀다. 그래서 경로가 구역 변을
따라가거나 모서리를 비스듬히 지나면 (1) 중심이 들어가지 않는 구역은 요청하지 않고 몸체가 경계를 스쳤고 (P5 사건 13 건),
(2) 막힘 거리(사슬 첫 입구까지)가 중심 진입까지라 막혀 선 자리에서 몸체가 이미 구역 안이었다 (2 건, 결정점 정지 −0.11 ·
−0.23 m). body_entry 면: 방문 · 요청 거리를 원판(robot_radius)이 구역에 처음 닿는 곳으로 재고 (zone_visits radius),
body 는 원판-다각형 겹침으로 재며, 허가 없이 이미 걸친 구역은 거리 0 으로 요청 · 막는다 (request_chain held — fail-closed).
traffic.yaml zones.body_entry 가 켠다 (load_path_gate — 키가 없으면 켬, 중앙 TrafficConfig 기본값과 같다).
PathGateConfig 를 직접 만들 때의 기본값 False 는 옛 중심점 판정 그대로다 (기존 단위 시험 · 비교용).
"""

from __future__ import annotations

import dataclasses
import math
from typing import Collection, FrozenSet, Mapping, Optional, Sequence, Tuple

import numpy as np

from amr_fleet.traffic_geometry import as_path, cumulative_length, project, sub_path
from amr_fleet.traffic_zones import request_chain, Zone, zone_visits, ZoneMap

ZONE_PREFIX = 'zone:'
UNKNOWN_POSE = 'zone:?pose'          # blocking 표식: 자세를 몰라 판정할 수 없다


def zone_resource(zone_id: str) -> str:
    """구역 id → 투표 자원 이름."""
    return ZONE_PREFIX + zone_id


@dataclasses.dataclass(frozen=True)
class PathGateConfig:
    """경로 게이트 파라미터 (거리 m)."""

    prerequest_m: float = 1.0       # 결정점(요청 거리)보다 이만큼 먼저 요청 — 투표 왕복 동안 멈추지 않으려고
    chain_gap_m: float = 1.5        # traffic.yaml zones.chain_gap_m
    robot_radius: float = 0.36      # traffic.yaml robot_radius (몸체 판정)
    max_path_m: float = 60.0        # 남은 경로를 이만큼만 본다 (traffic.yaml max_path_m)
    goal_tolerance_m: float = 0.3   # 남은 경로가 이보다 짧으면 도착 (사슬 없음)
    step_m: float = 0.2             # 경로 표본 간격
    # 구역 진입을 몸체 원판(robot_radius)으로 판정 (모듈 설명). 클래스 기본값은 옛 중심점 판정 — 기존 단위
    # 시험의 기대값을 그대로 두려고. 노드는 load_path_gate 로 traffic.yaml zones.body_entry (배포 true, 키가
    # 없으면 true — 중앙 TrafficConfig 기본값과 같다)를 쓴다
    body_entry: bool = False

    def __post_init__(self):
        for name in ('prerequest_m', 'chain_gap_m', 'robot_radius', 'goal_tolerance_m'):
            if not getattr(self, name) >= 0.0:
                raise ValueError(f'{name} 는 0 이상: {getattr(self, name)!r}')
        for name in ('max_path_m', 'step_m'):
            if not getattr(self, name) > 0.0:
                raise ValueError(f'{name} 는 양수: {getattr(self, name)!r}')


@dataclasses.dataclass(frozen=True)
class GateDecision:
    """decide() 결과 (모듈 설명)."""

    request: Tuple[str, ...] = ()
    blocking: Tuple[str, ...] = ()
    ahead: FrozenSet[str] = frozenset()
    body: FrozenSet[str] = frozenset()
    first_entry_m: Optional[float] = None     # 사슬 첫 입구까지 경로 거리 (사슬 없으면 None)
    remaining_m: Optional[float] = None       # 남은 경로 길이 (경로 없으면 None)


def remaining_path(plan: np.ndarray, x: float, y: float,
                   max_len: float, step: float = 0.2) -> Tuple[np.ndarray, float]:
    """남은 경로: plan 을 (x, y) 에 투영해 현재 위치부터 max_len 까지 + 전체 남은 길이 (traffic_manager 와 같다)."""
    cum = cumulative_length(plan)
    s0, d0 = project(plan, cum, x, y)
    end = min(float(cum[-1]), s0 + max_len)
    rem = sub_path(plan, cum, s0, end, step)
    path = np.vstack([[x, y], rem]) if d0 > 0.05 else rem
    return path, float(cum[-1]) - s0 + d0


class PathGate:
    """구역 배치 + 요청 거리로 경로 게이트 결정을 낸다."""

    def __init__(self, zones: Sequence[Zone], approach: Mapping[str, float],
                 cfg: PathGateConfig = PathGateConfig()):
        self.zone_map = ZoneMap([z for z in zones if z.polygon is not None])
        missing = set(self.zone_map.ids()) - set(approach)
        if missing:
            raise ValueError(f'요청 거리가 없는 구역: {sorted(missing)}')
        self.approach = {z: float(approach[z]) for z in self.zone_map.ids()}
        self.cfg = cfg
        self._pre = {z: a + cfg.prerequest_m for z, a in self.approach.items()}

    def polygons(self) -> dict:
        """'zone:<id>' → 다각형 (에이전트 해제 게이트용)."""
        return {zone_resource(z.zone_id): z.polygon for z in self.zone_map.zones()}

    def body_zones(self, x: float, y: float, yaw: float) -> FrozenSet[str]:
        """몸체가 걸친 구역 id — body_entry 면 원판(robot_radius)-다각형 겹침, 아니면 중심 + 외접 반경 네 점."""
        r = self.cfg.robot_radius
        if self.cfg.body_entry:
            return self.zone_map.zones_touching(x, y, r)
        pts = [(x, y)] + [(x + r * math.cos(yaw + k * math.pi / 2),
                           y + r * math.sin(yaw + k * math.pi / 2)) for k in range(4)]
        return frozenset(z for z in self.zone_map.zones_at(np.array(pts)) if z)

    def decide(self, pose: Optional[Tuple[float, float, float]], plan: Optional[np.ndarray],
               held: Collection[str], active: bool = True) -> GateDecision:
        """
        지금 결정.

        pose = (x, y, yaw) map 프레임 (None = 모름), plan = 로봇 전역 경로 (N, 2) (None = 없음),
        held = 지금 허가된 자원 ('zone:*'), active = 움직이는 단계인가 (아니면 경로를 보지 않는다).
        """
        if not active:
            body = (frozenset() if pose is None else
                    frozenset(zone_resource(z) for z in self.body_zones(*pose)))
            return GateDecision(body=body)
        if pose is None:
            return GateDecision(blocking=(UNKNOWN_POSE,))
        x, y, yaw = (float(v) for v in pose)
        body_ids = self.body_zones(x, y, yaw)
        body = frozenset(zone_resource(z) for z in body_ids)
        path = as_path(plan) if plan is not None else None
        if path is None or len(path) < 2:
            return GateDecision(body=body)
        rem_path, remaining = remaining_path(path, x, y, self.cfg.max_path_m, self.cfg.step_m)
        if remaining <= self.cfg.goal_tolerance_m or len(rem_path) < 2:
            return GateDecision(body=body, remaining_m=remaining)
        held = set(held)
        if self.cfg.body_entry:
            visits = zone_visits(rem_path, self.zone_map, self.cfg.step_m,
                                 radius=self.cfg.robot_radius)
            have = {h[len(ZONE_PREFIX):] for h in held if h.startswith(ZONE_PREFIX)}
            chain = request_chain(visits, self._pre, self.cfg.chain_gap_m, body_ids, held=have)
        else:
            visits = zone_visits(rem_path, self.zone_map, self.cfg.step_m)
            chain = request_chain(visits, self._pre, self.cfg.chain_gap_m, body_ids)
        ahead = frozenset(zone_resource(v.zone_id) for v in visits)
        if not chain:
            return GateDecision(ahead=ahead, body=body, remaining_m=remaining)
        request = tuple(zone_resource(v.zone_id) for v in chain)
        first = chain[0]
        blocking: Tuple[str, ...] = ()
        if first.s_in <= self.approach[first.zone_id]:
            blocking = tuple(r for r in request if r not in held)
        return GateDecision(request=request, blocking=blocking, ahead=ahead, body=body,
                            first_entry_m=float(first.s_in), remaining_m=remaining)


def load_path_gate(zones_file: str, traffic_params_file: str,
                   prerequest_m: float = 1.0) -> PathGate:
    """
    traffic_zones.yaml(구역) + traffic.yaml(요청 거리 · 사슬 간격 · 반경)으로 경로 게이트를 만든다.

    서버 traffic_manager 와 같은 파일 · 같은 값을 쓴다 (approach_distance_m · corridor_approach_m · 항목별
    approach_m · chain_gap_m · robot_radius · max_path_m · goal_tolerance_m · body_entry).
    """
    import yaml

    from amr_fleet.traffic_zones import approach_map, load_layout
    with open(traffic_params_file, encoding='utf-8') as f:
        doc = yaml.safe_load(f) or {}
    params = None
    for key, val in doc.items():
        if str(key).endswith('traffic_manager_node') and isinstance(val, dict):
            params = val.get('ros__parameters', {})
    if params is None:
        raise ValueError(f'{traffic_params_file}: traffic_manager_node 파라미터가 없다')
    z = params.get('zones', {})
    zones, _ = load_layout(zones_file, bool(z.get('corridor_same_direction', True)))
    approach = approach_map(zones, float(z.get('approach_distance_m', 2.0)),
                            float(z.get('corridor_approach_m', 2.5)))
    body_entry = z.get('body_entry', True)        # 중앙 TrafficConfig.zones.body_entry 기본값과 같게
    if not isinstance(body_entry, bool):
        raise ValueError(f'{traffic_params_file}: zones.body_entry 는 true/false: {body_entry!r}')
    cfg = PathGateConfig(prerequest_m=float(prerequest_m),
                         chain_gap_m=float(z.get('chain_gap_m', 1.5)),
                         robot_radius=float(params.get('robot_radius', 0.36)),
                         max_path_m=float(params.get('max_path_m', 60.0)),
                         goal_tolerance_m=float(params.get('goal_tolerance_m', 0.3)),
                         body_entry=body_entry)
    return PathGate(zones, approach, cfg)


__all__ = ['GateDecision', 'PathGate', 'PathGateConfig', 'UNKNOWN_POSE', 'ZONE_PREFIX',
           'load_path_gate', 'remaining_path', 'zone_resource']
