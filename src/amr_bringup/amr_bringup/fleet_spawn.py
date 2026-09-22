"""
다중 로봇 스폰 목록(config/fleet_spawn.yaml) 해석·검증 (launch 비의존 순수 모듈).

형식
    robots: [{name, x, y, yaw, comm_latency_ms?}, ...]   월드 좌표 [m, rad]
    spawn_z, pause_during_spawn, spawn_timeout_s, comm_latency_ms, min_separation_m   공통 옵션 (모두 선택)

- name 은 네임스페이스 = Gazebo 모델 이름이다: amr_ + 두 자리 (multi_robot.md §1, amr_99 까지).
- comm_latency_ms: 숫자 hi(→ [0, hi]) 또는 [lo, hi]. 0 ≤ lo ≤ hi ≤ 100 ms (명세 4.9 최대 100 ms).
  공통 값은 fleet_manager(assign_task 호출)와 모든 fleet_adapter(robot_state) 에, 로봇별 값은 그 로봇의
  fleet_adapter 에만 들어간다 — manager 의 지연 모델은 로봇 구분 없이 하나다 (amr_fleet fleet.yaml).
- 로봇 사이 거리 < min_separation_m 이면 거절한다 (스폰 순간 차체가 겹치면 물리가 튕겨 낸다).
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
import math
import re
from typing import Any, Dict, List, Optional, Sequence, Tuple

import yaml

ROBOT_NAME_RE = re.compile(r'^amr_\d{2}$')
SPEC_MAX_LATENCY_MS = 100.0          # 명세 4.9
MANAGER_KEY = '/fleet/fleet_manager_node'
ADAPTER_KEY = '/**/fleet_adapter_node'
_ROBOT_KEYS = {'name', 'x', 'y', 'yaw', 'comm_latency_ms'}
_SHARED_DEFAULTS = {
    'spawn_z': 0.02,                 # [m] spawn.launch.py 기본과 같다 (바퀴가 살짝 떠서 내려앉게)
    'pause_during_spawn': True,      # 스폰 동안 월드 일시 정지 (world_gate.py 참고)
    'spawn_timeout_s': 120.0,        # [s] 모델이 다 보일 때까지 기다리는 상한 (벽시계)
    'comm_latency_ms': None,         # None = amr_fleet fleet.yaml 값 그대로
    'min_separation_m': 1.0,         # [m] 로봇 중심 간 최소 거리
}


@dataclass(frozen=True)
class RobotSpec:
    """로봇 1대의 이름과 스폰 자세 (+ 선택: robot_state 송신 지연)."""

    name: str
    x: float
    y: float
    yaw: float
    comm_latency_ms: Optional[Tuple[float, float]] = None


@dataclass(frozen=True)
class FleetSpawn:
    """스폰 목록 전체 (로봇 순서 유지)."""

    robots: Tuple[RobotSpec, ...]
    spawn_z: float = 0.02
    pause_during_spawn: bool = True
    spawn_timeout_s: float = 120.0
    comm_latency_ms: Optional[Tuple[float, float]] = None
    min_separation_m: float = 1.0


def parse_latency(value: Any, where: str) -> Optional[Tuple[float, float]]:
    """comm_latency_ms 값 → (lo, hi) [ms]. None 은 그대로. 범위를 벗어나면 ValueError."""
    if value is None:
        return None
    if isinstance(value, bool):
        raise ValueError(f'{where}: comm_latency_ms 는 숫자 또는 [lo, hi] 여야 한다: {value!r}')
    if isinstance(value, (int, float)):
        lo, hi = 0.0, float(value)
    elif isinstance(value, (list, tuple)) and len(value) == 2 \
            and not any(isinstance(v, bool) for v in value):
        lo, hi = float(value[0]), float(value[1])
    else:
        raise ValueError(f'{where}: comm_latency_ms 는 숫자 또는 [lo, hi] 여야 한다: {value!r}')
    if not (0.0 <= lo <= hi <= SPEC_MAX_LATENCY_MS):
        raise ValueError(f'{where}: comm_latency_ms 는 0 <= lo <= hi <= {SPEC_MAX_LATENCY_MS:g} ms '
                         f'(명세 4.9): {value!r}')
    return lo, hi


def _number(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{where}: 유한한 숫자여야 한다: {value!r}')
    return float(value)


def parse_robot(item: Any, index: int) -> RobotSpec:
    """robots[] 항목 하나 → RobotSpec. 모르는 키·빠진 키·형식이 틀린 이름은 ValueError."""
    where = f'robots[{index}]'
    if not isinstance(item, dict):
        raise ValueError(f'{where}: {{name, x, y, yaw}} 사전이어야 한다: {item!r}')
    unknown = set(item) - _ROBOT_KEYS
    missing = {'name', 'x', 'y', 'yaw'} - set(item)
    if unknown or missing:
        raise ValueError(f'{where}: 모르는 키 {sorted(unknown)} / 빠진 키 {sorted(missing)}')
    name = str(item['name'])
    if not ROBOT_NAME_RE.match(name):
        raise ValueError(f'{where}: 이름은 amr_ + 두 자리 숫자 (multi_robot.md §1): {name!r}')
    return RobotSpec(name=name,
                     x=_number(item['x'], f'{where}.x'),
                     y=_number(item['y'], f'{where}.y'),
                     yaw=_number(item['yaw'], f'{where}.yaw'),
                     comm_latency_ms=parse_latency(item.get('comm_latency_ms'),
                                                   f'{where} ({name})'))


def check_separation(robots: Sequence[RobotSpec], min_separation_m: float) -> None:
    """이름 중복·로봇 간 거리 < min_separation_m 이면 ValueError."""
    seen = set()
    for r in robots:
        if r.name in seen:
            raise ValueError(f'로봇 이름 중복: {r.name} (Gazebo 모델 이름 = 네임스페이스)')
        seen.add(r.name)
    for i, a in enumerate(robots):
        for b in robots[i + 1:]:
            d = math.hypot(a.x - b.x, a.y - b.y)
            if d < min_separation_m:
                raise ValueError(f'{a.name} 과 {b.name} 의 스폰 거리 {d:.2f} m < '
                                 f'min_separation_m {min_separation_m:g} m')


def parse_fleet_spawn(data: Any) -> FleetSpawn:
    """YAML 로 읽은 dict → FleetSpawn (검증 포함)."""
    if not isinstance(data, dict) or not isinstance(data.get('robots'), list) \
            or not data['robots']:
        raise ValueError('스폰 목록에 비어 있지 않은 robots: [...] 가 있어야 한다')
    unknown = set(data) - set(_SHARED_DEFAULTS) - {'robots'}
    if unknown:
        raise ValueError(f'모르는 공통 옵션: {sorted(unknown)}')
    opts = dict(_SHARED_DEFAULTS, **{k: v for k, v in data.items() if k != 'robots'})
    if not isinstance(opts['pause_during_spawn'], bool):
        raise ValueError(f'pause_during_spawn 은 true/false: {opts["pause_during_spawn"]!r}')
    spawn = FleetSpawn(
        robots=tuple(parse_robot(item, i) for i, item in enumerate(data['robots'])),
        spawn_z=_number(opts['spawn_z'], 'spawn_z'),
        pause_during_spawn=opts['pause_during_spawn'],
        spawn_timeout_s=_number(opts['spawn_timeout_s'], 'spawn_timeout_s'),
        comm_latency_ms=parse_latency(opts['comm_latency_ms'], '공통'),
        min_separation_m=_number(opts['min_separation_m'], 'min_separation_m'))
    if spawn.spawn_timeout_s <= 0.0:
        raise ValueError(f'spawn_timeout_s 는 양수: {spawn.spawn_timeout_s}')
    check_separation(spawn.robots, spawn.min_separation_m)
    return spawn


def load_fleet_spawn(path: str) -> FleetSpawn:
    """스폰 목록 YAML 파일 → FleetSpawn."""
    with open(path, encoding='utf-8') as f:
        return parse_fleet_spawn(yaml.safe_load(f))


def select_robots(spawn: FleetSpawn, num_robots: int) -> Tuple[RobotSpec, ...]:
    """앞에서 num_robots 대. 1 미만이거나 목록보다 많으면 ValueError."""
    if not 1 <= num_robots <= len(spawn.robots):
        raise ValueError(f'num_robots 는 1 ~ {len(spawn.robots)} (스폰 목록 길이): {num_robots}')
    return spawn.robots[:num_robots]


def find_robot(spawn: FleetSpawn, name: str) -> Optional[RobotSpec]:
    """이름으로 찾기 (없으면 None)."""
    return next((r for r in spawn.robots if r.name == name), None)


def fleet_params_with_latency(fleet_params: Dict[str, Any], spawn: FleetSpawn,
                              robots: Sequence[RobotSpec]) -> Optional[Dict[str, Any]]:
    """
    amr_fleet 파라미터 dict(fleet.yaml 내용)에 공통·로봇별 comm_latency_ms 를 얹은 사본.

    얹을 값이 없으면 None (fleet.yaml 을 그대로 쓰면 된다).
    로봇별 값은 '/<robot>/fleet_adapter_node' 키로 '/**/fleet_adapter_node' 뒤에 둔다 — 같은 파일에서
    한 노드에 맞는 키가 여럿이면 뒤의 키가 이긴다 (rcl 파라미터 파일 규칙). 그래서 fleet_manager.launch.py
    의 comm_latency_ms 인자(모든 어댑터에 dict 로 덮어씀)는 쓰지 않고 파일로만 넘긴다.
    """
    per_robot = [r for r in robots if r.comm_latency_ms is not None]
    if spawn.comm_latency_ms is None and not per_robot:
        return None
    out = copy.deepcopy(fleet_params) if fleet_params else {}

    def params(key: str) -> Dict[str, Any]:
        node = out.setdefault(key, {})
        return node.setdefault('ros__parameters', {})

    if spawn.comm_latency_ms is not None:
        params(MANAGER_KEY)['comm_latency_ms'] = list(spawn.comm_latency_ms)
        params(ADAPTER_KEY)['comm_latency_ms'] = list(spawn.comm_latency_ms)
    for r in per_robot:
        key = f'/{r.name}/fleet_adapter_node'
        out.pop(key, None)            # 키 순서: 와일드카드 뒤에 오도록 다시 넣는다
        params(key)['comm_latency_ms'] = list(r.comm_latency_ms)
    return out


def robot_ids(robots: Sequence[RobotSpec]) -> List[str]:
    """이름 목록 (스폰 순서)."""
    return [r.name for r in robots]
