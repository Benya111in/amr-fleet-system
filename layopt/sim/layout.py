"""
배치 → Layout: 묶음 디렉터리(contract.md §1–2) 또는 지금 저장소 파일(기준 배치 개발용).

묶음: layout.json(도크 · 충전기 · 대기 슬롯 · 체류 지점), map.yaml(+pgm), config/traffic_zones.yaml, world.sdf
(작업자 · 지게차 · 셔틀 궤적), valid.json. 지금 파일: maps/warehouse.yaml,
src/amr_fleet/config/traffic_zones.yaml,
behavior.yaml(도크 · 충전기 staging), fleet_spawn.yaml(대기 슬롯), src/amr_simulation/worlds/warehouse.sdf.
지금 파일에는 체류 지점 정의가 없어서 contract §2 규칙대로 만든다 (대기 = 슬롯, 이웃 = 다른 슬롯 가까운 순;
도크·충전기 = staging, 이웃 = 바깥쪽 대기열 자세 하나 — 묶음의 정의와 다를 수 있다, sim.md).

동적 장애물은 world.sdf 에서 읽는다 — 사람 actor 의 웨이포인트(t, x, y)와 지게차 · 셔틀의 왕복 플러그인
(DiffDrive 속도·가속, TriggeredPublisher 반전 창). 배치가 바뀌면 생성기가 경로를 다시 놓으므로 묶음마다 읽는다.
"""

from __future__ import annotations

import dataclasses
import json
import math
import pathlib
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml

from layopt.sim import REPO

Pose = Tuple[float, float, float]
ROBOT_IDS = ('amr_01', 'amr_02', 'amr_03', 'amr_04', 'amr_05')
DOCK_IDS = ('dock_1', 'dock_2', 'dock_a', 'dock_b')
BASELINE_SPEC = 'v10-v20-v30-v40-v50'
HUMAN_RADIUS = 0.25         # perception predicted_layer 보행자 반경 (nav2_params.yaml predicted_layer)
# 차량 몸체 반치수 [m] (진행 방향, 옆) — models/forklift · shuttle_amr 크기 (gen_warehouse_world.py 주석)
VEHICLE_HALF = {'forklift_main': (1.1, 0.6), 'shuttle_amr': (0.3, 0.2)}
VEHICLE_DT = 0.1            # 차량 왕복 궤적 표본 간격 [s]


@dataclasses.dataclass(frozen=True)
class DwellPoint:
    """오래 서는 자세 (contract §2 dwell_points)."""

    id: str
    kind: str                                   # waiting | dock | charger | queue
    x: float
    y: float
    yaw: float
    neighbors: Tuple[Pose, ...] = ()


@dataclasses.dataclass
class ActorSpec:
    """동적 장애물 1 개: 닫힌 주기 궤적 (t 단조 증가, 마지막 = 주기)."""

    name: str
    kind: str                                   # human | vehicle
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    half_len: float                             # 진행 방향 반길이 [m]
    half_wid: float                             # 옆 반폭 [m]

    @property
    def period(self) -> float:
        return float(self.t[-1])


@dataclasses.dataclass
class Layout:
    """시뮬레이터가 쓰는 배치 한 개."""

    spec_id: str
    root: Optional[pathlib.Path]
    map_yaml: pathlib.Path
    zones_yaml: pathlib.Path
    docks: Dict[str, Pose]                      # 도크 id → staging
    chargers: Dict[str, Pose]                   # 충전기 id → staging
    slots: Dict[str, Pose]                      # 로봇 id → 대기 슬롯
    dwell_points: List[DwellPoint]
    actors: List[ActorSpec]
    valid: bool = True
    reasons: Tuple[str, ...] = ()
    source: str = 'bundle'                      # bundle | current

    @property
    def robot_ids(self) -> Tuple[str, ...]:
        return tuple(sorted(self.slots))

    def places(self) -> Dict[str, Pose]:
        """이름 → 자세 (도크 id, 'charger:<id>', 'slot:<robot>')."""
        out: Dict[str, Pose] = dict(self.docks)
        out.update({f'charger:{c}': p for c, p in self.chargers.items()})
        out.update({f'slot:{r}': p for r, p in self.slots.items()})
        return out


# ---------------------------------------------------------------------- 묶음
def load_bundle(root) -> Layout:
    """묶음 디렉터리 → Layout (contract.md §1–2)."""
    root = pathlib.Path(root)
    lj = json.loads((root / 'layout.json').read_text(encoding='utf-8'))
    docks = {d['id']: _pose(d['staging']) for d in lj['docks']}
    chargers = {c['id']: _pose(c['staging']) for c in lj['chargers']}
    slots = {s['robot']: (float(s['x']), float(s['y']), float(s.get('yaw', 0.0)))
             for s in lj['waiting_slots']}
    dwell = [DwellPoint(str(d['id']), str(d['kind']), float(d['x']), float(d['y']),
                        float(d.get('yaw', 0.0)),
                        tuple(_pose(n) for n in d.get('neighbors', ())))
             for d in lj.get('dwell_points', ())]
    if not dwell:
        dwell = synth_dwell_points(docks, chargers, slots)
    valid, reasons = True, ()
    vj = root / 'valid.json'
    if vj.is_file():
        v = json.loads(vj.read_text(encoding='utf-8'))
        valid, reasons = bool(v.get('valid', True)), tuple(v.get('reasons', ()))
    map_yaml = root / 'map.yaml'
    zones = root / 'config' / 'traffic_zones.yaml'
    sdf = root / 'world.sdf'
    actors = parse_world_actors(sdf) if sdf.is_file() else []
    return Layout(str(lj.get('spec_id', root.name)), root, map_yaml, zones, docks, chargers,
                  slots, dwell, actors, valid, reasons, 'bundle')


def _pose(p: Sequence[float]) -> Pose:
    return (float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0)


# ---------------------------------------------------------------------- 지금 파일
def load_current(repo: Optional[pathlib.Path] = None) -> Layout:
    """지금 저장소 파일로 만든 기준 배치 (묶음이 나오기 전 개발 · 보정용)."""
    repo = pathlib.Path(repo or REPO)
    beh = yaml.safe_load((repo / 'src/amr_behavior/config/behavior.yaml').read_text(
        encoding='utf-8'))['/**']['ros__parameters']['docks']
    docks = {d: _pose(beh[d]['staging']) for d in DOCK_IDS}
    chargers = {c.replace('charger_', ''): _pose(beh[c]['staging'])
                for c in beh['ids'] if c.startswith('charger_')}
    spawn = yaml.safe_load((repo / 'src/amr_bringup/config/fleet_spawn.yaml').read_text(
        encoding='utf-8'))
    slots = {r['name']: (float(r['x']), float(r['y']), float(r.get('yaw', 0.0)))
             for r in spawn['robots']}
    dwell = synth_dwell_points(docks, chargers, slots)
    sdf = repo / 'src/amr_simulation/worlds/warehouse.sdf'
    return Layout('current', None, repo / 'maps/warehouse.yaml',
                  repo / 'src/amr_fleet/config/traffic_zones.yaml', docks, chargers, slots,
                  dwell, parse_world_actors(sdf) if sdf.is_file() else [], True, (), 'current')


def synth_dwell_points(docks: Dict[str, Pose], chargers: Dict[str, Pose],
                       slots: Dict[str, Pose], queue_m: float = 2.5) -> List[DwellPoint]:
    """
    체류 지점 정의가 없을 때 contract §2 규칙으로 만든다.

    대기: 이웃 = 다른 슬롯 (가까운 순). 도크·충전기: staging, 이웃 = staging 에서 마커 반대쪽(바깥)으로
    queue_m 떨어진 대기열 자세 하나.
    """
    out: List[DwellPoint] = []
    for r, (x, y, th) in sorted(slots.items()):
        nb = sorted((p for q, p in slots.items() if q != r),
                    key=lambda p: (math.hypot(p[0] - x, p[1] - y), p))
        out.append(DwellPoint(f'wait:{r}', 'waiting', x, y, th, tuple(nb)))
    for kind, table in (('dock', docks), ('charger', chargers)):
        for d, (x, y, th) in sorted(table.items()):
            q = (x - queue_m * math.cos(th), y - queue_m * math.sin(th), th)
            out.append(DwellPoint(f'{kind}:{d}', kind, x, y, th, (q,)))
    return out


# ---------------------------------------------------------------------- world.sdf
def parse_world_actors(sdf_path) -> List[ActorSpec]:
    """world.sdf → 사람 actor(웨이포인트) + 왕복 차량(DiffDrive · TriggeredPublisher) 궤적."""
    text = pathlib.Path(sdf_path).read_text(encoding='utf-8')
    root = ET.fromstring(text)
    out: List[ActorSpec] = []
    for actor in root.iter('actor'):
        wps = []
        for wp in actor.iter('waypoint'):
            t = float(wp.findtext('time'))
            p = [float(v) for v in wp.findtext('pose').split()]
            wps.append((t, p[0], p[1]))
        if len(wps) < 2:
            continue
        a = np.array(wps)
        out.append(ActorSpec(actor.get('name'), 'human', a[:, 0] - a[0, 0], a[:, 1], a[:, 2],
                             HUMAN_RADIUS, HUMAN_RADIUS))
    for inc in root.iter('include'):
        name = inc.findtext('name') or ''
        plugins = inc.findall('plugin')
        dd = [p for p in plugins if p.get('name', '').endswith('DiffDrive')]
        trig = [p for p in plugins if p.get('name', '').endswith('TriggeredPublisher')]
        if not dd or len(trig) < 2:
            continue
        v = float(dd[0].findtext('max_linear_velocity'))
        acc = float(dd[0].findtext('max_linear_acceleration'))
        matches = []
        for p in trig:
            m = p.find('input/match')
            matches.append((m.get('field'), float(m.get('tol', '0')), float(m.text)))
        axis = matches[0][0].split('.')[-1]
        tol = matches[0][1]
        over = v * v / (2.0 * acc)
        trig_vals = sorted(m[2] for m in matches)
        lo = trig_vals[0] - over + tol                 # gen_warehouse_world.shuttle_plugins 의 역
        hi = trig_vals[-1] + over - tol
        px, py = [float(s) for s in inc.findtext('pose').split()[:2]]
        fixed = py if axis == 'x' else px
        start = px if axis == 'x' else py
        t, s = shuttle_profile(lo, hi, start, v, acc)
        xs = s if axis == 'x' else np.full_like(s, fixed)
        ys = np.full_like(s, fixed) if axis == 'x' else s
        hl, hw = VEHICLE_HALF.get(name, (0.5, 0.4))
        out.append(ActorSpec(name, 'vehicle', t, xs, ys, hl, hw))
    return out


def shuttle_profile(lo: float, hi: float, start: float, v: float, acc: float,
                    dt: float = VEHICLE_DT) -> Tuple[np.ndarray, np.ndarray]:
    """
    끝에서 가감속 a 로 반전하는 왕복 1 주기 (t, 위치). start 에서 +방향으로 출발한다고 본다.

    반전 창 시작 = 끝 − v²/(2a) 에서 반대 지령 → v²/(2a) 더 가서 멈추고 반대로 가속 (gen_warehouse_world 머리말).
    여기서는 [lo, hi] 사이를 등속 v, 끝에서 감속·가속 a 로 도는 대칭 왕복으로 근사한다.
    """
    length = hi - lo
    ta = v / acc
    cruise = max(0.0, length - v * ta) / v                 # 가속·감속 구간 길이 합 = v·ta
    leg = 2.0 * ta + cruise if length > v * ta else 2.0 * math.sqrt(length / acc)
    period = 2.0 * leg
    n = int(math.ceil(period / dt)) + 1
    t = np.linspace(0.0, period, n)

    def pos_one(tt: float) -> float:
        if length <= v * ta:
            th = leg / 2.0
            if tt <= th:
                return 0.5 * acc * tt * tt
            r = leg - tt
            return length - 0.5 * acc * r * r
        if tt <= ta:
            return 0.5 * acc * tt * tt
        if tt <= ta + cruise:
            return 0.5 * v * ta + v * (tt - ta)
        r = leg - tt
        return length - 0.5 * acc * r * r

    s = np.array([lo + (pos_one(tt) if tt <= leg else length - pos_one(tt - leg)) for tt in t])
    # 시작 위상: start 를 지나 +방향으로 가는 첫 시각에서 시작하게 돌린다
    k = int(np.argmin(np.abs(s[: n // 2] - start)))
    s = np.concatenate([s[k:-1], s[:k], s[k:k + 1]])
    return t, s


__all__ = ['ActorSpec', 'BASELINE_SPEC', 'DOCK_IDS', 'DwellPoint', 'Layout', 'Pose', 'ROBOT_IDS',
           'load_bundle', 'load_current', 'parse_world_actors', 'shuttle_profile',
           'synth_dwell_points']
