"""
배치 유효성 (valid.json): 고정 조건과 운용 조건을 지도·기하로 검사한다. 이유 문자열은 '<종류>:<내용>'.

종류 (histogram 키):
  generator   월드 생성기 check_layout 실패 (actor·차량 경로가 장애물과 겹침, 좁은 통로 순폭 등) — build 가 채운다
  resolve     수준 해석 실패 (levels.LayoutError: 충전소·구조물 자리 없음, 경로 재배치 불가)
  overlap     정적 장애물(랙·기둥·스테이션·구조물·도크 화물)끼리 겹침
  slot        대기 슬롯·충전소 staging 여유 부족: 지도 장애물까지 < 0.66 m (외접 반경 0.36 + 0.30),
              슬롯끼리 < 1.0 m (fleet_spawn min_separation_m), 구역(교차로 등) 안
  lane        대기 슬롯·충전소 staging 이 동적 장애물 경로에서 < 1.0 m (사람: 0.36 + 0.35 + 0.30) 또는
              차량 차선 상자(차체 반폭 + 0.3)에서 < 0.66 m
  reach       도달성: 통과 셀(벽까지 ≥ 0.25 m — 반폭 0.20 + 0.05, 좁은 통로 0.6 m 를 남긴다)의 한 연결 성분에
              슬롯 5·충전소 staging 3·도크 staging 4·좁은 통로 중심이 모두 있어야 한다
  zones       교통 구역 파일이 traffic_zones 파서를 통과하지 못하거나 spacing_conflicts 가 있다 (traffic_manager 가
              spacing_check: reject 로 기동을 거부한다)
  markers     위치 표지 마커 74 개(기둥 32 + 랙 42)·충전 마커 3 개가 아니거나 id 가 겹침
numpy·scipy·yaml 이 필요하다 (컨테이너).
"""

import math
import sys

import numpy as np
from scipy import ndimage
import yaml

from layopt.gen.common import REPO, ROBOT_RADIUS, TRAFFIC_SRC
from layopt.gen.levels import aabb_gap, charger_geometry, seg_point
from layopt.gen.raster import cell_of, clearance

sys.path.insert(0, str(REPO / 'src' / 'amr_fleet'))
from amr_fleet import traffic_resolution as tres  # noqa: E402
from amr_fleet import traffic_zones as tzm  # noqa: E402

SLOT_CLEAR = ROBOT_RADIUS + 0.30
PASS_CLEAR = 0.25
HUMAN_CLEAR = ROBOT_RADIUS + 0.35 + 0.30
VEHICLE_HALF_W = {'forklift_main': 0.7, 'shuttle_amr': 0.25}
VEHICLE_REACH = {'forklift_main': (1.3, 2.1), 'shuttle_amr': (0.35, 0.35)}


def traffic_params():
    with open(TRAFFIC_SRC, encoding='utf-8') as f:
        p = yaml.safe_load(f)['/fleet/traffic_manager_node']['ros__parameters']
    z = p['zones']
    return {'approach_m': float(z['approach_distance_m']),
            'corridor_approach_m': float(z['corridor_approach_m']),
            'chain_gap_m': float(z['chain_gap_m']), 'robot_radius': float(p['robot_radius']),
            'corridor_same_direction': bool(z['corridor_same_direction'])}


def zone_check(zones_yaml_text):
    """traffic_zones.yaml 문자열 → (이유 목록, 정보 dict). 운용 파서·간격 검사를 그대로 쓴다."""
    reasons = []
    tp = traffic_params()
    data = yaml.safe_load(zones_yaml_text)
    try:
        zones = tzm.zones_from_config(data.get('zones'), tp['corridor_same_direction'])
        pockets = tres.pockets_from_config(data.get('pockets'))
    except ValueError as exc:
        return [f'zones:parse {exc}'], {}
    amap = tzm.approach_map(zones, tp['approach_m'], tp['corridor_approach_m'])
    conf = tzm.spacing_conflicts(zones, amap, tp['chain_gap_m'], tp['robot_radius'])
    for a, b, g in conf:
        reasons.append(f'zones:spacing {a}-{b} {g:.2f} m')
    return reasons, {'n_zones': len(zones), 'n_pockets': len(pockets),
                     'spacing_conflicts': len(conf)}


def vehicle_boxes(gen):
    out = []
    for name, (axis, lo, hi, fixed, *_r) in gen.vehicle_table().items():
        w = VEHICLE_HALF_W[name] + 0.3
        back, front = VEHICLE_REACH[name]
        a0, a1 = lo - back, hi + front
        out.append((name, (a0, a1, fixed - w, fixed + w) if axis == 'x'
                    else (fixed - w, fixed + w, a0, a1)))
    return out


def check(res, gen, raster, unknown, tz, obstacles, zones_text):
    """→ (valid, reasons, info). gen 의 LAY 가 res 의 배치여야 한다 (build 가 build_sdf 로 맞춘다)."""
    reasons = []
    info = {}
    # 정적 장애물 겹침
    for i, (na, a) in enumerate(obstacles):
        for nb, b in obstacles[i + 1:]:
            same = any(na.startswith(k) and nb.startswith(k) for k in ('dock_', 'rack_narrow_'))
            if aabb_gap(a, b) <= 0.0 and not same:      # 좁은 통로 랙 2베이·도크 화물은 붙여 둔 것
                reasons.append(f'overlap:{na}~{nb}')
    # 슬롯·staging 여유
    blocked = raster.occupied | unknown
    clear = clearance(blocked, raster.resolution)
    poses = [(f"slot {s['robot']}", s['x'], s['y']) for s in res['slots']]
    poses += [(f"staging {c['id']}", *charger_geometry(c)[1][:2]) for c in res['chargers']]
    worst = {}
    for name, x, y in poses:
        r, c = cell_of(x, y, raster)
        v = float(clear[r, c]) if 0 <= r < clear.shape[0] and 0 <= c < clear.shape[1] else -1.0
        worst[name] = round(v, 3)
        if v < SLOT_CLEAR:
            reasons.append(f'slot:{name} 장애물까지 {v:.2f} m < {SLOT_CLEAR:.2f}')
        if any(tz_in(z, x, y) for z in tz['zones']):
            reasons.append(f'slot:{name} 교통 구역 안')
    info['pose_clearance_m'] = worst
    sl = res['slots']
    for i, a in enumerate(sl):
        for b in sl[i + 1:]:
            d = math.hypot(a['x'] - b['x'], a['y'] - b['y'])
            if d < 1.0:
                reasons.append(f"slot:{a['robot']}-{b['robot']} 간격 {d:.2f} m < 1.0")
    # 동적 장애물 경로
    specs = gen.actor_specs()
    lane_min = {}
    for name, x, y in poses:
        best = math.inf
        for aname, _v, wps, _d in specs:
            pts = [(w[1], w[2]) for w in wps]
            for (ax, ay), (bx, by) in zip(pts, pts[1:]):
                d = seg_point(ax, ay, bx, by, x, y)
                if d < best:
                    best, who = d, aname
        if best < HUMAN_CLEAR:
            reasons.append(f'lane:{name} {who} 경로까지 {best:.2f} m < {HUMAN_CLEAR:.2f}')
        for vname, box in vehicle_boxes(gen):
            dx = max(box[0] - x, x - box[1], 0.0)
            dy = max(box[2] - y, y - box[3], 0.0)
            dv = math.hypot(dx, dy)
            best = min(best, dv)
            if dv < SLOT_CLEAR:
                reasons.append(f'lane:{name} {vname} 차선까지 {dv:.2f} m < {SLOT_CLEAR:.2f}')
        lane_min[name] = round(best, 3)
    info['pose_lane_clearance_m'] = lane_min
    # 도달성
    passable = clear >= PASS_CLEAR
    lab, _n = ndimage.label(passable, structure=np.ones((3, 3), dtype=bool))
    targets = poses + [(f'dock {d}', *_dock_staging(gen, d)) for d in gen.DOCKS]
    targets.append(('narrow_aisle', gen.NARROW_CX, gen.NARROW_CY))
    comps = {}
    for name, x, y in targets:
        r, c = cell_of(x, y, raster)
        comps[name] = int(lab[r, c])
    main = comps[targets[0][0]]
    for name, comp in comps.items():
        if comp == 0 or comp != main:
            reasons.append(f'reach:{name} 슬롯 amr_01 과 이어지지 않음')
    info['reach_components'] = len(set(comps.values()))
    # 구역 파일
    zr, zinfo = zone_check(zones_text)
    reasons += zr
    info.update(zinfo)
    # 마커
    lm = {**gen.pillar_marker_poses(), **gen.rack_marker_poses()}
    if len(lm) != 74 or sorted(lm) != list(range(10, 84)):
        reasons.append(f'markers:위치 표지 {len(lm)} 개 (74 여야)')
    if sorted(c['marker_id'] for c in res['chargers']) != [4, 5, 6]:
        reasons.append('markers:충전 마커 id 가 4,5,6 이 아님')
    return not reasons, reasons, info


def tz_in(zone, x, y):
    xs = [p[0] for p in zone['polygon']]
    ys = [p[1] for p in zone['polygon']]
    return min(xs) <= x <= max(xs) and min(ys) <= y <= max(ys)


def _dock_staging(gen, name):
    x, y, kind, _mid = gen.DOCKS[name]
    sx = 1.0 if kind == 'outbound' else -1.0
    return sx * (30.0 - 0.01 - 2.21), y
