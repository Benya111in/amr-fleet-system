"""
배치 기하 → 교통 구역 · 대기 포켓 (모든 배치에 같은 알고리즘, traffic_zones.yaml 머리말 규칙의 일반화).

구역
- narrow_aisle (corridor, same_direction): 좁은 통로 중심 x ± 0.5, 랙 길이 y ± 2 (현행 손 작성 값과 같은 규칙).
- x_<통로>_<k> (intersection): 열 사이 통로(ab, bc)마다 두 열의 틈 중심(두 열이 모두 있는 x 범위 안)을 2 m 안쪽에서
  겹치는 것끼리 묶고(levels.junction_groups), 묶음의 [최소 - 1, 최대 + 1] x 통로 폭(남쪽 열 북면 ~ 북쪽 열 남면).
  같은 통로에서 이웃 구역 간격 g 가 chain_gap_m < g < 요청 거리 + 로봇 반경 (traffic.yaml: 1.5 < g < 2.36)
  이면 spacing_conflicts 가 기동을 거부하므로 두 구역을 하나로 합친다 (합친 뒤 다시 본다). k = 서→동 1 부터.
- dock_<id>_approach: 도크 판 벽에서 0.2 m 안쪽 ~ 3.2 m, 도크 y ± 0.75 (현행 규칙, 도크 고정).
포켓
- p_<통로>_<x>_{s,n}: 열 사이 통로의 랙마다 랙 앞면에서 0.6 m (남쪽 열 → s, 북쪽 열 → n), x 순 s → n.
- p_narrow_{n,s}_{w,e}: 좁은 통로 중심선에서 ±2.3 m, 끝에서 0.6 m 밖 (y = cy ± 2.6).
- p_north_{-15,0,15} (y 19), p_south_{-10,10} (y -19): 벽 앞 고정 후보.
- 후보 중 구역 안, 장애물(벽 포함)에서 0.45 m 안, 대기 슬롯·충전소 staging·구조물에서 2.0 m 안이면 뺀다.
표준 라이브러리만 쓴다 (간격 검사는 validate.py 가 amr_fleet.traffic_zones 로 다시 한다).
"""

import math

from layopt.gen.common import load_generator
from layopt.gen.levels import (aisles, charger_geometry, junction_groups, point_aabb,
                               rows_from_racks, struct_aabb)

APPROACH_M, CORRIDOR_APPROACH_M, CHAIN_GAP_M, ROBOT_R = 2.0, 2.5, 1.5, 0.36
DOCK_ZONE_X = (29.8, 26.8)          # 도크 구역 x: 벽 쪽 |x| 29.8 ~ 안쪽 26.8 (손 작성 값)
DOCK_ZONE_HALF_Y = 0.75
POCKET_FACE = 0.6
WALL_POCKETS = [('p_north_-15', -15.0, 19.0), ('p_north_0', 0.0, 19.0),
                ('p_north_15', 15.0, 19.0), ('p_south_-10', -10.0, -19.0),
                ('p_south_10', 10.0, -19.0)]


def _fmt(x):
    """포켓 id 의 x 표기: 현행 '-18', 반 단위 '-16.5'."""
    return f'{x:g}'


def _rect(x0, x1, y0, y1):
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def intersection_zones(rows):
    out = []
    for a in aisles(rows):
        if a['y0'] is None:
            continue
        spans = [[g[0] - 1.0, g[-1] + 1.0] for g in junction_groups(a['centers'])]
        merged = True
        while merged:
            merged = False
            for i in range(len(spans) - 1):
                g = spans[i + 1][0] - spans[i][1]
                if CHAIN_GAP_M < g < APPROACH_M + ROBOT_R:
                    spans[i] = [spans[i][0], spans[i + 1][1]]
                    del spans[i + 1]
                    merged = True
                    break
        for k, (x0, x1) in enumerate(spans, 1):
            out.append({'id': f"x_{a['id']}_{k}", 'kind': 'intersection',
                        'polygon': _rect(x0, x1, a['y0'], a['y1'])})
    return out


def dock_zones(docks):
    out = []
    for name, (x, y, _kind, _mid) in docks.items():
        s = 1.0 if x > 0 else -1.0
        x0, x1 = sorted((s * DOCK_ZONE_X[0], s * DOCK_ZONE_X[1]))
        out.append({'id': f'{name}_approach', 'kind': 'intersection',
                    'polygon': _rect(x0, x1, y - DOCK_ZONE_HALF_Y, y + DOCK_ZONE_HALF_Y)})
    return out


def narrow_zone(gen):
    cx, cy = gen.NARROW_CX, gen.NARROW_CY
    half = gen.RACK_L               # 좁은 통로 랙 2베이 × 2.0 m → y ± 2
    return {'id': 'narrow_aisle', 'kind': 'corridor', 'same_direction': True,
            'polygon': _rect(cx - 0.5, cx + 0.5, cy - half, cy + half)}


def in_polygon(px, py, poly):
    """축 정렬 사각형 다각형 (이 모듈이 만드는 것) 안인지 (경계 포함)."""
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    return min(xs) - 1e-9 <= px <= max(xs) + 1e-9 and min(ys) - 1e-9 <= py <= max(ys) + 1e-9


def pocket_candidates(rows, gen):
    out = []
    for a in aisles(rows):
        if a['y0'] is None:
            continue
        cand = [(x, 0, 's') for x in rows[a['south']]['xs']] + \
            [(x, 1, 'n') for x in rows[a['north']]['xs']]
        for x, _o, side in sorted(cand):
            y = a['y0'] + POCKET_FACE if side == 's' else a['y1'] - POCKET_FACE
            out.append((f"p_{a['id']}_{_fmt(x)}_{side}", x, y))
    cx, cy = gen.NARROW_CX, gen.NARROW_CY
    for side, sx in (('w', -1.0), ('e', 1.0)):
        for end, sy in (('n', 1.0), ('s', -1.0)):
            out.append((f'p_narrow_{end}_{side}', cx + sx * 2.3,
                        cy + sy * (gen.RACK_L + POCKET_FACE)))
    return out + list(WALL_POCKETS)


def derive(res, obstacles):
    """
    해석 결과(levels.resolve) + 정적 장애물 [(이름, AABB)] → (traffic_zones dict, 메모 목록).

    dict = {frame_id, zones: [...], pockets: [{id, x, y, yaw}]} (traffic_zones.yaml 과 같은 구조).
    """
    gen = load_generator()
    lay = res['gen_spec']
    rows = rows_from_racks(lay['racks'])
    zones = [narrow_zone(gen)] + intersection_zones(rows) + dock_zones(gen.DOCKS)
    notes = []
    keep_away = [(s['x'], s['y']) for s in res['slots']]
    keep_away += [tuple(charger_geometry(c)[1][:2]) for c in lay['chargers']]
    structs = [struct_aabb(s) for s in lay['structures']]
    pockets = []
    for pid, x, y in pocket_candidates(rows, gen):
        why = None
        if any(in_polygon(x, y, z['polygon']) for z in zones):
            why = '구역 안'
        elif abs(x) > 30.0 - 0.45 or abs(y) > 20.0 - 0.45 or \
                any(point_aabb(x, y, b) < 0.45 for _n, b in obstacles):
            why = '장애물 0.45 m 안'
        elif any(math.hypot(x - kx, y - ky) < 2.0 for kx, ky in keep_away) or \
                any(point_aabb(x, y, b) < 2.0 for b in structs):
            why = '슬롯·staging·구조물 2.0 m 안'
        if why:
            notes.append(f'포켓 {pid} ({x:g}, {y:g}) 뺌: {why}')
            continue
        pockets.append({'id': pid, 'x': x, 'y': y, 'yaw': 0.0})
    return {'frame_id': 'map', 'zones': zones, 'pockets': pockets}, notes
