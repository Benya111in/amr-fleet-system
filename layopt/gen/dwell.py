"""
체류 지점 (contract §2 dwell_points): 로봇이 오래 서는 자세와 동시에 근처에 설 수 있는 다른 로봇 자세.

- waiting:<robot>      대기 슬롯. 이웃 = 다른 슬롯 (가까운 순).
- dock:<dock>          도크 staging. 이웃 = 그 도크 접근 구역 앞 대기열 (첫째 = 구역 입구에서 요청 거리 2.0 m,
                       둘째 = 그 뒤 1.0 m — 로봇 길이 0.6 + 틈 0.4).
- charger:<id>         충전소 staging. 이웃 = 옆 충전소에서 충전 중인 로봇(도킹 완료 자세)과 staging 앞 대기열
                       (staging 에서 판 반대쪽으로 1.0 m).
- queue:<zone>:<w|e|n|s>  구역(교차로·좁은 통로·도크 접근) 앞에서 토큰을 기다리는 자세: 구역 경계에서 요청 거리
                       (교차로·도크 2.0 m, 좁은 통로 2.5 m) 밖, 구역 쪽을 본다. 다른 구역 안이거나 장애물 0.45 m 안이면
                       뺀다. 이웃 = 구역 안을 지나는 로봇(구역 중심), 줄 둘째(1.0 m 뒤), 같은 구역의 다른 대기 자세.
추가 필드 (contract 밖, 정보용): docked (dock·charger 의 도킹 완료 자세).
표준 라이브러리만 쓴다.
"""

import math

from layopt.gen.common import DOCK_STANDOFF, load_generator
from layopt.gen.levels import charger_geometry, point_aabb
from layopt.gen.zones import APPROACH_M, CORRIDOR_APPROACH_M, in_polygon

SECOND_IN_LINE = 1.0


def _p(x, y, yaw):
    return [round(x, 4), round(y, 4), round(math.atan2(math.sin(yaw), math.cos(yaw)), 4)]


def _sorted_nb(x, y, cands):
    return sorted(cands, key=lambda p: (math.hypot(p[0] - x, p[1] - y), p[0], p[1]))


def _bbox(poly):
    xs = [q[0] for q in poly]
    ys = [q[1] for q in poly]
    return min(xs), max(xs), min(ys), max(ys)


def queue_points(zone, zones, obstacles):
    """구역 하나 앞 대기 자세 [(side, x, y, yaw, 뒤쪽 단위 벡터)]."""
    x0, x1, y0, y1 = _bbox(zone['polygon'])
    d = CORRIDOR_APPROACH_M if zone['kind'] == 'corridor' else APPROACH_M
    xm, ym = (x0 + x1) / 2, (y0 + y1) / 2
    cand = [('w', x0 - d, ym, 0.0, (-1, 0)), ('e', x1 + d, ym, math.pi, (1, 0)),
            ('s', xm, y0 - d, math.pi / 2, (0, -1)), ('n', xm, y1 + d, -math.pi / 2, (0, 1))]
    out = []
    for side, x, y, yaw, back in cand:
        if abs(x) > 30.0 - 0.45 or abs(y) > 20.0 - 0.45:
            continue
        if any(point_aabb(x, y, b) < 0.45 for _n, b in obstacles):
            continue
        if any(in_polygon(x, y, z['polygon']) for z in zones):
            continue
        out.append((side, x, y, yaw, back))
    return out


def dwell_points(res, tz, obstacles):
    """해석 결과 + 교통 구역 dict + 장애물 → dwell_points 목록."""
    gen = load_generator()
    out = []
    slots = res['slots']
    for s in slots:
        nb = [_p(o['x'], o['y'], o['yaw']) for o in slots if o is not s]
        out.append({'id': f"wait:{s['robot']}", 'kind': 'waiting',
                    **dict(zip(('x', 'y', 'yaw'), _p(s['x'], s['y'], s['yaw']))),
                    'neighbors': _sorted_nb(s['x'], s['y'], nb)})
    zones = tz['zones']
    zmap = {z['id']: z for z in zones}
    for name, (x, y, kind, _mid) in gen.DOCKS.items():
        sx = 1.0 if kind == 'outbound' else -1.0          # 판 쪽 (동쪽 도크 +x)
        plate = 30.0 - 0.01
        stag = (sx * (plate - 2.21), y, 0.0 if sx > 0 else math.pi)
        docked = (sx * (plate - DOCK_STANDOFF), y, stag[2])
        z = zmap[f'{name}_approach']
        x0, x1, _y0, _y1 = _bbox(z['polygon'])
        inner = x0 if sx > 0 else x1
        q0 = inner - sx * APPROACH_M
        nb = [_p(q0, y, stag[2]), _p(q0 - sx * SECOND_IN_LINE, y, stag[2])]
        out.append({'id': f'dock:{name}', 'kind': 'dock',
                    **dict(zip(('x', 'y', 'yaw'), _p(*stag))),
                    'neighbors': _sorted_nb(stag[0], stag[1], nb), 'docked': _p(*docked)})
    geo = {c['id']: charger_geometry(c) for c in res['chargers']}
    for c in res['chargers']:
        _m, st, dk = geo[c['id']]
        nx, ny = math.cos(c['yaw']), math.sin(c['yaw'])
        nb = [_p(*geo[o['id']][2]) for o in res['chargers'] if o is not c]
        nb.append(_p(st[0] + nx * SECOND_IN_LINE, st[1] + ny * SECOND_IN_LINE, st[2]))
        out.append({'id': f"charger:{c['id']}", 'kind': 'charger',
                    **dict(zip(('x', 'y', 'yaw'), _p(*st))),
                    'neighbors': _sorted_nb(st[0], st[1], nb), 'docked': _p(*dk)})
    for z in zones:
        qs = queue_points(z, zones, obstacles)
        x0, x1, y0, y1 = _bbox(z['polygon'])
        xm, ym = (x0 + x1) / 2, (y0 + y1) / 2
        for side, x, y, yaw, back in qs:
            nb = [_p(xm, ym, yaw), _p(x + back[0] * SECOND_IN_LINE, y + back[1] * SECOND_IN_LINE,
                                      yaw)]
            nb += [_p(ox, oy, oyaw) for oside, ox, oy, oyaw, _b in qs if oside != side]
            out.append({'id': f"queue:{z['id']}:{side}", 'kind': 'queue',
                        **dict(zip(('x', 'y', 'yaw'), _p(x, y, yaw))),
                        'neighbors': _sorted_nb(x, y, nb)})
    return out
