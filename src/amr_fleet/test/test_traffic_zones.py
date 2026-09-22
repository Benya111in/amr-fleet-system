"""
traffic_zones: 구역 설정 · 지도 유도 · 경로 위 방문 · 진입 토큰(우선순위 · 기아 방지) · 창고 배치 파일.

창고 배치 검사는 feature/simulation-world 의 gen_warehouse_world.py 상수(아래 WAREHOUSE_*)로 점유 격자를
그려 config/traffic_zones.yaml 의 구역 · 포켓이 실제 배치와 맞는지 본다.
"""

import math
import pathlib

import numpy as np
import pytest

from amr_fleet.traffic_geometry import GridSpec, points_in_polygon
from amr_fleet.traffic_zones import (
    CORRIDOR, INTERSECTION, TokenRequest, ZoneMap, ZoneTokenManager, ZoneVisit, clearance_map,
    derive_zones_from_grid, load_layout, request_chain, zone_visits, zones_from_config,
)

CONFIG = pathlib.Path(__file__).resolve().parents[1] / 'config' / 'traffic_zones.yaml'


def sq(x0, y0, x1, y1):
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


# ---------------------------------------------------------------------- 설정
def test_zones_from_config_ok():
    zs = zones_from_config([
        {'id': 'x1', 'kind': 'Intersection', 'polygon': sq(0, 0, 2, 2)},
        {'id': 'c1', 'kind': 'corridor', 'polygon': sq(2, 0, 8, 1), 'same_direction': False},
        {'id': 'c2', 'kind': 'corridor', 'polygon': sq(8, 0, 9, 1)},
    ], corridor_same_direction=True)
    assert [z.kind for z in zs] == [INTERSECTION, CORRIDOR, CORRIDOR]
    assert zs[0].area_m2 == pytest.approx(4.0) and zs[0].centroid == (1.0, 1.0)
    assert [z.same_direction for z in zs] == [False, False, True]
    assert zones_from_config(None) == []


@pytest.mark.parametrize('entries,msg', [
    ('x', '목록'),
    ([3], '사전'),
    ([{'id': 'a', 'kind': 'corridor', 'polygon': sq(0, 0, 1, 1), 'z': 1}], '모르는 키'),
    ([{'id': 'a b', 'kind': 'corridor', 'polygon': sq(0, 0, 1, 1)}], 'id'),
    ([{'id': 'a', 'kind': 'corridor', 'polygon': sq(0, 0, 1, 1)}] * 2, '중복'),
    ([{'id': 'a', 'kind': 'room', 'polygon': sq(0, 0, 1, 1)}], 'kind'),
    ([{'id': 'a', 'kind': 'corridor', 'polygon': [[0, 0], [1, 1]]}], '3개'),
    ([{'id': 'a', 'kind': 'corridor', 'polygon': [[0, 0], [1, 'x'], [1, 1]]}], r'zones\[0\]'),
    ([{'id': 'a', 'kind': 'corridor', 'polygon': [[0, 0], [1], [1, 1]]}], '꼭짓점'),
    ([{'id': 'a', 'kind': 'corridor', 'polygon': [[0, 0], [1, math.nan], [1, 1]]}], '유한'),
    ([{'id': 'a', 'kind': 'corridor', 'polygon': [[0, 0], [1, 1], [2, 2]]}], '넓이'),
    ([{'id': 'a', 'kind': 'corridor', 'polygon': sq(0, 0, 1, 1), 'same_direction': 'y'}],
     'same_direction'),
])
def test_zones_from_config_errors(entries, msg):
    with pytest.raises(ValueError, match=msg):
        zones_from_config(entries)


def test_load_layout_errors(tmp_path):
    p = tmp_path / 'z.yaml'
    p.write_text('')
    assert load_layout(str(p)) == ([], [])
    for text, msg in [('[1, 2]', '사전'), ('foo: 1', '모르는 최상위 키'),
                      ('frame_id: odom', 'map 만'), ('pockets: 3', 'pockets')]:
        p.write_text(text)
        with pytest.raises(ValueError, match=msg):
            load_layout(str(p))
    with pytest.raises(OSError):
        load_layout(str(tmp_path / 'none.yaml'))


def test_zone_map_lookup_and_mask():
    zs = zones_from_config([
        {'id': 'a', 'kind': 'intersection', 'polygon': sq(0, 0, 2, 2)},
        {'id': 'b', 'kind': 'corridor', 'polygon': sq(1, 0, 5, 1)},      # 겹침: 먼저 정의한 a 가 이긴다
    ])
    zm = ZoneMap(zs)
    assert len(zm) == 2 and 'a' in zm and zm.ids() == ['a', 'b'] and zm.get('b').kind == CORRIDOR
    assert zm.zones_at(np.array([[1.5, 0.5], [3, 0.5], [9, 9]])) == ['a', 'b', None]
    assert zm.zone_at(0.5, 1.5) == 'a'
    spec = GridSpec(1.0, 0.0, 0.0, 6, 3)
    assert zm.mask(spec).sum() == 7            # 셀 중심 (0.5, 0.5)…: a 4칸 + b 3칸
    assert zm.mask(spec, ['b']).sum() == 3
    with pytest.raises(ValueError):
        ZoneMap(zs + zs)
    with pytest.raises(ValueError):
        ZoneMap(labels=np.zeros((2, 2), dtype=np.int32))


# ---------------------------------------------------------------------- 지도 유도
def _plus_map():
    """20 x 20 m, 0.1 m: 가로 · 세로 3 m 도로 십자 + 동쪽 끝 폭 0.6 m 좁은 통로 (막다른 길)."""
    res = 0.1
    occ = np.ones((200, 200), dtype=bool)
    occ[85:115, 10:145] = False          # 가로 도로 y 8.5..11.5, x 1..14.5
    occ[10:190, 85:115] = False          # 세로 도로
    occ[97:103, 145:195] = False         # 좁은 통로 y 9.7..10.3, x 14.5..19.5
    return occ, GridSpec(res, 0.0, 0.0, 200, 200)


def test_derive_zones_from_grid_finds_intersection_and_corridor():
    occ, spec = _plus_map()
    labels, zones = derive_zones_from_grid(occ, spec, passage_radius=0.22)
    kinds = {z.kind for z in zones}
    assert kinds == {INTERSECTION, CORRIDOR}
    zm = ZoneMap([], zones, labels, spec)
    inter = zm.zone_at(10.0, 10.0)
    corr = zm.zone_at(18.5, 10.0)
    assert inter is not None and zm.get(inter).kind == INTERSECTION
    assert corr is not None and zm.get(corr).kind == CORRIDOR and zm.get(corr).source == 'map'
    assert zm.zone_at(3.0, 10.0) is None     # 넓은 도로 (교차로 밖)
    clear = clearance_map(occ, spec.resolution)
    assert clear[100, 180] == pytest.approx(0.25, abs=0.06) and clear[0, 0] < 0


# ---------------------------------------------------------------------- 경로 위 방문
def test_zone_visits_and_request_chain():
    zm = ZoneMap(zones_from_config([
        {'id': 'x', 'kind': 'intersection', 'polygon': sq(4, -1, 6, 1)},
        {'id': 'c', 'kind': 'corridor', 'polygon': sq(6.5, -0.5, 10, 0.5)},
        {'id': 'y', 'kind': 'intersection', 'polygon': sq(14, -1, 16, 1)},
    ]))
    path = np.array([[0.0, 0.0], [20.0, 0.0]])
    v = zone_visits(path, zm, step=0.1)
    assert [x.zone_id for x in v] == ['x', 'c', 'y']
    assert v[0].s_in == pytest.approx(4.0, abs=0.11)
    assert v[1].s_out == pytest.approx(10.0, abs=0.11)
    assert v[0].direction() == pytest.approx((1.0, 0.0))
    assert ZoneVisit('q', 0, 0.1, (0, 0), (0.1, 0)).direction() is None
    assert zone_visits(path, zm, max_s=3.0) == []
    assert zone_visits(None, zm) == [] and zone_visits(path, ZoneMap()) == []
    assert request_chain(v, approach_m=2.5, chain_gap_m=1.0) == []       # 첫 구역이 4 m 앞
    near = zone_visits(np.array([[2.0, 0.0], [20.0, 0.0]]), zm, step=0.1)
    chain = request_chain(near, approach_m=2.5, chain_gap_m=1.0)
    assert [c.zone_id for c in chain] == ['x', 'c']                     # y 는 4 m 떨어져 따로
    inside = zone_visits(np.array([[5.0, 0.0], [20.0, 0.0]]), zm, step=0.1)
    assert [c.zone_id for c in request_chain(inside, 2.5, 1.0)] == ['c']  # 이미 x 안
    back = zone_visits(np.array([[3.0, 0.0], [5.0, 0.0], [3.0, 0.2], [5.0, 0.4]]), zm, step=0.1)
    assert [c.zone_id for c in request_chain(back, 2.5, 5.0)] == ['x']   # 재방문은 한 번만


# ---------------------------------------------------------------------- 진입 토큰
def _req(rid, zones, prio=100, deadline=None, direction=(1.0, 0.0)):
    return TokenRequest(rid, tuple(zones), {z: direction for z in zones}, prio, deadline)


def _up(**kw):
    return {k: set(v) for k, v in kw.items()}


def test_token_priority_then_deadline_then_wait():
    tm = ZoneTokenManager({'x': False})
    reqs = {'a': _req('a', 'x', 50), 'b': _req('b', 'x', 200), 'c': _req('c', 'x', 200, 5.0)}
    out = tm.update(0.0, reqs, {}, _up(a='x', b='x', c='x'))
    assert out['c'].granted and not out['a'].granted and not out['b'].granted
    assert out['a'].blockers == ('c',) and tm.holders('x') == ['c'] and tm.held_by('c') == ['x']
    # c 가 구역에 들어갔다 나오면 반납 → 다음은 우선순위가 높은 b
    tm.update(0.5, {k: reqs[k] for k in 'ab'}, _up(c='x'), _up(a='x', b='x', c='x'))
    out = tm.update(1.0, {k: reqs[k] for k in 'ab'}, {}, _up(a='x', b='x', c=''))
    assert out['b'].granted and not out['a'].granted and out['a'].waiting_s == pytest.approx(1.0)
    # 경로가 더는 구역을 지나지 않으면 반납
    out = tm.update(1.5, {'a': reqs['a']}, {}, _up(a='x', b=''))
    assert out['a'].granted and tm.holders('x') == ['a']
    assert tm.grant_log[-1][1:] == ('a', ('x',))


def test_token_chain_is_atomic_and_same_direction():
    tm = ZoneTokenManager({'x': False, 'c': True})
    # b 가 통로 c 를 같은 방향으로 이미 점유 → a 는 같은 방향 추종 허용, d 는 반대 방향이라 불가
    out = tm.update(0.0, {}, _up(b='c'), _up(b='c'), {'b': {'c': (1.0, 0.0)}})
    assert tm.holders('c') == ['b']
    out = tm.update(0.5, {'a': _req('a', 'xc'), 'd': _req('d', 'xc', 250, direction=(-1.0, 0))},
                    _up(b='c'), _up(a='xc', b='c', d='xc'))
    assert not out['d'].granted and 'b' in out['d'].blockers
    # d 가 x 를 못 받았으니 x 도 잡지 않는다(묶음 원자 부여) → a 는 x·c 모두 받는다
    assert out['a'].granted and tm.held_by('a') == ['c', 'x'] and tm.holders('x') == ['a']
    # 방향 모름(None) 이면 추종으로 보지 않는다
    tm2 = ZoneTokenManager({'c': True})
    tm2.update(0.0, {}, _up(b='c'), _up(b='c'), {'b': {'c': None}})
    out = tm2.update(0.5, {'a': _req('a', 'c')}, _up(b='c'), _up(a='c', b='c'))
    assert not out['a'].granted


def test_token_starvation_bound():
    """
    낮은 우선순위 a 는 나중에 온 로봇에게 max_bypass 번 추월당하면 그 구역을 예약받는다 (기아 없음).

    높은 우선순위 로봇이 끊임없이 들어와도 a 앞에 서는 로봇 수 ≤ (같은 주기 경쟁자) + max_bypass.
    """
    tm = ZoneTokenManager({'x': False}, max_bypass=2)
    reqs = {'a': _req('a', 'x', 0), 'h0': _req('h0', 'x', 255)}
    served, t, k = [], 1.0, 0
    holder = None
    while len(served) < 6:
        up = {r: {'x'} for r in reqs}
        if holder is not None:
            up[holder] = set()                                 # 앞 보유자가 나왔다 → 반납
        out = tm.update(t, reqs, {}, up)
        winner = [r for r, d in out.items() if d.granted]
        assert len(winner) == 1
        holder = winner[0]
        served.append(holder)
        del reqs[holder]
        if holder == 'a':
            break
        k += 1
        reqs[f'h{k}'] = _req(f'h{k}', 'x', 255)                # 새 고우선 로봇이 계속 온다
        tm.update(t + 0.1, reqs, {holder: {'x'}}, dict(up, **{holder: {'x'}}))   # 보유자 진입
        t += 0.2
    # h0 은 a 와 같은 주기에 요청 → 우선순위 차이는 추월이 아니다. h1, h2 가 추월 → 그다음은 a
    assert served == ['h0', 'h1', 'h2', 'a']


def test_token_occupancy_registration_forget_and_set_zones():
    tm = ZoneTokenManager({'x': False, 'y': False})
    tm.update(0.0, {}, _up(a='x', b='x'), _up(a='x', b='x'))   # 둘이 이미 x 안: 먼저 온 a 만 등록
    assert tm.holders('x') == ['a']
    tm.update(0.1, {}, _up(a='x', b='x', q='zz'), _up(a='x', b='x'))   # 모르는 구역은 무시
    tm.forget('a')
    assert tm.holders('x') == [] and tm.held_by('a') == []
    tm.update(0.2, {}, _up(b='x'), _up(b='x'))
    assert tm.holders('x') == ['b']
    tm.set_zones({'y': False})
    assert tm.holders('x') == [] and tm.holders('y') == []
    with pytest.raises(ValueError):
        ZoneTokenManager({}, max_bypass=-1)


def test_token_wait_survives_small_route_change():
    tm = ZoneTokenManager({'x': False, 'c': False})
    tm.update(0.0, {'h': _req('h', 'x')}, {}, _up(h='x'))
    out = tm.update(0.5, {'a': _req('a', 'x')}, {}, _up(a='x', h='x'))
    assert not out['a'].granted
    out = tm.update(1.0, {'a': _req('a', 'xc')}, {}, _up(a='xc', h='x'))
    assert out['a'].waiting_s == pytest.approx(0.5)            # 겹치는 구역이 남아 대기 시각 유지
    out = tm.update(1.5, {'a': _req('a', 'c')}, {}, _up(a='c', h='x'))
    assert out['a'].granted                                   # c 는 비어 있어 바로 부여


# ---------------------------------------------------------------------- 창고 배치 파일
# gen_warehouse_world.py (feature/simulation-world) 상수
WAREHOUSE_HALF = (30.0, 20.0)
WAREHOUSE_WALL_T = 0.2
WAREHOUSE_ROWS = (9.0, 3.0, -3.0)
WAREHOUSE_RACK_X = (-18.0, -12.0, -6.0, 0.0, 6.0, 12.0, 18.0)
WAREHOUSE_RACK = (2.0, 1.0)
WAREHOUSE_NARROW = (0.60, 0.0, -10.0)       # 순폭, 중심 x, y (랙 4베이를 90도 돌려 세움)
WAREHOUSE_PILLARS = ((-10, 12), (10, 12), (-10, -12), (10, -12), (-25, 6), (25, 6), (-25, -6),
                     (25, -6))
WAREHOUSE_PILLAR = 0.52
WAREHOUSE_CHARGERS = ((-26.0, -18.4), (-22.0, -18.4), (-18.0, -18.4))


def warehouse_grid(res=0.05):
    hx, hy = WAREHOUSE_HALF
    spec = GridSpec(res, -hx, -hy, int(round(2 * hx / res)), int(round(2 * hy / res)))
    gx, gy = spec.centers()
    occ = (np.abs(gx) > hx - WAREHOUSE_WALL_T) | (np.abs(gy) > hy - WAREHOUSE_WALL_T)

    def box(cx, cy, sx, sy):
        return (np.abs(gx - cx) <= sx / 2) & (np.abs(gy - cy) <= sy / 2)

    for y in WAREHOUSE_ROWS:
        for x in WAREHOUSE_RACK_X:
            occ |= box(x, y, *WAREHOUSE_RACK)
    clear, cx, cy = WAREHOUSE_NARROW
    off = clear / 2 + WAREHOUSE_RACK[1] / 2
    for sx in (-off, off):
        for dy in (-1.0, 1.0):
            occ |= box(cx + sx, cy + dy, WAREHOUSE_RACK[1], WAREHOUSE_RACK[0])
    for px, py in WAREHOUSE_PILLARS:
        occ |= box(px, py, WAREHOUSE_PILLAR, WAREHOUSE_PILLAR)
    for px, py in WAREHOUSE_CHARGERS:
        occ |= box(px, py, 0.4, 0.5)
    return occ, spec


def test_warehouse_layout_matches_world():
    from amr_fleet.traffic_resolution import pockets_from_config
    zones, raw_pockets = load_layout(str(CONFIG), corridor_same_direction=True)
    pockets = pockets_from_config(raw_pockets)
    occ, spec = warehouse_grid()
    clear = clearance_map(occ, spec.resolution)
    zm = ZoneMap(zones)
    ids = [z.zone_id for z in zones]
    assert 'narrow_aisle' in ids and zm.get('narrow_aisle').kind == CORRIDOR
    assert sum(z.kind == INTERSECTION for z in zones) == 12
    # 좁은 통로 중앙선은 구역 안이고 통과 가능 (벽까지 ≥ 반폭 0.2 + 여유 → passage_radius 0.22)
    for y in np.arange(-11.9, -8.0, 0.2):
        assert zm.zone_at(0.0, y) == 'narrow_aisle'
        assert spec.lookup(clear, np.array([[0.0, y]]))[0] >= 0.22
    # 구역끼리 겹치지 않고, 교차로 내부는 대부분 비어 있다
    gx, gy = spec.centers()
    pts = np.stack([gx.ravel(), gy.ravel()], axis=1)
    count = np.zeros(len(pts), dtype=int)
    for z in zones:
        inside = points_in_polygon(pts, z.polygon)
        count += inside
        if z.kind == INTERSECTION:
            assert (~occ.ravel()[inside]).mean() > 0.99, z.zone_id
    assert count.max() == 1
    # 포켓: 구역 밖, 장애물에서 외접 반경 0.36 m 이상, 좁은 통로 중앙선(x=0)에서 떨어져 있다
    assert len(pockets) >= 30 and len({p.pocket_id for p in pockets}) == len(pockets)
    for p in pockets:
        assert zm.zone_at(p.x, p.y) is None, p.pocket_id
        assert spec.lookup(clear, np.array([[p.x, p.y]]))[0] >= 0.36, p.pocket_id
    # 지도 유도(map)도 같은 좁은 통로를 1차선 통로로 찾는다
    coarse = occ[::2, ::2]
    cspec = GridSpec(0.1, spec.origin_x, spec.origin_y, coarse.shape[1], coarse.shape[0])
    labels, auto = derive_zones_from_grid(coarse, cspec, passage_radius=0.22)
    auto_map = ZoneMap([], auto, labels, cspec)
    hit = auto_map.zone_at(0.0, -10.0)
    assert hit is not None and auto_map.get(hit).kind == CORRIDOR
