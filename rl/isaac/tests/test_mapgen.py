"""
지도 변형 (mapgen·map_bundle) 시험 — 명목 지도 재현, 물체 묶음, 변형 규칙, 재계획 제외 코드, 2D 광선, 타일·메시.

경로 풀 재현 시험은 held-out 풀 캐시가 있을 때만 돈다 (AMR_ISAAC_RUNS/assets 또는 b1-binfra 의 assets).
"""

import json
import math
import os

import numpy as np
import pytest

from amr_isaac import eval_sets as ES
from amr_isaac import map_bundle as MB
from amr_isaac import mapgen as MG
from amr_isaac import paths as PA
from amr_isaac import warehouse_geom as W

HERE = os.path.dirname(os.path.abspath(__file__))
SUITE = os.path.join(HERE, '..', 'eval_sets', 'b1_map_suite_v1.json')
EVAL_SET = os.path.join(HERE, '..', 'eval_sets', 'b1_eval_sets_v1.json')


def _assets():
    for d in (os.path.join(os.environ.get('AMR_ISAAC_RUNS', ''), 'assets'),
              '/home/CAPYI_AI/runs/bonus1/b1-mapgen/assets',
              '/home/CAPYI_AI/runs/bonus1/b1-binfra/assets'):
        p = os.path.join(d, 'paths_58eaa34ae5_s1.npz')
        if os.path.exists(p):
            return p
    return None


POOL = _assets()
need_pool = pytest.mark.skipif(POOL is None, reason='held-out 경로 풀 캐시 없음')


@pytest.fixture(scope='module')
def nominal():
    return MG.nominal_layout()


@pytest.fixture(scope='module')
def groups(nominal):
    return MG.object_groups(nominal['names'], nominal['obb'])


@pytest.fixture(scope='module')
def suite():
    return MB.load_suite(SUITE)


def test_nominal_layout_matches_obstacles_2d(nominal):
    obb = W.obstacles_2d(W.load_boxes())
    assert np.array_equal(nominal['obb'], obb)
    assert nominal['obb'].shape == (129, 5)
    # 모든 띠 안 상자가 LiDAR 평면을 지난다 (2D 광선 대조의 전제)
    assert np.all((nominal['zr'][:, 0] <= MG.LIDAR_Z) & (nominal['zr'][:, 1] >= MG.LIDAR_Z))


def test_object_groups(nominal, groups):
    gid, names, anchor = groups
    assert len(names) == 44
    fixed = [nominal['names'][i] for i in np.flatnonzero(gid == MG.FIXED)]
    assert len(fixed) == 8
    assert all(n.startswith('warehouse_walls') or MG.model_of(n) in MG.WALL_PLATES for n in fixed)
    for i, n in enumerate(nominal['names']):
        m = MG.model_of(n)
        if m.startswith('rack_marker_'):
            g = gid[i]
            assert names[g].startswith('rack_')
            assert math.hypot(*(nominal['obb'][i, :2] - anchor[g])) < 0.55
        if m.startswith('pillar_') and '_marker_' in m:
            assert names[gid[i]] == m.split('_marker')[0]
    # 물체 종류 수: 랙 25 · 기둥 8 · 도크 상자 8 · 충전기 3
    kinds = [n.split('_')[0] for n in names]
    assert kinds.count('rack') == 25 and kinds.count('pillar') == 8
    assert kinds.count('dock') == 8 and kinds.count('charging') == 3


def _cond(kind, **kw):
    c = {'name': 'x', 'kind': kind, 'seed': 7}
    c.update(kw)
    return c


def test_identity_variants(nominal, groups):
    gid, _n, anchor = groups
    for cond in (_cond('jitter', sigma_xy=0.0, sigma_yaw_deg=0.0), _cond('scale', lo=1.0, hi=1.0),
                 _cond('remove', p=0.0)):
        v = MG.make_variant(nominal, gid, anchor, cond, 0)
        assert np.array_equal(v['obb'], nominal['obb']), cond['kind']
        assert np.array_equal(v['zr'], nominal['zr'])


def test_variant_deterministic_and_map_dependent(nominal, groups, suite):
    gid, _n, anchor = groups
    for cond in suite['conditions']:
        if cond['kind'] == 'add':
            continue
        a = MG.make_variant(nominal, gid, anchor, cond, 3)
        b = MG.make_variant(nominal, gid, anchor, cond, 3)
        c = MG.make_variant(nominal, gid, anchor, cond, 4)
        assert np.array_equal(a['obb'], b['obb'])
        assert not (a['obb'].shape == c['obb'].shape and np.array_equal(a['obb'], c['obb']))


def test_jitter_is_rigid_and_walls_fixed(nominal, groups):
    gid, _n, anchor = groups
    v = MG.make_variant(nominal, gid, anchor, _cond('jitter', sigma_xy=0.6, sigma_yaw_deg=12.0,
                                                    clip=3.0), 1)
    fixed = gid == MG.FIXED
    assert np.array_equal(v['obb'][fixed], nominal['obb'][fixed])
    assert np.array_equal(v['obb'][:, 2:4], nominal['obb'][:, 2:4])
    for g in range(anchor.shape[0]):
        m = np.flatnonzero(gid == g)
        if m.size < 2:
            continue
        d0 = np.hypot(*(nominal['obb'][m[0], :2] - nominal['obb'][m[1:], :2]).T)
        d1 = np.hypot(*(v['obb'][m[0], :2] - v['obb'][m[1:], :2]).T)
        assert np.allclose(d0, d1, atol=1e-9)
        dyaw = v['obb'][m, 4] - nominal['obb'][m, 4]
        assert np.allclose(dyaw, dyaw[0], atol=1e-12)
    # 물체의 yaw 는 그 물체의 회전만큼 돈다 (apply_jitter 의 moves 와 같다)
    obb2, moves = MG.apply_jitter(nominal['obb'], gid, anchor, np.random.default_rng([5, 1]),
                                  0.6, math.radians(12.0))
    for g in range(anchor.shape[0]):
        m = gid == g
        assert np.allclose(obb2[m, 4] - nominal['obb'][m, 4], moves[g, 2], atol=1e-12)
        rel0 = nominal['obb'][m, :2] - anchor[g]
        c, s_ = math.cos(moves[g, 2]), math.sin(moves[g, 2])
        exp = anchor[g] + rel0 @ np.array([[c, s_], [-s_, c]]) + moves[g, :2]
        assert np.allclose(obb2[m, :2], exp, atol=1e-12)
    # 자르기: 이동 ≤ 3σ (축마다)
    shift = v['obb'][~fixed, :2] - nominal['obb'][~fixed, :2]
    assert np.abs(shift).max() < 3 * 0.6 + 2.5        # 회전 지렛대(랙 반대각 ≤ 1.2 m) 포함 여유


def test_scale_keeps_markers_on_faces(nominal, groups):
    gid, names, anchor = groups
    v = MG.make_variant(nominal, gid, anchor, _cond('scale', lo=0.75, hi=1.25), 0)
    for i, n in enumerate(nominal['names']):
        if not MG.model_of(n).startswith('rack_marker_'):
            continue
        g = gid[i]
        main = [k for k, nn in enumerate(nominal['names']) if MG.model_of(nn) == names[g]][0]
        s = v['obb'][main, 3] / nominal['obb'][main, 3]
        d_nom = np.hypot(*(nominal['obb'][i, :2] - nominal['obb'][main, :2]))
        d_var = np.hypot(*(v['obb'][i, :2] - v['obb'][main, :2]))
        assert d_var == pytest.approx(s * d_nom, abs=1e-9)


def test_remove_drops_whole_objects(nominal, groups):
    gid, _n, anchor = groups
    v = MG.make_variant(nominal, gid, anchor, _cond('remove', p=0.5), 2)
    kept = set(v['gid'].tolist())
    for g in range(anchor.shape[0]):
        n_nom = int((gid == g).sum())
        n_var = int((v['gid'] == g).sum())
        assert n_var in (0, n_nom)
    assert MG.FIXED in kept
    assert v['info']['removed_objects'] == anchor.shape[0] - len(kept - {MG.FIXED})


def test_add_respects_overlap_and_keepout(nominal, groups):
    gid, _n, anchor = groups
    path = np.stack([np.linspace(-20.0, -5.0, 301), np.full(301, 6.0)], 1)   # 랙 열 A·B 사이 통로
    keep = np.array([path[0], path[-1], path[150]])     # 가운데 점: 상자가 놓이는 구간 안의 금지 점
    cond = _cond('add', n=10, half_lo=0.15, half_hi=0.45, arc_lo=0.3, arc_hi=0.7,
                 lateral_sigma=0.25, keepout_m=1.0, height=1.0)
    v = MG.make_variant(nominal, gid, anchor, cond, 0, host_paths=[path], keep_pts=keep)
    new = v['obb'][v['gid'] == MG.ADDED]
    assert new.shape[0] == 10
    assert np.all(MG.rect_point_distance(new, keep) >= 1.0)
    for i, b in enumerate(new):
        others = np.concatenate([nominal['obb'], np.delete(new, i, axis=0)])
        assert not MG.rects_overlap(b, others).any()
    assert np.all(v['zr'][v['gid'] == MG.ADDED] == [0.0, 1.0])


def test_rects_overlap_and_point_distance():
    a = np.array([0.0, 0.0, 1.0, 0.5, 0.0])
    b = np.array([[2.5, 0.0, 1.0, 0.5, 0.0], [1.9, 0.0, 1.0, 0.5, 0.0],
                  [0.0, 1.2, 0.5, 0.5, math.pi / 4]])
    assert MG.rects_overlap(a, b).tolist() == [False, True, True]
    d = MG.rect_point_distance(a, np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 1.5]]))
    assert d[0].tolist() == pytest.approx([0.0, 1.0, math.hypot(1.0, 1.0)])


def test_planner_codes(nominal):
    planner = MG.Planner(nominal['obb'])
    free = np.flatnonzero(planner.dist.ravel() >= 2.0)
    rng = np.random.default_rng(0)
    nx = planner.nx
    # 가까운 두 자유 셀 (3–30 m 경로가 나오게)
    for _ in range(200):
        g, s = rng.choice(free, 2, replace=False)
        gx, gy = PA.cell_center(g, nx, planner.origin)
        sx, sy = PA.cell_center(s, nx, planner.origin)
        if 5.0 < math.hypot(gx - sx, gy - sy) < 12.0:
            break
    code, pts, L = planner.plan(g, s)
    assert code == MG.VALID and 3.0 <= L <= 30.0 and pts.shape[0] <= MG.PATH_PAD
    assert np.allclose(pts[0], [sx, sy]) and np.allclose(pts[-1], [gx, gy])
    box = lambda x, y, h: np.array([[x, y, h, h, 0.0]])  # noqa: E731
    p2 = MG.Planner(np.concatenate([nominal['obb'], box(sx, sy, 0.3)]))
    assert p2.plan(g, s)[0] == MG.START_BLOCKED
    p3 = MG.Planner(np.concatenate([nominal['obb'], box(gx, gy, 0.3)]))
    assert p3.plan(g, s)[0] == MG.GOAL_BLOCKED
    ring = np.array([[gx + 1.0, gy, 0.05, 1.05, 0.0], [gx - 1.0, gy, 0.05, 1.05, 0.0],
                     [gx, gy + 1.0, 1.05, 0.05, 0.0], [gx, gy - 1.0, 1.05, 0.05, 0.0]])
    p4 = MG.Planner(np.concatenate([nominal['obb'], ring]))
    assert p4.plan(g, s)[0] == MG.NO_PATH


@need_pool
def test_replan_reproduces_pool_paths(nominal):
    pool = dict(np.load(POOL))
    doc = ES.load_sets(EVAL_SET)
    planner = MG.Planner(nominal['obb'])
    ids = doc['sets']['main_3_15'][:40] + doc['sets']['stress_3_15'][:10] + \
        doc['sets']['main_3_30'][:10]
    for pid in ids:
        g, s = pool['meta'][pid]
        code, pts, _L = planner.plan(g, s)
        ref = pool['pts'][pool['offs'][pid]:pool['offs'][pid + 1]]
        assert code == MG.VALID
        assert pts.shape == ref.shape and np.array_equal(pts, ref), pid


def test_raycast_2d_against_marching():
    rng = np.random.default_rng(1)
    obb = np.array([[3.0, 0.5, 0.5, 0.3, 0.4], [-2.0, -1.0, 0.2, 1.0, -0.3],
                    [0.0, 4.0, 2.0, 0.2, 0.0]])
    zr = np.tile([0.0, 1.0], (3, 1))
    ang = rng.uniform(-math.pi, math.pi, 200)
    r = MG.raycast_2d(np.array([0.1, -0.2]), ang, obb, zr, max_range=25.0)
    s = np.arange(0.0, 25.0, 0.0005)
    for k, a in enumerate(ang):
        pts = np.array([0.1, -0.2]) + s[:, None] * [math.cos(a), math.sin(a)]
        inside = (MG.rect_point_distance(obb, pts) == 0.0).any(axis=0)
        ref = s[np.argmax(inside)] if inside.any() else np.nan
        if np.isnan(ref):
            assert np.isnan(r[k])
        else:
            assert abs(r[k] - ref) <= 1e-3
    # 평면을 지나지 않는 상자는 보지 않는다
    zr2 = zr.copy()
    zr2[:, 0] = 0.5
    assert np.all(np.isnan(MG.raycast_2d(np.array([0.1, -0.2]), ang, obb, zr2)))


def test_tile_offsets_and_mesh(nominal):
    off = MG.tile_offsets(50, (66.0, 46.0))
    assert off.shape == (50, 2) and np.array_equal(off[0], [0.0, 0.0])
    assert len({tuple(x) for x in off.tolist()}) == 50
    with pytest.raises(ValueError):
        MG.tile_offsets(5, (50.0, 46.0))
    with pytest.raises(ValueError):
        MG.tile_offsets(60, (66.0, 46.0))
    v, t = MG.box_mesh(nominal['obb'][:5], nominal['zr'][:5])
    boxes = [W.Box('b', np.array([o[0], o[1], 0.5 * (z[0] + z[1])]),
                   np.array([2 * o[2], 2 * o[3], z[1] - z[0]]), W.rpy_matrix(0, 0, o[4]))
             for o, z in zip(nominal['obb'][:5], nominal['zr'][:5])]
    v2, t2 = W.triangle_mesh(boxes)
    assert np.array_equal(v, v2) and np.array_equal(t, t2)
    # 원래 SDF 상자 메시와 같은 모양 (꼭짓점 집합이 같다, 부동소수 오차 안)
    inb = MG.inband_boxes(W.load_boxes())[:5]
    v3, _t3 = W.triangle_mesh(inb)
    assert np.allclose(v, v3, atol=1e-5)


def test_suite_file_valid(suite):
    assert [c['name'] for c in suite['conditions']] == [
        'jitter_0p15', 'jitter_0p30', 'jitter_0p60', 'scale', 'remove', 'add']
    bad = json.loads(json.dumps(suite))
    bad['conditions'][0]['kind'] = 'warp'
    with pytest.raises(ValueError):
        MB.validate_suite(bad)


@need_pool
def test_bundle_small_build(nominal):
    pool = dict(np.load(POOL))
    doc = ES.load_sets(EVAL_SET)
    doc_small = dict(doc)
    doc_small['sets'] = {'main_3_15': doc['sets']['main_3_15'][:24],
                         'stress_3_15': doc['sets']['stress_3_15'][:8]}
    suite = {'format': MG.SUITE_FORMAT, 'sets': ['main_3_15', 'stress_3_15'],
             'maps_per_condition': 2, 'assign_seed': 5, 'tile_spacing_m': [66.0, 46.0],
             'conditions': [_cond('jitter', name='zero', sigma_xy=0.0, sigma_yaw_deg=0.0),
                            _cond('add', name='add', n=6, half_lo=0.15, half_hi=0.45,
                                  arc_lo=0.3, arc_hi=0.7, lateral_sigma=0.25, keepout_m=1.0,
                                  height=1.0)]}
    arr, summ = MB.build_bundle(suite, doc_small, pool, nominal=nominal, workers=1)
    T = arr['tile_offset'].shape[0]
    assert T == 1 + 2 * 2 + 1 and int(arr['far_tile']) == T - 1
    assert arr['row_cond'].shape[0] == 2 * 32
    zero = arr['row_cond'] == 0
    # σ = 0 지터 = 명목 지도 → 모든 행 유효, 경로가 풀과 비트 단위로 같다, 재분류도 원래 집합 그대로
    assert np.all(arr['row_code'][zero] == MG.VALID)
    assert not arr['row_changed'][zero].any()
    for r in np.flatnonzero(zero):
        pid = arr['row_pid'][r]
        ref = pool['pts'][pool['offs'][pid]:pool['offs'][pid + 1]]
        assert np.array_equal(MB.row_path(arr, r), ref)
    is_main = arr['row_set'][zero] == 0
    assert np.array_equal(arr['row_main_variant'][zero], is_main)
    # 같은 suite 다시 → 같은 배열 (난수 고정), 작업 수와 무관
    arr2, _s = MB.build_bundle(suite, doc_small, pool, nominal=nominal, workers=2)
    for k in arr:
        if arr[k].dtype.kind in 'fc':
            assert np.array_equal(arr[k], arr2[k], equal_nan=True), k
        else:
            assert np.array_equal(arr[k], arr2[k]), k
    assert summ['conditions']['zero']['main_3_15']['valid'] == 24
    # 타일 사각형: 0 과 먼 사본 = 명목, add 타일 = 명목 + 새 상자
    o0 = MB.tile_rects(arr, 0)[0]
    assert np.array_equal(o0, nominal['obb'])
    assert np.array_equal(MB.tile_rects(arr, T - 1)[0], nominal['obb'])
    t_add = int(np.flatnonzero(arr['tile_cond'] == 1)[0])
    ob, _z, g = MB.tile_rects(arr, t_add)
    assert np.array_equal(ob[g != MG.ADDED], nominal['obb']) and (g == MG.ADDED).sum() == 6


@need_pool
def test_bundle_clearance_uses_variant_map(nominal):
    """
    재계획 경로의 끝 직진 여유·1 차 모집단 분류는 그 행의 변형 지도 사각형으로 계산한다 (명목 사각형이 아니다).

    리뷰 반영 (2026-10-08): map_bundle 의 여유 계산을 명목 obb 로 바꾼 변이 (R3) 를 기존 시험이 잡지 못했다.
    """
    pool = dict(np.load(POOL))
    doc = ES.load_sets(EVAL_SET)
    doc_small = dict(doc)
    doc_small['sets'] = {'main_3_15': doc['sets']['main_3_15'][:24],
                         'stress_3_15': doc['sets']['stress_3_15'][:16]}
    suite = {'format': MG.SUITE_FORMAT, 'sets': ['main_3_15', 'stress_3_15'],
             'maps_per_condition': 2, 'assign_seed': 5, 'tile_spacing_m': [66.0, 46.0],
             'conditions': [_cond('jitter', name='j', sigma_xy=0.6, sigma_yaw_deg=12.0, clip=3.0),
                            _cond('scale', name='s', lo=0.75, hi=1.25)]}
    arr, _s = MB.build_bundle(suite, doc_small, pool, nominal=nominal, workers=1)
    valid = np.flatnonzero(arr['row_code'] == MG.VALID)
    assert valid.shape[0] > 0
    differs = 0
    for r in valid:
        pts = MB.row_path(arr, r)
        c_var = MG.end_clearance(pts, MB.tile_rects(arr, int(arr['row_tile'][r]))[0])
        assert arr['row_clear'][r] == c_var
        assert bool(arr['row_main_variant'][r]) == (c_var >= ES.THRESHOLD)
        differs += int(MG.end_clearance(pts, nominal['obb']) != c_var)
    # 시험이 명목·변형 사각형을 가를 수 있어야 의미가 있다
    assert differs > 0


def test_goal_assignment_balanced_and_deterministic():
    goals = np.array([5, 3, 9, 3, 11, 5, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30])
    a = MB.goal_assignment(goals, 4, 7)
    assert set(a) == set(np.unique(goals).tolist())
    cnt = np.bincount(list(a.values()), minlength=4)
    assert cnt.max() - cnt.min() <= 1 and cnt.min() >= 1
    assert a == MB.goal_assignment(goals, 4, 7)
    assert a != MB.goal_assignment(goals, 4, 8)
