"""
보너스 2 정답(GT) 생성기 시험 — src/amr_simulation/worlds/gen_warehouse_world.py 의 --gt-dir 경로.

- 기본 실행(warehouse.sdf)은 바이트 단위로 그대로이고, --gt-dir 만 주면 SDF 를 쓰지 않는다.
- 0.05 m 단면은 기존 평가기의 GT(amr_localization/world_geometry.rasterize)와 셀 단위로 같다 (같은 제외 집합).
- 합집합 표면 표본의 점 수가 해석값과 같다 (가려진 면 제거), 높이 대역·단면 규칙, 결정성.
"""

import filecmp
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

from amr_evaluation.slam3d import cloud
import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
REPO = SRC.parent
SIM = SRC / 'amr_simulation'
GEN = SIM / 'worlds' / 'gen_warehouse_world.py'
WORLD = SIM / 'worlds' / 'warehouse.sdf'
MODELS = SIM / 'models'

pytestmark = pytest.mark.skipif(not GEN.is_file(), reason='amr_simulation 이 이 워크스페이스에 없다')


def load_gen(tag='gen_gt'):
    """생성기를 새 모듈로 읽는다 (seed 난수 상태도 처음부터)."""
    spec = importlib.util.spec_from_file_location(tag, GEN)
    gen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)
    return gen


@pytest.fixture(scope='module')
def gen():
    return load_gen()


def world_geometry():
    """기존 평가기의 GT 모듈 (설치본 또는 소스 트리의 amr_localization)."""
    try:
        from amr_localization import world_geometry as wg
    except ImportError:
        sys.path.insert(0, str(SRC / 'amr_localization'))
        try:
            from amr_localization import world_geometry as wg
        except ImportError:
            pytest.skip('amr_localization 을 import 할 수 없다')
    return wg


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def test_default_run_is_byte_identical(tmp_path, gen):
    out = tmp_path / 'warehouse.sdf'
    gen.main([str(out)])
    assert filecmp.cmp(out, WORLD, shallow=False)
    # build_sdf 는 호출마다 같은 문자열 (seed 를 다시 건다) — GT 가 커밋된 월드와 같은 상자 배치를 쓴다는 전제
    assert gen.build_sdf()[0] == gen.build_sdf()[0] == WORLD.read_text()


def test_gt_flag_writes_no_sdf(tmp_path, gen):
    before = sha(WORLD)
    gen.main(['--gt-dir', str(tmp_path / 'gt'), '--gt-no-points', '--gt-res', '0.1'])
    assert sha(WORLD) == before
    assert not list(tmp_path.rglob('*.sdf'))
    meta = json.loads((tmp_path / 'gt' / 'gt_meta.json').read_text())
    assert meta['matches_committed_sdf'] is True
    assert meta['sdf_sha256'] == before
    assert 'points' not in meta


def test_slice_matches_existing_evaluator_gt(gen):
    """0.05 m, 같은 제외 집합 → world_geometry.rasterize 와 셀 단위로 같다 (ADNN₀ 재현의 전제)."""
    wg = world_geometry()
    sdf = gen.build_sdf()[0]
    prims, skipped = gen.gt_primitives(sdf, [str(MODELS)], gen.GT_LEGACY_EXCLUDE)
    assert skipped == []
    mine = gen.gt_raster(prims, 0.20, 0.20, 0.05, gen.GT_BOUNDS)
    loader = wg.WorldLoader([MODELS], wg.DEFAULT_EXCLUDE)
    ref = wg.rasterize(loader.load_world(WORLD), 0.20, 0.05, gen.GT_BOUNDS)
    assert mine.shape == ref.occupied.shape
    assert np.array_equal(mine, ref.occupied)
    # 기본 제외 집합은 shuttle_amr(움직이는 셔틀)을 더 뺀다 — 차이는 그 스폰 자리뿐
    prims2, _ = gen.gt_primitives(sdf, [str(MODELS)])
    diff = gen.gt_raster(prims2, 0.20, 0.20, 0.05, gen.GT_BOUNDS) != ref.occupied
    assert diff.any()
    shuttle = [p for p in prims if p.model == 'shuttle_amr']
    lo = np.min([p.aabb()[0] for p in shuttle], axis=0)
    hi = np.max([p.aabb()[1] for p in shuttle], axis=0)
    rows, cols = np.nonzero(diff)
    x = gen.GT_BOUNDS[0] + (cols + 0.5) * 0.05
    y = gen.GT_BOUNDS[2] + (rows + 0.5) * 0.05
    assert np.all((x >= lo[0]) & (x <= hi[0]) & (y >= lo[1]) & (y <= hi[1]))


def test_slice_height_matches_config(gen):
    wg = world_geometry()
    assert gen.GT_SLICE_Z[0] == pytest.approx(wg.scan_plane_height(str(REPO / 'config')))


def _synthetic(gen):
    """바닥 2x2 평면 + 1x1x1 상자(바닥 위) + 0.5x0.5x0.5 상자(그 위)."""
    eye = np.eye(4)

    def at(x, y, z):
        t = eye.copy()
        t[:3, 3] = [x, y, z]
        return t
    return [gen.GtPrim('floor', 'ground_plane', 'floor', 'plane', (2.0, 2.0), at(0, 0, 0)),
            gen.GtPrim('big', 'rack_X', 'rack', 'box', (1.0, 1.0, 1.0), at(0, 0, 0.5)),
            gen.GtPrim('small', 'rack_X_box_s', 'cargo', 'box', (0.5, 0.5, 0.5),
                       at(0, 0, 1.25))]


def test_surface_points_count_matches_analytic(gen):
    """
    가려진 면을 뺀 점 수가 해석값과 같다.

    간격 0.05: 바닥 1600 − 상자 아래 400 = 1200, 큰 상자 옆 4x400 + 윗면 400 − 100 (작은 상자 아래) = 1900,
    작은 상자 옆 4x100 + 윗면 100 = 500. 두 상자의 바닥면은 가려져 0.
    """
    prims = _synthetic(gen)
    pts, counts = gen.gt_surface_points(prims, spacing=0.05)
    assert counts == {'floor': 1200, 'rack': 1900, 'cargo': 500}
    assert len(pts) == 3600
    xyz = cloud.xyz_of(pts)
    # 어떤 점도 다른 고체 안(바깥 법선 반대쪽 깊숙이)에 있지 않다
    for p in prims[1:]:
        lo, hi = p.aabb()
        strictly = np.all((xyz > lo + 1e-3) & (xyz < hi - 1e-3), axis=1)
        assert not strictly.any()
    assert not np.any((xyz[:, 2] < -1e-6))
    # 바닥 점은 큰 상자 발자국 밖에만
    fl = xyz[pts['label'] == gen.GT_CLASSES.index('floor')]
    assert np.all((np.abs(fl[:, 0]) > 0.5) | (np.abs(fl[:, 1]) > 0.5))


def test_points_outside_room_are_dropped(gen):
    """
    방 경계(x = HALF_X)에 걸친 상자: 바깥쪽 절반의 윗면·옆면 점은 버린다 (실내 센서가 볼 수 없다).

    월드 자체에서는 면 단위 사전 거르기·천장 반공간 가림이 같은 일을 먼저 해서 이 점 필터를 가를 수 없다 —
    그래서 합성 상자로 따로 시험한다.
    """
    t = np.eye(4)
    t[:3, 3] = [gen.HALF_X, 0.0, 0.5]
    p = gen.GtPrim('edge', 'rack_E', 'rack', 'box', (1.0, 1.0, 1.0), t)
    pts, counts = gen.gt_surface_points([p], spacing=0.05)
    xyz = cloud.xyz_of(pts)
    assert np.all(xyz[:, 0] <= gen.HALF_X + 1e-6)
    # 윗면·아랫면은 안쪽 절반(10 x 20)씩, 옆면 y = ±0.5 는 안쪽 절반(10 x 20)씩, 안쪽 x 면 20 x 20 하나
    assert counts['rack'] == 2 * 200 + 2 * 200 + 400


def test_spacing_upper_bound(gen):
    """면 간격은 spacing 이하 (셀 중심 표본): 0.3 m 면을 0.07 로 → 5 표본 (간격 0.06)."""
    eye = np.eye(4)
    eye[:3, 3] = [0, 0, 0.15]
    p = gen.GtPrim('b', 'rack_Y', 'rack', 'box', (0.3, 0.3, 0.3), eye)
    pts, counts = gen.gt_surface_points([p], spacing=0.07)
    assert counts['rack'] == 6 * 25
    top = cloud.xyz_of(pts)[cloud.xyz_of(pts)[:, 2] > 0.29]
    xs = np.unique(np.round(top[:, 0], 6))
    assert len(xs) == 5 and np.allclose(np.diff(xs), 0.06)


def test_band_and_slice_rules(gen):
    eye = np.eye(4)

    def box(z0, z1, yaw=0.0, size=1.0):
        t = eye.copy()
        c, s = math.cos(yaw), math.sin(yaw)
        t[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
        t[:3, 3] = [0, 0, 0.5 * (z0 + z1)]
        return gen.GtPrim('b', 'rack_Z', 'rack', 'box', (size, size, z1 - z0), t)
    bounds = (-2.0, 2.0, -2.0, 2.0)
    shelf = [box(0.5, 1.0)]
    assert not gen.gt_raster(shelf, 0.20, 0.20, 0.05, bounds).any()
    assert not gen.gt_raster(shelf, 0.05, 0.40, 0.05, bounds).any()
    assert gen.gt_raster(shelf, 0.05, 2.0, 0.05, bounds).sum() == 400
    # 45도 돌린 1x1 상자의 발자국 면적 = 1 m² (0.01 셀 10 000 개 ± 경계 셀 1.5 %)
    rot = gen.gt_raster([box(0.0, 0.5, math.pi / 4)], 0.05, 0.40, 0.01, bounds).sum()
    assert abs(rot - 10000) <= 150
    # 대역은 yaw 만 회전한 상자만 — 기울어진 상자는 거부
    tilted = box(0.0, 0.5)
    tilted.R[:] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
    with pytest.raises(ValueError):
        gen.gt_raster([tilted], 0.05, 0.40, 0.05, bounds)


def test_export_is_deterministic_and_readable(tmp_path, gen):
    args = ['--gt-spacing', '0.05', '--gt-res', '0.05', '--gt-roi', '-2', '2', '-12', '-8']
    gen.main(['--gt-dir', str(tmp_path / 'a')] + args)
    gen.main(['--gt-dir', str(tmp_path / 'b')] + args)
    names = sorted(p.name for p in (tmp_path / 'a').iterdir())
    assert names == sorted(p.name for p in (tmp_path / 'b').iterdir())
    for n in names:
        assert filecmp.cmp(tmp_path / 'a' / n, tmp_path / 'b' / n, shallow=False), n
    meta = json.loads((tmp_path / 'a' / 'gt_meta.json').read_text())
    pts = cloud.read_ply(str(tmp_path / 'a' / 'surface_points.ply'))
    assert len(pts) == meta['points'] == sum(meta['points_by_class'].values())
    assert meta['points_by_class']['rack'] > 0 and meta['points_by_class']['vehicle'] == 0
    assert meta['not_in_mesh_or_points'] == [] and meta['skipped'] == []
    xyz = cloud.xyz_of(pts)
    assert xyz[:, 0].min() >= -2 and xyz[:, 0].max() <= 2
    assert xyz[:, 2].min() >= 0 and xyz[:, 2].max() <= gen.WALL_H + 1e-6
    for stem in ('slice_z0.200_r0.050', 'band_z0.050-0.400_r0.050', 'band_z0.050-2.000_r0.050',
                 'dynamic_swept_r0.050'):
        assert (tmp_path / 'a' / f'{stem}.pgm').is_file()
        assert (tmp_path / 'a' / f'{stem}.yaml').is_file()


def test_band_contains_slice_and_world_has_no_overhang(tmp_path, gen):
    """
    대역 0.05–2.0 ⊇ 대역 0.05–0.40 ⊇ 단면 0.20, 그리고 **이 월드에서는 비교 대역 = 단면** 이다.

    랙 선반(2.0 x 1.0)은 하단 데크와 발자국이 같아 돌출이 없다 (보고서의 "운용 대역에는 선반 돌출이 들어간다"는
    이 월드에 맞지 않는다 — evaluation.md §3). 0.01 m 에서도 운용 대역이 더하는 셀은 두께 0.02 m 표지판뿐이다
    ([계산] 2 356 셀). 월드에 돌출물을 넣으면 이 시험이 깨지고, 그때 evaluation.md 의 대역 해석을 고친다.
    """
    from amr_evaluation.slam3d import gridmap
    gen.main(['--gt-dir', str(tmp_path), '--gt-no-points', '--gt-res', '0.05'])
    s = gridmap.read_map_yaml(tmp_path / 'slice_z0.200_r0.050.yaml').occupied
    b1 = gridmap.read_map_yaml(tmp_path / 'band_z0.050-0.400_r0.050.yaml').occupied
    b2 = gridmap.read_map_yaml(tmp_path / 'band_z0.050-2.000_r0.050.yaml').occupied
    assert np.all(b1[s]) and np.all(b2[b1])
    assert np.array_equal(b1, s)
    sdf = gen.build_sdf()[0]
    prims, _ = gen.gt_primitives(sdf, [str(MODELS)])
    extra = gen.gt_raster(prims, 0.05, 2.0, 0.01) & ~gen.gt_raster(prims, 0.20, 0.20, 0.01)
    plates = [p for p in prims if p.label == 'plate']
    only_plates = gen.gt_raster(plates, 0.05, 2.0, 0.01)
    assert extra.any() and np.all(only_plates[extra])
    dyn = gridmap.read_map_yaml(tmp_path / 'dynamic_swept_r0.050.yaml')
    inside = cloud.region_from_raster(dyn.occupied, dyn.resolution, dyn.origin_x, dyn.origin_y)
    # worker_crossing 이 지나는 좁은 통로 입구 앞 (0, -7) 은 동적 영역, 좁은 통로 한가운데 (0, -10) 은 아니다
    assert inside(np.array([[0.0, -7.0], [0.0, -10.0]])).tolist() == [True, False]


def test_adnn0_reproduces_existing_map_value(gen):
    """
    생성기 검증: 0.05 m GT(기존 평가기와 같은 제외 집합)로 현행 slam_toolbox 지도의 ADNN₀ 2.59 cm 를 재현한다.

    기준값 출처: docs/algorithms/slam.md §6.1 표 "ADNN_0 (정렬 전) 2.59 cm" (지도 maps/warehouse.yaml,
    map_quality 0.025914127536902674 — 이 저장소 d8ae80c 에서 map_quality CLI 로 다시 잰 값).
    """
    from amr_evaluation.slam3d import gridmap
    yml = REPO / 'maps' / 'warehouse.yaml'
    if not yml.is_file():
        pytest.skip('maps/warehouse.yaml 없음')
    est = gridmap.read_map_yaml(yml)
    sdf = gen.build_sdf()[0]
    prims, _ = gen.gt_primitives(sdf, [str(MODELS)], gen.GT_LEGACY_EXCLUDE)
    occ = gen.gt_raster(prims, 0.20, 0.20, 0.05)
    gt = gridmap.Grid(occ, np.ones_like(occ), 0.05, gen.GT_BOUNDS[0], gen.GT_BOUNDS[2])
    m = gridmap.grid_metrics(est, gt)
    assert m['adnn0'] == pytest.approx(0.025914127536902674, abs=1e-12)
    assert round(100 * m['adnn0'], 2) == 2.59
    wg = world_geometry()
    sys.path.insert(0, str(SRC / 'amr_localization'))
    try:
        from amr_localization import map_quality
    except ImportError:
        pytest.skip('map_quality import 불가')
    q = map_quality.evaluate(map_quality.load_map(yml), wg.Raster(occ, 0.05, gen.GT_BOUNDS[0],
                                                                  gen.GT_BOUNDS[2]),
                             (0.0, 0.0, 0.0), (-30.0, 30.0, -20.0, 20.0))
    assert q.adnn_initial == pytest.approx(m['adnn0'], abs=1e-12)
    assert (round(q.chamfer, 4), round(q.iou_tol, 3), round(q.recall_tol, 3)) == (0.0367, 0.575,
                                                                                  0.741)


def test_whole_world_points_stay_inside_room(gen):
    """성긴 간격(0.25 m)으로 월드 전체 표본: 모든 점이 실내(외벽 안쪽 면 안, 바닥~천장)에 있고 부류가 다 나온다."""
    sdf = gen.build_sdf()[0]
    prims, _ = gen.gt_primitives(sdf, [str(MODELS)])
    pts, counts = gen.gt_surface_points(prims, spacing=0.25)
    xyz = cloud.xyz_of(pts)
    assert np.all(np.abs(xyz[:, 0]) <= gen.HALF_X + 1e-6)
    assert np.all(np.abs(xyz[:, 1]) <= gen.HALF_Y + 1e-6)
    assert np.all((xyz[:, 2] >= -1e-6) & (xyz[:, 2] <= gen.WALL_H + 1e-6))
    assert {c for c in gen.GT_CLASSES if counts.get(c)} == set(gen.GT_CLASSES) - {'vehicle',
                                                                                  'other'}
    # 바닥·천장은 방 넓이(60 x 40)를 넘지 않는다 (평면을 방으로 자른다)
    area = 0.25 * 0.25
    assert counts['ceiling'] * area == pytest.approx(60 * 40, rel=1e-6)
    assert counts['floor'] * area < 60 * 40


def _models(sdf_text):
    import xml.etree.ElementTree as ET
    world = ET.fromstring(sdf_text).find('world')
    names = {m.get('name') for m in world.findall('model')}
    names |= {i.findtext('name') for i in world.findall('include')}
    return world, names


S6_BLOCKED = ['s6-queue', 's6-queue2', 's6-wall']


def _actors(sdf_text):
    """SDF 의 actor 이름 → 웨이포인트 xy 배열 (생성기 specs 가 아니라 출력 SDF 에서 읽는다)."""
    world, _ = _models(sdf_text)
    out = {}
    for a in world.findall('actor'):
        xy = [[float(v) for v in w.findtext('pose').split()[:2]] for w in a.iter('waypoint')]
        out[a.get('name')] = np.array(xy)
    return out


def _blocker_aabbs(gen, sdf_text):
    """SDF 의 s6_blocker_* 시각체 → 월드 축 AABB (x0, x1, y0, y1)."""
    prims, _ = gen.gt_primitives(sdf_text, [str(MODELS)], gen.GT_EXCLUDE)
    out = []
    for p in prims:
        if p.model.startswith('s6_blocker'):
            h = np.abs(p.R) @ p.half()
            out.append((p.t[0] - h[0], p.t[0] + h[0], p.t[1] - h[1], p.t[1] + h[1]))
    return out


def _polyline_box_dist(xy, box, step=0.005):
    """꺾은선을 step 간격으로 촘촘히 표본해 AABB 까지 최소 거리 (생성기의 해석 거리와 독립인 확인)."""
    x0, x1, y0, y1 = box
    pts = [xy[:1]]
    for a, b in zip(xy[:-1], xy[1:]):
        n = max(int(np.ceil(np.hypot(*(b - a)) / step)), 1)
        pts.append(a + (b - a) * (np.arange(1, n + 1)[:, None] / n))
    p = np.vstack(pts)
    dx = np.maximum.reduce([x0 - p[:, 0], np.zeros(len(p)), p[:, 0] - x1])
    dy = np.maximum.reduce([y0 - p[:, 1], np.zeros(len(p)), p[:, 1] - y1])
    return float(np.hypot(dx, dy).min())


@pytest.mark.parametrize('variant', S6_BLOCKED)
def test_s6_variant_adds_only_blockers_matching_localizability(gen, variant):
    """S6 막힘 변형 = s6-open(대조군) + 정적 블로커만. 블로커 기하는 localizability.s6_blockers 와 같다."""
    from amr_evaluation.slam3d import localizability as L
    base = gen.build_sdf('s6-open')[0]
    sdf = gen.build_sdf(variant)[0]
    n = len(gen.S6_BLOCKERS[variant])
    same = base.replace('s6-open: 정적 블로커 0개', f'{variant}: 정적 블로커 {n}개')
    assert set(same.replace('s6-open', variant).splitlines()) <= set(sdf.splitlines())
    world, names = _models(sdf)
    _, base_names = _models(base)
    blockers = sorted(names - base_names)
    assert blockers == [f's6_blocker_{i}' for i in range(len(blockers))] and blockers
    assert base_names <= names
    prims, _ = gen.gt_primitives(sdf, [str(MODELS)], gen.GT_EXCLUDE)
    blk = [p for p in prims if p.model.startswith('s6_blocker')]
    # 월드 축 AABB (중심, 크기) — yaw ±π/2 AMR 은 x·y 크기가 바뀐다. SDF 자세는 소수 4 자리라 1e-4 허용
    got = sorted(np.r_[p.t, 2 * np.abs(p.R) @ p.half()].tolist() for p in blk)
    ref = L.s6_blockers(variant.split('-')[1])
    want = sorted(np.r_[c, 2 * np.abs(r) @ h].tolist() for c, r, h in zip(ref.c, ref.R, ref.h))
    assert np.allclose(got, want, atol=1e-4)


def test_s6_open_is_default_world_minus_dropped_actors(gen):
    """s6-open = 기본 월드에서 S6_DROP_ACTORS 의 actor 만 뺀 것 (모델·차량·정적 배치는 그대로)."""
    base = gen.build_sdf()[0]
    sdf, _, specs = gen.build_sdf('s6-open')
    assert _models(sdf)[1] == _models(base)[1]
    assert set(_actors(base)) - set(_actors(sdf)) == set(gen.S6_DROP_ACTORS)
    assert set(_actors(sdf)) == {sp[0] for sp in specs} and len(specs) == 4
    for name, xy in _actors(sdf).items():
        assert np.array_equal(xy, _actors(base)[name])
    assert set(gen.VEHICLES) <= _models(sdf)[1]


@pytest.mark.parametrize('variant', S6_BLOCKED)
def test_s6_blockers_clear_actor_paths_and_vehicle_lanes(gen, variant):
    """
    리뷰 B2: 블로커는 그 월드에 남은 모든 actor 경로에서 사람 반폭 + 0.1 m 이상, 차량 차선 밖.

    Fortress actor 는 정적 모델과 충돌하지 않으므로 경로가 블로커를 지나면 사람이 상자를 뚫고 지나간다.
    actor 경로는 출력 SDF 의 웨이포인트, 블로커는 출력 SDF 의 시각체에서 읽는다 (check_blockers 와 독립).
    """
    assert gen.BLOCKER_CLEARANCE >= 0.25 + 0.1
    sdf = gen.build_sdf(variant)[0]
    boxes = _blocker_aabbs(gen, sdf)
    assert len(boxes) == len(gen.S6_BLOCKERS[variant])
    for name, xy in _actors(sdf).items():
        for box in boxes:
            assert _polyline_box_dist(xy, box) >= gen.BLOCKER_CLEARANCE - 1e-3, (name, box)
    for vname, (axis, lo, hi, fixed, *_r) in gen.VEHICLES.items():
        assert vname in _models(sdf)[1]
        w = gen.VEHICLE_HALF_W[vname] + gen.BLOCKER_CLEARANCE
        back, front = gen.VEHICLE_REACH[vname]
        a0, a1 = lo - back - gen.BLOCKER_CLEARANCE, hi + front + gen.BLOCKER_CLEARANCE
        lane = (a0, a1, fixed - w, fixed + w) if axis == 'x' else (fixed - w, fixed + w, a0, a1)
        for x0, x1, y0, y1 in boxes:
            assert x1 <= lane[0] or x0 >= lane[1] or y1 <= lane[2] or y0 >= lane[3], (vname, lane)


def test_dropped_actors_do_cross_the_blockers(gen):
    """S6_DROP_ACTORS 가 빈 핑계가 아닌지: 기본 월드의 그 actor 경로는 실제로 블로커에 닿거나 너무 가깝다."""
    base_actors = _actors(gen.build_sdf()[0])
    queue2 = _blocker_aabbs(gen, gen.build_sdf('s6-queue2')[0])
    near = {n: min(_polyline_box_dist(xy, b) for b in queue2) for n, xy in base_actors.items()}
    assert near['worker_slow_south'] == pytest.approx(0.0, abs=1e-6)       # 남쪽 대기 AMR 을 관통
    assert near['worker_random'] == pytest.approx(0.20, abs=0.01)          # 북쪽 대기 AMR 가장자리 0.20 m
    assert {n for n, d in near.items() if d < gen.BLOCKER_CLEARANCE} == set(gen.S6_DROP_ACTORS)


def test_s1_variant_has_no_dynamic_obstacles(gen):
    sdf = gen.build_sdf('s1')[0]
    world, names = _models(sdf)
    _, base_names = _models(gen.build_sdf()[0])
    assert world.findall('actor') == []
    assert base_names - names == set(gen.VEHICLES)
    assert names <= base_names


def test_variant_requires_out_and_gt_ignores_variant(tmp_path, gen):
    with pytest.raises(SystemExit):
        gen.main(['--variant', 's1'])
    out = tmp_path / 'w.sdf'
    gen.main(['--variant', 's6-wall', str(out), '--gt-dir', str(tmp_path / 'gt'),
              '--gt-no-points', '--gt-res', '0.1'])
    assert 's6_blocker_1' in out.read_text()
    meta = json.loads((tmp_path / 'gt' / 'gt_meta.json').read_text())
    assert meta['matches_committed_sdf'] is True          # GT 는 언제나 기본 월드
    with pytest.raises(ValueError):
        gen.build_sdf('s9')


def test_blocker_overlapping_rack_is_rejected(gen):
    racks = gen.rack_instances()
    with pytest.raises(AssertionError, match='rack_narrow'):
        gen.check_blockers(racks, [('wall', 0.0, -7.9, 0.0)], [])  # 입구 안쪽으로 15 cm → 좁은 통로 랙과 겹침
    gen.check_blockers(racks, gen.S6_BLOCKERS['s6-wall'], [])      # 정의된 위치는 통과


def test_blocker_on_actor_path_or_vehicle_lane_is_rejected(gen):
    """리뷰 B2 회귀: 기본 월드 actor 전부를 두면 queue·queue2 블로커는 거절된다. 차선 위 블로커도 거절."""
    racks = gen.rack_instances()
    specs = {sp[0]: sp for sp in gen.actor_specs()}
    south = gen.S6_BLOCKERS['s6-queue2'][1:]
    with pytest.raises(AssertionError, match='worker_slow_south 경로에서 0.00 m'):
        gen.check_blockers(racks, south, [specs['worker_slow_south']])
    with pytest.raises(AssertionError, match='worker_random 경로에서 0.20 m'):
        gen.check_blockers(racks, gen.S6_BLOCKERS['s6-queue'], list(specs.values()))
    with pytest.raises(AssertionError, match='forklift_main 차선'):
        gen.check_blockers(racks, [('amr', 0.0, 15.0, 0.0)], [])
    with pytest.raises(AssertionError, match='shuttle_amr 차선'):
        gen.check_blockers(racks, [('amr', 9.0, 0.0, math.pi / 2)], [])


def test_blocker_actor_clearance_boundary(gen):
    """
    여유 경계: worker_crossing (y = -7 직선) 에서 블로커 윗변까지 c − 1 cm 는 거절, c + 1 cm 는 통과.

    AMR 블로커 yaw 0 → y 반폭 0.2. 랙(입구 y = -8)과는 겹치지 않는 위치다.
    """
    racks = gen.rack_instances()
    crossing = [sp for sp in gen.actor_specs() if sp[0] == 'worker_crossing']
    c = gen.BLOCKER_CLEARANCE
    with pytest.raises(AssertionError, match='worker_crossing'):
        gen.check_blockers(racks, [('amr', 0.0, -7.0 - 0.2 - (c - 0.01), 0.0)], crossing)
    gen.check_blockers(racks, [('amr', 0.0, -7.0 - 0.2 - (c + 0.01), 0.0)], crossing)


def test_blocker_vehicle_lane_clearance_boundary(gen):
    """셔틀 차선(x = 9 ± 차체 반폭 0.25)에서 블로커 옆면까지 c − 1 cm 는 거절, c + 1 cm 는 통과 (y = 0 통로)."""
    racks = gen.rack_instances()
    c = gen.BLOCKER_CLEARANCE
    edge = 9.0 + gen.VEHICLE_HALF_W['shuttle_amr']
    with pytest.raises(AssertionError, match='shuttle_amr 차선'):
        gen.check_blockers(racks, [('amr', edge + c - 0.01 + 0.2, 0.0, math.pi / 2)], [])
    gen.check_blockers(racks, [('amr', edge + c + 0.01 + 0.2, 0.0, math.pi / 2)], [])


@pytest.mark.parametrize('seg, box, want', [
    (((0, 0), (2, 0)), (0.5, 1.0, -0.2, 0.2), 0.0),            # 관통
    (((0, 1), (2, 1)), (0.5, 1.0, -0.2, 0.2), 0.8),            # 변과 평행
    (((-1, -1), (-1, 3)), (0.0, 1.0, 0.0, 1.0), 1.0),          # 옆을 지남
    (((2, 2), (3, 3)), (0.0, 1.0, 0.0, 1.0), math.sqrt(2)),    # 꼭짓점 쪽 끝점
    (((-1, 1.5), (1.5, -1)), (0.0, 1.0, 0.0, 1.0), 0.0),       # 모서리를 자름
    (((-1, 3.5), (3.5, -1)), (0.0, 1.0, 0.0, 1.0), 0.5 / math.sqrt(2)),   # 꼭짓점 (1,1) 대 선분
    (((3, 0.5), (3, 0.5)), (0.0, 1.0, 0.0, 1.0), 2.0),         # 길이 0 선분
    (((0.4, 0.4), (0.6, 0.6)), (0.0, 1.0, 0.0, 1.0), 0.0),     # 상자 안
    (((0.5, 0.5), (0.5, 0.5)), (0.0, 1.0, 0.0, 1.0), 0.0),     # 상자 안 점
])
def test_seg_box_dist_analytic(gen, seg, box, want):
    (ax, ay), (bx, by) = seg
    assert gen._seg_box_dist(ax, ay, bx, by, *box) == pytest.approx(want, abs=1e-12)
    assert gen._seg_box_dist(bx, by, ax, ay, *box) == pytest.approx(want, abs=1e-12)
