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
