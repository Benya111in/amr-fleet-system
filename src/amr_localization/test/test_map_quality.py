"""월드 SDF 래스터화와 맵 품질 지표 테스트 (합성 월드)."""

import json
import math
from pathlib import Path

from amr_localization import map_quality as mq
from amr_localization import world_geometry as wg
import numpy as np
import pytest
import yaml

MODEL_SDF = """<?xml version="1.0"?>
<sdf version="1.8"><model name="post"><static>true</static>
  <link name="link"><pose>0 0 0 0 0 0</pose>
    <visual name="body"><pose>0 0 1.0 0 0 0</pose>
      <geometry><box><size>0.4 0.4 2.0</size></box></geometry></visual>
    <visual name="top"><pose>0 0 2.5 0 0 0</pose>
      <geometry><sphere><radius>0.3</radius></sphere></geometry></visual>
    <visual name="mesh"><geometry><mesh><uri>x.dae</uri></mesh></geometry></visual>
  </link></model></sdf>
"""

WORLD_SDF = """<?xml version="1.0"?>
<sdf version="1.8"><world name="test">
  <model name="ground_plane"><link name="l"><visual name="v">
    <geometry><box><size>100 100 0.1</size></box></geometry></visual></link></model>
  <model name="walls"><static>true</static><link name="link">
    <visual name="north"><pose>0 4.9 1 0 0 0</pose>
      <geometry><box><size>12 0.2 2</size></box></geometry></visual>
    <visual name="south"><pose>0 -4.9 1 0 0 0</pose>
      <geometry><box><size>12 0.2 2</size></box></geometry></visual>
    <visual name="east"><pose>5.9 0 1 0 0 0</pose>
      <geometry><box><size>0.2 10 2</size></box></geometry></visual>
    <visual name="west"><pose>-5.9 0 1 0 0 0</pose>
      <geometry><box><size>0.2 10 2</size></box></geometry></visual>
    <visual name="low_curb"><pose>0 0 0.05 0 0 0</pose>
      <geometry><box><size>1 1 0.1</size></box></geometry></visual>
  </link>
  <model name="nested"><pose>-3 2 0 0 0 0</pose><link name="l">
    <visual name="c"><pose>0 0 0.5 0 0 0</pose>
      <geometry><cylinder><radius>0.25</radius><length>1.0</length></cylinder></geometry>
    </visual></link></model>
  </model>
  <include><uri>model://post</uri><name>post_1</name><pose>2 -2 0 0 0 0.785398</pose></include>
  <include><uri>model://post</uri><name>post_2</name><pose>3 2 0 0 0 0</pose></include>
  <include><uri>model://missing</uri><name>ghost</name></include>
  <include><uri>model://post</uri><name>forklift_main</name><pose>0 0 0 0 0 0</pose></include>
</world></sdf>
"""

ROOM = (-5.8, 5.8, -4.8, 4.8)


@pytest.fixture(scope='module')
def world(tmp_path_factory):
    root = tmp_path_factory.mktemp('world')
    (root / 'models' / 'post').mkdir(parents=True)
    (root / 'models' / 'post' / 'model.sdf').write_text(MODEL_SDF)
    (root / 'models' / 'broken').mkdir()
    (root / 'models' / 'broken' / 'model.sdf').write_text('<sdf version="1.8"></sdf>')
    path = root / 'test.sdf'
    path.write_text(WORLD_SDF.replace(
        '<include><uri>model://missing</uri>',
        '<include><uri>model://broken</uri><name>b</name></include><include>'
        '<uri>model://missing</uri>'))
    return root, path


def load_raster(world, res=0.05, height=0.20):
    root, path = world
    loader = wg.WorldLoader([root / 'models'])
    shapes = loader.load_world(path)
    raster = wg.rasterize(shapes, height, res, (-6.5, 6.5, -5.5, 5.5))
    return loader, raster


def cell(raster, x, y):
    c = int((x - raster.origin_x) / raster.resolution)
    r = int((y - raster.origin_y) / raster.resolution)
    return raster.occupied[r, c]


def test_pose_matrix_and_rpy():
    t = wg.pose_matrix('1 2 3 0 0 1.5707963')
    assert t[:3, 3] == pytest.approx([1, 2, 3])
    assert t[:3, :3] @ np.array([1, 0, 0]) == pytest.approx([0, 1, 0], abs=1e-6)
    assert np.allclose(wg.pose_matrix(None), np.eye(4))
    assert np.allclose(wg.pose_matrix('1 2'), wg.pose_matrix('1 2 0 0 0 0'))
    r = wg.rpy_matrix(0.3, -0.2, 1.0)
    assert r @ r.T == pytest.approx(np.eye(3))


def test_world_loader_shapes_and_skips(world):
    loader, raster = load_raster(world)
    names = [s.name for s in loader.shapes]
    assert not any(n.startswith('ground_plane') or 'forklift' in n for n in names)
    assert any('nested' in n for n in names)
    assert any('mesh' in s and 'unsupported' in s for s in loader.skipped)
    assert any('ghost' in s for s in loader.skipped)
    assert any(s.startswith('b:') for s in loader.skipped)
    # 단면: 벽, 45° 회전 기둥, 중첩 모델 원기둥은 점유 / 낮은 턱(0.1 m)·공중 구(2.5 m)는 비점유
    assert cell(raster, 0.0, 4.9) and cell(raster, 5.9, 0.0)
    assert cell(raster, 2.0, -2.0) and cell(raster, 2.0, -2.0 + 0.2)    # 회전 상자 대각 0.28
    assert not cell(raster, 2.0 + 0.25, -2.0 + 0.25)                    # 회전 상자 모서리 밖
    assert cell(raster, -3.0, 2.0) and not cell(raster, -3.0 + 0.3, 2.0)
    assert not cell(raster, 0.0, 0.0)
    assert not cell(raster, 3.0 + 0.35, 2.0 + 0.35)


def test_world_loader_errors(tmp_path):
    bad = tmp_path / 'bad.sdf'
    bad.write_text('<sdf version="1.8"></sdf>')
    with pytest.raises(ValueError):
        wg.WorldLoader([]).load_world(bad)


def test_write_and_load_map_roundtrip(world, tmp_path):
    _, raster = load_raster(world)
    pgm, yml = wg.write_map(raster, tmp_path / 'maps' / 'gt')
    est = mq.load_map(yml)
    assert est.resolution == pytest.approx(0.05)
    assert np.array_equal(est.occupied, raster.occupied)
    assert np.array_equal(est.free, ~raster.occupied)
    pts = est.cell_centers(est.occupied)
    assert len(pts) == int(raster.occupied.sum())


def test_scan_plane_height_from_config(tmp_path):
    # 단일 출처: robot_params base_link_height + sensors lidar.extrinsic.z (차체 안 슬롯 LiDAR 0.20 m)
    (tmp_path / 'robot_params.yaml').write_text(
        '/**:\n  ros__parameters:\n    robot: {base_link_height: 0.18}\n')
    (tmp_path / 'sensors.yaml').write_text(
        '/**:\n  ros__parameters:\n    lidar: {extrinsic: {x: 0.15, z: 0.02}}\n')
    assert wg.scan_plane_height(str(tmp_path)) == pytest.approx(0.20)
    assert wg.scan_plane_height(str(tmp_path / 'missing')) == wg.DEFAULT_SCAN_HEIGHT
    ws_config = Path(__file__).resolve().parents[3] / 'config'
    if (ws_config / 'sensors.yaml').is_file():
        assert wg.scan_plane_height(str(ws_config)) == pytest.approx(0.20)


def test_trinary_unknown_threshold(tmp_path):
    # map_saver 의 미지 픽셀 205 는 p = 0.196: free_thresh 0.25(nav2 기본 YAML)면 자유로 읽힌다
    assert mq.unknown_loads_as_free({'free_thresh': 0.25})
    assert not mq.unknown_loads_as_free({'free_thresh': mq.FREE_THRESH})
    img = np.array([[0, 205, 254]], dtype=np.uint8)
    (tmp_path / 'm.pgm').write_bytes(b'P5\n3 1\n255\n' + img.tobytes())
    for thresh, unknown in ((0.25, 0), (0.19, 1)):
        (tmp_path / 'm.yaml').write_text(f'image: m.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n'
                                         f'negate: 0\noccupied_thresh: 0.65\n'
                                         f'free_thresh: {thresh}\n')
        m = mq.load_map(tmp_path / 'm.yaml')
        assert int((~m.occupied & ~m.free).sum()) == unknown
        assert m.occupied[0, 0] and m.free[0, 2]


def band_points(face: float, thickness: int, res: float = 0.05) -> np.ndarray:
    """면 x = face 인 벽의 점유 띠 셀 중심 (면을 담은 셀 중심으로 ±(thickness−1)/2 셀, y ∈ [−5, 5])."""
    c0 = math.floor(face / res)
    half = (thickness - 1) // 2
    ys = (np.arange(int(10.0 / res)) + 0.5) * res - 5.0
    return np.array([[(c + 0.5) * res, y] for c in range(c0 - half, c0 + half + 1) for y in ys])


def old_innermost_face(pts: np.ndarray, face: float, inward: float, res: float) -> float:
    """이전 추정: 구간별 가장 안쪽 점유 셀 중심에서 반 셀 더 안쪽 (평균)."""
    sel = pts[np.abs(pts[:, 0] - face) <= 0.3]
    inner = sel[:, 0].max() if inward > 0 else sel[:, 0].min()
    return inner + inward * res / 2


def test_wall_face_estimator_is_stable_under_band_thickness():
    # 잡음이 띠를 두껍게 만들어도 면 추정(구간별 띠 중앙값)이 움직이지 않는다. 이전 추정(가장 안쪽 셀 +
    # 반 셀)은 두께 1 → 5 셀에서 면을 방 안쪽으로 2 셀 옮겨 간격이 작아 보였다 (리뷰: −9.6 / −14.4 cm)
    res = 0.05
    for face in (5.0, 4.987, 5.021):
        faces, gaps, old = [], [], []
        for thickness in (1, 3, 5):
            west, east = band_points(-face, thickness), band_points(face, thickness)
            walls, known = mq.wall_metrics(np.concatenate([west, east]), (-face, face, -5.0, 5.0),
                                           0.3, res)
            by = {w.name: w for w in walls}
            faces.append((by['west'].offset_error, by['east'].offset_error))
            gaps.append(known['width_x']['error'])
            old.append(old_innermost_face(east, face, -1.0, res)
                       - old_innermost_face(west, -face, 1.0, res) - 2 * face)
            assert by['west'].rms_residual < 1e-9 and by['west'].angle_error_deg < 1e-6
        # 면 오차 ≤ 반 셀 (면을 담은 셀 중심 규약), 두께와 무관
        assert all(abs(w) <= res / 2 + 1e-9 and abs(e) <= res / 2 + 1e-9 for w, e in faces)
        assert max(gaps) - min(gaps) < 1e-9
        # 이전 추정은 두께가 늘 때마다 간격이 2 셀(10 cm)씩 줄었다
        assert old[0] - old[2] == pytest.approx(4 * res)


def test_register_map_translation_and_rotation(tmp_path):
    # 등록: yaw 0 은 픽셀 그대로 원점만 이동, 회전은 재표본화(trinary 값 보존, 밖은 미지)
    img = np.full((40, 60), 254, dtype=np.uint8)
    img[5, :] = 0                      # 행 5 (위에서) = 벽
    img[:, 50:] = 205
    (tmp_path / 's.pgm').write_bytes(b'P5\n60 40\n255\n' + img.tobytes())
    (tmp_path / 's.yaml').write_text('image: s.pgm\nmode: trinary\nresolution: 0.05\n'
                                     'origin: [-1.0, -1.0, 0.0]\nnegate: 0\n'
                                     'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    pgm, yml = mq.register_map(tmp_path / 's.yaml', (0.3, -0.2, 0.0), tmp_path / 'out' / 't')
    meta = yaml.safe_load(yml.read_text())
    assert meta['origin'] == pytest.approx([-0.7, -1.2, 0.0])
    assert meta['free_thresh'] == pytest.approx(mq.FREE_THRESH)
    assert np.array_equal(mq.read_pgm(pgm), img)
    src = mq.load_map(tmp_path / 's.yaml')
    # 회전: 등록 맵의 점유 셀 = 원본 점유 셀을 같은 SE(2) 로 옮긴 위치 (셀 대각 이내)
    pose = (0.3, -0.2, math.radians(10.0))
    _, yml = mq.register_map(tmp_path / 's.yaml', pose, tmp_path / 'out' / 'r')
    reg = mq.load_map(yml)
    moved = mq.se2_apply(src.cell_centers(src.occupied), *pose)
    got = reg.cell_centers(reg.occupied)
    d = np.min(np.linalg.norm(got[:, None, :] - moved[None, :, :], axis=2), axis=1)
    assert np.max(d) <= 0.05 * math.sqrt(2) + 1e-9
    assert abs(len(got) - len(moved)) <= 0.1 * len(moved)
    assert set(np.unique(mq.read_pgm(yml.with_suffix('.pgm'))).tolist()) <= {0, 205, 254}
    # 반 셀 변위도 못 만드는 작은 회전(0.01° < min_resample_rotation)은 재표본화하지 않는다: 픽셀 그대로
    assert mq.min_resample_rotation(img.shape, 0.05) > math.radians(0.01)
    pgm, yml = mq.register_map(tmp_path / 's.yaml', (0.1, 0.0, math.radians(0.01)),
                               tmp_path / 'out' / 'small')
    assert np.array_equal(mq.read_pgm(pgm), img)
    assert yaml.safe_load(yml.read_text())['origin'] == pytest.approx([-0.9, -1.0, 0.0])


def test_read_pgm_ascii_and_negate(tmp_path):
    (tmp_path / 'a.pgm').write_text('P2\n# comment\n3 1\n255\n0 128 255\n')
    img = mq.read_pgm(tmp_path / 'a.pgm')
    assert img.tolist() == [[0, 128, 255]]
    (tmp_path / 'a.yaml').write_text('image: a.pgm\nresolution: 0.1\norigin: [1, 2, 0]\n'
                                     'negate: 1\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    m = mq.load_map(tmp_path / 'a.yaml')
    assert m.occupied[0].tolist() == [False, False, True]   # negate: 흰색이 점유
    assert m.free[0].tolist() == [True, False, False]
    (tmp_path / 'b.pgm').write_bytes(b'P6\n1 1\n255\n\x00\x00\x00')
    with pytest.raises(ValueError):
        mq.read_pgm(tmp_path / 'b.pgm')


def test_evaluate_recovers_offset_and_perfect_scores(world):
    _, gt = load_raster(world)
    # 추정 맵 = GT 를 (0.30, −0.20, 2°) 만큼 옮겨 그린 것 → map→world 정렬이 그 역변환을 찾아야 한다
    shift = (0.30, -0.20, math.radians(2.0))
    pts = np.stack(np.nonzero(gt.occupied)[::-1], 1).astype(float)
    world_pts = np.stack([gt.origin_x + (pts[:, 0] + 0.5) * gt.resolution,
                          gt.origin_y + (pts[:, 1] + 0.5) * gt.resolution], 1)
    c, s = math.cos(-shift[2]), math.sin(-shift[2])
    local = world_pts - np.array(shift[:2])
    map_pts = np.stack([c * local[:, 0] - s * local[:, 1], s * local[:, 0] + c * local[:, 1]], 1)
    res = gt.resolution
    origin = (-7.0, -6.0)
    occ = np.zeros((240, 280), dtype=bool)
    col = ((map_pts[:, 0] - origin[0]) / res).astype(int)
    row = ((map_pts[:, 1] - origin[1]) / res).astype(int)
    occ[row, col] = True
    free = ~occ
    est = mq.MapImage(occ, free, res, origin[0], origin[1])
    q = mq.evaluate(est, gt, (0.0, 0.0, 0.0), ROOM)
    assert q.adnn_initial > 0.1           # 정렬 전에는 옮긴 만큼 어긋난다
    assert q.alignment[0] == pytest.approx(shift[0], abs=0.03)
    assert q.alignment[1] == pytest.approx(shift[1], abs=0.03)
    assert q.alignment[2] == pytest.approx(shift[2], abs=math.radians(0.3))
    assert q.adnn < 0.03
    assert q.iou_tol > 0.85 and q.precision_tol > 0.9 and q.recall_tol > 0.9
    assert {w.name for w in q.walls} == {'north', 'south', 'east', 'west'}
    assert all(w.rms_residual < 0.03 and w.angle_error_deg < 0.5 for w in q.walls)
    # 면 위치 = 띠 점유 셀 중앙값: GT 래스터처럼 벽 0.2 m 가 꽉 찬 격자는 벽 중앙면(면 뒤 0.1 m)이 되어
    # 간격이 벽 두께만큼 커 보인다 (SLAM 격자는 관측 면 둘레 2.8~3.2 셀 띠 — slam.md §3)
    assert q.known_distance['width_x']['error'] == pytest.approx(0.2, abs=0.03)
    assert q.pillar_distance == {}                        # 합성 월드에 기본 기둥 좌표 없음
    assert all(1.0 <= w.thickness_cells <= 6.0 for w in q.walls)
    assert q.landmark_count >= 2 and q.landmark_error_max < 0.05
    md = mq.to_markdown(q, 'synthetic')
    assert 'ADNN' in md and 'width_x' in md and '| north |' in md


def test_pillar_distances_cancel_band_convention():
    # 기둥 네 면을 모두 면 뒤로 1 셀 치우친 띠로 그려도 중심 간 거리는 정확하다 (규약 무관)
    res = 0.05
    pts = []
    for cx, cy in ((-10.0, 12.0), (10.0, 12.0), (10.0, -12.0)):
        for k in np.arange(-5, 6):
            for d in (0.26 - 0.025, 0.26 - 0.075):        # 면 안쪽(뒤) 두 셀
                pts += [(cx + d, cy + k * res), (cx - d, cy + k * res),
                        (cx + k * res, cy + d), (cx + k * res, cy - d)]
    pd = mq.pillar_distances(np.array(pts), mq.DEFAULT_PILLARS)
    assert set(pd) == {'(-10,12)-(10,12)', '(10,-12)-(10,12)'}
    assert all(abs(v['error']) < 1e-9 for v in pd.values())
    assert pd['(10,-12)-(10,12)']['gt'] == pytest.approx(24.0)


def test_evaluate_rejects_empty_map(world):
    _, gt = load_raster(world)
    est = mq.MapImage(np.zeros((10, 10), bool), np.ones((10, 10), bool), 0.05, 0.0, 0.0)
    with pytest.raises(ValueError):
        mq.evaluate(est, gt, (0.0, 0.0, 0.0), ROOM)


def test_cli_end_to_end(world, tmp_path, capsys):
    root, path = world
    stem = tmp_path / 'gt'
    assert wg.main(['--world', str(path), '--models', str(root / 'models'), '--out', str(stem),
                    '--bounds', '-7', '7', '-6', '6']) == 0
    assert 'occupied cells' in capsys.readouterr().out
    out = tmp_path / 'report'
    # nav2 map_saver 기본 YAML(free_thresh 0.25)이면 경고, 등록 출력은 free_thresh 0.19
    yml = stem.with_suffix('.yaml')
    yml.write_text(yml.read_text().replace('free_thresh: 0.19', 'free_thresh: 0.25'))
    assert mq.main(['--map', str(yml), '--world', str(path), '--height', '0.20',
                    '--models', str(root / 'models'), '--room', *map(str, ROOM),
                    '--out', str(out), '--register-out', str(tmp_path / 'reg' / 'm')]) == 0
    text = capsys.readouterr().out
    assert 'WARNING: free_thresh 0.25' in text and 'registered map' in text
    data = json.loads((out / 'map_quality.json').read_text())
    assert data['adnn'] < 0.01 and data['adnn_initial'] < 0.01
    assert Path(out / 'map_quality.md').exists()
    reg = yaml.safe_load((tmp_path / 'reg' / 'm.yaml').read_text())
    assert reg['free_thresh'] == pytest.approx(mq.FREE_THRESH)
