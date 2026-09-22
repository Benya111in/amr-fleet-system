"""월드 SDF 래스터화와 맵 품질 지표 테스트 (합성 월드)."""

import json
import math
from pathlib import Path

from amr_localization import map_quality as mq
from amr_localization import world_geometry as wg
import numpy as np
import pytest

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


def load_raster(world, res=0.05, height=0.38):
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
    assert q.alignment[0] == pytest.approx(shift[0], abs=0.03)
    assert q.alignment[1] == pytest.approx(shift[1], abs=0.03)
    assert q.alignment[2] == pytest.approx(shift[2], abs=math.radians(0.3))
    assert q.adnn < 0.03
    assert q.iou_tol > 0.85 and q.precision_tol > 0.9 and q.recall_tol > 0.9
    assert {w.name for w in q.walls} == {'north', 'south', 'east', 'west'}
    assert all(w.rms_residual < 0.03 and w.angle_error_deg < 0.5 for w in q.walls)
    assert q.known_distance['width_x']['error'] == pytest.approx(0.0, abs=0.03)
    assert all(1.0 <= w.thickness_cells <= 6.0 for w in q.walls)
    assert q.landmark_count >= 2 and q.landmark_error_max < 0.05
    md = mq.to_markdown(q, 'synthetic')
    assert 'ADNN' in md and 'width_x' in md and '| north |' in md


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
    assert mq.main(['--map', str(stem.with_suffix('.yaml')), '--world', str(path),
                    '--models', str(root / 'models'), '--room', *map(str, ROOM),
                    '--out', str(out)]) == 0
    data = json.loads((out / 'map_quality.json').read_text())
    assert data['adnn'] < 0.01
    assert Path(out / 'map_quality.md').exists()
