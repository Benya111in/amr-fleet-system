"""
측정 보조 순수 모듈 시험.

world_gt(시각체 정답 면, 점-면 거리), cloud_io(PointCloud2 해석, CDR 스탬프, 주기),
make_static_world(동적 행위자 제거), sensor_check 의 광선-면 교차.
"""

import importlib.util
import math
from pathlib import Path
import struct
from types import SimpleNamespace

from amr_lidar3d import cloud_io as cio
from amr_lidar3d import world_gt as wg
import numpy as np
import pytest

PKG = Path(__file__).resolve().parents[1]
SIM = PKG.parent / 'amr_simulation'


def _load_script(name):
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), PKG / 'scripts' / name)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _box(center=(0, 0, 0), half=(1, 1, 1), yaw=0.0):
    return wg.Box('b', np.array(center, float), wg.rpy_matrix(0, 0, yaw), np.array(half, float))


def test_rpy_matrix_matches_axis_rotations():
    r = wg.rpy_matrix(0.0, 0.0, math.pi / 2)
    assert r @ np.array([1.0, 0, 0]) == pytest.approx([0, 1, 0], abs=1e-12)
    r = wg.rpy_matrix(0.0, math.pi / 2, 0.0)
    assert r @ np.array([1.0, 0, 0]) == pytest.approx([0, 0, -1], abs=1e-12)
    r = wg.rpy_matrix(0.3, -0.2, 1.1)
    assert r @ r.T == pytest.approx(np.eye(3), abs=1e-12)


def test_box_distance_inside_outside_rotated():
    b = _box(half=(1.0, 0.5, 0.25))
    pts = np.array([[2.0, 0, 0], [0.9, 0, 0], [0, 0, 0], [2.0, 1.5, 0], [0, 0, 0.25]])
    d = wg.surface_distance(pts, [b], [])
    assert d == pytest.approx([1.0, 0.1, 0.25, math.sqrt(2.0), 0.0])
    rb = _box(center=(5, 0, 0), half=(1.0, 0.1, 1.0), yaw=math.pi / 2)   # 90° 돌린 얇은 판
    assert wg.surface_distance(np.array([[5.0, 0.9, 0.0], [5.3, 0.0, 0.0]]), [rb], []) == \
        pytest.approx([0.1, 0.2])


def test_plane_distance_finite_and_downward_normal():
    floor = wg.Plane('floor', np.zeros(3), np.eye(3), np.array([10.0, 10.0]))
    ceiling = wg.Plane('ceil', np.array([0, 0, 8.0]), wg._frame_with_z(np.array([0, 0, -1.0])),
                       np.array([10.0, 10.0]))
    pts = np.array([[1.0, 1.0, 0.02], [12.0, 0.0, 0.0], [0.0, 0.0, 7.5]])
    d = wg.surface_distance(pts, [], [floor, ceiling])
    assert d == pytest.approx([0.02, 2.0, 0.5])


def test_compose_pose():
    a = (wg.rpy_matrix(0, 0, math.pi / 2), np.array([1.0, 0, 0]))
    b = (np.eye(3), np.array([1.0, 0, 0]))
    r, t = wg.compose(a, b)
    assert t == pytest.approx([1.0, 1.0, 0.0])


@pytest.mark.skipif(not (SIM / 'worlds' / 'warehouse.sdf').is_file(), reason='월드 없음')
def test_load_world_rack_visuals():
    boxes, planes, skipped = wg.load_world(str(SIM / 'worlds' / 'warehouse.sdf'),
                                           str(SIM / 'models'),
                                           exclude=('forklift', 'shuttle_amr'))
    names = {b.name for b in boxes}
    assert any(n.startswith('rack_A1/post_') for n in names)
    assert any(n.endswith('/deck') for n in names)
    assert not any(n.startswith('forklift') for n in names)
    assert {p.name for p in planes} >= {'ground_plane/visual', 'roof/ceiling'}
    # 랙 A1 의 기둥 위 점은 거리 0, 기둥 사이 빈 곳(데크 위 1 m)은 0 이 아니다 (충돌체 통짜 박스가 아니다)
    post = next(b for b in boxes if b.name == 'rack_A1/post_0')
    assert wg.surface_distance(post.center[None, :] + [0, 0, 0], [post], []) == \
        pytest.approx([0.04])
    deck = next(b for b in boxes if b.name == 'rack_A1/deck')
    above = deck.center + np.array([0.0, 0.0, 0.25 + 0.5])
    assert wg.surface_distance(above[None, :], boxes, planes)[0] > 0.2


def test_summarize():
    s = wg.summarize(np.array([0.0, 1.0, 2.0, np.inf]))
    assert s['n'] == 3 and s['median'] == 1.0 and s['max'] == 2.0
    assert wg.summarize(np.array([np.inf]))['n'] == 0


def _cloud(points, ring, pad=0):
    """Gazebo 배치(32 B, ring 24) 의 가짜 PointCloud2."""
    fields = [SimpleNamespace(name=n, offset=o, datatype=t, count=1) for n, o, t in
              (('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('intensity', 16, 7), ('ring', 24, 4))]
    data = bytearray()
    for (x, y, z), r in zip(points, ring):
        rec = bytearray(32)
        struct.pack_into('<fff', rec, 0, x, y, z)
        struct.pack_into('<f', rec, 16, 1.0)
        struct.pack_into('<H', rec, 24, r)
        data += rec
    data += bytes(pad)
    return SimpleNamespace(fields=fields, point_step=32, row_step=32 * len(points) + pad,
                           width=len(points), height=1, data=bytes(data), is_bigendian=False)


def test_cloud_array_and_fields():
    msg = _cloud([(1.0, 2.0, 3.0), (math.inf, 0.0, 0.0)], [3, 15], pad=8)
    arr = cio.cloud_array(msg)
    assert arr['ring'].tolist() == [3, 15]
    assert cio.xyz(arr)[0] == pytest.approx([1.0, 2.0, 3.0])
    assert not np.isfinite(cio.xyz(arr)[1, 0])
    f = cio.cloud_fields(msg)
    assert [x['name'] for x in f] == ['x', 'y', 'z', 'intensity', 'ring']
    assert f[4]['datatype'] == 'UINT16' and f[4]['offset'] == 24


def test_cdr_stamp_and_errors():
    raw = bytes([0, 1, 0, 0]) + struct.pack('<iI', 12, 500000000) + b'frame'
    assert cio.cdr_stamp(raw) == pytest.approx(12.5)
    with pytest.raises(ValueError):
        cio.cdr_stamp(bytes([0, 0, 0, 0]) + raw[4:])


def test_interval_stats_and_quat():
    s = cio.interval_stats([0.0, 0.1, 0.2, 0.3, 0.5])
    assert s['n'] == 5 and s['hz'] == pytest.approx(8.0) and s['dt_max'] == pytest.approx(0.2)
    assert cio.interval_stats([1.0]) == {'n': 1}
    yaw = 0.7
    q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    assert cio.quat_yaw(*q) == pytest.approx(yaw)
    assert cio.quat_matrix(*q) == pytest.approx(wg.rpy_matrix(0, 0, yaw))
    assert cio.wrap(np.array([3.5 * math.pi, 0.25]))[0] == pytest.approx(-0.5 * math.pi)


def test_ray_hits_box_and_plane():
    check = _load_script('lidar3d_sensor_check.py')
    box = _box(center=(5.0, 0, 1.0), half=(0.5, 0.5, 1.0))
    floor = wg.Plane('floor', np.zeros(3), np.eye(3), np.array([50.0, 50.0]))
    origin = np.array([0.0, 0.0, 0.9])
    dirs = np.array([[1.0, 0, 0], [-1.0, 0, 0], [math.cos(-0.2), 0, math.sin(-0.2)],
                     [0, 0, 1.0]])
    t = check.ray_box_hits(origin, dirs, [box], [floor])
    assert t[0] == pytest.approx(4.5)
    assert math.isinf(t[1])
    assert t[2] == pytest.approx(0.9 / math.sin(0.2))           # 바닥
    assert math.isinf(t[3])


def test_make_static_world(tmp_path):
    mod = _load_script('make_static_world.py')
    src = tmp_path / 'warehouse.sdf'
    src.write_text("""<?xml version="1.0"?><sdf version="1.8"><world name="warehouse">
      <include><uri>model://rack</uri><name>rack_A1</name></include>
      <include><uri>model://forklift</uri><name>forklift_main</name></include>
      <actor name="worker_a"><skin><filename>x</filename></skin></actor>
      <model name="walls"/></world></sdf>""", encoding='utf-8')
    out = tmp_path / 'out'
    assert mod.main([str(src), str(out)]) == 0
    text = (out / 'warehouse.sdf').read_text(encoding='utf-8')
    assert 'rack_A1' in text and 'walls' in text
    assert 'forklift' not in text and '<actor' not in text
