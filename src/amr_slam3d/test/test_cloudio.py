"""점 지도 파일 읽기·쓰기·변환 시험 (PLY binary, PCD ascii/binary, 쿼터니언 변환)."""

import math

from amr_slam3d import cloudio
import numpy as np
import pytest


def test_ply_roundtrip(tmp_path):
    pts = np.array([[1.0, 2.0, 3.0], [-4.5, 0.25, 0.9]])
    p = tmp_path / 'a.ply'
    cloudio.write_ply_xyz(str(p), pts)
    np.testing.assert_allclose(cloudio.read_ply_xyz(str(p)), pts, atol=1e-6)


def test_ply_with_extra_properties_and_trailing_element(tmp_path):
    """PCL 이 쓰는 형태: vertex 에 normal 등 다른 속성, 뒤에 camera 원소."""
    dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('normal_x', '<f4'), ('i', '<u1')])
    arr = np.zeros(3, dtype=dt)
    arr['x'], arr['y'], arr['z'] = [1, 2, 3], [4, 5, 6], [7, 8, 9]
    head = ['ply', 'format binary_little_endian 1.0', 'element vertex 3', 'property float x',
            'property float y', 'property float z', 'property float normal_x',
            'property uchar i', 'element camera 1', 'property float view_px', 'end_header']
    p = tmp_path / 'b.ply'
    p.write_bytes(('\n'.join(head) + '\n').encode() + arr.tobytes() + b'\x00' * 4)
    got = cloudio.read_ply_xyz(str(p))
    np.testing.assert_allclose(got, [[1, 4, 7], [2, 5, 8], [3, 6, 9]])


def test_pcd_binary_and_ascii(tmp_path):
    dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4')])
    arr = np.zeros(2, dtype=dt)
    arr['x'], arr['y'], arr['z'], arr['intensity'] = [1, 2], [3, 4], [5, 6], [7, 8]
    head = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n'
            'COUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 2\nDATA binary\n')
    p = tmp_path / 'c.pcd'
    p.write_bytes(head.encode() + arr.tobytes())
    np.testing.assert_allclose(cloudio.read_pcd_xyz(str(p)), [[1, 3, 5], [2, 4, 6]])
    q = tmp_path / 'd.pcd'
    q.write_text(head.replace('DATA binary', 'DATA ascii') + '1 3 5 7\n2 4 6 8\n')
    np.testing.assert_allclose(cloudio.read_pcd_xyz(str(q)), [[1, 3, 5], [2, 4, 6]])


def test_transform_yaw_90_and_translation():
    q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    out = cloudio.transform(np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]), (18.0, -16.0, 0.9), q)
    np.testing.assert_allclose(out, [[18.0, -15.0, 0.9], [18.0, -16.0, 1.9]], atol=1e-12)


def test_quat_matrix_is_rotation():
    r = cloudio.quat_matrix((0.1, -0.2, 0.3, 0.9))
    np.testing.assert_allclose(r @ r.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(r) == pytest.approx(1.0)


def test_finite_drops_inf_nan():
    pts = np.array([[1, 2, 3], [np.inf, 0, 0], [0, np.nan, 0]], dtype=float)
    assert cloudio.finite(pts).shape == (1, 3)
