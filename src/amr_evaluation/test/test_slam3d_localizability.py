"""
GT 기하 localizability (amr_evaluation.slam3d.localizability) 시험 — 광선 교차·정보 행렬의 해석값.

원본 연구 스크립트(localizability2.py) 수치의 재현은 저장소 밖 파일이 필요해 여기서 돌리지 않는다
(REPORT·evaluation.md §6 에 [계산] 으로 기록: 보고서 표 14 칸 전부 일치).
"""

import math

from amr_evaluation.slam3d import localizability as L
import numpy as np
import pytest


def test_cast_axis_aligned_and_rotated_box():
    b = L.box(5.0, 0.0, 0.0, 2.0, 2.0, 2.0)
    d = np.array([[1.0, 0, 0], [0, 1.0, 0]])
    t, n, k = L.cast(np.zeros(3), d, b)
    assert t[0] == pytest.approx(4.0) and n[0] == pytest.approx([-1, 0, 0]) and k[0] == 0
    assert math.isinf(t[1]) and k[1] == -1
    # 45° 돌린 정육면체: 모서리가 원점을 향한다 → 거리 5 − √2, 법선은 두 면 중 하나 (±45°)
    r = L.box(5.0, 0.0, 0.0, 2.0, 2.0, 2.0, math.pi / 4)
    t, n, _ = L.cast(np.zeros(3), d[:1], r)
    assert t[0] == pytest.approx(5.0 - math.sqrt(2.0))
    assert abs(n[0, 0]) == pytest.approx(math.sqrt(0.5)) and n[0, 0] < 0


def test_origin_inside_box_is_ignored():
    """센서가 안에 있는 상자(차체 슬롯)는 맞지 않는다 — 원본과 같은 규칙."""
    b = L.box(0.0, 0.0, 0.0, 1.0, 1.0, 1.0).concat(L.box(3.0, 0.0, 0.0, 1.0, 1.0, 1.0))
    t, _, k = L.cast(np.zeros(3), np.array([[1.0, 0, 0]]), b)
    assert k[0] == 1 and t[0] == pytest.approx(2.5)


def _corridor(open_ends=True):
    """통로 (x 방향): 벽 y = ±0.3 (길이 4 m, x −2..2). open_ends=False 면 x = ±3 에 끝벽."""
    b = L.box(0.0, 0.8, 0.5, 4.0, 1.0, 1.0, 0.0, 'rack').concat(
        L.box(0.0, -0.8, 0.5, 4.0, 1.0, 1.0, 0.0, 'rack'))
    if not open_ends:
        b = b.concat(L.box(3.5, 0.0, 0.5, 1.0, 3.0, 1.0, 0.0, 'wall')).concat(
            L.box(-3.5, 0.0, 0.5, 1.0, 3.0, 1.0, 0.0, 'wall'))
    return b


SENSOR_2D = L.Sensor('2d', (0.0,), 720, 0.20, 0.10, 25.0)


def test_open_corridor_is_degenerate_along_axis():
    """양끝이 열린 통로 (끝 너머 아무것도 없음): 통로 축 구속 점 0, λmin/λmax ≈ 0."""
    r = L.analyse(SENSOR_2D, (0.0, 0.0), 0.0, _corridor(True))
    assert r['along_points'] == 0 and r['lam_ratio_xy'] < 1e-9
    assert r['sigma_along_se2'] > 1.0          # 특이 정보 행렬 → 축 방향 표준편차 무한(또는 수치상 거대)


def test_end_walls_constrain_axis_and_blocker_as_outlier_removes_it():
    """끝벽이 있으면 축이 구속된다. 끝벽을 지도에 없는 블로커로 치면(정보에서 뺌) 다시 퇴화한다."""
    closed = _corridor(False)
    r = L.analyse(SENSOR_2D, (0.0, 0.0), 0.0, closed)
    assert r['along_points'] > 0 and r['lam_ratio_xy'] > 1e-3
    assert set(r['along_sources']) == {'wall'}
    r2 = L.analyse(SENSOR_2D, (0.0, 0.0), 0.0, closed, ignore_labels=('wall',))
    assert r2['along_points'] == 0 and r2['lam_ratio_xy'] < 1e-9
    assert r2['points'] < r['points']


def test_robot_body_occludes_downward_rays():
    """마스트 3D 센서의 아래쪽 빔은 자기 차체에 맞아 버려진다 (자기 몸 점은 정보에 없다)."""
    s = L.Sensor('down', (-80.0,), 36, 0.9, 0.1, 30.0)
    floor = L.box(0.0, 0.0, -0.05, 20.0, 20.0, 0.1, 0.0, 'floor')
    r = L.analyse(s, (0.0, 0.0), 0.0, floor)
    assert r['points'] == 0                   # −80° 빔은 반경 0.16 m 안 → 차체(0.6 x 0.4) 윗면
    s2 = L.Sensor('down', (-30.0,), 36, 0.9, 0.1, 30.0)
    assert L.analyse(s2, (0.0, 0.0), 0.0, floor)['points'] == 36


def test_boxes_from_primitives_plane_slab_below_normal():
    t = np.eye(4)[:3].ravel().tolist()
    b = L.boxes_from_primitives([{'name': 'f', 'label': 'floor', 'kind': 'plane',
                                  'size': [10.0, 10.0], 'T': t}])
    assert b.c[0] == pytest.approx([0, 0, -0.05]) and b.h[0] == pytest.approx([5, 5, 0.05])
    with pytest.raises(ValueError):
        L.boxes_from_primitives([{'name': 'c', 'label': 'x', 'kind': 'cylinder', 'size': [1, 1],
                                  'T': t}])


def test_s6_variants_defined():
    for v in ('queue', 'queue2', 'follow', 'wall'):
        assert len(L.s6_blockers(v)) >= 1
    assert L.s6_blockers('open') is None
    with pytest.raises(ValueError):
        L.s6_blockers('bogus')
