"""
E0 평가기 시험 — 참조 도구(alias_margin.py)와의 일치, 이웃 가림, 이웃 자리 생성.

    cd /ros2_ws && python3 -m pytest -q layopt/loc/test_e0.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from layopt.loc import e0, mapio, raycast

REF = mapio.REPO / 'docs' / 'research' / 'state-estimation' / 'checks' / 'alias_margin.py'


def _reference():
    """alias_margin.py 의 정의부만 실행 (모듈 끝의 보고 실행은 뺀다)."""
    src = REF.read_text(encoding='utf-8').splitlines()
    cut = [i for i, line in enumerate(src) if line.startswith('occ, spec = load_grid()')][0]
    g = {'__file__': str(REF), '__name__': 'alias_defs'}
    exec(compile('\n'.join(src[:cut]), str(REF), 'exec'), g)   # noqa: S102 — 저장소 내 참조 코드
    return g


@pytest.fixture(scope='module')
def ref():
    return _reference()


@pytest.fixture(scope='module')
def slam():
    m = mapio.load_map(mapio.REPO / 'maps' / 'warehouse.yaml')
    return m, mapio.distance_field(m)


def test_load_map_matches_reference(ref, slam):
    occ_ref, spec_ref = ref['load_grid']()
    m, _ = slam
    assert np.array_equal(occ_ref, m.occ)
    assert (spec_ref.width, spec_ref.height) == (m.spec.width, m.spec.height)
    assert spec_ref.origin_x == m.spec.origin_x and spec_ref.origin_y == m.spec.origin_y


@pytest.mark.parametrize('pose', [(18.0, -16.0, math.pi / 2), (0.8, 5.0, 0.0),
                                  (-21.0, 13.0, -math.pi / 2), (3.3, -7.7, 2.1)])
def test_raycast_bit_identical(ref, slam, pose):
    m, _ = slam
    want, amin_r, inc_r = ref['raycast'](m.occ, m.spec, pose)
    got, amin, inc = raycast.raycast(raycast.occupied_mask(m.occ), m.spec, pose)
    assert (amin, inc) == (amin_r, inc_r)
    assert np.array_equal(np.array(want), np.array(got))


def test_neighbor_box_occludes(slam):
    m, _ = slam
    pose = (18.0, -16.0, math.pi / 2)
    nb = [[20.0, -16.0, math.pi / 2]]
    world = raycast.occupied_mask(m.occ, e0.neighbor_cells(m.spec, nb))
    r0, amin, inc = raycast.raycast(raycast.occupied_mask(m.occ), m.spec, pose, noise=0.0)
    r1, _, _ = raycast.raycast(world, m.spec, pose, noise=0.0)
    # 빔 0 = −π (로봇 뒤). 로봇 헤딩 +y 에서 동쪽(+x) = 로봇 오른쪽 = 빔 각 −π/2 → 인덱스 180
    east = 180
    assert r1[east] < r0[east]
    # 라이다 (18, −15.85) 에서 이웃 상자 서쪽 면 x = 19.8 까지 ≈ 1.8 m
    assert 1.7 < r1[east] < 1.95
    # 기준 지도는 바뀌지 않는다
    assert int((m.occ >= 65).sum()) == int(raycast.occupied_mask(m.occ).sum())


def test_occluded_frac_counts_only_blocked_beams(slam):
    """두 이웃(양옆 2 m)이 가리는 빔 비율 ≈ 2 × 상자 시야각 / 360° (잡음 수열 밀림에 영향받지 않음)."""
    m, field = slam
    pose = (20.0, -16.0, math.pi / 2)
    nbs = [[18.0, -16.0, math.pi / 2], [22.0, -16.0, math.pi / 2]]
    r = e0.evaluate_pose(field, m.occ, pose, nbs)
    assert 0.07 < r['occluded_frac'] < 0.14
    assert r['inlier_raw'] < r['inlier']


def test_footprint_cells_count(slam):
    m, _ = slam
    rr, cc = mapio.footprint_cells(m.spec, (0.0, 0.0, 0.0), 0.6, 0.4)
    assert len(rr) == 12 * 8
    rr, cc = mapio.footprint_cells(m.spec, (0.0, 0.0, math.pi / 2), 0.6, 0.4)
    assert len(rr) == 12 * 8


def test_free_space_neighbors_are_free(slam):
    m, field = slam
    for pose in e0.sample_free_poses(field, 10, seed=3):
        nbs = e0.free_space_neighbors(m, pose)
        for n in nbs:
            assert e0.footprint_free(m, tuple(n))
            assert math.hypot(n[0] - pose[0], n[1] - pose[1]) >= 1.19


def test_sample_free_poses_deterministic(slam):
    _, field = slam
    a = e0.sample_free_poses(field, 7, seed=11)
    b = e0.sample_free_poses(field, 7, seed=11)
    assert a == b
    for x, y, yaw in a:
        assert yaw in (0.0, round(math.pi / 2, 6), round(math.pi, 6), round(-math.pi / 2, 6))


def test_evaluate_pose_matches_reference_margin(ref, slam):
    """참조 report() 와 같은 마진 (설문 spawn_alias.py 경로)."""
    m, field = slam
    rf = ref['DistanceField'](m.occ.ravel().tolist(), m.spec, 65)
    rf._occ = m.occ
    pose = (22.0, -16.0, math.pi / 2)
    _, _, margin_ref = ref['report'](rf, pose, 'test')
    got = e0.evaluate_pose(field, m.occ, pose)
    assert got['margin'] == pytest.approx(margin_ref, abs=5e-5)


def test_write_atomic_no_tmp_left(tmp_path: Path):
    p = tmp_path / 'x.json'
    e0.write_atomic(p, {'a': 1})
    assert p.exists() and not (tmp_path / 'x.json.tmp').exists()
