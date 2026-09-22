"""스캔-맵 일치도와 전역 가설 탐색 테스트 (합성 방 + 광선 투사)."""

import math

from amr_localization import global_seed
from amr_localization.scan_map_match import (
    compose, DistanceField, GridSpec, match_ratio, scan_endpoints, transform_points)
from grid_sim import make_room, raycast
import numpy as np
import pytest


@pytest.fixture(scope='module')
def room():
    grid, res = make_room()
    spec = GridSpec(grid.shape[1], grid.shape[0], res)
    return grid, DistanceField(grid.ravel().tolist(), spec)


def test_distance_field_values(room):
    grid, field = room
    # 외벽(두께 0.1 m) 안쪽 0.5 m 지점: 벽까지 ≈ 0.4 m (셀 중심 기준 반 셀 오차)
    d = field.lookup(np.array([0.6]), np.array([7.0]))
    assert d[0] == pytest.approx(0.5, abs=0.06)
    assert field.lookup(np.array([-5.0]), np.array([1.0]))[0] == field.max_distance
    assert field.free[20, 20]
    assert not field.free[0, 0]


def test_distance_field_without_obstacles():
    spec = GridSpec(10, 10, 0.1)
    field = DistanceField([0] * 100, spec, max_distance=1.5)
    assert np.all(field.field == 1.5)


def test_rotated_origin_lookup():
    spec = GridSpec(4, 2, 1.0, origin_x=10.0, origin_y=0.0, origin_yaw=math.pi / 2)
    data = [0] * 8
    data[1 * 4 + 3] = 100            # 행 1, 열 3 → 격자 좌표 (3.5, 1.5) → map (10 − 1.5, 3.5)
    field = DistanceField(data, spec)
    assert field.lookup(np.array([8.5]), np.array([3.5]))[0] == pytest.approx(0.0)


def test_scan_endpoints_decimation_and_invalid():
    ranges = [1.0, float('inf'), float('nan'), 30.0, 2.0, 0.0]
    xs, ys = scan_endpoints(ranges, 0.0, 0.5, 25.0)
    assert xs.size == 2
    assert xs[0] == pytest.approx(1.0) and ys[0] == pytest.approx(0.0)
    xs, _ = scan_endpoints([1.0] * 720, -math.pi, 2 * math.pi / 720, 25.0, max_beams=180)
    assert xs.size == 180


def test_transform_and_compose():
    x, y = transform_points(np.array([1.0]), np.array([0.0]), (1.0, 1.0, math.pi / 2))
    assert (x[0], y[0]) == pytest.approx((1.0, 2.0))
    assert compose((1.0, 1.0, math.pi / 2), (1.0, 0.0, math.pi)) == pytest.approx(
        (1.0, 2.0, -math.pi / 2))


def test_match_ratio_true_vs_kidnapped_pose(room):
    grid, field = room
    true_pose = (6.0, 6.0, 0.3)
    offset = (0.15, 0.0, 0.0)
    sensor = compose(true_pose, offset)
    ranges, amin, inc = raycast(grid, 0.05, sensor, noise=0.03, seed=1)
    ratio, valid = match_ratio(field, true_pose, offset, ranges, amin, inc, 25.0)
    assert valid > 150
    assert ratio > 0.95
    wrong, _ = match_ratio(field, (14.0, 5.0, 2.0), offset, ranges, amin, inc, 25.0)
    assert wrong < 0.4
    empty, n = match_ratio(field, true_pose, offset, [float('inf')] * 10, amin, inc, 25.0)
    assert (empty, n) == (0.0, 0)


def test_global_seed_finds_true_pose(room):
    grid, field = room
    true_pose = (10.3, 5.2, -2.1)
    offset = (0.15, 0.0, 0.0)
    ranges, amin, inc = raycast(grid, 0.05, compose(true_pose, offset), noise=0.03, seed=2)
    hyps = global_seed.search(field, field.free, ranges, amin, inc, 25.0, offset)
    assert 1 <= len(hyps) <= global_seed.SeedParams().max_candidates
    best = hyps[0]
    assert math.hypot(best.x - true_pose[0], best.y - true_pose[1]) < 0.15
    assert abs(math.atan2(math.sin(best.yaw - true_pose[2]),
                          math.cos(best.yaw - true_pose[2]))) < math.radians(3.0)
    assert best.score < 0.08
    assert best.ratio > 0.9
    assert all(h.ratio <= best.ratio for h in hyps)       # 최종 순위 = 전체 빔 인라이어 비율


def test_global_seed_degenerate_inputs(room):
    grid, field = room
    assert global_seed.search(field, field.free, [float('inf')] * 720, -math.pi,
                              2 * math.pi / 720, 25.0) == []
    no_free = np.zeros_like(field.free)
    ranges, amin, inc = raycast(grid, 0.05, (6.0, 6.0, 0.0))
    assert global_seed.search(field, no_free, ranges, amin, inc, 25.0) == []


def test_nms_keeps_distinct():
    hyp = global_seed.Hypothesis
    hyps = [hyp(0.0, 0.0, 0.0, 0.1), hyp(0.2, 0.0, 0.0, 0.2), hyp(5.0, 0.0, 0.0, 0.3),
            hyp(0.0, 0.0, math.pi, 0.4)]
    kept = global_seed.nms(hyps, 1.0, math.radians(20.0), 10)
    assert [h.score for h in kept] == [0.1, 0.3, 0.4]
    assert len(global_seed.nms(hyps, 1.0, math.radians(20.0), 1)) == 1


def test_global_seed_coarse_beams_longer_than_fine(room):
    # 회귀: coarse/fine 은 다른 빔 부분집합 → 테두리는 두 집합 중 가장 긴 빔 기준이어야 한다
    grid, field = room
    true_pose = (2.0, 2.0, 0.8)
    ranges, amin, inc = raycast(grid, 0.05, true_pose, noise=0.0, seed=3)
    params = global_seed.SeedParams(coarse_beams=720, fine_beams=7, max_candidates=2)
    hyps = global_seed.search(field, field.free, ranges, amin, inc, 25.0, params=params)
    assert 1 <= len(hyps) <= 2


def test_lookup_non_finite_is_max_distance(room):
    _, field = room
    d = field.lookup(np.array([np.nan, np.inf, 1.0]), np.array([1.0, 1.0, -np.inf]))
    assert np.all(d == field.max_distance)


@pytest.mark.parametrize('origin_yaw', [0.0, 0.7])
def test_grid_scorer_matches_float_scorer(origin_yaw):
    # 정수 오프셋 조회 = 부동소수 끝점 조회 (셀 중심 가설, 회전된 맵 원점 포함)
    grid, res = make_room()
    spec = GridSpec(grid.shape[1], grid.shape[0], res, 3.0, -2.0, origin_yaw)
    field = DistanceField(grid.ravel().tolist(), spec)
    rng = np.random.default_rng(4)
    beams = rng.uniform(-6.0, 6.0, size=(90, 2))
    rows = rng.integers(0, spec.height, 200)
    cols = rng.integers(0, spec.width, 200)
    xy = global_seed.cell_centers(spec, rows, cols)
    scorer = global_seed.GridScorer(field, float(np.max(np.hypot(beams[:, 0], beams[:, 1]))))
    for yaw in (-2.0, 0.0, 0.4, 3.0):
        grid_sc = scorer.scores(scorer.base_index(rows, cols), scorer.beam_offsets(beams, yaw),
                                1.0, 0.8)
        float_sc = global_seed.score_poses(field, beams, xy[:, 0], xy[:, 1],
                                           np.full(len(rows), yaw), 1.0, 0.8)
        # 경계에 정확히 떨어진 끝점의 반올림 차이만 허용
        assert np.mean(np.abs(grid_sc - float_sc)) < 1e-3
    assert len(global_seed.candidate_positions(field, field.free, 0.5, 0.3)) > 100
