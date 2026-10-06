"""amr_rl_bench.world_geom 시험 — 손으로 푼 사례 + Isaac 정적 기하와 대조."""

import math
import os
import sys

from amr_rl_bench import world_geom as G
import numpy as np
import pytest

from conftest import REPO


def test_footprint_clearance_box_ahead_and_overlap():
    """앞 1 m 상자: 풋프린트 앞면(0.30)에서 상자 면(0.95)까지 0.65. 겹치면 음수 (최소 겹침 폭)."""
    w = G.StaticWorld([[1.0, 0.0, 0.05, 0.5, 0.0]])
    assert w.footprint_clearance(0.0, 0.0, 0.0) == pytest.approx(0.65)
    assert w.footprint_clearance(0.0, 0.0, math.pi) == pytest.approx(0.65)
    # 옆으로 돌리면 반폭 0.20 → 0.75
    assert w.footprint_clearance(0.0, 0.0, math.pi / 2) == pytest.approx(0.75)
    # 앞면 1.00 이 상자 [0.95, 1.05] 안으로 0.05 들어갔다
    assert w.footprint_clearance(0.70, 0.0, 0.0) == pytest.approx(-0.05)
    assert w.footprint_clearance(-10.0, 0.0, 0.0, horizon=3.0) == 3.0


def test_straight_approach_and_point_clearance():
    """직진 접근: 앞 상자까지 0.65, 옆으로 비켜 있으면 inf, 이미 겹치면 0."""
    w = G.StaticWorld([[1.0, 0.0, 0.05, 0.5, 0.0]])
    assert w.straight_approach(0.0, 0.0, 0.0) == pytest.approx(0.65)
    assert w.straight_approach(0.0, 0.0, math.pi) == math.inf
    assert w.straight_approach(0.0, 0.8, 0.0) == math.inf        # 상자 |y| ≤ 0.5, 풋프린트 y 0.6–1.0
    assert w.straight_approach(0.0, 0.65, 0.0) == pytest.approx(0.65)   # 모서리 스침
    assert w.straight_approach(0.7, 0.0, 0.0) == 0.0
    assert w.point_clearance(0.0, 0.0) == pytest.approx(0.95)
    assert w.point_clearance(1.0, 0.0) == pytest.approx(-0.05)


def test_rotated_box():
    """45° 상자 꼭짓점이 풋프린트 앞면을 향할 때."""
    h = 0.5
    w = G.StaticWorld([[2.0, 0.0, h, h, math.pi / 4]])
    d = 2.0 - h * math.sqrt(2.0) - 0.30
    assert w.footprint_clearance(0.0, 0.0, 0.0) == pytest.approx(d)
    assert w.straight_approach(0.0, 0.0, 0.0) == pytest.approx(d)


def test_warehouse_rectangles_match_isaac():
    """창고 월드의 사각형이 Isaac 학습 환경(rl/isaac warehouse_geom)과 같다."""
    sdf = os.path.join(REPO, 'src', 'amr_simulation', 'worlds', 'warehouse.sdf')
    models = os.path.join(REPO, 'src', 'amr_simulation', 'models')
    isaac = os.path.join(REPO, 'rl', 'isaac')
    if not (os.path.exists(sdf) and os.path.isdir(isaac)):
        pytest.skip('저장소 월드·rl/isaac 없음')
    a = G.load_obb(sdf, models)
    sys.path.insert(0, isaac)
    try:
        from amr_isaac import warehouse_geom as W
    finally:
        sys.path.remove(isaac)
    b = W.obstacles_2d(W.load_boxes(sdf, models))
    assert a.shape == b.shape and a.shape[0] > 100
    np.testing.assert_allclose(a, b, atol=1e-12)
