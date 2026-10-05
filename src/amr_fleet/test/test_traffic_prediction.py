"""traffic_prediction: 시공간 표본 · 정면/교차/추종 분류 · 가장 이른 충돌."""

import math

import numpy as np
import pytest

from amr_fleet.traffic_prediction import (
    CROSSING, FOLLOWING, HEAD_ON, classify_conflict, find_conflict, predict_conflicts,
    predict_trajectory,
)

SAFE = 1.02
WIN = 2.0


def line(x0, y0, x1, y1):
    return np.array([[x0, y0], [x1, y1]], dtype=float)


def test_predict_trajectory_follows_path_and_stops_at_end():
    tr = predict_trajectory('a', 0.0, 0.1, 0.0, line(0, 0, 3, 0), speed=1.0, horizon_s=5.0,
                            dt_s=0.5)
    assert tr.moving and len(tr.t) == 11
    assert tr.xy[0].tolist() == [0.0, 0.1]              # 첫 표본은 실제 자세
    assert tr.xy[2].tolist() == pytest.approx([1.0, 0.0])
    assert tr.xy[-1].tolist() == pytest.approx([3.0, 0.0])   # 끝에서 머문다
    assert tr.heading[3] == pytest.approx(0.0)
    tr2 = predict_trajectory('a', 1.0, 0.0, 0.0, line(0, 0, 3, 0), s0=2.0, horizon_s=1.0,
                             dt_s=0.5)
    assert tr2.xy[1].tolist() == pytest.approx([2.5, 0.0])


@pytest.mark.parametrize('path,speed,x', [
    (None, 1.0, 0.0), (line(0, 0, 3, 0)[:1], 1.0, 0.0), (line(0, 0, 3, 0), 0.0, 0.0),
    (line(0, 0, 3, 0), 1.0, 3.0),                      # 이미 끝
])
def test_predict_trajectory_stationary_cases(path, speed, x):
    tr = predict_trajectory('a', x, 0.0, 0.7, path, speed=speed, horizon_s=2.0, dt_s=0.5)
    assert not tr.moving
    assert np.allclose(tr.xy, [[x, 0.0]]) and np.allclose(tr.heading, 0.7)


def test_predict_trajectory_rejects_bad_timing():
    with pytest.raises(ValueError):
        predict_trajectory('a', 0, 0, 0, None, horizon_s=0.0)
    with pytest.raises(ValueError):
        predict_trajectory('a', 0, 0, 0, None, dt_s=-1.0)


def test_classify_conflict():
    assert classify_conflict(0.0, math.pi) == HEAD_ON
    assert classify_conflict(0.0, 0.3) == FOLLOWING
    assert classify_conflict(0.0, math.pi / 2) == CROSSING
    assert classify_conflict(0.0, math.radians(140), head_on_deg=150.0) == CROSSING


def test_head_on_conflict_in_the_middle():
    a = predict_trajectory('a', 0, 0, 0, line(0, 0, 10, 0))
    b = predict_trajectory('b', 10, 0, math.pi, line(10, 0, 0, 0))
    c = find_conflict(a, b, SAFE, WIN)
    assert c is not None and c.kind == HEAD_ON
    assert c.x == pytest.approx(5.0, abs=0.6) and c.distance < SAFE
    assert c.t_first == pytest.approx(min(c.t_a, c.t_b))
    assert c.time_of('a') == c.t_a and c.time_of('b') == c.t_b
    assert c.other('a') == 'b' and c.other('b') == 'a'


def test_crossing_and_time_window():
    a = predict_trajectory('a', 0, 5, 0, line(0, 5, 10, 5))
    b = predict_trajectory('b', 5, 0, math.pi / 2, line(5, 0, 5, 10))
    c = find_conflict(a, b, SAFE, WIN)
    assert c is not None and c.kind == CROSSING and c.t_a == pytest.approx(c.t_b, abs=1.0)
    # b 가 4 s 늦게 출발 → 같은 자리를 time_window 밖에서 지나가므로 충돌 아님
    late = predict_trajectory('b', 5, -4, math.pi / 2, line(5, -4, 5, 10))
    assert find_conflict(a, late, SAFE, WIN) is None
    # 둘 다 정지면 예측 대상이 아니다
    s1 = predict_trajectory('s1', 0, 0, 0, None)
    s2 = predict_trajectory('s2', 0.5, 0, 0, None)
    assert find_conflict(s1, s2, SAFE, WIN) is None


def test_moving_into_stationary_and_following():
    parked = predict_trajectory('p', 6, 0, 0, None)
    a = predict_trajectory('a', 0, 0, 0, line(0, 0, 10, 0))
    c = find_conflict(a, parked, SAFE, WIN)
    assert c is not None and c.x == pytest.approx(5.5, abs=0.6)
    lead = predict_trajectory('l', 1.5, 0, 0, line(1.5, 0, 20, 0))
    follow = predict_trajectory('f', 0, 0, 0, line(0, 0, 20, 0))
    c2 = find_conflict(follow, lead, SAFE, WIN)
    assert c2 is not None and c2.kind == FOLLOWING


def test_predict_conflicts_sorted_pairs():
    trs = [
        predict_trajectory('c', 5, 0, math.pi / 2, line(5, 0, 5, 10)),
        predict_trajectory('a', 0, 5, 0, line(0, 5, 10, 5)),
        predict_trajectory('b', 30, 30, 0, line(30, 30, 40, 30)),
        predict_trajectory('d', 10, 5.2, math.pi, line(10, 5.2, 7, 5.2)),
    ]
    out = predict_conflicts(trs, SAFE, WIN)
    pairs = [(c.robot_a, c.robot_b) for c in out]
    assert ('a', 'c') in pairs and ('a', 'd') in pairs
    assert all(a < b for a, b in pairs) and 'b' not in {x for p in pairs for x in p}
    assert [c.t_first for c in out] == sorted(c.t_first for c in out)
