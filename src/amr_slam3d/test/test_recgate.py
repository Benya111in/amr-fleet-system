"""녹화 직후 공백 검사(recgate.py) 시험 — 간격·덮음·정지 구간·출발 시각 규칙과 파일럿 bag 형 정체."""

import math

from amr_slam3d import recgate
import numpy as np
import pytest

MS = 1_000_000


def _ticks(t0_ns, period_ns, n):
    return [t0_ns + i * period_ns for i in range(n)]


def _clean_series(seconds=30.0, t0=5.0):
    """모든 검사 토픽이 명목 주기 그대로인 계열 (정수 ns)."""
    out = {}
    for tp, per in recgate.GATE_PERIODS_NS.items():
        out[tp] = _ticks(int(t0 * 1e9), per, int(seconds * 1e9 // per) + 1)
    return out


def _gt(still0=12.0, move=10.0, still1=2.0, t0=5.0, rate=50.0):
    """정답 자세: still0 s 정지 → move s 동안 x 로 0.5 m/s → still1 s 정지."""
    n = int((still0 + move + still1) * rate) + 1
    t = t0 + np.arange(n) / rate
    rel = t - t0
    x = np.clip(rel - still0, 0.0, move) * 0.5 + 18.0
    return t, x, np.full(n, -16.0), np.full(n, math.pi / 2)


def test_exact_one_and_half_period_passes_but_more_fails():
    """1.5 주기 + 1 µs 까지는 통과 (정수 비교), 그보다 2 µs 크면 공백."""
    per = 20 * MS
    ok = recgate.interval_gaps([0, per, per + 30 * MS, 2 * per + 30 * MS], per)
    assert ok['ok'] and ok['n_gap'] == 0
    bad = recgate.interval_gaps([0, per, per + 30 * MS + 2_000], per)
    assert not bad['ok'] and bad['n_gap'] == 1
    assert bad['gaps'][0] == (pytest.approx(0.02), pytest.approx(0.030002))


def test_float_boundary_regression_from_pilot():
    """파일럿 EKF 의 7.475 → 7.505 s (정확히 1.5 주기)는 실수 빼기로 0.030000000000000249 이 되어 공백으로 세였다."""
    a, b = 7.475, 7.505
    assert b - a > 1.5 * 0.02                      # 실수로 세면 공백 (예전 bagstat 방식)
    ns = [round(v * 1e9) for v in (7.455, a, b, 7.525)]
    assert recgate.interval_gaps(ns, 20 * MS)['ok']


def test_duplicates_and_backwards_fail():
    per = 5 * MS
    assert not recgate.interval_gaps([0, per, per, 2 * per], per)['ok']
    r = recgate.interval_gaps([0, 2 * per, per, 3 * per], per)
    assert not r['ok'] and r['backwards'] == 1


def test_too_few_samples_fail():
    assert not recgate.interval_gaps([5], 5 * MS)['ok']
    assert not recgate.interval_gaps([], 5 * MS)['ok']


def test_coverage_detects_topic_that_stops_early():
    per = 5 * MS
    ref = (0, 10_000 * MS)
    assert recgate.coverage(0, 10_000 * MS - per, *ref, per)['ok']
    r = recgate.coverage(0, 9_000 * MS, *ref, per)          # 1 s 일찍 끊김
    assert not r['ok'] and r['tail_s'] == pytest.approx(1.0)
    assert not recgate.coverage(600 * MS, 10_000 * MS, *ref, per)['ok']   # 0.6 s 늦게 시작


def test_still_spans_start_end_and_depart():
    t, x, y, yaw = _gt(still0=12.0, move=10.0, still1=2.0)
    sp = recgate.still_spans(t, x, y, yaw)
    # 0.005 m 를 넘는 첫 표본: rel − 12 > 0.01 s → 12.02 s (50 Hz)
    assert sp['start_still_s'] == pytest.approx(12.02, abs=1e-9)
    assert sp['end_still_s'] == pytest.approx(2.0 + 0.0, abs=0.03)
    assert sp['depart_t'] == pytest.approx(5.0 + 12.02, abs=1e-9)


def test_still_spans_ignore_submillimetre_noise_but_see_rotation():
    t, x, y, yaw = _gt(still0=12.0, move=0.0, still1=0.0)
    x = x + 1e-4 * np.sin(np.arange(len(x)))
    sp = recgate.still_spans(t, x, y, yaw)
    assert math.isnan(sp['depart_t'])
    yaw2 = yaw.copy()
    yaw2[300:] += math.radians(0.3)                     # 제자리 회전도 움직임
    sp2 = recgate.still_spans(t, x, y, yaw2)
    assert sp2['depart_t'] == pytest.approx(t[300])


def test_clean_bag_passes():
    res = recgate.evaluate(_clean_series(), _gt())
    assert res['pass'], res['reasons']
    assert all(v['ok'] for v in res['topics'].values())


def test_pilot_like_stall_fails_on_clock_and_ekf():
    """파일럿 bag 의 sim 9.48–9.64 s 정체: /clock 23 ms 틱 누락, EKF 0.065·0.1 s 공백 → 실패."""
    s = _clean_series(seconds=20.0, t0=5.0)
    clk = [v for v in s['/clock'] if not (9_494 * MS < v < 9_517 * MS)]
    s['/clock'] = clk
    s[recgate.EKF_TF] = [v for v in s[recgate.EKF_TF] if not (9_475 * MS < v < 9_640 * MS)]
    res = recgate.evaluate(s, _gt(still0=12.0, move=5.0, still1=2.0))
    assert not res['pass']
    assert res['topics']['/clock']['n_gap'] == 1
    assert res['topics'][recgate.EKF_TF]['n_gap'] == 1
    assert any(r.startswith('/clock') for r in res['reasons'])
    # 점군·IMU 는 그대로라 통과 (파일럿과 같다)
    assert res['topics']['/amr_01/lidar3d/points']['ok']


def test_missing_topic_or_gt_fails():
    s = _clean_series()
    del s['/amr_01/imu/data']
    res = recgate.evaluate(s, _gt())
    assert not res['pass'] and any('imu/data' in r for r in res['reasons'])
    assert not recgate.evaluate(_clean_series(), None)['pass']


def test_protocol_still_rules():
    assert not recgate.evaluate(_clean_series(), _gt(still0=9.0))['pass']       # 처음 정지 < 10 s
    assert not recgate.evaluate(_clean_series(), _gt(still1=0.3))['pass']       # 끝 정지 < 0.5 s
    assert recgate.evaluate(_clean_series(), _gt(still0=10.5, still1=0.6))['pass']


def test_depart_window_for_s6_pairing():
    t, x, y, yaw = _gt(still0=12.0, move=5.0, still1=2.0, t0=5.0)   # 출발 17.02 s
    assert recgate.evaluate(_clean_series(), (t, x, y, yaw), depart_at=17.0)['pass']
    r = recgate.evaluate(_clean_series(), (t, x, y, yaw), depart_at=16.0)
    assert not r['pass'] and any('출발' in m for m in r['reasons'])
    assert not recgate.evaluate(_clean_series(), (t, x, y, yaw), depart_at=17.5)['pass']


def test_clock_lead_diagnostic():
    order = [('/clock', 9_480 * MS), ('/amr_01/lidar3d/points', 9_600 * MS),
             ('/clock', 9_600 * MS), ('/amr_01/lidar3d/points', 9_700 * MS),
             ('/amr_01/imu/data', 9_600 * MS)]
    lead = recgate.clock_lead(order)
    assert lead['/amr_01/lidar3d/points']['max_s'] == pytest.approx(0.12)
    assert lead['/amr_01/imu/data']['max_s'] == pytest.approx(0.0)
    assert '/amr_01/imu/data_raw' not in lead
