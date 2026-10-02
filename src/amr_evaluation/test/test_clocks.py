import math

from amr_evaluation import clocks
import pytest


def test_clock_domain_and_expected():
    assert clocks.clock_domain(0.5) == clocks.SIM
    assert clocks.clock_domain(123456.0) == clocks.SIM
    assert clocks.clock_domain(1.79e9) == clocks.WALL
    assert clocks.expected_domain(True) == clocks.SIM
    assert clocks.expected_domain(False) == clocks.WALL


def test_domains_of_ignores_zero_nan_and_text():
    assert clocks.domains_of([0.0, float('nan'), 'x', None, 12.0]) == [clocks.SIM]
    assert clocks.domains_of([12.0, 1.79e9]) == [clocks.SIM, clocks.WALL]
    assert clocks.domains_of([]) == []


def test_rtf_estimator_ratio_window_and_reset():
    est = clocks.RtfEstimator(window=2.0, min_span=1.0)
    assert math.isnan(est.value())
    for i in range(11):                   # 벽시계 0.1 s 마다 시뮬 0.05 s → RTF 0.5
        est.add(i * 0.05, 100.0 + i * 0.1)
    assert est.value() == pytest.approx(0.5)
    for i in range(11, 60):               # 이후 실시간 → 창(2 s) 이 지나면 1.0
        est.add(0.5 + (i - 10) * 0.1, 100.0 + i * 0.1)
    assert est.value() == pytest.approx(1.0)
    est.add(0.1, 110.0)                   # 시뮬 리셋 → 다시 쌓는다
    assert math.isnan(est.value())
    short = clocks.RtfEstimator(min_span=1.0)
    short.add(0.0, 0.0)
    short.add(0.1, 0.5)
    assert math.isnan(short.value())


def test_wall_latency_ms():
    assert clocks.wall_latency_ms(100.0, 0.5) == pytest.approx(200.0)
    assert math.isnan(clocks.wall_latency_ms(100.0, float('nan')))
    assert math.isnan(clocks.wall_latency_ms(100.0, 0.0))
    assert math.isnan(clocks.wall_latency_ms(100.0, None))
