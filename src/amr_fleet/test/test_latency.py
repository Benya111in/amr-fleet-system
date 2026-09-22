"""latency: 통신 지연 샘플링, 지연 모델, 지연 큐 (명세 4.9 통신 지연 ≤ 100 ms)."""

import random

import pytest

from amr_fleet.latency import DelayQueue, LatencyModel, parse_latency_range, sample_latency_s


def test_sample_latency_bounds_and_zero():
    rng = random.Random(7)
    samples = [sample_latency_s(100.0, rng) for _ in range(2000)]
    assert all(0.0 <= s <= 0.1 for s in samples)
    assert max(samples) > 0.09 and min(samples) < 0.01       # 분포가 구간을 채운다
    assert abs(sum(samples) / len(samples) - 0.05) < 0.005   # 평균 ≈ 50 ms
    assert sample_latency_s(0.0, rng) == 0.0
    assert sample_latency_s(-3.0, rng) == 0.0
    assert 0.0 <= sample_latency_s(10.0) <= 0.01             # 전역 random 도 허용


def test_sample_latency_min_bound():
    rng = random.Random(3)
    samples = [sample_latency_s(100.0, rng, min_ms=20.0) for _ in range(500)]
    assert all(0.02 <= s <= 0.1 for s in samples)
    assert sample_latency_s(10.0, rng, min_ms=50.0) == 0.01   # 하한 > 상한 → 상한으로


@pytest.mark.parametrize('value, expected', [
    ([0.0, 100.0], (0.0, 100.0)),
    ((10, 40), (10.0, 40.0)),
    ([80.0], (0.0, 80.0)),
    (100.0, (0.0, 100.0)),
    (0, (0.0, 0.0)),
])
def test_parse_latency_range_valid(value, expected):
    assert parse_latency_range(value) == expected


@pytest.mark.parametrize('value', [[-1.0, 10.0], [50.0, 10.0], [1.0, 2.0, 3.0], [], -5.0, True])
def test_parse_latency_range_invalid(value):
    with pytest.raises(ValueError):
        parse_latency_range(value)


def test_latency_model_uniform_range_and_counts():
    m = LatencyModel(simulate=True, comm_latency_ms=[0.0, 100.0], seed=11)
    assert m.enabled and not m.exceeds_spec()
    samples = [m.sample() for _ in range(3000)]
    assert list(m.history) == samples[-1000:]      # 주입한 지연 기록 (최근 1000 개)
    assert all(s is not None and 0.0 <= s <= 0.1 for s in samples)
    assert abs(sum(samples) / len(samples) - 0.05) < 0.004     # 설계 평균 ≈ 50 ms
    assert m.sent == 3000 and m.dropped == 0
    assert 'U[0, 100] ms' in m.describe() and 'seed=11' in m.describe()


def test_latency_model_seed_reproducible():
    a = LatencyModel(comm_latency_ms=[0.0, 100.0], seed=5)
    b = LatencyModel(comm_latency_ms=[0.0, 100.0], seed=5)
    assert [a.sample() for _ in range(20)] == [b.sample() for _ in range(20)]
    assert 'random' in LatencyModel(seed=0).describe()


def test_latency_model_drop_rate():
    m = LatencyModel(comm_latency_ms=[0.0, 10.0], drop_rate=0.25, seed=2)
    out = [m.sample() for _ in range(4000)]
    dropped = sum(1 for s in out if s is None)
    assert dropped == m.dropped and m.sent + m.dropped == 4000
    assert abs(dropped / 4000 - 0.25) < 0.03


def test_latency_model_disabled_and_zero():
    off = LatencyModel(simulate=False, comm_latency_ms=[0.0, 100.0], drop_rate=0.5)
    assert not off.enabled and off.describe() == 'off'
    assert all(off.sample() == 0.0 for _ in range(50)) and off.dropped == 0
    zero = LatencyModel(simulate=True, comm_latency_ms=[0.0, 0.0])
    assert not zero.enabled and zero.sample() == 0.0
    assert LatencyModel(comm_latency_ms=[0.0, 0.0], drop_rate=0.1).enabled
    assert LatencyModel(comm_latency_ms=[0.0, 250.0]).exceeds_spec()


@pytest.mark.parametrize('rate', [-0.1, 1.0, 2.0])
def test_latency_model_rejects_bad_drop_rate(rate):
    with pytest.raises(ValueError):
        LatencyModel(drop_rate=rate)


def test_delay_queue_releases_in_time_order():
    q = DelayQueue()
    assert len(q) == 0 and q.next_release() is None and q.pop_ready(1.0) == []
    assert q.push('late', now=0.0, delay_s=0.5) == 0.5
    assert q.push('early', now=0.0, delay_s=0.1) == 0.1
    assert q.push('same1', now=0.0, delay_s=0.1) == 0.1     # 같은 시각은 삽입 순
    q.push('negative_delay', now=0.0, delay_s=-1.0)
    assert len(q) == 4 and q.next_release() == 0.0
    assert q.pop_ready(0.0) == ['negative_delay']
    assert q.pop_ready(0.05) == []
    assert q.pop_ready(0.1) == ['early', 'same1']
    assert q.pop_ready(10.0) == ['late']
    assert len(q) == 0
    q.push('x', 0.0, 1.0)
    q.clear()
    assert len(q) == 0


def test_delay_queue_link_fifo_prevents_overtaking():
    q = DelayQueue()
    # 같은 링크: 두 번째가 더 짧은 지연을 뽑아도 첫 번째를 추월하지 않는다
    assert q.push('a1', now=0.00, delay_s=0.09, key='amr_01') == pytest.approx(0.09)
    assert q.push('a2', now=0.02, delay_s=0.01, key='amr_01') == pytest.approx(0.09)
    # 다른 링크와 키 없는 항목은 독립
    assert q.push('b1', now=0.02, delay_s=0.01, key='amr_02') == pytest.approx(0.03)
    assert q.push('n1', now=0.02, delay_s=0.0) == pytest.approx(0.02)
    assert q.pop_ready(0.05) == ['n1', 'b1']
    assert q.pop_ready(0.09) == ['a1', 'a2']
    # 링크가 비고 나면 새 메시지는 자기 지연만큼만 기다린다
    assert q.push('a3', now=1.0, delay_s=0.02, key='amr_01') == pytest.approx(1.02)
    q.clear()
    assert q.push('a4', now=0.5, delay_s=0.0, key='amr_01') == pytest.approx(0.5)
