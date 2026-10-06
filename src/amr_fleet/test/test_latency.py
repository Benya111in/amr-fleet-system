"""latency: 지연 구간 파싱, 지연 큐 (명세 4.9 통신 지연 ≤ 100 ms). 지연 모델은 test_comm_emulator."""

import pytest

from amr_fleet.latency import DelayQueue, parse_latency_range


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
