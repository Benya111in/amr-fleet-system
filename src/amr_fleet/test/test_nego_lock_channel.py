"""negotiation.task_lock (가짜 잠금 계약) · channel (링크 모델)."""

import random
import statistics

import pytest

from amr_fleet.negotiation.channel import derive_seed, PerLinkChannel, SharedStreamChannel
from amr_fleet.negotiation.task_lock import (ExclusiveTaskLock, LockStatus, OptimisticTaskLock,
                                             TaskLock)


# ---------- 작업 잠금 ----------

def test_task_lock_is_abstract():
    with pytest.raises(TypeError):
        TaskLock()


def test_optimistic_lock_always_grants():
    lock = OptimisticTaskLock()
    assert lock.try_acquire('t', 'a', 0).status is LockStatus.GRANTED
    assert lock.try_acquire('t', 'b', 0).status is LockStatus.GRANTED   # 중복을 막지 않는다
    assert lock.grants == {'t': ['a', 'b']}
    lock.release('t', 'a', 1)
    assert lock.holder('t') is None


def test_exclusive_lock_mutual_exclusion_and_release():
    lock = ExclusiveTaskLock()
    assert lock.try_acquire('t', 'a', 0).status is LockStatus.GRANTED
    assert lock.try_acquire('t', 'a', 1).status is LockStatus.GRANTED   # 멱등
    res = lock.try_acquire('t', 'b', 1)
    assert res.status is LockStatus.DENIED and res.holder == 'a'
    lock.release('t', 'b', 2)                                           # 남의 잠금은 못 푼다
    assert lock.holder('t') == 'a'
    lock.release('t', 'a', 2)
    lock.release('t', 'a', 2)                                           # 멱등
    assert lock.try_acquire('t', 'b', 3).status is LockStatus.GRANTED
    assert lock.denied == 1


def test_exclusive_lock_grant_delay_and_race():
    lock = ExclusiveTaskLock(grant_delay=2)
    assert lock.try_acquire('t', 'a', 0).status is LockStatus.PENDING
    assert lock.try_acquire('t', 'b', 1).status is LockStatus.PENDING
    assert lock.try_acquire('t', 'a', 2).status is LockStatus.GRANTED
    assert lock.try_acquire('t', 'b', 3).status is LockStatus.DENIED    # 늦게 모은 쪽이 진다
    with pytest.raises(ValueError):
        ExclusiveTaskLock(grant_delay=-1)


# ---------- 링크 모델 ----------

def test_shared_stream_consumes_like_reference():
    """verify_mrta2.py: 손실 추첨 1 회 / mrta_small.py: 손실 + (전달 시) 지연 1 회."""
    rng_ref = random.Random(7)
    expect = []
    for _ in range(50):
        lost = rng_ref.random() < 0.4
        expect.append((not lost, 0.0 if lost else rng_ref.uniform(0.0, 100.0)))
    ch = SharedStreamChannel(random.Random(7), p_loss=0.4, draw_delay=True)
    assert [ch.deliver('a', 'b', 1) for _ in range(50)] == expect
    rng_ref = random.Random(7)
    ch = SharedStreamChannel(random.Random(7), p_loss=0.4)
    assert [ch.deliver('a', 'b', 1)[0] for _ in range(50)] == [
        not rng_ref.random() < 0.4 for _ in range(50)]


def test_per_link_channel_is_deterministic_and_link_independent():
    a = PerLinkChannel(seed=3, p_loss=0.3)
    b = PerLinkChannel(seed=3, p_loss=0.3)
    seq_a = [a.deliver('r1', 'r2', i) for i in range(30)]
    # b 는 다른 링크를 먼저 많이 쓴다 → r1→r2 난수열은 밀리지 않아야 한다
    for i in range(100):
        b.deliver('r3', 'r4', i)
    assert [b.deliver('r1', 'r2', i) for i in range(30)] == seq_a
    assert seq_a != [PerLinkChannel(seed=4, p_loss=0.3).deliver('r1', 'r2', i) for i in range(30)]
    assert derive_seed(1, 'a', 'b') == derive_seed(1, 'a', 'b') != derive_seed(1, 'b', 'a')


def test_per_link_channel_loss_rate_and_delay_range():
    ch = PerLinkChannel(seed=1, p_loss=0.2, delay_ms=(10.0, 30.0))
    out = [ch.deliver('a', 'b', i) for i in range(20000)]
    loss = sum(1 for ok, _ in out if not ok) / len(out)
    assert abs(loss - 0.2) < 0.015
    delays = [d for ok, d in out if ok]
    assert min(delays) >= 10.0 and max(delays) <= 30.0


def test_burst_loss_keeps_mean_but_lengthens_runs():
    def runs(ch):
        lost = [not ch.deliver('a', 'b', i)[0] for i in range(40000)]
        lengths, cur = [], 0
        for x in lost:
            if x:
                cur += 1
            elif cur:
                lengths.append(cur)
                cur = 0
        return sum(lost) / len(lost), statistics.mean(lengths)
    p_b, run_b = runs(PerLinkChannel(seed=2, p_loss=0.2))
    p_g, run_g = runs(PerLinkChannel(seed=2, p_loss=0.2, burst_rho=0.8))
    assert abs(p_b - 0.2) < 0.02 and abs(p_g - 0.2) < 0.03
    assert run_g > 2.5 * run_b             # Bernoulli 평균 연속 1.25 vs Gilbert 1/(0.8·0.2) ≈ 6.25


def test_partition_cuts_link_in_window():
    ch = PerLinkChannel(seed=0, p_loss=0.0, partitions={('a', 'b'): (5, 8)})
    got = [ch.deliver('a', 'b', r)[0] for r in range(10)]
    assert got == [True] * 5 + [False] * 3 + [True] * 2
    assert all(ch.deliver('b', 'a', r)[0] for r in range(10))


def test_channel_argument_checks():
    with pytest.raises(ValueError):
        PerLinkChannel(seed=0, p_loss=1.0)
    with pytest.raises(ValueError):
        PerLinkChannel(seed=0, burst_rho=1.0)
    with pytest.raises(ValueError):
        PerLinkChannel(seed=0, delay_ms=(5.0, 1.0))
    with pytest.raises(ValueError):
        SharedStreamChannel(random.Random(0), p_loss=-0.1)
