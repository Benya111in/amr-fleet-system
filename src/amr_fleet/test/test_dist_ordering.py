"""시각 · 요청 ID · 순서 키 시험 — (sec, nanosec) 전체 비교 (Open-RMF #551 회귀 방지)."""

import random

import pytest

from amr_fleet.distributed import messages as m
from amr_fleet.distributed.messages import VoteMsg
from amr_fleet.distributed.ordering import claim_key, NSEC, PriorityKey, ReqId, Stamp


def test_stamp_normalizes_and_converts():
    assert Stamp(1, NSEC + 5) == Stamp(2, 5)
    assert Stamp(0, -1) == Stamp(-1, NSEC - 1)
    assert Stamp.from_seconds(1.25) == Stamp(1, 250_000_000)
    assert Stamp(3, 500_000_000).to_seconds() == 3.5
    assert Stamp(1, 900_000_000).add(0.2) == Stamp(2, 100_000_000)
    assert Stamp.coerce(2) == Stamp(2, 0)
    assert Stamp.from_dict(Stamp(4, 7).to_dict()) == Stamp(4, 7)
    with pytest.raises(ValueError):
        Stamp.from_seconds(float('nan'))


def test_stamp_order_uses_sec_and_nanosec_on_random_pairs():
    """무작위 10 000 쌍: Stamp 순서 == 정수 나노초 순서. nanosec 만 보면 약 절반이 틀린다."""
    rng = random.Random(551)
    wrong_nsec_only = 0
    for _ in range(10_000):
        a = Stamp(rng.randrange(0, 100), rng.randrange(0, NSEC))
        b = Stamp(rng.randrange(0, 100), rng.randrange(0, NSEC))
        assert (a < b) == (a.total_ns() < b.total_ns())
        assert (claim_key(a, 'x') < claim_key(b, 'x')) == (a.total_ns() < b.total_ns())
        if (a.nanosec < b.nanosec) != (a.total_ns() < b.total_ns()):
            wrong_nsec_only += 1
    assert wrong_nsec_only > 4000           # 대조: #551 의 nanosec 만 비교는 이만큼 틀린다


def test_claim_key_ties_break_by_robot_id():
    t = Stamp(5, 5)
    assert claim_key(t, 'amr_01') < claim_key(t, 'amr_02')
    assert claim_key(Stamp(4, NSEC - 1), 'amr_09') < claim_key(Stamp(5, 0), 'amr_01')


def test_priority_key_order():
    r = ReqId('b', 1, 1)
    base = PriorityKey(priority=1, deadline=Stamp(100), since=10)
    starved_old = PriorityKey(priority=0, deadline=None, since=5, starved=True)
    starved_new = PriorityKey(priority=9, deadline=None, since=7, starved=True)
    high = PriorityKey(priority=5, deadline=None, since=50)
    early = PriorityKey(priority=1, deadline=Stamp(50), since=99)
    older = PriorityKey(priority=1, deadline=Stamp(100), since=3)
    order = sorted([base, high, early, older, starved_new, starved_old],
                   key=lambda k: k.sort_key(r))
    assert order == [starved_old, starved_new, high, early, older, base]
    # 마감 없음 = 가장 늦음
    assert PriorityKey(1, Stamp(10**9), 1).sort_key(r) < PriorityKey(1, None, 1).sort_key(r)
    # 같으면 robot_id → incarnation → seq
    k = PriorityKey(1, None, 1)
    assert k.sort_key(ReqId('a', 9, 9)) < k.sort_key(ReqId('b', 1, 1))
    assert k.sort_key(ReqId('a', 1, 2)) < k.sort_key(ReqId('a', 2, 1))
    assert PriorityKey.from_dict(early.to_dict()) == early


def test_req_id_order_and_roundtrip():
    assert ReqId('a', 1, 9) < ReqId('a', 2, 1) < ReqId('b', 0, 0)
    assert ReqId.from_list(ReqId('a', 3, 4).to_list()) == ReqId('a', 3, 4)
    assert str(ReqId('a', 3, 4)) == 'a#3.4'


def test_vote_msg_roundtrip_and_validation():
    msg = VoteMsg(m.REQUEST, 'a', 'b', 2, ReqId('a', 1, 1), {'z1', 'z0'},
                  PriorityKey(3, Stamp(9, 1), 4, True), Stamp(7, 2), 11, '')
    assert isinstance(msg.resources, frozenset)
    assert VoteMsg.from_dict(msg.to_dict()) == msg
    assert msg.to_dict()['resources'] == ['z0', 'z1']
    assert 'REQUEST(a#1.1 {z0,z1} e2) a->b' == msg.short()
    with pytest.raises(ValueError):
        VoteMsg('GRANT', 'a', 'b', 0, ReqId('a', 1, 1))
