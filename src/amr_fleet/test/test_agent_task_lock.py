"""
작업 ID 잠금 어댑터 VoteTaskLock 단위 시험 (agent/task_lock_vote.py).

TaskLock 계약(negotiation/task_lock.py): 배타, GRANTED 비선점, 무응답 = 미허가, 확정 뒤에는 풀지 않는다.
여기서 GRANTED 는 "잠정 과반(이동 허가)" 이고 픽업 허가는 is_held (과반 ENTER_ACK) 다.
"""

from amr_fleet.agent.task_lock_vote import task_resource, VoteTaskLock
from amr_fleet.distributed.simnet import Cluster
from amr_fleet.distributed.vote_core import VoteConfig
from amr_fleet.negotiation.task_lock import LockStatus

MEMBERS = ('a', 'b', 'c', 'd', 'e')
CFG = VoteConfig(MEMBERS, tentative_s=10.0, retry_s=0.5, backoff_min_s=0.2, backoff_max_s=0.5)


class _Rig:
    """Cluster 로봇마다 VoteTaskLock 하나. 잠금이 쌓은 메시지는 pump() 가 망으로 보낸다."""

    def __init__(self, seed=0):
        self.c = Cluster(CFG, seed=seed)
        self.locks = {r: VoteTaskLock(self.c.agents[r], lambda r=r: self.c.clock[r], hold_s=5.0)
                      for r in MEMBERS}

    def pump(self):
        for lk in self.locks.values():
            self.c.send(lk.drain())

    def settle(self, rounds=5):
        for _ in range(rounds):
            self.pump()
            self.c.settle(rounds=1)
            for lk in self.locks.values():
                lk.poll()
        self.pump()


def test_pending_then_granted_on_tentative_quorum_then_held_after_commit():
    g = _Rig(1)
    lk = g.locks['a']
    assert lk.try_acquire('t1', 'a', 0.0).status is LockStatus.PENDING
    assert lk.phase('t1') == 'pending'
    g.pump()
    g.c.deliver_all()
    assert lk.try_acquire('t1', 'a', 1.0).status is LockStatus.GRANTED   # 이동 허가
    assert lk.phase('t1') == 'voted' and not lk.is_held('t1')            # 픽업 허가는 아직
    assert lk.commit('t1') == 'voted'
    g.pump()
    g.c.deliver_all()
    assert lk.is_held('t1') and lk.phase('t1') == 'held'
    assert lk.holder('t1') == 'a'


def test_held_lock_is_never_released_even_after_done():
    g = _Rig(2)
    lk = g.locks['a']
    lk.try_acquire('t1', 'a', 0.0)
    lk.commit('t1')
    g.settle()
    assert lk.is_held('t1')
    lk.release('t1', 'a', 0.0)
    assert lk.drain() == []                       # RELEASE 를 보내지 않는다
    assert lk.is_held('t1')


def test_release_before_commit_cancels_the_request():
    g = _Rig(3)
    lk = g.locks['a']
    lk.try_acquire('t1', 'a', 0.0)
    g.pump()
    g.c.deliver_all()
    lk.release('t1', 'a', 0.0)
    out = lk.drain()
    assert out and all(m.kind == 'ABANDON' for m in out)
    assert lk.pursuing() is None


def test_second_robot_is_denied_after_first_commits():
    g = _Rig(4)
    a, b = g.locks['a'], g.locks['b']
    a.try_acquire('t1', 'a', 0.0)
    a.commit('t1')
    g.settle()
    assert a.is_held('t1')
    assert b.try_acquire('t1', 'b', 0.0).status is LockStatus.PENDING
    for _ in range(10):
        g.settle(rounds=1)
        if b.try_acquire('t1', 'b', 0.0).status is LockStatus.DENIED:
            break
    assert b.is_denied('t1') and b.phase('t1') == 'denied'
    assert b.commit('t1') == 'denied' and not b.is_held('t1')
    assert g.c.check() == []


def test_own_unreleased_commit_votes_do_not_count_as_denial():
    """내 옛 시도의 확정 표(ABANDON 을 아직 못 받은 투표자)가 내는 busy 로 스스로 DENIED 하지 않는다."""
    g = _Rig(5)
    lk = g.locks['a']
    ag = g.c.agents['a']
    lk.try_acquire('t1', 'a', 0.0)
    lk.commit('t1')
    g.pump()
    # ENTER 까지만 배달하고 ENTER_ACK 는 버린다 → 투표자는 확정, a 는 ENTERING
    while g.c.net:
        msg = g.c.net[0]
        if msg.kind == 'ENTER_ACK':
            g.c.drop(0)
        else:
            g.c.deliver(0)
    assert ag.phase() == 'ENTERING'
    # 시도 기한 만료 → ABANDON (투표자에게 아직 안 감) → 새 시도의 REQUEST 는 busy 를 받는다
    g.c.advance('a', 6.0)
    g.c.tick('a')
    g.c.advance('a', 1.0)
    g.c.tick('a')                                  # 백오프 뒤 재시도 (+ 옛 시도 RELEASE 재전송)
    old = [m for m in g.c.net if m.kind in ('ABANDON', 'RELEASE')]
    g.c.net = [m for m in g.c.net if m.kind not in ('ABANDON', 'RELEASE')]
    assert old and {m.req.seq for m in old} == {1}
    g.c.deliver_all()                              # 새 REQUEST 만 → 옛 확정 표 때문에 busy
    view = ag.attempt_view()
    assert view is not None and len(view['busy']) >= 3
    lk.poll()
    assert not lk.is_denied('t1')                  # 내 옛 확정 표 탓이라 거부로 보지 않는다
    g.c.send(old)                                  # 늦게 온 ABANDON/RELEASE 가 옛 표를 풀면
    g.c.deliver_all()
    lk.commit('t1')
    for _ in range(10):
        g.settle(rounds=1)
        if lk.is_held('t1'):
            break
    assert lk.is_held('t1')
    assert g.c.check() == []


def test_task_resource_name():
    assert task_resource('src.000001') == 'task:src.000001'


def test_busy_from_a_minority_of_voters_is_not_a_denial():
    """
    남의 확정 표가 소수 투표자에게만 남아 있을 때 (확정 단계 도중 포기한 요청의 ABANDON 이 아직 안 감) 거부로 보지 않는다.

    과반 불가(n − q + 1 = 3)가 아니면 기다리면 풀릴 수 있다 — 문턱을 1 로 낮추면 이 작업을 영영 놓친다.
    """
    g = _Rig(6)
    b, a = g.locks['b'], g.locks['a']
    b.try_acquire('t1', 'b', 0.0)
    b.commit('t1')
    g.pump()
    # b 의 ENTER 를 투표자 c 에게만 배달 → c 만 확정, 나머지 ENTER · 모든 ENTER_ACK 는 버린다
    while g.c.net:
        msg = g.c.net[0]
        if msg.kind == 'ENTER' and msg.dst != 'c' or msg.kind == 'ENTER_ACK':
            g.c.drop(0)
        else:
            g.c.deliver(0)
    assert 'task:t1' in [z for z, v in g.c.agents['c'].votes.items() if v.committed]
    # b 는 포기 (시도 기한) — ABANDON 은 붙잡아 둔다
    g.c.advance('b', 6.0)
    g.c.tick('b')
    held_back = [m for m in g.c.net if m.src == 'b']
    g.c.net = [m for m in g.c.net if m.src != 'b']
    g.locks['b'].agent.logical = None              # b 는 다시 시도하지 않는다
    for v in ('a', 'c', 'd', 'e'):                 # 다른 투표자의 b 잠정 표는 기한으로 풀린다 (c 의 확정 표는 남는다)
        g.c.advance(v, 11.0)
        g.c.tick(v)
    g.c.deliver_all()
    a.try_acquire('t1', 'a', 0.0)
    g.pump()
    g.c.deliver_all()
    view = g.c.agents['a'].attempt_view()
    assert view is not None and 1 <= len(view['busy']) < a.deny_threshold
    a.poll()
    assert not a.is_denied('t1')
    g.c.send(held_back)                            # 늦게 온 ABANDON 이 c 의 확정 표를 풀면
    a.commit('t1')
    for _ in range(10):
        g.settle(rounds=1)
        if a.is_held('t1'):
            break
    assert a.is_held('t1')
    assert g.c.check() == []
