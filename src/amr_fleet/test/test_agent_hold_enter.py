"""
투표 코어의 지연 확정 (hold_enter + commit, P3 작업 ID 잠금) 시험.

과반 VOTE 를 모은 뒤 VOTED 에서 멈추고 commit() 때만 ENTER 를 보내는지, 그리고 이 대기 단계가 끼어도
"같은 자원을 허가받은 로봇 ≤ 1" 과 "픽업 시작(허가)은 작업마다 평생 한 번" 이 깨지지 않는지 본다.
무작위 시나리오 시험은 hypothesis 없이 결정적 난수로 돈다 (CI 이미지에 hypothesis 가 없다 — f1 교훈).
"""

import os
import random

import pytest

from amr_fleet.distributed import messages as m
from amr_fleet.distributed.simnet import Cluster
from amr_fleet.distributed.vote_core import ENTERING, VOTED, VOTING, VoteConfig

MEMBERS = ('a', 'b', 'c', 'd', 'e')
CFG = VoteConfig(MEMBERS, tentative_s=10.0, retry_s=0.5, backoff_min_s=0.5, backoff_max_s=1.0)
TASK = 'task:t1'


def _hold(c, r, res=(TASK,), hold_s=None):
    c.send(c.agents[r].request(sorted(res), c.clock[r], hold_enter=True, hold_s=hold_s))


def _commit(c, r):
    c.send(c.agents[r].commit(c.clock[r]))


def _committed_votes(c, res):
    return sum(1 for v in c.members if res in c.committed()[v])


def test_hold_enter_stops_at_voted_without_enter():
    c = Cluster(CFG, seed=1)
    _hold(c, 'a')
    sent = []
    while c.net:
        sent.append(c.net[0])
        c.deliver(0)
    a = c.agents['a']
    assert a.phase() == VOTED
    assert not a.may_enter(TASK)
    assert not any(x.kind == m.ENTER for x in sent)
    assert _committed_votes(c, TASK) == 0               # 잠정 표뿐
    assert len(a.attempt_view()['votes']) >= CFG.quorum
    assert c.check() == []
    # VOTED 에서는 재전송하지 않는다 (과반 표를 쥐고 기다린다)
    c.advance_all(1.0)
    c.tick('a')
    assert not any(x.src == 'a' and x.kind == m.REQUEST for x in c.net)


def test_commit_from_voted_reaches_held_with_quorum_of_committed_votes():
    c = Cluster(CFG, seed=2)
    _hold(c, 'a')
    c.deliver_all()
    _commit(c, 'a')
    assert c.agents['a'].phase() == ENTERING
    assert c.stores['a'].load()['entering'] is not None   # ENTER 보내기 전에 ENTERING 영속화
    c.deliver_all()
    assert c.agents['a'].may_enter(TASK)
    assert _committed_votes(c, TASK) >= CFG.quorum
    assert c.check() == []


def test_commit_before_quorum_goes_straight_to_enter():
    c = Cluster(CFG, seed=3)
    _hold(c, 'a')
    _commit(c, 'a')                                      # 아직 VOTING
    assert c.agents['a'].phase() == VOTING
    c.deliver_all()
    assert c.agents['a'].may_enter(TASK)


def test_competitor_cannot_commit_while_other_holds_tentative_quorum():
    c = Cluster(CFG, seed=4)
    _hold(c, 'a')
    c.deliver_all()
    _hold(c, 'b')
    _commit(c, 'b')
    c.deliver_all()
    assert not c.agents['b'].may_enter(TASK)             # a 의 잠정 표가 과반을 막는다
    _commit(c, 'a')
    c.settle(rounds=10)
    assert c.agents['a'].may_enter(TASK)
    assert not c.agents['b'].may_enter(TASK)
    view = c.agents['b'].attempt_view()
    assert view is None or len(view['busy']) + len(view['contended']) >= 1
    assert c.check() == []


def test_expired_hold_lets_other_commit_and_late_commit_fails():
    """잠정 표가 기한으로 풀리면 다른 로봇이 확정할 수 있다 — 늦게 commit 한 로봇은 허가를 못 받는다."""
    c = Cluster(CFG, seed=5)
    _hold(c, 'a', hold_s=2.0)
    c.deliver_all()
    assert c.agents['a'].phase() == VOTED
    c.advance_all(2.5)
    c.tick_all()                                         # 투표자 만료 + a 시도 기한 → ABANDON
    c.deliver_all()
    _hold(c, 'b')
    _commit(c, 'b')
    c.deliver_all()
    assert c.agents['b'].may_enter(TASK)
    _commit(c, 'a')                                      # a 는 백오프 뒤 재시도하며 확정을 원한다
    max_busy = 0
    for _ in range(20):
        c.settle(rounds=1)
        view = c.agents['a'].attempt_view()
        if view is not None:
            max_busy = max(max_busy, len(view['busy']))
    assert not c.agents['a'].may_enter(TASK)
    # 확정 표가 막는다는 NACK(busy) 를 과반 불가 수만큼 받는다 (어댑터의 DENIED 근거)
    assert max_busy >= len(MEMBERS) - CFG.quorum + 1
    assert c.check() == []


def test_hold_s_must_be_positive():
    c = Cluster(CFG, seed=6)
    with pytest.raises(ValueError):
        c.agents['a'].request([TASK], 0.0, hold_enter=True, hold_s=0.0)


def test_plain_request_still_enters_without_commit():
    c = Cluster(CFG, seed=7)
    c.request('a', [TASK])
    c.deliver_all()
    assert c.agents['a'].may_enter(TASK)


# ---------------------------------------------------------------------- 무작위 시나리오
N_SCENARIOS = int(os.environ.get('AMR_AGENT_HOLD_SCENARIOS', '150'))
N_STEPS = int(os.environ.get('AMR_AGENT_HOLD_STEPS', '120'))


def _scenario(seed):
    """
    여러 로봇이 같은 작업 몇 개를 hold_enter 로 잡고 아무 때나 commit · 포기 · 크래시 · 재기동한다.

    픽업 시작 = 그 로봇이 task 를 허가받은 첫 순간. 작업 잠금은 완료 뒤에도 풀지 않으므로 픽업 시작은 작업마다
    평생 한 로봇이어야 한다. 매 단계 Cluster.check() (허가 배타 · 과반 확정 · 낮은 epoch) 도 본다.
    """
    rng = random.Random(seed)
    tasks = ['task:t1', 'task:t2']
    cfg = VoteConfig(MEMBERS, tentative_s=rng.choice([2.0, 5.0, 10.0]), retry_s=0.5,
                     backoff_min_s=0.2, backoff_max_s=1.0)
    c = Cluster(cfg, seed=seed)
    started = {t: set() for t in tasks}
    contested = 0           # 다른 로봇이 이미 픽업을 시작한 작업에 commit 한 횟수 (위험한 경우)
    for _ in range(N_STEPS):
        r = rng.choice(MEMBERS)
        a = c.agents.get(r)
        op = rng.random()
        if a is None:
            if op < 0.3:
                # 재기동: 허가를 받은 작업은 곧바로 픽업을 시작한 것으로 세어 inside 에 넣었으므로
                # boot(inside) 가 그 잠금을 놓지 않는다 (작업 잠금은 완료 뒤에도 풀지 않는다)
                c.restart(r)
        elif op < 0.15:
            t = rng.choice(tasks)
            if t not in a.authorized():
                c.send(a.request([t], c.clock[r], hold_enter=True,
                                 hold_s=rng.choice([None, 1.0, 4.0])))
        elif op < 0.35:
            if any(started[t] - {r} for t in a.requested()):
                contested += 1
            c.send(a.commit(c.clock[r]))
        elif op < 0.38:
            c.send(a.cancel(c.clock[r]))
        elif op < 0.40:
            c.crash(r)
        elif op < 0.75:
            c.chaos(rng, loss=0.2, dup=0.1, hold=0.2, max_events=40)
        elif op < 0.9:
            c.advance(r, rng.choice([0.1, 0.5, 1.5]))
            c.tick(r)
        else:
            c.advance_all(0.5)
            c.tick_all()
        for rr in MEMBERS:
            ag = c.agents.get(rr)
            if ag is None:
                continue
            for t in tasks:
                if ag.may_enter(t):
                    started[t].add(rr)
                    c.inside[rr].add(t)            # 픽업 시작 = 몸체 진입과 같게 센다 (I2·I3)
        bad = c.check()
        dup = {t: sorted(rs) for t, rs in started.items() if len(rs) > 1}
        if bad or dup:
            return bad, dup, c.trace[-40:], 0, contested
    return [], {}, [], sum(len(rs) for rs in started.values()), contested


@pytest.mark.parametrize('block', range(5))
def test_random_hold_commit_scenarios_never_double_start(block):
    per = max(1, N_SCENARIOS // 5)
    starts = contested = 0
    for seed in range(block * per, (block + 1) * per):
        bad, dup, trace, n, k = _scenario(seed)
        assert not bad and not dup, f'seed {seed}: {bad} {dup}\n' + '\n'.join(trace)
        starts += n
        contested += k
    # 공허 검사: 픽업 시작이 실제로 일어났고 (허가가 막혀서 통과한 것이 아니다), 이미 시작된 작업에
    # 다른 로봇이 commit 하는 위험한 경우도 실제로 나왔다
    assert starts >= per // 2, f'픽업 시작 {starts} 건 / 시나리오 {per}'
    assert contested >= 1, f'경합 commit {contested} 건'


def test_random_scenarios_actually_start_pickups():
    """공허 검사: 같은 생성기에서 픽업 시작이 실제로 여러 번 나온다."""
    starts = 0
    for seed in range(30):
        rng = random.Random(seed)
        c = Cluster(VoteConfig(MEMBERS, tentative_s=5.0, retry_s=0.5, backoff_min_s=0.2,
                               backoff_max_s=1.0), seed=seed)
        r = rng.choice(MEMBERS)
        c.send(c.agents[r].request(['task:t1'], 0.0, hold_enter=True))
        c.chaos(rng, loss=0.2, dup=0.1, hold=0.2)
        c.send(c.agents[r].commit(c.clock[r]))
        c.settle(rounds=10)
        starts += int(c.agents[r].may_enter('task:t1'))
    assert starts >= 20
