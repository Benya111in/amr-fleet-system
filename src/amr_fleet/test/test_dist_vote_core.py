"""투표 코어 단위 시험 — 전이 하나하나 (protocol.md 5 절)."""

import pytest

from amr_fleet.distributed import messages as m
from amr_fleet.distributed.messages import VoteMsg
from amr_fleet.distributed.ordering import PriorityKey, ReqId, Stamp
from amr_fleet.distributed.simnet import Cluster
from amr_fleet.distributed.store import MemoryStore
from amr_fleet.distributed.vote_core import ENTERING, VOTING, VoteConfig, ZoneAgent

MEMBERS = ('a', 'b', 'c', 'd', 'e')
CFG = VoteConfig(MEMBERS, tentative_s=10.0, retry_s=0.5, backoff_min_s=0.5, backoff_max_s=1.0)


def boot(r='a', cfg=CFG, store=None, now=0.0, inside=()):
    store = store if store is not None else MemoryStore()
    agent, out = ZoneAgent.boot(r, cfg, store, now, inside=inside)
    return agent, store, out


def req_msg(src, dst, req, res, deadline=10.0, epoch=0, prio=0, since=1, starved=False):
    return VoteMsg(m.REQUEST, src, dst, epoch, req, frozenset(res),
                   PriorityKey(prio, None, since, starved), Stamp.coerce(deadline), since)


def kinds(msgs, kind=None):
    return [x for x in msgs if kind is None or x.kind == kind]


R1 = ReqId('b', 1, 1)
R2 = ReqId('c', 1, 1)


# ---------------------------------------------------------------------- 설정
def test_config_requires_static_majority():
    assert CFG.quorum == 3
    with pytest.raises(ValueError):
        VoteConfig(MEMBERS, quorum=2)           # 2 + 2 ≤ 5: 두 정족수가 안 겹친다
    with pytest.raises(ValueError):
        VoteConfig(('a', 'a', 'b'))
    assert VoteConfig(('a', 'b', 'c', 'd')).quorum == 3


# ---------------------------------------------------------------------- 투표자
def test_voter_grants_exclusive_tentative_vote():
    v, store, _ = boot('a')
    out = v.handle(req_msg('b', 'a', R1, ['z']), 1.0)
    assert [x.kind for x in out] == [m.VOTE]
    assert store.load()['votes']['z']['req'] == R1.to_list()      # 응답 전에 영속화됐다
    out = v.handle(req_msg('c', 'a', R2, ['z']), 1.0)
    assert [(x.kind, x.reason) for x in out] == [(m.NACK, m.R_CONTENDED)]
    assert v.votes['z'].req == R1


def test_voter_chain_is_all_or_nothing():
    v, _, _ = boot('a')
    v.handle(req_msg('b', 'a', R1, ['z1']), 1.0)
    out = v.handle(req_msg('c', 'a', R2, ['z0', 'z1']), 1.0)
    assert kinds(out, m.VOTE) == []
    assert 'z0' not in v.votes                                 # z0 만 따로 주지 않는다


def test_tentative_expires_on_voter_clock_then_next_pending_gets_vote():
    v, _, _ = boot('a')
    v.handle(req_msg('b', 'a', R1, ['z'], deadline=5.0), 1.0)
    v.handle(req_msg('c', 'a', R2, ['z'], deadline=50.0), 1.0)   # 대기열
    assert v.tick(4.9) == []
    out = v.tick(5.0)
    assert [(x.kind, x.req) for x in out] == [(m.VOTE, R2)]
    assert v.votes['z'].req == R2


def test_committed_vote_never_expires_and_blocks_others():
    v, _, _ = boot('a')
    v.handle(req_msg('b', 'a', R1, ['z'], deadline=5.0), 1.0)
    out = v.handle(VoteMsg(m.ENTER, 'b', 'a', 0, R1, frozenset({'z'})), 2.0)
    assert [x.kind for x in out] == [m.ENTER_ACK]
    v.tick(10_000.0)
    assert v.votes['z'].committed and v.votes['z'].req == R1
    out = v.handle(req_msg('c', 'a', R2, ['z'], deadline=20_000.0), 10_000.0)
    assert [(x.kind, x.reason) for x in out] == [(m.NACK, m.R_BUSY)]


def test_enter_after_expiry_is_nacked():
    v, _, _ = boot('a')
    v.handle(req_msg('b', 'a', R1, ['z'], deadline=5.0), 1.0)
    out = v.handle(VoteMsg(m.ENTER, 'b', 'a', 0, R1, frozenset({'z'})), 6.0)
    assert [(x.kind, x.reason) for x in out] == [(m.NACK, m.R_NOT_TENTATIVE)]
    assert 'z' not in v.votes


def test_release_frees_committed_and_tombstones():
    v, _, _ = boot('a')
    v.handle(req_msg('b', 'a', R1, ['z0', 'z1']), 1.0)
    v.handle(VoteMsg(m.ENTER, 'b', 'a', 0, R1, frozenset({'z0', 'z1'})), 1.0)
    out = v.handle(VoteMsg(m.RELEASE, 'b', 'a', 0, R1, frozenset({'z0'})), 2.0)   # 부분 해제
    assert [x.kind for x in out] == [m.RELEASE_ACK]
    assert 'z0' not in v.votes and v.votes['z1'].committed
    assert R1 in v.tombstones
    # 늦게 온 옛 REQUEST · ENTER 는 묘비로 거절 (좀비 표 방지)
    out = v.handle(req_msg('b', 'a', R1, ['z0', 'z1'], deadline=9.0), 3.0)
    assert [(x.kind, x.reason) for x in out] == [(m.NACK, m.R_FINISHED)]
    out = v.handle(VoteMsg(m.ENTER, 'b', 'a', 0, R1, frozenset({'z0', 'z1'})), 3.0)
    assert [(x.kind, x.reason) for x in out] == [(m.NACK, m.R_FINISHED)]


def test_release_of_other_request_does_not_free_vote():
    v, _, _ = boot('a')
    v.handle(req_msg('b', 'a', R1, ['z']), 1.0)
    v.handle(VoteMsg(m.ENTER, 'b', 'a', 0, R1, frozenset({'z'})), 1.0)
    v.handle(VoteMsg(m.RELEASE, 'c', 'a', 0, R2, frozenset({'z'})), 2.0)
    assert v.votes['z'].req == R1


def test_pending_queue_follows_priority_key_on_release():
    v, _, _ = boot('a')
    v.handle(req_msg('b', 'a', R1, ['z'], deadline=100.0), 1.0)
    v.handle(VoteMsg(m.ENTER, 'b', 'a', 0, R1, frozenset({'z'})), 1.0)
    low = ReqId('c', 1, 1)
    high = ReqId('d', 1, 1)
    v.handle(req_msg('c', 'a', low, ['z'], deadline=100.0, prio=0, since=2), 2.0)
    v.handle(req_msg('d', 'a', high, ['z'], deadline=100.0, prio=5, since=3), 2.0)
    out = v.handle(VoteMsg(m.RELEASE, 'b', 'a', 0, R1, frozenset({'z'})), 3.0)
    assert [(x.kind, x.req) for x in out if x.kind == m.VOTE] == [(m.VOTE, high)]


def test_starved_request_reserves_resource():
    v, _, _ = boot('a')
    v.handle(req_msg('b', 'a', R1, ['z1'], deadline=100.0), 1.0)
    starved = ReqId('c', 1, 9)
    v.handle(req_msg('c', 'a', starved, ['z0', 'z1'], deadline=100.0, since=1, starved=True),
             1.0)
    other = ReqId('d', 1, 1)
    out = v.handle(req_msg('d', 'a', other, ['z0'], deadline=100.0, prio=9, since=5), 1.0)
    assert kinds(out, m.VOTE) == []                     # z0 는 기아 요청에 예약
    out = v.handle(VoteMsg(m.ABANDON, 'b', 'a', 0, R1, frozenset({'z1'})), 2.0)
    assert [(x.kind, x.req) for x in out if x.kind == m.VOTE] == [(m.VOTE, starved)]


def test_newer_request_of_same_robot_supersedes_old_tentative():
    v, _, _ = boot('a')
    old, new = ReqId('b', 1, 1), ReqId('b', 1, 2)
    v.handle(req_msg('b', 'a', old, ['z']), 1.0)
    out = v.handle(req_msg('b', 'a', new, ['z']), 1.5)
    assert [(x.kind, x.req) for x in out] == [(m.VOTE, new)]
    assert old in v.tombstones


def test_expired_request_deadline_is_nacked():
    v, _, _ = boot('a')
    out = v.handle(req_msg('b', 'a', R1, ['z'], deadline=1.0), 2.0)
    assert [(x.kind, x.reason) for x in out] == [(m.NACK, m.R_EXPIRED)]


# ---------------------------------------------------------------------- epoch
def test_higher_epoch_drops_tentative_keeps_committed():
    v, _, _ = boot('a')
    v.handle(req_msg('b', 'a', R1, ['z0']), 1.0)
    v.handle(VoteMsg(m.ENTER, 'b', 'a', 0, R1, frozenset({'z0'})), 1.0)
    v.handle(req_msg('c', 'a', R2, ['z1']), 1.0)
    assert set(v.votes) == {'z0', 'z1'}
    v.handle(VoteMsg(m.RELEASE_ACK, 'd', 'a', 3, ReqId('a', 1, 99)), 2.0)   # epoch 3 을 본다
    assert v.max_epoch == 3
    assert set(v.votes) == {'z0'} and v.votes['z0'].committed


def test_lower_epoch_message_changes_nothing():
    v, _, _ = boot('a')
    v.handle(req_msg('b', 'a', R1, ['z'], epoch=2), 1.0)
    v.handle(VoteMsg(m.ENTER, 'b', 'a', 2, R1, frozenset({'z'})), 1.0)
    before = v.protocol_state()
    for msg in (req_msg('c', 'a', R2, ['z'], epoch=1),
                VoteMsg(m.RELEASE, 'b', 'a', 1, R1, frozenset({'z'})),
                VoteMsg(m.ENTER, 'b', 'a', 1, R1, frozenset({'z'}))):
        out = v.handle(msg, 2.0)
        assert [(x.kind, x.reason, x.epoch) for x in out] == [(m.NACK, m.R_STALE_EPOCH, 2)]
        assert v.protocol_state() == before
    assert v.stats['rejected_low_epoch'] == 3


def test_stale_epoch_release_is_retried_after_adoption():
    c = Cluster(CFG)
    c.request('a', ['z'])
    c.settle(3)
    assert c.agents['a'].authorized() == {'z'}
    c.reset_clock('b', 0.0)              # b 는 epoch 1 로 간다
    c.deliver_all()
    c.enter('a')
    c.exit('a', 'z')                     # a 의 RELEASE(epoch 0) 는 b 에서 거절 → a 가 따라와 재전송
    c.settle(6)
    assert all('z' not in c.agents[r].votes for r in MEMBERS)
    assert c.agents['a'].max_epoch == 1
    assert c.check() == []


# ---------------------------------------------------------------------- 요청자
def _drive_votes(agent, voters, now=1.0):
    """요청자 agent 에 voters 의 VOTE 를 넣는다 → 나온 ENTER."""
    a = agent.attempt
    out = []
    for v in voters:
        out += agent.handle(VoteMsg(m.VOTE, v, agent.me, 0, a.req, a.resources), now)
    return out


def test_requester_needs_quorum_enter_acks():
    r, store, _ = boot('a')
    r.request(['z'], 0.0)
    assert r.attempt.phase == VOTING
    enters = _drive_votes(r, ['a', 'b'])
    assert enters == [] and r.attempt.phase == VOTING
    enters = _drive_votes(r, ['c'])
    assert sorted(x.dst for x in enters if x.kind == m.ENTER) == ['a', 'b', 'c']
    assert r.attempt.phase == ENTERING
    assert store.load()['entering'] is not None             # ENTERING 영속 뒤에 ENTER
    req = r.attempt.req
    for v in ('a', 'b'):
        r.handle(VoteMsg(m.ENTER_ACK, v, 'a', 0, req, frozenset({'z'})), 1.0)
    assert not r.may_enter('z')                              # 2/5 로는 안 된다
    r.handle(VoteMsg(m.ENTER_ACK, 'b', 'a', 0, req, frozenset({'z'})), 1.0)   # 중복
    assert not r.may_enter('z')
    r.handle(VoteMsg(m.ENTER_ACK, 'c', 'a', 0, req, frozenset({'z'})), 1.0)
    assert r.may_enter('z')
    assert ZoneAgent.held_in_state(store.load()) == {req: {'z'}}


def test_requester_ignores_ack_for_other_request_and_cleans_up():
    r, _, _ = boot('a')
    r.request(['z'], 0.0)
    _drive_votes(r, ['a', 'b', 'c'])
    old = ReqId('a', r.incarnation, r.attempt.req.seq + 7)
    out = []
    for v in ('a', 'b', 'c'):
        out += r.handle(VoteMsg(m.ENTER_ACK, v, 'a', 0, old, frozenset({'z'})), 1.0)
    assert not r.may_enter('z')
    assert sorted(x.dst for x in out if x.kind == m.RELEASE) == ['a', 'b', 'c']


def test_requester_rejects_ack_with_different_chain():
    r, _, _ = boot('a')
    r.request(['z0', 'z1'], 0.0)
    _drive_votes(r, ['a', 'b', 'c'])
    req = r.attempt.req
    for v in ('a', 'b', 'c'):
        r.handle(VoteMsg(m.ENTER_ACK, v, 'a', 0, req, frozenset({'z0'})), 1.0)
    assert not r.may_enter('z0')


def test_requester_abandons_at_deadline_and_backs_off():
    r, _, _ = boot('a')
    r.request(['z'], 0.0)
    out = r.tick(10.0)
    assert sorted(x.dst for x in out if x.kind == m.ABANDON) == list(MEMBERS)
    assert r.attempt is None and r.logical.backoff_until > Stamp.coerce(10.0)
    assert r.stats['abandons'] == 1
    out = r.tick(100.0)
    assert len(kinds(out, m.REQUEST)) == 5 and r.attempt.req.seq == 2


def test_contended_split_abandons_early():
    r, _, _ = boot('a')
    r.request(['z'], 0.0)
    req = r.attempt.req
    out = []
    for v in ('c', 'd', 'e'):
        out += r.handle(VoteMsg(m.NACK, v, 'a', 0, req, frozenset({'z'}),
                                reason=m.R_CONTENDED), 1.0)
    assert len(kinds(out, m.ABANDON)) == 5 and r.attempt is None


def test_busy_does_not_abandon_early():
    r, _, _ = boot('a')
    r.request(['z'], 0.0)
    req = r.attempt.req
    for v in MEMBERS:
        r.handle(VoteMsg(m.NACK, v, 'a', 0, req, frozenset({'z'}), reason=m.R_BUSY), 1.0)
    assert r.attempt is not None and r.attempt.phase == VOTING


def test_starved_flag_after_max_bypass_attempts():
    r, _, _ = boot('a')
    r.request(['z'], 0.0)
    seen = []
    t = 0.0
    for _ in range(4):
        seen.append(r.attempt.key.starved)
        t += 10.0
        r.tick(t)          # 기한 → 포기
        t += 100.0
        r.tick(t)          # 백오프 끝 → 재시도
    assert seen == [False, False, True, True]
    assert r.attempt.key.since == r.logical.since


def test_request_skips_already_held_resources():
    c = Cluster(CFG)
    c.request('a', ['z0'])
    c.settle(3)
    c.request('a', ['z0', 'z1'])
    assert c.agents['a'].attempt.resources == frozenset({'z1'})


# ---------------------------------------------------------------------- 재시작
def test_restart_keeps_votes_and_bumps_incarnation():
    c = Cluster(CFG)
    c.request('a', ['z'])
    c.settle(3)
    inc = c.agents['b'].incarnation
    c.crash('b')
    c.restart('b')
    assert c.agents['b'].incarnation == inc + 1
    assert c.agents['b'].votes['z'].committed
    c.request('c', ['z'])
    c.settle(5)
    assert not c.agents['c'].authorized()
    assert c.check() == []


def test_restart_releases_held_resources_not_inside():
    c = Cluster(CFG)
    c.request('a', ['z0', 'z1'])
    c.settle(3)
    c.enter('a')
    c.inside['a'].discard('z1')          # 몸체는 z0 에만 (z1 은 들어가기 전)
    c.crash('a')
    c.restart('a')
    assert c.agents['a'].authorized() == {'z0'}
    c.settle(5)
    assert all('z1' not in c.agents[r].votes for r in MEMBERS)
    assert all(c.agents[r].votes['z0'].committed for r in MEMBERS)


def test_restart_during_entering_abandons_attempt():
    store = MemoryStore()
    r, _, _ = boot('a', store=store)
    r.request(['z'], 0.0)
    _drive_votes(r, ['a', 'b', 'c'])
    assert store.load()['entering'] is not None
    r2, _, out = boot('a', store=store, now=1.0)
    assert r2.incarnation == 2
    assert sorted(x.dst for x in out if x.kind == m.ABANDON) == list(MEMBERS)
    assert store.load()['entering'] is None and store.load()['incarnation'] == 2


# ---------------------------------------------------------------------- 멤버십 · 활성
def test_view_only_limits_targets_quorum_stays_static():
    c = Cluster(CFG)
    c.set_view('a', ['b'])                # 자기 + b = 2 < 3 → 절대 허가 못 받음
    c.request('a', ['z'])
    c.settle(30)
    assert not c.agents['a'].authorized()
    c.set_view('a', ['b', 'c'])
    c.settle(30)
    assert c.agents['a'].authorized() == {'z'}


def test_all_robots_eventually_served_on_lossless_network():
    c = Cluster(CFG, seed=3)
    for r in MEMBERS:
        c.request(r, ['z'])
    served = []
    for _ in range(400):
        c.settle(1)
        for r in MEMBERS:
            if c.enter(r):
                served.append(r)
                assert c.check() == []
                c.exit(r, 'z')
        if len(served) == len(MEMBERS):
            break
    assert sorted(served) == sorted(MEMBERS)
    assert c.check() == []


# ---------------------------------------------------------------------- 크래시 재시작 이중 투표
def _authorize_with_commits_at(c, robot, zone, committers):
    """요청자 robot 이 zone 을 허가받게 하되 ENTER 는 committers 에게만 닿게 한다 (나머지는 잠정)."""
    c.request(robot, [zone])
    guard = 0
    while c.net and guard < 1000:
        guard += 1
        msg = c.net[0]
        if msg.kind == m.ENTER and msg.dst not in committers:
            c.drop(0)
        else:
            c.deliver(0)
    assert c.agents[robot].authorized() == {zone}


def test_restarted_voter_does_not_double_vote():
    """
    재시작한 투표자는 확정 표를 되살려 이중 투표하지 않는다.

    a 가 z 를 {a,b,c} 의 확정 표로 쥔 채, d·e 의 잠정 표는 만료되고, c 가 재시작한다.

    c 가 확정 표를 디스크에서 되살리지 못하면 e 가 {c,d,e} 로 과반을 모아 함께 들어간다 (I1 위반).
    영속화 누락 · 응답 뒤 영속화 변이를 잡는 결정적 시나리오.
    """
    c = Cluster(CFG)
    _authorize_with_commits_at(c, 'a', 'z', {'a', 'b', 'c'})
    c.enter('a')
    for r in ('d', 'e'):
        c.advance(r, 20.0)
        c.tick(r)
    c.deliver_all()
    assert 'z' not in c.agents['d'].votes and 'z' not in c.agents['e'].votes
    c.crash('c')
    c.restart('c')
    c.request('e', ['z'])
    c.settle(10)
    assert not c.agents['e'].authorized()
    assert c.check() == []


def test_voter_crash_between_vote_and_enter_is_safe():
    """잠정 표를 준 투표자가 ENTER 전에 재시작해도 그 잠정 표를 기억한다 — 다른 요청자가 끼지 못한다."""
    c = Cluster(CFG)
    c.request('a', ['z'])
    for _ in range(5):                              # REQUEST 만 배달 (VOTE 는 아직 a 에 안 감)
        i = next(i for i, x in enumerate(c.net) if x.kind == m.REQUEST)
        c.deliver(i)
    for v in ('b', 'c'):
        c.crash(v)
        c.restart(v)
        assert c.agents[v].votes['z'].req.robot == 'a'
    c.request('e', ['z'])
    c.settle(3, dt=0.1)
    assert sum(1 for r in MEMBERS if c.agents[r].authorized()) == 1
    assert c.check() == []
