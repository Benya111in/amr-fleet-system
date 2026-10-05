"""negotiation.agent: 입찰·병합·패배·안정·국소 확정·하트비트·건강 게이트·작업 잠금."""

import pytest

from amr_fleet.negotiation.agent import AgentConfig, CbaaAgent
from amr_fleet.negotiation.bids import Bid, LOCKED_COST, NO_BID
from amr_fleet.negotiation.task_lock import ExclusiveTaskLock, LockResult, LockStatus, TaskLock

IDS = ['a', 'b', 'c']


def make(costs_by_robot, tasks=('t1', 't2'), lock=None, **cfg):
    config = AgentConfig(**cfg)
    return {rid: CbaaAgent(rid, list(costs_by_robot), list(tasks), costs, config, lock)
            for rid, costs in costs_by_robot.items()}


def full_round(agents, now, drop=()):
    """모든 로봇이 서로 받는 한 라운드. drop 의 (보낸, 받는) 링크는 유실."""
    for ag in agents.values():
        ag.start_round(now)
    msgs = {rid: ag.make_message(now) for rid, ag in agents.items()}
    for k, msg in msgs.items():
        for i, ag in agents.items():
            if i != k and (k, i) not in drop:
                ag.receive(msg, now)
    return {rid: ag.finish_round(now) for rid, ag in agents.items()}


def test_config_validation():
    with pytest.raises(ValueError):
        AgentConfig(stable_rounds=0).validate()
    with pytest.raises(ValueError):
        AgentConfig(heartbeat_timeout=0).validate()
    with pytest.raises(ValueError):
        AgentConfig(liveness='gossip').validate()
    with pytest.raises(KeyError):
        AgentConfig(rule='nope').validate()


def test_bid_picks_cheapest_winnable_task_and_tie_by_order():
    ag = CbaaAgent('a', IDS, ['t1', 't2', 't3'], {'t1': 5.0, 't2': 3.0, 't3': 3.0})
    assert ag.start_round(1) == 't2'                      # 같은 비용 3.0 → 후보 순서 앞
    assert ag.table['t2'] == Bid(3.0, 'a')
    assert ag.start_round(2) is None                      # L_t = 1: 쥔 동안 다시 입찰하지 않는다


def test_bid_skips_unwinnable_and_nonfinite():
    ag = CbaaAgent('b', IDS, ['t1', 't2'], {'t1': 1.0, 't2': float('inf')})
    ag.table['t1'] = Bid(0.5, 'a')                        # 이미 더 나은 입찰
    assert ag.start_round(1) is None


@pytest.mark.parametrize('rule', ['max', 'cbba'])
def test_three_robots_converge_to_greedy_and_commit(rule):
    # 탐욕: (b,t1)=1 → (a,t2)=2 → c 는 남는다
    agents = make({'a': {'t1': 1.5, 't2': 2.0}, 'b': {'t1': 1.0, 't2': 4.0},
                   'c': {'t1': 3.0, 't2': 3.0}}, rule=rule, stable_rounds=2)
    started = {}
    for now in range(1, 12):
        for rid, t in full_round(agents, now).items():
            if t:
                started[rid] = t
    assert started == {'a': 't2', 'b': 't1'}
    assert agents['c'].assigned is None and agents['c'].executing is None
    # 실행 시작 작업은 LOCKED 로 방송된다
    assert agents['c'].table['t1'] == Bid(LOCKED_COST, 'b')
    assert agents['c'].table['t2'] == Bid(LOCKED_COST, 'a')


def test_loser_drops_and_rebids_next_round():
    agents = make({'a': {'t1': 1.0, 't2': 5.0}, 'b': {'t1': 2.0, 't2': 6.0}},
                  stable_rounds=5)
    full_round(agents, 1)                                 # 둘 다 t1 입찰 → b 가 진다
    assert agents['b'].assigned is None
    agents['b'].start_round(2)
    assert agents['b'].assigned == 't2'


def test_commit_needs_k_stable_rounds():
    agents = make({'a': {'t1': 1.0}, 'b': {'t1': 2.0}}, tasks=('t1',), stable_rounds=3)
    commits = [full_round(agents, now)['a'] for now in range(1, 6)]
    # 라운드 1 에 a 가 입찰(변화) → 2·3·4 안정 → 4 번째 라운드에 확정
    assert commits == [None, None, None, 't1', None]


def test_require_all_heard_blocks_commit_until_everyone_heard_directly():
    agents = make({'a': {'t1': 1.0}, 'b': {'t1': 2.0}, 'c': {'t1': 3.0}}, tasks=('t1',),
                  stable_rounds=1, heartbeat_timeout=None)
    full_round(agents, 1)
    out = full_round(agents, 2, drop={('c', 'a')})        # a 가 c 에게서 못 받았다
    assert out['a'] is None
    assert full_round(agents, 3)['a'] == 't1'


def test_without_require_all_heard_commits_on_stability_only():
    agents = make({'a': {'t1': 1.0}, 'b': {'t1': 2.0}}, tasks=('t1',), stable_rounds=1,
                  require_all_heard=False)
    full_round(agents, 1)
    assert full_round(agents, 2, drop={('b', 'a')})['a'] == 't1'


@pytest.mark.parametrize('rule', ['max', 'cbba'])
def test_heartbeat_invalidates_silent_holder(rule):
    agents = make({'a': {'t1': 1.0}, 'b': {'t1': 2.0}}, tasks=('t1',), rule=rule,
                  stable_rounds=10, heartbeat_timeout=2)
    full_round(agents, 1)
    assert agents['b'].table['t1'].winner == 'a'
    b = agents['b']
    for now in (2, 3, 4):                                  # a 침묵 (이탈)
        b.start_round(now)
        b.finish_round(now)
    assert not b.is_alive('a', 4)
    assert b.table['t1'] == NO_BID
    assert b.start_round(5) == 't1'                        # 재입찰


def test_heartbeat_none_keeps_dead_holder_forever():
    agents = make({'a': {'t1': 1.0}, 'b': {'t1': 2.0}}, tasks=('t1',), heartbeat_timeout=None)
    full_round(agents, 1)
    b = agents['b']
    for now in range(2, 50):
        b.start_round(now)
        b.finish_round(now)
    assert b.table['t1'].winner == 'a'                    # 고착 (연구 계산의 300/300 고착과 같은 원인)


def test_indirect_liveness_uses_relayed_info_time():
    agents = make({'a': {'t1': 1.0}, 'b': {'t1': 2.0}, 'c': {'t1': 3.0}}, tasks=('t1',),
                  stable_rounds=10, heartbeat_timeout=2, liveness='indirect')
    full_round(agents, 1)
    for now in (2, 3, 4, 5):                               # a→b 링크만 끊김, a→c→b 로 소식 전달
        full_round(agents, now, drop={('a', 'b')})
    assert agents['b'].is_alive('a', 5)
    assert agents['b'].table['t1'].winner == 'a'
    direct = make({'a': {'t1': 1.0}, 'b': {'t1': 2.0}, 'c': {'t1': 3.0}}, tasks=('t1',),
                  stable_rounds=10, heartbeat_timeout=2, liveness='direct')
    full_round(direct, 1)
    for now in (2, 3, 4, 5):
        full_round(direct, now, drop={('a', 'b')})
    assert not direct['b'].is_alive('a', 5)               # 직접 기준은 오탐


def test_health_gate_withdraws_and_cbba_propagates_withdrawal():
    agents = make({'a': {'t1': 1.0}, 'b': {'t1': 2.0}}, tasks=('t1',), rule='cbba',
                  stable_rounds=10)
    full_round(agents, 1)
    agents['a'].set_localization_healthy(False, 1.5)
    assert agents['a'].assigned is None and agents['a'].table['t1'] == NO_BID
    assert agents['a'].start_round(2) is None             # 건강이 나쁘면 입찰하지 않는다
    msg = agents['a'].make_message(2)
    assert not msg.healthy
    agents['b'].receive(msg, 2)
    agents['b'].finish_round(2)
    assert agents['b'].table['t1'] == NO_BID               # 철회 전파 + 건강 무효화
    assert agents['b'].start_round(3) == 't1'


def test_health_gate_keeps_executing_task():
    agents = make({'a': {'t1': 1.0}}, tasks=('t1',), stable_rounds=1, heartbeat_timeout=None)
    a = agents['a']
    a.start_round(1)
    a.finish_round(1)
    assert a.start_round(2) is None
    assert a.finish_round(2) == 't1'
    a.set_localization_healthy(False, 3)
    assert a.executing == 't1'


def test_locked_entry_is_sticky_under_both_rules():
    for rule in ('max', 'cbba'):
        a = CbaaAgent('a', IDS, ['t1'], {'t1': 1.0}, AgentConfig(rule=rule))
        a.table['t1'] = Bid(LOCKED_COST, 'c')
        msg_b = CbaaAgent('b', IDS, ['t1'], {}).make_message(5)       # b 는 승자 없음, 시각 최신
        a.receive(msg_b, 5)
        assert a.table['t1'] == Bid(LOCKED_COST, 'c')
        # 잠금은 늘 받아들인다 (s 비교 없이)
        a2 = CbaaAgent('a', IDS, ['t1'], {'t1': 1.0}, AgentConfig(rule=rule))
        a2.start_round(1)
        b = CbaaAgent('b', IDS, ['t1'], {}, AgentConfig(rule=rule))
        b.table['t1'] = Bid(LOCKED_COST, 'c')
        a2.receive(b.make_message(0), 1)
        assert a2.table['t1'] == Bid(LOCKED_COST, 'c')


def test_exclusive_lock_denial_marks_holder_and_rebids_elsewhere():
    lock = ExclusiveTaskLock()
    assert lock.try_acquire('t1', 'z', 0).status is LockStatus.GRANTED
    a = CbaaAgent('a', ['a'], ['t1', 't2'], {'t1': 1.0, 't2': 2.0},
                  AgentConfig(stable_rounds=1, heartbeat_timeout=None), lock)
    a.start_round(1)
    a.finish_round(1)
    a.start_round(2)
    assert a.finish_round(2) is None                      # 거절
    assert a.table['t1'] == Bid(LOCKED_COST, 'z') and a.assigned is None
    assert a.n_lock_denied == 1
    assert a.start_round(3) == 't2'


class _DenyUnknown(TaskLock):
    def try_acquire(self, task_id, robot_id, now):
        return LockResult(LockStatus.DENIED, None)

    def release(self, task_id, robot_id, now):
        return None


def test_denial_without_holder_blocks_task():
    a = CbaaAgent('a', ['a'], ['t1', 't2'], {'t1': 1.0, 't2': 2.0},
                  AgentConfig(stable_rounds=1, heartbeat_timeout=None), _DenyUnknown())
    a.start_round(1)
    a.finish_round(1)
    a.start_round(2)
    a.finish_round(2)
    assert 't1' in a.blocked
    assert a.start_round(3) == 't2'


def test_pending_lock_waits_then_grants():
    lock = ExclusiveTaskLock(grant_delay=2)
    a = CbaaAgent('a', ['a'], ['t1'], {'t1': 1.0},
                  AgentConfig(stable_rounds=1, heartbeat_timeout=None), lock)
    a.start_round(1)
    a.finish_round(1)
    out = []
    for now in (2, 3, 4):
        a.start_round(now)
        out.append(a.finish_round(now))
    assert out == [None, None, 't1']
    assert lock.holder('t1') == 'a'


def test_losing_while_pending_releases_request():
    lock = ExclusiveTaskLock(grant_delay=10)
    a = CbaaAgent('a', ['a', 'b'], ['t1'], {'t1': 2.0},
                  AgentConfig(stable_rounds=1, heartbeat_timeout=None,
                              require_all_heard=False), lock)
    a.start_round(1)
    a.finish_round(1)
    a.start_round(2)
    a.finish_round(2)
    assert a.pending_lock == 't1'
    b = CbaaAgent('b', ['a', 'b'], ['t1'], {'t1': 1.0})
    b.start_round(3)
    a.start_round(3)
    a.receive(b.make_message(3), 3)
    a.finish_round(3)
    assert a.assigned is None and a.pending_lock is None
    assert ('t1', 'a') not in lock._pending


def test_set_tasks_drops_removed_and_adds_new():
    a = CbaaAgent('a', IDS, ['t1', 't2'], {'t1': 1.0, 't2': 2.0})
    a.start_round(1)
    assert a.assigned == 't1'
    a.set_tasks(['t2', 't3'])
    assert a.assigned is None and set(a.table) == {'t2', 't3'}
    assert a.table['t3'] == NO_BID
    a.set_costs({'t2': 2.0, 't3': 0.5})
    assert a.start_round(2) == 't3'


def test_message_carries_full_state():
    a = CbaaAgent('a', IDS, ['t1'], {'t1': 1.0})
    a.start_round(7)
    msg = a.make_message(7)
    assert msg.sender == 'a' and msg.stamp == 7 and msg.assigned == 't1'
    assert msg.info_time['a'] == 7 and set(msg.info_time) == set(IDS)
    assert msg.bids['t1'] == Bid(1.0, 'a') and msg.healthy and not msg.executing
    a.receive(msg, 7)                                     # 자기 메시지는 무시
    assert a.n_received == 0


def test_unknown_sender_becomes_peer():
    a = CbaaAgent('a', ['a'], ['t1'], {'t1': 2.0})
    z = CbaaAgent('z', ['z'], ['t1'], {'t1': 1.0})
    z.start_round(1)
    a.receive(z.make_message(1), 1)
    assert 'z' in a.peers and a.table['t1'] == Bid(1.0, 'z')
