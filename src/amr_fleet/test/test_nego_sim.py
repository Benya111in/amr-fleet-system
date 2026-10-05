"""negotiation.sim: 오프라인 동기 라운드 시뮬레이터 — 합의 성질과 고장 주입."""

import random

import pytest

from amr_fleet.negotiation import scenario
from amr_fleet.negotiation.agent import AgentConfig
from amr_fleet.negotiation.channel import PerLinkChannel, SharedStreamChannel
from amr_fleet.negotiation.cost import EuclideanCost
from amr_fleet.negotiation.sim import assignment_indices, HOLDER, run_case, SimConfig
from amr_fleet.negotiation.task_lock import ExclusiveTaskLock

COST = EuclideanCost(1.0)


def case_and_cost(nt, seed, mode='scatter'):
    case = scenario.make_case(nt, random.Random(seed), mode)
    return case, COST.matrix(case.robots, case.tasks)


def run(case, cost, cfg, ch, sim=None, **kw):
    return run_case(cost, case.robot_ids, case.task_ids, cfg, ch, sim or SimConfig(), **kw)


def test_sim_config_validation():
    with pytest.raises(ValueError):
        SimConfig(termination='never').validate()
    with pytest.raises(ValueError):
        SimConfig(max_rounds=0).validate()


@pytest.mark.parametrize('rule', ['max', 'cbba'])
@pytest.mark.parametrize('nt', [3, 5, 8])
def test_perfect_comm_local_commit_equals_central_greedy(rule, nt):
    """완전 통신: 국소 확정(K=1, 전원 수신)으로도 CBAA = 중앙 탐욕 (CBBA 정리 1, L_t = 1)."""
    for k in range(40):
        case, cost = case_and_cost(nt, 1000 + k)
        res = run(case, cost, AgentConfig(rule=rule, stable_rounds=1),
                  PerLinkChannel(seed=k, p_loss=0.0))
        assert res.dup == 0 and res.unassigned == 0 and res.stuck == 0
        assert assignment_indices(res, case.robot_ids, case.task_ids) == \
            scenario.central_greedy(cost)
        # 수렴 상한: 표 확정까지 N_min·D = min(작업, 로봇) 라운드 (D = 1). 그 뒤 안정 확인 K=1,
        # 잠금 방송을 받은 빈 로봇의 표가 한 번 더 바뀌고 다시 K 라운드 안정해야 끝난다
        assert res.rounds <= min(nt, 5) + 3


@pytest.mark.parametrize('rule', ['max', 'cbba'])
def test_lossy_comm_global_oracle_still_equals_greedy(rule):
    """손실 40 % 에서도 전역 종료 오라클로 멈추면 할당은 중앙 탐욕과 같다 (연구 계산 300/300)."""
    for k in range(60):
        case, cost = case_and_cost(5, 2000 + k)
        cfg = AgentConfig(rule=rule, stable_rounds=2, local_commit=False,
                          announce_commits=False, heartbeat_timeout=None)
        res = run(case, cost, cfg, PerLinkChannel(seed=k, p_loss=0.4),
                  SimConfig(termination='global', max_rounds=200))
        assert assignment_indices(res, case.robot_ids, case.task_ids) == \
            scenario.central_greedy(cost)


def test_strict_lock_gives_zero_duplicates_where_optimistic_duplicates():
    """K=1·전원 수신 없음·손실 40 %: 낙관 모드는 중복이 나고 배타 잠금은 0 이다 (설계상 참)."""
    dup_opt = dup_strict = unassigned_strict = 0
    for k in range(150):
        case, cost = case_and_cost(3, 3000 + k)
        cfg = AgentConfig(stable_rounds=1, require_all_heard=False, heartbeat_timeout=None)
        opt = run(case, cost, cfg, PerLinkChannel(seed=k, p_loss=0.4))
        strict = run(case, cost, cfg, PerLinkChannel(seed=k, p_loss=0.4),
                     task_lock=ExclusiveTaskLock())
        dup_opt += opt.dup > 0
        dup_strict += strict.dup > 0
        unassigned_strict += strict.unassigned > 0
    assert dup_opt > 0
    assert dup_strict == 0
    assert unassigned_strict == 0          # 거절된 로봇은 다른 작업에 재입찰한다


@pytest.mark.parametrize('rule', ['max', 'cbba'])
def test_holder_dropout_heartbeat_prevents_stuck(rule):
    """작업을 쥔 로봇이 4 라운드째 이탈: H 없으면 고착, H=3 이면 다른 로봇이 이어받는다."""
    stuck_none = stuck_hb = 0
    for k in range(60):
        case, cost = case_and_cost(3, 99 + k)
        for hb in (None, 3):
            cfg = AgentConfig(rule=rule, stable_rounds=2, heartbeat_timeout=hb,
                              local_commit=False, announce_commits=False)
            res = run(case, cost, cfg, PerLinkChannel(seed=k, p_loss=0.0),
                      SimConfig(termination='fixed', max_rounds=30), dropouts={HOLDER: 4})
            if hb is None:
                stuck_none += res.stuck > 0
            else:
                stuck_hb += res.stuck > 0 or res.unassigned > 0
    assert stuck_none == 60
    assert stuck_hb == 0


def test_dropout_recovery_waits_for_heartbeat_then_reassigns():
    """보유 로봇 이탈: 이어받기는 H 가 지난 뒤에만 일어나고, 고정 지평선 안에서 끝난다."""
    recs = []
    for k in range(30):
        case, cost = case_and_cost(3, 4000 + k)
        cfg = AgentConfig(stable_rounds=3, heartbeat_timeout=5, liveness='indirect')
        res = run(case, cost, cfg, PerLinkChannel(seed=k), SimConfig(termination='fixed',
                                                                     max_rounds=40),
                  task_lock=ExclusiveTaskLock(), dropouts={HOLDER: 3})
        (victim, (rnd, task)), = res.dropped.items()
        assert rnd == 3 and task is not None
        rec = res.recovery_rounds(victim)
        assert rec is not None and rec > 5                   # H 전에는 아무도 가져가지 않는다
        assert res.stuck == 0 and res.unassigned == 0 and res.dup == 0
        recs.append(rec)
    assert max(recs) <= 5 + 1 + 3 + 3                        # H + 무효화 1 + 입찰·안정 K + 여유


def test_sick_idle_robot_is_not_counted_as_capable():
    case, cost = case_and_cost(5, 11)
    cfg = AgentConfig(stable_rounds=2)
    res = run(case, cost, cfg, PerLinkChannel(seed=0), task_lock=ExclusiveTaskLock(),
              health_events=[(1, 'amr_01', False)])
    assert res.unhealthy == ['amr_01'] and 'amr_01' not in res.commit_round
    assert res.unassigned == 0 and len(res.holders) == 4    # 4 대가 5 건 중 4 건


def test_dropout_right_after_commit_needs_strict_lock():
    """
    확정 직후(잠금 방송 전) 이탈: 낙관 모드는 H 뒤 다른 로봇이 같은 작업을 다시 확정해 중복이 난다.

    배타 잠금은 그 재확정을 거절하므로 중복이 없다 — 협상과 실행권을 나누는 이유 (보고서 50·56행).
    """
    dup_opt = 0
    for k in range(30):
        case, cost = case_and_cost(3, 7 + k)
        cfg = AgentConfig(stable_rounds=1, heartbeat_timeout=2)
        base = run(case, cost, cfg, PerLinkChannel(seed=k), task_lock=ExclusiveTaskLock())
        first = min(base.commit_round, key=lambda r: (base.commit_round[r], r))
        task = next(t for t, r in base.assignment.items() if r == first)
        drop = {first: base.commit_round[first] + 1}
        fixed = SimConfig(termination='fixed', max_rounds=20)       # 다 확정한 뒤에도 계속 돈다
        strict = run(case, cost, cfg, PerLinkChannel(seed=k), fixed,
                     task_lock=ExclusiveTaskLock(), dropouts=drop)
        assert first in strict.dead
        assert strict.dup == 0 and strict.holders[task] == [first]
        assert strict.lock_denied > 0
        opt = run(case, cost, cfg, PerLinkChannel(seed=k), fixed, dropouts=drop)
        dup_opt += opt.dup > 0
    assert dup_opt > 0


@pytest.mark.parametrize('rule', ['max', 'cbba'])
def test_health_event_reassigns_withdrawn_task(rule):
    """입찰 뒤 위치추정 건강이 나빠진 로봇의 작업을 다른 로봇이 가져간다 (미할당·고착 없음)."""
    moved = 0
    for k in range(40):
        case, cost = case_and_cost(3, 500 + k)
        cfg = AgentConfig(rule=rule, stable_rounds=3)
        base = run(case, cost, cfg, PerLinkChannel(seed=k, p_loss=0.1),
                   task_lock=ExclusiveTaskLock())
        victim = base.assignment.get(case.task_ids[0])
        if victim is None:
            continue
        res = run(case, cost, cfg, PerLinkChannel(seed=k, p_loss=0.1),
                  task_lock=ExclusiveTaskLock(), health_events=[(2, victim, False)])
        assert res.dup == 0 and res.stuck == 0 and res.unassigned == 0
        assert victim not in res.commit_round
        moved += 1
    assert moved > 30


def test_event_driven_sends_fewer_broadcasts():
    total = {False: 0, True: 0}
    for ev in (False, True):
        for k in range(30):
            case, cost = case_and_cost(5, 900 + k)
            res = run(case, cost, AgentConfig(stable_rounds=2, local_commit=False,
                                              announce_commits=False, rule='max'),
                      SharedStreamChannel(random.Random(k), p_loss=0.2),
                      SimConfig(termination='global', event_driven=ev, max_rounds=200))
            total[ev] += res.broadcasts
    assert total[True] < total[False]


def test_wall_clock_accounts_timeout_on_loss():
    case, cost = case_and_cost(3, 1)
    clean = run(case, cost, AgentConfig(stable_rounds=1), PerLinkChannel(seed=0, p_loss=0.0))
    assert 0.0 < clean.wall_ms <= 100.0 * clean.rounds
    lossy = run(case, cost, AgentConfig(stable_rounds=1, require_all_heard=False),
                PerLinkChannel(seed=0, p_loss=0.5))
    assert lossy.wall_ms >= 150.0             # 유실이 있으면 그 라운드는 시한 150 ms 를 다 쓴다
