"""
fleetsim 실행: 결정성, 불변식(동시 점유 0 · 작업 보존 · 운동학 한계), 고장 의미, 판정의 힘(양성 대조).

짧은 지평선(sim 300–420 s)으로 돈다. 스윕 결과의 근거가 되는 성질만 시험한다.
"""

import json
import os
import subprocess
import sys

import pytest

from amr_fleet.fleetsim.config import ARMS, Arm, RunConfig, task_stream
from amr_fleet.fleetsim.robot import ACC, V_MAX
from amr_fleet.fleetsim.sim import FleetSim, run
from amr_fleet.fleetsim.world import DEFAULT_MAP, default_world

pytestmark = pytest.mark.skipif(not DEFAULT_MAP.is_file(), reason='maps/warehouse.yaml 없음')

SAFE_ARMS = ['A1_nearest_central', 'A4_hungarian_vote', 'A5_cbaa_central', 'A6_cbaa_vote',
             'A9_hybrid_vote']


def cfg(arm, **kw):
    kw.setdefault('horizon_s', 360.0)
    kw.setdefault('seed', 101)
    return RunConfig(ARMS[arm], **kw)


def _strip(res):
    return {k: v for k, v in res.items() if k != 'wall_s'}


def test_same_config_same_result():
    for arm in ('A1_nearest_central', 'A6_cbaa_vote', 'A9_hybrid_vote'):
        a = run(cfg(arm, loss=0.2, fault='F1'))
        b = run(cfg(arm, loss=0.2, fault='F1'))
        assert _strip(a) == _strip(b), arm


def test_result_independent_of_hash_seed():
    """문자열 집합 순회 순서가 결과에 새지 않는다: PYTHONHASHSEED 가 다른 하위 프로세스에서 같은 digest."""
    code = ('import json; from amr_fleet.fleetsim.config import ARMS, RunConfig; '
            'from amr_fleet.fleetsim.sim import run; '
            'r = run(RunConfig(ARMS["A6_cbaa_vote"], loss=0.2, fault="F5", seed=7, '
            'horizon_s=360.0)); print(json.dumps([r["digest"], r["completed"], r["msgs_sent"]]))')
    outs = []
    for hs in ('0', '12345'):
        env = dict(os.environ, PYTHONHASHSEED=hs)
        out = subprocess.run([sys.executable, '-c', code], env=env, capture_output=True,
                             text=True, timeout=300)
        assert out.returncode == 0, out.stderr
        outs.append(json.loads(out.stdout.strip().splitlines()[-1]))
    assert outs[0] == outs[1]


def test_pairs_share_exogenous_inputs_and_rep_changes_only_micro_noise():
    a = FleetSim(cfg('A1_nearest_central', seed=5, rep=0))
    b = FleetSim(cfg('A6_cbaa_vote', seed=5, rep=0, fault='F6'))
    c = FleetSim(cfg('A1_nearest_central', seed=5, rep=1))
    ta = [(x.t, x.pickup, x.dropoff, x.spec.priority) for x in a.arrivals]
    assert ta == [(x.t, x.pickup, x.dropoff, x.spec.priority) for x in b.arrivals]
    assert ta == [(x.t, x.pickup, x.dropoff, x.spec.priority) for x in c.arrivals]
    assert a.comm.seed == b.comm.seed != c.comm.seed
    d = FleetSim(cfg('A1_nearest_central', seed=6))
    assert ta != [(x.t, x.pickup, x.dropoff, x.spec.priority) for x in d.arrivals]


def test_task_stream_is_poisson_dock_to_dock():
    w = default_world()
    arr = task_stream(RunConfig(ARMS['A1_nearest_central'], seed=3, horizon_s=20000.0),
                      w.docks, w.places)
    assert all(a.pickup != a.dropoff and a.pickup in w.docks and a.dropoff in w.docks
               for a in arr)
    rate = len(arr) / 20000.0
    assert rate == pytest.approx(1.0 / 30.0, rel=0.08)


@pytest.mark.parametrize('fault', ['F0', 'F1', 'F5', 'F6'])
@pytest.mark.parametrize('loss', [0.0, 0.4])
def test_safety_invariants_hold(loss, fault):
    """실제 차체 기준 용량 1 구역 동시 점유 0, 엄격 잠금 비교군 중복 실행 0, 운동학 한계, 작업 보존."""
    for arm in SAFE_ARMS:
        r = run(cfg(arm, loss=loss, fault=fault, t_fault=150.0, partition_s=120.0))
        where = (arm, loss, fault)
        assert r['cooccupancy_events'] == 0, (where, r['cooccupancy_detail'])
        if ARMS[arm].needs_task_lock:
            assert r['dup_tasks'] == 0, (where, r['dup_detail'])
        assert r['max_speed'] <= V_MAX + 1e-9, where
        assert r['max_decel'] <= ACC + 1e-6, where
        assert r['completed'] + r['unfinished'] == r['n_tasks'], where
        assert sum(r['q_bins']) == r['completed'], where


def test_oracle_sees_cooccupancy_without_traffic_management():
    """양성 대조: 교통 관리를 끄면(유령 기준선) 같은 오라클이 동시 점유를 센다 — 0 이 공허하지 않다."""
    seen = sum(run(cfg('X0_hungarian_none', seed=s, horizon_s=600.0))['cooccupancy_events']
               for s in (101, 102, 103))
    assert seen > 0


def test_server_crash_stops_central_but_not_fallback():
    base = dict(loss=0.05, fault='F1', t_fault=150.0, horizon_s=600.0, seed=11)
    cen = run(cfg('A1_nearest_central', **base))
    hyb = run(cfg('A9_hybrid_vote', **base))
    dis = run(cfg('A6_cbaa_vote', **base))
    # 중앙: 충돌 뒤에는 이미 받은 작업만 (새 할당 없음) — 충돌 뒤 완료는 그때 진행 중이던 수 이하
    assert cen['completed_after_fault'] <= 5
    assert hyb['first_fallback_s'] is not None
    # 마지막 하트비트(충돌 직전, 주기 0.5 s) + dead_s 5 s, 비컨 주기 0.5 s 안에 과반 판정
    assert 150.0 + 4.4 <= hyb['first_fallback_s'] <= 150.0 + 6.5
    assert hyb['completed_after_fault'] > cen['completed_after_fault']
    assert dis['completed_after_fault'] > cen['completed_after_fault']
    assert hyb['cbaa_tasks'] > 0


def test_no_fallback_without_fault():
    r = run(cfg('A9_hybrid_vote', loss=0.2, horizon_s=600.0))
    assert r['first_fallback_s'] is None and r['cbaa_tasks'] == 0


def test_robot_crash_freezes_victim():
    sim = FleetSim(cfg('A6_cbaa_vote', fault='F6', t_fault=100.0, horizon_s=300.0))
    res = sim.run()
    v = sim.by_id[res['victim']]
    assert not v.alive
    assert sum(1 for r in sim.robots if r.alive) == 4


def test_arm_validation():
    with pytest.raises(ValueError):
        Arm('bad', 'cbaa_strict', 'vote', fallback=True)
    with pytest.raises(ValueError):
        Arm('bad', 'nope', 'vote')
    with pytest.raises(ValueError):
        RunConfig(ARMS['A1_nearest_central'], loss=1.0)
    with pytest.raises(ValueError):
        RunConfig(ARMS['A1_nearest_central'], fault='F9')


def test_idle_robots_wait_at_their_waiting_pose():
    """하역 뒤 대기 자세(스폰 자세)로 돌아가 IDLE — 복귀 중에는 IDLE 이 아니다 (accept_while_returning false)."""
    sim = FleetSim(cfg('A1_nearest_central', horizon_s=900.0, arrival_rate=1.0 / 120.0))
    res = sim.run()
    assert res['completed'] >= 1
    w = sim.world
    for r in sim.robots:
        if r.idle():
            assert (r.x, r.y) == pytest.approx(w.places[f'spawn_{r.id}'])
        if r.stage == 'return':
            assert not r.idle()


@pytest.mark.parametrize('seed,t_fault', [(1, 200.0), (4, 200.0), (5, 300.0)])
def test_dead_robot_inside_zone_keeps_it_closed(seed, t_fault):
    """
    F6: 이탈 로봇이 도크 구역 안에서 멈추면 그 구역은 fail-closed 로 닫힌 채 남는다.

    중앙(lost → 몸체가 걸친 구역 토큰만 유지)에서 다른 로봇이 그 구역 정지선에 서고, 동시 점유는 0 이다.
    시드는 피해 로봇이 구역 안에서 멈추는 경우를 찾아 고정했다 (P4 노트 P4-1 변이 검사).
    """
    sim = FleetSim(cfg('A1_nearest_central', fault='F6', seed=seed, t_fault=t_fault,
                       horizon_s=t_fault + 300.0, arrival_rate=1.0 / 45.0))
    res = sim.run()
    v = sim.by_id[res['victim']]
    held = sim.world.body_zones(v.x, v.y, v.th)
    assert held, '피해 로봇이 구역 안에 있지 않다 — 시나리오가 가드를 시험하지 못한다'
    assert any(r.alive and r.wait_zone in held for r in sim.robots)
    assert res['cooccupancy_events'] == 0, res['cooccupancy_detail']


def test_task_lock_prevents_duplicates_under_aggressive_commit(monkeypatch):
    """
    작업 ID 투표 잠금의 힘: 확정을 일부러 이르게(K=1, 전원 수신 없음) 하고 손실 0.4 로 협상 표가 갈리게 한다.

    낙관(잠금 없음) A7 은 중복 실행이 생기고(오라클 양성 대조), 엄격 A6 은 0 이어야 한다.
    """
    from amr_fleet.negotiation import agent as nego_agent
    early = classmethod(lambda c: c(rule='cbba', stable_rounds=1, require_all_heard=False,
                                    heartbeat_timeout=5.0, liveness='indirect'))
    monkeypatch.setattr(nego_agent.AgentConfig, 'strict_mode', early)
    monkeypatch.setattr(nego_agent.AgentConfig, 'optimistic_mode', early)
    opt = strict = 0
    for s in (2, 5, 6):
        base = dict(loss=0.4, seed=s, arrival_rate=1.0 / 30.0, horizon_s=600.0)
        opt += run(cfg('A7_cbaa_opt_vote', **base))['dup_tasks']
        strict += run(cfg('A6_cbaa_vote', **base))['dup_tasks']
    assert opt > 0
    assert strict == 0
