"""
fleetsim 실행: 결정성, 불변식(동시 점유 0 · 작업 보존 · 운동학 한계), 고장 의미, 판정의 힘(양성 대조).

짧은 지평선(sim 300–420 s)으로 돈다. 스윕 결과의 근거가 되는 성질만 시험한다.
"""

import json
import math
import os
import subprocess
import sys

import pytest

from amr_fleet.comm_emulator import FLEET
from amr_fleet.fleetsim import plan
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


@pytest.mark.parametrize('fault', ['F0', 'F5'])
def test_no_fallback_without_server_loss(fault):
    """
    서버가 살아 있으면 전환하지 않는다 — 분할(F5)의 소수쪽 2 대가 하트비트를 잃어도 과반이 아니다.

    F5 칸은 리뷰 변이 X3(과반 대신 자기 의심만으로 전환)을 잡는다: 소수쪽이 혼자 협상 모드로 들어간다.
    """
    r = run(cfg('A9_hybrid_vote', loss=0.2, fault=fault, horizon_s=600.0, t_fault=300.0,
                partition_s=180.0))
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


@pytest.mark.parametrize('seed', [8, 13, 24])
def test_task_lock_quorum_prevents_partition_duplicates(seed):
    """
    작업 ID 잠금의 **과반 정족수** 가 분할 중복을 막는다 (리뷰 B2, 변이 X4).

    F5(분할 180 s) 손실 0 에서 낙관 A7 은 소수쪽과 다수쪽이 같은 작업을 실행한다(양성 대조, 시드마다 > 0).
    엄격 A6 은 소수쪽 2 대가 3 표를 못 모아 잠그지 못하므로 0 이어야 한다. 자기 표만으로 잠기게 하면(X4) 이 시험이
    실패한다 — 이른 확정 시험(위)은 executor_of · 한 라운드 지연이 막아 정족수를 시험하지 못했다.
    시드는 P4 리뷰 재실행에서 A7 이 중복을 낸 칸을 골랐다 (원본 A6 0 · A7 1, X4 변이 A6 1).
    """
    base = dict(loss=0.0, fault='F5', seed=seed, arrival_rate=1.0 / 45.0, t_fault=300.0,
                partition_s=180.0, horizon_s=600.0)
    opt = run(cfg('A7_cbaa_opt_vote', **base))
    strict = run(cfg('A6_cbaa_vote', **base))
    assert opt['dup_tasks'] > 0, opt['dup_detail']
    assert strict['dup_tasks'] == 0, strict['dup_detail']


HOLD_ARMS = ['A1h_nearest_hold', 'A5h_cbaa_hold']


# p0.md §5: 결정점 거리 D = 1.85 m. 도크 구역 넷만 approach_m 1.5 로 미달 (배치 간격 2 m 때문, 문서화된 위험)
DECISION_DISTANCE_M = 1.85


@pytest.mark.parametrize('arm', HOLD_ARMS)
def test_hold_mode_protects_zones_meeting_decision_distance(arm):
    """
    현행 hold 의미(A1h · A5h), 서버 생존 · 손실 0: 결정점 거리를 지키는 구역(교차로 · 통로)은 동시 점유 0.

    hold 는 거부 목록이라 요청 거리가 결정점 거리보다 짧은 도크 구역(p0.md §5 "미달")에서는 거절 hold 가 늦어 들어갈 수
    있다 — 그것은 시험이 아니라 스윕 결과로 보고한다 (허가 목록 A1 은 이 위험을 가린다, 리뷰 B1). hold 를 내지 않는
    서버(변이 H2)는 hold_true_pub 단언이 잡는다. 운동학 한계 · 작업 보존도 함께.
    """
    w = default_world()
    short = {z for z, a in w.approach.items() if a < DECISION_DISTANCE_M}
    assert short == set(plan.BELOW_DECISION_ZONES)
    for s in (101, 102):
        r = run(cfg(arm, loss=0.0, seed=s, horizon_s=600.0, arrival_rate=1.0 / 45.0))
        bad = [v for v in r['cooccupancy_detail'] if v['zone'] not in short]
        assert not bad, (arm, s, bad)
        assert r['max_speed'] <= V_MAX + 1e-9 and r['max_decel'] <= ACC + 1e-6, (arm, s)
        assert r['completed'] + r['unfinished'] == r['n_tasks']
        assert r['completed'] >= 5 and r['server']['hold_true_pub'] > 0, (arm, s, r['server'])


def test_hold_true_stops_robot_with_bounded_decel_and_false_releases():
    """
    hold=true 를 받으면 그 자리에서 최대 감속 ≤ 1.0 m/s² 로 서고 더 나아가지 않는다, false 면 다시 간다 (변이 H1).

    현행 실행기는 hold=true 에 주행 goal 을 취소한다 — 정지선이 아니라 지금 자리에서 선다.
    """
    sim = FleetSim(cfg('A1h_nearest_hold'))
    r = sim.robots[0]
    r.on_message(0.0, FLEET, 'hold', (1, False))
    r._start_leg(0.0, 'dock_a', 'to_pickup', 'probe')
    t, dt = 0.0, sim.cfg.dt
    while r.v < 0.6 and t < 10.0:
        r.step(t, dt)
        t += dt
    assert r.v >= 0.6, '가속하지 못했다 (hold=false 인데 섰다)'
    r.on_message(t, FLEET, 'hold', (2, True))
    s0, v0, prev = r.s, r.v, r.v
    for _ in range(30):
        r.step(t, dt)
        t += dt
        assert prev - r.v <= ACC * dt + 1e-9
        prev = r.v
    assert r.v == 0.0
    assert r.s - s0 <= v0 * v0 / (2.0 * ACC) + v0 * dt + 1e-6
    s1 = r.s
    for _ in range(20):
        r.step(t, dt)
        t += dt
    assert r.s == s1
    r.on_message(t, FLEET, 'hold', (3, False))
    for _ in range(20):
        r.step(t, dt)
        t += dt
    assert r.s > s1 + 0.1


@pytest.mark.parametrize('seed', [11, 101])
def test_latched_hold_fails_open_after_server_crash(seed):
    """
    현행 latched hold: 서버가 죽으면 마지막 값이 남는다 (hold_failsafe.md D4 · §4 위험 1, 리뷰 B1).

    충돌 순간 hold=false 였던 로봇은 관리 없이 계속 움직이고(fail-open), hold=true 였던 로봇은 그 자리에 선다.
    허가 목록 모형(A1)은 충돌 뒤 새 구역에 못 들어가므로 A1h 가 충돌 뒤 더 많이 끝낸다.
    """
    base = dict(loss=0.0, fault='F1', t_fault=150.0, horizon_s=600.0, seed=seed,
                arrival_rate=1.0 / 45.0)
    snap = {}

    class CrashSpy(FleetSim):
        def _crash_server(self, now):
            snap.update({r.id: (r.hold, r.x, r.y) for r in self.robots})
            super()._crash_server(now)

    sim = CrashSpy(cfg('A1h_nearest_hold', **base))
    res = sim.run()
    assert len(snap) == 5
    moved = {r.id: math.hypot(r.x - snap[r.id][1], r.y - snap[r.id][2]) for r in sim.robots}
    held = [rid for rid, (h, _, _) in snap.items() if h]
    free = [rid for rid, (h, _, _) in snap.items() if not h]
    assert all(moved[rid] <= 0.6 for rid in held), (held, moved)       # 최대 감속 정지 거리 0.5 m
    assert free and max(moved[rid] for rid in free) > 1.0, (free, moved)
    cen = run(cfg('A1_nearest_central', **base))
    assert res['completed_after_fault'] > cen['completed_after_fault']


def test_hold_follows_latest_seq_and_waits_before_first_value():
    """수신한 hold: 첫 값 전 = 대기(D2), 신뢰 경로 재전송으로 늦게 온 옛 seq 는 새 값을 덮지 않는다 (변이 H4)."""
    sim = FleetSim(cfg('A1h_nearest_hold'))
    r = sim.robots[0]
    zone = sorted(sim.world.boxes)[0]
    assert r.hold is True and not r.permitted(zone)
    r.on_message(1.0, FLEET, 'hold', (2, False))
    assert r.permitted(zone)
    r.on_message(1.1, FLEET, 'hold', (1, True))
    assert r.hold is False and r.permitted(zone)
    r.on_message(1.2, FLEET, 'hold', (3, True))
    assert not r.permitted(zone)
