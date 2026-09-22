from types import SimpleNamespace as NS

from amr_evaluation import cte_gate
from amr_evaluation.cte_gate import GateConfig, PlanGate


def _moving(gate, t=10.0):
    return gate.check(t, 0.5, 0.0)


def test_no_plan_then_accept_and_stopped():
    gate = PlanGate()
    assert _moving(gate) == cte_gate.NO_PLAN
    gate.on_plan(8.0, 8.0)
    assert _moving(gate) is None
    assert gate.check_projection(False) is None
    assert gate.check(10.0, 0.01, 0.01) == cte_gate.STOPPED
    assert gate.check(10.0, 0.0, 0.3) is None          # 제자리 회전은 정지가 아니다
    assert gate.check_projection(True) == cte_gate.ENDPOINT
    assert gate.accepted == 1
    assert gate.skipped[cte_gate.STOPPED] == 1 and gate.skipped[cte_gate.NO_PLAN] == 1
    assert 'stopped 1' in gate.summary() and 'endpoint 1' in gate.summary()
    gate.clear_plan()
    assert _moving(gate) == cte_gate.NO_PLAN


def test_nav_status_latest_goal_and_stale_plan():
    gate = PlanGate()
    gate.on_plan(100.0, 100.0)
    gate.on_nav_status([(90.0, cte_gate.GOAL_EXECUTING)])
    assert _moving(gate) is None
    # 목표 성공 (주차·도킹으로 넘어감) → 제외
    gate.on_nav_status([(90.0, 4)])
    assert _moving(gate) == cte_gate.NAV_INACTIVE
    # 새 목표 수락, 경로는 아직 이전 것 → stale
    gate.on_nav_status([(90.0, 4), (120.0, cte_gate.GOAL_ACCEPTED)])
    assert _moving(gate) == cte_gate.STALE_PLAN
    gate.on_plan(120.5, 121.0)
    assert _moving(gate) is None
    # 스탬프 0 인 경로는 수신 시각으로 비교한다
    gate.on_nav_status([(130.0, cte_gate.GOAL_EXECUTING)])
    gate.on_plan(0.0, 131.0)
    assert gate.plan_stamp == 131.0 and _moving(gate) is None
    # 상태 목록이 비면(목표 없음) 비활성
    gate.on_nav_status([])
    assert _moving(gate) == cte_gate.NAV_INACTIVE


def test_phase_gating_and_grace():
    gate = PlanGate(GateConfig(active_phases=('moving',), phase_grace=1.0))
    gate.on_plan(10.0, 10.0)
    gate.on_phase('moving', 11.0)
    assert _moving(gate) is None                        # 1 s 안쪽에 받은 경로는 새 주행 것
    gate.on_phase('Docking ', 20.0)
    assert _moving(gate) == cte_gate.PHASE
    gate.on_phase('loading', 25.0)
    assert _moving(gate) == cte_gate.PHASE
    gate.on_phase('moving', 40.0)                       # 새 주행 시작, 경로는 10 s 에 받은 것
    assert _moving(gate) == cte_gate.STALE_PLAN
    gate.on_phase('moving', 41.0)                       # 계속 moving: 시작 시각 유지
    assert gate.phase_start_rx == 40.0
    gate.on_plan(40.5, 40.5)
    assert _moving(gate) is None


def test_plan_timeout_and_reset():
    gate = PlanGate(GateConfig(plan_timeout=2.0))
    gate.on_plan(1.0, 1.0)
    assert gate.check(2.5, 0.5, 0.0) is None
    assert gate.check(3.5, 0.5, 0.0) == cte_gate.STALE_PLAN
    gate.on_nav_status([(1.0, cte_gate.GOAL_EXECUTING)])
    gate.on_phase('moving', 1.0)
    gate.reset()
    assert gate.plan_rx is None and not gate.nav_seen and gate.phase is None
    assert gate.check(0.5, 0.5, 0.0) == cte_gate.NO_PLAN
    assert gate.skipped[cte_gate.STALE_PLAN] == 1       # 카운터는 유지
    assert PlanGate().summary() == '채택 0, 제외 0'


def test_plan_timeout_default_expires_plan_without_state_sources():
    """상태 원천이 없어도 기본 3 s 만료: 도착 뒤(재계획이 멈춘 뒤) 옛 경로로 채점하지 않는다."""
    assert GateConfig().plan_timeout == 3.0
    gate = PlanGate()
    for t in (1.0, 2.0, 3.0):                            # 주행 중: 1 Hz 재계획
        gate.on_plan(t, t)
        assert gate.check(t + 0.9, 0.5, 0.0) is None
    assert gate.check(5.9, 0.5, 0.0) is None            # 재계획 두 번 누락까지는 채택
    assert gate.check(6.1, 0.5, 0.0) == cte_gate.STALE_PLAN
    off = PlanGate(GateConfig(plan_timeout=0.0))
    off.on_plan(1.0, 1.0)
    assert off.check(1000.0, 0.5, 0.0) is None           # 0 이면 끔


def test_goal_status_tuples():
    def st(sec, nsec, status):
        return NS(goal_info=NS(stamp=NS(sec=sec, nanosec=nsec)), status=status)
    out = cte_gate.goal_status_tuples([st(3, 500_000_000, 2), st(1, 0, 4)])
    assert out == [(3.5, 2), (1.0, 4)]
    assert cte_gate.goal_status_tuples([]) == []
