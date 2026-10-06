"""모드 상태기 단위 시험 (agent/modes.py, 보고서 145–158행 · p3.md §3)."""

import pytest

from amr_fleet.agent.modes import (
    DEGRADED, HOLD_NEW, ISOLATED, ModeConfig, ModeMachine, NORMAL, RESYNC,
)
from amr_fleet.distributed.failure_detector import ALIVE, DEAD, SUSPECT, UNKNOWN

CFG = ModeConfig(resync_s=3.0, isolate_s=1.0, boot_grace_s=2.0, unknown_dead_s=5.0)


def _run(mm, t0, t1, status, maj=False, conn=True, active=True, dt=0.1):
    t = t0
    while t < t1 - 1e-9:
        t = round(t + dt, 6)
        mm.update(t, status, maj, conn, active)
    return t


def test_normal_suspect_degraded_resync_normal_sequence():
    mm = ModeMachine(CFG, 0.0)
    t = _run(mm, 0.0, 1.0, ALIVE)
    assert mm.mode == NORMAL
    t = _run(mm, t, 2.0, SUSPECT)
    assert mm.mode == HOLD_NEW
    t = _run(mm, t, 3.0, DEAD)
    assert mm.mode == HOLD_NEW               # 나 혼자 DEAD 로는 전환하지 않는다
    t = _run(mm, t, 4.0, DEAD, maj=True)
    assert mm.mode == DEGRADED
    t = _run(mm, t, 5.0, ALIVE, maj=True)    # 4.1 부터 ALIVE
    assert mm.mode == RESYNC
    t = _run(mm, t, 7.0, ALIVE)              # 2.9 s < resync_s
    assert mm.mode == RESYNC
    t = _run(mm, t, 7.2, ALIVE)              # 3.1 s ≥ resync_s
    assert mm.mode == NORMAL
    seq = [(a, b) for _, a, b, _ in mm.transitions]
    assert seq == [(NORMAL, HOLD_NEW), (HOLD_NEW, DEGRADED), (DEGRADED, RESYNC),
                   (RESYNC, NORMAL)]


def test_hold_new_is_reversible():
    mm = ModeMachine(CFG, 0.0)
    t = _run(mm, 0.0, 1.0, SUSPECT)
    assert mm.mode == HOLD_NEW
    _run(mm, t, 1.5, ALIVE)
    assert mm.mode == NORMAL


def test_resync_falls_back_on_flapping_and_restarts_hysteresis():
    mm = ModeMachine(CFG, 0.0)
    t = _run(mm, 0.0, 1.0, DEAD, maj=True)
    assert mm.mode == DEGRADED
    t = _run(mm, t, 3.0, ALIVE)
    assert mm.mode == RESYNC
    t = _run(mm, t, 3.5, SUSPECT)            # 깜빡임
    assert mm.mode == DEGRADED
    t = _run(mm, t, 4.0, ALIVE)
    assert mm.mode == RESYNC
    t = _run(mm, t, 6.5, ALIVE)              # 깜빡임 뒤 2.5 s — 히스테리시스는 처음부터
    assert mm.mode == RESYNC
    _run(mm, t, 7.2, ALIVE)
    assert mm.mode == NORMAL


def test_resync_waits_for_active_server():
    mm = ModeMachine(CFG, 0.0)
    t = _run(mm, 0.0, 1.0, DEAD, maj=True)
    t = _run(mm, t, 10.0, ALIVE, active=False)   # 관찰 중인 서버
    assert mm.mode == RESYNC
    _run(mm, t, 10.2, ALIVE, active=True)
    assert mm.mode == NORMAL


def test_isolated_overlays_and_returns_to_base():
    mm = ModeMachine(CFG, 0.0)
    t = _run(mm, 0.0, 1.5, ALIVE, conn=False)
    assert mm.mode == NORMAL                 # 부팅 유예
    t = _run(mm, t, 3.2, ALIVE, conn=False)
    assert mm.mode == ISOLATED
    assert not mm.allows_new_entry() and not mm.accepts_central_tasks()
    _run(mm, t, 3.4, ALIVE, conn=True)
    assert mm.mode == NORMAL
    assert [b for _, _, b, _ in mm.transitions] == [ISOLATED, NORMAL]


def test_never_heard_server_counts_as_dead_after_unknown_dead_s():
    mm = ModeMachine(CFG, 0.0)
    assert not mm.server_dead(UNKNOWN, 4.9)
    assert mm.server_dead(UNKNOWN, 5.0)
    t = _run(mm, 0.0, 4.0, UNKNOWN)
    assert mm.mode == NORMAL
    _run(mm, t, 5.5, UNKNOWN)
    assert mm.mode == HOLD_NEW


def test_mode_permissions():
    mm = ModeMachine(CFG, 0.0)
    assert mm.accepts_central_tasks() and not mm.negotiates() and mm.allows_new_entry()
    _run(mm, 0.0, 1.0, DEAD, maj=True)
    assert mm.negotiates() and not mm.accepts_central_tasks()


def test_bad_config():
    with pytest.raises(ValueError):
        ModeConfig(resync_s=-1.0)


def test_own_dead_without_majority_never_degrades_from_normal():
    """NORMAL 에서 내 감시가 곧바로 DEAD 여도 (예: 자기 정지 뒤) 과반 동의 없이는 DEGRADED 로 가지 않는다."""
    mm = ModeMachine(CFG, 0.0)
    t = _run(mm, 0.0, 1.0, ALIVE)
    _run(mm, t, 1.1, DEAD, maj=False)
    assert mm.mode == HOLD_NEW
    assert [b for _, _, b, _ in mm.transitions] == [HOLD_NEW]
