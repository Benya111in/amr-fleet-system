"""robot_status: fleet_adapter 의 상태 매핑 (components.md §5.6)."""

import math

import pytest

from amr_fleet.kpi import (
    STATUS_CHARGING, STATUS_DOCKING, STATUS_ERROR, STATUS_ESTOP, STATUS_IDLE, STATUS_LOADING,
    STATUS_MOVING,
)
from amr_fleet.robot_status import (
    PHASE_TO_STATUS, STATUS_NAMES, battery_percent, map_status,
)


@pytest.mark.parametrize('phase, expected', [
    ('moving', STATUS_MOVING), ('Navigating', STATUS_MOVING), ('returning', STATUS_MOVING),
    ('docking', STATUS_DOCKING), ('undocking', STATUS_DOCKING),
    ('loading', STATUS_LOADING), ('UNLOADING', STATUS_LOADING),
    ('charging', STATUS_CHARGING),
    ('error', STATUS_ERROR), ('lost', STATUS_ERROR),
    ('idle', STATUS_IDLE), ('waiting', STATUS_IDLE), ('', STATUS_IDLE), (None, STATUS_IDLE),
    ('  moving  ', STATUS_MOVING), ('something_new', STATUS_IDLE),
    # 작업 중 단계는 배정 대상(IDLE)이 아니다 (리뷰: RECOVERING 이 IDLE 로 보였다)
    ('PERCEIVING', STATUS_MOVING), ('RECOVERING', STATUS_MOVING),
])
def test_phase_mapping(phase, expected):
    assert map_status(False, phase) == expected


def test_estop_overrides_everything():
    for phase in PHASE_TO_STATUS:
        assert map_status(True, phase) == STATUS_ESTOP
        assert map_status(True, phase, lost=True) == STATUS_ESTOP   # E-stop 이 lost 보다 위
    assert STATUS_NAMES[STATUS_ESTOP] == 'ESTOP' and len(STATUS_NAMES) == 7


def test_localization_lost_overrides_the_phase():
    """측위 상실은 phase 와 무관하게 ERROR 다 — IDLE 로 보이면 fleet 가 계속 배정한다 (§4.6)."""
    for phase in PHASE_TO_STATUS:
        assert map_status(False, phase, lost=True) == STATUS_ERROR
    # phase 가 비어 있어도(실행기가 아직 아무것도 발행 안 함) lost 면 ERROR
    assert map_status(False, None, lost=True) == STATUS_ERROR


def test_lost_is_not_sticky():
    """해제되면 phase 로 되돌아가야 한다 — 들러붙으면 한 번 lost 된 로봇을 영구히 잃는다."""
    assert map_status(False, 'idle', lost=True) == STATUS_ERROR
    assert map_status(False, 'idle', lost=False) == STATUS_IDLE
    assert map_status(False, 'moving', lost=True) == STATUS_ERROR
    assert map_status(False, 'moving', lost=False) == STATUS_MOVING


def test_lost_defaults_to_false_so_the_old_call_shape_is_unchanged():
    """기존 두 인자 호출의 뜻이 바뀌면 안 된다 (다른 호출부·테스트가 그대로 쓴다)."""
    for phase in PHASE_TO_STATUS:
        assert map_status(False, phase) == map_status(False, phase, lost=False)


def test_battery_percent():
    assert battery_percent(0.5) == 50.0
    assert battery_percent(1.2) == 100.0
    assert battery_percent(-0.1) == 0.0
    assert battery_percent(math.nan) == 100.0
    assert battery_percent(math.nan, fallback=42.0) == 42.0
    assert battery_percent(None, fallback=7.0) == 7.0
