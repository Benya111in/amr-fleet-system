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
])
def test_phase_mapping(phase, expected):
    assert map_status(False, phase) == expected


def test_estop_overrides_everything():
    for phase in PHASE_TO_STATUS:
        assert map_status(True, phase) == STATUS_ESTOP
    assert STATUS_NAMES[STATUS_ESTOP] == 'ESTOP' and len(STATUS_NAMES) == 7


def test_battery_percent():
    assert battery_percent(0.5) == 50.0
    assert battery_percent(1.2) == 100.0
    assert battery_percent(-0.1) == 0.0
    assert battery_percent(math.nan) == 100.0
    assert battery_percent(math.nan, fallback=42.0) == 42.0
    assert battery_percent(None, fallback=7.0) == 7.0
