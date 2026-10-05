"""
fleet_adapter 의 RobotState.status 매핑 (components.md §5.6).

    estop → ESTOP, executor/phase → MOVING/DOCKING/LOADING/CHARGING
    (perceiving·recovering = MOVING), lost/error → ERROR, 그 외 IDLE
"""

from __future__ import annotations

import math
from typing import Dict, Optional

from amr_fleet.kpi import (
    STATUS_CHARGING, STATUS_DOCKING, STATUS_ERROR, STATUS_ESTOP, STATUS_IDLE, STATUS_LOADING,
    STATUS_MOVING,
)

STATUS_NAMES: Dict[int, str] = {
    STATUS_IDLE: 'IDLE', STATUS_MOVING: 'MOVING', STATUS_DOCKING: 'DOCKING',
    STATUS_LOADING: 'LOADING', STATUS_CHARGING: 'CHARGING', STATUS_ERROR: 'ERROR',
    STATUS_ESTOP: 'ESTOP',
}

# task_executor_node 의 executor/phase 문자열 → RobotState.status
PHASE_TO_STATUS: Dict[str, int] = {
    'moving': STATUS_MOVING, 'navigating': STATUS_MOVING, 'returning': STATUS_MOVING,
    # 작업 수행 중인 단계 — IDLE 로 보이면 배정 대상이 된다 (fleet_manager 는 IDLE 에만 배정)
    'perceiving': STATUS_MOVING, 'recovering': STATUS_MOVING,
    'docking': STATUS_DOCKING, 'undocking': STATUS_DOCKING,
    'loading': STATUS_LOADING, 'unloading': STATUS_LOADING,
    'charging': STATUS_CHARGING,
    'error': STATUS_ERROR, 'lost': STATUS_ERROR,
    'idle': STATUS_IDLE, 'waiting': STATUS_IDLE, '': STATUS_IDLE,
}


def map_status(estop: bool, phase: Optional[str]) -> int:
    """E-stop 이 최우선, 다음 phase 문자열(대소문자 무시), 모르는 phase 는 IDLE."""
    if estop:
        return STATUS_ESTOP
    key = (phase or '').strip().lower()
    return PHASE_TO_STATUS.get(key, STATUS_IDLE)


def battery_percent(percentage: float, fallback: float = 100.0) -> float:
    """sensor_msgs/BatteryState.percentage(0~1, 미측정 NaN) → RobotState.battery_level(0~100 %)."""
    if percentage is None or math.isnan(percentage):
        return fallback
    return max(0.0, min(100.0, percentage * 100.0))
