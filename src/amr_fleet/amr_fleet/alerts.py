"""
/fleet/alerts 이름 규약과 /fleet/traffic_events 해석 (연구 브리프 fleet-traffic-deadlock §5.5 · §7.2).

- 알림: diagnostic_msgs/DiagnosticStatus name = fleet/<TYPE>, level WARN/ERROR,
  hardware_id = robot_id, values 에 task_id · robots (대시보드 알림 배너가 그대로 표시).
- 교통 이벤트: traffic_manager_node 가 name = traffic/DEADLOCK (탐지) | traffic/RESOLVED (해소) 로
  보낸다. 탐지만 FleetStatus.deadlock_count 에 센다.
"""

from __future__ import annotations

from typing import Dict

ALERT_ESTOP = 'fleet/ESTOP'                      # 로봇 E-stop (ERROR)
ALERT_ROBOT_ERROR = 'fleet/ROBOT_ERROR'          # RobotState.STATUS_ERROR 진입 (WARN)
ALERT_TASK_FAILED = 'fleet/TASK_FAILED'          # 작업 FAILED 전이 (ERROR)
ALERT_TASK_REQUEUED = 'fleet/TASK_REQUEUED'      # FAILED → PENDING 재시도 (WARN)
ALERT_DEADLINE_MISSED = 'fleet/DEADLINE_MISSED'  # 종료 전 마감 초과 (WARN)
ALERT_INVALID_TASK = 'fleet/INVALID_TASK'        # JSON 스키마 위반 작업 요청 (WARN)

TRAFFIC_DEADLOCK = 'traffic/DEADLOCK'
TRAFFIC_RESOLVED = 'traffic/RESOLVED'


def is_deadlock_event(name: str) -> bool:
    """traffic/DEADLOCK 이면 True (대소문자·접두 무시). RESOLVED 등 다른 이벤트는 False."""
    return name.strip().split('/')[-1].upper() == 'DEADLOCK'


def alert_values(task_id: str = '', robot_id: str = '', **extra: object) -> Dict[str, str]:
    """알림 values: task_id, robots(로봇 1대면 그 id) + 추가 키. 빈 값은 뺀다."""
    out: Dict[str, object] = {'task_id': task_id, 'robots': robot_id}
    out.update(extra)
    return {k: str(v) for k, v in out.items() if v is not None and v != ''}
