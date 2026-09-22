"""
/fleet/alerts 이름 규약과 /fleet/traffic_events 해석 (연구 브리프 fleet-traffic-deadlock §5.5 · §7.2).

- 알림: diagnostic_msgs/DiagnosticStatus name = fleet/<TYPE>, level OK/WARN/ERROR,
  hardware_id = robot_id, values 에 task_id · robots (대시보드 알림 배너가 그대로 표시).
  OK 는 해제·복구 통지다 (fleet/ESTOP 해제, fleet/ROBOT_RECOVERED) — 배너는 WARN 이상만 띄운다.
- 교통 이벤트: traffic_manager_node 가 /fleet/traffic_events 로 name = traffic/DEADLOCK (탐지) |
  traffic/RESOLVED (해소) | traffic/ESCALATED (전략 1 → 2, 희생 로봇 교체) | traffic/UNRESOLVED (해소 실패) |
  traffic/STALL (사이클 없는 1대 정체) 를 보낸다. 탐지(DEADLOCK)만 FleetStatus.deadlock_count 에 센다.
  교착 알림은 traffic_manager_node 가 /fleet/alerts 에 fleet/DEADLOCK 으로 직접 낸다
  (탐지 ERROR, 해소 OK, 해소 실패 ERROR — fleet/ESTOP 의 진입/해제와 같은 방식).
"""

from __future__ import annotations

from typing import Dict

from amr_fleet.task_schema import safe_text

ALERT_ESTOP = 'fleet/ESTOP'                      # 로봇 E-stop (ERROR 진입, OK 해제)
ALERT_ROBOT_ERROR = 'fleet/ROBOT_ERROR'          # RobotState.STATUS_ERROR 진입 (WARN)
ALERT_ROBOT_LOST = 'fleet/ROBOT_LOST'            # robot_state_timeout_s 동안 robot_state 없음 (ERROR)
ALERT_ROBOT_RECOVERED = 'fleet/ROBOT_RECOVERED'  # 끊겼던 로봇의 robot_state 재수신 (OK)
ALERT_TASK_FAILED = 'fleet/TASK_FAILED'          # 작업 FAILED 전이 (ERROR)
ALERT_TASK_REQUEUED = 'fleet/TASK_REQUEUED'      # FAILED → PENDING 재시도 (WARN)
ALERT_TASK_CONFLICT = 'fleet/TASK_CONFLICT'      # 소유 로봇이 아닌 보고·늦은 수락 (WARN/ERROR)
ALERT_ASSIGN_TIMEOUT = 'fleet/ASSIGN_TIMEOUT'    # assign_task 응답 없음 → 확인 전까지 보류 (WARN)
ALERT_DEADLINE_MISSED = 'fleet/DEADLINE_MISSED'  # 종료 전 마감 초과 (WARN)
ALERT_INVALID_TASK = 'fleet/INVALID_TASK'        # 스키마·의미 규칙 위반 작업 요청 (WARN)
ALERT_INTERNAL_ERROR = 'fleet/INTERNAL_ERROR'    # 콜백 예외 — 노드는 계속 돈다 (ERROR)
ALERT_DEADLOCK = 'fleet/DEADLOCK'                # 교착 탐지 (ERROR) · 해소 (OK) · 해소 실패 (ERROR)

TRAFFIC_DEADLOCK = 'traffic/DEADLOCK'          # 교착 탐지 (deadlock_count 에 센다)
TRAFFIC_RESOLVED = 'traffic/RESOLVED'          # 해소 (values: strategy, resolve_time_s)
TRAFFIC_ESCALATED = 'traffic/ESCALATED'        # 전략 1 → 2 전환 · 희생 로봇 교체 (우선순위 역전)
TRAFFIC_UNRESOLVED = 'traffic/UNRESOLVED'      # 해소 실패 (상한 시간 · 후보 소진 · 시도 초과)
TRAFFIC_STALL = 'traffic/STALL'                # 사이클 없는 1대 정체 (경고)


def is_deadlock_event(name: str) -> bool:
    """traffic/DEADLOCK 이면 True (대소문자·접두 무시). RESOLVED 등 다른 이벤트는 False."""
    return name.strip().split('/')[-1].upper() == 'DEADLOCK'


def alert_values(task_id: str = '', robot_id: str = '', **extra: object) -> Dict[str, str]:
    """알림 values: task_id, robots(로봇 1대면 그 id) + 추가 키. 빈 값은 빼고 문자열은 다듬는다."""
    out: Dict[str, object] = {'task_id': task_id, 'robots': robot_id}
    out.update(extra)
    return {k: safe_text(v) for k, v in out.items() if v is not None and v != ''}
