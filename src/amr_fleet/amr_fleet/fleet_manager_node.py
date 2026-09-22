"""
fleet_manager_node — 중앙 작업 관리자 (components.md §3.6 / §5.6, 명세 4.8 · 4.9).

네임스페이스 /fleet 에서 뜬다.
- Sub  task_request (std_msgs/String, JSON). 스키마 + 의미 규칙 위반은 fleet/INVALID_TASK 알림 후 버린다
- SrvS assign_task (amr_msgs/AssignTask, robot_id 가 비면 자동 할당). JSON 경로와 같은 검증, 위반은 success=false
- Sub  /amr_XX/robot_state, /amr_XX/task_status
- SrvC /amr_XX/assign_task (호출 전 송신 지연 큐: 메시지마다 U(comm_latency_ms) ms, drop_rate 유실)
- Pub  task_events (Task, 전이마다) · status (FleetStatus, 1 Hz) · alerts (DiagnosticArray)
- Sub  traffic_events (DiagnosticArray, 교착 카운트)
- 로그 logs/tasks_YYYYmmdd.csv (작업), logs/allocation_YYYYmmdd.csv (할당 최적성 지표). 날짜는 벽시계

시계: 모든 시각(마감·접수·이벤트 stamp)은 노드 시계다 (use_sim_time 이면 /clock). ISO 마감은 접수 때
노드 시계로 옮긴다 (task_schema.parse_deadline).

task_events 스탬프 (components.md §5.6):
- header.stamp              전이 시각. 기존 의미 그대로.
- pickup_pose.header.stamp  명령 시각 = 이 작업을 수락한 assign_task 요청의 stamp (fleet 이 할당을
                            송신 지연 큐에 넣은 시각). 아직 명령이 없으면 (0, 0).
- dropoff_pose.header.stamp 접수 시각 (task_request / assign_task 를 받은 시각).
IN_PROGRESS 이벤트 하나로 "명령 → 첫 움직임"(pickup stamp)과 "접수 → 첫 움직임"(dropoff stamp)을 잰다.

결과가 불확실한 호출 (중복 실행 방지, AssignTask 에는 취소가 없다)
- 유실(drop_rate)·서비스 없음: 로봇이 받지 못했으므로 곧바로 다음 후보로 재할당.
- 보냈는데 assign_timeout_s 안에 응답 없음: fleet/ASSIGN_TIMEOUT 후 그 로봇에 예약한 채 기다린다.
  늦은 성공 응답 · task_status IN_PROGRESS · robot_state.current_task_id 중 하나가 오면 그 로봇에서 시작.
  assign_reconcile_s 가 더 지난 뒤 받은 robot_state 에도 그 작업이 없으면(= 수락 안 함) 예약을 풀고
  재할당한다. 로봇이 끊겨도(robot_state_timeout_s) 푼다.
- 예약을 푼 뒤의 늦은 수락은 작업이 아직 대기 중이면 그 로봇에서 이어 가고, 다른 로봇이 가졌으면
  fleet/TASK_CONFLICT(ERROR) 로 알리고 무시한다.
- task_status 는 소유 로봇(진행 중), 지금 호출 대상, 보낸 적 있는 로봇·고정 로봇(대기 중)의 보고만
  반영한다. 그 밖은 fleet/TASK_CONFLICT 로 알리고 무시한다.

로봇 생존: robot_state 가 robot_state_timeout_s 동안 없으면 오프라인(fleet/ROBOT_LOST, FleetStatus 에서
status=ERROR), 호출 중이던 작업은 재할당, 진행 중이던 작업은 FAILED(robot_lost) → max_task_retries 안에서
재시도. 다시 들어오면 fleet/ROBOT_RECOVERED(OK).

모든 콜백은 예외를 삼키고 fleet/INTERNAL_ERROR 로 알린다 — 입력 하나로 노드가 죽지 않는다.
순수 모듈(task_schema / scheduler / allocation / task_state / kpi / latency)이 로직을 갖고,
이 노드는 ROS 입출력과 타이머만 담당한다.
"""

from __future__ import annotations

import csv
import dataclasses
import datetime as _dt
import functools
import math
import os
import pathlib
import re
import time
import traceback
from typing import Any, Dict, List, Optional

from amr_fleet.alerts import (
    ALERT_ASSIGN_TIMEOUT, ALERT_DEADLINE_MISSED, ALERT_ESTOP, ALERT_INTERNAL_ERROR,
    ALERT_INVALID_TASK, ALERT_ROBOT_ERROR, ALERT_ROBOT_LOST, ALERT_ROBOT_RECOVERED,
    ALERT_TASK_CONFLICT, ALERT_TASK_FAILED, ALERT_TASK_REQUEUED, alert_values, is_deadlock_event,
)
from amr_fleet.allocation import (
    RobotInfo, allocate, available_strategies, create_strategy, select_batch,
)
from amr_fleet.kpi import STATUS_ERROR, STATUS_ESTOP, STATUS_IDLE, KpiTracker
from amr_fleet.latency import LatencyModel
from amr_fleet.scheduler import SchedulerConfig, TaskScheduler
from amr_fleet.task_msg import float_to_time, spec_from_msg, spec_to_msg
from amr_fleet.task_schema import (
    ROBOT_ID_RE, STATUS_COMPLETED, STATUS_FAILED, STATUS_IN_PROGRESS, STATUS_NAMES,
    STATUS_PENDING, TASK_ID_RE, TaskSchema, TaskSpec, TaskValidationError, load_item_masses,
    safe_text,
)
from amr_fleet.task_state import TaskEvent, TaskLogWriter, TaskRecord, TaskStateMachine
from amr_fleet.timer_queue import TimerQueue
from amr_msgs.msg import FleetStatus, RobotState, Task
from amr_msgs.srv import AssignTask
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

ROBOT_STATE_RE = re.compile(r'^/([A-Za-z0-9_]+)/robot_state$')
ROBOT_STATE_TYPE = 'amr_msgs/msg/RobotState'
TASK_ID_HINT_RE = re.compile(r'"task_id"\s*:\s*"([A-Za-z0-9_.:-]{1,64})"')
ALLOCATION_LOG_HEADER = [
    'time', 'strategy', 'n_tasks', 'n_robots', 'n_assigned', 'total_travel_distance_m',
    'approach_distance_m', 'makespan_s', 'compute_time_ms', 'assignments',
]
# 시작할 때 한 번만 읽는 파라미터 — 실행 중 변경(ros2 param set)은 거절한다 (재시작 필요)
STARTUP_ONLY_PARAMS = frozenset({
    'robot_ids', 'discover_robots', 'allocation_strategy', 'nominal_speed', 'simulate_latency',
    'comm_latency_ms', 'drop_rate', 'seed', 'log_dir', 'log_time_format', 'allocation_period_s',
    'status_period_s', 'discovery_period_s', 'throughput_window_s', 'utilization_mode',
    'task_schema_path', 'robot_params_path',
})
# 실행 중 바꿀 수 있는 수치 파라미터와 하한 (매번 읽는다)
RUNTIME_MIN = {
    'allocation_batch_size': 0, 'allocation_batch_window_s': 0.0, 'robot_state_timeout_s': 0.1,
    'assign_timeout_s': 0.01, 'assign_reconcile_s': 0.0, 'reject_backoff_s': 0.0,
    'max_task_retries': 0, 'task_timeout_s': 0.0,
}


def _guarded(fn):
    """콜백 예외를 삼키고 fleet/INTERNAL_ERROR 로 알린다 (노드는 계속 돈다)."""
    @functools.wraps(fn)
    def wrapper(self, *args, **kwargs):
        try:
            return fn(self, *args, **kwargs)
        except Exception as exc:  # noqa: B902 — 어떤 입력도 노드를 죽이지 않는다
            self._internal_error(fn.__name__, exc)
            return None
    return wrapper


def parse_robot_ids(value: Any) -> List[str]:
    """robot_ids 파라미터: 쉼표 구분 문자열("amr_01,amr_02") 또는 문자열 배열. 형식이 틀린 id 는 뺀다."""
    items = value.split(',') if isinstance(value, str) else list(value or [])
    out: List[str] = []
    for item in items:
        rid = str(item).strip()
        if rid and ROBOT_ID_RE.match(rid) and rid not in out:
            out.append(rid)
    return out


class _Robot:
    """로봇 1대의 통신 핸들과 최근 상태."""

    def __init__(self, robot_id: str):
        self.robot_id = robot_id
        self.state: Optional[RobotState] = None
        self.last_seen = -1.0e9
        self.lost = False                   # robot_state_timeout_s 동안 robot_state 없음
        self.load = 0                       # 누적 할당 수 (load_balance)
        self.inflight: Optional[str] = None  # 응답 대기 중인 task_id
        self.backoff_until = -1.0e9
        self.sub_state = None
        self.sub_task = None
        self.client = None


@dataclasses.dataclass
class _Dispatch:
    """응답을 기다리는 assign_task 호출 1건."""

    robot_id: str
    request: AssignTask.Request
    command_time: float             # 명령 시각 = 요청 header.stamp (노드 시계)
    timeout_at: float               # 이 시각까지 응답이 없으면 유실은 재할당, 보낸 호출은 보류
    delay_s: Optional[float]        # 주입한 송신 지연 (None = 유실)
    future: Any = None
    unconfirmed: bool = False       # 응답 시간 초과 — 수락 여부를 확인할 때까지 예약 유지


class FleetManagerNode(Node):
    """JSON 작업 → 스케줄링 → 할당 → 로봇 서비스 호출 → 이벤트/KPI/로그."""

    def __init__(self, **kwargs):
        kwargs.setdefault('namespace', '/fleet')
        super().__init__('fleet_manager_node', **kwargs)
        self._declare_params()
        p = self._param
        self.internal_errors = 0
        self.invalid_tasks = 0
        self.conflicts = 0
        self._latency = LatencyModel(simulate=bool(p('simulate_latency')),
                                     comm_latency_ms=p('comm_latency_ms'),
                                     drop_rate=float(p('drop_rate')), seed=int(p('seed')))
        if self._latency.exceeds_spec():
            self.get_logger().warn(f'comm_latency_ms 상한 {self._latency.max_ms} ms > 명세 100 ms')

        # --- 순수 모듈 ---
        item_masses = load_item_masses(p('robot_params_path') or None)
        self._schema = TaskSchema(p('task_schema_path') or None, item_masses=item_masses)
        self._item_masses = item_masses
        self._scheduler = TaskScheduler(SchedulerConfig(
            band_width=int(p('scheduler.band_width')),
            deadline_weight=float(p('scheduler.deadline_weight')),
            fifo_weight=float(p('scheduler.fifo_weight')),
            age_boost_rate=float(p('scheduler.age_boost_rate')),
            age_boost_max=float(p('scheduler.age_boost_max'))))
        default_log_dir = os.path.join(os.environ.get('ROS_WS', '.'), 'logs')
        self._log_dir = pathlib.Path(p('log_dir') or default_log_dir)
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._task_log = TaskLogWriter(self._log_dir, 'tasks', time_format=p('log_time_format'))
        self._sm = TaskStateMachine(on_event=self._on_task_event, log_writer=self._task_log,
                                    max_retries=int(p('max_task_retries')),
                                    on_log_error=self._on_log_error)
        self._kpi = KpiTracker(throughput_window_s=float(p('throughput_window_s')),
                               utilization_mode=str(p('utilization_mode')))
        strategy = p('allocation_strategy')
        if strategy not in available_strategies():
            self.get_logger().error(
                f'allocation_strategy={strategy!r} 없음. nearest 로 대체 ({available_strategies()})')
            strategy = 'nearest'
        self._strategy = create_strategy(strategy, nominal_speed=float(p('nominal_speed')))

        # --- ROS 인터페이스 (전역 /fleet/*) ---
        qos_events = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self._pub_events = self.create_publisher(Task, 'task_events', qos_events)
        self._pub_status = self.create_publisher(FleetStatus, 'status', 10)
        self._pub_alerts = self.create_publisher(DiagnosticArray, 'alerts', qos_events)
        self.create_subscription(String, 'task_request', self._on_task_request, 50)
        self.create_subscription(DiagnosticArray, 'traffic_events', self._on_traffic_event, 20)
        self.create_service(AssignTask, 'assign_task', self._on_assign_task_srv)

        self._robots: Dict[str, _Robot] = {}
        self._inflight: Dict[str, _Dispatch] = {}   # task_id → 응답 대기 호출
        self._sent: Dict[str, Dict[str, float]] = {}  # task_id → {보낸 로봇: 명령 시각}
        self._outbox = TimerQueue(self, self._on_outbox_release)   # 송신 지연 큐 (task_id)
        self._batch_trigger = TimerQueue(self, self._on_batch_window)
        self._allocating = False
        self._realloc_requested = False
        for rid in parse_robot_ids(p('robot_ids')):
            self._add_robot(rid)

        self.create_timer(float(p('allocation_period_s')), self._on_allocation_timer)
        self.create_timer(float(p('status_period_s')), self._on_status_timer)
        if bool(p('discover_robots')):
            self.create_timer(float(p('discovery_period_s')), self._discover_robots)
        self._clock_watchdog = None
        if bool(p('use_sim_time')):
            # /clock 없이 use_sim_time 이면 노드 시계가 0 에 머물러 타이머가 돌지 않는다 → 벽시계로 감시
            self._clock_watchdog = self.create_timer(
                5.0, self._check_clock, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self._kpi.start(self._now())
        self.get_logger().info(
            f'fleet_manager 시작: strategy={self._strategy.name}, robots={sorted(self._robots)}, '
            f'comm_latency={self._latency.describe()}, '
            f'batch_window={float(p("allocation_batch_window_s")) * 1e3:g} ms, '
            f'use_sim_time={p("use_sim_time")}, log_dir={self._log_dir}')

    # ------------------------------------------------------------------ 파라미터
    def _declare_params(self) -> None:
        defaults = {
            'robot_ids': '',                 # "amr_01,amr_02" 또는 문자열 배열. 비우면 탐색만
            'discover_robots': True,         # /<id>/robot_state 토픽으로 로봇 자동 등록
            'allocation_strategy': 'nearest',
            'allocation_batch_size': 0,      # 0 = 가용 로봇 수만큼 (우선순위 순 상위 K 개를 배치)
            'nominal_speed': 1.0,            # [m/s] 이동 시간 추정용
            'simulate_latency': True,        # false = 지연·유실 없이 바로 호출 (실기)
            'comm_latency_ms': [0.0, 100.0],  # assign_task 호출마다 U(lo, hi) ms (숫자면 [0, 값])
            'drop_rate': 0.0,                # 호출 유실 확률 → assign_timeout_s 뒤 재할당
            'seed': 0,                       # 0 = 무작위
            'log_dir': '',                   # '' = $ROS_WS/logs
            'log_time_format': 'iso',        # iso | epoch
            'allocation_period_s': 0.5,
            'allocation_batch_window_s': 0.01,  # JSON 도착 후 모으는 시간 (0 = 즉시). 서비스는 즉시
            'status_period_s': 1.0,
            'discovery_period_s': 2.0,
            'robot_state_timeout_s': 5.0,    # 이보다 오래 robot_state 가 없으면 후보 제외 + 오프라인
            'assign_timeout_s': 3.0,         # 서비스 응답 대기 상한 (넘으면 보류, 유실은 재할당)
            'assign_reconcile_s': 2.0,       # 보류 후 robot_state 로 "수락 안 함" 을 확인하는 시간
            'reject_backoff_s': 1.0,         # 거절한 로봇에 재시도까지 대기
            'max_task_retries': 0,           # 실패 후 재시도 횟수 (0 = 재할당 없음)
            'task_timeout_s': 600.0,         # 진행 중 작업 상한 (0 = 없음)
            'throughput_window_s': 1800.0,   # 처리량 슬라이딩 창 (30 분)
            'utilization_mode': 'assigned',  # assigned (작업 보유 시간) | moving (이동·도킹·적재)
            'task_schema_path': '',
            'robot_params_path': '',
            'scheduler.band_width': 32,
            'scheduler.deadline_weight': 1.0,
            'scheduler.fifo_weight': 0.0,
            'scheduler.age_boost_rate': 0.5,
            'scheduler.age_boost_max': 64.0,
        }
        for name, value in defaults.items():
            # comm_latency_ms 는 숫자/목록, robot_ids 는 문자열/문자열 배열 어느 쪽이든 받는다
            descriptor = ParameterDescriptor(
                dynamic_typing=name in ('comm_latency_ms', 'robot_ids'))
            self.declare_parameter(name, value, descriptor)

    def _param(self, name: str):
        return self.get_parameter(name).value

    def _on_set_parameters(self, params) -> SetParametersResult:
        """시작 전용 파라미터 변경은 거절, 실행 중 파라미터는 하한 검사 (max_task_retries 는 즉시 반영)."""
        for prm in params:
            if prm.name in STARTUP_ONLY_PARAMS or prm.name.startswith('scheduler.'):
                return SetParametersResult(
                    successful=False, reason=f'{prm.name} 는 시작 시에만 읽는다 (노드 재시작 필요)')
            lo = RUNTIME_MIN.get(prm.name)
            v = prm.value
            if lo is not None and (isinstance(v, bool) or not isinstance(v, (int, float))
                                   or not math.isfinite(v) or v < lo):
                return SetParametersResult(successful=False, reason=f'{prm.name} >= {lo} 이어야 한다')
        for prm in params:
            if prm.name == 'max_task_retries':
                self._sm.max_retries = int(prm.value)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ 유틸
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _alert(self, level: bytes, name: str, message: str, hardware_id: str = '',
               values: Optional[Dict[str, object]] = None) -> None:
        message = safe_text(message, 300)
        status = DiagnosticStatus(level=level, name=name, message=message, hardware_id=hardware_id)
        status.values = [KeyValue(key=k, value=str(v)) for k, v in (values or {}).items()]
        arr = DiagnosticArray()
        arr.header.stamp = self._stamp()
        arr.status = [status]
        self._pub_alerts.publish(arr)
        # rclpy 로거는 호출 위치마다 심각도가 고정이라 분기마다 따로 부른다
        text = f'alert {name} [{hardware_id}]: {message}'
        if level == DiagnosticStatus.ERROR:
            self.get_logger().error(text)
        elif level == DiagnosticStatus.OK:
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)

    def _internal_error(self, where: str, exc: BaseException) -> None:
        self.internal_errors += 1
        try:
            self.get_logger().error(f'{where} 예외 (무시하고 계속): {exc!r}\n{traceback.format_exc()}')
            self._alert(DiagnosticStatus.ERROR, ALERT_INTERNAL_ERROR,
                        f'{where}: {type(exc).__name__}',
                        values={'callback': where, 'error': safe_text(exc, 120)})
        except Exception:  # noqa: B902 — 알림 실패가 다시 콜백을 죽이지 않게
            pass

    def _on_log_error(self, rec: TaskRecord, exc: OSError) -> None:
        self.get_logger().error(f'작업 로그 쓰기 실패 {rec.task_id}: {exc}')

    @_guarded
    def _check_clock(self) -> None:
        if self.get_clock().now().nanoseconds == 0:
            self.get_logger().warn('use_sim_time=true 인데 /clock 을 받지 못했다. 시뮬레이터 없이 '
                                   '단독으로 띄웠다면 use_sim_time:=false 로 실행한다')
        else:
            self._clock_watchdog.cancel()

    # ------------------------------------------------------------------ 로봇 등록
    def _add_robot(self, robot_id: str) -> None:
        if robot_id in self._robots:
            return
        r = _Robot(robot_id)
        r.sub_state = self.create_subscription(
            RobotState, f'/{robot_id}/robot_state',
            functools.partial(self._on_robot_state, robot_id), 10)
        r.sub_task = self.create_subscription(
            Task, f'/{robot_id}/task_status',
            functools.partial(self._on_task_status, robot_id), 20)
        r.client = self.create_client(AssignTask, f'/{robot_id}/assign_task')
        self._robots[robot_id] = r
        self.get_logger().info(f'로봇 등록: {robot_id}')

    @_guarded
    def _discover_robots(self) -> None:
        for name, types in self.get_topic_names_and_types():
            m = ROBOT_STATE_RE.match(name)
            if m and ROBOT_STATE_TYPE in types and m.group(1) != 'fleet':
                self._add_robot(m.group(1))

    # ------------------------------------------------------------------ 작업 접수
    @_guarded
    def _on_task_request(self, msg: String) -> None:
        now = self._now()
        try:
            spec = self._schema.from_json(msg.data, now, wall_now=time.time())
            self._check_admissible(spec)
        except TaskValidationError as exc:
            hint = TASK_ID_HINT_RE.search(msg.data[:4096] if isinstance(msg.data, str) else '')
            self._reject_task('task_request', str(exc), hint.group(1) if hint else '')
            self.get_logger().warn(f'버린 작업 요청 원문(앞 200자): {msg.data[:200]!r}')
            return
        self._admit(spec, now)
        self._request_batch_allocation()

    def _on_assign_task_srv(self, req: AssignTask.Request, resp: AssignTask.Response):
        try:
            return self._handle_assign_task(req, resp)
        except Exception as exc:  # noqa: B902 — 서비스는 항상 응답을 돌려준다
            self._internal_error('_on_assign_task_srv', exc)
            resp.success, resp.robot_id, resp.message = False, '', 'internal error'
            return resp

    def _handle_assign_task(self, req: AssignTask.Request, resp: AssignTask.Response):
        now = self._now()
        spec = spec_from_msg(req.task, now, item_masses=self._item_masses)
        spec.created = now          # 접수 시각은 노드 시계 (클라이언트 stamp 는 다른 시계일 수 있다)
        try:
            self._schema.validate_spec(spec, now)
            self._check_admissible(spec)
        except TaskValidationError as exc:
            hint = req.task.task_id if TASK_ID_RE.match(req.task.task_id or '') else ''
            self._reject_task('assign_task', str(exc), hint)
            resp.success, resp.robot_id, resp.message = False, '', safe_text(exc)
            return resp
        self._admit(spec, now)
        self._allocate_round()      # 응답에 robot_id 를 주려고 배치 창 없이 바로 할당
        d = self._inflight.get(spec.task_id)
        rid = d.robot_id if d is not None else ''
        resp.success = True
        resp.robot_id = rid or spec.robot_id
        resp.message = f'{spec.task_id}: ' + ('dispatching' if rid else 'queued')
        return resp

    def _check_admissible(self, spec: TaskSpec) -> None:
        """검증을 통과한 작업이 이 플릿에 들어올 수 있는지: 새 task_id, 등록된 고정 로봇."""
        if spec.task_id in self._sm:
            raise TaskValidationError(f'중복 task_id: {spec.task_id}')
        if spec.robot_id and spec.robot_id not in self._robots:
            raise TaskValidationError(
                f'robot_id {spec.robot_id} 는 등록되지 않은 로봇 (등록: {",".join(sorted(self._robots))})')

    def _reject_task(self, source: str, reason: str, task_id: str = '') -> None:
        self.invalid_tasks += 1
        self._alert(DiagnosticStatus.WARN, ALERT_INVALID_TASK, reason,
                    values=alert_values(task_id, source=source))

    def _admit(self, spec: TaskSpec, now: float) -> None:
        self._sm.add(spec, now)
        self._scheduler.push(spec)
        self.get_logger().info(
            f'작업 수신 {spec.task_id}: priority={spec.priority}, deadline='
            f'{"-" if spec.deadline is None else f"{spec.deadline - now:.0f}s"}, '
            f'item={spec.item_type}({spec.item_mass:g} kg), robot={spec.robot_id or "auto"}')

    def _request_batch_allocation(self) -> None:
        """
        JSON 도착 후 allocation_batch_window_s 동안 모아 한 라운드로 할당한다.

        한꺼번에 들어온 작업(한 묶음을 연달아 발행하면 수 ms 안에 도착)을 한 번에 넘겨야 hungarian
        같은 배치 전략이 의미가 있다. 창은 응답 예산(명령 → 첫 움직임 평균 200 ms, sequences.md §1
        "플릿 할당 ≤ 20 ms")을 넘지 않게 10 ms 로 둔다. 0 이면 도착마다 즉시 할당.
        """
        window = float(self._param('allocation_batch_window_s'))
        if window <= 0.0:
            self._allocate_round()
        elif len(self._batch_trigger) == 0:
            self._batch_trigger.push(None, window)

    @_guarded
    def _on_batch_window(self, _item) -> None:
        self._allocate_round()

    # ------------------------------------------------------------------ 로봇 입력
    @_guarded
    def _on_robot_state(self, robot_id: str, msg: RobotState) -> None:
        now = self._now()
        r = self._robots[robot_id]
        prev = r.state
        r.state, r.last_seen = msg, now
        if r.lost:
            r.lost = False
            self._alert(DiagnosticStatus.OK, ALERT_ROBOT_RECOVERED, 'robot_state 재수신 — 온라인',
                        robot_id, alert_values(robot_id=robot_id))
        self._update_kpi_robot(robot_id, now)
        pos = msg.pose.pose.position
        if math.isfinite(pos.x) and math.isfinite(pos.y):
            self._sm.update_pose(robot_id, pos.x, pos.y)
        prev_status = None if prev is None else int(prev.status)
        status = int(msg.status)
        if status == STATUS_ESTOP and prev_status != STATUS_ESTOP:
            self._alert(DiagnosticStatus.ERROR, ALERT_ESTOP, 'E-stop 활성', robot_id,
                        alert_values(msg.current_task_id, robot_id))
        elif prev_status == STATUS_ESTOP and status != STATUS_ESTOP:
            self._alert(DiagnosticStatus.OK, ALERT_ESTOP, 'E-stop 해제', robot_id,
                        alert_values(msg.current_task_id, robot_id))
        if status == STATUS_ERROR and prev_status != STATUS_ERROR:
            self._alert(DiagnosticStatus.WARN, ALERT_ROBOT_ERROR, '로봇 오류 상태', robot_id,
                        alert_values(msg.current_task_id, robot_id))
        self._reconcile_from_state(robot_id, msg.current_task_id, now)

    def _reconcile_from_state(self, robot_id: str, current: str, now: float) -> None:
        """robot_state.current_task_id 가 호출 중이거나 예약을 푼 작업이면 그 로봇에서 시작한다."""
        if not current:
            return
        r = self._robots[robot_id]
        if r.inflight == current:
            self._confirm_dispatch(current, robot_id, now, 'robot_state')
            return
        rec = self._sm.get(current)
        if (rec is not None and rec.status == STATUS_PENDING and current not in self._inflight
                and robot_id in self._sent.get(current, {})
                and self._sm.active_task_of(robot_id) is None):
            self._late_start(current, robot_id, now, 'robot_state')

    def _update_kpi_robot(self, robot_id: str, now: float) -> None:
        """가동률 집계 갱신: 최근 상태 + 진행 중 작업 유무 (robot_state 수신·작업 시작/종료 때)."""
        r = self._robots.get(robot_id)
        if r is None:
            return
        status = int(r.state.status) if r.state is not None else STATUS_IDLE
        if r.lost:
            status = STATUS_ERROR
        self._kpi.update_robot(robot_id, status, now,
                               assigned=self._sm.active_task_of(robot_id) is not None)

    @_guarded
    def _on_task_status(self, robot_id: str, msg: Task) -> None:
        now = self._now()
        task_id = msg.task_id
        rec = self._sm.get(task_id)
        if rec is None:
            self.get_logger().warn(f'{robot_id}: 모르는 task_status {safe_text(task_id, 64)!r} 무시')
            return
        if msg.robot_id and msg.robot_id != robot_id:
            self.get_logger().warn(f'{robot_id}/task_status 의 robot_id={msg.robot_id!r} — '
                                   f'토픽의 로봇({robot_id}) 기준으로 판단')
        status = int(msg.status)
        if status == STATUS_IN_PROGRESS:
            self._on_robot_started(task_id, robot_id, now, 'task_status', allow_pinned=True)
        elif status == STATUS_COMPLETED:
            if rec.status == STATUS_IN_PROGRESS and rec.robot_id == robot_id:
                self._sm.complete(task_id, now)
                self._kpi.record_completion(now, rec.duration_s or 0.0)
            elif not (rec.status == STATUS_COMPLETED and rec.robot_id == robot_id):
                self._conflict(rec, robot_id, 'COMPLETED 보고', DiagnosticStatus.WARN)
        elif status == STATUS_FAILED:
            d = self._inflight.get(task_id)
            if rec.status == STATUS_IN_PROGRESS and rec.robot_id == robot_id:
                self._fail_task(task_id, now, 'robot_reported', robot_id)
            elif rec.status == STATUS_PENDING and d is not None and d.robot_id == robot_id:
                self._fail_task(task_id, now, 'robot_reported', robot_id)
            elif not (rec.status == STATUS_FAILED and rec.robot_id == robot_id):
                self._conflict(rec, robot_id, 'FAILED 보고', DiagnosticStatus.WARN)

    def _on_robot_started(self, task_id: str, robot_id: str, now: float, via: str,
                          allow_pinned: bool = False) -> None:
        """로봇이 작업을 시작(수락)했다고 알릴 때: 소유권을 따져 시작 · 무시 · 충돌 알림."""
        rec = self._sm.get(task_id)
        if rec is None:
            return
        if rec.status == STATUS_IN_PROGRESS:
            if rec.robot_id != robot_id:
                self._conflict(rec, robot_id, f'{via}: 이미 {rec.robot_id} 가 진행 중 (중복 실행)',
                               DiagnosticStatus.ERROR)
            return
        if rec.status != STATUS_PENDING:      # 이미 끝남: 같은 로봇의 늦은 알림은 조용히 무시
            if rec.robot_id != robot_id:
                self._conflict(rec, robot_id, f'{via}: 작업이 이미 {STATUS_NAMES[rec.status]}',
                               DiagnosticStatus.ERROR)
            return
        d = self._inflight.get(task_id)
        if d is not None:
            if d.robot_id == robot_id:
                self._confirm_dispatch(task_id, robot_id, now, via)
            else:
                self._conflict(rec, robot_id, f'{via}: 지금 할당 대상은 {d.robot_id}',
                               DiagnosticStatus.ERROR)
            return
        known = robot_id in self._sent.get(task_id, {}) or (
            allow_pinned and robot_id == rec.pinned_robot_id)
        if known and self._sm.active_task_of(robot_id) is None:
            self._late_start(task_id, robot_id, now, via)
        else:
            self._conflict(rec, robot_id, f'{via}: 이 로봇에 맡기지 않은 작업',
                           DiagnosticStatus.ERROR)

    def _late_start(self, task_id: str, robot_id: str, now: float, via: str) -> None:
        """예약을 푼 뒤 도착한 수락: 작업이 아직 대기 중이고 다른 호출이 없으니 그 로봇에서 이어 간다."""
        self.get_logger().warn(f'{task_id}: {robot_id} 의 늦은 수락({via}) — 그 로봇에서 진행')
        self._scheduler.remove(task_id)
        self._start_task(task_id, robot_id, now)

    def _conflict(self, rec: TaskRecord, robot_id: str, what: str, level: bytes) -> None:
        self.conflicts += 1
        self._alert(level, ALERT_TASK_CONFLICT, f'{rec.task_id}: {robot_id} 무시 — {what}',
                    robot_id, alert_values(rec.task_id, robot_id, owner=rec.robot_id,
                                           status=STATUS_NAMES[rec.status]))

    @_guarded
    def _on_traffic_event(self, msg: DiagnosticArray) -> None:
        # traffic_manager 규약: traffic/DEADLOCK(탐지) 만 센다, traffic/RESOLVED 는 세지 않는다
        for st in msg.status:
            if is_deadlock_event(st.name):
                self._kpi.record_deadlock()
                self.get_logger().warn(f'교착 이벤트: {st.name} {safe_text(st.message)} '
                                       f'(count={self._kpi.deadlock_count})')

    # ------------------------------------------------------------------ 상태 전이 → 이벤트
    def _on_task_event(self, ev: TaskEvent) -> None:
        rec = self._sm.get(ev.task_id)
        msg = spec_to_msg(rec.spec, stamp=float_to_time(ev.time))
        msg.status = ev.new_status
        msg.robot_id = rec.robot_id
        # 스탬프 의미는 모듈 설명 참고: pickup = 명령 시각(없으면 0), dropoff = 접수 시각
        msg.pickup_pose.header.stamp = float_to_time(rec.command_time)
        msg.dropoff_pose.header.stamp = float_to_time(rec.created_time)
        self._pub_events.publish(msg)
        self.get_logger().info(f'task_event {ev.task_id}: {ev.name} robot={rec.robot_id or "-"} '
                               f'({ev.reason})')
        if ev.new_status in (STATUS_COMPLETED, STATUS_FAILED):
            self._sent.pop(ev.task_id, None)     # 끝난 작업의 이전 수신 로봇 보고는 더 받지 않는다
        if ev.new_status != STATUS_PENDING and rec.robot_id:
            self._update_kpi_robot(rec.robot_id, ev.time)   # 작업 시작/종료 = 가동 구간 경계

    def _fail_task(self, task_id: str, now: float, reason: str, robot_id: str = '') -> None:
        rec = self._sm.get(task_id)
        self._clear_inflight(task_id)
        self._scheduler.remove(task_id)
        if robot_id and not rec.robot_id:
            rec.robot_id = rec.spec.robot_id = robot_id   # 시작 전 실패도 보고한 로봇을 남긴다
        self._sm.fail(task_id, now, reason)
        self._kpi.record_failure(now)
        rid = rec.robot_id
        self._alert(DiagnosticStatus.ERROR, ALERT_TASK_FAILED, f'{task_id} 실패: {reason}', rid,
                    alert_values(task_id, rid, attempts=rec.attempts, failures=rec.failures,
                                 reason=reason))
        if self._sm.retry(task_id, now) is not None:
            self._scheduler.push(rec.spec)
            self._alert(DiagnosticStatus.WARN, ALERT_TASK_REQUEUED,
                        f'{task_id} 재시도 큐 투입 ({rec.failures}/{self._sm.max_retries})', rid,
                        alert_values(task_id, rid, failures=rec.failures))

    # ------------------------------------------------------------------ 할당
    def _candidates(self, now: float) -> List[RobotInfo]:
        timeout = float(self._param('robot_state_timeout_s'))
        out: List[RobotInfo] = []
        for rid, r in self._robots.items():
            st = r.state
            if (st is None or r.lost or now - r.last_seen > timeout or r.inflight is not None
                    or now < r.backoff_until or st.status != STATUS_IDLE
                    or st.current_task_id or self._sm.active_task_of(rid) is not None):
                continue
            x, y = st.pose.pose.position.x, st.pose.pose.position.y
            if not (math.isfinite(x) and math.isfinite(y)):
                continue            # 자세가 유효하지 않은 로봇은 거리 비용을 만들 수 없다
            out.append(RobotInfo(rid, x, y, load=r.load))
        return out

    @_guarded
    def _on_allocation_timer(self) -> None:
        self._check_dispatch_timeouts(self._now())
        if len(self._batch_trigger) == 0:   # 배치 창이 열려 있으면 그 라운드에 맡긴다
            self._allocate_round()

    def _allocate_round(self) -> None:
        # 거절 → 즉시 재할당이 콜백 안에서 다시 불러도 한 번에 한 라운드만 돈다
        if self._allocating:
            self._realloc_requested = True
            return
        self._allocating = True
        try:
            self._realloc_requested = True
            while self._realloc_requested:
                self._realloc_requested = False
                self._allocate_once(self._now())
        finally:
            self._allocating = False

    def _allocate_once(self, now: float) -> None:
        robots = self._candidates(now)
        if not robots:
            return
        pending = [t for t in self._scheduler.ordered(now) if t.task_id not in self._inflight]
        batch = select_batch(pending, robots, int(self._param('allocation_batch_size')))
        if not batch:
            return
        result = allocate(self._strategy, batch, robots)
        if not result.assignments:
            return
        m = result.metrics
        self.get_logger().info(
            f'할당[{m["strategy"]}] {int(m["n_assigned"])}/{int(m["n_tasks"])} 작업, '
            f'{int(m["n_robots"])} 로봇: 총 이동 {m["total_travel_distance_m"]:.1f} m, '
            f'접근 {m["approach_distance_m"]:.1f} m, makespan {m["makespan_s"]:.1f} s, '
            f'{m["compute_time_ms"]:.2f} ms → {result.assignments}')
        for task_id, rid in result.assignments.items():
            self._dispatch(task_id, rid)
        self._write_allocation_log(now, result.assignments, m)

    def _dispatch(self, task_id: str, robot_id: str) -> None:
        """
        할당 결과를 송신 지연 큐에 넣는다.

        stamp = 큐에 넣는 시각 = 명령 시각. 로봇은 수신 시각과의 차로 통신 지연을 재고, task_events 는
        이 값을 pickup_pose.header.stamp 로 싣는다 (할당 계산·로그 기록 시간은 통신 지연에 넣지 않는다).
        """
        spec = self._scheduler.get(task_id)
        req = AssignTask.Request()
        req.task = spec_to_msg(spec)
        req.task.robot_id = robot_id
        delay = self._latency.sample()
        now = self._now()
        req.task.header.stamp = float_to_time(now)
        req.task.pickup_pose.header.stamp = req.task.dropoff_pose.header.stamp = \
            req.task.header.stamp
        timeout_at = now + (delay or 0.0) + float(self._param('assign_timeout_s'))
        self._inflight[task_id] = _Dispatch(robot_id, req, now, timeout_at, delay)
        self._robots[robot_id].inflight = task_id
        if delay is None:
            self.get_logger().warn(f'{task_id} → {robot_id}: 호출 유실(drop_rate), 시간 초과 후 재할당')
        elif self._latency.enabled:
            self._outbox.push(task_id, delay, key=robot_id)   # 로봇별 링크 FIFO
            self.get_logger().info(f'{task_id} → {robot_id}: 송신 지연 {delay * 1e3:.1f} ms')
        else:
            self._send(task_id)

    @_guarded
    def _on_outbox_release(self, task_id: str) -> None:
        self._send(task_id)

    def _send(self, task_id: str) -> None:
        d = self._inflight.get(task_id)
        if d is None or d.future is not None:
            return
        r = self._robots[d.robot_id]
        if not r.client.service_is_ready():
            self._abort_dispatch(task_id, d.robot_id, 'assign_task 서비스 없음')
            return
        self._sent.setdefault(task_id, {})[d.robot_id] = d.command_time
        d.future = r.client.call_async(d.request)
        d.future.add_done_callback(
            functools.partial(self._on_assign_response, task_id, d.robot_id))

    @_guarded
    def _on_assign_response(self, task_id: str, robot_id: str, future) -> None:
        now = self._now()
        d = self._inflight.get(task_id)
        if d is None or d.future is not future:
            self._on_late_response(task_id, robot_id, future, now)
            return
        self._clear_inflight(task_id)
        try:
            resp = future.result()
        except Exception as exc:  # noqa: B902 — 서비스 호출 실패는 종류를 가리지 않는다
            self._abort_dispatch(task_id, robot_id, f'호출 실패: {exc}')
            return
        if not resp.success:
            self._abort_dispatch(task_id, robot_id, f'거절: {safe_text(resp.message, 80)}')
            return
        if d.unconfirmed:
            self.get_logger().warn(f'{task_id} → {robot_id}: 시간 초과 뒤 수락 응답 — 그 로봇에서 진행')
        rec = self._sm.get(task_id)
        if rec is not None and rec.status == STATUS_PENDING:
            self._scheduler.remove(task_id)
            self._start_task(task_id, robot_id, now)

    def _on_late_response(self, task_id: str, robot_id: str, future, now: float) -> None:
        """예약을 푼(또는 이미 확인된) 호출의 응답. 거절이면 할 일이 없고, 수락이면 소유권을 따진다."""
        try:
            resp = future.result()
        except Exception:  # noqa: B902
            return
        if resp.success:
            self._on_robot_started(task_id, robot_id, now, '늦은 assign_task 응답')

    def _confirm_dispatch(self, task_id: str, robot_id: str, now: float, via: str) -> None:
        """호출 대상 로봇이 (응답 전에) 수락을 알렸다: 예약을 확정하고 시작한다."""
        d = self._inflight.get(task_id)
        if d is None or d.robot_id != robot_id:
            return
        self._clear_inflight(task_id)
        if d.unconfirmed:
            self.get_logger().warn(f'{task_id} → {robot_id}: 응답 없이 {via} 로 수락 확인')
        rec = self._sm.get(task_id)
        if rec is not None and rec.status == STATUS_PENDING:
            self._scheduler.remove(task_id)
            self._start_task(task_id, robot_id, now)

    def _start_task(self, task_id: str, robot_id: str, now: float) -> None:
        """PENDING → IN_PROGRESS. 이동 거리 적분을 로봇의 마지막 자세에서 시작한다."""
        command_time = self._sent.get(task_id, {}).get(robot_id)
        if self._sm.start(task_id, robot_id, now, command_time=command_time) is None:
            return          # 서비스 응답과 task_status 가 둘 다 알린 경우 (멱등)
        r = self._robots[robot_id]
        r.load += 1
        if r.state is not None:
            p = r.state.pose.pose.position
            if math.isfinite(p.x) and math.isfinite(p.y):
                self._sm.update_pose(robot_id, p.x, p.y)

    def _check_dispatch_timeouts(self, now: float) -> None:
        timeout = float(self._param('assign_timeout_s'))
        reconcile = float(self._param('assign_reconcile_s'))
        for task_id, d in list(self._inflight.items()):
            if self._inflight.get(task_id) is not d or now <= d.timeout_at:
                continue
            if d.future is None:        # 보내지 못했다 (유실 또는 송신 큐가 밀림) → 안전하게 재할당
                self._abort_dispatch(task_id, d.robot_id,
                                     '호출 유실' if d.delay_s is None else '송신 지연 큐 초과')
                continue
            if d.future.done():
                continue                # 응답 콜백이 곧 처리한다
            if not d.unconfirmed:
                d.unconfirmed = True
                self._alert(DiagnosticStatus.WARN, ALERT_ASSIGN_TIMEOUT,
                            f'{task_id} → {d.robot_id}: {timeout:g} s 동안 응답 없음 — 수락 여부를 '
                            f'확인할 때까지 재할당 보류', d.robot_id,
                            alert_values(task_id, d.robot_id))
                continue
            r = self._robots[d.robot_id]
            if (r.state is not None and r.last_seen >= d.timeout_at + reconcile
                    and r.state.current_task_id != task_id):
                self._abort_dispatch(task_id, d.robot_id,
                                     f'응답 없음, {reconcile:g} s 뒤 robot_state 에도 작업 없음')

    def _abort_dispatch(self, task_id: str, robot_id: str, why: str) -> None:
        """호출 보류: 그 로봇은 reject_backoff_s 동안 제외하고 곧바로 다음 후보에 할당한다."""
        self._clear_inflight(task_id)
        r = self._robots.get(robot_id)
        if r is not None:
            r.backoff_until = self._now() + float(self._param('reject_backoff_s'))
        self.get_logger().warn(f'{task_id} → {robot_id} 할당 보류 ({why}); 다음 후보로 재할당')
        self._allocate_round()

    def _clear_inflight(self, task_id: str) -> None:
        d = self._inflight.pop(task_id, None)
        if d is not None and self._robots[d.robot_id].inflight == task_id:
            self._robots[d.robot_id].inflight = None

    def _write_allocation_log(self, now: float, assignments: Dict[str, str],
                              m: Dict[str, float]) -> None:
        day = _dt.datetime.fromtimestamp(time.time()).strftime('%Y%m%d')   # 파일 날짜는 벽시계
        path = self._log_dir / f'allocation_{day}.csv'
        try:
            new_file = not path.exists() or path.stat().st_size == 0
            with open(path, 'a', newline='', encoding='utf-8') as f:
                w = csv.writer(f)
                if new_file:
                    w.writerow(ALLOCATION_LOG_HEADER)
                w.writerow([
                    self._task_log.format_time(now), m['strategy'], int(m['n_tasks']),
                    int(m['n_robots']), int(m['n_assigned']),
                    f'{m["total_travel_distance_m"]:.3f}', f'{m["approach_distance_m"]:.3f}',
                    f'{m["makespan_s"]:.3f}', f'{m["compute_time_ms"]:.3f}',
                    ';'.join(f'{t}={r}' for t, r in assignments.items()),
                ])
        except OSError as exc:
            self.get_logger().warn(f'할당 로그 쓰기 실패: {exc}')

    # ------------------------------------------------------------------ 1 Hz 상태
    def _check_liveness(self, now: float) -> None:
        """robot_state 가 끊긴 로봇 → 오프라인 알림, 호출 중 작업 재할당, 진행 중 작업 FAILED(robot_lost)."""
        timeout = float(self._param('robot_state_timeout_s'))
        for rid, r in list(self._robots.items()):
            if r.state is None or r.lost or now - r.last_seen <= timeout:
                continue
            r.lost = True
            rec = self._sm.active_task_of(rid)
            task_id = rec.task_id if rec is not None else (r.inflight or '')
            self._alert(DiagnosticStatus.ERROR, ALERT_ROBOT_LOST,
                        f'{rid}: robot_state {now - r.last_seen:.1f} s 없음 — 오프라인', rid,
                        alert_values(task_id, rid))
            self._update_kpi_robot(rid, now)
            if r.inflight is not None:
                self._abort_dispatch(r.inflight, rid, '로봇 오프라인')
            if rec is not None:
                self._fail_task(rec.task_id, now, 'robot_lost', rid)

    def _robot_states(self) -> List[RobotState]:
        """FleetStatus.robots: 오프라인 로봇은 마지막 자세 · status=ERROR · 작업 없음으로 보인다."""
        out: List[RobotState] = []
        for rid, r in self._robots.items():
            st = r.state
            if st is None:
                continue
            if r.lost:
                off = RobotState()
                off.header = st.header
                off.robot_id = st.robot_id or rid
                off.pose = st.pose
                off.battery_level = st.battery_level
                off.status = STATUS_ERROR
                st = off
            out.append(st)
        return out

    @_guarded
    def _on_status_timer(self) -> None:
        now = self._now()
        self._check_liveness(now)
        for rec in self._sm.check_deadlines(now):
            self._alert(DiagnosticStatus.WARN, ALERT_DEADLINE_MISSED,
                        f'{rec.task_id} 마감 초과 ({now - rec.spec.deadline:.0f} s)', rec.robot_id,
                        alert_values(rec.task_id, rec.robot_id, status=STATUS_NAMES[rec.status]))
        task_timeout = float(self._param('task_timeout_s'))
        if task_timeout > 0.0:
            for rec in self._sm.with_status(STATUS_IN_PROGRESS):
                if rec.start_time is not None and now - rec.start_time > task_timeout:
                    self._fail_task(rec.task_id, now, 'timeout', rec.robot_id)

        snap = self._kpi.snapshot(now, self._sm.counts())
        msg = FleetStatus()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = 'map'
        msg.robots = self._robot_states()
        msg.tasks_pending = snap['tasks_pending']
        msg.tasks_in_progress = snap['tasks_in_progress']
        msg.tasks_completed = snap['tasks_completed']
        msg.tasks_failed = snap['tasks_failed']
        msg.throughput = float(snap['throughput'])
        msg.avg_task_duration = float(snap['avg_task_duration'])
        msg.robot_utilization = float(snap['robot_utilization'])
        msg.deadlock_count = snap['deadlock_count']
        self._pub_status.publish(msg)


def main(args=None):
    """ros2 run amr_fleet fleet_manager_node."""
    rclpy.init(args=args)
    node = FleetManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
