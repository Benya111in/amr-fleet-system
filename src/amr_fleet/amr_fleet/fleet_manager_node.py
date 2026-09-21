"""
fleet_manager_node — 중앙 작업 관리자 (components.md §3.6 / §5.6, 명세 4.8 · 4.9).

네임스페이스 /fleet 에서 뜬다.
- Sub  task_request (std_msgs/String, JSON)
- SrvS assign_task (amr_msgs/AssignTask, robot_id 가 비면 자동 할당)
- Sub  /amr_XX/robot_state, /amr_XX/task_status
- SrvC /amr_XX/assign_task (호출 전 송신 지연 큐: 메시지마다 U(comm_latency_ms) ms, drop_rate 유실)
- Pub  task_events (Task, 전이마다) · status (FleetStatus, 1 Hz) · alerts (DiagnosticArray)
- Sub  traffic_events (DiagnosticArray, 교착 카운트)
- 로그 logs/tasks_YYYYmmdd.csv (작업), logs/allocation_YYYYmmdd.csv (할당 최적성 지표)

순수 모듈(task_schema / scheduler / allocation / task_state / kpi / latency)이 로직을 갖고,
이 노드는 ROS 입출력과 타이머만 담당한다.
"""

from __future__ import annotations

import csv
import dataclasses
import datetime as _dt
import functools
import os
import pathlib
import re
from typing import Any, Dict, List, Optional

from amr_fleet.alerts import (
    ALERT_DEADLINE_MISSED, ALERT_ESTOP, ALERT_INVALID_TASK, ALERT_ROBOT_ERROR, ALERT_TASK_FAILED,
    ALERT_TASK_REQUEUED, alert_values, is_deadlock_event,
)
from amr_fleet.allocation import RobotInfo, allocate, available_strategies, create_strategy
from amr_fleet.kpi import STATUS_ERROR, STATUS_ESTOP, STATUS_IDLE, KpiTracker
from amr_fleet.latency import LatencyModel
from amr_fleet.scheduler import SchedulerConfig, TaskScheduler
from amr_fleet.task_msg import float_to_time, spec_from_msg, spec_to_msg
from amr_fleet.task_schema import (
    ITEM_TYPES, STATUS_COMPLETED, STATUS_FAILED, STATUS_IN_PROGRESS, STATUS_NAMES, STATUS_PENDING,
    TaskSchema, TaskSpec, TaskValidationError, load_item_masses,
)
from amr_fleet.task_state import TaskEvent, TaskLogWriter, TaskStateMachine
from amr_fleet.timer_queue import TimerQueue
from amr_msgs.msg import FleetStatus, RobotState, Task
from amr_msgs.srv import AssignTask
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

ROBOT_STATE_RE = re.compile(r'^/([A-Za-z0-9_]+)/robot_state$')
ROBOT_STATE_TYPE = 'amr_msgs/msg/RobotState'
ALLOCATION_LOG_HEADER = [
    'time', 'strategy', 'n_tasks', 'n_robots', 'n_assigned', 'total_travel_distance_m',
    'approach_distance_m', 'makespan_s', 'compute_time_ms', 'assignments',
]


class _Robot:
    """로봇 1대의 통신 핸들과 최근 상태."""

    def __init__(self, robot_id: str):
        self.robot_id = robot_id
        self.state: Optional[RobotState] = None
        self.last_seen = -1.0e9
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
    timeout_at: float               # 이 시각까지 응답이 없으면 보류 → 재할당
    delay_s: Optional[float]        # 주입한 송신 지연 (None = 유실)
    future: Any = None


class FleetManagerNode(Node):
    """JSON 작업 → 스케줄링 → 할당 → 로봇 서비스 호출 → 이벤트/KPI/로그."""

    def __init__(self, **kwargs):
        kwargs.setdefault('namespace', '/fleet')
        super().__init__('fleet_manager_node', **kwargs)
        self._declare_params()
        p = self._param
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
                                    max_retries=int(p('max_task_retries')))
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
        self._outbox = TimerQueue(self, self._send)   # 송신 지연 큐 (task_id 를 해제)
        self._batch_trigger = TimerQueue(self, lambda _: self._allocate_round())
        self._allocating = False
        self._realloc_requested = False
        for rid in [s.strip() for s in str(p('robot_ids')).split(',') if s.strip()]:
            self._add_robot(rid)

        self.create_timer(float(p('allocation_period_s')), self._on_allocation_timer)
        self.create_timer(float(p('status_period_s')), self._on_status_timer)
        if bool(p('discover_robots')):
            self.create_timer(float(p('discovery_period_s')), self._discover_robots)
        self._kpi.start(self._now())
        self.get_logger().info(
            f'fleet_manager 시작: strategy={self._strategy.name}, robots={sorted(self._robots)}, '
            f'comm_latency={self._latency.describe()}, log_dir={self._log_dir}')

    # ------------------------------------------------------------------ 파라미터
    def _declare_params(self) -> None:
        defaults = {
            'robot_ids': '',                 # 쉼표 구분 (예: "amr_01,amr_02"). 비우면 탐색만
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
            'allocation_batch_window_s': 0.2,  # JSON 도착 후 모으는 시간 (0 = 즉시). 서비스는 즉시
            'status_period_s': 1.0,
            'discovery_period_s': 2.0,
            'robot_state_timeout_s': 5.0,    # 이보다 오래된 robot_state 는 후보 제외
            'assign_timeout_s': 3.0,         # 서비스 응답 대기 상한
            'reject_backoff_s': 1.0,         # 거절한 로봇에 재시도까지 대기
            'max_task_retries': 0,           # FAILED → PENDING 재시도 횟수
            'task_timeout_s': 0.0,           # 진행 중 작업 상한 (0 = 없음)
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
            # comm_latency_ms 는 숫자/목록(정수·실수) 어느 쪽이든 받는다
            descriptor = ParameterDescriptor(dynamic_typing=name == 'comm_latency_ms')
            self.declare_parameter(name, value, descriptor)

    def _param(self, name: str):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------ 유틸
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _alert(self, level: bytes, name: str, message: str, hardware_id: str = '',
               values: Optional[Dict[str, object]] = None) -> None:
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
        else:
            self.get_logger().warn(text)

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

    def _discover_robots(self) -> None:
        for name, types in self.get_topic_names_and_types():
            m = ROBOT_STATE_RE.match(name)
            if m and ROBOT_STATE_TYPE in types and m.group(1) != 'fleet':
                self._add_robot(m.group(1))

    # ------------------------------------------------------------------ 입력 콜백
    def _on_task_request(self, msg: String) -> None:
        now = self._now()
        try:
            spec = self._schema.from_json(msg.data, now)
        except TaskValidationError as exc:
            self._alert(DiagnosticStatus.WARN, ALERT_INVALID_TASK, str(exc),
                        values={'payload': msg.data[:200]})
            return
        if self._admit(spec, now):
            self._request_batch_allocation()

    def _request_batch_allocation(self) -> None:
        """
        JSON 도착 후 allocation_batch_window_s 동안 모아 한 라운드로 할당한다.

        몰려 들어온 작업을 한 번에 넘겨야 hungarian 같은 배치 전략이 의미가 있다
        (도착마다 즉시 할당하면 어떤 전략이든 1:N 탐욕 할당이 된다). 0 이면 즉시 할당.
        """
        window = float(self._param('allocation_batch_window_s'))
        if window <= 0.0:
            self._allocate_round()
        elif len(self._batch_trigger) == 0:
            self._batch_trigger.push(None, window)

    def _on_assign_task_srv(self, req: AssignTask.Request, resp: AssignTask.Response):
        now = self._now()
        spec = spec_from_msg(req.task, now, item_masses=self._item_masses)
        if spec.item_type not in ITEM_TYPES:
            resp.success, resp.message = False, f'item_type 은 {ITEM_TYPES} 중 하나'
            return resp
        if not self._admit(spec, now):
            resp.success, resp.message = False, f'중복 task_id {spec.task_id}'
            return resp
        self._allocate_round()
        dispatch = self._inflight.get(spec.task_id)
        rid = dispatch.robot_id if dispatch is not None else ''
        resp.success = True
        resp.robot_id = rid or spec.robot_id
        resp.message = f'{spec.task_id}: ' + ('dispatching' if rid else 'queued')
        return resp

    def _admit(self, spec: TaskSpec, now: float) -> bool:
        if spec.task_id in self._sm:
            self.get_logger().warn(f'중복 task_id 무시: {spec.task_id}')
            return False
        self._sm.add(spec, now)
        self._scheduler.push(spec)
        self.get_logger().info(
            f'작업 수신 {spec.task_id}: priority={spec.priority}, deadline='
            f'{"-" if spec.deadline is None else f"{spec.deadline - now:.0f}s"}, '
            f'item={spec.item_type}({spec.item_mass:g} kg), robot={spec.robot_id or "auto"}')
        return True

    def _on_robot_state(self, robot_id: str, msg: RobotState) -> None:
        now = self._now()
        r = self._robots[robot_id]
        prev = r.state
        r.state, r.last_seen = msg, now
        self._update_kpi_robot(robot_id, now)
        self._sm.update_pose(robot_id, msg.pose.pose.position.x, msg.pose.pose.position.y)
        prev_status = None if prev is None else int(prev.status)
        if msg.status == STATUS_ESTOP and prev_status != STATUS_ESTOP:
            self._alert(DiagnosticStatus.ERROR, ALERT_ESTOP, 'E-stop 활성', robot_id,
                        alert_values(msg.current_task_id, robot_id))
        elif msg.status == STATUS_ERROR and prev_status != STATUS_ERROR:
            self._alert(DiagnosticStatus.WARN, ALERT_ROBOT_ERROR, '로봇 오류 상태', robot_id,
                        alert_values(msg.current_task_id, robot_id))

    def _update_kpi_robot(self, robot_id: str, now: float) -> None:
        """가동률 집계 갱신: 최근 상태 + 진행 중 작업 유무 (robot_state 수신·작업 시작/종료 때)."""
        r = self._robots.get(robot_id)
        if r is None:
            return
        status = int(r.state.status) if r.state is not None else STATUS_IDLE
        self._kpi.update_robot(robot_id, status, now,
                               assigned=self._sm.active_task_of(robot_id) is not None)

    def _on_task_status(self, robot_id: str, msg: Task) -> None:
        now = self._now()
        rec = self._sm.get(msg.task_id)
        if rec is None:
            self.get_logger().warn(f'{robot_id}: 모르는 task_status {msg.task_id!r} 무시')
            return
        if msg.status == STATUS_IN_PROGRESS:
            if rec.status == STATUS_PENDING:
                self._clear_inflight(msg.task_id)
                self._scheduler.remove(msg.task_id)
                self._start_task(msg.task_id, robot_id, now)
        elif msg.status == STATUS_COMPLETED:
            if rec.status == STATUS_IN_PROGRESS:
                self._sm.complete(msg.task_id, now)
                self._kpi.record_completion(now, rec.duration_s or 0.0)
        elif msg.status == STATUS_FAILED:
            if rec.status in (STATUS_PENDING, STATUS_IN_PROGRESS):
                self._fail_task(msg.task_id, now, 'robot_reported', robot_id)

    def _on_traffic_event(self, msg: DiagnosticArray) -> None:
        # traffic_manager 규약: traffic/DEADLOCK(탐지) 만 센다, traffic/RESOLVED 는 세지 않는다
        for st in msg.status:
            if is_deadlock_event(st.name):
                self._kpi.record_deadlock()
                self.get_logger().warn(f'교착 이벤트: {st.name} {st.message} '
                                       f'(count={self._kpi.deadlock_count})')

    # ------------------------------------------------------------------ 상태 전이 → 이벤트
    def _on_task_event(self, ev: TaskEvent) -> None:
        rec = self._sm.get(ev.task_id)
        msg = spec_to_msg(rec.spec, stamp=float_to_time(ev.time))
        msg.status = ev.new_status
        msg.robot_id = rec.robot_id
        self._pub_events.publish(msg)
        self.get_logger().info(f'task_event {ev.task_id}: {ev.name} robot={rec.robot_id or "-"} '
                               f'({ev.reason})')
        if ev.new_status != STATUS_PENDING and rec.robot_id:
            self._update_kpi_robot(rec.robot_id, ev.time)   # 작업 시작/종료 = 가동 구간 경계

    def _fail_task(self, task_id: str, now: float, reason: str, robot_id: str = '') -> None:
        rec = self._sm.get(task_id)
        self._clear_inflight(task_id)
        self._scheduler.remove(task_id)
        self._sm.fail(task_id, now, reason)
        self._kpi.record_failure(now)
        rid = robot_id or rec.robot_id
        self._alert(DiagnosticStatus.ERROR, ALERT_TASK_FAILED, f'{task_id} 실패: {reason}', rid,
                    alert_values(task_id, rid, attempts=rec.attempts, reason=reason))
        if self._sm.can_retry(task_id) and self._sm.retry(task_id, now) is not None:
            self._scheduler.push(rec.spec)
            self._alert(DiagnosticStatus.WARN, ALERT_TASK_REQUEUED,
                        f'{task_id} 재시도 큐 투입 (attempt {rec.attempts + 1})', rid,
                        alert_values(task_id, rid, attempts=rec.attempts))

    # ------------------------------------------------------------------ 할당
    def _candidates(self, now: float) -> List[RobotInfo]:
        timeout = float(self._param('robot_state_timeout_s'))
        out: List[RobotInfo] = []
        for rid, r in self._robots.items():
            st = r.state
            if (st is None or now - r.last_seen > timeout or r.inflight is not None
                    or now < r.backoff_until or st.status != STATUS_IDLE
                    or st.current_task_id or self._sm.active_task_of(rid) is not None):
                continue
            out.append(RobotInfo(rid, st.pose.pose.position.x, st.pose.pose.position.y,
                                 load=r.load))
        return out

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
        if not pending:
            return
        k = int(self._param('allocation_batch_size')) or len(robots)
        result = allocate(self._strategy, pending[:k], robots)
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

        stamp = 큐에 넣는 시각. 로봇이 수신 시각과의 차로 통신 지연을 잰다 (할당 계산·로그 기록
        시간은 통신 지연에 넣지 않는다).
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
        self._inflight[task_id] = _Dispatch(robot_id, req, timeout_at, delay)
        self._robots[robot_id].inflight = task_id
        if delay is None:
            self.get_logger().warn(f'{task_id} → {robot_id}: 호출 유실(drop_rate), 시간 초과 후 재할당')
        elif self._latency.enabled:
            self._outbox.push(task_id, delay, key=robot_id)   # 로봇별 링크 FIFO
            self.get_logger().info(f'{task_id} → {robot_id}: 송신 지연 {delay * 1e3:.1f} ms')
        else:
            self._send(task_id)

    def _send(self, task_id: str) -> None:
        d = self._inflight.get(task_id)
        if d is None or d.future is not None:
            return
        r = self._robots[d.robot_id]
        if not r.client.service_is_ready():
            self._abort_dispatch(task_id, d.robot_id, 'assign_task 서비스 없음')
            return
        d.future = r.client.call_async(d.request)
        d.future.add_done_callback(
            functools.partial(self._on_assign_response, task_id, d.robot_id))

    def _on_assign_response(self, task_id: str, robot_id: str, future) -> None:
        d = self._inflight.get(task_id)
        if d is None or d.future is not future:
            return          # 이미 시간 초과로 보류된 호출의 늦은 응답
        now = self._now()
        self._clear_inflight(task_id)
        try:
            resp = future.result()
        except Exception as exc:  # noqa: B902 — 서비스 호출 실패는 종류를 가리지 않는다
            self._abort_dispatch(task_id, robot_id, f'호출 실패: {exc}')
            return
        if not resp.success:
            self._abort_dispatch(task_id, robot_id, f'거절: {resp.message}')
            return
        rid = resp.robot_id if resp.robot_id in self._robots else robot_id
        self._scheduler.remove(task_id)
        self._start_task(task_id, rid, now)

    def _start_task(self, task_id: str, robot_id: str, now: float) -> None:
        """PENDING → IN_PROGRESS. 이동 거리 적분을 로봇의 마지막 자세에서 시작한다."""
        if self._sm.start(task_id, robot_id, now) is None:
            return          # 서비스 응답과 task_status 가 둘 다 알린 경우 (멱등)
        r = self._robots[robot_id]
        r.load += 1
        if r.state is not None:
            p = r.state.pose.pose.position
            self._sm.update_pose(robot_id, p.x, p.y)

    def _check_dispatch_timeouts(self, now: float) -> None:
        for task_id, d in list(self._inflight.items()):
            if now > d.timeout_at and (d.future is None or not d.future.done()):
                why = '호출 유실' if d.delay_s is None else '응답 시간 초과'
                self._abort_dispatch(task_id, d.robot_id, why)

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
        day = _dt.datetime.fromtimestamp(now).strftime('%Y%m%d')
        path = self._log_dir / f'allocation_{day}.csv'
        new_file = not path.exists() or path.stat().st_size == 0
        with open(path, 'a', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            if new_file:
                w.writerow(ALLOCATION_LOG_HEADER)
            w.writerow([
                self._task_log.format_time(now), m['strategy'], int(m['n_tasks']),
                int(m['n_robots']), int(m['n_assigned']), f'{m["total_travel_distance_m"]:.3f}',
                f'{m["approach_distance_m"]:.3f}', f'{m["makespan_s"]:.3f}',
                f'{m["compute_time_ms"]:.3f}',
                ';'.join(f'{t}={r}' for t, r in assignments.items()),
            ])

    # ------------------------------------------------------------------ 1 Hz 상태
    def _on_status_timer(self) -> None:
        now = self._now()
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
        msg.robots = [r.state for r in self._robots.values() if r.state is not None]
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
