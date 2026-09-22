"""
노드 고장 경로(한 프로세스): fleet_manager + 가짜 로봇 3대 (+ latched E-stop 용 어댑터).

가짜 로봇은 robot_state 를 10 Hz 로 내고, assign_task 를 모드대로 처리한다:
accept(즉시 수락) · reject(거절) · slow(delay_s 뒤 수락) · silent(응답하지 않음).
거절 → 재할당, 응답 지연 → 보류 후 늦은 수락(중복 없음), 무응답 → robot_state 확인 후 재할당 +
늦은 보고는 TASK_CONFLICT, 로봇 끊김 → ROBOT_LOST · 작업 FAILED · 재시도, 작업 시간 초과,
마감 초과, 파라미터 콜백, 콜백 예외 격리, use_sim_time 에서의 ISO 마감 변환을 확인한다.
지연 시뮬레이션은 끈다 (결정적인 순서를 보려고).
"""

import datetime as dt
import json
import math
import pathlib
import time

import pytest

try:
    from amr_fleet.fleet_adapter_node import FleetAdapterNode
    from amr_fleet.fleet_manager_node import FleetManagerNode, parse_robot_ids
    from amr_msgs.msg import FleetStatus, RobotState, Task
    from amr_msgs.srv import AssignTask
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.task import Future
    from rclpy.time import Time
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import Bool, String, UInt8
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason='rclpy/amr_msgs 가 없는 환경')

ROBOTS = ('amr_f1', 'amr_f2', 'amr_f3')
GHOST = 'amr_f4'          # 등록만 되어 있고 assign_task 서버가 없는 로봇 (robot_state 는 시험이 직접 낸다)
START = {'amr_f1': (0.0, 0.0), 'amr_f2': (5.0, 0.0), 'amr_f3': (10.0, 0.0)}
IDLE, MOVING = 0, 1


def _params(d):
    return [Parameter(k, value=v) for k, v in d.items()]


class FakeRobot:
    """robot_state 10 Hz + 모드별 assign_task 서버 + task_status 발행기."""

    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.node = rclpy.create_node('fake_robot', namespace='/' + robot_id)
        self.mode = 'accept'
        self.delay_s = 1.0
        self.status = IDLE
        self.current = ''
        self.paused = False
        self.requests = []
        self.waiting = []                   # 응답을 미루는 호출의 Future (정리 때 풀어 준다)
        self.x, self.y = START[robot_id]
        # 서비스 코루틴이 기다리는 동안에도 robot_state 타이머가 돌도록 재진입 그룹을 쓴다
        # (기본 상호 배타 그룹이면 await 중에 같은 노드의 다른 콜백이 막힌다)
        self.cbg = ReentrantCallbackGroup()
        self.pub_state = self.node.create_publisher(RobotState, 'robot_state', 10)
        self.pub_status = self.node.create_publisher(Task, 'task_status', 10)
        self.node.create_service(AssignTask, 'assign_task', self._on_assign,
                                 callback_group=self.cbg)
        self.node.create_timer(0.1, self._tick, callback_group=self.cbg)

    def reset(self):
        self.mode, self.status, self.current, self.paused = 'accept', IDLE, '', False

    def _tick(self):
        if self.paused:
            return
        msg = RobotState(robot_id=self.robot_id, status=self.status, current_task_id=self.current,
                         battery_level=90.0)
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.x, self.y
        self.pub_state.publish(msg)

    async def _on_assign(self, req, resp):
        task_id = req.task.task_id
        self.requests.append(task_id)
        resp.robot_id = self.robot_id
        if self.mode == 'reject':
            resp.success, resp.message = False, 'busy'
            return resp
        if self.mode in ('slow', 'silent'):
            fut = Future()
            self.waiting.append(fut)
            timer = None
            if self.mode == 'slow':
                timer = self.node.create_timer(self.delay_s, lambda: fut.set_result(True),
                                               callback_group=self.cbg)
            ok = await fut                      # silent: 정리 때까지 응답하지 않는다
            if timer is not None:
                self.node.destroy_timer(timer)
            if not ok:
                resp.success, resp.message = False, 'shutdown'
                return resp
        self.current, self.status = task_id, MOVING
        resp.success, resp.message = True, 'accepted'
        return resp

    def report(self, task_id, status):
        self.pub_status.publish(Task(task_id=task_id, robot_id=self.robot_id, status=status))
        if status in (Task.STATUS_COMPLETED, Task.STATUS_FAILED) and self.current == task_id:
            self.current, self.status = '', IDLE


@pytest.fixture(scope='module')
def rig(tmp_path_factory):
    log_dir = str(tmp_path_factory.mktemp('fault_logs'))
    rclpy.init()
    fakes = {rid: FakeRobot(rid) for rid in ROBOTS}
    manager = FleetManagerNode(parameter_overrides=_params({
        'robot_ids': list(ROBOTS) + [GHOST], 'discover_robots': False,
        'allocation_strategy': 'nearest',
        'simulate_latency': False, 'allocation_period_s': 0.1, 'status_period_s': 0.2,
        'assign_timeout_s': 0.5, 'assign_reconcile_s': 0.5, 'reject_backoff_s': 0.3,
        'robot_state_timeout_s': 2.0, 'max_task_retries': 1, 'log_dir': log_dir,
    }))
    client = rclpy.create_node('fault_test_client')
    got = {'events': [], 'alerts': [], 'status': []}
    qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
    client.create_subscription(Task, '/fleet/task_events', got['events'].append, qos)
    client.create_subscription(DiagnosticArray, '/fleet/alerts', got['alerts'].append, qos)
    client.create_subscription(FleetStatus, '/fleet/status', got['status'].append, 10)
    request = client.create_publisher(String, '/fleet/task_request', qos)
    ghost_state = client.create_publisher(RobotState, f'/{GHOST}/robot_state', 10)
    srv = client.create_client(AssignTask, '/fleet/assign_task')
    executor = SingleThreadedExecutor()
    for node in [manager, client] + [f.node for f in fakes.values()]:
        executor.add_node(node)

    def spin_until(pred, timeout=20.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
            if pred():
                return True
        return False

    def spin_for(sec):
        spin_until(lambda: False, timeout=sec)

    def submit(doc):
        request.publish(String(data=json.dumps(doc)))

    assert spin_until(lambda: all(manager._robots[r].state is not None for r in ROBOTS)
                      and request.get_subscription_count() >= 1)
    yield dict(manager=manager, fakes=fakes, got=got, spin=spin_until, spin_for=spin_for,
               submit=submit, executor=executor, client=client, log_dir=log_dir,
               ghost_state=ghost_state, srv=srv)
    for f in fakes.values():                 # 응답을 미루던 서비스 코루틴을 끝낸다
        for fut in f.waiting:
            if not fut.done():
                fut.set_result(False)
    spin_for(0.3)
    executor.shutdown(timeout_sec=2.0)
    for node in [manager, client] + [f.node for f in fakes.values()]:
        node.destroy_node()
    rclpy.try_shutdown()


@pytest.fixture
def fleet(rig):
    """시험마다 가짜 로봇을 초기 상태로 되돌리고, 끝나면 대기/진행 작업이 남지 않았는지 본다."""
    for f in rig['fakes'].values():
        f.reset()
    m = rig['manager']
    for r in m._robots.values():
        r.backoff_until = -1.0e9
    m.set_parameters(_params({'assign_reconcile_s': 0.5, 'reject_backoff_s': 0.3,
                              'task_timeout_s': 600.0, 'max_task_retries': 1,
                              'allocation_batch_window_s': 0.01}))
    rig['spin_for'](0.4)
    yield rig
    counts = m._sm.counts()
    assert counts['pending'] == 0 and counts['in_progress'] == 0, counts
    assert not m._inflight


def _events(got, task_id):
    return [(e.status, e.robot_id) for e in got['events'] if e.task_id == task_id]


def _alerts(got, name, task_id=None):
    out = []
    for a in got['alerts']:
        for s in a.status:
            values = {kv.key: kv.value for kv in s.values}
            if s.name == name and (task_id is None or values.get('task_id') == task_id):
                out.append(s)
    return out


def _task(task_id, robot='', x=0.0, **extra):
    doc = {'task_id': task_id, 'pickup': {'x': x, 'y': 0.0}, 'dropoff': {'x': x + 1.0, 'y': 0.0},
           'item_type': 'small'}
    if robot:
        doc['robot_id'] = robot
    doc.update(extra)
    return doc


def _running(got, task_id, robot):
    return (Task.STATUS_IN_PROGRESS, robot) in _events(got, task_id)


def _finish(fleet, task_id, robot):
    fleet['fakes'][robot].report(task_id, Task.STATUS_COMPLETED)
    assert fleet['spin'](lambda: (Task.STATUS_COMPLETED, robot) in _events(fleet['got'], task_id))


def test_rejection_reassigns_to_next_candidate(fleet):
    f1, f2, f3 = (fleet['fakes'][r] for r in ROBOTS)
    f1.mode, f3.status = 'reject', MOVING          # f3 는 후보 아님, f1 이 가장 가깝지만 거절
    fleet['submit'](_task('rej_1', x=0.0))
    assert fleet['spin'](lambda: _running(fleet['got'], 'rej_1', 'amr_f2'))
    assert 'rej_1' in f1.requests and f2.requests.count('rej_1') == 1
    _finish(fleet, 'rej_1', 'amr_f2')


def test_slow_acceptance_is_waited_for_not_duplicated(fleet):
    got, m = fleet['got'], fleet['manager']
    f1, f2, f3 = (fleet['fakes'][r] for r in ROBOTS)
    m.set_parameters(_params({'assign_reconcile_s': 5.0}))
    f1.mode, f1.delay_s, f3.status = 'slow', 1.2, MOVING
    fleet['submit'](_task('slow_1', x=0.0))
    assert fleet['spin'](lambda: _running(got, 'slow_1', 'amr_f1'))
    assert _alerts(got, 'fleet/ASSIGN_TIMEOUT', 'slow_1')        # 시간 초과는 알리되
    assert 'slow_1' not in f2.requests                          # 다른 로봇에 다시 보내지 않았다
    assert not _alerts(got, 'fleet/TASK_CONFLICT', 'slow_1')
    _finish(fleet, 'slow_1', 'amr_f1')
    assert [s for s, _ in _events(got, 'slow_1')].count(Task.STATUS_IN_PROGRESS) == 1


def test_silent_robot_is_released_after_state_check_and_late_report_conflicts(fleet):
    got = fleet['got']
    f1, f2, f3 = (fleet['fakes'][r] for r in ROBOTS)
    f1.mode, f3.status = 'silent', MOVING
    fleet['submit'](_task('silent_1', x=0.0))
    assert fleet['spin'](lambda: _alerts(got, 'fleet/ASSIGN_TIMEOUT', 'silent_1'))
    assert not _running(got, 'silent_1', 'amr_f2')              # 보류 중에는 재할당하지 않는다
    # robot_state 가 작업을 보고하지 않는 것을 확인한 뒤 다음 후보(f2)로
    assert fleet['spin'](lambda: _running(got, 'silent_1', 'amr_f2'))
    # 뒤늦게 f1 이 시작했다고 알리면 중복 실행 → ERROR 충돌 알림, 소유는 f2 그대로
    f1.report('silent_1', Task.STATUS_IN_PROGRESS)
    assert fleet['spin'](lambda: _alerts(got, 'fleet/TASK_CONFLICT', 'silent_1'))
    alert = _alerts(got, 'fleet/TASK_CONFLICT', 'silent_1')[0]
    assert alert.level == DiagnosticStatus.ERROR and alert.hardware_id == 'amr_f1'
    assert fleet['manager']._sm.get('silent_1').robot_id == 'amr_f2'
    _finish(fleet, 'silent_1', 'amr_f2')


def test_released_task_is_taken_by_late_acceptance_when_still_pending(fleet):
    got, m = fleet['got'], fleet['manager']
    f1, f2, f3 = (fleet['fakes'][r] for r in ROBOTS)
    m.set_parameters(_params({'assign_reconcile_s': 0.2, 'reject_backoff_s': 30.0}))
    f1.mode, f1.delay_s = 'slow', 1.6
    f2.status = f3.status = MOVING                               # 넘겨줄 다른 후보가 없다
    fleet['submit'](_task('late_1', x=0.0))
    assert fleet['spin'](lambda: _alerts(got, 'fleet/ASSIGN_TIMEOUT', 'late_1'))
    assert fleet['spin'](lambda: 'late_1' not in m._inflight)    # 보류를 풀었다 (대기 중)
    assert m._sm.get('late_1').status == Task.STATUS_PENDING
    # 늦은 수락 응답 → 아직 대기 중이므로 그 로봇에서 이어 간다 (충돌 아님)
    assert fleet['spin'](lambda: _running(got, 'late_1', 'amr_f1'))
    assert not _alerts(got, 'fleet/TASK_CONFLICT', 'late_1')
    _finish(fleet, 'late_1', 'amr_f1')

    # robot_state 로 늦은 수락을 알게 되는 경우 (응답도 task_status 도 없이)
    f1.mode = 'silent'
    m._robots['amr_f1'].backoff_until = -1.0e9
    fleet['submit'](_task('late_2', x=0.0))
    assert fleet['spin'](lambda: _alerts(got, 'fleet/ASSIGN_TIMEOUT', 'late_2'))
    assert fleet['spin'](lambda: 'late_2' not in m._inflight)
    f1.current, f1.status = 'late_2', MOVING
    assert fleet['spin'](lambda: _running(got, 'late_2', 'amr_f1'))
    _finish(fleet, 'late_2', 'amr_f1')


def test_robot_state_confirms_an_unanswered_dispatch(fleet):
    got, m = fleet['got'], fleet['manager']
    f1, f2, f3 = (fleet['fakes'][r] for r in ROBOTS)
    m.set_parameters(_params({'assign_reconcile_s': 5.0}))
    f1.mode, f3.status = 'silent', MOVING
    fleet['submit'](_task('rs_1', x=0.0))
    assert fleet['spin'](lambda: _alerts(got, 'fleet/ASSIGN_TIMEOUT', 'rs_1'))
    f1.current, f1.status = 'rs_1', MOVING                      # 응답은 없지만 상태로 수락을 알린다
    assert fleet['spin'](lambda: _running(got, 'rs_1', 'amr_f1'))
    assert 'rs_1' not in f2.requests
    _finish(fleet, 'rs_1', 'amr_f1')


def test_failed_before_start_and_other_conflict_branches(fleet):
    got, m = fleet['got'], fleet['manager']
    f1, f2, f3 = (fleet['fakes'][r] for r in ROBOTS)
    f1.mode, f2.status, f3.status = 'silent', MOVING, MOVING
    fleet['submit'](_task('pf_1', x=0.0))
    assert fleet['spin'](lambda: 'pf_1' in f1.requests)
    f2.report('pf_1', Task.STATUS_IN_PROGRESS)                  # 지금 호출 대상은 f1
    assert fleet['spin'](lambda: _alerts(got, 'fleet/TASK_CONFLICT', 'pf_1'))
    f1.mode = 'accept'
    f1.report('pf_1', Task.STATUS_FAILED)                       # 호출 대상이 시작 전에 실패를 알림
    assert fleet['spin'](lambda: (Task.STATUS_FAILED, 'amr_f1') in _events(got, 'pf_1'))
    assert fleet['spin'](lambda: _running(got, 'pf_1', 'amr_f1'))   # 재시도 1 회 → 다시 f1
    _finish(fleet, 'pf_1', 'amr_f1')
    assert [s for s, _ in _events(got, 'pf_1')] == [
        Task.STATUS_PENDING, Task.STATUS_FAILED, Task.STATUS_PENDING, Task.STATUS_IN_PROGRESS,
        Task.STATUS_COMPLETED]
    n = len(_alerts(got, 'fleet/TASK_CONFLICT', 'pf_1'))
    f2.report('pf_1', Task.STATUS_IN_PROGRESS)                  # 끝난 작업의 시작 보고
    assert fleet['spin'](lambda: len(_alerts(got, 'fleet/TASK_CONFLICT', 'pf_1')) == n + 1)
    f2.report('nope_task', Task.STATUS_COMPLETED)               # 모르는 작업: 로그만
    f1.pub_status.publish(Task(task_id='pf_1', robot_id='amr_f9', status=Task.STATUS_COMPLETED))
    # 보낸 적 없는 로봇의 시작 보고 (대기 중, 호출 없음)
    fleet['submit'](_task('ns_1', robot='amr_f3', x=10.0))
    assert fleet['spin'](lambda: m._sm.get('ns_1') is not None)
    f2.report('ns_1', Task.STATUS_IN_PROGRESS)
    assert fleet['spin'](lambda: _alerts(got, 'fleet/TASK_CONFLICT', 'ns_1'))
    assert m._sm.get('ns_1').status == Task.STATUS_PENDING
    f3.status = IDLE
    assert fleet['spin'](lambda: _running(got, 'ns_1', 'amr_f3'))
    _finish(fleet, 'ns_1', 'amr_f3')
    assert len(_alerts(got, 'fleet/TASK_CONFLICT', 'pf_1')) == n + 1


def test_dropped_and_undeliverable_calls_are_reassigned(fleet):
    got, m = fleet['got'], fleet['manager']
    f1, f2, f3 = (fleet['fakes'][r] for r in ROBOTS)
    f3.status = MOVING
    original = m._latency.sample
    dropped = []

    def drop_once():
        if not dropped:
            dropped.append(True)
            return None                                           # drop_rate 유실 흉내
        return original()

    m._latency.sample = drop_once
    try:
        fleet['submit'](_task('drop_1', x=0.0))
        assert fleet['spin'](lambda: _running(got, 'drop_1', 'amr_f2'))
    finally:
        m._latency.sample = original
    assert dropped and 'drop_1' not in f1.requests             # 유실된 호출은 f1 에 닿지 않았다
    assert not _alerts(got, 'fleet/ASSIGN_TIMEOUT', 'drop_1')  # 보류 없이 바로 재할당
    _finish(fleet, 'drop_1', 'amr_f2')

    # 서버가 없는 로봇(가장 가까움) → 곧바로 다음 후보
    f1.status = MOVING
    ghost = RobotState(robot_id=GHOST, status=IDLE)
    ghost.pose.pose.position.x = -1.0
    for _ in range(3):
        fleet['ghost_state'].publish(ghost)
        fleet['spin_for'](0.1)
    fleet['submit'](_task('ghost_1', x=-1.0))
    assert fleet['spin'](lambda: _running(got, 'ghost_1', 'amr_f2'))
    _finish(fleet, 'ghost_1', 'amr_f2')


def test_invalid_robot_pose_and_error_state(fleet):
    got = fleet['got']
    f1, f2, f3 = (fleet['fakes'][r] for r in ROBOTS)
    f1.x, f3.status = float('nan'), MOVING                    # 자세가 NaN 인 로봇은 후보에서 뺀다
    errors = fleet['manager'].internal_errors
    try:
        fleet['spin_for'](0.3)
        fleet['submit'](_task('nanpose_1', x=0.0))
        assert fleet['spin'](lambda: _running(got, 'nanpose_1', 'amr_f2'))
        assert fleet['manager'].internal_errors == errors
    finally:
        f1.x = 0.0
    _finish(fleet, 'nanpose_1', 'amr_f2')
    f2.status = 5
    assert fleet['spin'](lambda: any(a.hardware_id == 'amr_f2'
                                     for a in _alerts(got, 'fleet/ROBOT_ERROR')))
    f2.status = IDLE


def test_zero_batch_window_and_service_internal_error(fleet):
    got, m = fleet['got'], fleet['manager']
    fleet['fakes']['amr_f3'].status = MOVING
    m.set_parameters(_params({'allocation_batch_window_s': 0.0}))
    fleet['submit'](_task('nowin_1', x=0.0))
    assert fleet['spin'](lambda: _running(got, 'nowin_1', 'amr_f1'))
    _finish(fleet, 'nowin_1', 'amr_f1')

    def boom(req, resp):
        raise RuntimeError('injected')

    m._handle_assign_task = boom
    try:
        req = AssignTask.Request()
        req.task.task_id, req.task.item_type = 'srv_boom', 'small'
        assert fleet['srv'].wait_for_service(timeout_sec=5.0)
        future = fleet['srv'].call_async(req)
        assert fleet['spin'](future.done)
        resp = future.result()
        assert not resp.success and resp.message == 'internal error'
        assert fleet['spin'](lambda: any(
            {kv.key: kv.value for kv in a.values}.get('callback') == '_on_assign_task_srv'
            for a in _alerts(got, 'fleet/INTERNAL_ERROR')))
    finally:
        del m._handle_assign_task


def test_lost_robot_releases_its_pending_dispatch(fleet):
    got, m = fleet['got'], fleet['manager']
    f1, f2, f3 = (fleet['fakes'][r] for r in ROBOTS)
    m.set_parameters(_params({'assign_reconcile_s': 5.0}))
    f1.status = f2.status = MOVING
    f3.mode = 'silent'
    fleet['submit'](_task('lostd_1', x=10.0))
    assert fleet['spin'](lambda: 'lostd_1' in f3.requests)
    f3.paused = True
    assert fleet['spin'](lambda: _alerts(got, 'fleet/ROBOT_LOST', 'lostd_1'))
    assert fleet['spin'](lambda: 'lostd_1' not in m._inflight)
    assert m._sm.get('lostd_1').status == Task.STATUS_PENDING     # 시작 전이라 실패가 아니다
    f2.status = IDLE
    assert fleet['spin'](lambda: _running(got, 'lostd_1', 'amr_f2'))
    f3.paused = False
    _finish(fleet, 'lostd_1', 'amr_f2')


def test_task_status_from_robot_never_asked_is_a_conflict(fleet):
    got = fleet['got']
    f1, f2, f3 = (fleet['fakes'][r] for r in ROBOTS)
    f1.status = f2.status = MOVING
    fleet['submit'](_task('own_2', robot='amr_f3', x=10.0))
    assert fleet['spin'](lambda: _running(got, 'own_2', 'amr_f3'))
    f2.report('own_2', Task.STATUS_IN_PROGRESS)
    f2.report('own_2', Task.STATUS_COMPLETED)
    assert fleet['spin'](lambda: len(_alerts(got, 'fleet/TASK_CONFLICT', 'own_2')) >= 2)
    assert fleet['manager']._sm.get('own_2').status == Task.STATUS_IN_PROGRESS
    _finish(fleet, 'own_2', 'amr_f3')
    fleet['fakes']['amr_f3'].report('own_2', Task.STATUS_COMPLETED)   # 같은 로봇의 중복 완료: 무시
    fleet['spin_for'](0.3)
    assert len(_alerts(got, 'fleet/TASK_CONFLICT', 'own_2')) == 2


def test_lost_robot_goes_offline_fails_task_and_requeues(fleet):
    got, m = fleet['got'], fleet['manager']
    f1, f2, f3 = (fleet['fakes'][r] for r in ROBOTS)
    f1.status = f2.status = MOVING
    fleet['submit'](_task('lost_1', x=10.0))
    assert fleet['spin'](lambda: _running(got, 'lost_1', 'amr_f3'))
    f3.paused = True                                             # robot_state 가 끊긴다
    assert fleet['spin'](lambda: _alerts(got, 'fleet/ROBOT_LOST', 'lost_1'))
    lost = _alerts(got, 'fleet/ROBOT_LOST', 'lost_1')[-1]
    assert lost.level == DiagnosticStatus.ERROR and lost.hardware_id == 'amr_f3'
    assert fleet['spin'](lambda: (Task.STATUS_FAILED, 'amr_f3') in _events(got, 'lost_1'))
    assert m._sm.get('lost_1').status == Task.STATUS_PENDING     # max_task_retries=1 → 재시도
    assert fleet['spin'](lambda: _alerts(got, 'fleet/TASK_REQUEUED', 'lost_1'))
    # FleetStatus 에서 오프라인 로봇은 ERROR 로 보인다 (마지막 상태 IDLE/MOVING 이 아니라)
    assert fleet['spin'](lambda: any(r.robot_id == 'amr_f3' and r.status == 5
                                     for r in got['status'][-1].robots))
    f1.status = IDLE                                             # 다른 로봇이 비면 재할당
    assert fleet['spin'](lambda: _running(got, 'lost_1', 'amr_f1'))
    f3.paused, f3.current, f3.status = False, '', IDLE
    assert fleet['spin'](lambda: any(a.hardware_id == 'amr_f3'
                                     for a in _alerts(got, 'fleet/ROBOT_RECOVERED')))
    assert _alerts(got, 'fleet/ROBOT_RECOVERED')[-1].level == DiagnosticStatus.OK
    _finish(fleet, 'lost_1', 'amr_f1')


def test_task_timeout_and_deadline_alerts(fleet):
    got, m = fleet['got'], fleet['manager']
    f1, f2, f3 = (fleet['fakes'][r] for r in ROBOTS)
    m.set_parameters(_params({'task_timeout_s': 0.6, 'max_task_retries': 0}))
    f1.status = f3.status = MOVING
    fleet['submit'](_task('tmo_1', x=5.0, deadline=0.3))
    assert fleet['spin'](lambda: _running(got, 'tmo_1', 'amr_f2'))
    assert fleet['spin'](lambda: _alerts(got, 'fleet/DEADLINE_MISSED', 'tmo_1'))
    assert fleet['spin'](lambda: (Task.STATUS_FAILED, 'amr_f2') in _events(got, 'tmo_1'))
    assert fleet['spin'](lambda: _alerts(got, 'fleet/TASK_FAILED', 'tmo_1'))   # 이벤트 뒤에 온다
    failed = _alerts(got, 'fleet/TASK_FAILED', 'tmo_1')[-1]
    assert {kv.key: kv.value for kv in failed.values}['reason'] == 'timeout'
    assert not _alerts(got, 'fleet/TASK_REQUEUED', 'tmo_1')      # 재시도 0
    f2.report('tmo_1', Task.STATUS_FAILED)                      # 로봇 쪽 정리 (같은 로봇: 무시)
    fleet['spin_for'](0.3)
    assert not _alerts(got, 'fleet/TASK_CONFLICT', 'tmo_1')


def test_parameter_callback_guards_startup_only_values(fleet):
    m = fleet['manager']
    bad = m.set_parameters(_params({'allocation_strategy': 'hungarian'}))[0]
    assert not bad.successful and '재시작' in bad.reason
    assert not m.set_parameters(_params({'assign_timeout_s': -1.0}))[0].successful
    assert not m.set_parameters(_params({'scheduler.band_width': 8}))[0].successful
    assert m.set_parameters(_params({'max_task_retries': 3}))[0].successful
    assert m._sm.max_retries == 3
    assert parse_robot_ids('amr_01, amr_02,,bad id,amr_01') == ['amr_01', 'amr_02']
    assert parse_robot_ids(['amr_03', ' amr_04 ']) == ['amr_03', 'amr_04']


def test_callback_exception_is_contained(fleet):
    got, m = fleet['got'], fleet['manager']
    original = m._scheduler.ordered

    def boom(now):
        raise RuntimeError('injected')

    before = m.internal_errors
    m._scheduler.ordered = boom
    try:
        fleet['submit'](_task('boom_1', x=5.0))
        # 배치 창·주기 할당 타이머 콜백에서 난 예외를 가드가 삼키고 알린다
        assert fleet['spin'](lambda: m.internal_errors > before and any(
            {kv.key: kv.value for kv in a.values}.get('callback') in (
                '_on_batch_window', '_on_allocation_timer')
            for a in _alerts(got, 'fleet/INTERNAL_ERROR')))
    finally:
        m._scheduler.ordered = original
    # 노드는 계속 돌고, 예외가 사라지면 작업이 진행된다
    assert fleet['spin'](lambda: any(_running(got, 'boom_1', r) for r in ROBOTS))
    rid = m._sm.get('boom_1').robot_id
    _finish(fleet, 'boom_1', rid)


def test_latched_estop_reaches_adapter_started_later(rig):
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = rig['client'].create_publisher(Bool, '/amr_l1/safety/estop_active', latched)
    pub.publish(Bool(data=True))                  # 어댑터가 뜨기 전에 래치
    adapter = FleetAdapterNode(namespace='/amr_l1', parameter_overrides=_params({
        'simulate_latency': False, 'publish_rate_hz': 10.0, 'log_dir': rig['log_dir']}))
    seen = []
    sub = rig['client'].create_subscription(RobotState, '/amr_l1/robot_state', seen.append, 10)
    rig['executor'].add_node(adapter)
    try:
        assert rig['spin'](lambda: adapter._estop and any(s.status == 6 for s in seen))
        # serve_assign_task 기본 false: 실제 서버(task_executor_node)와 겹치지 않는다
        services = adapter.get_service_names_and_types_by_node('fleet_adapter_node', '/amr_l1')
        assert not adapter._serving and '/amr_l1/assign_task' not in [n for n, _ in services]
    finally:
        rig['executor'].remove_node(adapter)
        rig['client'].destroy_subscription(sub)
        rig['client'].destroy_publisher(pub)
        adapter.destroy_node()


def test_adapter_inputs_and_mock_server(rig):
    adapter = FleetAdapterNode(namespace='/amr_a1', parameter_overrides=_params({
        'simulate_latency': False, 'serve_assign_task': True, 'log_dir': rig['log_dir']}))
    warn_only = FleetAdapterNode(namespace='/amr_a2', parameter_overrides=_params({
        'auto_complete_after_s': 1.0, 'log_dir': rig['log_dir']}))   # 서버 없이 모의 완료 → 경고
    try:
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.pose.pose.position.x = 2.5
        adapter._on_odom(odom)
        adapter._on_battery(BatteryState(percentage=0.5))
        adapter._on_phase(String(data='moving'))
        adapter._on_zone(UInt8(data=2))
        st = adapter._build_state()
        assert st.pose.pose.position.x == 2.5 and st.battery_level == 50.0 and st.status == 1
        assert adapter._zone == 2
        adapter._on_phase(String(data='idle'))
        req = AssignTask.Request()
        req.task.task_id = 'ad_1'
        adapter._on_estop(Bool(data=True))
        resp = adapter._on_assign_task(req, AssignTask.Response())
        assert not resp.success and resp.message == 'estop'
        adapter._on_estop(Bool(data=False))
        assert adapter._on_assign_task(req, AssignTask.Response()).success
        req2 = AssignTask.Request()
        req2.task.task_id = 'ad_2'
        assert adapter._on_assign_task(req2, AssignTask.Response()).message == 'busy:ad_1'
        adapter._on_task_status(Task(task_id='ad_1', status=Task.STATUS_COMPLETED))
        assert adapter._current is None
        adapter._handle_assign_task = lambda req, resp: 1 / 0
        resp = adapter._on_assign_task(req2, AssignTask.Response())
        assert not resp.success and resp.message == 'internal error'
        assert warn_only._pub_task_status is not None and not warn_only._serving
    finally:
        adapter.destroy_node()
        warn_only.destroy_node()


def test_iso_deadline_uses_node_clock_under_sim_time(rig, tmp_path):
    sim = FleetManagerNode(namespace='/fleet_sim', parameter_overrides=_params({
        'use_sim_time': True, 'discover_robots': True, 'log_dir': str(tmp_path),
        'comm_latency_ms': [0.0, 150.0], 'allocation_strategy': 'bogus'}))
    try:
        assert sim._strategy.name == 'nearest' and sim._latency.exceeds_spec()
        clock = sim.get_clock()
        sim._check_clock()                     # /clock 없음 → 경고 (노드 시계 0)
        clock.set_ros_time_override(Time(seconds=120))
        sim._check_clock()                     # 시계가 돌면 감시 타이머를 끈다
        assert sim._clock_watchdog.is_canceled()
        assert sim._now() == pytest.approx(120.0)
        sim._discover_robots()                 # /amr_fX/robot_state 토픽으로 등록
        assert set(ROBOTS) <= set(sim._robots)
        iso = dt.datetime.fromtimestamp(time.time() + 60.0).astimezone().isoformat()
        sim._on_task_request(String(data=json.dumps(_task('sim_iso', deadline=iso))))
        sim._on_task_request(String(data=json.dumps(_task('sim_none'))))
        rec = sim._sm.get('sim_iso')
        assert rec.spec.deadline == pytest.approx(180.0, abs=2.0)   # 벽시계 +60 s → sim 180 s
        order = [t.task_id for t in sim._scheduler.ordered(sim._now())]
        assert order == ['sim_iso', 'sim_none']
        clock.set_ros_time_override(Time(seconds=200))
        sim._on_status_timer()
        assert sim._sm.get('sim_iso').deadline_missed
        assert sim._task_log.path_for().name == f'tasks_{dt.date.today():%Y%m%d}.csv'
        # 로그 쓰기 실패는 전이를 막지 않고 로그로만 남는다
        sim._task_log.log_dir = pathlib.Path('/proc/no_such_dir/logs')
        sim._sm.start('sim_none', 'amr_zz', 200.0)
        sim._sm.complete('sim_none', 201.0)
        assert sim._sm.get('sim_none').status == Task.STATUS_COMPLETED
        assert sim.internal_errors == 0 and math.isfinite(sim._now())
    finally:
        sim.destroy_node()
