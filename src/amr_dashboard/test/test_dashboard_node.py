"""
dashboard_node: 실제 rclpy 로 구독 → StateStore, 작업 요청 발행, E-stop 래치/전체 해제, reset_estop 호출.

다른 패키지 테스트와 섞이지 않게 전용 컨텍스트와 /tdash/* 토픽 이름을 쓴다. main()(werkzeug 서버 +
신호 처리)은 기능 실행에서 검증한다. rclpy/amr_msgs 가 없는 순수 pytest 환경에서는 건너뛴다.
"""

import json
import os
import time

import fakes
import pytest

try:
    from amr_msgs.msg import FleetStatus, Task
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import OccupancyGrid
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.task import Future
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger

    from amr_dashboard import estop as es
    from amr_dashboard.action_log import default_log_dir
    from amr_dashboard.dashboard_node import DashboardNode, parse_robot_ids, qos_reliable
    HAVE_ROS = True
except ImportError:  # ROS 환경 밖 (순수 pytest)
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason='rclpy/amr_msgs 없음 (ROS 환경 밖)')

ROBOTS = ['tdash_01', 'tdash_02']
PARAMS = {
    'robot_ids': ROBOTS,
    'topics.fleet_status': '/tdash/fleet/status',
    'topics.fleet_alerts': '/tdash/fleet/alerts',
    'topics.task_events': '/tdash/fleet/task_events',
    'topics.map': '/tdash/map',
    'topics.task_request': '/tdash/fleet/task_request',
    'topics.fleet_estop': '/tdash/fleet/estop',
}
TIMEOUT = 15.0  # [s] 부하가 큰 호스트에서도 DDS 디스커버리가 끝나도록 넉넉히


@pytest.fixture
def ros():
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    overrides = [Parameter(k, value=v) for k, v in PARAMS.items()]
    node = DashboardNode(context=ctx, parameter_overrides=overrides)
    helper = rclpy.create_node('tdash_helper', context=ctx)
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    executor.add_node(helper)
    try:
        yield node, helper, executor
    finally:
        executor.shutdown()
        node.destroy_node()
        helper.destroy_node()
        rclpy.shutdown(context=ctx)


@pytest.fixture
def ros_no_reset():
    """reset_estop_service 가 '' 인 구성 (값 발행만)."""
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    overrides = [Parameter(k, value=v) for k, v in PARAMS.items()]
    overrides.append(Parameter('topics.reset_estop_service', value=''))
    node = DashboardNode(context=ctx, parameter_overrides=overrides)
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown(context=ctx)


def spin_until(executor, cond, timeout=TIMEOUT, each=None):
    """cond() 가 참이 될 때까지 spin (each 가 있으면 매 회 호출: 디스커버리 전 유실 대비 재발행)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if each is not None:
            each()
        executor.spin_once(timeout_sec=0.05)
        if cond():
            return True
    return cond()


def latched_sub(helper, msg_type, topic, sink):
    return helper.create_subscription(
        msg_type, topic, lambda m: sink.append(m), qos_reliable(1, transient_local=True))


def test_subscriptions_feed_state_store(ros):
    node, helper, executor = ros
    store = node.store
    pub_status = helper.create_publisher(FleetStatus, PARAMS['topics.fleet_status'], 10)
    pub_alerts = helper.create_publisher(DiagnosticArray, PARAMS['topics.fleet_alerts'], 10)
    pub_task = helper.create_publisher(Task, PARAMS['topics.task_events'], 10)
    pub_map = helper.create_publisher(OccupancyGrid, PARAMS['topics.map'],
                                      qos_reliable(1, transient_local=True))
    pub_map.publish(fakes.real_grid())   # 래치: 구독자가 늦게 붙어도 받는다

    status = fakes.real_fleet_status(robots=[fakes.real_robot_state('tdash_03')])
    assert spin_until(executor, lambda: store.snapshot()['status'] is not None,
                      each=lambda: pub_status.publish(status))
    assert spin_until(executor, lambda: store.snapshot()['alerts'],
                      each=lambda: pub_alerts.publish(fakes.real_diag_array()))
    assert spin_until(executor, lambda: store.snapshot()['task_events'],
                      each=lambda: pub_task.publish(fakes.real_task(task_id='T-node-1')))
    assert spin_until(executor, lambda: store.map_meta() is not None)

    snap = store.snapshot()
    assert snap['status']['kpi']['tasks_completed'] == 10
    assert snap['status']['robots'][0]['robot_id'] == 'tdash_03'
    assert snap['robot_ids'] == ROBOTS + ['tdash_03']       # FleetStatus 에서 본 로봇 추가
    assert snap['alerts'][-1]['level_name'] == 'ERROR'
    assert snap['task_events'][-1]['task_id'] == 'T-node-1'
    assert snap['map']['width'] == 4 and snap['world']['width'] == pytest.approx(2.0)


def test_publish_task_request(ros):
    node, helper, executor = ros
    task = {'task_id': 'T-node-2', 'priority': 7, 'pickup': {'x': 1.0, 'y': 1.0},
            'dropoff': {'x': 2.0, 'y': 2.0}, 'item_type': 'small'}
    # 구독자(fleet_manager)가 없으면 보내지 않고 0 → web_app 이 503
    assert node._task_pub.get_subscription_count() == 0
    assert node.publish_task_request(json.dumps(task), task) == 0
    received = []
    helper.create_subscription(String, PARAMS['topics.task_request'], received.append, 10)
    assert spin_until(executor, lambda: node._task_pub.get_subscription_count() >= 1)
    assert node.publish_task_request(json.dumps(task), task) == 1
    assert spin_until(executor, lambda: received)
    assert len(received) == 1 and json.loads(received[0].data) == task


def test_estop_latched_and_release_all(ros):
    node, helper, executor = ros
    # 구독자가 없을 때 발행 → 늦게 붙은 transient_local 구독자가 받아야 한다 (래치)
    out = node.publish_estop('tdash_01', True)
    assert out.ok() and out.published == ['/tdash_01/estop'] and out.reset == {}
    r1, r2, fleet = [], [], []
    latched_sub(helper, Bool, '/tdash_01/estop', r1)
    latched_sub(helper, Bool, '/tdash_02/estop', r2)
    latched_sub(helper, Bool, PARAMS['topics.fleet_estop'], fleet)
    assert spin_until(executor, lambda: r1)
    assert r1[-1].data is True and not r2 and not fleet

    out = node.publish_estop('all', True)
    assert out.published == [PARAMS['topics.fleet_estop']]
    assert spin_until(executor, lambda: fleet and fleet[-1].data is True)
    assert r1[-1].data is True  # 전체 정지는 /fleet/estop 만

    # 전체 해제: /fleet/estop 과 로봇별 래치 모두 false + reset_estop (서버 있는 로봇만)
    calls = []

    def on_reset(request, response):
        calls.append(time.time())
        response.success = True
        response.message = 'reset'
        return response

    helper.create_service(Trigger, '/tdash_01/safety/reset_estop', on_reset)
    assert spin_until(executor, lambda: node._reset_client('tdash_01').service_is_ready())
    out = node.publish_estop('all', False)
    assert out.reset == {'tdash_01': es.PENDING, 'tdash_02': es.NO_SERVER}
    assert spin_until(executor, lambda: not out.pending())
    assert out.wait(0.0)
    assert spin_until(executor, lambda: fleet[-1].data is False and r1[-1].data is False
                      and r2 and r2[-1].data is False)
    assert out.reset == {'tdash_01': es.OK, 'tdash_02': es.NO_SERVER}
    assert out.still_latched() == ['tdash_02'] and not out.ok()   # 서버 없음 = 해제 확인 불가

    # 로봇별 해제는 그 로봇만
    node.publish_estop('tdash_01', True)
    assert spin_until(executor, lambda: r1[-1].data is True)
    out = node.publish_estop('tdash_01', False)
    assert spin_until(executor, lambda: r1[-1].data is False and len(calls) == 2
                      and not out.pending())
    assert out.ok() and out.reset == {'tdash_01': es.OK}


def test_release_rejected_by_safety_node(ros):
    node, helper, executor = ros

    def reject(request, response):
        response.success = False
        response.message = 'obstacle within 0.3 m'
        return response

    helper.create_service(Trigger, '/tdash_02/safety/reset_estop', reject)
    assert spin_until(executor, lambda: node._reset_client('tdash_02').service_is_ready())
    out = node.publish_estop('tdash_02', False)
    assert spin_until(executor, lambda: not out.pending())
    assert out.reset == {'tdash_02': es.REJECTED}
    assert out.messages['tdash_02'] == 'obstacle within 0.3 m'
    assert out.still_latched() == ['tdash_02']


def test_invalid_robot_id_does_not_abort_release_all(ros, monkeypatch):
    """리뷰 재현 (verify_ros [2]): 토픽 이름으로 못 쓰는 id 하나가 나머지 로봇의 해제를 막지 않는다."""
    node, helper, executor = ros
    monkeypatch.setattr(node.store, 'known_robot_ids', lambda: ['rv-bad', 'tdash_01'])
    got = []
    latched_sub(helper, Bool, '/tdash_01/estop', got)
    node.publish_estop('tdash_01', True)
    assert spin_until(executor, lambda: got and got[-1].data is True)
    out = node.publish_estop('all', False)
    assert 'rv-bad' in out.publish_failed and 'tdash_01' not in out.publish_failed
    assert out.reset == {'tdash_01': es.NO_SERVER}
    assert spin_until(executor, lambda: got[-1].data is False)
    assert out.still_latched() == ['rv-bad', 'tdash_01']
    assert any(e.startswith('rv-bad: E-stop 값 발행 실패') for e in out.errors())
    # 로봇별 활성화도 마찬가지로 예외 대신 결과로 돌려준다
    bad = node.publish_estop('rv-bad', True)
    assert not bad.ok() and 'rv-bad' in bad.publish_failed


def test_estop_for_robot_seen_later_creates_publisher(ros):
    node, helper, executor = ros
    node.publish_estop('tdash_09', True)   # 설정에 없던 로봇 (FleetStatus 로 나중에 나타난 경우)
    got = []
    latched_sub(helper, Bool, '/tdash_09/estop', got)
    assert spin_until(executor, lambda: got and got[-1].data is True)


class _Logger:
    def __init__(self):
        self.lines = []

    def __getattr__(self, level):
        return lambda msg: self.lines.append((level, msg))


def test_reset_done_callback_paths(ros, monkeypatch):
    node, _, _ = ros
    log = _Logger()
    monkeypatch.setattr(node, 'get_logger', lambda: log)
    out = es.EstopOutcome('tdash_01', False)
    failed = Future()
    failed.set_exception(RuntimeError('boom'))
    node._on_reset_done('tdash_01', failed, out)
    assert out.reset['tdash_01'] == es.ERROR and out.messages['tdash_01'] == 'boom'
    assert log.lines[-1] == ('error', 'tdash_01: reset_estop 실패: boom')
    rejected = Future()
    rejected.set_result(Trigger.Response(success=False, message='cause still present'))
    node._on_reset_done('tdash_02', rejected, out)
    assert out.reset['tdash_02'] == es.REJECTED
    assert log.lines[-1] == ('warn', 'tdash_02: reset_estop → rejected cause still present')
    ok = Future()
    ok.set_result(Trigger.Response(success=True, message='done'))
    node._on_reset_done('tdash_03', ok)                       # 결과 객체 없이도 로그만
    assert log.lines[-1] == ('info', 'tdash_03: reset_estop → ok done')
    # 서버 없음은 WARN 으로 남기고 결과에 no_server
    assert node._call_reset_estop('tdash_02') == es.NO_SERVER
    assert log.lines[-1][0] == 'warn' and 'reset_estop 서비스 없음' in log.lines[-1][1]


def test_reset_done_mixed_results_with_real_logger(ros):
    """실제 rclpy 로거: 성공(info) 뒤 거부(warn) — 한 호출 위치에서 심각도를 바꾸면 ValueError 로 실행기가 죽었다."""
    node, _, _ = ros
    out = es.EstopOutcome('all', False)
    for rid, success in (('tdash_01', True), ('tdash_02', False), ('tdash_01', True)):
        fut = Future()
        fut.set_result(Trigger.Response(success=success, message='m'))
        node._on_reset_done(rid, fut, out)
    assert out.reset == {'tdash_01': es.OK, 'tdash_02': es.REJECTED}


def test_spin_guarded_survives_callback_exceptions(ros):
    """콜백 예외 하나로 스핀 스레드가 끝나지 않는다 (실제 실행기 + 예외 내는 구독)."""
    import threading
    from amr_dashboard.dashboard_node import spin_guarded
    node, helper, executor = ros
    got = []

    def cb(msg):
        got.append(msg.data)
        if msg.data == 'boom':
            raise ValueError('callback failed')
    helper.create_subscription(String, '/tdash/guarded', cb, qos_reliable(10))
    pub = node.create_publisher(String, '/tdash/guarded', qos_reliable(10))
    log = _Logger()
    stop = threading.Event()
    result = {}
    th = threading.Thread(target=lambda: result.setdefault(
        'errors', spin_guarded(executor, log, lambda: not stop.is_set())), daemon=True)
    th.start()
    try:
        deadline = time.monotonic() + TIMEOUT
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        pub.publish(String(data='boom'))
        while 'boom' not in got and time.monotonic() < deadline:
            time.sleep(0.05)
        pub.publish(String(data='after'))
        while 'after' not in got and time.monotonic() < deadline:
            time.sleep(0.05)
        assert th.is_alive() and got == ['boom', 'after']
    finally:
        stop.set()
        th.join(timeout=5.0)
    assert result['errors'] == 1
    assert log.lines[0][0] == 'error' and 'ValueError: callback failed' in log.lines[0][1]


def test_spin_guarded_stops_on_shutdown():
    from rclpy.executors import ShutdownException
    from amr_dashboard.dashboard_node import spin_guarded

    class Executor:
        calls = 0

        def spin_once(self, timeout_sec):
            Executor.calls += 1
            raise ShutdownException()
    assert spin_guarded(Executor(), _Logger(), lambda: True) == 0 and Executor.calls == 1
    assert spin_guarded(Executor(), _Logger(), lambda: False) == 0 and Executor.calls == 1


def test_reset_service_not_configured(ros_no_reset):
    node = ros_no_reset
    out = node.publish_estop('tdash_01', False)
    assert out.reset == {'tdash_01': es.NOT_CONFIGURED} and out.ok()


def test_parameters_and_paths(ros, monkeypatch):
    node, _, _ = ros
    assert node.robot_topic('amr_01', '/estop') == '/amr_01/estop'
    assert node.robot_topic('amr_01', 'safety/reset_estop') == '/amr_01/safety/reset_estop'
    assert node.web_dir.endswith('/share/amr_dashboard/web')
    monkeypatch.setenv('ROS_WS', '/tmp/tdash_ws')
    assert node.log_dir == default_log_dir() == '/tmp/tdash_ws/logs'
    assert node.get_parameter('host').value == '127.0.0.1'
    assert node.get_parameter('port').value == 8080
    assert node.get_parameter('executor_threads').value == 1
    assert node.allowed_hosts == ['localhost', '127.0.0.1', '::1']
    monkeypatch.setenv('AMR_DASHBOARD_TOKEN', 'from-env')
    assert node.api_token == 'from-env'


def test_robot_ids_string_form_and_invalid_ids():
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    overrides = [Parameter('robot_ids', value='r_1, r_2,bad-id,,r_3'),
                 Parameter('allowed_hosts', value=['dash.lan']),
                 Parameter('api_token', value='param-token')]
    node = DashboardNode(context=ctx, parameter_overrides=overrides)
    try:
        assert node.robot_ids == ['r_1', 'r_2', 'r_3']
        assert 'dash.lan' in node.allowed_hosts and node.api_token == 'param-token'
    finally:
        node.destroy_node()
        rclpy.shutdown(context=ctx)
    assert parse_robot_ids(['a', ' b ', '']) == ['a', 'b']
    assert parse_robot_ids(None) == []


# ----- launch: config:= 의 값이 launch 기본값에 가려지지 않는다 -----

def _launch_module():
    import importlib.util
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, '..', 'launch', 'dashboard.launch.py')
    spec = importlib.util.spec_from_file_location('dashboard_launch', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_launch_overrides_only_given_args(tmp_path):
    from launch import LaunchContext
    mod = _launch_module()
    assert mod.overrides_from({'port': '', 'host': '', 'log_dir': '', 'api_token': ''}) == {}
    assert mod.overrides_from({'port': ' 18078 ', 'host': '0.0.0.0'}) == \
        {'port': 18078, 'host': '0.0.0.0'}
    with pytest.raises(ValueError):
        mod.overrides_from({'port': 'eighty'})
    custom = str(tmp_path / 'custom.yaml')
    empty = {'port': '', 'host': '', 'log_dir': '', 'api_token': ''}
    # config:= 만 주면 YAML 값(host/port/log_dir)이 그대로 — launch 기본값이 덮어쓰지 않는다
    assert mod.node_parameters(custom, empty, 'false') == [custom, {'use_sim_time': False}]
    assert mod.node_parameters(custom, dict(empty, port='9000'), 'true') == \
        [custom, {'port': 9000, 'use_sim_time': True}]
    ctx = LaunchContext()
    ctx.launch_configurations.update(dict(empty, config=custom, use_sim_time='false'))
    actions = mod._node(ctx)
    assert len(actions) == 1 and 'dashboard_node' in str(actions[0].node_executable)
    desc = mod.generate_launch_description()
    defaults = {a.name: a.default_value for a in desc.entities if hasattr(a, 'default_value')}
    assert set(defaults) == {'config', 'port', 'host', 'log_dir', 'api_token', 'use_sim_time'}


# ----- main(): 바인드 실패·정상 종료 경로 -----

def test_main_bind_failure_returns_1(monkeypatch):
    from amr_dashboard import dashboard_node as dn

    def boom(*_a, **_k):
        raise OSError(98, 'Address already in use')
    monkeypatch.setattr(dn, 'make_server', boom)
    args = ['--ros-args', '-p', 'robot_ids:=["m_1"]', '-p', 'reset_timeout:=0.1']
    assert dn.main(args=args) == 1
    assert not rclpy.ok()


def test_main_serves_until_shutdown(monkeypatch, tmp_path):
    from amr_dashboard import dashboard_node as dn
    seen = {}

    class FakeServer:
        def serve_forever(self):
            seen['served'] = True

        def shutdown(self):
            seen['shutdown'] = True

    def fake_make_server(host, port, app, threaded):
        seen['bind'] = (host, port, threaded)
        seen['app'] = app
        return FakeServer()

    handlers = {}
    monkeypatch.setattr(dn, 'make_server', fake_make_server)
    monkeypatch.setattr(dn.signal, 'signal', lambda sig, fn: handlers.setdefault(sig, fn))
    assert dn.main(args=['--ros-args', '-p', f'log_dir:={tmp_path}', '-p', 'port:=18099']) == 0
    assert seen['bind'] == ('127.0.0.1', 18099, True) and seen['served']
    client = seen['app'].test_client()
    assert client.get('/api/health', headers={'Host': 'attacker.example'}).status_code == 400
    handlers[dn.signal.SIGINT](2, None)                       # 신호 → 서버 종료 요청
    time.sleep(0.2)
    assert seen.get('shutdown') is True
