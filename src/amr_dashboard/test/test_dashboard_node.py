"""
dashboard_node: 실제 rclpy 로 구독 → StateStore, 작업 요청 발행, E-stop 래치/전체 해제, reset_estop 호출.

다른 패키지 테스트와 섞이지 않게 전용 컨텍스트와 /tdash/* 토픽 이름을 쓴다. main()(werkzeug 서버 +
신호 처리)은 기능 실행에서 검증한다. rclpy/amr_msgs 가 없는 순수 pytest 환경에서는 건너뛴다.
"""

import json
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

    from amr_dashboard.action_log import default_log_dir
    from amr_dashboard.dashboard_node import DashboardNode, qos_reliable
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
    received = []
    helper.create_subscription(String, PARAMS['topics.task_request'], received.append, 10)
    assert spin_until(executor, lambda: node._task_pub.get_subscription_count() >= 1)
    task = {'task_id': 'T-node-2', 'priority': 7, 'pickup': {'x': 1.0, 'y': 1.0},
            'dropoff': {'x': 2.0, 'y': 2.0}, 'item_type': 'small'}
    node.publish_task_request(json.dumps(task), task)
    assert spin_until(executor, lambda: received)
    assert json.loads(received[0].data) == task


def test_estop_latched_and_release_all(ros):
    node, helper, executor = ros
    # 구독자가 없을 때 발행 → 늦게 붙은 transient_local 구독자가 받아야 한다 (래치)
    node.publish_estop('tdash_01', True)
    r1, r2, fleet = [], [], []
    latched_sub(helper, Bool, '/tdash_01/estop', r1)
    latched_sub(helper, Bool, '/tdash_02/estop', r2)
    latched_sub(helper, Bool, PARAMS['topics.fleet_estop'], fleet)
    assert spin_until(executor, lambda: r1)
    assert r1[-1].data is True and not r2 and not fleet

    node.publish_estop('all', True)
    assert spin_until(executor, lambda: fleet and fleet[-1].data is True)
    assert r1[-1].data is True  # 전체 정지는 /fleet/estop 만

    # 전체 해제: /fleet/estop 과 로봇별 래치 모두 false + reset_estop 호출 (서버 있는 로봇만)
    calls = []

    def on_reset(request, response):
        calls.append(time.time())
        response.success = True
        response.message = 'reset'
        return response

    helper.create_service(Trigger, '/tdash_01/safety/reset_estop', on_reset)
    assert spin_until(executor, lambda: node._reset_client('tdash_01').service_is_ready())
    node.publish_estop('all', False)
    assert spin_until(executor, lambda: fleet[-1].data is False and r1[-1].data is False
                      and r2 and r2[-1].data is False and calls)
    assert len(calls) == 1   # tdash_02 는 서버가 없어 값 발행만

    # 로봇별 해제는 그 로봇만
    node.publish_estop('tdash_01', True)
    assert spin_until(executor, lambda: r1[-1].data is True)
    node.publish_estop('tdash_01', False)
    assert spin_until(executor, lambda: r1[-1].data is False and len(calls) == 2)


def test_estop_for_robot_seen_later_creates_publisher(ros):
    node, helper, executor = ros
    node.publish_estop('tdash_09', True)   # 설정에 없던 로봇 (FleetStatus 로 나중에 나타난 경우)
    got = []
    latched_sub(helper, Bool, '/tdash_09/estop', got)
    assert spin_until(executor, lambda: got and got[-1].data is True)


def test_reset_done_callback_paths(ros):
    node, _, _ = ros
    failed = Future()
    failed.set_exception(RuntimeError('boom'))
    node._on_reset_done('tdash_01', failed)       # 예외는 로그만 남긴다
    rejected = Future()
    rejected.set_result(Trigger.Response(success=False, message='cause still present'))
    node._on_reset_done('tdash_01', rejected)
    assert node._call_reset_estop('tdash_02') is False  # 서버 없음


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
