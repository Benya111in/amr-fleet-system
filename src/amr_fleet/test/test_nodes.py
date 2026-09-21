"""
노드 통합(한 프로세스): fleet_manager + fleet_adapter ×2 + 테스트 클라이언트를 한 실행기에서 돌린다.

작업 JSON → PENDING → (지연 큐) → assign_task → IN_PROGRESS → 모의 완료 → COMPLETED,
/fleet/status 카운터, 작업 로그 CSV, 통신 지연 로그, 강제 FAILED·E-stop·잘못된 JSON 알림을 확인한다.
rclpy·amr_msgs 가 없는 환경에서는 건너뛴다 (test_task_msg 와 같은 skipif 방식).
"""

import csv
import json
import time

import pytest

try:
    from amr_fleet.fleet_adapter_node import FleetAdapterNode
    from amr_fleet.fleet_manager_node import FleetManagerNode
    from amr_fleet.timer_queue import TimerQueue
    from amr_msgs.msg import FleetStatus, Task
    from amr_msgs.srv import AssignTask
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason='rclpy/amr_msgs 가 없는 환경')

ROBOTS = ('amr_t1', 'amr_t2')
LATENCY_MS = [5.0, 30.0]


def _params(d):
    return [Parameter(k, value=v) for k, v in d.items()]


@pytest.fixture(scope='module')
def fleet(tmp_path_factory):
    log_dir = str(tmp_path_factory.mktemp('fleet_logs'))
    rclpy.init()
    manager = FleetManagerNode(parameter_overrides=_params({
        'robot_ids': ','.join(ROBOTS), 'discover_robots': False,
        'allocation_strategy': 'hungarian', 'allocation_period_s': 0.1,
        'status_period_s': 0.2, 'comm_latency_ms': LATENCY_MS, 'seed': 7, 'log_dir': log_dir,
    }))
    # amr_t2 는 모의 완료를 늦춰 강제 FAILED 시험이 부하 속에서도 완료와 경합하지 않게 한다
    adapters = [FleetAdapterNode(namespace='/' + rid, parameter_overrides=_params({
        'publish_rate_hz': 10.0, 'comm_latency_ms': LATENCY_MS, 'seed': 3,
        'auto_complete_after_s': auto_s, 'log_dir': log_dir,
    })) for rid, auto_s in zip(ROBOTS, (1.0, 2.5))]
    client = rclpy.create_node('fleet_test_client')
    got = {'events': [], 'alerts': [], 'status': []}
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    client.create_subscription(Task, '/fleet/task_events', got['events'].append, qos)
    client.create_subscription(DiagnosticArray, '/fleet/alerts', got['alerts'].append, qos)
    client.create_subscription(FleetStatus, '/fleet/status', got['status'].append, 10)
    pubs = {
        'request': client.create_publisher(String, '/fleet/task_request', qos),
        'estop': client.create_publisher(Bool, f'/{ROBOTS[0]}/safety/estop_active', 10),
        'status_t2': client.create_publisher(Task, f'/{ROBOTS[1]}/task_status', 10),
    }
    srv = client.create_client(AssignTask, '/fleet/assign_task')
    executor = SingleThreadedExecutor()
    for node in [manager, client] + adapters:
        executor.add_node(node)

    def spin_until(pred, timeout=20.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)
            if pred():
                return True
        return False

    # 로봇 상태가 들어와 할당 후보가 될 때까지
    assert spin_until(lambda: all(r.state is not None for r in manager._robots.values()))
    yield dict(manager=manager, client=client, got=got, pubs=pubs, srv=srv, spin=spin_until,
               log_dir=log_dir)
    executor.shutdown()
    for node in [manager, client] + adapters:
        node.destroy_node()
    rclpy.try_shutdown()


def _events_of(got, task_id):
    return [(e.status, e.robot_id) for e in got['events'] if e.task_id == task_id]


def _submit(fleet, doc):
    fleet['pubs']['request'].publish(String(data=json.dumps(doc)))


def test_json_tasks_complete_with_events_log_and_status(fleet):
    got = fleet['got']
    ids = ['it_a', 'it_b', 'it_c']
    for i, tid in enumerate(ids):
        _submit(fleet, {'task_id': tid, 'priority': 50 * i, 'deadline': 60,
                        'pickup': {'x': float(i), 'y': 0.0}, 'dropoff': {'x': 5.0, 'y': 5.0},
                        'item_type': 'small'})
    done = fleet['spin'](lambda: all(
        any(s == Task.STATUS_COMPLETED for s, _ in _events_of(got, t)) for t in ids))
    assert done, [(_events_of(got, t)) for t in ids]
    for tid in ids:
        statuses = [s for s, _ in _events_of(got, tid)]
        assert statuses == [Task.STATUS_PENDING, Task.STATUS_IN_PROGRESS, Task.STATUS_COMPLETED]
        assert _events_of(got, tid)[-1][1] in ROBOTS

    # 작업 로그: 명세 4.8 컬럼, 완료 3줄
    manager = fleet['manager']
    path = manager._task_log.path_for(time.time())
    with open(path, newline='', encoding='utf-8') as f:
        rows = list(csv.reader(f))
    assert rows[0] == ['task_id', 'robot_id', 'start_time', 'end_time', 'distance_m',
                       'duration_s', 'result']
    by_id = {r[0]: r for r in rows[1:]}
    assert set(ids) <= set(by_id)
    assert all(by_id[t][6] == 'completed' and float(by_id[t][5]) >= 0.5 for t in ids)

    # /fleet/status 가 카운터를 따라온다
    assert fleet['spin'](lambda: got['status'] and got['status'][-1].tasks_completed >= 3)
    st = got['status'][-1]
    assert st.tasks_in_progress == 0 and st.throughput > 0.0 and st.avg_task_duration >= 0.5
    assert len(st.robots) == len(ROBOTS) and 0.0 < st.robot_utilization <= 1.0

    # 통신 지연 로그 [cmd_time, response_time, latency_ms]: 주입 하한보다 이르지 않다.
    # 상한은 전송·스케줄링 지연이 더해져 호스트 부하에 좌우되므로 여기서는 느슨하게만 본다
    # (정밀 측정은 기능 시험에서 부하 기록과 함께 한다).
    lat = []
    for rid in ROBOTS:
        p = path.parent / f'comm_latency_{rid}_{path.stem.split("_")[-1]}.csv'
        if p.exists():
            with open(p, newline='', encoding='utf-8') as f:
                rows = list(csv.reader(f))
            assert rows[0] == ['cmd_time', 'response_time', 'latency_ms']
            lat += [float(r[2]) for r in rows[1:]]
    assert len(lat) >= 3
    assert min(lat) >= LATENCY_MS[0] - 1.0 and max(lat) <= LATENCY_MS[1] + 2000.0


def test_assign_task_service_auto(fleet):
    srv = fleet['srv']
    assert srv.wait_for_service(timeout_sec=5.0)
    req = AssignTask.Request()
    req.task.task_id = 'it_srv'
    req.task.item_type = 'medium'
    req.task.pickup_pose.header.frame_id = 'map'
    req.task.dropoff_pose.pose.position.x = 3.0
    future = srv.call_async(req)
    assert fleet['spin'](future.done)
    resp = future.result()
    assert resp.success and 'it_srv' in resp.message
    got = fleet['got']
    assert fleet['spin'](
        lambda: Task.STATUS_COMPLETED in [s for s, _ in _events_of(got, 'it_srv')])
    dup = fleet['srv'].call_async(req)
    assert fleet['spin'](dup.done)
    assert not dup.result().success          # 같은 task_id 는 거절


def _alerts(got, name):
    return [s for a in got['alerts'] for s in a.status if s.name == name]


def test_forced_failure_raises_alert(fleet):
    got = fleet['got']
    _submit(fleet, {'task_id': 'it_fail', 'pickup': {'x': 1.0, 'y': 1.0},
                    'dropoff': {'x': 2.0, 'y': 2.0}, 'item_type': 'large',
                    'robot_id': ROBOTS[1]})
    assert fleet['spin'](
        lambda: Task.STATUS_IN_PROGRESS in [s for s, _ in _events_of(got, 'it_fail')])
    msg = Task(task_id='it_fail', robot_id=ROBOTS[1], status=Task.STATUS_FAILED)
    fleet['pubs']['status_t2'].publish(msg)
    assert fleet['spin'](lambda: _alerts(got, 'fleet/TASK_FAILED'))
    alert = _alerts(got, 'fleet/TASK_FAILED')[0]
    assert alert.level == DiagnosticStatus.ERROR and alert.hardware_id == ROBOTS[1]
    assert [s for s, _ in _events_of(got, 'it_fail')][-1] == Task.STATUS_FAILED


def test_estop_and_invalid_json_alerts(fleet):
    got = fleet['got']
    fleet['pubs']['estop'].publish(Bool(data=True))
    assert fleet['spin'](lambda: _alerts(got, 'fleet/ESTOP'))
    alert = _alerts(got, 'fleet/ESTOP')[0]
    assert alert.level == DiagnosticStatus.ERROR and alert.hardware_id == ROBOTS[0]
    fleet['pubs']['request'].publish(String(data='{"pickup": 1}'))
    assert fleet['spin'](lambda: _alerts(got, 'fleet/INVALID_TASK'))
    fleet['pubs']['estop'].publish(Bool(data=False))


def test_timer_queue_releases_in_order_on_time(fleet):
    out = []
    q = TimerQueue(fleet['client'], lambda item: out.append((item, q.now())))
    t0 = q.now()
    for item, delay in (('c', 0.15), ('a', 0.05), ('b', 0.10)):
        q.push(item, delay)
    assert len(q) == 3
    assert fleet['spin'](lambda: len(out) == 3, timeout=5.0)
    assert [i for i, _ in out] == ['a', 'b', 'c'] and q.released == 3
    for (_, t), delay in zip(out, (0.05, 0.10, 0.15)):
        assert t - t0 >= delay - 1e-3          # 일찍 풀리지 않는다 (늦음은 호스트 부하 몫)
        assert t - t0 <= delay + 2.0
    q.push('x', 10.0)
    q.clear()
    assert len(q) == 0


def test_traffic_deadlock_events_counted_once(fleet):
    got = fleet['got']
    pub = fleet['client'].create_publisher(DiagnosticArray, '/fleet/traffic_events', 10)
    assert fleet['spin'](lambda: pub.get_subscription_count() >= 1, timeout=10.0)
    before = got['status'][-1].deadlock_count
    arr = DiagnosticArray()
    arr.status = [DiagnosticStatus(level=DiagnosticStatus.WARN, name='traffic/DEADLOCK',
                                   message='cycle amr_t1->amr_t2->amr_t1'),
                  DiagnosticStatus(level=DiagnosticStatus.OK, name='traffic/RESOLVED')]
    pub.publish(arr)
    assert fleet['spin'](lambda: got['status'][-1].deadlock_count == before + 1, timeout=10.0)
