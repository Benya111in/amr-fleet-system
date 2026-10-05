"""
노드 통합(한 프로세스): fleet_manager + fleet_adapter ×2 + 테스트 클라이언트를 한 실행기에서 돌린다.

작업 JSON → PENDING → (지연 큐) → assign_task → IN_PROGRESS → 모의 완료 → COMPLETED,
/fleet/status 카운터, 작업 로그 CSV, 통신 지연 로그, 이벤트 스탬프(명령·접수 시각), 강제 FAILED ·
latched E-stop · 잘못된/적대적 JSON · 서비스 검증 · 고정 작업 head-of-line · 소유권 검사를 확인한다.
결과가 불확실한 호출·로봇 끊김은 test_nodes_faults.py 가 가짜 로봇으로 시험한다.
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
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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
        'auto_complete_after_s': auto_s, 'serve_assign_task': True, 'log_dir': log_dir,
    })) for rid, auto_s in zip(ROBOTS, (1.0, 2.5))]
    client = rclpy.create_node('fleet_test_client')
    got = {'events': [], 'alerts': [], 'status': []}
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    client.create_subscription(Task, '/fleet/task_events', got['events'].append, qos)
    client.create_subscription(DiagnosticArray, '/fleet/alerts', got['alerts'].append, qos)
    client.create_subscription(FleetStatus, '/fleet/status', got['status'].append, 10)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pubs = {
        'request': client.create_publisher(String, '/fleet/task_request', qos),
        # safety_node 와 같은 latched 발행자 (어댑터 구독은 transient_local)
        'estop': client.create_publisher(Bool, f'/{ROBOTS[0]}/safety/estop_active', latched),
        'status_t1': client.create_publisher(Task, f'/{ROBOTS[0]}/task_status', 10),
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
               log_dir=log_dir, adapters=adapters)
    executor.shutdown()
    for node in [manager, client] + adapters:
        node.destroy_node()
    rclpy.try_shutdown()


def _events_of(got, task_id):
    return [(e.status, e.robot_id) for e in got['events'] if e.task_id == task_id]


def _sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _first_event(got, task_id, status):
    return next((e for e in got['events'] if e.task_id == task_id and e.status == status), None)


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

    # 주입한 지연(표본)은 설정 구간 [5, 30] ms 안이어야 한다 — 명세 ≤ 100 ms 를 여기서 확인한다.
    injected = list(manager._latency.history)
    assert len(injected) >= 3
    assert all(LATENCY_MS[0] * 1e-3 <= d <= LATENCY_MS[1] * 1e-3 for d in injected)
    # 통신 지연 로그 [cmd_time, response_time, latency_ms]: 측정값은 주입 하한보다 이르지 않다
    # (상한 쪽은 전송·스케줄링 지연이 더해져 호스트 부하에 좌우되므로 주입값으로 본다).
    lat = []
    for rid in ROBOTS:
        p = path.parent / f'comm_latency_{rid}_{path.stem.split("_")[-1]}.csv'
        if p.exists():
            with open(p, newline='', encoding='utf-8') as f:
                rows = list(csv.reader(f))
            assert rows[0] == ['cmd_time', 'response_time', 'latency_ms']
            lat += [float(r[2]) for r in rows[1:]]
    assert len(lat) >= 3 and min(lat) >= LATENCY_MS[0] - 1.0


def test_task_event_stamps_carry_command_and_reception_time(fleet):
    got = fleet['got']
    for tid in ('it_a', 'it_b', 'it_c'):
        pending = _first_event(got, tid, Task.STATUS_PENDING)
        running = _first_event(got, tid, Task.STATUS_IN_PROGRESS)
        done = _first_event(got, tid, Task.STATUS_COMPLETED)
        received = _sec(pending.header.stamp)
        # PENDING: 명령 없음 (0), 접수 시각 = 이벤트 시각
        assert _sec(pending.pickup_pose.header.stamp) == 0.0
        assert _sec(pending.dropoff_pose.header.stamp) == pytest.approx(received, abs=1e-6)
        # IN_PROGRESS: header = 수락 확인 시각, pickup = 명령(assign_task 요청) 시각, dropoff = 접수
        command = _sec(running.pickup_pose.header.stamp)
        assert received <= command <= _sec(running.header.stamp)
        assert _sec(running.dropoff_pose.header.stamp) == pytest.approx(received, abs=1e-6)
        # 명령 → 수락 확인 사이에는 주입한 송신 지연(≥ 5 ms)이 들어 있다
        assert _sec(running.header.stamp) - command >= LATENCY_MS[0] * 1e-3 - 1e-3
        assert _sec(done.pickup_pose.header.stamp) == pytest.approx(command, abs=1e-6)


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
    n_invalid = len(_alerts(got, 'fleet/INVALID_TASK'))
    fleet['pubs']['request'].publish(String(data='{"pickup": 1}'))
    assert fleet['spin'](lambda: len(_alerts(got, 'fleet/INVALID_TASK')) == n_invalid + 1)
    fleet['pubs']['estop'].publish(Bool(data=False))
    # 해제도 알린다 (OK 레벨 — 대시보드 배너는 WARN 이상만 띄운다)
    assert fleet['spin'](lambda: any(a.level == DiagnosticStatus.OK
                                     for a in _alerts(got, 'fleet/ESTOP')))
    assert fleet['spin'](lambda: fleet['manager']._robots[ROBOTS[0]].state.status == 0)


HOSTILE_PAYLOADS = [
    '{"pickup": {"x": NaN, "y": 0}, "dropoff": {"x": 1, "y": 1}, "item_type": "small"}',
    '{"pickup": {"x": Infinity, "y": 0}, "dropoff": {"x": 1, "y": 1}, "item_type": "small"}',
    '{"pickup": {"x": 1e400, "y": 0}, "dropoff": {"x": 1, "y": 1}, "item_type": "small"}',
    '{"task_id": "far_iso", "pickup": {"x": 1, "y": 0}, "dropoff": {"x": 1, "y": 1}, '
    '"item_type": "small", "deadline": "2100-01-01T00:00:00Z"}',
    '{"task_id": "far_rel", "pickup": {"x": 1, "y": 0}, "dropoff": {"x": 1, "y": 1}, '
    '"item_type": "small", "deadline": 3e9}',
    '{"task_id": "typo_robot", "pickup": {"x": 1, "y": 0}, "dropoff": {"x": 1, "y": 1}, '
    '"item_type": "small", "robot_id": "amr_5"}',
    '[' * 8000 + ']' * 8000,
    '{"pickup": ' + '[' * 900 + ']' * 900 + '}',
    '"not an object"',
    '{"task_id": "<img src=x onerror=alert(1)>", "pickup": {"x": 1, "y": 0}, '
    '"dropoff": {"x": 1, "y": 1}, "item_type": "small"}',
    'x' * 20000,
]


def test_hostile_json_burst_never_kills_the_manager(fleet):
    got, manager = fleet['got'], fleet['manager']
    before = len(_alerts(got, 'fleet/INVALID_TASK'))
    n_tasks = len(manager._sm)
    for text in HOSTILE_PAYLOADS:
        _submit_raw(fleet, text)
    assert fleet['spin'](lambda: len(_alerts(got, 'fleet/INVALID_TASK'))
                         >= before + len(HOSTILE_PAYLOADS))
    assert len(manager._sm) == n_tasks and manager.internal_errors == 0
    new = _alerts(got, 'fleet/INVALID_TASK')[before:]
    assert all('<' not in a.message and '<' not in ''.join(kv.value for kv in a.values)
               for a in new)
    assert {kv.value for a in new for kv in a.values if kv.key == 'task_id'} >= {
        'far_iso', 'far_rel', 'typo_robot'}
    # 버스트 뒤에도 정상 작업은 끝까지 돈다
    _submit(fleet, {'task_id': 'after_burst', 'pickup': {'x': 0.5, 'y': 0.0},
                    'dropoff': {'x': 1.0, 'y': 1.0}, 'item_type': 'small', 'deadline': 600})
    assert fleet['spin'](
        lambda: Task.STATUS_COMPLETED in [s for s, _ in _events_of(got, 'after_burst')])


def _submit_raw(fleet, text):
    fleet['pubs']['request'].publish(String(data=text))


def _call(fleet, req):
    future = fleet['srv'].call_async(req)
    assert fleet['spin'](future.done)
    return future.result()


def test_assign_task_service_applies_the_same_validation(fleet):
    assert fleet['srv'].wait_for_service(timeout_sec=5.0)
    got = fleet['got']
    bad = []
    r1 = AssignTask.Request()
    r1.task.task_id = '<img src=x onerror=alert(1)>'
    r1.task.item_type = 'small'
    bad.append(r1)
    r2 = AssignTask.Request()
    r2.task.task_id = 'srv_nan'
    r2.task.item_type = 'small'
    r2.task.pickup_pose.pose.position.x = float('nan')
    bad.append(r2)
    r3 = AssignTask.Request()
    r3.task.task_id = 'srv_ghost'
    r3.task.item_type = 'small'
    r3.task.robot_id = 'amr_99'
    bad.append(r3)
    r4 = AssignTask.Request()
    r4.task.task_id = 'srv_far'
    r4.task.item_type = 'small'
    r4.task.deadline.sec = 2 ** 31 - 1
    bad.append(r4)
    for req in bad:
        resp = _call(fleet, req)
        assert not resp.success and resp.robot_id == '' and '<' not in resp.message
    assert not any(e.task_id in ('srv_nan', 'srv_ghost', 'srv_far') or '<' in e.task_id
                   for e in got['events'])


def test_pinned_task_for_busy_robot_does_not_block_idle_robot(fleet):
    got = fleet['got']
    t1, t2 = ROBOTS
    _submit(fleet, {'task_id': 'hol_busy', 'pickup': {'x': 1.0, 'y': 0.0},
                    'dropoff': {'x': 2.0, 'y': 0.0}, 'item_type': 'small', 'robot_id': t2})
    assert fleet['spin'](lambda: (Task.STATUS_IN_PROGRESS, t2) in _events_of(got, 'hol_busy'))
    _submit(fleet, {'task_id': 'hol_next', 'priority': 200, 'pickup': {'x': 1.0, 'y': 0.0},
                    'dropoff': {'x': 2.0, 'y': 0.0}, 'item_type': 'small', 'robot_id': t2})
    _submit(fleet, {'task_id': 'hol_auto', 'priority': 0, 'pickup': {'x': 3.0, 'y': 0.0},
                    'dropoff': {'x': 4.0, 'y': 0.0}, 'item_type': 'small'})
    assert fleet['spin'](lambda: (Task.STATUS_IN_PROGRESS, t1) in _events_of(got, 'hol_auto'))
    # 빈 amr_t1 은 amr_t2 의 작업이 끝나기 전에 미지정 작업을 받았다
    auto_start = _first_event(got, 'hol_auto', Task.STATUS_IN_PROGRESS)
    busy_done = _first_event(got, 'hol_busy', Task.STATUS_COMPLETED)
    assert busy_done is None or _sec(auto_start.header.stamp) < _sec(busy_done.header.stamp)
    assert fleet['spin'](lambda: all(Task.STATUS_COMPLETED in [s for s, _ in _events_of(got, t)]
                                     for t in ('hol_busy', 'hol_next', 'hol_auto')))
    assert _events_of(got, 'hol_next')[-1] == (Task.STATUS_COMPLETED, t2)


def test_task_status_from_non_owner_is_ignored_with_alert(fleet):
    got = fleet['got']
    t1, t2 = ROBOTS
    before = len(_alerts(got, 'fleet/TASK_CONFLICT'))
    _submit(fleet, {'task_id': 'own_job', 'pickup': {'x': 1.0, 'y': 0.0},
                    'dropoff': {'x': 2.0, 'y': 0.0}, 'item_type': 'small', 'robot_id': t2})
    assert fleet['spin'](lambda: (Task.STATUS_IN_PROGRESS, t2) in _events_of(got, 'own_job'))
    for status in (Task.STATUS_COMPLETED, Task.STATUS_IN_PROGRESS, Task.STATUS_FAILED):
        fleet['pubs']['status_t1'].publish(Task(task_id='own_job', robot_id=t1, status=status))
    assert fleet['spin'](lambda: len(_alerts(got, 'fleet/TASK_CONFLICT')) >= before + 3)
    conflict = _alerts(got, 'fleet/TASK_CONFLICT')[-1]
    assert conflict.hardware_id == t1
    assert {kv.key: kv.value for kv in conflict.values}['owner'] == t2
    # 진짜 소유자의 완료만 반영된다 (한 번)
    assert fleet['spin'](
        lambda: Task.STATUS_COMPLETED in [s for s, _ in _events_of(got, 'own_job')])
    assert _events_of(got, 'own_job') == [(Task.STATUS_PENDING, t2), (Task.STATUS_IN_PROGRESS, t2),
                                          (Task.STATUS_COMPLETED, t2)]


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
