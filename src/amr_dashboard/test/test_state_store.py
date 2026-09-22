"""state_store: 메시지 → JSON dict 변환과 스레드 안전 저장소/구독자 큐."""

import json
import math
import queue
import threading
import time

import fakes
import pytest

from amr_dashboard import state_store as ss

BUILDERS = [pytest.param(fakes.FAKE, id='fake')]
if fakes.HAVE_ROS:
    BUILDERS.append(pytest.param(fakes.REAL, id='real_amr_msgs'))


def test_finite_and_stamp_helpers():
    assert ss.finite(1.5) == 1.5
    assert ss.finite(float('nan'), 0.0) == 0.0
    assert ss.finite(float('inf')) is None
    assert ss.finite('abc', -1) == -1
    assert ss.stamp_to_sec(fakes.fake_stamp(3, 250_000_000)) == pytest.approx(3.25)
    assert ss.level_to_int(b'\x02') == 2
    assert ss.level_to_int(b'') == 0
    assert ss.level_to_int(1) == 1


def test_quat_to_yaw_roundtrip():
    for yaw in (0.0, 0.5, -1.0, 3.0, -3.1):
        x, y, z, w = fakes.yaw_to_quat(yaw)
        q = fakes.NS(x=x, y=y, z=z, w=w)
        assert ss.quat_to_yaw(q) == pytest.approx(yaw, abs=1e-9)


@pytest.mark.parametrize('b', BUILDERS)
def test_robot_state_to_dict(b):
    d = ss.robot_state_to_dict(b['robot_state']('amr_04', 10.0, 20.0, 1.0, 55.0, 'T-7', 6))
    assert d['robot_id'] == 'amr_04'
    assert d['pose']['x'] == pytest.approx(10.0)
    assert d['pose']['y'] == pytest.approx(20.0)
    assert d['pose']['yaw'] == pytest.approx(1.0, abs=1e-6)
    assert d['pose']['frame_id'] == 'map'
    assert d['battery_level'] == pytest.approx(55.0)
    assert d['current_task_id'] == 'T-7'
    assert d['status'] == 6 and d['status_name'] == 'ESTOP'
    assert d['stamp'] == pytest.approx(100.5)
    json.dumps(d, allow_nan=False)


@pytest.mark.parametrize('b', BUILDERS)
def test_fleet_status_to_dict(b):
    d = ss.fleet_status_to_dict(b['fleet_status'](), received_at=123.0)
    assert d['received_at'] == 123.0
    assert [r['robot_id'] for r in d['robots']] == ['amr_01', 'amr_02']
    assert d['robots'][1]['status_name'] == 'IDLE'
    assert d['kpi'] == {
        'tasks_pending': 3, 'tasks_in_progress': 2, 'tasks_completed': 10, 'tasks_failed': 1,
        'throughput': pytest.approx(12.5), 'avg_task_duration': pytest.approx(48.0),
        'robot_utilization': pytest.approx(0.8), 'deadlock_count': 0,
    }
    json.dumps(d, allow_nan=False)


@pytest.mark.parametrize('b', BUILDERS)
def test_diagnostic_array_to_dicts(b):
    alerts = ss.diagnostic_array_to_dicts(b['diag_array'](), received_at=5.0)
    assert len(alerts) == 1
    a = alerts[0]
    assert a['level'] == 2 and a['level_name'] == 'ERROR'
    assert a['name'] == 'task_failed'
    assert a['message'] == '도킹 3회 실패'
    assert a['hardware_id'] == 'amr_03'
    assert a['values'] == {'task_id': 'T-9'}
    assert a['received_at'] == 5.0


@pytest.mark.parametrize('b', BUILDERS)
def test_task_to_dict(b):
    d = ss.task_to_dict(b['task']('T-42', '', 250, 3, 'large', 25.0, 2000), received_at=1.0)
    assert d['task_id'] == 'T-42'
    assert d['robot_id'] == ''
    assert d['priority'] == 250
    assert d['deadline'] == pytest.approx(2000.0)
    assert d['pickup_pose']['x'] == pytest.approx(5.0)
    assert d['dropoff_pose']['yaw'] == pytest.approx(1.5, abs=1e-6)
    assert d['item_type'] == 'large' and d['item_mass'] == pytest.approx(25.0)
    assert d['status'] == 3 and d['status_name'] == 'FAILED'


@pytest.mark.parametrize('b', BUILDERS)
def test_occupancy_grid_meta(b):
    meta = ss.occupancy_grid_meta(b['grid']())
    assert meta['width'] == 4 and meta['height'] == 3
    assert meta['resolution'] == pytest.approx(0.5)
    assert meta['origin'] == {'x': pytest.approx(1.0), 'y': pytest.approx(2.0), 'yaw': 0.0}
    assert meta['frame_id'] == 'map'


def test_unknown_status_names():
    d = ss.robot_state_to_dict(fakes.fake_robot_state(status=99))
    assert d['status_name'] == 'UNKNOWN'
    t = ss.task_to_dict(fakes.fake_task(status=42))
    assert t['status_name'] == 'UNKNOWN'


def test_nan_values_are_sanitised():
    msg = fakes.fake_fleet_status(throughput=float('nan'), utilization=float('inf'))
    d = ss.fleet_status_to_dict(msg)
    assert d['kpi']['throughput'] == 0.0
    assert d['kpi']['robot_utilization'] == 0.0
    json.dumps(d, allow_nan=False)


# ----- StateStore -----

@pytest.mark.parametrize('b', BUILDERS)
def test_store_update_and_snapshot(b):
    store = ss.StateStore(robot_ids=['amr_01'])
    q = store.subscribe()
    store.update_status(b['fleet_status']())
    store.add_alerts(b['diag_array']())
    store.add_task_event(b['task']())
    store.update_map(b['grid']())
    snap = store.snapshot()
    assert snap['status']['kpi']['tasks_completed'] == 10
    assert len(snap['alerts']) == 1
    assert len(snap['task_events']) == 1
    assert snap['tasks']['T-1']['status_name'] == 'IN_PROGRESS'
    assert snap['map']['width'] == 4
    # 지도를 받으면 월드 범위가 지도 메타데이터를 따른다
    assert snap['world'] == {'origin_x': pytest.approx(1.0), 'origin_y': pytest.approx(2.0),
                             'width': pytest.approx(2.0), 'height': pytest.approx(1.5)}
    # 설정된 로봇 + 상태에서 본 로봇
    assert snap['robot_ids'] == ['amr_01', 'amr_02']
    assert snap['sse_clients'] == 1
    events = [q.get_nowait()[0] for _ in range(4)]
    assert events == ['status', 'alerts', 'task_event', 'map_updated']
    json.dumps(snap, allow_nan=False)


def test_store_default_world_and_empty_snapshot():
    store = ss.StateStore()
    snap = store.snapshot()
    assert snap['world'] == ss.DEFAULT_WORLD
    assert snap['status'] is None and snap['map'] is None
    assert snap['estops'] == {'all': False}
    assert store.map_png() is None and store.map_json() is None and store.map_meta() is None
    assert store.known_robot_ids() == []


def test_store_map_png_and_json_cached():
    store = ss.StateStore()
    store.update_map(fakes.fake_grid())
    png1 = store.map_png()
    assert png1.startswith(b'\x89PNG')
    assert store.map_png() is png1
    doc = store.map_json()
    assert doc['encoding'] == 'rle' and doc['width'] == 4
    assert doc['data'][0] == [100, 1] and doc['data'][-1] == [-1, 1]
    assert store.map_json() is doc
    # 새 지도가 오면 캐시가 비워진다
    store.update_map(fakes.fake_grid(width=2, height=2, data=[0, 0, 0, 0]))
    assert store.map_png() is not png1
    assert store.map_json()['data'] == [[0, 4]]


def test_store_ignores_degenerate_map_for_world():
    store = ss.StateStore()
    store.update_map(fakes.fake_grid(width=0, height=0, data=[]))
    assert store.world == ss.DEFAULT_WORLD


def test_store_empty_alert_array_not_broadcast():
    store = ss.StateStore()
    q = store.subscribe()
    msg = fakes.fake_diag_array()
    msg.status = []
    assert store.add_alerts(msg) == []
    assert q.empty()


def test_store_history_limits():
    store = ss.StateStore(max_alerts=2, max_task_events=3, max_tasks=2)
    for i in range(5):
        store.add_alerts(fakes.fake_diag_array(name=f'a{i}'))
        store.add_task_event(fakes.fake_task(task_id=f'T-{i}'))
    snap = store.snapshot()
    assert [a['name'] for a in snap['alerts']] == ['a3', 'a4']
    assert [t['task_id'] for t in snap['task_events']] == ['T-2', 'T-3', 'T-4']
    assert list(snap['tasks']) == ['T-3', 'T-4']


def test_store_estop_and_task_request_events():
    store = ss.StateStore(robot_ids=['amr_01', 'amr_02'])
    q = store.subscribe()
    data = store.set_estop('amr_01', True)
    assert data['estops'] == {'all': False, 'amr_01': True} and 'detail' not in data
    store.set_estop('all', True)
    assert store.snapshot()['estops'] == {'all': True, 'amr_01': True}
    assert store.estop_active('all') and not store.estop_active('amr_02')
    store.set_estop('all', False)
    assert store.snapshot()['estops'] == {'all': False, 'amr_01': False}
    task = {'task_id': 'T-x'}
    assert store.record_task_request(task) is task
    assert store.snapshot()['last_task_request'] == task
    names = [q.get_nowait()[0] for _ in range(4)]
    assert names == ['estop', 'estop', 'estop', 'task_request']


def test_store_estop_release_keeps_unconfirmed_robots_latched():
    store = ss.StateStore(robot_ids=['amr_01', 'amr_02', 'amr_03'])
    store.set_estop('amr_01', True)
    store.set_estop('all', True)
    detail = {'reset': {'amr_01': 'ok', 'amr_02': 'no_server', 'amr_03': 'rejected'}}
    data = store.set_estop('all', False, still_latched=['amr_02', 'amr_03'], detail=detail)
    # 전체 해제: /fleet/estop 은 풀리지만 리셋이 확인되지 않은 로봇은 계속 정지로 보인다
    assert data['estops'] == {'all': False, 'amr_01': False, 'amr_02': True, 'amr_03': True}
    assert data['detail'] == detail
    data = store.set_estop('amr_02', False, still_latched=['amr_02'])
    assert data['estops']['amr_02'] is True
    data = store.set_estop('amr_02', False)
    assert data['estops']['amr_02'] is False


def test_store_rejects_robot_ids_that_are_not_topic_names():
    store = ss.StateStore(robot_ids=['amr_01', 'bad-id', '1amr'])
    robots = [fakes.fake_robot_state(rid) for rid in ('rv-bad', 'rv_10', '', 'a/b', 'amr_01')]
    store.update_status(fakes.fake_fleet_status(robots=robots))
    snap = store.snapshot()
    assert store.known_robot_ids() == ['amr_01', 'rv_10'] == snap['robot_ids']
    assert snap['invalid_robot_ids'] == ['a/b', 'rv-bad']
    assert ss.valid_robot_id('AMR_1') and not ss.valid_robot_id(5) and not ss.valid_robot_id('')


def test_snapshot_carries_seq_and_limits():
    store = ss.StateStore(max_alerts=7, max_task_events=8, max_tasks=9)
    assert store.snapshot()['seq'] == 0
    store.add_alerts(fakes.fake_diag_array())
    snap = store.snapshot()
    assert snap['seq'] == 1
    assert snap['limits'] == {'max_alerts': 7, 'max_task_events': 8, 'max_tasks': 9}


def test_subscribe_with_snapshot_has_no_duplicates_or_gaps():
    """리뷰 nit: 구독 → 스냅샷 사이에 들어온 알림이 두 번 보이면 안 된다 (동시 생산자로 경합 유도)."""
    store = ss.StateStore(max_alerts=100_000, queue_size=100_000)
    stop = threading.Event()
    produced = []

    def producer():
        i = 0
        while not stop.is_set() and i < 3000:
            store.add_alerts(fakes.fake_diag_array(name=f'a{i}'))
            produced.append(i)
            i += 1

    t = threading.Thread(target=producer)
    t.start()
    checks = 0
    while t.is_alive() and checks < 50:
        q, snap = store.subscribe_with_snapshot()
        seen = [a['name'] for a in snap['alerts']]
        last = snap['seq']
        time.sleep(0.002)
        while True:
            try:
                _, data, seq = q.get_nowait()
            except queue.Empty:
                break
            assert seq == last + 1              # 빠짐 없이 이어진다
            last = seq
            seen += [a['name'] for a in data]
        store.unsubscribe(q)
        assert len(seen) == len(set(seen))     # 스냅샷과 큐에 같은 알림이 없다
        assert seen == [f'a{i}' for i in range(len(seen))]
        checks += 1
    stop.set()
    t.join()
    assert checks > 0


def test_broadcast_drops_oldest_for_slow_subscriber():
    store = ss.StateStore(queue_size=2)
    q = store.subscribe()
    for i in range(5):
        assert store.broadcast('e', i) == 1
    got = [q.get_nowait()[1] for _ in range(2)]
    assert got == [3, 4]
    assert store.snapshot()['events_dropped'] == 3
    store.unsubscribe(q)
    store.unsubscribe(q)  # 두 번 제거해도 조용히 무시
    assert store.subscriber_count() == 0
    assert store.broadcast('e', 'nobody') == 0


def test_broadcast_sequence_ids_increase():
    store = ss.StateStore()
    q = store.subscribe()
    store.broadcast('a', 1)
    store.broadcast('b', 2)
    _, _, s1 = q.get_nowait()
    _, _, s2 = q.get_nowait()
    assert s2 == s1 + 1


def test_store_thread_safety_under_concurrent_updates():
    store = ss.StateStore(queue_size=10_000)
    q = store.subscribe()
    n = 200

    def writer():
        for _ in range(n):
            store.update_status(fakes.fake_fleet_status())

    threads = [threading.Thread(target=writer) for _ in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    count = 0
    while True:
        try:
            q.get_nowait()
            count += 1
        except queue.Empty:
            break
    assert count == 4 * n
    assert math.isfinite(store.snapshot()['status']['kpi']['throughput'])
