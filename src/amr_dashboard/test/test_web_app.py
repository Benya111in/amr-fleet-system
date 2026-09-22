"""web_app: Flask test_client 로 REST/SSE 경로와 정적 페이지 무결성을 검사한다."""

import html.parser
import json
import os
import re
import threading
import time

import fakes
import pytest

from amr_dashboard import estop as es
from amr_dashboard import sse
from amr_dashboard.action_log import ActionLog
from amr_dashboard.state_store import StateStore
from amr_dashboard.web_app import create_app, default_web_dir

WEB_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'web')


def _record_estop(calls):
    def on_estop(target, active):
        calls['estops'].append((target, active))
        return es.simple_outcome(target, active)
    return on_estop


@pytest.fixture
def env(tmp_path):
    store = StateStore(robot_ids=['amr_01', 'amr_02'])
    calls = {'tasks': [], 'estops': []}
    log = ActionLog(str(tmp_path / 'logs'))

    def on_task(s, t):
        calls['tasks'].append((s, t))
        return 1                                    # 구독자 1 (fleet_manager)
    app = create_app(
        store, web_dir=WEB_DIR, on_task_request=on_task, on_estop=_record_estop(calls),
        heartbeat_period=0.2, action_log=log)
    app.config['TESTING'] = True
    return app.test_client(), store, calls, log


def make_client(store=None, **kwargs):
    store = store or StateStore(robot_ids=['amr_01', 'amr_02'])
    app = create_app(store, web_dir=WEB_DIR, **kwargs)
    app.config['TESTING'] = True
    return app.test_client(), store


def read_events(resp, count, timeout=3.0):
    """스트리밍 응답에서 SSE 이벤트 count 개를 timeout 안에 읽는다."""
    it = iter(resp.response)
    out = []

    def reader():
        for _ in range(count):
            chunk = next(it)
            out.extend(sse.parse_sse(chunk.decode('utf-8') if isinstance(chunk, bytes) else chunk))

    t = threading.Thread(target=reader, daemon=True)
    t.start()
    t.join(timeout)
    assert not t.is_alive(), f'SSE 이벤트 {count}개를 {timeout}s 안에 받지 못했다 (받은 것: {out})'
    return out


class _Collector(html.parser.HTMLParser):
    """index.html 에서 id 속성과 정적 파일 참조를 모은다."""

    def __init__(self):
        super().__init__()
        self.ids = set()
        self.assets = []
        self.title = ''
        self._in_title = False

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if 'id' in attrs:
            self.ids.add(attrs['id'])
        if tag == 'link' and attrs.get('rel') == 'stylesheet':
            self.assets.append(attrs['href'])
        if tag == 'script' and attrs.get('src'):
            self.assets.append(attrs['src'])
        if tag == 'title':
            self._in_title = True

    def handle_endtag(self, tag):
        if tag == 'title':
            self._in_title = False

    def handle_data(self, data):
        if self._in_title:
            self.title += data


def test_index_page_assets_and_ids(env):
    client, _, _, _ = env
    resp = client.get('/')
    assert resp.status_code == 200
    assert resp.mimetype == 'text/html'
    page = resp.get_data(as_text=True)
    parser = _Collector()
    parser.feed(page)
    assert 'AMR' in parser.title
    assert parser.assets == ['/static/style.css', '/static/app.js']
    for href in parser.assets:
        r = client.get(href)
        assert r.status_code == 200, href
        assert r.mimetype in ('text/css', 'text/javascript', 'application/javascript'), href
    # app.js 가 $('id') 로 찾는 요소가 모두 index.html 에 있어야 한다 (헤드리스 렌더 검사)
    app_js = client.get('/static/app.js').get_data(as_text=True)
    used = set(re.findall(r"\$\('([a-z0-9-]+)'\)", app_js))
    assert used, 'app.js 에서 $(id) 사용을 찾지 못했다'
    missing = used - parser.ids
    assert not missing, f'index.html 에 없는 id: {sorted(missing)}'
    assert "new EventSource('/events')" in app_js
    assert '긴급 정지' in page and '작업 투입' in page and 'KPI' in page
    assert client.get('/static/missing.js').status_code == 404


def test_default_web_dir_is_sibling_web_dir():
    # 소스 트리에서는 src/amr_dashboard/web, 설치 트리에서는 노드가 share 경로를 명시해 준다
    import amr_dashboard.web_app as module
    package_root = os.path.dirname(os.path.dirname(os.path.abspath(module.__file__)))
    assert default_web_dir() == os.path.join(package_root, 'web')


def test_api_state_and_health(env):
    client, store, _, _ = env
    snap = client.get('/api/state').get_json()
    assert snap['status'] is None and snap['robot_ids'] == ['amr_01', 'amr_02']
    store.update_status(fakes.fake_fleet_status())
    snap = client.get('/api/state').get_json()
    assert snap['status']['kpi']['tasks_pending'] == 3
    assert [r['robot_id'] for r in snap['status']['robots']] == ['amr_01', 'amr_02']
    health = client.get('/api/health').get_json()
    assert health['ok'] is True and health['has_status'] is True and health['has_map'] is False
    assert health['sse_clients'] == 0 and health['auth_required'] is False
    assert snap['seq'] >= 1 and snap['limits']['max_tasks'] == 200


def test_api_map(env):
    client, store, _, _ = env
    assert client.get('/api/map').status_code == 404
    assert client.get('/api/map?format=json').status_code == 404
    store.update_map(fakes.fake_grid())
    png = client.get('/api/map')
    assert png.status_code == 200 and png.mimetype == 'image/png'
    assert png.data.startswith(b'\x89PNG') and png.headers['X-Map-Stamp'] == '7.000000'
    doc = client.get('/api/map?format=json').get_json()
    assert doc['width'] == 4 and doc['encoding'] == 'rle' and doc['data'][0] == [100, 1]
    bad = client.get('/api/map?format=bmp')
    assert bad.status_code == 400


def test_post_task_valid(env, tmp_path):
    client, store, calls, log = env
    payload = {
        'priority': 10, 'deadline_sec': 30,
        'pickup_pose': {'x': 1.0, 'y': 2.0, 'yaw': 0.0},
        'dropoff_pose': {'x': 10.0, 'y': 20.0, 'yaw': 1.0},
        'item_type': 'small',
    }
    resp = client.post('/api/tasks', json=payload)
    assert resp.status_code == 202, resp.get_json()
    body = resp.get_json()
    assert body['ok'] is True and body['delivered_to'] == 1
    task = body['task']
    assert task['priority'] == 10 and task['deadline'] == 30.0 and task['item_type'] == 'small'
    assert task['pickup'] == {'x': 1.0, 'y': 2.0, 'yaw': 0.0, 'frame_id': 'map'}
    assert set(task) == {'task_id', 'priority', 'deadline', 'pickup', 'dropoff', 'item_type'}
    assert len(calls['tasks']) == 1
    json_str, task_arg = calls['tasks'][0]
    assert json.loads(json_str) == task_arg == task
    assert store.snapshot()['last_task_request'] == task
    assert log.records_written == 1
    files = os.listdir(tmp_path / 'logs')
    line = json.loads((tmp_path / 'logs' / files[0]).read_text(encoding='utf-8').splitlines()[0])
    assert line['action'] == 'task_request' and line['task_id'] == task['task_id']


def test_post_task_invalid(env):
    client, _, calls, _ = env
    resp = client.post('/api/tasks', json={'priority': 999, 'item_type': 'huge'})
    assert resp.status_code == 400
    body = resp.get_json()
    assert body['ok'] is False and len(body['errors']) >= 3
    assert calls['tasks'] == []
    # JSON 이 아닌 본문
    resp = client.post('/api/tasks', data='not json', content_type='text/plain')
    assert resp.status_code == 400 and resp.get_json()['errors'] == ['JSON 객체가 필요하다']
    # 지도 범위 밖 (기본 월드 60 x 40)
    resp = client.post('/api/tasks', json={
        'pickup': {'x': 99, 'y': 1}, 'dropoff': {'x': 1, 'y': 1}, 'item_type': 'small'})
    assert resp.status_code == 400 and any('지도 범위 밖' in e for e in resp.get_json()['errors'])
    # 플릿 매니저 스키마가 거부할 모르는 필드는 여기서 먼저 거부한다
    resp = client.post('/api/tasks', json={
        'pickup': {'x': 1, 'y': 1}, 'dropoff': {'x': 2, 'y': 2}, 'item_type': 'small',
        'item_mass': 3.0})
    assert resp.status_code == 400 and resp.get_json()['errors'] == ['모르는 필드: item_mass']
    assert calls['tasks'] == []


def test_post_estop(env):
    client, store, calls, _ = env
    resp = client.post('/api/estop', json={'robot_id': 'amr_01', 'active': True})
    assert resp.status_code == 200
    body = resp.get_json()
    assert body.pop('detail')['still_latched'] == []
    assert body == {'ok': True, 'target': 'amr_01', 'active': True,
                    'estops': {'all': False, 'amr_01': True}}
    resp = client.post('/api/estop', json={'robot_id': 'all', 'active': True})
    assert resp.status_code == 200 and resp.get_json()['estops']['all'] is True
    # 전체 E-stop 중 로봇별 해제는 409 (먼저 전체를 푼다) — 발행하지 않는다
    resp = client.post('/api/estop', json={'robot_id': 'amr_01', 'active': False})
    assert resp.status_code == 409 and '전체 E-stop' in resp.get_json()['errors'][0]
    resp = client.post('/api/estop', json={'active': False})
    assert resp.status_code == 200 and resp.get_json()['target'] == 'all'
    assert calls['estops'] == [('amr_01', True), ('all', True), ('all', False)]
    assert store.snapshot()['estops'] == {'all': False, 'amr_01': False}
    # 로봇이 FleetStatus 에 나타나면 대상으로 허용된다
    unknown = {'robot_id': 'amr_09', 'active': True}
    assert client.post('/api/estop', json=unknown).status_code == 400
    store.update_status(fakes.fake_fleet_status(robots=[fakes.fake_robot_state('amr_09')]))
    assert client.post('/api/estop', json=unknown).status_code == 200
    # 잘못된 입력
    bad_active = {'robot_id': 'amr_01', 'active': 'on'}
    assert client.post('/api/estop', json=bad_active).status_code == 400
    assert client.post('/api/estop', data='x', content_type='text/plain').status_code == 400
    assert len(calls['estops']) == 4


def test_events_stream_first_two_events(env):
    client, store, _, _ = env
    resp = client.get('/events')
    assert resp.status_code == 200
    assert resp.mimetype == 'text/event-stream'
    assert resp.headers['Cache-Control'] == 'no-cache'
    assert resp.headers['X-Accel-Buffering'] == 'no'
    events = read_events(resp, 1)
    assert events[0]['event'] == 'snapshot'
    snap = json.loads(events[0]['data'])
    assert snap['robot_ids'] == ['amr_01', 'amr_02'] and snap['sse_clients'] == 1
    store.update_status(fakes.fake_fleet_status())
    events = read_events(resp, 1)
    assert events[0]['event'] == 'status' and events[0]['id'] is not None
    assert json.loads(events[0]['data'])['kpi']['tasks_completed'] == 10
    # 이벤트가 없으면 heartbeat_period(0.2 s) 뒤 heartbeat
    events = read_events(resp, 1)
    assert events[0]['event'] == 'heartbeat'
    resp.close()
    assert store.subscriber_count() == 0


def test_events_multiple_clients_receive_broadcast(env):
    client, store, _, _ = env
    a = client.get('/events')
    b = client.get('/events')
    read_events(a, 1)
    read_events(b, 1)
    assert store.subscriber_count() == 2
    store.add_alerts(fakes.fake_diag_array())
    for resp in (a, b):
        ev = read_events(resp, 1)[0]
        assert ev['event'] == 'alerts'
        assert json.loads(ev['data'])[0]['level_name'] == 'ERROR'
    a.close()
    b.close()
    assert store.subscriber_count() == 0


def test_app_without_callbacks_or_log(tmp_path):
    store = StateStore()
    app = create_app(store, web_dir=WEB_DIR)
    client = app.test_client()
    resp = client.post('/api/tasks', json={
        'pickup_pose': {'x': 1, 'y': 1}, 'dropoff_pose': {'x': 2, 'y': 2}, 'item_type': 'medium'})
    assert resp.status_code == 202 and resp.get_json()['delivered_to'] is None
    assert client.post('/api/estop', json={'active': True}).status_code == 200
    # 콜백이 None 을 돌려줘도(옛 연동) 값만 기록한 것으로 본다
    client2, _ = make_client(on_estop=lambda target, active: None)
    assert client2.post('/api/estop', json={'active': True}).status_code == 200


TASK = {'pickup': {'x': 1, 'y': 1}, 'dropoff': {'x': 2, 'y': 2}, 'item_type': 'small'}


# ----- 보안: Host / Origin / 토큰 -----

def test_foreign_host_header_rejected_dns_rebinding():
    """리뷰 재현: Host: attacker.example 로 온 전체 해제 요청은 400, 발행하지 않는다."""
    calls = {'estops': []}
    client, store = make_client(on_estop=_record_estop(calls))
    store.set_estop('all', True)
    resp = client.post('/api/estop', json={'robot_id': 'all', 'active': False},
                       headers={'Host': 'attacker.example:18077'})
    assert resp.status_code == 400 and 'Host' in resp.get_json()['errors'][0]
    assert calls['estops'] == [] and store.estop_active('all')
    assert client.get('/api/state', headers={'Host': 'attacker.example'}).status_code == 400
    for host in ('127.0.0.1:8080', 'localhost:8080', '[::1]:8080'):
        assert client.get('/api/health', headers={'Host': host}).status_code == 200, host
    lan, _ = make_client(allowed_hosts=['localhost', 'dash.lan'])
    assert lan.get('/api/health', headers={'Host': 'dash.lan:8080'}).status_code == 200
    open_, _ = make_client(allowed_hosts=['*'])
    assert open_.get('/api/health', headers={'Host': 'attacker.example'}).status_code == 200


def test_cross_origin_post_rejected():
    calls = {'estops': []}
    client, _ = make_client(on_estop=_record_estop(calls))
    resp = client.post('/api/estop', json={'active': True},
                       headers={'Origin': 'http://attacker.example:8080'})
    assert resp.status_code == 403 and calls['estops'] == []
    same = client.post('/api/estop', json={'active': True},
                       headers={'Origin': 'http://localhost', 'Host': 'localhost'})
    assert same.status_code == 200 and calls['estops'] == [('all', True)]


def test_control_token_required_when_configured(monkeypatch):
    calls = {'estops': [], 'tasks': []}
    client, _ = make_client(on_estop=_record_estop(calls), api_token='s3cret',
                            on_task_request=lambda s, t: calls['tasks'].append(t) or 1)
    assert client.get('/api/health').get_json()['auth_required'] is True
    assert client.get('/api/state').status_code == 200          # 조회는 토큰 없이
    resp = client.post('/api/estop', json={'active': True})
    assert resp.status_code == 401 and resp.get_json()['auth'] == 'token'
    bad = client.post('/api/tasks', json=TASK, headers={'X-Dashboard-Token': 'nope'})
    assert bad.status_code == 401
    assert calls == {'estops': [], 'tasks': []}
    ok = client.post('/api/estop', json={'active': True}, headers={'X-Dashboard-Token': 's3cret'})
    assert ok.status_code == 200
    ok = client.post('/api/tasks', json=TASK, headers={'X-Dashboard-Token': 's3cret'})
    assert ok.status_code == 202 and len(calls['tasks']) == 1


# ----- 작업 투입: 구독자 없음 → 503 -----

def test_post_task_without_subscriber_is_503(tmp_path):
    log = ActionLog(str(tmp_path / 'logs'))
    client, store = make_client(on_task_request=lambda s, t: 0, action_log=log)
    resp = client.post('/api/tasks', json=TASK)
    assert resp.status_code == 503
    assert 'fleet_manager' in resp.get_json()['errors'][0]
    assert store.snapshot()['last_task_request'] is None       # 투입으로 기록하지 않는다
    line = json.loads(open(log.path_for(time.time()), encoding='utf-8').read().splitlines()[0])
    assert line['action'] == 'task_request_rejected' and line['reason'] == 'no_subscriber'


# ----- E-stop 해제: reset_estop 결과가 응답·표시 상태가 된다 -----

class PendingOutcome:
    """발행 뒤 리셋 응답이 나중에 오는 해제 (on_estop 대역)."""

    def __init__(self, results, delay=0.05, publish_failed=None):
        self.results = results
        self.delay = delay
        self.publish_failed = publish_failed or {}
        self.calls = []

    def __call__(self, target, active):
        self.calls.append((target, active))
        out = es.EstopOutcome(target, active)
        for rid, err in self.publish_failed.items():
            out.fail_publish(rid, err)
        if not active:
            for rid, (status, msg) in self.results.items():
                out.set_reset(rid, es.PENDING)
                if status != es.PENDING:
                    threading.Timer(self.delay, out.set_reset, args=(rid, status, msg)).start()
        return out


def test_release_success_waits_for_reset():
    fake = PendingOutcome({'amr_01': (es.OK, 'reset')})
    client, store = make_client(on_estop=fake)
    store.set_estop('amr_01', True)
    resp = client.post('/api/estop', json={'robot_id': 'amr_01', 'active': False})
    body = resp.get_json()
    assert resp.status_code == 200 and body['ok'] is True
    assert body['detail']['reset'] == {'amr_01': 'ok'}
    assert store.snapshot()['estops']['amr_01'] is False


def _no_server(target, active):
    out = es.EstopOutcome(target, active)
    out.set_reset(target, es.NO_SERVER, 'service not available')
    return out


@pytest.mark.parametrize('on_estop, expected', [
    (PendingOutcome({'amr_01': (es.REJECTED, 'cause present')}), es.REJECTED),
    (PendingOutcome({'amr_01': (es.PENDING, '')}), es.TIMEOUT),      # 응답 없음
    (_no_server, es.NO_SERVER),                                      # safety_node 없음
])
def test_release_failure_is_502_and_robot_stays_stopped(on_estop, expected):
    client, store = make_client(on_estop=on_estop, reset_timeout=0.2)
    q = store.subscribe()
    store.set_estop('amr_01', True)
    resp = client.post('/api/estop', json={'robot_id': 'amr_01', 'active': False})
    body = resp.get_json()
    assert resp.status_code == 502 and body['ok'] is False
    assert body['estops']['amr_01'] is True and store.estop_active('amr_01')
    assert body['detail']['still_latched'] == ['amr_01']
    assert f'amr_01: reset_estop {expected}' in body['errors'][0]
    events = []
    while not q.empty():
        events.append(q.get_nowait())
    last = [d for e, d, _ in events if e == 'estop'][-1]
    assert last['estops']['amr_01'] is True and last['detail']['reset']['amr_01'] == expected


def test_release_all_with_invalid_robot_reports_and_continues():
    fake = PendingOutcome({'amr_02': (es.OK, '')}, publish_failed={'rv-bad': 'InvalidTopicName'})
    client, store = make_client(on_estop=fake)
    store.set_estop('all', True)
    resp = client.post('/api/estop', json={'robot_id': 'all', 'active': False})
    body = resp.get_json()
    assert resp.status_code == 502 and body['estops']['all'] is False
    assert body['estops']['rv-bad'] is True and body['detail']['reset'] == {'amr_02': 'ok'}
    assert resp.content_type.startswith('application/json')


def test_activation_during_pending_release_wins():
    """리셋 응답을 기다리는 동안 들어온 정지가 늦게 끝난 해제 결과에 덮이지 않는다 (409)."""
    fake = PendingOutcome({'amr_01': (es.OK, '')}, delay=0.3)
    client, store = make_client(on_estop=fake, reset_timeout=2.0)
    store.set_estop('amr_01', True)
    out = {}

    def release():
        out['resp'] = client.post('/api/estop', json={'robot_id': 'amr_01', 'active': False})

    t = threading.Thread(target=release)
    t.start()
    time.sleep(0.1)
    stop = client.post('/api/estop', json={'robot_id': 'amr_01', 'active': True})
    t.join()
    assert stop.status_code == 200
    assert out['resp'].status_code == 409
    assert '다른 E-stop 조작' in out['resp'].get_json()['errors'][0]
    assert store.estop_active('amr_01')
    assert fake.calls == [('amr_01', False), ('amr_01', True)]


def test_concurrent_estop_publish_order_matches_display():
    """리뷰 재현 (verify_pure a): 발행 순서와 표시·SSE 순서가 같아야 한다 (잠금 하나로 직렬화)."""
    order = []

    def slow(target, active):
        order.append((target, active))
        if active:
            time.sleep(0.3)                  # 부하 흉내: 활성 발행 뒤 지연
        return es.simple_outcome(target, active)

    client, store = make_client(on_estop=slow)
    q = store.subscribe()
    ts = [threading.Thread(target=client.post, args=('/api/estop',),
                           kwargs={'json': {'robot_id': 'amr_01', 'active': a}})
          for a in (True, False)]
    ts[0].start()
    time.sleep(0.05)
    ts[1].start()
    for t in ts:
        t.join()
    shown = []
    while not q.empty():
        e, d, _ = q.get_nowait()
        if e == 'estop':
            shown.append(d['active'])
    assert order == [('amr_01', True), ('amr_01', False)]
    assert shown == [True, False]
    assert store.estop_active('amr_01') is False     # 마지막 발행(해제) = 표시
