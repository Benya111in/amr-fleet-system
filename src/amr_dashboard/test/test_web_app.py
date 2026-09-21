"""web_app: Flask test_client 로 REST/SSE 경로와 정적 페이지 무결성을 검사한다."""

import html.parser
import json
import os
import re
import threading

import fakes
import pytest

from amr_dashboard import sse
from amr_dashboard.action_log import ActionLog
from amr_dashboard.state_store import StateStore
from amr_dashboard.web_app import create_app, default_web_dir

WEB_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'web')


@pytest.fixture
def env(tmp_path):
    store = StateStore(robot_ids=['amr_01', 'amr_02'])
    calls = {'tasks': [], 'estops': []}
    log = ActionLog(str(tmp_path / 'logs'))
    app = create_app(
        store, web_dir=WEB_DIR,
        on_task_request=lambda s, t: calls['tasks'].append((s, t)),
        on_estop=lambda target, active: calls['estops'].append((target, active)),
        heartbeat_period=0.2, action_log=log)
    app.config['TESTING'] = True
    return app.test_client(), store, calls, log


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
    assert health['sse_clients'] == 0


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
    assert body['ok'] is True
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
    assert resp.get_json() == {'ok': True, 'target': 'amr_01', 'active': True,
                               'estops': {'all': False, 'amr_01': True}}
    resp = client.post('/api/estop', json={'robot_id': 'all', 'active': True})
    assert resp.status_code == 200 and resp.get_json()['estops']['all'] is True
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
    assert resp.status_code == 202
    assert client.post('/api/estop', json={'active': True}).status_code == 200
