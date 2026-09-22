"""
Flask 앱 팩토리 (rclpy 미의존). ROS 연동은 콜백 두 개로 주입한다.

    GET  /                  단일 페이지 앱 (web/index.html)
    GET  /static/<file>     web/ 정적 파일 (app.js, style.css)
    GET  /events            SSE 스트림: snapshot(id = seq) → status / alerts / task_event /
                            map_updated / estop / task_request, 이벤트가 없으면 heartbeat_period 마다
                            heartbeat
    GET  /api/state         전체 상태 JSON 스냅샷 (seq, limits 포함)
    GET  /api/map           지도: ?format=png (기본, 8-bit 그레이) | json (런렝스)
    GET  /api/health        {ok, sse_clients, uptime_sec, has_status, has_map, auth_required}
    POST /api/tasks         JSON Task Description → 검증 → on_task_request(json_str, task)
                            202 발행됨 | 400 검증 실패 | 503 /fleet/task_request 구독자 없음(보내지 않음)
    POST /api/estop         {robot_id: 'all'|id, active: bool} → on_estop(target, active)
                            200 발행(+해제면 reset_estop 성공) | 400 입력 오류 | 409 전체 E-stop 중
                            로봇별 해제 또는 뒤이은 조작에 밀림 | 502 값 발행·reset_estop 실패
                            (그 로봇은 계속 정지로 표시)

모든 요청: Host 헤더가 allowed_hosts 에 없으면 400 (DNS rebinding 방어). POST: Origin 이 있으면 Host 와
같아야 하고(403), api_token 이 설정됐으면 X-Dashboard-Token 헤더가 맞아야 한다(401). security 모듈 참고.

서버는 werkzeug 스레드 서버(threaded=True)로 띄운다. eventlet/flask-socketio 는 rclpy 를 막으므로 쓰지 않는다.
"""

import json
import os
import threading
import time

from flask import Flask, jsonify, request, Response, send_from_directory

from amr_dashboard import estop as es
from amr_dashboard import security
from amr_dashboard import sse
from amr_dashboard.task_schema import validate_estop, validate_task

SSE_HEADERS = {
    'Cache-Control': 'no-cache',
    'X-Accel-Buffering': 'no',   # nginx 등 프록시의 응답 버퍼링 금지
    'Connection': 'keep-alive',
}
CONTROL_PATHS = ('/api/tasks', '/api/estop')


def default_web_dir() -> str:
    """패키지 소스 트리의 web/ (share 디렉토리를 모를 때의 폴백)."""
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'web')


def create_app(store, web_dir=None, on_task_request=None, on_estop=None,
               heartbeat_period=5.0, action_log=None, allowed_hosts=None, api_token='',
               reset_timeout=2.0):
    """
    Flask 앱을 만든다.

    store: StateStore.
    on_task_request(json_str, task_dict) → 구독자 수(int, 0 이면 보내지 않음) 또는 None(알 수 없음).
    on_estop(target, active) → estop.EstopOutcome (None 이면 ROS 연동 없음: 값만 기록).
    allowed_hosts: 받아들일 Host 이름 목록 (None 이면 루프백만). api_token: 조작 API 토큰 ('' 이면 없음).
    reset_timeout: 해제 때 reset_estop 응답을 기다리는 시간 [s].
    action_log: ActionLog 또는 None.
    """
    web_dir = os.path.abspath(web_dir or default_web_dir())
    app = Flask('amr_dashboard', static_folder=web_dir, static_url_path='/static')
    app.json.ensure_ascii = False
    app.json.sort_keys = False
    app.config['STORE'] = store
    app.config['HEARTBEAT_PERIOD'] = float(heartbeat_period)
    hosts = list(security.LOOPBACK_NAMES if allowed_hosts is None else allowed_hosts)
    token = api_token or ''
    # E-stop 조작 하나(발행 + 표시 갱신 + 이벤트)를 통째로 직렬화: 발행 순서 = 표시·SSE 순서
    estop_lock = threading.Lock()
    estop_gen = {'n': 0}

    def log_action(action, **fields):
        if action_log is not None:
            action_log.append(action, remote=request.remote_addr, **fields)

    def fail(status, *errors, **extra):
        body = {'ok': False, 'errors': list(errors)}
        body.update(extra)
        return jsonify(body), status

    @app.before_request
    def check_origin():
        host = request.headers.get('Host', '')
        if not security.host_allowed(host, hosts):
            return fail(400, f'허용하지 않은 Host: {host!r} (allowed_hosts 파라미터)')
        if request.method == 'POST':
            if not security.origin_allowed(request.headers.get('Origin'), host):
                return fail(403, '다른 출처(Origin)의 조작 요청은 받지 않는다')
            if request.path in CONTROL_PATHS and not security.token_ok(
                    request.headers.get(security.TOKEN_HEADER), token):
                return fail(401, f'조작 토큰이 필요하다 ({security.TOKEN_HEADER} 헤더)', auth='token')
        return None

    @app.get('/')
    def index():
        return send_from_directory(web_dir, 'index.html')

    @app.get('/events')
    def events():
        stream = sse.event_stream(store, heartbeat_period=app.config['HEARTBEAT_PERIOD'])
        return Response(stream, mimetype='text/event-stream', headers=SSE_HEADERS)

    @app.get('/api/state')
    def api_state():
        return jsonify(store.snapshot())

    @app.get('/api/health')
    def api_health():
        snap = store.snapshot()
        return jsonify({
            'ok': True,
            'sse_clients': snap['sse_clients'],
            'uptime_sec': snap['uptime_sec'],
            'has_status': snap['status'] is not None,
            'has_map': snap['map'] is not None,
            'auth_required': bool(token),
        })

    @app.get('/api/map')
    def api_map():
        fmt = request.args.get('format', 'png').lower()
        if fmt == 'json':
            doc = store.map_json()
            if doc is None:
                return fail(404, '지도를 아직 받지 못했다')
            return jsonify(doc)
        if fmt != 'png':
            return fail(400, 'format 은 png 또는 json')
        png = store.map_png()
        if png is None:
            return fail(404, '지도를 아직 받지 못했다')
        meta = store.map_meta()
        headers = {'Cache-Control': 'no-cache', 'X-Map-Stamp': f'{meta["stamp"]:.6f}'}
        return Response(png, mimetype='image/png', headers=headers)

    @app.post('/api/tasks')
    def api_tasks():
        payload = request.get_json(silent=True)
        task, errors = validate_task(payload, now=time.time(), world=store.world)
        if errors:
            return fail(400, *errors)
        json_str = json.dumps(task, ensure_ascii=False)
        delivered = None
        if on_task_request is not None:
            delivered = on_task_request(json_str, task)
            if delivered == 0:
                log_action('task_request_rejected', task_id=task['task_id'],
                           reason='no_subscriber')
                return fail(503, '/fleet/task_request 구독자가 없다 (fleet_manager 미연결) — '
                                 '작업을 보내지 않았다. fleet_manager 기동 후 다시 투입한다')
        store.record_task_request(task)
        log_action('task_request', task_id=task['task_id'], task=task)
        return jsonify({'ok': True, 'task': task, 'delivered_to': delivered}), 202

    @app.post('/api/estop')
    def api_estop():
        payload = request.get_json(silent=True)
        target, active, errors = validate_estop(payload, store.known_robot_ids())
        if errors:
            return fail(400, *errors)
        with estop_lock:
            if not active and target != 'all' and store.estop_active('all'):
                return fail(409, '전체 E-stop 이 활성이다 — 로봇별 해제 전에 전체를 해제한다')
            estop_gen['n'] += 1
            gen = estop_gen['n']
            outcome = (on_estop(target, active) if on_estop is not None else None) or \
                es.simple_outcome(target, active)
            waiting = bool(outcome.pending())
            if not waiting:
                data = store.set_estop(target, active, outcome.still_latched(), outcome.detail())
        if waiting:
            # 리셋 응답은 잠금 밖에서 기다린다 — 그동안 들어온 E-stop 활성화를 막지 않는다
            outcome.wait(reset_timeout)
            with estop_lock:
                if estop_gen['n'] != gen:
                    log_action('estop', target=target, active=active, superseded=True,
                               detail=outcome.detail())
                    return fail(409, '해제 결과를 기다리는 동안 다른 E-stop 조작이 들어와 이 결과는 '
                                     '표시에 반영하지 않았다', detail=outcome.detail(),
                                estops=store.snapshot()['estops'])
                data = store.set_estop(target, active, outcome.still_latched(), outcome.detail())
        log_action('estop', target=target, active=active, ok=outcome.ok(), detail=outcome.detail())
        body = {'ok': outcome.ok(), 'target': target, 'active': active, 'estops': data['estops'],
                'detail': outcome.detail()}
        if not outcome.ok():
            body['errors'] = outcome.errors()
            return jsonify(body), 502
        return jsonify(body)

    return app
