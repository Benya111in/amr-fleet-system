"""
Flask 앱 팩토리 (rclpy 미의존). ROS 연동은 콜백 두 개로 주입한다.

    GET  /                  단일 페이지 앱 (web/index.html)
    GET  /static/<file>     web/ 정적 파일 (app.js, style.css)
    GET  /events            SSE 스트림: snapshot → status / alerts / task_event / map_updated /
                            estop / task_request, 이벤트가 없으면 heartbeat_period 마다 heartbeat
    GET  /api/state         전체 상태 JSON 스냅샷
    GET  /api/map           지도: ?format=png (기본, 8-bit 그레이) | json (런렝스)
    GET  /api/health        {ok, sse_clients, uptime_sec}
    POST /api/tasks         JSON Task Description → 검증 → on_task_request(json_str, task) → 202
    POST /api/estop         {robot_id: 'all'|id, active: bool} → on_estop(target, active) → 200

서버는 werkzeug 스레드 서버(threaded=True)로 띄운다. eventlet/flask-socketio 는 rclpy 를 막으므로 쓰지 않는다.
"""

import json
import os
import time

from flask import Flask, Response, jsonify, request, send_from_directory

from amr_dashboard import sse
from amr_dashboard.task_schema import validate_estop, validate_task

SSE_HEADERS = {
    'Cache-Control': 'no-cache',
    'X-Accel-Buffering': 'no',   # nginx 등 프록시의 응답 버퍼링 금지
    'Connection': 'keep-alive',
}


def default_web_dir() -> str:
    """패키지 소스 트리의 web/ (share 디렉토리를 모를 때의 폴백)."""
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'web')


def create_app(store, web_dir=None, on_task_request=None, on_estop=None,
               heartbeat_period=5.0, action_log=None):
    """
    Flask 앱을 만든다.

    store: StateStore. on_task_request(json_str, task_dict) / on_estop(target, active) 는
    노드가 ROS 발행으로 연결한다 (None 이면 기록만 한다). action_log: ActionLog 또는 None.
    """
    web_dir = os.path.abspath(web_dir or default_web_dir())
    app = Flask('amr_dashboard', static_folder=web_dir, static_url_path='/static')
    app.json.ensure_ascii = False
    app.json.sort_keys = False
    app.config['STORE'] = store
    app.config['HEARTBEAT_PERIOD'] = float(heartbeat_period)

    def log_action(action, **fields):
        if action_log is not None:
            action_log.append(action, remote=request.remote_addr, **fields)

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
        })

    @app.get('/api/map')
    def api_map():
        fmt = request.args.get('format', 'png').lower()
        if fmt == 'json':
            doc = store.map_json()
            if doc is None:
                return jsonify({'ok': False, 'errors': ['지도를 아직 받지 못했다']}), 404
            return jsonify(doc)
        if fmt != 'png':
            return jsonify({'ok': False, 'errors': ['format 은 png 또는 json']}), 400
        png = store.map_png()
        if png is None:
            return jsonify({'ok': False, 'errors': ['지도를 아직 받지 못했다']}), 404
        meta = store.map_meta()
        headers = {'Cache-Control': 'no-cache', 'X-Map-Stamp': f'{meta["stamp"]:.6f}'}
        return Response(png, mimetype='image/png', headers=headers)

    @app.post('/api/tasks')
    def api_tasks():
        payload = request.get_json(silent=True)
        task, errors = validate_task(payload, now=time.time(), world=store.world)
        if errors:
            return jsonify({'ok': False, 'errors': errors}), 400
        json_str = json.dumps(task, ensure_ascii=False)
        if on_task_request is not None:
            on_task_request(json_str, task)
        store.record_task_request(task)
        log_action('task_request', task_id=task['task_id'], task=task)
        return jsonify({'ok': True, 'task': task}), 202

    @app.post('/api/estop')
    def api_estop():
        payload = request.get_json(silent=True)
        target, active, errors = validate_estop(payload, store.known_robot_ids())
        if errors:
            return jsonify({'ok': False, 'errors': errors}), 400
        if on_estop is not None:
            on_estop(target, active)
        data = store.set_estop(target, active)
        log_action('estop', target=target, active=active)
        return jsonify({'ok': True, 'target': target, 'active': active, 'estops': data['estops']})

    return app
