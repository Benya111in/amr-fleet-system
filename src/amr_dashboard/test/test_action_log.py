"""action_log: JSONL 조작 로그."""

import json
import os

from amr_dashboard.action_log import ActionLog, default_log_dir


def test_default_log_dir_uses_ros_ws(monkeypatch):
    monkeypatch.setenv('ROS_WS', '/opt/ws')
    assert default_log_dir() == '/opt/ws/logs'
    monkeypatch.delenv('ROS_WS')
    assert default_log_dir() == os.path.join(os.getcwd(), 'logs')


def test_append_writes_jsonl(tmp_path):
    log = ActionLog(str(tmp_path / 'logs'))
    assert log.append('estop', target='amr_01', active=True)
    assert log.append('task_request', task_id='T-1', task={'한글': 'ok'})
    files = os.listdir(tmp_path / 'logs')
    assert len(files) == 1
    assert files[0].startswith('dashboard_actions_') and files[0].endswith('.jsonl')
    lines = (tmp_path / 'logs' / files[0]).read_text(encoding='utf-8').splitlines()
    assert len(lines) == 2
    first = json.loads(lines[0])
    assert first['action'] == 'estop' and first['target'] == 'amr_01' and first['active'] is True
    assert 'ts' in first
    assert json.loads(lines[1])['task'] == {'한글': 'ok'}
    assert log.records_written == 2 and log.last_error is None


def test_path_for_uses_local_date():
    log = ActionLog('/x', prefix='p')
    assert log.path_for(0).startswith('/x/p_19')
    assert log.path_for(0).endswith('.jsonl')


def test_append_failure_reports_and_continues(tmp_path):
    blocker = tmp_path / 'file'
    blocker.write_text('x')
    errors = []
    log = ActionLog(str(blocker / 'logs'), on_error=errors.append)  # 파일 아래 디렉토리 → OSError
    assert log.append('estop', target='all', active=False) is False
    assert log.records_written == 0
    assert log.last_error and errors and '조작 로그 기록 실패' in errors[0]
