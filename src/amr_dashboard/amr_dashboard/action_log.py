"""
운영자 조작 로그 (순수 파이썬): 작업 투입·E-stop 을 JSONL 로 남긴다.

파일: <log_dir>/dashboard_actions_YYYYmmdd.jsonl, 한 줄 = 조작 하나
    {"ts": 1790000000.123, "action": "task_request", "task_id": "...", "remote": "127.0.0.1", ...}
    {"ts": ..., "action": "estop", "target": "amr_01" | "all", "active": true, ...}
log_dir 기본값은 $ROS_WS/logs (워크스페이스 logs/ 디렉토리). 기록 실패는 대시보드를 멈추지 않는다.
"""

import datetime
import json
import os
import threading
import time


def default_log_dir() -> str:
    """$ROS_WS/logs (ROS_WS 미설정 시 현재 디렉토리 아래 logs)."""
    return os.path.join(os.environ.get('ROS_WS', os.getcwd()), 'logs')


class ActionLog:
    """일자별 JSONL 조작 로그. 스레드 안전."""

    def __init__(self, log_dir=None, prefix='dashboard_actions', on_error=None):
        self.log_dir = log_dir or default_log_dir()
        self.prefix = prefix
        self._on_error = on_error
        self._lock = threading.Lock()
        self.records_written = 0
        self.last_error = None

    def path_for(self, ts: float) -> str:
        """해당 시각의 로그 파일 경로."""
        day = datetime.datetime.fromtimestamp(ts).strftime('%Y%m%d')
        return os.path.join(self.log_dir, f'{self.prefix}_{day}.jsonl')

    def append(self, action: str, **fields) -> bool:
        """레코드 한 줄을 추가한다. 성공 여부를 돌려준다 (실패는 on_error 로 알린다)."""
        record = {'ts': time.time(), 'action': action}
        record.update(fields)
        line = json.dumps(record, ensure_ascii=False) + '\n'
        with self._lock:
            try:
                os.makedirs(self.log_dir, exist_ok=True)
                with open(self.path_for(record['ts']), 'a', encoding='utf-8') as f:
                    f.write(line)
                self.records_written += 1
                return True
            except OSError as exc:
                self.last_error = str(exc)
                if self._on_error is not None:
                    self._on_error(f'조작 로그 기록 실패 ({self.log_dir}): {exc}')
                return False
