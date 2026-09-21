"""
측정 로그 입출력 (rclpy 비의존 순수 모듈).

CSV 열은 명세 4.10 표의 로그 포맷을 앞에 그대로 두고, 추가 열은 그 뒤에 붙인다.
런 디렉토리: <output_dir>/<run_name>/ (기본 $ROS_WS/logs/eval/<run_name>/).
"""

import csv
from datetime import datetime
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

# 명세 표의 열 (앞부분) + 추가 열 (뒷부분)
POSE_COLUMNS = ['timestamp', 'gt_x', 'gt_y', 'est_x', 'est_y', 'error', 'yaw_error', 'segment']
CTE_COLUMNS = ['timestamp', 'planned_x', 'planned_y', 'actual_x', 'actual_y', 'cte', 'segment']
RESPONSE_COLUMNS = ['cmd_time', 'response_time', 'latency_ms', 'cmd_id']
CPU_BASE_COLUMNS = ['timestamp', 'cpu_total_percent']   # 뒤에 cpu0, cpu1, ... 이 붙는다

POSE_FILE = 'pose_error.csv'
CTE_FILE = 'cte.csv'
RESPONSE_FILE = 'response_time.csv'
CPU_FILE = 'cpu.csv'
REPORT_FILE = 'report.md'


def default_log_root() -> Path:
    """기본 로그 루트: $ROS_WS/logs/eval, ROS_WS 미설정이면 ./logs/eval."""
    ws = os.environ.get('ROS_WS', '').strip()
    base = Path(ws) if ws else Path.cwd()
    return base / 'logs' / 'eval'


def default_run_name(now: Optional[datetime] = None) -> str:
    """run_YYYYmmdd_HHMMSS."""
    return (now or datetime.now()).strftime('run_%Y%m%d_%H%M%S')


def resolve_run_dir(output_dir: str = '', run_name: str = '', create: bool = True) -> Path:
    """
    런 디렉토리 경로를 정하고(필요하면 만들어) 돌려준다.

    output_dir 이 비면 default_log_root(), run_name 이 비면 default_run_name().
    """
    root = Path(output_dir).expanduser() if output_dir else default_log_root()
    run = run_name or default_run_name()
    path = root / run
    if create:
        path.mkdir(parents=True, exist_ok=True)
    return path


class CsvWriter:
    """
    헤더 한 줄 + 행 단위 flush 하는 CSV 기록기.

    노드가 강제 종료돼도 그때까지의 행은 남는다. with 문으로 쓸 수 있다.
    """

    def __init__(self, path: Path, columns: Sequence[str], float_fmt: str = '.6f'):
        self.path = Path(path)
        self.columns = list(columns)
        self.float_fmt = float_fmt
        self.rows = 0
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.path, 'w', newline='', encoding='utf-8')
        self._writer = csv.writer(self._fh)
        self._writer.writerow(self.columns)
        self._fh.flush()

    def __enter__(self) -> 'CsvWriter':
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _fmt(self, v: Any) -> Any:
        if isinstance(v, bool):
            return int(v)
        if isinstance(v, float):
            return format(v, self.float_fmt)
        return v

    def write(self, values: Sequence[Any]) -> None:
        if len(values) != len(self.columns):
            raise ValueError(f'expected {len(self.columns)} values, got {len(values)}')
        self._writer.writerow([self._fmt(v) for v in values])
        self._fh.flush()
        self.rows += 1

    def close(self) -> None:
        if not self._fh.closed:
            self._fh.close()


def _parse(v: str) -> Any:
    try:
        return float(v)
    except ValueError:
        return v


def read_csv(path: Path) -> List[Dict[str, Any]]:
    """CSV → 행 dict 목록. 숫자로 읽히는 값은 float, 나머지는 문자열."""
    path = Path(path)
    with open(path, newline='', encoding='utf-8') as fh:
        reader = csv.DictReader(fh)
        return [{k: _parse(v) for k, v in row.items()} for row in reader]


def column(rows: Sequence[Dict[str, Any]], name: str) -> List[Any]:
    """행 목록에서 열 하나를 뽑는다."""
    return [r[name] for r in rows]


def write_text(path: Path, text: str) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8')
    return path
