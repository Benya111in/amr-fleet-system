"""
측정 로그 입출력 (rclpy 비의존 순수 모듈).

CSV 열은 명세 4.10 표의 로그 포맷을 앞에 그대로 두고, 추가 열은 그 뒤에 붙인다.
런 디렉토리: <output_dir>/<run_name>/ (기본 $ROS_WS/logs/eval/<run_name>/).
파일 이름: <지표>.csv, 로봇 네임스페이스 아래 로거는 <지표>_<ns>.csv (예: pose_error_amr_01.csv) —
여러 로봇을 같은 run_name 으로 재도 서로 덮어쓰지 않고, analyze 가 한 런으로 합쳐 판정한다.
"""

import csv
from datetime import datetime
import os
from pathlib import Path
import re
from typing import Any, Dict, List, Optional, Sequence

# 명세 표의 열 (앞부분) + 추가 열 (뒷부분)
POSE_COLUMNS = ['timestamp', 'gt_x', 'gt_y', 'est_x', 'est_y', 'error', 'yaw_error', 'segment']
CTE_COLUMNS = ['timestamp', 'planned_x', 'planned_y', 'actual_x', 'actual_y', 'cte', 'segment']
RESPONSE_COLUMNS = ['cmd_time', 'response_time', 'latency_ms', 'cmd_id', 'cmd_source', 'rtf',
                    'latency_wall_ms']
CPU_BASE_COLUMNS = ['timestamp', 'cpu_total_percent']   # 뒤에 귀속 열·rtf·load1·cpu0, cpu1, ... 이 붙는다

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


def namespaced_name(filename: str, namespace: str) -> str:
    """'pose_error.csv' + '/amr_01' → 'pose_error_amr_01.csv' (네임스페이스가 '/' 나 '' 면 그대로)."""
    ns = re.sub(r'[^A-Za-z0-9_]+', '_', namespace.strip('/'))
    if not ns:
        return filename
    stem, dot, ext = filename.rpartition('.')
    return f'{stem}_{ns}.{ext}' if dot else f'{filename}_{ns}'


def unique_path(path: Path) -> Path:
    """이미 있는 path 면 <stem>_1<ext>, <stem>_2<ext> … 중 없는 첫 경로, 없으면 path 그대로."""
    path = Path(path)
    if not path.exists():
        return path
    for i in range(1, 10000):
        cand = path.with_name(f'{path.stem}_{i}{path.suffix}')
        if not cand.exists():
            return cand
    raise FileExistsError(f'{path}: 빈 이름을 찾지 못했다')


def metric_files(run_dir: Path, filename: str) -> List[Path]:
    """런 디렉토리에서 한 지표의 CSV 들: <stem>.csv 와 <stem>_<접미>.csv (이름순)."""
    run_dir = Path(run_dir)
    stem = Path(filename).stem
    found = [p for p in run_dir.glob(f'{stem}*.csv')
             if p.is_file() and (p.stem == stem or p.stem.startswith(stem + '_'))]
    return sorted(found)


class CsvWriter:
    """
    헤더 한 줄 + 행 단위 flush 하는 CSV 기록기.

    노드가 강제 종료돼도 그때까지의 행은 남는다. with 문으로 쓸 수 있다. 기존 파일은 덮어쓰지 않는다
    (배타적 생성 — 이미 있으면 FileExistsError, 이름은 unique_path 로 고른다).
    """

    def __init__(self, path: Path, columns: Sequence[str], float_fmt: str = '.6f'):
        self.path = Path(path)
        self.columns = list(columns)
        self.float_fmt = float_fmt
        self.rows = 0
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.path, 'x', newline='', encoding='utf-8')
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


def _parse(v: Any) -> Any:
    try:
        return float(v)
    except (TypeError, ValueError):
        return v


def read_csv(path: Path, skipped: Optional[List[int]] = None) -> List[Dict[str, Any]]:
    """
    CSV → 행 dict 목록. 숫자로 읽히는 값은 float, 나머지는 문자열.

    열 수가 헤더와 다른 행(기록 도중 잘린 마지막 줄 등)은 버리고, skipped 가 주어지면 그 줄 번호를
    넣는다 (csv.DictReader 는 모자란 칸을 None, 남는 칸을 키 None 에 담는다).
    """
    path = Path(path)
    rows: List[Dict[str, Any]] = []
    with open(path, newline='', encoding='utf-8', errors='replace') as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            if None in row or any(v is None for v in row.values()):
                if skipped is not None:
                    skipped.append(reader.line_num)
                continue
            rows.append({k: _parse(v) for k, v in row.items()})
    return rows


def read_header(path: Path) -> List[str]:
    """CSV 헤더 (빈 파일이면 [])."""
    with open(Path(path), newline='', encoding='utf-8', errors='replace') as fh:
        return next(csv.reader(fh), [])


def column(rows: Sequence[Dict[str, Any]], name: str) -> List[Any]:
    """행 목록에서 열 하나를 뽑는다."""
    return [r[name] for r in rows]


def write_text(path: Path, text: str) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8')
    return path
