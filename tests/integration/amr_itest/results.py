"""
시나리오 결과 기록 (rclpy 비의존 순수 모듈).

logs/itest/<시나리오 id>/
  result.json   메타데이터(지표·기준·로그 포맷) + 구성(실제/대역) + 측정값 + 판정 항목 + 호스트 부하
  *.csv         명세 4.10 로그 포맷의 원시 측정 (시나리오마다 다름)
  junit.xml     러너(amr_itest.launch_test_ros)가 쓴다 (scripts/run_integration.sh)
  launch.log    러너 전체 출력 (scripts/run_integration.sh)

최종 합격 여부(status)는 JUnit 이 기준이다. report.py 가 JUnit 상태를 result.json 에 덧붙여
logs/itest/summary.json 을 만든다.
"""

import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import time
from typing import Any, Dict, Iterable, List, Optional, Sequence

SCHEMA = 'amr_itest/result/v1'
RESULT_FILE = 'result.json'
KEEP_ON_CLEAN = ('launch.log',)


def host_load() -> Dict[str, Any]:
    """호스트 부하 (os.getloadavg, CPU 수). 외부 부하가 크면 시간 지표는 잠정치로 표시한다."""
    try:
        load = list(os.getloadavg())
    except OSError:
        load = [math.nan] * 3
    cpus = os.cpu_count() or 1
    return {'loadavg': [round(v, 2) for v in load], 'cpus': cpus,
            'provisional_under_load': bool(load[0] > 0.5 * cpus)}


def jsonable(v: Any) -> Any:
    """NaN/inf 는 JSON 표준이 아니므로 문자열로, 그 외 numpy 스칼라 등은 float/int 로."""
    if isinstance(v, bool) or v is None or isinstance(v, str):
        return v
    if isinstance(v, int):
        return v
    if isinstance(v, float):
        return v if math.isfinite(v) else str(v)
    if isinstance(v, dict):
        return {str(k): jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [jsonable(x) for x in v]
    if hasattr(v, 'item'):
        return jsonable(v.item())
    return str(v)


class ScenarioRecord:
    """시나리오 한 번의 결과 기록기. 쓰기는 모두 즉시 파일에 반영한다(중간 크래시 대비)."""

    def __init__(self, meta: Dict[str, Any], log_dir: Path,
                 settings: Optional[Dict[str, Any]] = None):
        self.log_dir = Path(log_dir)
        self.data: Dict[str, Any] = {
            'schema': SCHEMA,
            'scenario': dict(meta),
            'settings': dict(settings or {}),
            'backend': None,
            'components': {},
            'host': {'start': host_load()},
            'started_at': datetime.now(timezone.utc).isoformat(timespec='seconds'),
            'measurements': {},
            'checks': [],
            'notes': [],
            'state': 'running',
        }
        self._t0 = time.monotonic()

    # --- 준비 ---
    def begin(self, clean: bool = True) -> 'ScenarioRecord':
        """
        로그 디렉토리를 (비우고) 만들고 초기 result.json 을 쓴다.

        launch.log 는 남긴다 — run_integration.sh 가 이미 열어 쓰고 있는 파일이다.
        """
        if clean and self.log_dir.is_dir():
            for child in self.log_dir.iterdir():
                if child.name in KEEP_ON_CLEAN:
                    continue
                if child.is_dir():
                    shutil.rmtree(child, ignore_errors=True)
                else:
                    child.unlink()
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.write()
        return self

    @property
    def path(self) -> Path:
        return self.log_dir / RESULT_FILE

    # --- 내용 ---
    def set_backend(self, backend: str) -> None:
        self.data['backend'] = backend
        self.write()

    def set_components(self, components: Dict[str, str]) -> None:
        self.data['components'] = dict(components)
        self.write()

    def measure(self, key: str, value: Any) -> None:
        """측정값 기록 (같은 키는 덮어쓴다)."""
        self.data['measurements'][key] = value
        self.write()

    def check(self, name: str, value: Any, threshold: Any, passed: bool, unit: str = '') -> bool:
        """판정 항목 기록. passed 를 그대로 돌려줘 assert 에 바로 쓴다."""
        self.data['checks'].append({'name': name, 'value': value, 'threshold': threshold,
                                    'unit': unit, 'passed': bool(passed)})
        self.write()
        return bool(passed)

    def note(self, text: str) -> None:
        self.data['notes'].append(text)
        self.write()

    def skipped(self, reason: str) -> None:
        self.data['state'] = 'skipped'
        self.data['skip_reason'] = reason
        self.finish()

    def finish(self) -> None:
        """종료 시각·소요 시간·종료 시 부하를 기록한다 (post-shutdown 테스트에서 호출)."""
        if self.data['state'] == 'running':
            self.data['state'] = 'finished'
        self.data['host']['end'] = host_load()
        self.data['finished_at'] = datetime.now(timezone.utc).isoformat(timespec='seconds')
        self.data['duration_s'] = round(time.monotonic() - self._t0, 1)
        self.write()

    def elapsed(self) -> float:
        """기록 시작(시나리오 로드) 이후 wall 시간 [s]."""
        return time.monotonic() - self._t0

    def failed_checks(self) -> List[Dict[str, Any]]:
        return [c for c in self.data['checks'] if not c['passed']]

    # --- 파일 ---
    def write(self) -> Path:
        """result.json 을 원자적으로 쓴다 (tmp → rename)."""
        self.log_dir.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix('.json.tmp')
        tmp.write_text(json.dumps(jsonable(self.data), ensure_ascii=False, indent=2) + '\n',
                       encoding='utf-8')
        tmp.replace(self.path)
        return self.path

    def write_csv(self, name: str, columns: Sequence[str], rows: Iterable[Sequence[Any]]) -> Path:
        """명세 로그 포맷 CSV 한 개를 쓴다 (float 은 6 자리)."""
        path = self.log_dir / name
        with open(path, 'w', newline='', encoding='utf-8') as fh:
            w = csv.writer(fh)
            w.writerow(list(columns))
            for row in rows:
                w.writerow([f'{v:.6f}' if isinstance(v, float) else v for v in row])
        return path

    def write_text(self, name: str, text: str) -> Path:
        path = self.log_dir / name
        path.write_text(text, encoding='utf-8')
        return path


def load(path: Path) -> Dict[str, Any]:
    """result.json 을 읽는다 (없으면 빈 dict)."""
    path = Path(path)
    if not path.is_file():
        return {}
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except json.JSONDecodeError:
        return {}
