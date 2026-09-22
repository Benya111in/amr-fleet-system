"""
amr_evaluation(feature/evaluation-tools) 연동: 로거 CSV 를 analyze 로 판정한다.

로거 노드는 stack.Stack.eval_logger() 가 시나리오 로그 디렉토리(logs/itest/<id>/)에 CSV 를 쓰게
띄운다. 여기서는 그 디렉토리를 amr_evaluation.analyze.analyze_run() 에 넘겨 명세 기준표를 얻고,
행을 result.json 에 옮긴다. amr_evaluation 이 없으면 available() 이 False 이고 시나리오는
amr_itest.metrics 의 하네스 자체 계산만으로 판정한다.
"""

import math
from pathlib import Path
from typing import Any, Dict, List, Optional

from amr_itest import requirements as req

LOGGERS = ('pose_error_logger', 'response_time_logger', 'cte_logger', 'cpu_sampler')


def available(executable: Optional[str] = None) -> bool:
    """amr_evaluation 파이썬 모듈 (+ 지정한 로거 실행 파일) 이 설치돼 있는가."""
    if not req.has_python_module('amr_evaluation'):
        return False
    return executable is None or req.has_executable('amr_evaluation', executable)


def _num(v: Any) -> Any:
    if isinstance(v, float) and not math.isfinite(v):
        return None
    return v


def analyze(run_dir: Path, pose_thresholds=(0.03, 0.05, 0.08), response_ms: float = 200.0,
            cpu_percent: float = 80.0) -> Dict[str, Any]:
    """
    analyze_run() 결과를 dict 로: {'rows': [...], 'warnings': [...], 'passed': bool, 'markdown'}.

    report.md 도 run_dir 에 남긴다 (그림은 생략 — 컨테이너 폰트/시간 절약).
    """
    from amr_evaluation import analyze as an
    from amr_evaluation import io as eio
    from amr_evaluation import segments

    th = an.Thresholds()
    th.pose = {segments.STOP: pose_thresholds[0], segments.STRAIGHT: pose_thresholds[1],
               segments.TURN: pose_thresholds[2]}
    th.response_mean_ms = response_ms
    th.cpu_mean_percent = cpu_percent
    report = an.analyze_run(Path(run_dir), th, plots=False)
    rows: List[Dict[str, Any]] = []
    for r in report.rows:
        rows.append({'metric': r.metric, 'segment': r.segment, 'count': r.count,
                     'stat': r.stat, 'value': _num(r.value), 'unit': r.unit,
                     'threshold': r.threshold, 'passed': r.passed,
                     'extras': {k: _num(v) for k, v in r.extras.items()}})
    md = report.to_markdown()
    eio.write_text(Path(run_dir) / eio.REPORT_FILE, md)
    return {'rows': rows, 'warnings': list(report.warnings), 'passed': report.passed(),
            'markdown': md}


def gated_rows(result: Dict[str, Any], metric_prefix: str) -> List[Dict[str, Any]]:
    """판정이 있는(passed 가 None 이 아닌) 행 중 metric 이 metric_prefix 로 시작하는 것."""
    return [r for r in result.get('rows', [])
            if r['passed'] is not None and str(r['metric']).startswith(metric_prefix)]
