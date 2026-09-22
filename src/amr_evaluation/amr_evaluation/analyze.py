"""
측정 로그 분석 CLI (rclpy 비의존).

    ros2 run amr_evaluation analyze --input logs/eval/<run>/
    python3 -m amr_evaluation.analyze --input logs/eval/<run>/

런 디렉토리의 pose_error / cte / response_time / cpu CSV (로봇별 <지표>_<ns>.csv 도 합친다)를 읽어
명세 기준(정지 3 / 직선 5 / 회전 8 cm, CTE 직선 5 / 곡선 10 cm, 응답 200 ms, CPU 80 %)과 비교한
Markdown 표(report.md)와 PNG 그림을 같은 디렉토리에 쓰고, 표를 stdout 에 찍는다.

종료 코드 (CI/통합 테스트 게이트):
    0 통과
    1 기준 위반 (--no-fail 이면 0), --strict 에서 경고
    2 런 디렉토리 없음 / 인자 오류
    3 데이터 부족: 필수 지표(--require, 기본 전부)의 CSV 가 없거나 헤더뿐이거나, 판정 구간의 표본이
      최소 개수 미만 (--allow-missing 이면 경고로만 남기고 판정에서 뺀다)
    4 분석 중 예상하지 못한 오류
"""

import argparse
from dataclasses import dataclass, field
import math
from pathlib import Path
import sys
import traceback
from typing import Any, Dict, List, Optional, Sequence, Tuple

from amr_evaluation import clocks
from amr_evaluation import io
from amr_evaluation import metrics
from amr_evaluation import segments
import numpy as np

POSE_METRIC = '위치 추정 오차'
POSE_ALIGNED_METRIC = '위치 추정 오차 (SE2 정렬 후, 참고)'
CTE_METRIC = '경로 추종 CTE'
RESPONSE_METRIC = '응답 시간'
CPU_METRIC = 'CPU 사용률'
RTF_METRIC = '실시간 계수 RTF (참고)'
ALL = '전체'

GATE_METRICS = ('rmse', 'max', 'p95', 'mean')
METRIC_KEYS = ('pose', 'cte', 'response', 'cpu')
METRIC_FILES = {'pose': io.POSE_FILE, 'cte': io.CTE_FILE, 'response': io.RESPONSE_FILE,
                'cpu': io.CPU_FILE}
# 지표별 필수 숫자 열 (없으면 그 파일은 분석하지 않는다)
REQUIRED_COLUMNS = {
    'pose': ('timestamp', 'gt_x', 'gt_y', 'est_x', 'est_y', 'error'),
    'cte': ('timestamp', 'cte'),
    'response': ('cmd_time', 'response_time', 'latency_ms'),
    'cpu': ('timestamp',),
}
TIME_COLUMNS = {'pose': ('timestamp',), 'cte': ('timestamp',),
                'response': ('cmd_time', 'response_time'), 'cpu': ('timestamp',)}
RTF_WARN_BELOW = 0.95

EXIT_PASS = 0
EXIT_FAIL = 1
EXIT_NO_INPUT = 2
EXIT_INSUFFICIENT = 3
EXIT_ERROR = 4

# 구간별 고정 색 (색각 이상 대비 팔레트, 라벨 → 색 고정)
SEGMENT_COLORS = {
    segments.STOP: '#006BA4',
    segments.STRAIGHT: '#FF800E',
    segments.TURN: '#595959',
    segments.CURVE: '#595959',
}
# 그림 라벨은 영문 (컨테이너 기본 폰트에 한글 글리프가 없어 깨진다)
SEGMENT_EN = {
    segments.STOP: 'stop',
    segments.STRAIGHT: 'straight',
    segments.TURN: 'turn',
    segments.CURVE: 'curve',
}


@dataclass
class Thresholds:
    """명세 기준값. 거리는 m, 응답은 ms, CPU 는 %. *_min_samples 는 판정 구간별 최소 표본 수."""

    pose: Dict[str, float] = field(default_factory=lambda: {
        segments.STOP: 0.03, segments.STRAIGHT: 0.05, segments.TURN: 0.08})
    cte: Dict[str, float] = field(default_factory=lambda: {
        segments.STRAIGHT: 0.05, segments.CURVE: 0.10})
    response_mean_ms: float = 200.0
    cpu_mean_percent: float = 80.0
    pose_min_samples: int = 100       # 명세: 최소 100 시점 — 구간(정지/직선/회전)마다
    cte_min_samples: int = 100        # 구간(직선/곡선)마다
    response_min_samples: int = 50    # 명세: 50 회 이상
    cpu_min_samples: int = 60         # multi_robot.md §7: 60 s 이상 (1 Hz)


@dataclass
class MetricRow:
    """리포트 표의 한 행."""

    metric: str
    segment: str
    count: int
    stat: str
    value: float
    unit: str
    threshold: Optional[float] = None
    passed: Optional[bool] = None       # None: 판정 없음 (샘플 없음 또는 참고 행)
    extras: Dict[str, Any] = field(default_factory=dict)   # 값이 str 이면 그대로 출력
    gated: bool = False                 # 판정 대상 행 (표본 수 검사 대상)


@dataclass
class Report:
    """분석 결과."""

    run_dir: Path
    rows: List[MetricRow] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    insufficient: List[str] = field(default_factory=list)   # 데이터 부족 사유
    allow_missing: bool = False
    plots: List[Path] = field(default_factory=list)

    def violations(self) -> List[MetricRow]:
        return [r for r in self.rows if r.passed is False]

    def data_ok(self) -> bool:
        return self.allow_missing or not self.insufficient

    def passed(self, strict: bool = False) -> bool:
        if self.violations() or not self.data_ok():
            return False
        return not (strict and self.warnings)

    def verdict(self, strict: bool = False) -> str:
        if self.violations():
            return 'FAIL'
        if not self.data_ok():
            return 'INSUFFICIENT'
        if strict and self.warnings:
            return 'FAIL'
        return 'PASS'

    def exit_code(self, strict: bool = False, no_fail: bool = False) -> int:
        """종료 코드 (모듈 docstring). 위반이 있으면 1 이 데이터 부족 3 보다 앞선다."""
        if self.violations() and not no_fail:
            return EXIT_FAIL
        if not self.data_ok():
            return EXIT_INSUFFICIENT
        if strict and self.warnings and not no_fail:
            return EXIT_FAIL
        return EXIT_PASS

    def to_markdown(self, strict: bool = False) -> str:
        lines = [f'# 성능 측정 결과: {self.run_dir.name}', '',
                 f'런 디렉토리: `{self.run_dir}`', '',
                 '| 지표 | 구간 | 샘플 수 | 통계 | 측정값 | 기준 | 판정 | 참고 |',
                 '| --- | --- | --- | --- | --- | --- | --- | --- |']
        for r in self.rows:
            thr = '-' if r.threshold is None else _fmt(r.threshold, r.unit)
            verdict = '-' if r.passed is None else ('PASS' if r.passed else '**FAIL**')
            extras = ' / '.join(
                f'{k} {v if isinstance(v, str) else _fmt(v, r.unit)}'
                for k, v in r.extras.items())
            lines.append(f'| {r.metric} | {r.segment} | {r.count} | {r.stat} | '
                         f'{_fmt(r.value, r.unit)} | {thr} | {verdict} | {extras or "-"} |')
        lines.append('')
        if self.insufficient:
            tail = ' (--allow-missing: 판정에서 제외):' if self.allow_missing else ':'
            lines.append('데이터 부족' + tail)
            lines.extend(f'- {w}' for w in self.insufficient)
            lines.append('')
        if self.warnings:
            lines.append('경고:')
            lines.extend(f'- {w}' for w in self.warnings)
            lines.append('')
        if self.plots:
            lines.append('그림:')
            lines.extend(f'- `{p.name}`' for p in self.plots)
            lines.append('')
        names = {'PASS': 'PASS', 'FAIL': 'FAIL', 'INSUFFICIENT': 'INSUFFICIENT (데이터 부족)'}
        lines.append(f'종합 판정: **{names[self.verdict(strict)]}**'
                     + (' (strict: 경고도 실패로 간주)' if strict else ''))
        lines.append('')
        return '\n'.join(lines)


def _fmt(v: float, unit: str) -> str:
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return 'n/a'
    if unit == 'm':
        return f'{v * 100:.2f} cm'
    if unit == 'ms':
        return f'{v:.1f} ms'
    if unit == '%':
        return f'{v:.1f} %'
    if unit == 'deg':
        return f'{v:.2f}°'
    if unit == 'ratio':
        return f'{v:.2f}'
    return f'{v:.4g} {unit}'.strip()


def _summary_row(metric: str, segment: str, values: Sequence[float], stat: str, unit: str,
                 threshold: Optional[float], gate: bool = True) -> MetricRow:
    s = metrics.summarize(values)
    value = getattr(s, stat) if s.count else float('nan')
    passed: Optional[bool] = None
    if gate and threshold is not None and s.count:
        passed = bool(value <= threshold)
    extras = {} if not s.count else {'max': s.max, 'p95': s.p95, 'mean': s.mean}
    extras.pop(stat, None)
    return MetricRow(metric, segment, s.count, stat, value, unit, threshold, passed, extras,
                     gated=gate and threshold is not None)


def _floats(rows: Sequence[Dict[str, Any]], name: str) -> np.ndarray:
    """열 → float 배열 (숫자가 아닌 칸은 NaN)."""
    out = []
    for r in rows:
        v = r.get(name)
        out.append(float(v) if isinstance(v, float) else float('nan'))
    return np.asarray(out, dtype=float)


def analyze_pose(rows: Sequence[Dict[str, Any]], th: Thresholds, gate_metric: str = 'rmse',
                 align: bool = False) -> List[MetricRow]:
    """위치 추정 오차: 구간별 RMSE/max/p95 (+ 전체, 헤딩 오차, 선택적 SE(2) 정렬 후)."""
    out: List[MetricRow] = []
    if not rows:
        return out
    seg = np.array(io.column(rows, 'segment'))
    err = _floats(rows, 'error')
    for name in segments.MOTION_SEGMENTS:
        out.append(_summary_row(POSE_METRIC, name, err[seg == name], gate_metric, 'm',
                                th.pose.get(name)))
    out.append(_summary_row(POSE_METRIC, ALL, err, gate_metric, 'm', None, gate=False))
    if 'yaw_error' in rows[0]:
        yaw = np.degrees(_floats(rows, 'yaw_error'))
        out.append(_summary_row(POSE_METRIC + ' (헤딩)', ALL, yaw, gate_metric, 'deg', None,
                                gate=False))
    if align and len(rows) >= 2:
        gt = np.column_stack([_floats(rows, 'gt_x'), _floats(rows, 'gt_y')])
        est = np.column_stack([_floats(rows, 'est_x'), _floats(rows, 'est_y')])
        yaw, tx, ty = metrics.se2_align(est, gt)
        aligned = metrics.apply_se2(est, yaw, tx, ty)
        err_al = np.linalg.norm(aligned - gt, axis=1)
        for name in segments.MOTION_SEGMENTS:
            out.append(_summary_row(POSE_ALIGNED_METRIC, name, err_al[seg == name], gate_metric,
                                    'm', th.pose.get(name), gate=False))
        out.append(MetricRow(POSE_ALIGNED_METRIC, '정렬 변환', len(rows), 'yaw',
                             math.degrees(yaw), 'deg',
                             extras={'tx': f'{tx:.4f} m', 'ty': f'{ty:.4f} m'}))
    return out


def analyze_cte(rows: Sequence[Dict[str, Any]], th: Thresholds) -> List[MetricRow]:
    """경로 추종 CTE: 구간별 평균 |CTE| (명세 산출 방법) 를 판정, RMSE/max/p95 병기."""
    out: List[MetricRow] = []
    if not rows:
        return out
    seg = np.array(io.column(rows, 'segment'))
    cte = _floats(rows, 'cte')
    for name in segments.PATH_SEGMENTS:
        out.append(_summary_row(CTE_METRIC, name, cte[seg == name], 'mean', 'm',
                                th.cte.get(name)))
    out.append(_summary_row(CTE_METRIC, ALL, cte, 'mean', 'm', None, gate=False))
    return out


def _run_rtf(resp: Sequence[Dict[str, Any]], cpu: Sequence[Dict[str, Any]]) -> np.ndarray:
    """응답·CPU 로그의 rtf 값 (유한한 양수만)."""
    vals = []
    for rows in (resp, cpu):
        if rows and 'rtf' in rows[0]:
            v = _floats(rows, 'rtf')
            vals.append(v[np.isfinite(v) & (v > 0.0)])
    return np.concatenate(vals) if vals else np.zeros(0)


def analyze_response(rows: Sequence[Dict[str, Any]], th: Thresholds, clock: str = 'wall',
                     rtf_fill: float = float('nan'), notes: Optional[List[str]] = None
                     ) -> List[MetricRow]:
    """
    응답 시간: 평균(판정) / 최대.

    clock='wall' (기본): 벽시계 환산 지연(latency_wall_ms; 비어 있는 행은 latency_ms / rtf_fill)으로 판정하고
    스탬프 기준 지연은 참고 행 — 설계(multi_robot.md §7)가 응답 KPI 를 실시간 기준으로 둔다.
    환산값을 만들 수 없으면(옛 로그·RTF 미측정) 스탬프 기준으로 판정하고 notes 에 적는다.
    """
    if not rows:
        return []
    lat = _floats(rows, 'latency_ms')
    wall = _floats(rows, 'latency_wall_ms') if 'latency_wall_ms' in rows[0] else \
        np.full(len(rows), np.nan)
    if math.isfinite(rtf_fill) and rtf_fill > 0.0:
        wall = np.where(np.isfinite(wall), wall, lat / rtf_fill)
    has_wall = bool(np.isfinite(wall).any())
    if clock == 'wall' and not has_wall:
        if notes is not None:
            notes.append('벽시계 환산 지연을 만들 수 없어(rtf 없음) 스탬프 기준 지연으로 판정 — '
                         'use_sim_time 이고 RTF < 1 이면 실제보다 짧게 보인다')
        clock = 'sim'
    out = [_summary_row(RESPONSE_METRIC, '스탬프 기준', lat, 'mean', 'ms', th.response_mean_ms,
                        gate=clock == 'sim')]
    if has_wall:
        out.insert(0 if clock == 'wall' else 1, _summary_row(
            RESPONSE_METRIC, '벽시계 환산', wall, 'mean', 'ms', th.response_mean_ms,
            gate=clock == 'wall'))
    return out


def cpu_percent_columns(rows: Sequence[Dict[str, Any]]) -> List[str]:
    """CPU 로그의 사용률 열 (cpu_total_percent, cpu_<그룹>_percent, cpu_cgroup_percent)."""
    if not rows:
        return []
    return [c for c in rows[0] if c.startswith('cpu_') and c.endswith('_percent')]


def analyze_cpu(rows: Sequence[Dict[str, Any]], th: Thresholds,
                column: str = 'cpu_total_percent') -> List[MetricRow]:
    """CPU 사용률: column 평균(판정) / 최대 / p95 + 나머지 사용률 열·호스트 부하는 참고 행."""
    if not rows:
        return []
    out = []
    if column in rows[0]:
        out.append(_summary_row(CPU_METRIC, column, _floats(rows, column), 'mean', '%',
                                th.cpu_mean_percent))
    for col in cpu_percent_columns(rows):
        if col == column:
            continue
        row = _summary_row(CPU_METRIC, f'{col} (참고)', _floats(rows, col), 'mean', '%',
                           th.cpu_mean_percent, gate=False)
        procs = col.replace('cpu_', 'procs_', 1).replace('_percent', '')
        if procs in rows[0]:
            n = _floats(rows, procs)
            n = n[np.isfinite(n)]
            if n.size:
                row.extras['프로세스'] = f'{int(np.median(n))} 개 (중앙값)'
        out.append(row)
    if 'load1' in rows[0]:
        out.append(_summary_row(CPU_METRIC, '호스트 부하 load1 (참고)', _floats(rows, 'load1'),
                                'mean', '', None, gate=False))
    return out


def _numeric_rows(rows: List[Dict[str, Any]], cols: Sequence[str]) -> Tuple[list, int]:
    """필수 열이 모두 숫자인 행만 → (행, 버린 수)."""
    good = [r for r in rows if all(isinstance(r.get(c), float) for c in cols)]
    return good, len(rows) - len(good)


def load_metric(run_dir: Path, key: str, report: Report, required: bool) -> List[Dict[str, Any]]:
    """
    한 지표의 CSV 들(<지표>.csv, <지표>_<ns>.csv)을 읽어 합친다. 행마다 '_src' (파일 이름) 를 단다.

    파일 없음·헤더뿐·필수 열 없음은 required 면 데이터 부족, 아니면 경고로 적는다.
    잘린 줄(열 수 불일치)·숫자가 아닌 칸은 버리고 경고한다.
    """
    name = METRIC_FILES[key]
    missing = report.insufficient if required else report.warnings
    files = io.metric_files(run_dir, name)
    stem = Path(name).stem
    if not files:
        missing.append(f'{stem}*.csv 없음')
        return []
    rows: List[Dict[str, Any]] = []
    for path in files:
        header = io.read_header(path)
        lacking = [c for c in REQUIRED_COLUMNS[key] if c not in header]
        if lacking:
            missing.append(f'{path.name}: 필수 열 없음 ({", ".join(lacking)})')
            continue
        skipped: List[int] = []
        file_rows = io.read_csv(path, skipped)
        file_rows, bad = _numeric_rows(file_rows, REQUIRED_COLUMNS[key])
        if skipped or bad:
            where = f' (줄 {", ".join(map(str, skipped[:5]))})' if skipped else ''
            report.warnings.append(f'{path.name}: 손상된 행 {len(skipped) + bad} 개 건너뜀{where}')
        for r in file_rows:
            r['_src'] = path.name
        rows += file_rows
    if not rows:
        missing.append(f'{", ".join(p.name for p in files)}: 유효한 행이 없음 (헤더뿐)')
    return rows


def _check_counts(report: Report, rows: List[MetricRow], metric: str, min_n: int,
                  required: bool) -> None:
    """판정 구간 표본 수 < min_n → 데이터 부족 (필수 지표) 또는 경고."""
    sink = report.insufficient if required else report.warnings
    for r in rows:
        if r.metric == metric and r.gated and r.count < min_n:
            sink.append(f'{metric} "{r.segment}" 표본 {r.count} 개 < 최소 {min_n} 개'
                        + (' — 판정 불가' if r.count == 0 else ''))


def _check_clock_domains(report: Report, data: Dict[str, List[Dict[str, Any]]]) -> None:
    """파일별 시각 열의 시계 영역(시뮬/벽시계)이 섞였으면 경고."""
    found: Dict[str, List[str]] = {}
    for key, rows in data.items():
        by_file: Dict[str, List[float]] = {}
        for r in rows:
            for col in TIME_COLUMNS[key]:
                by_file.setdefault(r['_src'], []).append(r.get(col))
        for src, values in by_file.items():
            for dom in clocks.domains_of(values):
                found.setdefault(dom, []).append(src)
    if len(found) > 1:
        detail = '; '.join(f'{dom}: {", ".join(sorted(set(srcs)))}' for dom, srcs in found.items())
        report.warnings.append(f'시계 영역 혼합 (시뮬 시간과 벽시계 에포크): {detail} — 노드들의 '
                               'use_sim_time 을 맞춘다')


def analyze_run(run_dir: Path, th: Thresholds = Thresholds(), gate_metric: str = 'rmse',
                align: bool = False, plots: bool = True,
                require: Sequence[str] = METRIC_KEYS, allow_missing: bool = False,
                response_clock: str = 'wall', cpu_column: str = 'cpu_total_percent') -> Report:
    """런 디렉토리 하나를 분석해 Report 를 만든다 (그림은 plots=True 일 때 PNG 로 저장)."""
    run_dir = Path(run_dir)
    report = Report(run_dir, allow_missing=allow_missing)
    req = set(require)
    data = {key: load_metric(run_dir, key, report, key in req) for key in METRIC_KEYS}
    pose, cte, resp, cpu = data['pose'], data['cte'], data['response'], data['cpu']

    rtf = _run_rtf(resp, cpu)
    rtf_med = float(np.median(rtf)) if rtf.size else float('nan')
    report.rows += analyze_pose(pose, th, gate_metric, align)
    report.rows += analyze_cte(cte, th)
    report.rows += analyze_response(resp, th, response_clock, rtf_med, report.warnings)
    report.rows += analyze_cpu(cpu, th, cpu_column)
    if rtf.size:
        report.rows.append(_summary_row(RTF_METRIC, ALL, rtf, 'mean', 'ratio', None, gate=False))
        if float(np.mean(rtf)) < RTF_WARN_BELOW:
            report.warnings.append(
                f'RTF 평균 {float(np.mean(rtf)):.2f} < {RTF_WARN_BELOW} — 시뮬레이션이 실시간보다 느리다: '
                '스탬프(시뮬 시간) 기준 지연은 벽시계 지연의 RTF 배로 짧게 보인다')

    _check_counts(report, report.rows, POSE_METRIC, th.pose_min_samples, 'pose' in req)
    _check_counts(report, report.rows, CTE_METRIC, th.cte_min_samples, 'cte' in req)
    _check_counts(report, report.rows, RESPONSE_METRIC, th.response_min_samples,
                  'response' in req)
    _check_counts(report, report.rows, CPU_METRIC, th.cpu_min_samples, 'cpu' in req)
    if cpu and cpu_column not in cpu[0]:
        (report.insufficient if 'cpu' in req else report.warnings).append(
            f'CPU 열 {cpu_column} 없음 (있는 열: {", ".join(cpu_percent_columns(cpu)) or "-"})')
    _cpu_attribution_note(report, cpu, cpu_column, th)
    _check_clock_domains(report, data)

    if plots:
        report.plots = write_plots(run_dir, th, pose, cte, resp, cpu, cpu_column)
    return report


def _cpu_attribution_note(report: Report, cpu: List[Dict[str, Any]], column: str,
                          th: Thresholds) -> None:
    """호스트 전체로 판정해 실패했는데 우리 프로세스 귀속 값은 기준 안이면 알려 준다."""
    if not cpu or column != 'cpu_total_percent' or 'cpu_ros_percent' not in cpu[0]:
        return
    total = next((r for r in report.rows if r.metric == CPU_METRIC and r.segment == column), None)
    ros = metrics.summarize(_floats(cpu, 'cpu_ros_percent'))
    if (total is not None and total.passed is False and ros.count
            and ros.mean <= th.cpu_mean_percent):
        report.warnings.append(
            f'호스트 전체 CPU 가 기준을 넘었지만 우리 ROS 프로세스 귀속 값은 평균 {ros.mean:.1f} % — '
            '공유 호스트의 다른 부하일 수 있다. 시스템 전체가 보이는 곳(같은 컨테이너/pid: host)에서 '
            '쟀다면 --cpu-column cpu_ros_percent 로 판정한다')


# --- 그림 ------------------------------------------------------------------

def _plt():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.style.use('tableau-colorblind10')
    return plt


def _plot_series_by_segment(plt, path: Path, t, y, seg, title: str, ylabel: str,
                            thresholds: Dict[str, float], unit_scale: float = 1.0) -> Path:
    fig, ax = plt.subplots(figsize=(9, 3.6))
    order = np.argsort(t, kind='stable')
    t, y, seg = t[order], y[order], seg[order]
    t0 = t[0] if len(t) else 0.0
    for name in dict.fromkeys(seg):
        m = seg == name
        ax.plot(t[m] - t0, y[m] * unit_scale, '.', ms=3, color=SEGMENT_COLORS.get(name),
                label=SEGMENT_EN.get(name, name))
    for name, thr in thresholds.items():
        ax.axhline(thr * unit_scale, ls='--', lw=1, color=SEGMENT_COLORS.get(name),
                   alpha=0.7, label=f'{SEGMENT_EN.get(name, name)} limit')
    ax.set_xlabel('time [s]')
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(alpha=0.3)
    ax.legend(loc='upper right', fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def write_plots(run_dir: Path, th: Thresholds, pose, cte, resp, cpu,
                cpu_column: str = 'cpu_total_percent') -> List[Path]:
    """있는 로그마다 PNG 한 장씩 (Agg 백엔드, 디스플레이 불필요)."""
    plt = _plt()
    out: List[Path] = []
    if pose:
        t = _floats(pose, 'timestamp')
        seg = np.array(io.column(pose, 'segment'))
        out.append(_plot_series_by_segment(
            plt, run_dir / 'pose_error.png', t, _floats(pose, 'error'), seg,
            'Pose error vs ground truth', 'error [cm]', th.pose, 100.0))
        fig, ax = plt.subplots(figsize=(5, 5))
        for src in dict.fromkeys(r['_src'] for r in pose):
            part = [r for r in pose if r['_src'] == src]
            tag = Path(src).stem.replace('pose_error', '').strip('_')
            ax.plot(_floats(part, 'gt_x'), _floats(part, 'gt_y'), '-', lw=1.5,
                    label=f'GT {tag}'.strip())
            ax.plot(_floats(part, 'est_x'), _floats(part, 'est_y'), '.', ms=2,
                    label=f'estimate {tag}'.strip())
        ax.set_aspect('equal')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_title('Trajectory')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        p = run_dir / 'trajectory.png'
        fig.savefig(p, dpi=120)
        plt.close(fig)
        out.append(p)
    if cte:
        out.append(_plot_series_by_segment(
            plt, run_dir / 'cte.png', _floats(cte, 'timestamp'), _floats(cte, 'cte'),
            np.array(io.column(cte, 'segment')), 'Cross-track error (left +)', 'cte [cm]',
            th.cte, 100.0))
    if resp:
        lat = _floats(resp, 'latency_ms')
        fig, ax = plt.subplots(figsize=(9, 3.6))
        ax.bar(np.arange(len(lat)), lat, width=0.8, label='stamp')
        if 'latency_wall_ms' in resp[0]:
            ax.plot(np.arange(len(lat)), _floats(resp, 'latency_wall_ms'), 'k.', ms=4,
                    label='wall (latency / rtf)')
        ax.axhline(th.response_mean_ms, ls='--', lw=1, color='#595959', label='limit (mean)')
        ax.set_xlabel('command #')
        ax.set_ylabel('latency [ms]')
        ax.set_title('Response time (command -> first motion)')
        ax.grid(alpha=0.3, axis='y')
        ax.legend(fontsize=8)
        fig.tight_layout()
        p = run_dir / 'response_time.png'
        fig.savefig(p, dpi=120)
        plt.close(fig)
        out.append(p)
    if cpu:
        t = _floats(cpu, 'timestamp')
        order = np.argsort(t, kind='stable')
        fig, ax = plt.subplots(figsize=(9, 3.6))
        for col in cpu_percent_columns(cpu):
            ax.plot(t[order] - t[order][0], _floats(cpu, col)[order], '-',
                    lw=1.8 if col == cpu_column else 1.0,
                    label=col.replace('cpu_', '').replace('_percent', ''))
        ax.axhline(th.cpu_mean_percent, ls='--', lw=1, color='#595959', label='limit (mean)')
        ax.set_ylim(0, 100)
        ax.set_xlabel('time [s]')
        ax.set_ylabel('cpu [%]')
        ax.set_title(f'CPU usage (gate: {cpu_column})')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        p = run_dir / 'cpu.png'
        fig.savefig(p, dpi=120)
        plt.close(fig)
        out.append(p)
    return out


# --- CLI -------------------------------------------------------------------

def _float_list(text: str, n: int, name: str) -> List[float]:
    vals = [float(v) for v in text.split(',')]
    if len(vals) != n:
        raise argparse.ArgumentTypeError(f'{name}: {n} values expected, got {len(vals)}')
    return vals


def _metric_list(text: str) -> List[str]:
    vals = [v.strip() for v in text.split(',') if v.strip()]
    bad = [v for v in vals if v not in METRIC_KEYS]
    if bad:
        raise argparse.ArgumentTypeError(f'unknown metric(s) {bad}; choose from {METRIC_KEYS}')
    return vals


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='analyze', description='amr_evaluation 로그 분석 (명세 4.10 기준 판정). '
        '종료 코드: 0 통과, 1 기준 위반, 2 입력 없음, 3 데이터 부족, 4 오류')
    p.add_argument('--input', '-i', default='',
                   help='런 디렉토리 (기본: $ROS_WS/logs/eval 아래 가장 최근 런)')
    p.add_argument('--no-fail', action='store_true', help='기준 위반이 있어도 종료 코드 0')
    p.add_argument('--strict', action='store_true', help='경고도 실패로 간주 (종료 코드 1)')
    p.add_argument('--allow-missing', action='store_true',
                   help='데이터 부족(CSV 없음·헤더뿐·구간 표본 부족)을 실패(3)가 아니라 경고로')
    p.add_argument('--require', type=_metric_list, default=list(METRIC_KEYS),
                   metavar='pose,cte,response,cpu',
                   help='반드시 있어야 하는 지표 (기본 전부). 나머지는 있으면 판정, 없으면 경고')
    p.add_argument('--no-plots', action='store_true', help='PNG 그림 생략')
    p.add_argument('--align', action='store_true',
                   help='SE(2) 정렬 후 위치 오차도 참고로 계산 (map↔world 상수 오프셋 확인용)')
    p.add_argument('--gate-metric', choices=GATE_METRICS, default='rmse',
                   help='위치 추정 오차 판정에 쓰는 통계 (기본 rmse)')
    p.add_argument('--pose-thresholds', default='0.03,0.05,0.08', metavar='STOP,STRAIGHT,TURN',
                   help='위치 오차 기준 [m] (기본 0.03,0.05,0.08)')
    p.add_argument('--cte-thresholds', default='0.05,0.10', metavar='STRAIGHT,CURVE',
                   help='CTE 기준 [m] (기본 0.05,0.10)')
    p.add_argument('--response-ms', type=float, default=200.0, help='평균 응답 시간 기준 [ms]')
    p.add_argument('--response-clock', choices=('wall', 'sim'), default='wall',
                   help='응답 판정 시계: wall = 벽시계 환산(latency/rtf, 기본), sim = 스탬프 기준')
    p.add_argument('--cpu-percent', type=float, default=80.0, help='평균 CPU 사용률 기준 [%%]')
    p.add_argument('--cpu-column', default='cpu_total_percent',
                   help='CPU 판정 열 (기본 호스트 전체 cpu_total_percent; 시스템 귀속은 '
                        'cpu_ros_percent / cpu_cgroup_percent)')
    p.add_argument('--min-pose-samples', type=int, default=100, help='위치 오차 구간별 최소 표본')
    p.add_argument('--min-cte-samples', type=int, default=100, help='CTE 구간별 최소 표본')
    p.add_argument('--min-response-samples', type=int, default=50, help='응답 시간 최소 표본')
    p.add_argument('--min-cpu-samples', type=int, default=60, help='CPU 최소 표본')
    return p


def thresholds_from_args(args: argparse.Namespace) -> Thresholds:
    pose = _float_list(args.pose_thresholds, 3, '--pose-thresholds')
    cte = _float_list(args.cte_thresholds, 2, '--cte-thresholds')
    return Thresholds(
        pose=dict(zip(segments.MOTION_SEGMENTS, pose)),
        cte=dict(zip(segments.PATH_SEGMENTS, cte)),
        response_mean_ms=args.response_ms,
        cpu_mean_percent=args.cpu_percent,
        pose_min_samples=args.min_pose_samples,
        cte_min_samples=args.min_cte_samples,
        response_min_samples=args.min_response_samples,
        cpu_min_samples=args.min_cpu_samples,
    )


def latest_run_dir(root: Path) -> Optional[Path]:
    """가장 최근에 수정된 root 의 하위 디렉토리 (없으면 None)."""
    if not root.is_dir():
        return None
    runs = [p for p in root.iterdir() if p.is_dir()]
    return max(runs, key=lambda p: p.stat().st_mtime) if runs else None


def run(argv: Optional[Sequence[str]] = None) -> int:
    """CLI 본체. 종료 코드는 모듈 docstring."""
    args = build_parser().parse_args(argv)
    run_dir = Path(args.input) if args.input else latest_run_dir(io.default_log_root())
    if run_dir is None or not run_dir.is_dir():
        print(f'런 디렉토리를 찾을 수 없다: {run_dir or io.default_log_root()}', file=sys.stderr)
        return EXIT_NO_INPUT
    try:
        th = thresholds_from_args(args)
    except (ValueError, argparse.ArgumentTypeError) as exc:
        print(f'인자 오류: {exc}', file=sys.stderr)
        return EXIT_NO_INPUT
    report = analyze_run(run_dir, th, args.gate_metric, args.align, plots=not args.no_plots,
                         require=args.require, allow_missing=args.allow_missing,
                         response_clock=args.response_clock, cpu_column=args.cpu_column)
    text = report.to_markdown(args.strict)
    io.write_text(run_dir / io.REPORT_FILE, text)
    print(text)
    print(f'리포트: {run_dir / io.REPORT_FILE}')
    return report.exit_code(args.strict, args.no_fail)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI 진입점: 예상하지 못한 예외는 트레이스백을 찍고 4 (기준 위반 1 과 구분)."""
    try:
        return run(argv)
    except SystemExit as exc:          # argparse 오류(2)·--help(0)
        return int(exc.code) if isinstance(exc.code, int) else EXIT_NO_INPUT
    except Exception:                  # CI 가 오류와 기준 위반을 구분하도록 모든 예외를 4 로
        traceback.print_exc()
        return EXIT_ERROR


if __name__ == '__main__':
    sys.exit(main())
