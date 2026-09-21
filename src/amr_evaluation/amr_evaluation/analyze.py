"""
측정 로그 분석 CLI (rclpy 비의존).

    ros2 run amr_evaluation analyze --input logs/eval/<run>/
    python3 -m amr_evaluation.analyze --input logs/eval/<run>/

런 디렉토리의 pose_error.csv / cte.csv / response_time.csv / cpu.csv 를 읽어
명세 기준(정지 3 / 직선 5 / 회전 8 cm, CTE 직선 5 / 곡선 10 cm, 응답 200 ms, CPU 80 %)과
비교한 Markdown 표(report.md)와 PNG 그림을 같은 디렉토리에 쓰고, 표를 stdout 에 찍는다.
기준을 하나라도 넘으면 종료 코드 1 (--no-fail 이면 0) 이라 CI/통합 테스트 게이트로 쓸 수 있다.
"""

import argparse
from dataclasses import dataclass, field
import math
from pathlib import Path
import sys
from typing import Any, Dict, List, Optional, Sequence

from amr_evaluation import io
from amr_evaluation import metrics
from amr_evaluation import segments
import numpy as np

POSE_METRIC = '위치 추정 오차'
POSE_ALIGNED_METRIC = '위치 추정 오차 (SE2 정렬 후, 참고)'
CTE_METRIC = '경로 추종 CTE'
RESPONSE_METRIC = '응답 시간'
CPU_METRIC = 'CPU 사용률'
ALL = '전체'

GATE_METRICS = ('rmse', 'max', 'p95', 'mean')

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
    """명세 기준값. 거리는 m, 응답은 ms, CPU 는 %."""

    pose: Dict[str, float] = field(default_factory=lambda: {
        segments.STOP: 0.03, segments.STRAIGHT: 0.05, segments.TURN: 0.08})
    cte: Dict[str, float] = field(default_factory=lambda: {
        segments.STRAIGHT: 0.05, segments.CURVE: 0.10})
    response_mean_ms: float = 200.0
    cpu_mean_percent: float = 80.0
    pose_min_samples: int = 100
    response_min_samples: int = 50


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


@dataclass
class Report:
    """분석 결과."""

    run_dir: Path
    rows: List[MetricRow] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    plots: List[Path] = field(default_factory=list)

    def violations(self) -> List[MetricRow]:
        return [r for r in self.rows if r.passed is False]

    def passed(self, strict: bool = False) -> bool:
        if self.violations():
            return False
        return not (strict and self.warnings)

    def to_markdown(self, strict: bool = False) -> str:
        lines = [f'# 성능 측정 결과: {self.run_dir.name}', '',
                 f'런 디렉토리: `{self.run_dir}`', '',
                 '| 지표 | 구간 | 샘플 수 | 통계 | 측정값 | 기준 | 판정 | 참고 (max / p95 / mean) |',
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
        if self.warnings:
            lines.append('경고:')
            lines.extend(f'- {w}' for w in self.warnings)
            lines.append('')
        if self.plots:
            lines.append('그림:')
            lines.extend(f'- `{p.name}`' for p in self.plots)
            lines.append('')
        lines.append(f'종합 판정: **{"PASS" if self.passed(strict) else "FAIL"}**'
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
    return MetricRow(metric, segment, s.count, stat, value, unit, threshold, passed, extras)


def analyze_pose(rows: Sequence[Dict[str, Any]], th: Thresholds, gate_metric: str = 'rmse',
                 align: bool = False) -> List[MetricRow]:
    """위치 추정 오차: 구간별 RMSE/max/p95 (+ 전체, 헤딩 오차, 선택적 SE(2) 정렬 후)."""
    out: List[MetricRow] = []
    if not rows:
        return out
    seg = np.array(io.column(rows, 'segment'))
    err = np.array(io.column(rows, 'error'), dtype=float)
    for name in segments.MOTION_SEGMENTS:
        out.append(_summary_row(POSE_METRIC, name, err[seg == name], gate_metric, 'm',
                                th.pose.get(name)))
    out.append(_summary_row(POSE_METRIC, ALL, err, gate_metric, 'm', None, gate=False))
    if 'yaw_error' in rows[0]:
        yaw = np.degrees(np.array(io.column(rows, 'yaw_error'), dtype=float))
        out.append(_summary_row(POSE_METRIC + ' (헤딩)', ALL, yaw, gate_metric, 'deg', None,
                                gate=False))
    if align and len(rows) >= 2:
        gt = np.column_stack([io.column(rows, 'gt_x'), io.column(rows, 'gt_y')]).astype(float)
        est = np.column_stack([io.column(rows, 'est_x'), io.column(rows, 'est_y')]).astype(float)
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
    cte = np.array(io.column(rows, 'cte'), dtype=float)
    for name in segments.PATH_SEGMENTS:
        out.append(_summary_row(CTE_METRIC, name, cte[seg == name], 'mean', 'm',
                                th.cte.get(name)))
    out.append(_summary_row(CTE_METRIC, ALL, cte, 'mean', 'm', None, gate=False))
    return out


def analyze_response(rows: Sequence[Dict[str, Any]], th: Thresholds) -> List[MetricRow]:
    """응답 시간: 평균(판정) / 최대."""
    if not rows:
        return []
    lat = np.array(io.column(rows, 'latency_ms'), dtype=float)
    return [_summary_row(RESPONSE_METRIC, ALL, lat, 'mean', 'ms', th.response_mean_ms)]


def analyze_cpu(rows: Sequence[Dict[str, Any]], th: Thresholds) -> List[MetricRow]:
    """CPU 사용률: 평균(판정) / 최대 / p95."""
    if not rows:
        return []
    cpu = np.array(io.column(rows, 'cpu_total_percent'), dtype=float)
    return [_summary_row(CPU_METRIC, ALL, cpu, 'mean', '%', th.cpu_mean_percent)]


def _load(run_dir: Path, name: str, report: Report) -> List[Dict[str, Any]]:
    path = run_dir / name
    if not path.is_file():
        report.warnings.append(f'{name} 없음 — 해당 지표는 건너뜀')
        return []
    rows = io.read_csv(path)
    if not rows:
        report.warnings.append(f'{name} 에 행이 없음')
    return rows


def analyze_run(run_dir: Path, th: Thresholds = Thresholds(), gate_metric: str = 'rmse',
                align: bool = False, plots: bool = True) -> Report:
    """런 디렉토리 하나를 분석해 Report 를 만든다 (그림은 plots=True 일 때 PNG 로 저장)."""
    run_dir = Path(run_dir)
    report = Report(run_dir)
    pose = _load(run_dir, io.POSE_FILE, report)
    cte = _load(run_dir, io.CTE_FILE, report)
    resp = _load(run_dir, io.RESPONSE_FILE, report)
    cpu = _load(run_dir, io.CPU_FILE, report)

    report.rows += analyze_pose(pose, th, gate_metric, align)
    report.rows += analyze_cte(cte, th)
    report.rows += analyze_response(resp, th)
    report.rows += analyze_cpu(cpu, th)

    if pose and len(pose) < th.pose_min_samples:
        report.warnings.append(
            f'위치 추정 샘플 {len(pose)} 개 < 목표 {th.pose_min_samples} 개 (명세: 최소 100 시점)')
    for r in report.rows:
        if r.metric in (POSE_METRIC, CTE_METRIC) and r.segment != ALL and r.count == 0:
            report.warnings.append(f'{r.metric} "{r.segment}" 구간 샘플 없음 — 판정 불가')
    if resp and len(resp) < th.response_min_samples:
        report.warnings.append(
            f'응답 시간 샘플 {len(resp)} 개 < 목표 {th.response_min_samples} 회 (명세: 50 회 이상)')
    if not (pose or cte or resp or cpu):
        report.warnings.append('분석할 로그가 하나도 없음')

    if plots:
        report.plots = write_plots(run_dir, th, pose, cte, resp, cpu)
    return report


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


def write_plots(run_dir: Path, th: Thresholds, pose, cte, resp, cpu) -> List[Path]:
    """있는 로그마다 PNG 한 장씩 (Agg 백엔드, 디스플레이 불필요)."""
    plt = _plt()
    out: List[Path] = []
    if pose:
        t = np.array(io.column(pose, 'timestamp'), dtype=float)
        err = np.array(io.column(pose, 'error'), dtype=float)
        seg = np.array(io.column(pose, 'segment'))
        out.append(_plot_series_by_segment(
            plt, run_dir / 'pose_error.png', t, err, seg, 'Pose error vs ground truth',
            'error [cm]', th.pose, 100.0))
        fig, ax = plt.subplots(figsize=(5, 5))
        ax.plot(io.column(pose, 'gt_x'), io.column(pose, 'gt_y'), '-', lw=1.5, label='GT')
        ax.plot(io.column(pose, 'est_x'), io.column(pose, 'est_y'), '.', ms=2, label='estimate')
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
        t = np.array(io.column(cte, 'timestamp'), dtype=float)
        e = np.array(io.column(cte, 'cte'), dtype=float)
        seg = np.array(io.column(cte, 'segment'))
        out.append(_plot_series_by_segment(
            plt, run_dir / 'cte.png', t, e, seg, 'Cross-track error (left +)', 'cte [cm]',
            th.cte, 100.0))
    if resp:
        lat = np.array(io.column(resp, 'latency_ms'), dtype=float)
        fig, ax = plt.subplots(figsize=(9, 3.6))
        ax.bar(np.arange(len(lat)), lat, width=0.8)
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
        t = np.array(io.column(cpu, 'timestamp'), dtype=float)
        c = np.array(io.column(cpu, 'cpu_total_percent'), dtype=float)
        fig, ax = plt.subplots(figsize=(9, 3.6))
        ax.plot(t - t[0], c, '-', lw=1.5, label='total')
        ax.axhline(th.cpu_mean_percent, ls='--', lw=1, color='#595959', label='limit (mean)')
        ax.set_ylim(0, 100)
        ax.set_xlabel('time [s]')
        ax.set_ylabel('cpu [%]')
        ax.set_title('CPU usage')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        p = run_dir / 'cpu.png'
        fig.savefig(p, dpi=120)
        plt.close(fig)
        out.append(p)
    return out


# --- CLI -------------------------------------------------------------------

def _floats(text: str, n: int, name: str) -> List[float]:
    vals = [float(v) for v in text.split(',')]
    if len(vals) != n:
        raise argparse.ArgumentTypeError(f'{name}: {n} values expected, got {len(vals)}')
    return vals


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='analyze', description='amr_evaluation 로그 분석 (명세 4.10 기준 판정)')
    p.add_argument('--input', '-i', default='',
                   help='런 디렉토리 (기본: $ROS_WS/logs/eval 아래 가장 최근 런)')
    p.add_argument('--no-fail', action='store_true', help='기준 위반이 있어도 종료 코드 0')
    p.add_argument('--strict', action='store_true', help='경고(샘플 부족 등)도 실패로 간주')
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
    p.add_argument('--cpu-percent', type=float, default=80.0, help='평균 CPU 사용률 기준 [%%]')
    return p


def thresholds_from_args(args: argparse.Namespace) -> Thresholds:
    pose = _floats(args.pose_thresholds, 3, '--pose-thresholds')
    cte = _floats(args.cte_thresholds, 2, '--cte-thresholds')
    return Thresholds(
        pose=dict(zip(segments.MOTION_SEGMENTS, pose)),
        cte=dict(zip(segments.PATH_SEGMENTS, cte)),
        response_mean_ms=args.response_ms,
        cpu_mean_percent=args.cpu_percent,
    )


def latest_run_dir(root: Path) -> Optional[Path]:
    """가장 최근에 수정된 root 의 하위 디렉토리 (없으면 None)."""
    if not root.is_dir():
        return None
    runs = [p for p in root.iterdir() if p.is_dir()]
    return max(runs, key=lambda p: p.stat().st_mtime) if runs else None


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI 진입점. 종료 코드: 0 통과, 1 기준 위반, 2 입력 없음."""
    args = build_parser().parse_args(argv)
    run_dir = Path(args.input) if args.input else latest_run_dir(io.default_log_root())
    if run_dir is None or not run_dir.is_dir():
        print(f'런 디렉토리를 찾을 수 없다: {run_dir or io.default_log_root()}', file=sys.stderr)
        return 2
    th = thresholds_from_args(args)
    report = analyze_run(run_dir, th, args.gate_metric, args.align, plots=not args.no_plots)
    text = report.to_markdown(args.strict)
    io.write_text(run_dir / io.REPORT_FILE, text)
    print(text)
    print(f'리포트: {run_dir / io.REPORT_FILE}')
    if report.passed(args.strict) or args.no_fail:
        return 0
    return 1


if __name__ == '__main__':
    sys.exit(main())
