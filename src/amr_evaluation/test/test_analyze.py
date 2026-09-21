import math
from pathlib import Path

from amr_evaluation import analyze
from amr_evaluation import io
from amr_evaluation import segments
import numpy as np
import pytest


def make_run(root: Path, noise: float = 0.005, n: int = 300, offset=(0.0, 0.0),
             cte_curve: float = 0.02, latency_ms: float = 120.0, cpu: float = 35.0,
             n_resp: int = 60, seed: int = 1) -> Path:
    """합성 런: 원(r=2, 0.5 m/s) 위 GT + 정지 구간, 추정 = GT + N(0, noise) + offset."""
    rng = np.random.default_rng(seed)
    run = root / 'run_test'
    run.mkdir(parents=True, exist_ok=True)
    with io.CsvWriter(run / io.POSE_FILE, io.POSE_COLUMNS) as w:
        for i in range(n):
            t = i * 0.02
            moving = i % 3 != 0
            th = 0.25 * t
            gx, gy = 2 * math.cos(th), 2 * math.sin(th)
            ex, ey = gx + offset[0] + rng.normal(0, noise), gy + offset[1] + rng.normal(0, noise)
            seg = segments.TURN if moving else segments.STOP
            if i % 7 == 0:
                seg = segments.STRAIGHT
            w.write([t, gx, gy, ex, ey, math.hypot(ex - gx, ey - gy), rng.normal(0, 0.01), seg])
    with io.CsvWriter(run / io.CTE_FILE, io.CTE_COLUMNS) as w:
        for i in range(n):
            seg = segments.STRAIGHT if i % 2 else segments.CURVE
            e = (0.01 if seg == segments.STRAIGHT else cte_curve) * (1 if i % 4 else -1)
            w.write([i * 0.05, 0.0, 0.0, 0.0, e, e, seg])
    with io.CsvWriter(run / io.RESPONSE_FILE, io.RESPONSE_COLUMNS) as w:
        for i in range(n_resp):
            w.write([float(i), i + latency_ms / 1000.0, latency_ms, f'task_{i}'])
    with io.CsvWriter(run / io.CPU_FILE, io.CPU_BASE_COLUMNS + ['cpu0', 'cpu1']) as w:
        for i in range(30):
            w.write([float(i), cpu, cpu + 5, cpu - 5])
    return run


def test_passing_run(tmp_path):
    run = make_run(tmp_path)
    report = analyze.analyze_run(run, plots=False)
    assert report.violations() == []
    assert report.passed()
    metrics_seen = {(r.metric, r.segment) for r in report.rows}
    assert (analyze.POSE_METRIC, segments.STOP) in metrics_seen
    assert (analyze.POSE_METRIC, segments.TURN) in metrics_seen
    assert (analyze.CTE_METRIC, segments.CURVE) in metrics_seen
    assert (analyze.RESPONSE_METRIC, analyze.ALL) in metrics_seen
    assert (analyze.CPU_METRIC, analyze.ALL) in metrics_seen
    turn = next(r for r in report.rows
                if r.metric == analyze.POSE_METRIC and r.segment == segments.TURN)
    assert turn.stat == 'rmse' and 0.003 < turn.value < 0.012
    assert set(turn.extras) == {'max', 'p95', 'mean'}
    md = report.to_markdown()
    assert '**FAIL**' not in md and '종합 판정: **PASS**' in md
    assert not report.warnings


def test_main_exit_codes(tmp_path):
    run = make_run(tmp_path)
    assert analyze.main(['--input', str(run), '--no-plots']) == 0
    assert (run / io.REPORT_FILE).is_file()
    bad = make_run(tmp_path / 'bad', noise=0.10)
    assert analyze.main(['--input', str(bad), '--no-plots']) == 1
    assert analyze.main(['--input', str(bad), '--no-plots', '--no-fail']) == 0
    assert '**FAIL**' in (bad / io.REPORT_FILE).read_text(encoding='utf-8')
    assert analyze.main(['--input', str(tmp_path / 'missing'), '--no-plots']) == 2


def test_main_default_input_is_latest_run(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_WS', str(tmp_path))
    assert analyze.main(['--no-plots']) == 2
    run = make_run(tmp_path / 'logs' / 'eval')
    assert analyze.latest_run_dir(tmp_path / 'logs' / 'eval') == run
    assert analyze.main(['--no-plots']) == 0
    assert analyze.latest_run_dir(tmp_path / 'nope') is None


def test_each_threshold_gates(tmp_path):
    base = Path(tmp_path)
    assert not analyze.analyze_run(make_run(base / 'a', cte_curve=0.12), plots=False).passed()
    assert not analyze.analyze_run(make_run(base / 'b', latency_ms=250.0), plots=False).passed()
    assert not analyze.analyze_run(make_run(base / 'c', cpu=85.0), plots=False).passed()
    # 기준을 느슨하게 주면 통과
    th = analyze.Thresholds(response_mean_ms=300.0, cpu_mean_percent=90.0,
                            cte={segments.STRAIGHT: 0.05, segments.CURVE: 0.2})
    assert analyze.analyze_run(make_run(base / 'd', cte_curve=0.12, latency_ms=250.0,
                                        cpu=85.0), th, plots=False).passed()


def test_gate_metric_max_is_stricter(tmp_path):
    run = make_run(tmp_path, noise=0.02)
    assert analyze.analyze_run(run, gate_metric='rmse', plots=False).passed()
    assert not analyze.analyze_run(run, gate_metric='max', plots=False).passed()


def test_align_removes_constant_offset(tmp_path):
    run = make_run(tmp_path, noise=0.001, offset=(0.5, -0.2))
    report = analyze.analyze_run(run, align=True, plots=False)
    assert not report.passed()                         # 원시 오차는 50 cm 대
    aligned = [r for r in report.rows if r.metric == analyze.POSE_ALIGNED_METRIC
               and r.segment in segments.MOTION_SEGMENTS]
    assert aligned and all(r.value < 0.005 for r in aligned)
    assert all(r.passed is None for r in aligned)      # 참고 행은 판정하지 않는다
    tf = next(r for r in report.rows if r.segment == '정렬 변환')
    assert 'tx' in tf.extras and 'm' in tf.extras['tx']
    assert '정렬 변환' in report.to_markdown()


def test_warnings_and_strict(tmp_path):
    run = make_run(tmp_path, n=30, n_resp=5)
    report = analyze.analyze_run(run, plots=False)
    assert report.passed() and not report.passed(strict=True)
    assert any('100' in w for w in report.warnings)
    assert any('50' in w for w in report.warnings)
    assert analyze.main(['--input', str(run), '--no-plots']) == 0
    assert analyze.main(['--input', str(run), '--no-plots', '--strict']) == 1
    assert 'strict' in (run / io.REPORT_FILE).read_text(encoding='utf-8')


def test_missing_segment_and_missing_files(tmp_path):
    run = tmp_path / 'partial'
    run.mkdir()
    with io.CsvWriter(run / io.POSE_FILE, io.POSE_COLUMNS) as w:
        for i in range(120):
            w.write([i * 0.02, 0.0, 0.0, 0.01, 0.0, 0.01, 0.0, segments.STRAIGHT])
    report = analyze.analyze_run(run, plots=False)
    assert report.passed()
    assert any(segments.STOP in w for w in report.warnings)
    assert any(io.CTE_FILE in w for w in report.warnings)
    stop = next(r for r in report.rows if r.segment == segments.STOP)
    assert stop.count == 0 and stop.passed is None and math.isnan(stop.value)
    assert 'n/a' in report.to_markdown()

    empty = tmp_path / 'empty'
    empty.mkdir()
    report = analyze.analyze_run(empty, plots=False)
    assert report.rows == [] and any('하나도' in w for w in report.warnings)
    (empty / io.CPU_FILE).write_text('timestamp,cpu_total_percent\n', encoding='utf-8')
    assert any('행이 없음' in w for w in analyze.analyze_run(empty, plots=False).warnings)


def test_threshold_args():
    args = analyze.build_parser().parse_args(
        ['--pose-thresholds', '0.01,0.02,0.03', '--cte-thresholds', '0.04,0.05',
         '--response-ms', '150', '--cpu-percent', '70'])
    th = analyze.thresholds_from_args(args)
    assert th.pose == {segments.STOP: 0.01, segments.STRAIGHT: 0.02, segments.TURN: 0.03}
    assert th.cte == {segments.STRAIGHT: 0.04, segments.CURVE: 0.05}
    assert (th.response_mean_ms, th.cpu_mean_percent) == (150.0, 70.0)
    with pytest.raises(Exception):
        analyze.thresholds_from_args(analyze.build_parser().parse_args(
            ['--pose-thresholds', '0.01,0.02']))


def test_fmt_units():
    assert analyze._fmt(0.0123, 'm') == '1.23 cm'
    assert analyze._fmt(123.456, 'ms') == '123.5 ms'
    assert analyze._fmt(45.678, '%') == '45.7 %'
    assert analyze._fmt(1.234, 'deg') == '1.23°'
    assert analyze._fmt(float('nan'), 'm') == 'n/a'
    assert analyze._fmt(2.0, 'x') == '2 x'


def test_plots_written(tmp_path):
    pytest.importorskip('matplotlib')
    run = make_run(tmp_path)
    report = analyze.analyze_run(run, plots=True)
    names = {p.name for p in report.plots}
    assert names == {'pose_error.png', 'trajectory.png', 'cte.png', 'response_time.png',
                     'cpu.png'}
    assert all(p.stat().st_size > 1000 for p in report.plots)
    assert 'pose_error.png' in report.to_markdown()
