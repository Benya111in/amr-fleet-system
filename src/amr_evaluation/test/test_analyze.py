import math
from pathlib import Path

from amr_evaluation import analyze
from amr_evaluation import io
from amr_evaluation import segments
import numpy as np
import pytest

CPU_COLUMNS = io.CPU_BASE_COLUMNS + ['cpu_ros_percent', 'procs_ros', 'cpu_cgroup_percent',
                                     'rtf', 'load1', 'cpu0', 'cpu1']


def make_run(root: Path, noise: float = 0.005, n: int = 420, offset=(0.0, 0.0),
             cte_curve: float = 0.02, latency_ms: float = 120.0, cpu: float = 35.0,
             cpu_ros: float = 20.0, n_resp: int = 60, n_cpu: int = 70, rtf: float = 1.0,
             seed: int = 1, suffix: str = '', t0: float = 0.0) -> Path:
    """합성 런: 원(r=2, 0.5 m/s) 위 GT, 정지/직선/회전 구간 각 n/3, 추정 = GT + N(0, noise) + offset."""
    rng = np.random.default_rng(seed)
    run = root / 'run_test'
    run.mkdir(parents=True, exist_ok=True)
    seg_cycle = (segments.STOP, segments.STRAIGHT, segments.TURN)
    with io.CsvWriter(run / f'pose_error{suffix}.csv', io.POSE_COLUMNS) as w:
        for i in range(n):
            t = t0 + i * 0.02
            th = 0.25 * i * 0.02
            gx, gy = 2 * math.cos(th), 2 * math.sin(th)
            ex, ey = gx + offset[0] + rng.normal(0, noise), gy + offset[1] + rng.normal(0, noise)
            w.write([t, gx, gy, ex, ey, math.hypot(ex - gx, ey - gy), rng.normal(0, 0.01),
                     seg_cycle[i % 3]])
    with io.CsvWriter(run / f'cte{suffix}.csv', io.CTE_COLUMNS) as w:
        for i in range(300):
            seg = segments.STRAIGHT if i % 2 else segments.CURVE
            e = (0.01 if seg == segments.STRAIGHT else cte_curve) * (1 if i % 4 else -1)
            w.write([t0 + i * 0.05, 0.0, 0.0, 0.0, e, e, seg])
    with io.CsvWriter(run / f'response_time{suffix}.csv', io.RESPONSE_COLUMNS) as w:
        for i in range(n_resp):
            w.write([t0 + float(i), t0 + i + latency_ms / 1000.0, latency_ms, f'task_{i}',
                     'dispatch', rtf, latency_ms / rtf])
    with io.CsvWriter(run / f'cpu{suffix}.csv', CPU_COLUMNS) as w:
        for i in range(n_cpu):
            w.write([t0 + float(i), cpu, cpu_ros, 7, cpu_ros + 1.0, rtf, 3.5, cpu + 5, cpu - 5])
    return run


def test_passing_run(tmp_path):
    run = make_run(tmp_path)
    report = analyze.analyze_run(run, plots=False)
    assert report.violations() == [] and report.insufficient == []
    assert report.passed() and report.verdict() == 'PASS' and report.exit_code() == 0
    metrics_seen = {(r.metric, r.segment) for r in report.rows}
    assert (analyze.POSE_METRIC, segments.STOP) in metrics_seen
    assert (analyze.POSE_METRIC, segments.TURN) in metrics_seen
    assert (analyze.CTE_METRIC, segments.CURVE) in metrics_seen
    assert (analyze.RESPONSE_METRIC, '벽시계 환산') in metrics_seen
    assert (analyze.RESPONSE_METRIC, '스탬프 기준') in metrics_seen
    assert (analyze.CPU_METRIC, 'cpu_total_percent') in metrics_seen
    assert (analyze.CPU_METRIC, 'cpu_ros_percent (참고)') in metrics_seen
    assert (analyze.RTF_METRIC, analyze.ALL) in metrics_seen
    turn = next(r for r in report.rows
                if r.metric == analyze.POSE_METRIC and r.segment == segments.TURN)
    assert turn.stat == 'rmse' and 0.003 < turn.value < 0.012
    assert set(turn.extras) == {'max', 'p95', 'mean'}
    ros = next(r for r in report.rows if r.segment == 'cpu_ros_percent (참고)')
    assert ros.passed is None and ros.extras['프로세스'] == '7 개 (중앙값)'
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
    assert analyze.main(['--input', str(run), '--require', 'pose,bogus']) == 2
    assert analyze.main(['--input', str(run), '--pose-thresholds', '0.1,x,0.2']) == 2


def test_main_default_input_is_latest_run(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_WS', str(tmp_path))
    assert analyze.main(['--no-plots']) == 2
    run = make_run(tmp_path / 'logs' / 'eval')
    assert analyze.latest_run_dir(tmp_path / 'logs' / 'eval') == run
    assert analyze.main(['--no-plots']) == 0
    assert analyze.latest_run_dir(tmp_path / 'nope') is None


def test_unexpected_error_is_exit_4(tmp_path, monkeypatch, capsys):
    run = make_run(tmp_path)

    def boom(*_a, **_k):
        raise RuntimeError('boom')
    monkeypatch.setattr(analyze, 'analyze_run', boom)
    assert analyze.main(['--input', str(run), '--no-plots']) == analyze.EXIT_ERROR
    assert 'RuntimeError: boom' in capsys.readouterr().err


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


def test_empty_run_is_insufficient_not_pass(tmp_path):
    empty = tmp_path / 'empty'
    empty.mkdir()
    report = analyze.analyze_run(empty, plots=False)
    assert report.rows == [] and not report.passed()
    assert report.verdict() == 'INSUFFICIENT' and report.exit_code() == analyze.EXIT_INSUFFICIENT
    assert len(report.insufficient) == 4
    assert analyze.main(['--input', str(empty), '--no-plots']) == 3
    assert 'INSUFFICIENT' in (empty / io.REPORT_FILE).read_text(encoding='utf-8')
    # --no-fail 은 기준 위반만 봐준다
    assert analyze.main(['--input', str(empty), '--no-plots', '--no-fail']) == 3
    assert analyze.main(['--input', str(empty), '--no-plots', '--allow-missing']) == 0


def test_header_only_csvs_are_insufficient(tmp_path):
    run = tmp_path / 'hdr'
    run.mkdir()
    for name, cols in ((io.POSE_FILE, io.POSE_COLUMNS), (io.CTE_FILE, io.CTE_COLUMNS),
                       (io.RESPONSE_FILE, io.RESPONSE_COLUMNS), (io.CPU_FILE, CPU_COLUMNS)):
        io.CsvWriter(run / name, cols).close()
    report = analyze.analyze_run(run, plots=False)
    assert report.exit_code() == 3
    assert sum('헤더뿐' in w for w in report.insufficient) == 4
    md = report.to_markdown()
    assert '데이터 부족:' in md and 'INSUFFICIENT' in md


def test_missing_segment_and_partial_runs(tmp_path):
    run = tmp_path / 'partial'
    run.mkdir()
    with io.CsvWriter(run / io.POSE_FILE, io.POSE_COLUMNS) as w:
        for i in range(120):
            w.write([i * 0.02, 0.0, 0.0, 0.01, 0.0, 0.01, 0.0, segments.STRAIGHT])
    report = analyze.analyze_run(run, plots=False)
    assert not report.passed() and report.exit_code() == 3
    assert any(segments.STOP in w and '판정 불가' in w for w in report.insufficient)
    assert any('cte*.csv 없음' == w for w in report.insufficient)
    stop = next(r for r in report.rows if r.segment == segments.STOP)
    assert stop.count == 0 and stop.passed is None and math.isnan(stop.value)
    assert 'n/a' in report.to_markdown()
    # 필요한 지표만 요구하면: 직선만 있는 위치 추정 런도 정지/회전 구간 부족으로 여전히 3
    assert analyze.main(['--input', str(run), '--no-plots', '--require', 'pose']) == 3
    # --allow-missing 이면 경고로만 남기고 통과
    loose = analyze.analyze_run(run, plots=False, allow_missing=True)
    assert loose.passed() and loose.exit_code() == 0 and loose.insufficient
    assert '--allow-missing' in loose.to_markdown()


def test_single_metric_run_with_require(tmp_path):
    run = make_run(tmp_path)
    for p in run.glob('*.csv'):
        if not p.name.startswith('response_time'):
            p.unlink()
    assert analyze.main(['--input', str(run), '--no-plots']) == 3
    assert analyze.main(['--input', str(run), '--no-plots', '--require', 'response']) == 0
    report = analyze.analyze_run(run, plots=False, require=['response'])
    assert any('pose_error*.csv 없음' in w for w in report.warnings)


def test_per_segment_minimum(tmp_path):
    run = make_run(tmp_path, n=150)                 # 구간마다 50 개 < 100
    assert analyze.main(['--input', str(run), '--no-plots']) == 3
    assert analyze.main(['--input', str(run), '--no-plots', '--min-pose-samples', '50']) == 0
    report = analyze.analyze_run(make_run(tmp_path / 'r', n_resp=5, n_cpu=10), plots=False)
    assert any('응답 시간' in w and '5 개 < 최소 50' in w for w in report.insufficient)
    assert any('CPU' in w and '10 개 < 최소 60' in w for w in report.insufficient)


def test_warnings_and_strict(tmp_path):
    run = make_run(tmp_path, rtf=0.5)               # RTF 경고만 있고 기준·데이터는 충분
    report = analyze.analyze_run(run, plots=False, response_clock='sim')
    assert report.passed() and not report.passed(strict=True)
    assert any('RTF' in w for w in report.warnings)
    assert analyze.main(['--input', str(run), '--no-plots', '--response-clock', 'sim']) == 0
    assert analyze.main(['--input', str(run), '--no-plots', '--response-clock', 'sim',
                         '--strict']) == 1
    assert 'strict' in (run / io.REPORT_FILE).read_text(encoding='utf-8')


def test_response_gate_uses_wall_clock_by_default(tmp_path):
    # 스탬프 기준 120 ms 지만 RTF 0.5 → 벽시계 240 ms: 기본(wall)은 실패, sim 은 통과
    run = make_run(tmp_path, rtf=0.5)
    report = analyze.analyze_run(run, plots=False)
    wall = next(r for r in report.rows if r.segment == '벽시계 환산')
    stamp = next(r for r in report.rows if r.segment == '스탬프 기준')
    assert wall.value == pytest.approx(240.0) and wall.passed is False
    assert stamp.value == pytest.approx(120.0) and stamp.passed is None
    sim = analyze.analyze_run(run, plots=False, response_clock='sim')
    assert next(r for r in sim.rows if r.segment == '스탬프 기준').passed is True
    assert next(r for r in sim.rows if r.segment == '벽시계 환산').passed is None


def test_response_old_log_without_wall_columns(tmp_path):
    run = make_run(tmp_path)
    (run / io.RESPONSE_FILE).unlink()
    with io.CsvWriter(run / io.RESPONSE_FILE, ['cmd_time', 'response_time', 'latency_ms',
                                               'cmd_id']) as w:
        for i in range(60):
            w.write([float(i), i + 0.1, 100.0, f't{i}'])
    (run / io.CPU_FILE).unlink()
    cols = [c for c in CPU_COLUMNS if c != 'rtf']
    with io.CsvWriter(run / io.CPU_FILE, cols) as w:
        for i in range(70):
            w.write([float(i), 30.0, 20.0, 7, 21.0, 3.5, 30.0, 30.0])
    report = analyze.analyze_run(run, plots=False)
    stamp = next(r for r in report.rows if r.metric == analyze.RESPONSE_METRIC)
    assert stamp.segment == '스탬프 기준' and stamp.passed is True
    assert any('rtf 없음' in w for w in report.warnings)


def test_response_wall_fill_from_cpu_rtf(tmp_path):
    run = make_run(tmp_path, rtf=0.8)
    (run / io.RESPONSE_FILE).unlink()
    with io.CsvWriter(run / io.RESPONSE_FILE, io.RESPONSE_COLUMNS) as w:
        for i in range(60):          # 로거 초반이라 rtf 가 비어 있는 행 → CPU 로그 RTF 로 환산
            w.write([float(i), i + 0.1, 100.0, f't{i}', 'event', float('nan'), float('nan')])
    report = analyze.analyze_run(run, plots=False)
    wall = next(r for r in report.rows if r.segment == '벽시계 환산')
    assert wall.value == pytest.approx(125.0) and wall.passed is True


def test_cpu_column_choice_and_attribution(tmp_path):
    run = make_run(tmp_path, cpu=90.0, cpu_ros=30.0)
    report = analyze.analyze_run(run, plots=False)
    assert not report.passed()
    assert any('귀속' in w and 'cpu_ros_percent' in w for w in report.warnings)
    ok = analyze.analyze_run(run, plots=False, cpu_column='cpu_ros_percent')
    assert ok.passed()
    gated = next(r for r in ok.rows if r.metric == analyze.CPU_METRIC and r.passed is not None)
    assert gated.segment == 'cpu_ros_percent' and gated.value == pytest.approx(30.0)
    assert analyze.main(['--input', str(run), '--no-plots', '--cpu-column', 'cpu_gpu']) == 3


def test_corrupt_rows_are_skipped_with_warning(tmp_path):
    run = make_run(tmp_path)
    with open(run / io.POSE_FILE, 'a', encoding='utf-8') as fh:
        fh.write('9.0,0.0,0.0,0.01,abc,0.01,0.0,직선\n3.000000,0.000000,0.0')   # 숫자 아님 + 잘린 줄
    report = analyze.analyze_run(run, plots=False)
    assert report.passed()
    assert any('pose_error.csv: 손상된 행 2 개' in w for w in report.warnings)
    (run / io.CTE_FILE).write_text('timestamp,planned_x\n1.0,2.0\n', encoding='utf-8')
    report = analyze.analyze_run(run, plots=False)
    assert any('필수 열 없음 (cte)' in w for w in report.insufficient)


def test_multi_robot_files_are_merged(tmp_path):
    make_run(tmp_path, n=210, suffix='_amr_01', n_resp=30, n_cpu=35)
    run = make_run(tmp_path, n=210, suffix='_amr_02', n_resp=30, n_cpu=35, seed=2)
    report = analyze.analyze_run(run, plots=False)
    assert report.passed(), report.insufficient
    stop = next(r for r in report.rows
                if r.metric == analyze.POSE_METRIC and r.segment == segments.STOP)
    assert stop.count == 140
    wall = next(r for r in report.rows if r.segment == '벽시계 환산')
    assert wall.count == 60


def test_clock_domain_mix_warning(tmp_path):
    run = make_run(tmp_path)
    (run / io.RESPONSE_FILE).unlink()
    make_run(tmp_path / 'w', t0=1.79e9)
    (tmp_path / 'w' / 'run_test' / io.RESPONSE_FILE).rename(run / io.RESPONSE_FILE)
    report = analyze.analyze_run(run, plots=False)
    warn = next(w for w in report.warnings if '시계 영역 혼합' in w)
    assert 'wall: response_time.csv' in warn and 'sim:' in warn


def test_threshold_args():
    args = analyze.build_parser().parse_args(
        ['--pose-thresholds', '0.01,0.02,0.03', '--cte-thresholds', '0.04,0.05',
         '--response-ms', '150', '--cpu-percent', '70', '--min-cte-samples', '20'])
    th = analyze.thresholds_from_args(args)
    assert th.pose == {segments.STOP: 0.01, segments.STRAIGHT: 0.02, segments.TURN: 0.03}
    assert th.cte == {segments.STRAIGHT: 0.04, segments.CURVE: 0.05}
    assert (th.response_mean_ms, th.cpu_mean_percent) == (150.0, 70.0)
    assert (th.pose_min_samples, th.cte_min_samples, th.response_min_samples,
            th.cpu_min_samples) == (100, 20, 50, 60)
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
    assert analyze._fmt(0.5, '') == '0.5'
    assert analyze._fmt(1.0, 'ratio') == '1.00'


def test_plots_written(tmp_path):
    pytest.importorskip('matplotlib')
    make_run(tmp_path, n=210, suffix='_amr_01')
    run = make_run(tmp_path, n=210, suffix='_amr_02', seed=3)
    report = analyze.analyze_run(run, plots=True)
    names = {p.name for p in report.plots}
    assert names == {'pose_error.png', 'trajectory.png', 'cte.png', 'response_time.png',
                     'cpu.png'}
    assert all(p.stat().st_size > 1000 for p in report.plots)
    assert 'pose_error.png' in report.to_markdown()
