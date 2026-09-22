"""results · config · requirements · catalog · report."""

import json
from pathlib import Path
import unittest

from amr_itest import catalog, config, junit, report, requirements as req, results
from amr_itest.scenario import Context, GAZEBO, KINEMATIC
import pytest


def test_record_lifecycle(tmp_path):
    d = tmp_path / 'x'
    d.mkdir()
    (d / 'old.csv').write_text('stale')
    (d / 'sub').mkdir()
    (d / 'launch.log').write_text('keep me')
    rec = results.ScenarioRecord({'id': 'x'}, d, {'robot': 'amr_01'}).begin()
    assert not (d / 'old.csv').exists() and not (d / 'sub').exists()
    assert (d / 'launch.log').read_text() == 'keep me'
    rec.set_backend('kinematic')
    rec.set_components({'safety': 'standin'})
    rec.measure('m', {'a': float('nan'), 'b': [1, 2.5, float('inf')]})
    assert rec.check('ok', 1.0, 2.0, True, 'm') is True
    assert rec.check('bad', 3.0, 2.0, False) is False
    rec.note('hello')
    rec.write_csv('t.csv', ['a', 'b'], [[1, 0.1234567], ['x', 2.0]])
    rec.write_text('t.txt', 'txt')
    rec.finish()
    data = results.load(rec.path)
    assert data['state'] == 'finished' and data['backend'] == 'kinematic'
    assert data['measurements']['m'] == {'a': 'nan', 'b': [1, 2.5, 'inf']}
    assert [c['name'] for c in rec.failed_checks()] == ['bad']
    assert (d / 't.csv').read_text().splitlines()[1] == '1,0.123457'
    assert results.load(tmp_path / 'none.json') == {}
    (tmp_path / 'broken.json').write_text('{')
    assert results.load(tmp_path / 'broken.json') == {}
    rec2 = results.ScenarioRecord({'id': 'y'}, tmp_path / 'y').begin()
    rec2.skipped('no gpu')
    assert results.load(rec2.path)['skip_reason'] == 'no gpu'
    assert results.jsonable(type('N', (), {'item': lambda self: 3})()) == 3
    assert results.jsonable(object()).startswith('<object')
    load = results.host_load()
    assert load['cpus'] >= 1 and len(load['loadavg']) == 3


def test_latency_gate_always_max(monkeypatch):
    """부하 중에도 최댓값으로 판정한다 (예전: p95 로 바꿔 20회 중 1회 2 s 정지도 통과)."""
    import numpy as np
    from amr_itest import cases, metrics
    one_bad = metrics.latency_summary([0.3] * 19 + [1990.0])
    assert np.percentile([0.3] * 19 + [1990.0], 95) < 100.0        # p95 였다면 통과
    for loaded in (False, True):
        monkeypatch.setattr(results, 'host_load', lambda loaded=loaded: {
            'provisional_under_load': loaded})
        assert cases.latency_gate(one_bad, 100.0) == ('max', 1990.0, False)
    assert not cases.latency_gate({'count': 0, 'p95': float('nan'), 'max': float('nan')},
                                  100.0)[2]
    assert cases.latency_gate(metrics.latency_summary([-0.05, 5.0]), 20.0)[2]   # 음수 그대로


def test_retry_under_load():
    """실패 + 부하일 때만 한 번 다시 재고, 두 시도를 모두 돌려준다."""
    from amr_itest import cases
    runs = iter([{'count': 1, 'max': 150.0, 'mean': 150.0}, {'count': 1, 'max': 5.0, 'mean': 5.0}])
    summary, ok, attempts = cases.retry_under_load(lambda: next(runs), 20.0, lambda: True)
    assert ok and summary['max'] == 5.0 and len(attempts) == 2
    runs = iter([{'count': 1, 'max': 150.0, 'mean': 150.0}])
    summary, ok, attempts = cases.retry_under_load(lambda: next(runs), 20.0, lambda: False)
    assert not ok and len(attempts) == 1                            # 부하 없으면 재측정 없이 실패
    runs = iter([{'count': 1, 'max': 150.0, 'mean': 1.0}, {'count': 1, 'max': 160.0, 'mean': 1.0}])
    summary, ok, attempts = cases.retry_under_load(lambda: next(runs), 20.0, lambda: True)
    assert not ok and summary['max'] == 160.0


def test_settings_from_env(itest_env, monkeypatch):
    s = config.Settings.from_env()
    assert s.robot == 'amr_01' and s.namespace == '/amr_01' and s.log_root == itest_env
    assert s.frame('odom') == 'odom' and s.timeout(10.0) == 10.0
    monkeypatch.setenv('ITEST_ROBOT', '/amr_03/')
    monkeypatch.setenv('ITEST_FRAME_PREFIX', 'amr_03/')
    monkeypatch.setenv('ITEST_TIMEOUT_SCALE', '2.5')
    monkeypatch.setenv('ITEST_SIM', 'kinematic')
    s = config.Settings.from_env()
    assert s.robot == 'amr_03' and s.frame('odom') == 'amr_03/odom' and s.frame('map') == 'map'
    assert s.timeout(2.0) == 5.0 and s.sim == 'kinematic'
    for var, bad in (('ITEST_SIM', 'isaac'), ('ITEST_PROFILE', 'x'), ('ITEST_STANDINS', 'x'),
                     ('ITEST_TIMEOUT_SCALE', '-1')):
        monkeypatch.setenv(var, bad)
        with pytest.raises(ValueError):
            config.Settings.from_env()
        monkeypatch.delenv(var)


def test_config_loading(tmp_path, monkeypatch):
    assert (config.config_dir() / 'robot_params.yaml').is_file()
    assert config.get(config.robot_params(), 'limits.max_linear_velocity') == 2.0
    assert config.get(config.sensors(), 'lidar.missing.key', 'd') == 'd'
    with pytest.raises(KeyError):          # 기본값 없이 읽는 키는 필수 (옛 기본값으로 조용히 재지 않는다)
        config.get(config.sensors(), 'lidar.missing.key')
    # LiDAR 평면 = base_link_height + lidar.extrinsic.z (차체 안 0.20 m, 예전 기본값 0.38 m 가 아니다)
    assert config.scan_plane_height() == pytest.approx(
        config.robot_params()['robot']['base_link_height']
        + config.sensors()['lidar']['extrinsic']['z'])
    assert config.scan_plane_height() < 0.3
    assert config.ekf_params('ekf_filter_node_odom')['world_frame'] == 'odom'
    y = tmp_path / 'p.yaml'
    y.write_text('node_a:\n  ros__parameters:\n    k: 1\n')
    assert config.load_ros_params(y) == {'k': 1}
    with pytest.raises(KeyError):
        config.load_ros_params(y, 'node_b')
    (tmp_path / 'config').mkdir()
    monkeypatch.setenv('ITEST_REPO_ROOT', str(tmp_path))
    assert config.repo_root() == tmp_path


def test_requirements():
    assert req.has_package('rclpy') and not req.has_package('no_such_pkg_xyz')
    assert req.executable_path('no_such_pkg_xyz', 'x') is None
    assert req.launch_file('no_such_pkg_xyz', 'a.launch.py') is None
    assert req.config_file('no_such_pkg_xyz', 'a.yaml') is None
    assert req.has_python_module('json') and not req.has_python_module('no_such_mod_xyz')
    reqs = [req.package('no_such_pkg_xyz', 'why'), req.executable('rclpy', 'nope'),
            req.launch('rclpy', 'nope.launch.py'), req.module('json'),
            req.file(Path(__file__)), req.file(Path('/nonexistent'), 'f'), req.gpu(),
            req.config('rclpy', 'nope.yaml')]
    missing = req.missing(reqs)
    assert any(m.startswith('config file rclpy/config/nope.yaml') for m in missing)
    assert 'package no_such_pkg_xyz not found (why)' in missing
    assert any(m.startswith('executable rclpy/nope') for m in missing)
    assert any(m.startswith('launch file rclpy/launch/nope.launch.py') for m in missing)
    assert any(m.startswith('file /nonexistent') for m in missing)
    assert isinstance(req.gpu_device_present(), bool)
    with pytest.raises(ValueError):
        req.Requirement('bogus').missing()


def test_catalog():
    assert len(catalog.SCENARIOS) == 14
    assert [s.number for s in catalog.SCENARIOS] == list(range(1, 15))
    assert catalog.get(9).id == '09_emergency_stop'
    for key in ('9', '09_emergency_stop', 'emergency_stop', 'test_09_emergency_stop.py'):
        assert catalog.by_id(key).number == 9
    with pytest.raises(KeyError):
        catalog.by_id('nope')
    # 모든 패키지가 머지됐다: 전부 구현 (skip 은 needs 가 설치돼 있으면 실패로 센다)
    assert all(s.implemented for s in catalog.SCENARIOS)
    assert all(s.needs for s in catalog.SCENARIOS)
    workspace = {p.name for p in (Path(__file__).parents[3] / 'src').iterdir() if p.is_dir()}
    for s in catalog.SCENARIOS:
        assert set(s.needs) <= workspace, (s.id, set(s.needs) - workspace)
    assert all(Path(__file__).parents[1].joinpath(f'test_{s.id}.py').is_file()
               for s in catalog.SCENARIOS)
    table = catalog.table_markdown()
    assert table.count('\n') == 16 and '장시간' in table
    # README 표는 catalog 에서 생성한다 — 손으로 고친 표가 판정 기준과 어긋나지 않게
    readme = (Path(__file__).parents[1] / 'README.md').read_text(encoding='utf-8')
    stale = [row[:40] for row in table.splitlines() if row.startswith('|') and row not in readme]
    assert not stale, f'README 표가 catalog 와 다름 (table_markdown 으로 재생성): {stale}'
    meta = catalog.get(4).meta()
    assert meta['id'] == '04_ekf_accuracy' and meta['backends'] == [GAZEBO, KINEMATIC]
    # 명세 판정 구성이 먼저 (auto): 04 실제 AMCL, 13 실제 실행기 체인
    assert catalog.get(4).profiles[0] == 'system' and catalog.get(13).profiles[0] == 'system'
    assert 'amr_fleet' in catalog.get(12).needs


def test_context_backend_selection(itest_env, monkeypatch):
    ctx = Context(catalog.get(9)).begin()
    assert ctx.select_backend() == KINEMATIC and not ctx.use_sim_time
    assert ctx.select_profile() == 'component'
    assert ctx.path('a.csv').parent == ctx.log_dir and ctx.timeout(3.0) == 3.0
    monkeypatch.setenv('ITEST_SIM', 'gazebo')
    ctx2 = Context(catalog.get(13))
    monkeypatch.setattr(req, 'gpu_device_present', lambda: False)
    with pytest.raises(unittest.SkipTest) as exc:
        ctx2.select_backend()
    assert 'GPU' in str(exc.value) and ctx2.skip_reason
    monkeypatch.setenv('ITEST_SIM', 'kinematic')
    with pytest.raises(unittest.SkipTest):
        Context(catalog.get(1)).select_backend()        # 01 은 gazebo 전용
    monkeypatch.setenv('ITEST_SIM', 'auto')
    with pytest.raises(unittest.SkipTest) as exc:
        Context(catalog.get(1)).select_backend()
    assert 'no usable backend' in str(exc.value)
    monkeypatch.setenv('ITEST_PROFILE', 'system')
    with pytest.raises(unittest.SkipTest):
        Context(catalog.get(9)).select_profile()
    # system 구성은 Gazebo 만: GPU 가 없으면 운동학으로 조용히 바뀌지 않고 건너뛴다
    monkeypatch.setenv('ITEST_PROFILE', 'auto')
    ctx4 = Context(catalog.get(4))
    with pytest.raises(unittest.SkipTest) as exc:
        ctx4.select()
    assert ctx4.profile == 'system' and 'gazebo' in str(exc.value)
    monkeypatch.setenv('ITEST_SIM', 'kinematic')
    with pytest.raises(unittest.SkipTest) as exc:
        Context(catalog.get(13)).select()
    assert "backend 'kinematic' not supported" in str(exc.value)
    monkeypatch.setenv('ITEST_PROFILE', 'component')
    assert Context(catalog.get(13)).select() == (KINEMATIC, 'component')
    ctx3 = Context(catalog.get(3))
    with pytest.raises(unittest.SkipTest):
        ctx3.require([req.package('no_such_pkg_xyz')], 'slam')
    assert ctx3.skip_reason.startswith('slam: package no_such_pkg_xyz')
    ctx3.require([])
    assert ctx3.backend_missing(KINEMATIC) == []


def _scenario_dir(root, sid, status, checks=(), skip=''):
    d = root / sid
    d.mkdir(parents=True)
    junit.synthetic(d / 'junit.xml', sid, 'case', status, skip or 'msg', 1.0)
    data = {'backend': 'kinematic', 'checks': list(checks), 'duration_s': 12.0}
    if skip:
        data['skip_reason'] = skip
    (d / results.RESULT_FILE).write_text(json.dumps(data))


def test_report_skip_policy(tmp_path, monkeypatch, capsys):
    """필요 패키지(needs)가 설치된 구현 시나리오의 skip·partial 은 실패다 (GPU 없음도 통과가 아니다)."""
    installed = {'amr_simulation', 'amr_description'}
    monkeypatch.setattr(req, 'has_package', lambda p: p in installed)
    _scenario_dir(tmp_path, '01_sensor_topics', junit.SKIPPED, skip='GPU device not found')
    _scenario_dir(tmp_path, '03_slam_map', junit.SKIPPED, skip='package amr_bringup not found')
    d = tmp_path / '09_emergency_stop'
    d.mkdir()
    (d / 'junit.xml').write_text(
        '<testsuites><testsuite name="s"><testcase name="a"/>'
        '<testcase name="b"><skipped message="gazebo only"/></testcase></testsuite></testsuites>')
    summary = report.collect(tmp_path)
    by = {e['id']: e for e in summary['scenarios']}
    assert by['09_emergency_stop']['status'] == junit.PARTIAL
    assert by['09_emergency_stop']['cases'] == {'total': 2, 'executed': 1, 'passed': 1,
                                                'failed': 0, 'error': 0, 'skipped': 1}
    assert by['01_sensor_topics']['missing_needs'] == []
    assert by['03_slam_map']['missing_needs'] == ['amr_bringup', 'amr_localization']
    assert report.skip_is_failure(by['01_sensor_topics'])          # 설치됐는데 skip
    assert not report.skip_is_failure(by['03_slam_map'])           # 패키지가 없어서 skip
    assert report.skip_is_failure(by['09_emergency_stop']) is False  # needs 일부 미설치
    installed.update({'amr_localization', 'amr_navigation', 'amr_perception'})
    assert report.skip_is_failure(report.collect(tmp_path, ['9'])['scenarios'][0])
    assert report.exit_code(summary) == 1
    assert report.exit_code(summary, report.FAIL_ON_SKIP_NONE) == 0
    only_missing = report.collect(tmp_path, ['3'])
    assert report.exit_code(only_missing) == 0
    assert report.exit_code(only_missing, report.FAIL_ON_SKIP_ALL) == 1
    line = report.counts_line(summary)
    assert '실패로 센 skip 1' in line and '테스트 케이스 4: 실행 1, skip 3' in line
    assert report.main(['--log-dir', str(tmp_path), '--scenarios', '3']) == 0
    assert report.main(['--log-dir', str(tmp_path), '--scenarios', '1']) == 1
    assert 'skipped (실패로 셈)' in (tmp_path / report.SUMMARY_MD).read_text()
    assert report.main(['--log-dir', str(tmp_path), '--scenarios', '1', '--allow-skip']) == 0
    assert '실패로 셈)' not in (tmp_path / report.SUMMARY_MD).read_text()
    assert report.main(['--log-dir', str(tmp_path), '--fail-on-skip', '--allow-skip']) == 2
    capsys.readouterr()


def test_report_collect_write_and_exit(tmp_path, capsys, monkeypatch):
    monkeypatch.setattr(req, 'has_package', lambda p: False)       # needs 미설치 → skip 은 skip
    _scenario_dir(tmp_path, '09_emergency_stop', junit.PASSED,
                  [{'name': 'latency', 'value': 5.0, 'passed': True}])
    _scenario_dir(tmp_path, '03_slam_map', junit.SKIPPED, skip='package x not found')
    _scenario_dir(tmp_path, '04_ekf_accuracy', junit.FAILED,
                  [{'name': 'rmse', 'value': 0.1, 'passed': False}])
    (tmp_path / 'unit').mkdir()
    junit.synthetic(tmp_path / 'unit' / 'junit.xml', 'unit', 'u', junit.PASSED)
    summary = report.collect(tmp_path)
    assert [e['id'] for e in summary['scenarios']] == ['03_slam_map', '04_ekf_accuracy',
                                                       '09_emergency_stop']
    assert summary['counts'] == {'skipped': 1, 'failed': 1, 'passed': 1}
    totals = report.write(summary, tmp_path)
    assert totals['tests'] == 4 and totals['failures'] == 1
    md = (tmp_path / report.SUMMARY_MD).read_text()
    assert 'FAIL rmse=0.1' in md and 'package x not found' in md
    assert report.exit_code(summary) == 1
    ok = report.collect(tmp_path, ['9', '3'])
    assert report.exit_code(ok) == 0 and report.exit_code(ok, report.FAIL_ON_SKIP_ALL) == 1
    assert report.collect(tmp_path, [])['scenarios'] == []
    assert report.main(['--log-dir', str(tmp_path), '--scenarios', '9']) == 0
    assert report.main(['--list']) == 0 and '14_soak' in capsys.readouterr().out
    assert report.main(['--readme-table']) == 0
    assert report.main(['--resolve', '4', 'soak']) == 0
    assert capsys.readouterr().out.split()[-2:] == ['04_ekf_accuracy', '14_soak']
    assert report.main(['--resolve', 'nope']) == 2
    assert report.main(['--default-set']) == 0 and '14_soak' not in capsys.readouterr().out
    assert report.main(['--log-dir', str(tmp_path), '--synthetic', '13_response_time', 'error',
                        'timeout']) == 0
    assert junit.status_of(tmp_path / '13_response_time' / 'junit.xml') == junit.ERROR
    long = report._short({'status': 'passed', 'failed_checks': [],
                          'checks': [{'name': 'n' * 300, 'value': 1}]})
    assert len(long) == 160 and long.endswith('…')
