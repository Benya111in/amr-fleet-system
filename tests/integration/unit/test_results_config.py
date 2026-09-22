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


def test_latency_gate(monkeypatch):
    from amr_itest import cases
    summary = {'count': 20, 'mean': 20.0, 'p95': 60.0, 'max': 130.0}
    monkeypatch.setattr(results, 'host_load', lambda: {'provisional_under_load': False})
    assert cases.latency_gate(summary, 100.0) == ('max', 130.0, False)
    monkeypatch.setattr(results, 'host_load', lambda: {'provisional_under_load': True})
    assert cases.latency_gate(summary, 100.0) == ('p95 (provisional_under_load)', 60.0, True)
    assert not cases.latency_gate({'count': 0, 'p95': float('nan'), 'max': float('nan')},
                                  100.0)[2]


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
            req.file(Path(__file__)), req.file(Path('/nonexistent'), 'f'), req.gpu()]
    missing = req.missing(reqs)
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
    implemented = {s.number for s in catalog.SCENARIOS if s.implemented}
    assert implemented == {1, 2, 4, 9, 13}
    assert all(Path(__file__).parents[1].joinpath(f'test_{s.id}.py').is_file()
               for s in catalog.SCENARIOS)
    table = catalog.table_markdown()
    assert table.count('\n') == 16 and '장시간' in table
    meta = catalog.get(4).meta()
    assert meta['id'] == '04_ekf_accuracy' and meta['backends'] == [KINEMATIC, GAZEBO]


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


def test_report_collect_write_and_exit(tmp_path, capsys):
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
    assert report.exit_code(ok) == 0 and report.exit_code(ok, fail_on_skip=True) == 1
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
