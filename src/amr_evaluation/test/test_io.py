from datetime import datetime
from pathlib import Path

from amr_evaluation import io
import pytest


def test_default_log_root(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_WS', str(tmp_path))
    assert io.default_log_root() == tmp_path / 'logs' / 'eval'
    monkeypatch.delenv('ROS_WS')
    assert io.default_log_root() == Path.cwd() / 'logs' / 'eval'


def test_default_run_name():
    assert io.default_run_name(datetime(2026, 9, 22, 3, 4, 5)) == 'run_20260922_030405'
    assert io.default_run_name().startswith('run_')


def test_resolve_run_dir(monkeypatch, tmp_path):
    d = io.resolve_run_dir(str(tmp_path), 'r1')
    assert d == tmp_path / 'r1' and d.is_dir()
    monkeypatch.setenv('ROS_WS', str(tmp_path / 'ws'))
    d2 = io.resolve_run_dir('', '', create=False)
    assert d2.parent == tmp_path / 'ws' / 'logs' / 'eval'
    assert d2.name.startswith('run_') and not d2.exists()


def test_csv_round_trip(tmp_path):
    path = tmp_path / 'sub' / 'pose_error.csv'
    with io.CsvWriter(path, io.POSE_COLUMNS) as w:
        w.write([1.5, 0.0, 0.0, 0.01, 0.0, 0.01, -0.001, '직선'])
        w.write([1.52, 1.0, 2.0, 1.0, 2.0, 0.0, 0.0, '정지'])
        assert w.rows == 2
    rows = io.read_csv(path)
    assert len(rows) == 2
    assert list(rows[0]) == io.POSE_COLUMNS
    assert rows[0]['error'] == pytest.approx(0.01)
    assert rows[0]['segment'] == '직선'
    assert io.column(rows, 'gt_x') == [0.0, 1.0]
    header = path.read_text(encoding='utf-8').splitlines()[0]
    assert header == 'timestamp,gt_x,gt_y,est_x,est_y,error,yaw_error,segment'


def test_csv_writer_rejects_wrong_width_and_formats_bool(tmp_path):
    w = io.CsvWriter(tmp_path / 'x.csv', ['a', 'b'])
    with pytest.raises(ValueError):
        w.write([1.0])
    w.write([True, 'txt'])
    w.close()
    w.close()   # 두 번 닫아도 무해
    rows = io.read_csv(tmp_path / 'x.csv')
    assert rows == [{'a': 1.0, 'b': 'txt'}]


def test_write_text(tmp_path):
    p = io.write_text(tmp_path / 'a' / 'report.md', '# hi\n')
    assert p.read_text(encoding='utf-8') == '# hi\n'
