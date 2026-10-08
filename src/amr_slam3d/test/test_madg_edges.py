"""madg_lockstep 오라클 토픽의 짝 규칙 시험 — bag 끝 묶음만 짝 없음을 허용한다 (b2-rec s3_aisle_a1)."""

import importlib.util
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'madg_lockstep.py'


def _mod():
    spec = importlib.util.spec_from_file_location('madg_lockstep', SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_all_paired_and_edge_runs_are_allowed():
    m = _mod()
    assert m.edge_unpaired([True] * 5) == (0, 0)
    assert m.edge_unpaired([False, True, True, True]) == (1, 0)
    assert m.edge_unpaired([False, False, True, True, False]) == (2, 1)
    assert m.edge_unpaired([]) == (0, 0)


def test_unpaired_in_the_middle_is_an_error():
    m = _mod()
    with pytest.raises(RuntimeError, match='가운데'):
        m.edge_unpaired([True, True, False, True, True])


def test_edge_run_longer_than_limit_is_an_error():
    m = _mod()
    n = m.EDGE_UNPAIRED_MAX
    assert m.edge_unpaired([False] * n + [True]) == (n, 0)
    with pytest.raises(RuntimeError, match='너무 많다'):
        m.edge_unpaired([False] * (n + 1) + [True])
    with pytest.raises(RuntimeError, match='너무 많다'):
        m.edge_unpaired([True] + [False] * (n + 1))
