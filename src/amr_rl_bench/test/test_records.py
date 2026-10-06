"""amr_rl_bench.records 시험 — gzbench/v1 열과 episode/v1 투영 (rl/eval 스키마로 검사)."""

import csv
import os
import sys

from amr_rl_bench import records as R
import pytest

from conftest import REPO


def _schema():
    path = os.path.join(REPO, 'rl', 'eval')
    if not os.path.isdir(os.path.join(path, 'amr_rl_eval')):
        pytest.skip('rl/eval 없음')
    sys.path.insert(0, path)
    try:
        from amr_rl_eval import schema
    finally:
        sys.path.remove(path)
    return schema


def meta(**kw):
    """실행 정보 (시나리오 하나)."""
    m = {'run_tag': 't', 'exec_index': 3, 'scenario_id': 'gz_static_v1_m001',
         'scenario_set': 'gz_static_v1_main', 'difficulty': 'main', 'scenario_seed': 1,
         'controller': 'rl', 'arm': 'B_ppo', 'policy_id': 'v12_s1', 'commit': 'abc1234',
         'setup_ok': True}
    m.update(kw)
    return m


@pytest.mark.parametrize('outcome,contacts', [('success', 0), ('collision', 1), ('stall', 0),
                                              ('timeout', 0)])
def test_episode_v1_projection_validates(outcome, contacts):
    """계약 결과마다 episode/v1 행이 rl/eval 검사를 통과한다."""
    schema = _schema()
    g = R.gzbench_row(meta(), {'outcome_contract': outcome, 'contacts_contract': contacts,
                               't_contract_s': 12.5, 'path_length_m': 10.0, 'interventions': 0,
                               'min_clear_static_m': 0.3, 'rtf': 0.9, 'src_dwa': 2})
    row, errs = schema.validate_row({k: R.fmt(v) for k, v in
                                     R.episode_v1_row(g, 0, 1).items()})
    assert errs == []
    assert row['outcome'] == outcome and row['simulator'] == 'gazebo'
    assert row['fallback_count'] == 2


def test_setup_failure_is_error_row():
    """준비 실패는 outcome error, 접촉 0 (분석에서 빼지 않고 센다)."""
    schema = _schema()
    g = R.gzbench_row(meta(setup_ok=False, setup_error='teleport'),
                      {'outcome_contract': 'error'})
    row, errs = schema.validate_row({k: R.fmt(v) for k, v in R.episode_v1_row(g).items()})
    assert errs == [] and row['outcome'] == 'error'


def test_gzbench_columns_and_csv(tmp_path):
    """넓은 행은 COLUMNS 를 다 갖고, CSV 는 머리말 한 번 + 행."""
    g = R.gzbench_row(meta(), {'outcome_contract': 'success', 'strict_gt': True,
                               'final_dist_gt_m': float('nan'), 'unknown_key': 1})
    assert list(g.keys()) == R.COLUMNS
    assert g['schema'] == R.SCHEMA and 'unknown_key' not in g
    p = tmp_path / 'g.csv'
    R.append_csv(str(p), R.COLUMNS, g)
    R.append_csv(str(p), R.COLUMNS, g)
    rows = list(csv.DictReader(open(p, encoding='utf-8')))
    assert len(rows) == 2 and rows[0]['strict_gt'] == '1' and rows[0]['final_dist_gt_m'] == ''
