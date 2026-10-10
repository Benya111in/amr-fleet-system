"""
실험 4 E2 분석 시험 — 예측식·보정 판정·Holm·Fisher·합성 실행 디렉터리.

    cd /ros2_ws && python3 -m pytest -q layopt/loc/test_analyze_exp4.py
"""

import json
import math
from pathlib import Path

from layopt.loc import analyze_exp4 as a4
import numpy as np
import pytest


def test_p_pred_matches_exp1_fit():
    assert a4.p_pred(-a4.A / a4.B) == pytest.approx(0.5)
    assert a4.p_pred(0.0) == pytest.approx(1 / (1 + math.exp(-3.5147)))
    assert a4.p_pred(0.2) < 1e-5


def test_holm():
    assert a4.holm([0.01, 0.04, 0.03]) == pytest.approx([0.03, 0.06, 0.06])
    assert a4.holm([0.5, 0.9]) == pytest.approx([1.0, 1.0])


def test_calib_verdict_poisson_interval():
    rows = [{'y_mis': 1, 'p_pred': 0.5} for _ in range(10)] + \
           [{'y_mis': 0, 'p_pred': 0.5} for _ in range(10)]
    v = a4.calib_verdict(rows)
    assert v['expected'] == 10.0 and v['observed'] == 10 and v['verdict'] == '보정 유지'
    rows = [{'y_mis': 0, 'p_pred': 0.5} for _ in range(20)]
    assert a4.calib_verdict(rows)['verdict'].startswith('보정 깨짐 (과대')
    rows = [{'y_mis': 1, 'p_pred': 0.05} for _ in range(20)]
    assert a4.calib_verdict(rows)['verdict'].startswith('보정 깨짐 (과소')


def test_calib_bins_equal_frequency():
    rows = [{'y_mis': int(i % 3 == 0), 'p_pred': i / 100} for i in range(23)]
    b = a4.calib_bins(rows)
    assert len(b) == 5 and sum(x['n'] for x in b) == 23
    assert max(x['n'] for x in b) - min(x['n'] for x in b) <= 1
    assert all(b[i]['p_hi'] <= b[i + 1]['p_lo'] for i in range(4))


def _fake(tmp: Path, sid: str, wait_mis: int, seed: int):
    rng = np.random.default_rng(seed)
    trials, res = [], []
    for i in range(20):
        kind = 'waiting' if i < 5 else 'free'
        margin = 0.02 if kind == 'waiting' else float(rng.uniform(0.0, 0.25))
        for k in (0, 2, 4):
            tid = f'{sid.replace("-", "")}_p{i}_k{k}'
            trials.append({'trial_id': tid, 'order': len(trials), 'pose_id': f'p{i}', 'kind': kind,
                           'k': k, 'spec_id': sid, 'e0': {'margin': margin}})
            if kind == 'waiting':
                mis = (i * 3 + k // 2) < wait_mis
            else:
                mis = margin < 0.04
            res.append({'trial_id': tid, 'status': 'done',
                        'outcome': 'misconverge' if mis else 'correct',
                        'natural': {'lost': False}, 'declared': not mis})
    d = tmp / sid
    d.mkdir()
    (d / 'trials.json').write_text(json.dumps({'trials': trials}))
    (d / 'trials.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in res))
    return d


def test_analyze_synthetic(tmp_path: Path):
    d1 = _fake(tmp_path, 'v13-v20-v30-v40-v50', 2, 1)
    d2 = _fake(tmp_path, 'v11-v21-v31-v41-v51', 13, 2)
    s = a4.analyze([d1, d2], tmp_path / 'out')
    assert s['n_done'] == 120
    p13 = s['per_bundle']['v13-v20-v30-v40-v50']
    assert p13['mis_waiting'] == {'events': 2, 'n': 15, 'ci95': p13['mis_waiting']['ci95']}
    assert p13['fisher_waiting_vs_exp1']['p_two_sided'] < 0.001
    assert s['per_bundle']['v11-v21-v31-v41-v51']['fisher_waiting_vs_exp1']['p_two_sided'] == 1.0
    assert s['auroc_all']['n_pose'] == 40          # (묶음, 자세) 군집
    assert len(s['calibration_bins_all']) == 5
    with pytest.raises(FileExistsError):
        a4.analyze([d1], tmp_path / 'out')


def test_duplicate_trial_across_runs_rejected(tmp_path: Path):
    d1 = _fake(tmp_path, 'v13-v20-v30-v40-v50', 2, 1)
    with pytest.raises(ValueError):
        a4.rows_from([d1, d1])
