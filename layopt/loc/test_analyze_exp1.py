"""
실험 1 분석 시험 — AUROC·로지스틱·군집 부트스트랩·게이트 A·합성 실행 디렉터리.

    cd /ros2_ws && python3 -m pytest -q layopt/loc/test_analyze_exp1.py
"""

import json
from pathlib import Path

from layopt.loc import analyze_exp1 as ax
import numpy as np
import pytest


def test_auroc_known_values():
    y = np.array([0, 0, 1, 1])
    assert ax.auroc(np.array([0.1, 0.2, 0.8, 0.9]), y) == 1.0
    assert ax.auroc(np.array([0.9, 0.8, 0.2, 0.1]), y) == 0.0
    assert ax.auroc(np.array([0.5, 0.5, 0.5, 0.5]), y) == 0.5
    # 손 계산: 쌍 (pos, neg) 4 개 중 3 개 맞음
    assert ax.auroc(np.array([0.1, 0.6, 0.5, 0.9]), y) == pytest.approx(0.75)
    assert ax.auroc(np.array([0.1, 0.2]), np.array([0, 0])) is None


def test_gate_a_thresholds():
    assert ax.gate_a(0.8).startswith('지지')
    assert ax.gate_a(0.7999).startswith('약한 지지')
    assert ax.gate_a(0.7).startswith('약한 지지')
    assert ax.gate_a(0.6999).startswith('기각')
    assert ax.gate_a(None).startswith('판정 불가')


def test_logistic_recovers_slope():
    rng = np.random.default_rng(0)
    x = rng.uniform(-2, 2, 4000)
    y = (rng.uniform(size=x.size) < 1 / (1 + np.exp(-(0.5 - 2.0 * x)))).astype(float)
    fit = ax.logistic(x, y)
    assert fit['b1'] == pytest.approx(-2.0, abs=0.2)
    assert fit['b0'] == pytest.approx(0.5, abs=0.15)
    assert ax.logistic(np.ones(5), np.array([0, 1, 0, 1, 0.]))['note'] == 'constant predictor'
    assert ax.logistic(x[:5], np.zeros(5))['note'] == 'one class only'
    sep = ax.logistic(np.array([0., 1, 2, 3]), np.array([0., 0, 1, 1]))
    assert 'separation' in sep['note']


def test_cluster_boot_resamples_whole_groups():
    groups = ['a', 'a', 'b', 'b', 'c']
    seen = []
    vals, dropped = ax.cluster_boot(groups, lambda idx: seen.append(sorted(idx)) or len(idx),
                                    n=50, seed=1)
    assert dropped == 0 and len(vals) == 50
    for idx in seen:          # a·b 는 항상 둘씩 함께
        assert idx.count(0) == idx.count(1) and idx.count(2) == idx.count(3)


def test_wilson():
    lo, hi = ax.wilson(0, 10)
    assert lo == 0.0 and 0.25 < hi < 0.35
    assert ax.wilson(0, 0) == [None, None]


def _fake_run(tmp: Path):
    trials, res = [], []
    rng = np.random.default_rng(3)
    for i in range(30):
        margin = float(rng.uniform(-0.05, 0.25))
        for k in (0, 2):
            tid = f'p{i}_k{k}'
            trials.append({'trial_id': tid, 'order': len(trials), 'pose_id': f'free:p{i}',
                           'kind': 'free', 'x': 0, 'y': 0, 'yaw': 0, 'k': k, 'k_used': k,
                           'neighbors': [], 'quintile': i % 5,
                           'e0': {'margin': margin, 'inlier': 1.0, 'inlier_raw': 0.9,
                                  'occluded_frac': 0.1 * k}})
            mis = margin < 0.03
            res.append({'trial_id': tid, 'status': 'done', 'natural': {'lost': k == 2 and i < 3},
                        'outcome': 'misconverge' if mis else 'correct',
                        'outcome_plus1s': 'correct', 'outcome_amcl': 'correct',
                        'err_m': 50.0 if mis else 0.05, 'declared': not mis,
                        'recovery_s': None if mis else 12.0, 'trigger': 'forced_reg_reject',
                        'rtf': 0.2 if i < 5 else 0.5,
                        'seeds_true': [True], 'wall_s': 100.0, 'load_start': [10, 9, 8],
                        'load_end': [11, 9, 8]})
    (tmp / 'trials.json').write_text(json.dumps({'trials': trials}))
    (tmp / 'trials.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in res))


def test_analyze_synthetic(tmp_path: Path):
    run = tmp_path / 'run'
    run.mkdir()
    _fake_run(run)
    s = ax.analyze(run, tmp_path / 'out')
    assert s['n_done'] == 60 and s['n_pose'] == 30
    assert s['auroc_primary']['auroc'] == 1.0           # 마진 < 0.03 이 정확히 오수렴
    assert s['auroc_sensitivity']['drop_rtf_below_0.25']['n'] == 50
    assert s['gate_A'].startswith('지지')
    assert s['natural_lost']['events'] == 3
    assert s['logit_lost_inlier']['note'] == 'constant predictor'
    assert (tmp_path / 'out' / 'tidy.csv').read_text().count('\n') == 61
    with pytest.raises(FileExistsError):
        ax.analyze(run, tmp_path / 'out')               # 출력 덮어쓰기 거부
