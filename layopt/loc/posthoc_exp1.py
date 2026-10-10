"""
실험 1 사후 탐색 분석 (사전 등록 외 — 결과를 본 뒤 정한 질문, 판정에 쓰지 않는다).

    python3 -m layopt.loc.posthoc_exp1 --analysis <analysis>/exp1_<run_id>

질문:
  1. 자세 종류 차이(대기 슬롯 대 도크)만으로 AUROC 가 높게 나온 것은 아닌가 → 자유 공간 안에서만 AUROC.
  2. k 가 마진과 따로 오수렴을 늘리는가 → 같은 마진 구간 안 k 별 비율, 로지스틱(마진 + k) 우도비.
  3. TRACKING 을 선언했는데 틀린 곳인 경우("확신한 오답") 는 몇 건이고 어디인가.
  4. 실스캔 여백(scan_filtered 에 같은 운용 코드)과 E0 가상 여백의 일치, 실스캔 여백의 AUROC.
"""

import argparse
import csv
import json
from pathlib import Path
import sys

from layopt.loc import analyze_exp1 as ax
import numpy as np


def _f(v):
    return None if v in (None, '', 'None') else float(v)


def logistic2(X: np.ndarray, y: np.ndarray, iters: int = 100):
    """다변수 로지스틱 MLE (뉴턴법). 반환 (계수, 로그우도)."""
    X = np.column_stack([np.ones(len(y)), X])
    b = np.zeros(X.shape[1])
    for _ in range(iters):
        p = 1.0 / (1.0 + np.exp(-np.clip(X @ b, -40, 40)))
        H = (X * (p * (1 - p))[:, None]).T @ X + 1e-9 * np.eye(X.shape[1])
        step = np.linalg.solve(H, X.T @ (y - p))
        b += step
        if np.max(np.abs(step)) < 1e-8:
            break
    p = np.clip(1.0 / (1.0 + np.exp(-np.clip(X @ b, -40, 40))), 1e-12, 1 - 1e-12)
    return b, float(np.sum(y * np.log(p) + (1 - y) * np.log(1 - p)))


def main(argv=None) -> int:
    """명령줄."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--analysis', required=True)
    a = ap.parse_args(argv)
    d = Path(a.analysis)
    rows = list(csv.DictReader((d / 'tidy.csv').open(encoding='utf-8')))
    for r in rows:
        r['m'] = _f(r['e0_margin'])
        r['y'] = int(r['y_mis'])
        r['k'] = int(r['k'])
    out = {'note': '사전 등록 외 사후 탐색 — 판정(게이트 A)에 쓰지 않는다'}
    # 1. 자세 종류 안 AUROC
    out['auroc_within_kind'] = {}
    for kind in sorted({r['kind'] for r in rows}):
        sub = [r for r in rows if r['kind'] == kind]
        s = -np.array([r['m'] for r in sub])
        y = np.array([r['y'] for r in sub])
        g = [r['pose_id'] for r in sub]
        auc = ax.auroc(s, y)
        vals, dropped = ax.cluster_boot(g, lambda idx: ax.auroc(s[idx], y[idx]))
        out['auroc_within_kind'][kind] = {
            'n': len(sub), 'events': int(y.sum()), 'n_pose': len(set(g)),
            'auroc': None if auc is None else round(auc, 4),
            'ci95': [round(float(np.percentile(vals, 2.5)), 4),
                     round(float(np.percentile(vals, 97.5)), 4)] if vals else None,
            'boot_dropped_one_class': dropped}
    # 2. k 효과: 마진 구간 × k 비율, 로지스틱 마진 vs 마진 + k
    bins = [-1.0, 0.03, 0.05, 0.08, 1.0]
    tab = []
    for lo, hi in zip(bins[:-1], bins[1:]):
        for k in (0, 2, 4):
            sub = [r for r in rows if lo <= r['m'] < hi and r['k'] == k]
            ev = sum(r['y'] for r in sub)
            tab.append({'margin_bin': [lo, hi], 'k': k, 'n': len(sub), 'events': ev,
                        'rate': round(ev / len(sub), 3) if sub else None})
    out['mis_by_margin_bin_k'] = tab
    m = np.array([r['m'] for r in rows])
    k = np.array([r['k'] for r in rows], dtype=float)
    y = np.array([r['y'] for r in rows], dtype=float)
    b1, ll1 = logistic2(m[:, None], y)
    b2, ll2 = logistic2(np.column_stack([m, k]), y)
    from scipy.stats import chi2
    lr = 2 * (ll2 - ll1)
    out['logit_margin_plus_k'] = {
        'margin_only': {'b': [round(v, 4) for v in b1], 'loglik': round(ll1, 3)},
        'margin_k': {'b': [round(v, 4) for v in b2], 'loglik': round(ll2, 3)},
        'lr_stat': round(lr, 3), 'lr_p_naive': float(chi2.sf(lr, 1)),
        'note': 'p 는 자세 군집을 무시한 순진한 값 (참고만)'}
    out['margin_at_p50'] = round(-b1[0] / b1[1], 4)
    # 3. 확신한 오답 (TRACKING 선언 + 오차 > 1 m)
    cw = [r for r in rows if r['declared'] == '1' and r['outcome'] == 'misconverge']
    out['declared_but_wrong'] = [{'trial_id': r['trial_id'], 'kind': r['kind'],
                                  'e0_margin': r['m'], 'err_m': _f(r['err_m'])} for r in cw]
    out['failed_wrong'] = sum(1 for r in rows if r['declared'] == '0'
                              and r['outcome'] == 'misconverge')
    out['declared_correct'] = sum(1 for r in rows if r['declared'] == '1'
                                  and r['outcome'] == 'correct')
    out['undeclared_correct'] = sum(1 for r in rows if r['declared'] == '0'
                                    and r['outcome'] == 'correct')
    # 4. 실스캔 여백
    rs_path = d / 'realscan_margin_posthoc.csv'
    if rs_path.exists():
        rs = {r['trial_id']: r for r in csv.DictReader(rs_path.open(encoding='utf-8'))}
        pairs = [(r['m'], _f(rs[r['trial_id']].get('real_margin_lost')), r['y'], r['pose_id'])
                 for r in rows if r['trial_id'] in rs
                 and _f(rs[r['trial_id']].get('real_margin_lost')) is not None]
        e0m = np.array([p[0] for p in pairs])
        rm = np.array([p[1] for p in pairs])
        yy = np.array([p[2] for p in pairs])
        g = [p[3] for p in pairs]
        auc = ax.auroc(-rm, yy)
        vals, _ = ax.cluster_boot(g, lambda idx: ax.auroc(-rm[idx], yy[idx]))
        out['realscan'] = {
            'n': len(pairs), 'pearson_e0_vs_real': round(float(np.corrcoef(e0m, rm)[0, 1]), 4),
            'mean_abs_diff': round(float(np.mean(np.abs(e0m - rm))), 4),
            'median_diff_real_minus_e0': round(float(np.median(rm - e0m)), 4),
            'auroc_real_margin': None if auc is None else round(auc, 4),
            'auroc_real_ci95': [round(float(np.percentile(vals, 2.5)), 4),
                                round(float(np.percentile(vals, 97.5)), 4)] if vals else None,
            'note': 'real = LOST 시각 scan_filtered 에 운용 global_seed (참 자세 기준)'}
    res = d / 'posthoc_exploratory.json'
    if res.exists():
        print(f'이미 있다: {res}', file=sys.stderr)
        return 3
    res.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding='utf-8')
    print(json.dumps(out, ensure_ascii=False, indent=1)[:4000])
    return 0


if __name__ == '__main__':
    sys.exit(main())
