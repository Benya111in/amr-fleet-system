"""
실험 4 E2 분석 (사전 등록 — 시행 목록 exp4_trials.json 과 같은 커밋, 결과 전).

    python3 -m layopt.loc.analyze_exp4 --run-dir <e2 run> [--run-dir …] --out <analysis>/<dir>

목적 (design §6 실험 4, 조정자 지시): 실험 1 의 P_mis(마진) 곡선이 다른 배치에서도 성립하는가, 그리고 대기 슬롯
오수렴률이 실제로 떨어지는가.

정의 (실험 1 과 같다, analyze_exp1 의 함수를 쓴다):
  y_mis   = 판정 == misconverge (오차 > 1.0 m). ambiguous 는 주 분석에서 0, 민감도에서 뺀다. unknown 은 뺀다.
  p_pred  = σ(A + B · E0 margin), A = 3.5147, B = −78.0774 (실험 1 적합값, 고정 — 다시 적합하지 않는다).
산출:
  1. 묶음별 오수렴률: 대기 슬롯, 전체 (Wilson 95%). 자연 LOST 율.
  2. 보정 곡선: p_pred 등빈도 5 구간 (전체, 그리고 묶음별) — 구간마다 n, 평균 예측, 관측 비율, 기대·관측 수.
  3. 보정 판정 (묶음별·전체): 관측 오수렴 수 O 가 Poisson(E = Σ p_pred) 의 95% 구간
     [ppf(0.025), ppf(0.975)] 안이면 "보정 유지", 아래면 "보정 깨짐 (과대 예측)", 위면 "보정 깨짐 (과소 예측)".
  4. 전체 AUROC (−margin → y_mis), (묶음, 자세) 군집 부트스트랩 2,000 회, 시드 20261011. 묶음별 AUROC 는 참고.
  5. 대기 슬롯 오수렴률 Fisher 정확 검정: 묶음별 x/n 대 실험 1 기준 배치 13/15.
     주 = 양측 p, Holm 보정(묶음 3 개). 참고 = 단측 (새 묶음 비율이 더 낮다).
"""

import argparse
import csv
import json
import math
from pathlib import Path
import sys
from typing import Dict, List

from layopt.loc import analyze_exp1 as ax
import numpy as np

A, B = 3.5147, -78.0774          # 실험 1 로지스틱 (analysis/exp1_20261009T180550Z_exp1)
BASE_WAIT = (13, 15)             # 실험 1 기준 배치 대기 슬롯 오수렴 / 시행
SEED = 20261011
N_BINS = 5


def p_pred(margin: float) -> float:
    """실험 1 로지스틱 예측 오수렴 확률."""
    z = A + B * margin
    return 1.0 / (1.0 + math.exp(-max(min(z, 40.0), -40.0)))


def rows_from(run_dirs: List[Path]) -> List[Dict]:
    """실행 디렉터리들 → 시행당 한 행 (spec_id 포함)."""
    rows = []
    seen = set()
    for d in run_dirs:
        spec = json.loads((d / 'trials.json').read_text(encoding='utf-8'))
        by_id = {t['trial_id']: t for t in spec['trials']}
        done = {}
        for line in (d / 'trials.jsonl').read_text(encoding='utf-8').splitlines():
            if line.strip():
                r = json.loads(line)
                if r.get('status') == 'done':
                    done[r['trial_id']] = r
        for tid, r in done.items():
            if tid in seen:
                raise ValueError(f'같은 시행이 두 실행에 있다: {tid}')
            seen.add(tid)
            t = by_id[tid]
            m = t['e0']['margin']
            out = r.get('outcome')
            rows.append({
                'trial_id': tid, 'spec_id': t['spec_id'], 'pose_id': t['pose_id'],
                'cluster': f'{t["spec_id"]}|{t["pose_id"]}', 'kind': t['kind'], 'k': t['k'],
                'k_used': t.get('k_used', t['k']), 'e0_margin': m,
                'p_pred': None if m is None else round(p_pred(m), 6),
                'outcome': out, 'err_m': r.get('err_m'),
                'y_mis': None if out in (None, 'unknown') else int(out == 'misconverge'),
                'natural_lost': int(bool(r.get('natural', {}).get('lost'))),
                'declared': int(bool(r.get('declared'))), 'trigger': r.get('trigger'),
                'rtf': r.get('rtf'), 'run_dir': str(d)})
    rows.sort(key=lambda x: (x['spec_id'], x['trial_id']))
    return rows


def calib_bins(rows: List[Dict], n_bins: int = N_BINS) -> List[Dict]:
    """p_pred 등빈도 구간 (정렬 뒤 np.array_split, 안정 정렬)."""
    use = sorted([r for r in rows if r['y_mis'] is not None and r['p_pred'] is not None],
                 key=lambda r: r['p_pred'])
    out = []
    for i, part in enumerate(np.array_split(np.arange(len(use)), min(n_bins, max(len(use), 1)))):
        sub = [use[j] for j in part]
        if not sub:
            continue
        e = sum(r['p_pred'] for r in sub)
        o = sum(r['y_mis'] for r in sub)
        out.append({'bin': i, 'n': len(sub), 'p_lo': sub[0]['p_pred'], 'p_hi': sub[-1]['p_pred'],
                    'mean_pred': round(e / len(sub), 4), 'obs_rate': round(o / len(sub), 4),
                    'expected': round(e, 3), 'observed': o, 'obs_ci95': ax.wilson(o, len(sub))})
    return out


def calib_verdict(rows: List[Dict]) -> Dict:
    """관측 오수렴 수 대 Poisson(Σ p_pred) 95% 구간."""
    from scipy.stats import poisson
    use = [r for r in rows if r['y_mis'] is not None and r['p_pred'] is not None]
    e = float(sum(r['p_pred'] for r in use))
    o = int(sum(r['y_mis'] for r in use))
    lo, hi = (int(poisson.ppf(0.025, e)), int(poisson.ppf(0.975, e))) if e > 0 else (0, 0)
    if lo <= o <= hi:
        v = '보정 유지'
    elif o < lo:
        v = '보정 깨짐 (과대 예측: 관측이 적다)'
    else:
        v = '보정 깨짐 (과소 예측: 관측이 많다)'
    return {'n': len(use), 'observed': o, 'expected': round(e, 3), 'poisson95': [lo, hi],
            'verdict': v}


def holm(ps: List[float]) -> List[float]:
    """Holm 보정 p."""
    order = sorted(range(len(ps)), key=lambda i: ps[i])
    adj, run = [0.0] * len(ps), 0.0
    for rank, i in enumerate(order):
        run = max(run, min(1.0, (len(ps) - rank) * ps[i]))
        adj[i] = run
    return adj


def auroc_cluster(rows: List[Dict]) -> Dict:
    """−margin AUROC + (묶음, 자세) 군집 부트스트랩."""
    use = [r for r in rows if r['y_mis'] is not None and r['e0_margin'] is not None]
    if not use:
        return {'n': 0, 'auroc': None}
    s = -np.array([r['e0_margin'] for r in use])
    y = np.array([r['y_mis'] for r in use])
    a = ax.auroc(s, y)
    vals, dropped = ax.cluster_boot([r['cluster'] for r in use],
                                    lambda idx: ax.auroc(s[idx], y[idx]), seed=SEED)
    return {'n': len(use), 'events': int(y.sum()), 'n_pose': len({r['cluster'] for r in use}),
            'auroc': None if a is None else round(a, 4),
            'ci95_cluster_boot': [round(float(np.percentile(vals, 2.5)), 4),
                                  round(float(np.percentile(vals, 97.5)), 4)] if vals else None,
            'boot_valid': len(vals), 'boot_dropped_one_class': dropped}


def analyze(run_dirs: List[Path], out: Path) -> Dict:
    """전체 → out/ (tidy.csv, summary.json, summary.txt)."""
    from scipy.stats import fisher_exact
    rows = rows_from(run_dirs)
    out.mkdir(parents=True, exist_ok=False)
    fields = list(rows[0].keys()) if rows else ['trial_id']
    with (out / 'tidy.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    bundles = sorted({r['spec_id'] for r in rows})
    per = {}
    fisher_two = []
    for sid in bundles:
        sub = [r for r in rows if r['spec_id'] == sid]
        known = [r for r in sub if r['y_mis'] is not None]
        wait = [r for r in known if r['kind'] == 'waiting']
        xw, nw = sum(r['y_mis'] for r in wait), len(wait)
        table = [[xw, nw - xw], [BASE_WAIT[0], BASE_WAIT[1] - BASE_WAIT[0]]]
        p2 = float(fisher_exact(table, alternative='two-sided')[1]) if nw else None
        p1 = float(fisher_exact(table, alternative='less')[1]) if nw else None
        fisher_two.append(p2)
        per[sid] = {
            'n_done': len(sub),
            'mis_all': {'events': sum(r['y_mis'] for r in known), 'n': len(known),
                        'ci95': ax.wilson(sum(r['y_mis'] for r in known), len(known))},
            'mis_waiting': {'events': xw, 'n': nw, 'ci95': ax.wilson(xw, nw)},
            'natural_lost': {'events': sum(r['natural_lost'] for r in sub), 'n': len(sub)},
            'mis_by_kind': ax.rate_table(sub, 'y_mis', ['kind']),
            'fisher_waiting_vs_exp1': {'table': table, 'p_two_sided': p2,
                                       'p_one_sided_lower': p1},
            'calibration_bins': calib_bins(sub), 'calibration': calib_verdict(sub),
            'auroc_ref': auroc_cluster(sub)}
    for sid, p_adj in zip(bundles, holm([p if p is not None else 1.0 for p in fisher_two])):
        per[sid]['fisher_waiting_vs_exp1']['p_two_sided_holm'] = round(p_adj, 6)
    summ = {
        'run_dirs': [str(d) for d in run_dirs], 'n_done': len(rows), 'bundles': bundles,
        'logit_exp1': {'a': A, 'b': B}, 'baseline_waiting_exp1': list(BASE_WAIT),
        'outcome_counts': {k: sum(1 for r in rows if r['outcome'] == k)
                           for k in ('correct', 'ambiguous', 'misconverge', 'unknown')},
        'per_bundle': per,
        'calibration_bins_all': calib_bins(rows), 'calibration_all': calib_verdict(rows),
        'auroc_all': auroc_cluster(rows),
        'sensitivity': {
            'drop_ambiguous_calibration': calib_verdict(
                [r for r in rows if r['outcome'] != 'ambiguous']),
            'drop_ambiguous_auroc': auroc_cluster(
                [r for r in rows if r['outcome'] != 'ambiguous'])},
        'params': {'boot': ax.BOOT, 'seed': SEED, 'bins': N_BINS},
    }
    (out / 'summary.json').write_text(json.dumps(summ, ensure_ascii=False, indent=1),
                                      encoding='utf-8')
    lines = [f'n_done={len(rows)} bundles={bundles}', f'AUROC all {summ["auroc_all"]}',
             f'calibration all {summ["calibration_all"]}']
    for sid in bundles:
        p = per[sid]
        lines.append(f'{sid}: waiting {p["mis_waiting"]} all {p["mis_all"]} '
                     f'fisher {p["fisher_waiting_vs_exp1"]} calib {p["calibration"]}')
    (out / 'summary.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return summ


def main(argv=None) -> int:
    """명령줄."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-dir', action='append', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    s = analyze([Path(d) for d in a.run_dir], Path(a.out))
    print(open(Path(a.out) / 'summary.txt', encoding='utf-8').read())
    return 0 if s['n_done'] else 1


if __name__ == '__main__':
    sys.exit(main())
