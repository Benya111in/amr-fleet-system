"""
실험 1 분석 (사전 등록 — 시행 목록과 함께 결과 전에 커밋한다).

    python3 -m layopt.loc.analyze_exp1 --run-dir <e2 run> --out <analysis>/exp1_<run_id>

입력: 실행 디렉터리의 trials.json(시행 목록 사본, 시행마다 같은 자세·k·이웃의 E0 값)과 trials.jsonl(done 시행).
정의 (prereg.md §2·§3·§4):
  y_mis   = 판정(outcome) == misconverge (오차 > 1.0 m). ambiguous(0.5–1.0 m)는 주 분석에서 0,
            민감도 분석에서 뺀다.
            outcome unknown(오차 없음)은 뺀다.
  y_lost  = 자연 단계 20 s 안 LOST.
  위험 점수 = −E0 margin (마진이 작을수록 위험). margin 이 None(별칭 없음)인 시행은 AUROC 에서 빼고 수를 적는다.
  AUROC   = Mann–Whitney (동률 0.5). 95% CI = 자세(pose_id) 단위 군집 부트스트랩 2,000 회 백분위
            (한 부류만 뽑힌 표본은 버리고 수를 적는다), 시드 20261010.
  로지스틱 = P(y_mis | margin), P(y_lost | inlier) (보조: inlier_raw, occluded_frac). 뉴턴법 MLE,
            계수 CI 는 같은 군집 부트스트랩. 분리(separation)·상수 예측 변수는 추정 불가로 표시한다.
  게이트 A (H1) = 점추정 AUROC ≥ 0.8 지지, 0.7–0.8 약한 지지, < 0.7 기각.
            오수렴이 0 건이거나 전부 오수렴이면 AUROC 를 정할 수 없다 → "판정 불가" 이며 게이트 A 미통과로
            처리한다 (R_loc 는 E2 측정표 = 자세 종류 × k 비율로 대체).
"""

import argparse
import csv
import json
import math
from pathlib import Path
import sys
from typing import Dict, List, Optional, Sequence

import numpy as np

BOOT = 2000
SEED = 20261010
FIELDS = ['trial_id', 'pose_id', 'kind', 'k', 'k_used', 'x', 'y', 'yaw', 'quintile',
          'e0_margin', 'e0_inlier', 'e0_inlier_raw', 'e0_occluded_frac', 'e0_rho_true',
          'e0_second_rho', 'e0_second_dist', 'e0_truth_rank',
          'natural_lost', 'natural_inlier_min', 'natural_inlier_mean', 'trigger',
          'declared', 'recovery_s', 'err_m', 'err_yaw_deg', 'outcome', 'y_mis',
          'err_m_plus1s', 'outcome_plus1s', 'err_m_amcl', 'outcome_amcl',
          'n_reinit', 'n_seed', 'amcl_global', 'seed1_true', 'final_state',
          'gate_margin_max', 'status_margin_median', 'attempt', 'session', 'wall_s', 'rtf',
          'load_start_1m', 'load_end_1m', 'errata_dropped']


def auroc(score: np.ndarray, y: np.ndarray) -> Optional[float]:
    """Mann–Whitney AUROC (동률 0.5). 한 부류뿐이면 None."""
    pos, neg = score[y == 1], score[y == 0]
    if pos.size == 0 or neg.size == 0:
        return None
    from scipy.stats import rankdata
    r = rankdata(np.concatenate([pos, neg]))
    return float((r[:pos.size].sum() - pos.size * (pos.size + 1) / 2.0) / (pos.size * neg.size))


def logistic(x: np.ndarray, y: np.ndarray, iters: int = 100) -> Dict:
    """1 변수 로지스틱 MLE (뉴턴법). 반환 {b0, b1, se0, se1, converged, note}."""
    if y.size == 0 or y.min() == y.max():
        return {'b0': None, 'b1': None, 'note': 'one class only'}
    if np.ptp(x) == 0:
        return {'b0': None, 'b1': None, 'note': 'constant predictor'}
    X = np.column_stack([np.ones_like(x), x])
    b = np.zeros(2)
    note = ''
    for _ in range(iters):
        p = 1.0 / (1.0 + np.exp(-np.clip(X @ b, -40, 40)))
        g = X.T @ (y - p)
        H = (X * (p * (1 - p))[:, None]).T @ X + 1e-9 * np.eye(2)
        step = np.linalg.solve(H, g)
        b = b + step
        if np.max(np.abs(step)) < 1e-8:
            break
        if np.max(np.abs(b)) > 1e4:
            note = 'separation (coefficients diverge)'
            break
    p = 1.0 / (1.0 + np.exp(-np.clip(X @ b, -40, 40)))
    if not note and np.max(np.abs(p - y)) < 1e-3:
        note = 'separation (perfect fit)'
    H = (X * (p * (1 - p))[:, None]).T @ X
    try:
        cov = np.linalg.inv(H)
        se = np.sqrt(np.maximum(np.diag(cov), 0.0))
    except np.linalg.LinAlgError:
        se = np.array([math.nan, math.nan])
        note = note or 'singular Hessian'
    ll = float(np.sum(y * np.log(np.clip(p, 1e-12, 1))
                      + (1 - y) * np.log(np.clip(1 - p, 1e-12, 1))))
    return {'b0': float(b[0]), 'b1': float(b[1]), 'se0': float(se[0]), 'se1': float(se[1]),
            'loglik': ll, 'n': int(y.size), 'events': int(y.sum()), 'note': note}


def cluster_boot(groups: Sequence[str], fn, n: int = BOOT, seed: int = SEED):
    """자세 단위 군집 부트스트랩: fn(index 배열) 값들 (None 은 버림). 반환 (값 목록, 버린 수)."""
    groups = np.asarray(groups)
    uniq = np.unique(groups)
    members = {g: np.nonzero(groups == g)[0] for g in uniq}
    rng = np.random.default_rng(seed)
    vals, dropped = [], 0
    for _ in range(n):
        pick = rng.choice(uniq, size=uniq.size, replace=True)
        idx = np.concatenate([members[g] for g in pick])
        v = fn(idx)
        if v is None:
            dropped += 1
        else:
            vals.append(v)
    return vals, dropped


def wilson(k: int, n: int, z: float = 1.959964) -> List[Optional[float]]:
    """이항 비율 Wilson 95% 구간."""
    if n == 0:
        return [None, None]
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return [round(c - h, 4), round(c + h, 4)]


def gate_a(a: Optional[float]) -> str:
    """prereg §2 H1 판정."""
    if a is None:
        return '판정 불가 (오수렴이 한 부류뿐) → 게이트 A 미통과로 처리, R_loc 는 E2 측정표로 대체'
    if a >= 0.8:
        return '지지 (AUROC ≥ 0.8) → 게이트 A 통과'
    if a >= 0.7:
        return '약한 지지 (0.7 ≤ AUROC < 0.8) → 게이트 A 통과'
    return '기각 (AUROC < 0.7) → 게이트 A 미통과, R_loc 는 E2 측정표로 대체'


def tidy_rows(run_dir: Path) -> List[Dict]:
    """trials.json(목록) × trials.jsonl(done) → 시행당 한 행."""
    spec = json.loads((run_dir / 'trials.json').read_text(encoding='utf-8'))
    by_id = {t['trial_id']: t for t in spec['trials']}
    done = {}
    for line in (run_dir / 'trials.jsonl').read_text(encoding='utf-8').splitlines():
        if line.strip():
            r = json.loads(line)
            if r.get('status') == 'done':
                done[r['trial_id']] = r      # 같은 id 가 둘이면 마지막 (러너는 done 뒤 다시 돌지 않는다)
    rows = []
    for tid, r in done.items():
        t = by_id[tid]
        e = t.get('e0', {})
        nat = r.get('natural', {})
        seeds_true = r.get('seeds_true') or []
        rows.append({
            'trial_id': tid, 'pose_id': t['pose_id'], 'kind': t['kind'], 'k': t['k'],
            'k_used': t.get('k_used', t['k']), 'x': t['x'], 'y': t['y'], 'yaw': t['yaw'],
            'quintile': t.get('quintile'),
            'e0_margin': e.get('margin'), 'e0_inlier': e.get('inlier'),
            'e0_inlier_raw': e.get('inlier_raw'), 'e0_occluded_frac': e.get('occluded_frac'),
            'e0_rho_true': e.get('rho_true'), 'e0_second_rho': e.get('second_rho'),
            'e0_second_dist': e.get('second_dist'), 'e0_truth_rank': e.get('truth_rank'),
            'natural_lost': int(bool(nat.get('lost'))),
            'natural_inlier_min': nat.get('inlier_min'),
            'natural_inlier_mean': nat.get('inlier_mean'), 'trigger': r.get('trigger'),
            'declared': int(bool(r.get('declared'))), 'recovery_s': r.get('recovery_s'),
            'err_m': r.get('err_m'), 'err_yaw_deg': r.get('err_yaw_deg'),
            'outcome': r.get('outcome'),
            'y_mis': None if r.get('outcome') in (None, 'unknown') else int(
                r.get('outcome') == 'misconverge'),
            'err_m_plus1s': r.get('err_m_plus1s'), 'outcome_plus1s': r.get('outcome_plus1s'),
            'err_m_amcl': r.get('err_m_amcl'), 'outcome_amcl': r.get('outcome_amcl'),
            'n_reinit': r.get('n_reinit'), 'n_seed': r.get('n_seed'),
            'amcl_global': int(bool(r.get('amcl_global'))),
            'seed1_true': None if not seeds_true else int(bool(seeds_true[0])),
            'final_state': r.get('final_state'), 'gate_margin_max': r.get('gate_margin_max'),
            'status_margin_median': r.get('status_margin_median'),
            'attempt': r.get('attempt'), 'session': r.get('session'), 'wall_s': r.get('wall_s'),
            'rtf': r.get('rtf'),
            'load_start_1m': (r.get('load_start') or [None])[0],
            'load_end_1m': (r.get('load_end') or [None])[0]})
    rows.sort(key=lambda d: d['trial_id'])
    return rows


def rate_table(rows: List[Dict], key: str, by: Sequence[str]) -> List[Dict]:
    """by 조합별 key(0/1) 비율 + Wilson 구간."""
    out: Dict = {}
    for r in rows:
        if r[key] is None:
            continue
        g = tuple(r[b] for b in by)
        c = out.setdefault(g, [0, 0])
        c[0] += int(r[key])
        c[1] += 1
    return [{**dict(zip(by, g)), 'events': c[0], 'n': c[1], 'rate': round(c[0] / c[1], 4),
             'ci95': wilson(c[0], c[1])}
            for g, c in sorted(out.items(), key=lambda kv: str(kv[0]))]


def auroc_block(rows: List[Dict], pred: str, sign: float, ycol: str = 'y_mis',
                drop_ambiguous: bool = False) -> Dict:
    """예측 변수 하나의 AUROC + 군집 부트스트랩 CI."""
    use = [r for r in rows if r[ycol] is not None and r[pred] is not None
           and not (drop_ambiguous and r['outcome'] == 'ambiguous')]
    excluded = sum(1 for r in rows if r[ycol] is not None and r[pred] is None)
    if not use:
        return {'predictor': pred, 'n': 0, 'auroc': None}
    s = sign * np.array([float(r[pred]) for r in use])
    y = np.array([int(r[ycol]) for r in use])
    g = [r['pose_id'] for r in use]
    a = auroc(s, y)
    vals, dropped = cluster_boot(g, lambda idx: auroc(s[idx], y[idx]))
    ci = [round(float(np.percentile(vals, 2.5)), 4),
          round(float(np.percentile(vals, 97.5)), 4)] if vals else [None, None]
    return {'predictor': pred, 'direction': 'higher risk = ' + ('lower' if sign < 0 else 'higher'),
            'n': len(use), 'events': int(y.sum()), 'n_pose': len(set(g)),
            'excluded_missing_predictor': excluded,
            'auroc': None if a is None else round(a, 4), 'ci95_cluster_boot': ci,
            'boot_valid': len(vals), 'boot_dropped_one_class': dropped}


def logit_block(rows: List[Dict], pred: str, ycol: str) -> Dict:
    """로지스틱 1 변수 + 계수 군집 부트스트랩 CI."""
    use = [r for r in rows if r[ycol] is not None and r[pred] is not None]
    if not use:
        return {'predictor': pred, 'outcome': ycol, 'n': 0}
    x = np.array([float(r[pred]) for r in use])
    y = np.array([int(r[ycol]) for r in use], dtype=float)
    fit = logistic(x, y)
    res = {'predictor': pred, 'outcome': ycol, **fit}
    if fit.get('b1') is not None and not fit.get('note'):
        g = [r['pose_id'] for r in use]

        def f(idx):
            ff = logistic(x[idx], y[idx])
            return None if ff.get('b1') is None or ff.get('note') else ff['b1']
        vals, dropped = cluster_boot(g, f, n=BOOT)
        res['b1_ci95_cluster_boot'] = ([round(float(np.percentile(vals, 2.5)), 4),
                                        round(float(np.percentile(vals, 97.5)), 4)]
                                       if vals else [None, None])
        res['boot_valid'] = len(vals)
    return res


def _median(vals) -> Optional[float]:
    v = [x for x in vals if x is not None]
    return float(np.median(v)) if v else None


# 2026-10-10 05:10 추가 (주 분석 정의 불변, 민감도·탐색만): 조정자 정오표
# analysis/errata/20261009T181209Z_queue_blocked.json 에서 기준 묶음의 막힌 queue 체류 지점.
ERRATA = Path('/home/CAPYI_AI/runs/layopt/analysis/errata/20261009T181209Z_queue_blocked.json')
RTF_LOW = 0.25


def errata_ids(spec_id: str) -> List[str]:
    """정오표의 제외 queue id (파일이 없으면 빈 목록)."""
    if not ERRATA.exists():
        return []
    data = json.loads(ERRATA.read_text(encoding='utf-8'))
    return data.get('drop_queue_ids', {}).get(spec_id, [])


def analyze(run_dir: Path, out: Path) -> Dict:
    """전체 분석 → out/ (tidy.csv, summary.json, summary.txt)."""
    rows = tidy_rows(run_dir)
    spec = json.loads((run_dir / 'trials.json').read_text(encoding='utf-8'))
    spec_id = spec.get('spec_id', '')
    dropped = set(errata_ids(spec_id))
    for r in rows:
        r['errata_dropped'] = int(r['pose_id'] in dropped)
    out.mkdir(parents=True, exist_ok=False)
    with (out / 'tidy.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    known = [r for r in rows if r['y_mis'] is not None]
    primary = auroc_block(rows, 'e0_margin', -1.0)
    summ = {
        'run_dir': str(run_dir), 'n_done': len(rows), 'n_outcome_known': len(known),
        'n_pose': len({r['pose_id'] for r in rows}),
        'outcome_counts': {k: sum(1 for r in rows if r['outcome'] == k)
                           for k in ('correct', 'ambiguous', 'misconverge', 'unknown')},
        'natural_lost': {'events': sum(r['natural_lost'] for r in rows), 'n': len(rows),
                         'ci95': wilson(sum(r['natural_lost'] for r in rows), len(rows))},
        'misconverge': {'events': sum(r['y_mis'] for r in known), 'n': len(known),
                        'ci95': wilson(sum(r['y_mis'] for r in known), len(known))},
        'lost_by_k': rate_table(rows, 'natural_lost', ['k']),
        'lost_by_kind': rate_table(rows, 'natural_lost', ['kind']),
        'lost_by_kind_k': rate_table(rows, 'natural_lost', ['kind', 'k']),
        'mis_by_k': rate_table(rows, 'y_mis', ['k']),
        'mis_by_kind': rate_table(rows, 'y_mis', ['kind']),
        'mis_by_kind_k': rate_table(rows, 'y_mis', ['kind', 'k']),
        'mis_by_quintile': rate_table([r for r in rows if r['quintile'] is not None], 'y_mis',
                                      ['quintile']),
        'auroc_primary': primary,
        'gate_A': gate_a(primary.get('auroc')),
        'auroc_sensitivity': {
            'drop_ambiguous': auroc_block(rows, 'e0_margin', -1.0, drop_ambiguous=True),
            # 아래 둘은 실험 시작 뒤(결과 분석 전) 추가한 민감도 — 주 판정은 auroc_primary
            'drop_errata_queue': auroc_block([r for r in rows if not r['errata_dropped']],
                                             'e0_margin', -1.0),
            'drop_rtf_below_0.25': auroc_block(
                [r for r in rows if r['rtf'] is None or r['rtf'] >= RTF_LOW], 'e0_margin', -1.0),
            'errata_ids': sorted(dropped),
        },
        'logit_mis_margin': logit_block(rows, 'e0_margin', 'y_mis'),
        'logit_lost_inlier': logit_block(rows, 'e0_inlier', 'natural_lost'),
        'exploratory': {
            'logit_lost_inlier_raw': logit_block(rows, 'e0_inlier_raw', 'natural_lost'),
            'logit_lost_occluded': logit_block(rows, 'e0_occluded_frac', 'natural_lost'),
            'auroc_mis_rho_true': auroc_block(rows, 'e0_rho_true', -1.0),
            'auroc_mis_occluded': auroc_block(rows, 'e0_occluded_frac', 1.0),
            'auroc_lost_inlier': auroc_block(rows, 'e0_inlier', -1.0, ycol='natural_lost'),
            'auroc_lost_inlier_raw': auroc_block(rows, 'e0_inlier_raw', -1.0,
                                                 ycol='natural_lost'),
        },
        'outcome_agreement': {
            'plus1s_same': sum(1 for r in rows if r['outcome'] == r['outcome_plus1s']),
            'amcl_same': sum(1 for r in rows if r['outcome'] == r['outcome_amcl']),
            'n': len(rows)},
        'trigger_counts': {t: sum(1 for r in rows if r['trigger'] == t)
                           for t in sorted({r['trigger'] for r in rows if r['trigger']})},
        'declared': sum(r['declared'] for r in rows),
        'recovery_s_median': _median([r['recovery_s'] for r in rows]),
        'wall_s_median': float(np.median([r['wall_s'] for r in rows])) if rows else None,
        'load_start_1m_range': ([min(r['load_start_1m'] for r in rows),
                                 max(r['load_start_1m'] for r in rows)] if rows else None),
        'params': {'boot': BOOT, 'seed': SEED},
    }
    (out / 'summary.json').write_text(json.dumps(summ, ensure_ascii=False, indent=1),
                                      encoding='utf-8')
    lines = [f'n_done={summ["n_done"]} n_pose={summ["n_pose"]} '
             f'outcomes={summ["outcome_counts"]}',
             f'natural LOST {summ["natural_lost"]}', f'misconverge {summ["misconverge"]}',
             f'AUROC(-margin → mis) {primary}', f'gate A: {summ["gate_A"]}']
    (out / 'summary.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return summ


def main(argv=None) -> int:
    """명령줄."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-dir', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    s = analyze(Path(a.run_dir), Path(a.out))
    keys = ('n_done', 'outcome_counts', 'natural_lost', 'misconverge', 'auroc_primary', 'gate_A')
    print(json.dumps({k: s[k] for k in keys}, ensure_ascii=False, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
