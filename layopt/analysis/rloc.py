"""R_loc·비교군·H2/H3 판정 (prereg §2·§6·§7, 일지 2026-10-10 08:4x 규칙).

R_loc(배치, 시드) = Σ_{체류 지점 d, 이웃 수 k} dwell_s(d, k) × λ_LOST × P_mis(margin_E0(d, k_e0))
  - k_e0: 0→0, 1·2→2, 3·4→4
  - λ_LOST: analysis/*_lambda_lost (P5 실측, 배치 무관 상수)
  - P_mis(m) = σ(a + b·m): 실험 1 강제 복구 시행 로지스틱 적합
  - 정오표의 queue 지점은 뺀다
단위: 1 시간 교대당 기대 오수렴 사건 수 (부하 1.0, 시드 평균).

사용 (amr-ai/rl:0.3 컨테이너, numpy·scipy·matplotlib):
  python3 rloc.py --e1 <dir> [<dir> ...] --e0 <dir> --errata <json> --lam <json>
                  --pmis '{"a": .., "b": ..}' --out <새 디렉터리>
"""
import argparse
import csv
import json
import math
import os

import numpy as np

BASE = 'v10-v20-v30-v40-v50'
K_MAP = {0: 0, 1: 2, 2: 2, 3: 4, 4: 4}


def load_e1(dirs):
    rows, seen = [], set()
    for d in dirs:
        for line in open(os.path.join(d, 'results.jsonl')):
            r = json.loads(line)
            key = (r['spec_id'], r['seed'], r['load'])
            if key in seen:
                continue
            seen.add(key)
            rows.append(r)
    return rows


def load_e0(d, specs):
    out = {}
    for s in specs:
        p = os.path.join(d, s + '.json')
        if not os.path.exists(p):
            continue
        out[s] = {(q['id'], q['k']): q['margin'] for q in json.load(open(p))['points']}
    return out


def pmis(m, a, b):
    return 1.0 / (1.0 + math.exp(-(a + b * m)))


def rloc_row(r, e0s, drop, lam, a, b):
    tot, miss = 0.0, 0
    for dw in r['dwell']:
        if dw['id'] in drop:
            continue
        for k, s in dw['k_hist_s'].items():
            if s <= 0:
                continue
            m = e0s.get((dw['id'], K_MAP[int(k)]))
            if m is None:
                miss += 1
                continue
            tot += s * lam * pmis(m, a, b)
    return tot * 3600.0 / r['horizon_s'], miss


def boot_ci(x, n=2000, seed=0):
    rng = np.random.default_rng(seed)
    x = np.asarray(x)
    meds = [np.median(rng.choice(x, len(x))) for _ in range(n)]
    return float(np.percentile(meds, 2.5)), float(np.percentile(meds, 97.5))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--e1', nargs='+', required=True)
    ap.add_argument('--e0', required=True)
    ap.add_argument('--errata', required=True)
    ap.add_argument('--lam', required=True)
    ap.add_argument('--pmis', required=True)
    ap.add_argument('--load', type=float, default=1.0)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    os.makedirs(args.out)  # 이미 있으면 실패 (덮어쓰기 금지)
    pm = json.loads(args.pmis)
    a, b = pm['a'], pm['b']
    lam = json.load(open(args.lam))['lambda_per_s']
    errata = json.load(open(args.errata))['drop_queue_ids']
    rows = [r for r in load_e1(args.e1) if r['load'] == args.load]
    specs = sorted({r['spec_id'] for r in rows})
    e0 = load_e0(args.e0, specs)
    per = {}
    n_miss = 0
    for r in rows:
        s = r['spec_id']
        if s not in e0:
            continue
        rl, miss = rloc_row(r, e0[s], set(errata.get(s, [])), lam, a, b)
        n_miss += miss
        per.setdefault(s, {})[r['seed']] = (r['throughput_per_h'], r['task_p95_s'], rl,
                                           r['deadlocks'], r['unresolved'], r['gate_wait_s'])
    base = per[BASE]
    table = []
    for s, d in per.items():
        seeds = sorted(set(d) & set(base))
        T = np.array([d[i][0] for i in seeds])
        TB = np.array([base[i][0] for i in seeds])
        R = np.array([d[i][2] for i in seeds])
        RB = np.array([base[i][2] for i in seeds])
        ratio = T / TB
        lo, hi = boot_ci(ratio)
        table.append({'spec_id': s, 'n': len(seeds), 'T': float(T.mean()), 'T_ratio_med': float(np.median(ratio)),
                      'T_ratio_ci': [lo, hi], 'p95': float(np.mean([d[i][1] for i in seeds])),
                      'R_loc': float(R.mean()), 'R_ratio': float(R.mean() / RB.mean()) if RB.mean() else None,
                      'deadlocks': float(np.mean([d[i][3] for i in seeds])),
                      'unresolved': float(np.mean([d[i][4] for i in seeds])),
                      'gate_wait_s': float(np.mean([d[i][5] for i in seeds])),
                      '_T': T.tolist(), '_R': R.tolist(), '_seeds': seeds})
    by = {t['spec_id']: t for t in table}
    tb = by[BASE]
    traffic = max(table, key=lambda t: (t['T'], -t['p95']))['spec_id']
    locopt = min(table, key=lambda t: (t['R_loc'], -t['T']))['spec_id']
    feas = [t for t in table if t['T'] >= 0.95 * tb['T']]
    knee = min(feas, key=lambda t: (t['R_loc'], -t['T']))['spec_id']
    fix = 'v13-v20-v30-v40-v50'
    groups = {'baseline': BASE, 'human_fix': fix, 'traffic_opt': traffic, 'loc_opt': locopt, 'knee': knee}
    from scipy.stats import wilcoxon
    tests = {}
    pv = []
    for g, s in groups.items():
        if g == 'baseline' or s not in by:
            continue
        t = by[s]
        seeds = t['_seeds']
        rb = [base[i][2] for i in seeds]
        tbs = [base[i][0] for i in seeds]
        pr = wilcoxon(t['_R'], rb).pvalue if any(x != y for x, y in zip(t['_R'], rb)) else 1.0
        pt = wilcoxon(t['_T'], tbs).pvalue if any(x != y for x, y in zip(t['_T'], tbs)) else 1.0
        tests[g] = {'spec_id': s, 'p_R': float(pr), 'p_T': float(pt)}
        pv += [(g, 'p_R', pr), (g, 'p_T', pt)]
    # Holm
    pv.sort(key=lambda x: x[2])
    m = len(pv)
    run = 0.0
    for i, (g, k, p) in enumerate(pv):
        run = max(run, min(1.0, (m - i) * p))
        tests[g][k + '_holm'] = run
    h2 = by[traffic]['R_loc'] > 0.9 * tb['R_loc']
    h3 = any(t['R_loc'] <= 0.2 * tb['R_loc'] for t in feas)
    verdict = {'H2_supported': bool(h2), 'H2_detail': {'traffic_opt': traffic, 'R_ratio': by[traffic]['R_ratio']},
               'H3_supported': bool(h3), 'H3_detail': {'knee': knee, 'R_ratio': by[knee]['R_ratio'],
                                                        'T_ratio_med': by[knee]['T_ratio_med']}}
    res = {'params': {'lambda_per_s': lam, 'pmis': pm, 'load': args.load, 'e1': args.e1, 'e0': args.e0,
                      'k_map': K_MAP, 'missing_margin_cells': n_miss},
           'groups': groups, 'group_rows': {g: {k: v for k, v in by[s].items() if not k.startswith('_')}
                                            for g, s in groups.items() if s in by},
           'tests': tests, 'verdict': verdict, 'n_specs': len(table)}
    json.dump(res, open(os.path.join(args.out, 'rloc_summary.json'), 'x'), indent=1)
    with open(os.path.join(args.out, 'rloc_table.csv'), 'x', newline='') as f:
        w = csv.writer(f)
        cols = ['spec_id', 'n', 'T', 'T_ratio_med', 'p95', 'R_loc', 'R_ratio', 'deadlocks', 'unresolved',
                'gate_wait_s']
        w.writerow(cols)
        for t in sorted(table, key=lambda t: t['R_loc']):
            w.writerow([t[c] for c in cols])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6, 4.2))
    ax.scatter([t['T'] for t in table], [t['R_loc'] for t in table], s=12, c='#9aa4b1', label='평가 배치')
    mk = {'baseline': ('#c0392b', 'o'), 'human_fix': ('#8e44ad', 's'), 'traffic_opt': ('#2471a3', '^'),
          'loc_opt': ('#117a65', 'D'), 'knee': ('#d68910', '*')}
    for g, s in groups.items():
        t = by[s]
        ax.scatter([t['T']], [t['R_loc']], s=110 if g == 'knee' else 60, c=mk[g][0], marker=mk[g][1],
                   label=f'{g}: {s}', zorder=3)
    ax.axvline(0.95 * tb['T'], ls='--', lw=0.8, c='#555')
    ax.set_xlabel('throughput [tasks/h] (load 1.0)')
    ax.set_ylabel('R_loc [expected misconvergence /h]')
    ax.legend(fontsize=6, loc='upper left')
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'pareto.png'), dpi=160)
    print(json.dumps({'groups': groups, 'verdict': verdict, 'tests': tests, 'n_specs': len(table),
                      'missing': n_miss}, indent=1))


if __name__ == '__main__':
    main()
