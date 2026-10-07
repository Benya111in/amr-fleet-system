"""
지도 변형 평가 집계 (b1-mapgen 사전 등록 분석) — numpy + amr_rl_eval, Isaac 비의존.

정의 (노트 항목 2026-10-08 b1-mapgen 「판정 규칙」 과 같다 — 결과 전에 커밋):
  시드 하나 = eval_mapgen.py 의 episodes.npz 하나. 결말 코드 0 성공(엄격) 1 충돌 2 시간 상한 3 정체.
  명목      = 명목 지도의 main_3_15 2 048 행 (cond −1).
  1 차 모집단 (조건 c) = 조건 c 의 main_3_15 유효 행 중 변형 지도에서도 끝 직진 여유 ≥ 0.40 m 인 행 (main_variant).
  P1map     = 시드마다 6 조건 1 차 모집단 엄격 성공의 같은 무게 평균.  Δ = 평균(DR 켬 5 시드) − 평균(DR 끔 5 시드).
  S1        = 짝 열화: 조건마다 (같은 행의 명목 엄격 성공 − 변형 엄격 성공) 의 6 조건 평균; 효과 = 평균(끔) − 평균(켬).
  S2        = 조건별 Δ (6 개) — 정확 순열 p 에 Holm (α 0.05).
  S3        = 충돌률: 시드마다 6 조건 1 차 모집단을 모은 충돌 에피소드 비율; Δ = 켬 − 끔.
  구간 = 팔별 시드 복원 추출 부트스트랩 95 % (reps 4000, seed 0 — b1-dr dr_analysis.boot_diff 와 같은 난수 순서),
  검정 = 10 시드 값의 정확 순열 (5/5 분할 252 개, 양측, 통계량 = 평균 차).
"""

import itertools
import json
import math
import os

import numpy as np

from amr_rl_eval import aggregate as AG
from amr_rl_eval import binomial as BN
from amr_rl_eval import multiplicity as MU

REPS = 4000
BSEED = 0
Z95 = 1.959963984540054
NOMINAL_SET = 'main_3_15'
STRESS_SET = 'stress_3_15'
IDENTITY_SKIP = ('set_index', 'cond_index', 'goal_group', 'got')
ROW_EXTRA = ('tile', 'bundle_row', 'pid_orig', 'row_j', 'main_variant')


def load_run(d):
    """평가 디렉터리 → (eval.json dict, episodes dict)."""
    with open(os.path.join(d, 'eval.json'), encoding='utf-8') as f:
        ev = json.load(f)
    ep = dict(np.load(os.path.join(d, 'episodes.npz')))
    return ev, ep


def _rates(ep, m):
    """마스크 m 의 결말 비율과 기술 지표."""
    n = int(m.sum())
    if n == 0:
        return {'n': 0}
    o = ep['outcome'][m]
    succ = o == 0
    t = ep['time_s'][m]
    d_end = ep['d_end'][m]
    return {'n': n, 'strict': float(succ.mean()), 'nav2': float(ep['nav2_reach'][m].mean()),
            'collision': float((o == 1).mean()), 'collisions': int((o == 1).sum()),
            'time_limit': float((o == 2).mean()), 'stall': float((o == 3).mean()),
            'intervention_per_ep': float(ep['cost'][m][:, 1].mean()),
            'time_s_mean_success': float(t[succ].mean()) if succ.any() else None,
            'fail_end_median_m': float(np.median(d_end[~succ])) if (~succ).any() else None,
            'fail_within_0p25m': int(((~succ) & (d_end <= 0.25)).sum())}


def seed_metrics(ev, ep, bundle=None):
    """
    시드 하나의 지표 dict.

    반환 {'nominal': …, 'cond': {조건: {'primary', 'valid_main', 'stress', 'stress_narrow',
    'matched_nominal_strict', ('near_added', 'far_added')}}}.
    """
    sets = ev['sets']
    conds = ev['conditions']
    got = ep['got'].astype(bool)
    si_main = sets.index(NOMINAL_SET)
    si_stress = sets.index(STRESS_SET) if STRESS_SET in sets else -1
    nom = (ep['cond_index'] == -1) & (ep['set_index'] == si_main) & got
    out = {'nominal': _rates(ep, nom), 'cond': {}}
    nom_by_j = {}
    for k in np.flatnonzero(nom):
        nom_by_j[int(ep['row_j'][k])] = bool(ep['outcome'][k] == 0)
    for ci, cn in enumerate(conds):
        c = (ep['cond_index'] == ci) & got
        main = c & (ep['set_index'] == si_main)
        prim = main & ep['main_variant'].astype(bool)
        blk = {'primary': _rates(ep, prim), 'valid_main': _rates(ep, main)}
        if si_stress >= 0:
            st = c & (ep['set_index'] == si_stress)
            blk['stress'] = _rates(ep, st)
            blk['stress_narrow'] = _rates(ep, st & ~ep['main_variant'].astype(bool))
        js = [int(j) for j in ep['row_j'][prim]]
        blk['matched_nominal_strict'] = float(np.mean([nom_by_j[j] for j in js])) if js else None
        if bundle is not None and prim.any():
            near = bundle['row_near_added'][ep['bundle_row'][prim]] < 0.5
            if near.any() and (~near).any():
                idx = np.flatnonzero(prim)
                m_near = np.zeros_like(prim)
                m_near[idx[near]] = True
                blk['near_added'] = _rates(ep, m_near)
                blk['far_added'] = _rates(ep, prim & ~m_near)
        out['cond'][cn] = blk
    return out


def boot_diff(a, b, reps=REPS, seed=BSEED):
    """평균 차 a − b 와 팔별 시드 복원 추출 부트스트랩 95 % (b1-dr dr_analysis 와 같은 난수 순서)."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    rng = np.random.default_rng(seed)
    d = a[rng.integers(0, a.size, (reps, a.size))].mean(1) - \
        b[rng.integers(0, b.size, (reps, b.size))].mean(1)
    lo, hi = np.percentile(d, [2.5, 97.5])
    return float(a.mean() - b.mean()), [float(lo), float(hi)]


def perm_p(a, b):
    """정확 순열 검정 (양측, 통계량 = 평균 차, 모든 분할) → (p, 분할 수)."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    allv = np.concatenate([a, b])
    n, na = allv.size, a.size
    obs = abs(a.mean() - b.mean())
    cnt = tot = 0
    for idx in itertools.combinations(range(n), na):
        m = np.zeros(n, dtype=bool)
        m[list(idx)] = True
        d = abs(allv[m].mean() - allv[~m].mean())
        cnt += d >= obs - 1e-12
        tot += 1
    return cnt / tot, tot


def verdict(diff, ci, p, alpha=0.05):
    """사전 등록 판정 — 'up' (켬 > 끔 확인) · 'down' (켬 < 끔 확인) · 'undetermined' (가르지 못함)."""
    if diff > 0 and ci[0] > 0 and p <= alpha:
        return 'up'
    if diff < 0 and ci[1] < 0 and p <= alpha:
        return 'down'
    return 'undetermined'


def compare(on, off):
    """켬 − 끔 비교 블록 (구간·순열·개선 확률·짝 부호)."""
    diff, ci = boot_diff(on, off)
    p, tot = perm_p(on, off)
    pi = AG.probability_of_improvement_ci(np.asarray(on)[:, None], np.asarray(off)[:, None],
                                          reps=REPS, seed=BSEED)
    return {'on': [round(float(x), 6) for x in on], 'off': [round(float(x), 6) for x in off],
            'mean_on': float(np.mean(on)), 'mean_off': float(np.mean(off)),
            'diff_on_minus_off': diff, 'boot95': ci, 'perm_p': p, 'perm_n': tot,
            'verdict': verdict(diff, ci, p), 'prob_improvement_on_gt_off': list(pi),
            'paired_sign_on_gt_off': int(sum(x > y for x, y in zip(on, off))),
            'paired_ties': int(sum(x == y for x, y in zip(on, off)))}


def fail_seeds(v, thr=0.90):
    """실패 시드 (엄격 < thr) 수와 Wilson 95 %."""
    k = int(sum(x < thr for x in v))
    lo, hi = BN.wilson_interval(k, len(v), Z95)
    return {'failed': k, 'n': len(v), 'wilson95': [lo, hi]}


def min_perm_p(n_a, n_b):
    """두 팔 크기의 정확 순열 검정이 낼 수 있는 가장 작은 양측 p (동률 없음)."""
    return 2.0 / math.comb(n_a + n_b, n_a)


def identity_check(ep, ref):
    """
    명목 행 (cond −1) 의 에피소드 배열이 기존 평가 episodes.npz 와 비트 단위로 같은가.

    기존 평가는 같은 행·같은 순서 (eval_unbiased --eval_set) — 길이가 같아야 하고, 기존 쪽의 행 메타 열
    (IDENTITY_SKIP) 을 뺀 모든 배열을 비교한다. 반환 {'pass', 'n', 'mismatch': [키]}.
    """
    m = ep['cond_index'] == -1
    n_ref = int(np.asarray(ref['outcome']).shape[0])
    res = {'n_rows': int(m.sum()), 'n_ref': n_ref, 'mismatch': [], 'missing': []}
    if int(m.sum()) != n_ref:
        res['pass'] = False
        return res
    for k, v in ref.items():
        if k in IDENTITY_SKIP:
            continue
        if k not in ep:
            res['missing'].append(k)
            continue
        a, b = np.asarray(ep[k])[m], np.asarray(v)
        eq = a.shape == b.shape and (np.array_equal(a, b, equal_nan=True)
                                     if a.dtype.kind in 'fc' else np.array_equal(a, b))
        if not eq:
            res['mismatch'].append(k)
    res['compared_keys'] = sorted(k for k in ref if k not in IDENTITY_SKIP)
    res['pass'] = not res['mismatch'] and not res['missing']
    return res


def analyze(on_runs, off_runs, refs_on=None, refs_off=None, bundle=None):
    """
    사전 등록 분석 → 결과 dict.

    on_runs·off_runs = [(시드, eval dict, episodes dict)], refs_* = {시드: episodes dict} (기존 명목 평가).
    """
    seeds_on = [s for s, _e, _p in on_runs]
    seeds_off = [s for s, _e, _p in off_runs]
    conds = on_runs[0][1]['conditions']
    sig = {json.dumps([e['bundle_sha256'], e['eval_seed'], e['safety_filter'], e['sets'],
                       e['conditions'], e['conditions_run'], e['nominal_tile'],
                       e['rows_per_cond'], e['zero_policy'], e['max_envs']])
           for _s, e, _p in on_runs + off_runs}
    if len(sig) != 1:
        raise ValueError(f'평가 조건이 섞였다: {sig}')
    unfinished = {arm: [e['unfinished'] for _s, e, _p in runs]
                  for arm, runs in (('on', on_runs), ('off', off_runs))}
    met = {'on': [seed_metrics(e, p, bundle) for _s, e, p in on_runs],
           'off': [seed_metrics(e, p, bundle) for _s, e, p in off_runs]}

    def per_seed(arm, f):
        return [f(m) for m in met[arm]]

    def p1(m):
        return float(np.mean([m['cond'][c]['primary']['strict'] for c in conds]))

    def s1(m):
        return float(np.mean([m['cond'][c]['matched_nominal_strict'] -
                              m['cond'][c]['primary']['strict'] for c in conds]))

    def s3(m):
        n = sum(m['cond'][c]['primary']['n'] for c in conds)
        k = sum(m['cond'][c]['primary']['collisions'] for c in conds)
        return k / n

    res = {'seeds': {'on': seeds_on, 'off': seeds_off}, 'conditions': conds,
           'eval_signature': json.loads(next(iter(sig))), 'unfinished': unfinished,
           'min_perm_p': min_perm_p(len(seeds_on), len(seeds_off)),
           'holm_first_threshold': 0.05 / len(conds)}
    res['holm_can_reject'] = res['min_perm_p'] <= res['holm_first_threshold']
    on1, off1 = per_seed('on', p1), per_seed('off', p1)
    res['P1map'] = compare(on1, off1)
    res['P1map']['iqm'] = {}
    for arm, v in (('on', on1), ('off', off1)):
        est, lo, hi = AG.stratified_bootstrap(np.asarray(v)[:, None], func=AG.iqm, reps=REPS,
                                              seed=BSEED)
        res['P1map']['iqm'][arm] = [est, lo, hi]
    res['P1map']['fail_seeds'] = {'on': fail_seeds(on1), 'off': fail_seeds(off1)}
    # S1: 효과 = 끔 − 켬 (양수 = DR 이 열화를 줄임) — 판정 함수는 같은 규칙을 이 부호로
    res['S1_degradation'] = compare(per_seed('off', s1), per_seed('on', s1))
    res['S1_degradation']['note'] = "on/off 칸이 바뀐 비교: 'on' 열 = 끔 팔, 'off' 열 = 켬 팔"
    res['S1_degradation']['off_arm_values'] = per_seed('off', s1)
    res['S1_degradation']['on_arm_values'] = per_seed('on', s1)
    res['S2'] = {}
    ps = []
    for c in conds:
        blk = compare(per_seed('on', lambda m: m['cond'][c]['primary']['strict']),
                      per_seed('off', lambda m: m['cond'][c]['primary']['strict']))
        blk['fail_seeds'] = {arm: fail_seeds(per_seed(arm, lambda m: m['cond'][c]['primary']
                                                      ['strict'])) for arm in ('on', 'off')}
        res['S2'][c] = blk
        ps.append(blk['perm_p'])
    rej, padj = MU.holm(ps, alpha=0.05)
    for c, r, pa in zip(conds, rej, padj):
        res['S2'][c]['holm_reject'] = bool(r)
        res['S2'][c]['holm_p'] = float(pa)
    res['S3_collision'] = compare(per_seed('on', s3), per_seed('off', s3))
    res['nominal'] = compare(per_seed('on', lambda m: m['nominal']['strict']),
                             per_seed('off', lambda m: m['nominal']['strict']))
    # 기술: 조건·팔별 시드 평균
    keys = ('strict', 'nav2', 'collision', 'collisions', 'time_limit', 'stall',
            'intervention_per_ep', 'time_s_mean_success', 'fail_end_median_m', 'n')
    desc = {}
    for c in ['nominal'] + list(conds):
        for pop in (('nominal',) if c == 'nominal' else ('primary', 'valid_main', 'stress',
                                                         'stress_narrow', 'near_added',
                                                         'far_added')):
            row = {}
            for arm in ('on', 'off'):
                vals = {}
                for k in keys:
                    xs = []
                    for m in met[arm]:
                        b = m['nominal'] if c == 'nominal' else m['cond'][c].get(pop)
                        if b and b.get('n') and b.get(k) is not None:
                            xs.append(b[k])
                    if xs:
                        vals[k] = float(np.sum(xs)) if k in ('collisions', 'n') else \
                            float(np.mean(xs))
                        vals[k + '_per_seed'] = [round(float(x), 6) for x in xs]
                row[arm] = vals
            if row['on'] or row['off']:
                desc[f'{c}|{pop}'] = row
    res['descriptive'] = desc
    # 스트레스: 조건별 Δ (기술 — 판정하지 않음)
    res['stress_descriptive'] = {}
    for c in conds:
        a = per_seed('on', lambda m: m['cond'][c].get('stress', {}).get('strict'))
        b = per_seed('off', lambda m: m['cond'][c].get('stress', {}).get('strict'))
        if all(x is not None for x in a + b):
            res['stress_descriptive'][c] = compare(a, b)
    if refs_on is not None or refs_off is not None:
        res['identity'] = {}
        for arm, runs, refs in (('on', on_runs, refs_on or {}), ('off', off_runs, refs_off or {})):
            for s, _e, ep in runs:
                if s in refs:
                    res['identity'][f'{arm}_s{s}'] = identity_check(ep, refs[s])
        res['identity_all_pass'] = all(v['pass'] for v in res['identity'].values()) and \
            len(res['identity']) == len(on_runs) + len(off_runs)
    return res


def fmt(x, nd=4, sign=False):
    """표 숫자."""
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return '—'
    return f'{x:+.{nd}f}' if sign else f'{x:.{nd}f}'


def markdown(res):
    """결과 dict → 마크다운 표 (결과 문서에 옮길 원본)."""
    L = []
    P = res['P1map']
    L.append('## 사전 등록 판정\n')
    L.append('| 지표 | 켬 시드별 | 끔 시드별 | Δ 켬 − 끔 [시드 부트 95 %] | 순열 p | 판정 |')
    L.append('|---|---|---|---|---|---|')

    def row(name, b, flip=False):
        on = ' · '.join(fmt(x) for x in (b['off'] if flip else b['on']))
        off = ' · '.join(fmt(x) for x in (b['on'] if flip else b['off']))
        L.append(f"| {name} | {on} | {off} | {fmt(b['diff_on_minus_off'], sign=True)} "
                 f"[{fmt(b['boot95'][0], sign=True)}, {fmt(b['boot95'][1], sign=True)}] "
                 f"| {b['perm_p']:.3f} | {b['verdict']} |")
    row('P1map (6 조건 1 차 모집단 엄격 성공 평균)', P)
    L.append(f"| S1 짝 열화 (끔 − 켬, 양수 = DR 이 열화를 줄임; 표의 켬·끔 열은 각 팔의 열화) | "
             f"{' · '.join(fmt(x) for x in res['S1_degradation']['on_arm_values'])} | "
             f"{' · '.join(fmt(x) for x in res['S1_degradation']['off_arm_values'])} | "
             f"{fmt(res['S1_degradation']['diff_on_minus_off'], sign=True)} "
             f"[{fmt(res['S1_degradation']['boot95'][0], sign=True)}, "
             f"{fmt(res['S1_degradation']['boot95'][1], sign=True)}] | "
             f"{res['S1_degradation']['perm_p']:.3f} | {res['S1_degradation']['verdict']} |")
    row('S3 충돌률 (6 조건 1 차 모집단 합)', res['S3_collision'])
    row('명목 main_3_15 (대조)', res['nominal'])
    L.append('')
    L.append(f"IQM 켬 {fmt(P['iqm']['on'][0])} [{fmt(P['iqm']['on'][1])}, {fmt(P['iqm']['on'][2])}]"
             f" · 끔 {fmt(P['iqm']['off'][0])} [{fmt(P['iqm']['off'][1])}, "
             f"{fmt(P['iqm']['off'][2])}]; P(켬 > 끔) {fmt(P['prob_improvement_on_gt_off'][0], 2)}"
             f" [{fmt(P['prob_improvement_on_gt_off'][1], 2)}, "
             f"{fmt(P['prob_improvement_on_gt_off'][2], 2)}]; 짝 부호 켬 > 끔 "
             f"{P['paired_sign_on_gt_off']}/{len(P['on'])}. 최소 순열 p {res['min_perm_p']:.4f}, "
             f"Holm 첫 문턱 {res['holm_first_threshold']:.4f} → 기각 가능 {res['holm_can_reject']}.\n")
    L.append('## 조건별 (S2, 1 차 모집단)\n')
    L.append('| 조건 | 켬 평균 | 끔 평균 | Δ [부트 95 %] | 순열 p | Holm p | 실패 시드 켬/끔 |')
    L.append('|---|---|---|---|---|---|---|')
    for c, b in res['S2'].items():
        L.append(f"| {c} | {fmt(b['mean_on'])} | {fmt(b['mean_off'])} | "
                 f"{fmt(b['diff_on_minus_off'], sign=True)} [{fmt(b['boot95'][0], sign=True)}, "
                 f"{fmt(b['boot95'][1], sign=True)}] | {b['perm_p']:.3f} | {b['holm_p']:.3f} | "
                 f"{b['fail_seeds']['on']['failed']}/{b['fail_seeds']['off']['failed']} |")
    L.append('\n## 기술 (시드 평균, 충돌·n 은 5 시드 합)\n')
    L.append('| 조건·모집단 | 팔 | n | 엄격 | Nav2 | 충돌 수 | 시간 상한 | 정체 | 개입/ep | '
             '성공 시간 s | 실패 끝 거리 중앙 m |')
    L.append('|---|---|---|---|---|---|---|---|---|---|---|')
    for k, row_ in res['descriptive'].items():
        for arm in ('on', 'off'):
            v = row_.get(arm) or {}
            if not v:
                continue
            L.append(f"| {k} | {arm} | {int(v.get('n', 0))} | {fmt(v.get('strict'))} | "
                     f"{fmt(v.get('nav2'))} | {int(v.get('collisions', 0))} | "
                     f"{fmt(v.get('time_limit'))} | {fmt(v.get('stall'))} | "
                     f"{fmt(v.get('intervention_per_ep'), 3)} | "
                     f"{fmt(v.get('time_s_mean_success'), 2)} | "
                     f"{fmt(v.get('fail_end_median_m'), 3)} |")
    if res.get('stress_descriptive'):
        L.append('\n## 스트레스 (기술 — 판정 없음)\n')
        L.append('| 조건 | 켬 평균 | 끔 평균 | Δ [부트 95 %] | 순열 p |')
        L.append('|---|---|---|---|---|')
        for c, b in res['stress_descriptive'].items():
            L.append(f"| {c} | {fmt(b['mean_on'])} | {fmt(b['mean_off'])} | "
                     f"{fmt(b['diff_on_minus_off'], sign=True)} "
                     f"[{fmt(b['boot95'][0], sign=True)}, "
                     f"{fmt(b['boot95'][1], sign=True)}] | {b['perm_p']:.3f} |")
    if 'identity' in res:
        L.append('\n## 명목 행 비트 대조 (기존 평가 episodes.npz)\n')
        for k, v in res['identity'].items():
            L.append(f"- {k}: {'같음' if v['pass'] else '다름'} (행 {v['n_rows']}, "
                     f"기준 {v['n_ref']}, 다른 키 {v['mismatch']}, 없는 키 {v['missing']})")
    return '\n'.join(L) + '\n'
