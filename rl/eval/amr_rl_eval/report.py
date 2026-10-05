"""
결과 CSV → 사전 등록 판정표 (rl/eval/PROTOCOL.md §6 을 코드로 옮긴 것).

판정 낱말은 네 가지뿐이다 (COORDINATION.md §5):
  충족      자료가 있고 사전 등록 기준을 넘었다
  미달      자료가 있고 기준을 못 넘었다 (표본 상한에 닿았거나 검정이 기각하지 못했다)
  판정 불가  자료가 기준을 볼 만큼 없다 (진행 중, 시드 부족, 오류율 초과, 열 없음)
  미측정    그 지표를 계산할 열이 비어 있다
고정 순서 검정에서 앞 가설이 충족이 아니면 뒤 가설은 '검정 안 함' 이다 (α 를 쓰지 않는다).
"""

from collections import defaultdict
import math

import numpy as np

from . import aggregate, binomial, cvar, multiplicity, noninferiority as ni
from . import paired, schema, sequential, srcc

MET, NOT_MET, UNDECIDABLE, UNMEASURED, SKIPPED = '충족', '미달', '판정 불가', '미측정', '검정 안 함'

PREREG = {
    # PROTOCOL.md §6 — 값을 바꾸려면 실행 전에 PROTOCOL 개정을 먼저 커밋한다
    'alpha': 0.025,
    'delta_collision': 0.02,
    'delta_collision_aux': 0.01,
    'delta_success': 0.03,
    'sprt_p0': 0.02,
    'sprt_p1': 0.005,
    'sprt_n_max': 600,
    'primary_arm': 'C_hybrid',
    'reference_arm': 'A_dwa',
    'secondary_arms': ('B_ppo', 'B_sac', 'D_param'),
    'confirm_simulator': 'gazebo',
    'confirm_set': 'gz_new600',
    'gate_collision_cp_upper': 0.005,
    'gate_success_delta': 0.02,
    'error_rate_max': 0.02,
    'cvar_alphas': (0.05, 0.10, 0.25),
    'cvar_min_episodes_per_level': 50,
    'srcc_min_policies': 10,
    'min_seeds': 5,
    'spec_block': 30,
    'spec_dev_m': 1.0,
    'spec_return_s': 5.0,
}


def sprt_design(prereg=PREREG):
    """사전 등록 SPRT (Ville 경계, 포기 경계 없음, 상한 n_max)."""
    return sequential.SPRTDesign(p0=prereg['sprt_p0'], p1=prereg['sprt_p1'],
                                 alpha=prereg['alpha'], boundary='ville', futility=False,
                                 n_max=prereg['sprt_n_max'])


def load(paths, strict=True):
    """여러 결과 CSV 를 읽어 합친다."""
    rows = []
    for p in paths:
        rows += schema.read_csv(p, strict=strict)
    return rows


def select(rows, **kw):
    """열 값으로 거른다 (값이 튜플이면 그중 하나)."""
    out = []
    for r in rows:
        ok = True
        for k, v in kw.items():
            if isinstance(v, tuple):
                ok = ok and r.get(k) in v
            else:
                ok = ok and r.get(k) == v
        if ok:
            out.append(r)
    return out


def _vals(rows, col, only=None):
    v = [r[col] for r in rows if r.get(col) is not None and (only is None or only(r))]
    return np.array(v, dtype=float)


def _iqm(a):
    a = np.asarray(a, dtype=float)
    a = a[np.isfinite(a)]
    return aggregate.iqm(a) if a.size else math.nan


# ---------------------------------------------------------------- 1. 비교군 요약

def arm_summary(rows):
    """(arm, simulator, safety_node) 별 기술 통계 — 성공·접촉·시간·이격·개입·명세 점검."""
    groups = defaultdict(list)
    for r in rows:
        groups[(r['arm'], r['simulator'], r['safety_node'])].append(r)
    out = []
    for (arm, sim, sn), g in sorted(groups.items()):
        n = len(g)
        n_err = sum(r['outcome'] == 'error' for r in g)
        valid = [r for r in g if r['outcome'] != 'error']
        x_s = sum(schema.success(r) for r in g)
        x_c = sum(schema.collided(r) for r in valid)
        lo, hi = binomial.cp_interval(x_s, n) if n else (math.nan, math.nan)
        t_succ = _vals(g, 'time_s', schema.success)
        iv = _vals(g, 'interventions')
        out.append({
            'arm': arm, 'simulator': sim, 'safety_node': sn, 'n': n, 'errors': n_err,
            'error_rate': n_err / n if n else math.nan,
            'policies': len({r['policy_id'] for r in g}),
            'seeds': len({r['train_seed'] for r in g if r['train_seed'] is not None}),
            'success': x_s, 'success_rate': x_s / n if n else math.nan,
            'success_lo': lo, 'success_hi': hi,
            'collisions': x_c, 'n_valid': len(valid),
            'collision_cp_upper': binomial.cp_upper(x_c, len(valid)) if valid else math.nan,
            'timeouts': sum(r['outcome'] in ('timeout', 'stall') for r in g),
            'time_median': float(np.median(t_succ)) if t_succ.size else math.nan,
            'time_iqm': _iqm(t_succ),
            'min_clearance_p5': (float(np.percentile(_vals(g, 'min_clearance_m'), 5))
                                 if _vals(g, 'min_clearance_m').size else math.nan),
            'interventions_mean': float(iv.mean()) if iv.size else math.nan,
            'interventions_n': int(iv.size),
            'cte_mean': float(np.mean(_vals(g, 'cte_mean_m'))) if _vals(
                g, 'cte_mean_m').size else math.nan,
            'dev_over': int((_vals(g, 'max_dev_m') > PREREG['spec_dev_m']).sum()),
            'return_over': int((_vals(g, 'return_max_s') > PREREG['spec_return_s']).sum()),
        })
    return out


# ---------------------------------------------------------------- 2. 시드 집계 (rliable)

def seed_matrix(rows, metric='success'):
    """
    (시드 × 난도층) 점수 행렬 — 칸 = 그 시드·층의 평균 (success 는 성공률).

    시드가 없는 비교군(DWA)은 정책 하나를 run 하나로 본다. 빈 칸이 있으면 그 층을 뺀다.
    """
    runs = sorted({(r['train_seed'] if r['train_seed'] is not None else -1) for r in rows})
    tasks = sorted({r['difficulty'] for r in rows})
    cell = defaultdict(list)
    for r in rows:
        s = r['train_seed'] if r['train_seed'] is not None else -1
        v = 1.0 if (metric == 'success' and schema.success(r)) else (
            0.0 if metric == 'success' else r.get(metric))
        if v is not None:
            cell[(s, r['difficulty'])].append(float(v))
    keep = [t for t in tasks if all(cell.get((s, t)) for s in runs)]
    mat = np.array([[np.mean(cell[(s, t)]) for t in keep] for s in runs], dtype=float)
    return mat, runs, keep


def seed_aggregate(rows, simulator, reps=2000, seed=0, backend='auto', prereg=PREREG):
    """비교군별 성공률 IQM + 층화 부트스트랩 CI, 기준 대비 개선 확률."""
    sub = select(rows, simulator=simulator, safety_node='on')
    arms = sorted({r['arm'] for r in sub})
    mats = {a: seed_matrix(select(sub, arm=a)) for a in arms}
    common = None
    for a in arms:
        t = set(mats[a][2])
        common = t if common is None else common & t
    common = sorted(common or [])
    out = []
    if not common:
        return out
    trimmed = {}
    for a in arms:
        mat, runs, tasks = mats[a]
        idx = [tasks.index(t) for t in common]
        trimmed[a] = mat[:, idx]
    ci = aggregate.interval_estimates(trimmed, aggregate.iqm, reps=reps, seed=seed,
                                      backend=backend)
    ref = prereg['reference_arm']
    for a in arms:
        n_runs = trimmed[a].shape[0]
        row = {'arm': a, 'simulator': simulator, 'runs': n_runs, 'tasks': common,
               'iqm': ci[a][0], 'lo': ci[a][1], 'hi': ci[a][2]}
        if a != ref and ref in trimmed:
            row['poi_vs_ref'] = aggregate.probability_of_improvement(trimmed[a], trimmed[ref])
        if a != ref and n_runs < prereg['min_seeds']:
            row['note'] = f'시드 {n_runs} < {prereg["min_seeds"]} — {UNDECIDABLE}'
        out.append(row)
    return out


# ---------------------------------------------------------------- 3. 짝짓기

def pair_rows(rows_t, rows_r):
    """(scenario_set, scenario_id, scenario_seed, episode) 로 짝짓는다 → [(t, r), ...]."""
    def key(r):
        return (r['scenario_set'], r['scenario_id'], r['scenario_seed'], r['episode'])
    ref = {}
    for r in rows_r:
        ref.setdefault(key(r), r)
    return [(t, ref[key(t)]) for t in rows_t if key(t) in ref]


def _exec_order(rows):
    if all(r.get('exec_index') is not None for r in rows):
        return sorted(rows, key=lambda r: r['exec_index']), True
    return list(rows), False


# ---------------------------------------------------------------- 4. 확증 (고정 순서)

def confirmatory(rows, prereg=PREREG, primary_policy=None, arm=None):
    """
    주 비교군 C 대 A — Gazebo 새 시드 묶음에서 고정 순서 H1~H5.

    반환 dict: policy, n, pairs, exec_order_given, hypotheses(list), sprt(state).
    """
    arm = arm or prereg['primary_arm']
    ref_arm = prereg['reference_arm']
    base = select(rows, simulator=prereg['confirm_simulator'],
                  scenario_set=prereg['confirm_set'], safety_node='on')
    t_rows = select(base, arm=arm)
    r_rows = select(base, arm=ref_arm)
    res = {'arm': arm, 'reference': ref_arm, 'hypotheses': []}
    hyps = res['hypotheses']
    names = ['H1 접촉률 NI δ=2%p (Ville SPRT)', 'H2 성공률 NI δ=3%p (Tango 짝)',
             'H3 성공률 우월 (McNemar mid-p)', 'H4 복귀 시간 우월 (Wilcoxon)',
             'H5 접촉률 NI δ=1%p (FM·Newcombe, 보조)']
    policies = sorted({r['policy_id'] for r in t_rows})
    if primary_policy is None and len(policies) == 1:
        primary_policy = policies[0]
    res['policy'] = primary_policy
    if not t_rows or primary_policy is None:
        why = '결과 없음' if not t_rows else f'주 체크포인트가 여러 개 {policies} — 사전 지정 필요'
        for nm in names:
            hyps.append({'name': nm, 'verdict': UNDECIDABLE, 'detail': why})
        return res
    t_rows = select(t_rows, policy_id=primary_policy)
    n_err = sum(r['outcome'] == 'error' for r in t_rows)
    res['errors'] = n_err
    if t_rows and n_err / len(t_rows) > prereg['error_rate_max']:
        for nm in names:
            hyps.append({'name': nm, 'verdict': UNDECIDABLE,
                         'detail': f'오류율 {n_err}/{len(t_rows)} > {prereg["error_rate_max"]:.0%}'})
        return res
    ordered, given = _exec_order([r for r in t_rows if r['outcome'] != 'error'])
    res['exec_order_given'] = given
    design = sprt_design(prereg)
    st = sequential.run_sprt([schema.collided(r) for r in ordered], design)
    res['sprt'] = st
    res['n'] = len(ordered)
    # H1
    if not given:
        v1, d1 = UNDECIDABLE, 'exec_index 없음 — 순차 감시 순서를 알 수 없다'
    elif st.decision == 'ni':
        v1, d1 = MET, f'n={st.n} 에서 NI 선언 (접촉 {st.failures})'
    elif st.decision == 'continue':
        v1, d1 = UNDECIDABLE, (f'진행 중 n={st.n}, 접촉 {st.failures}, Λ={st.llr:.3f} '
                               f'< {design.upper:.3f}')
    else:
        v1, d1 = NOT_MET, f'상한 {design.n_max} 도달, 접촉 {st.failures} — NI 미입증'
    verdicts = [(names[0], v1, d1)]
    # 짝
    pairs = pair_rows([r for r in t_rows if r['outcome'] != 'error'],
                      [r for r in r_rows if r['outcome'] != 'error'])
    res['pairs'] = len(pairs)
    a = prereg['alpha']
    zc = ni.z_quantile(1 - a)
    if pairs:
        fail_t = [not schema.success(t) for t, _ in pairs]
        fail_r = [not schema.success(r) for _, r in pairs]
        _, n10, n01, _ = ni.paired_table(fail_t, fail_r)
        z2 = ni.tango_z(n10, n01, len(pairs), prereg['delta_success'])
        verdicts.append((names[1], MET if z2 < -zc else NOT_MET,
                         f'쌍 {len(pairs)}, C만 실패 {n10}, A만 실패 {n01}, z={z2:.3f}'))
        p3 = paired.mcnemar_midp(n01, n10, 'greater')
        verdicts.append((names[2], MET if p3 <= a else NOT_MET,
                         f'C만 성공 {n01}, A만 성공 {n10}, 단측 mid-p={p3:.4f}'))
        rt = np.array([t['return_max_s'] if t['return_max_s'] is not None else np.nan
                       for t, _ in pairs], dtype=float)
        rr = np.array([r['return_max_s'] if r['return_max_s'] is not None else np.nan
                       for _, r in pairs], dtype=float)
        if np.isfinite(rt).sum() == 0 or np.isfinite(rr).sum() == 0:
            verdicts.append((names[3], UNMEASURED, 'return_max_s 비어 있음'))
        else:
            w = paired.wilcoxon_paired(rt, rr, alternative='less')
            verdicts.append((names[3], MET if w['p_value'] <= a else NOT_MET,
                             f'쌍 {w["n_pairs"]}, HL 차 {w["hl"]:.3f} s, p={w["p_value"]:.4f}'))
    else:
        for nm in names[1:4]:
            verdicts.append((nm, UNDECIDABLE, '같은 시드의 DWA 짝 없음'))
    # H5 (고정 n, 비짝지은)
    x_t = sum(schema.collided(r) for r in ordered)
    r_valid = [r for r in r_rows if r['outcome'] != 'error']
    if r_valid:
        x_r = sum(schema.collided(r) for r in r_valid)
        t5 = ni.ni_test(x_t, len(ordered), x_r, len(r_valid), prereg['delta_collision_aux'], a)
        verdicts.append((names[4], MET if t5['ni'] else NOT_MET,
                         f'C {x_t}/{len(ordered)}, A {x_r}/{len(r_valid)}, FM z={t5["z"]:.3f}, '
                         f'Newcombe 상한 {100 * t5["newcombe_upper"]:.2f} %'))
    else:
        verdicts.append((names[4], UNDECIDABLE, 'DWA 결과 없음'))
    go = True
    for nm, v, d in verdicts:
        if go:
            hyps.append({'name': nm, 'verdict': v, 'detail': d})
            go = v == MET
        else:
            hyps.append({'name': nm, 'verdict': SKIPPED, 'detail': f'(참고) {v}: {d}'})
    return res


def secondary(rows, prereg=PREREG):
    """보조 비교군 (B·D) — 탐색적: SPRT 상태와 FM NI p 의 Holm 보정 (확증 아님)."""
    base = select(rows, simulator=prereg['confirm_simulator'],
                  scenario_set=prereg['confirm_set'], safety_node='on')
    ref = [r for r in select(base, arm=prereg['reference_arm']) if r['outcome'] != 'error']
    out, pvals = [], []
    for a in prereg['secondary_arms']:
        t = [r for r in select(base, arm=a) if r['outcome'] != 'error']
        if not t:
            continue
        ordered, _ = _exec_order(t)
        st = sequential.run_sprt([schema.collided(r) for r in ordered], sprt_design(prereg))
        x_t = sum(schema.collided(r) for r in t)
        p = math.nan
        if ref:
            x_r = sum(schema.collided(r) for r in ref)
            p = ni.ni_test(x_t, len(t), x_r, len(ref), prereg['delta_collision'],
                           prereg['alpha'])['p_value']
        out.append({'arm': a, 'n': len(t), 'collisions': x_t, 'sprt': st.decision,
                    'sprt_n': st.n, 'fm_p': p})
        pvals.append(p)
    if out and all(not math.isnan(p) for p in pvals):
        rej, adj = multiplicity.holm(pvals, prereg['alpha'])
        for o, r_, a_ in zip(out, rej, adj):
            o['holm_p'] = float(a_)
            o['holm_ni'] = bool(r_)
    return out


# ---------------------------------------------------------------- 5. 2D 진출 게이트

def gate_2d(rows, prereg=PREREG):
    """
    Gazebo 진출 게이트 — 2D 충돌 CP 95 % 상한 ≤ 0.5 % 그리고 2D-DWA 대비 성공률 NI δ=2%p (Tango 짝).

    체크포인트(policy_id) 마다 판정한다.
    """
    sub = select(rows, simulator='sim2d', safety_node='on')
    ref = [r for r in select(sub, arm=prereg['reference_arm']) if r['outcome'] != 'error']
    out = []
    for pol in sorted({r['policy_id'] for r in sub if r['arm'] != prereg['reference_arm']}):
        t = [r for r in select(sub, policy_id=pol) if r['outcome'] != 'error']
        x_c = sum(schema.collided(r) for r in t)
        ub = binomial.cp_upper(x_c, len(t))
        pairs = pair_rows(t, ref)
        if pairs:
            _, n10, n01, _ = ni.paired_table([not schema.success(a) for a, _ in pairs],
                                             [not schema.success(b) for _, b in pairs])
            z = ni.tango_z(n10, n01, len(pairs), prereg['gate_success_delta'])
            ni_ok = z < -ni.z_quantile(1 - prereg['alpha'])
        else:
            z, ni_ok = math.nan, False
        c_ok = ub <= prereg['gate_collision_cp_upper']
        verdict = MET if (c_ok and ni_ok) else (UNDECIDABLE if not pairs else NOT_MET)
        out.append({'policy_id': pol, 'arm': t[0]['arm'] if t else '', 'n': len(t),
                    'collisions': x_c, 'cp_upper': ub, 'pairs': len(pairs), 'tango_z': z,
                    'verdict': verdict})
    return out


# ---------------------------------------------------------------- 6. CVaR·SRCC·명세

def cvar_table(rows, simulator='sim2d', prereg=PREREG, seed=0):
    """비교군별 최악 α % 레벨 성공률 (plug-in·cross-fit). 레벨 = scenario_id."""
    sub = select(rows, simulator=simulator, safety_node='on')
    out = []
    for a in sorted({r['arm'] for r in sub}):
        lv = defaultdict(list)
        for r in select(sub, arm=a):
            if r['outcome'] != 'error':
                lv[r['scenario_id']].append(1.0 if schema.success(r) else 0.0)
        levels = [v for v in lv.values() if len(v) >= 2]
        if len(levels) < 2:
            continue
        for al in prereg['cvar_alphas']:
            b = cvar.cvar_both(levels, al, seed)
            b['arm'] = a
            b['enough'] = b['episodes_per_level_min'] >= prereg['cvar_min_episodes_per_level']
            out.append(b)
    return out


def srcc_table(rows, prereg=PREREG, reps=1000, seed=0):
    """시뮬레이터 쌍별 정책 순위 상관 — 공통 scenario_id 의 정책별 성공률."""
    sub = select(rows, safety_node='on')
    sims = sorted({r['simulator'] for r in sub})
    out = []
    for i, sa in enumerate(sims):
        for sb in sims[i + 1:]:
            pol_a = defaultdict(lambda: defaultdict(list))
            pol_b = defaultdict(lambda: defaultdict(list))
            for r in select(sub, simulator=sa):
                pol_a[r['policy_id']][r['scenario_id']].append(float(schema.success(r)))
            for r in select(sub, simulator=sb):
                pol_b[r['policy_id']][r['scenario_id']].append(float(schema.success(r)))
            pols = sorted(set(pol_a) & set(pol_b))
            if len(pols) < 3:
                continue
            scen = sorted(set.intersection(*[set(pol_a[p]) & set(pol_b[p]) for p in pols]))
            if not scen:
                continue
            ma = np.array([[np.mean(pol_a[p][s]) for s in scen] for p in pols])
            mb = np.array([[np.mean(pol_b[p][s]) for s in scen] for p in pols])
            rep = srcc.srcc_report(ma.mean(1), mb.mean(1))
            bs = srcc.scenario_bootstrap(ma, mb, reps=reps, seed=seed)
            rep.update(sim_a=sa, sim_b=sb, n_scenarios=len(scen), boot_lo=bs['lo'],
                       boot_hi=bs['hi'], policies=pols,
                       verdict=(MET if rep['n_policies'] >= prereg['srcc_min_policies']
                                else UNDECIDABLE))
            rep['scores_a'] = ma.mean(1).tolist()
            rep['scores_b'] = mb.mean(1).tolist()
            out.append(rep)
    return out


def spec_checks(rows, prereg=PREREG):
    """
    명세 4.7 점검 (Gazebo, 비교군별) — 30 회 묶음 0 접촉·이탈 ≤ 1 m·복귀 ≤ 5 s.

    묶음 = exec_index(없으면 행 순서) 순서로 자른 30 회. 판정 문장의 OC(충돌률 2 % 도 54.5 % 통과)를 함께 싣는다.
    """
    out = []
    sub = select(rows, simulator='gazebo', safety_node='on')
    for a in sorted({r['arm'] for r in sub}):
        g, _ = _exec_order([r for r in select(sub, arm=a) if r['outcome'] != 'error'])
        k = prereg['spec_block']
        blocks = [g[i:i + k] for i in range(0, len(g) - k + 1, k)]
        clean = sum(all(not schema.collided(r) for r in b) for b in blocks)
        dev = _vals(g, 'max_dev_m')
        ret = _vals(g, 'return_max_s')
        out.append({
            'arm': a, 'blocks': len(blocks), 'blocks_zero_contact': clean,
            'contact_verdict': (UNDECIDABLE if not blocks else
                                MET if clean == len(blocks) else NOT_MET),
            'dev_max': float(dev.max()) if dev.size else math.nan,
            'dev_verdict': (UNMEASURED if not dev.size else
                            MET if dev.max() <= prereg['spec_dev_m'] else NOT_MET),
            'return_max': float(ret.max()) if ret.size else math.nan,
            'return_verdict': (UNMEASURED if not ret.size else
                               MET if ret.max() <= prereg['spec_return_s'] else NOT_MET),
            'oc_pass_at_2pct': binomial.pass_probability(0.02, k),
        })
    return out


def descriptive_pairs(rows, simulator='gazebo', prereg=PREREG):
    """비교군별 기준 대비 짝 차이 (Hodges–Lehmann + 구간) — 주행 시간·CTE·이격·개입."""
    sub = select(rows, simulator=simulator, safety_node='on')
    ref = [r for r in select(sub, arm=prereg['reference_arm']) if r['outcome'] != 'error']
    out = []
    for a in sorted({r['arm'] for r in sub} - {prereg['reference_arm']}):
        t = [r for r in select(sub, arm=a) if r['outcome'] != 'error']
        pairs = pair_rows(t, ref)
        for col in ('time_s', 'cte_mean_m', 'min_clearance_m', 'interventions', 'jerk_lin_rms'):
            ok = [(x[col], y[col]) for x, y in pairs if x[col] is not None and y[col] is not None
                  and math.isfinite(x[col]) and math.isfinite(y[col])]
            if col == 'time_s':
                ok = [(x['time_s'], y['time_s']) for x, y in pairs
                      if schema.success(x) and schema.success(y)]
            if len(ok) < 6:
                out.append({'arm': a, 'metric': col, 'pairs': len(ok), 'verdict': UNMEASURED
                            if not ok else UNDECIDABLE})
                continue
            d = np.array([x - y for x, y in ok])
            hl, lo, hi = paired.hodges_lehmann(d)
            out.append({'arm': a, 'metric': col, 'pairs': len(ok), 'hl': hl, 'lo': lo, 'hi': hi,
                        'verdict': '기술'})
    return out


def evaluate(rows, prereg=PREREG, primary_policy=None, reps=2000, backend='auto'):
    """전체 판정 묶음."""
    sims = sorted({r['simulator'] for r in rows})
    return {
        'n_rows': len(rows),
        'summary': arm_summary(rows),
        # Gazebo 는 배포 체크포인트 하나를 평가한다 — 시드 집계는 2D·Isaac 만 (PROTOCOL.md §6.3)
        'seed_aggregate': {s: seed_aggregate(rows, s, reps=reps, backend=backend, prereg=prereg)
                           for s in sims if s != 'gazebo'},
        'gate_2d': gate_2d(rows, prereg),
        'confirmatory': confirmatory(rows, prereg, primary_policy),
        'secondary': secondary(rows, prereg),
        'cvar': cvar_table(rows, 'sim2d', prereg),
        'srcc': srcc_table(rows, prereg),
        'spec': spec_checks(rows, prereg),
        'pairs_gazebo': descriptive_pairs(rows, 'gazebo', prereg),
        'prereg': prereg,
    }


# ---------------------------------------------------------------- 마크다운

def _p(x, nd=2, pct=False):
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return '—'
    return f'{100 * x:.{nd}f} %' if pct else f'{x:.{nd}f}'


def render_markdown(res):
    """evaluate() 결과 → 마크다운 판정표."""
    L = []
    w = L.append
    c = res['confirmatory']
    w('# 보너스 1 — 학습 정책 대 DWA 판정표')
    w('')
    w(f'행 {res["n_rows"]}. 판정 낱말: {MET} · {NOT_MET} · {UNDECIDABLE} · {UNMEASURED} · {SKIPPED}. '
      '사전 등록: rl/eval/PROTOCOL.md §6.')
    w('')
    w(f'## 1. 확증 — {c["arm"]} 대 {c["reference"]} (Gazebo `{PREREG["confirm_set"]}`, 고정 순서)')
    w('')
    w(f'주 체크포인트: `{c.get("policy")}` · 유효 에피소드 {c.get("n", 0)} · 짝 {c.get("pairs", 0)}')
    w('')
    w('| 가설 | 판정 | 근거 |')
    w('| --- | --- | --- |')
    for h in c['hypotheses']:
        w(f'| {h["name"]} | **{h["verdict"]}** | {h["detail"]} |')
    w('')
    w('## 2. 비교군 요약')
    w('')
    w('| 비교군 | 시뮬레이터 | safety | n | 오류 | 정책/시드 | 성공률 [CP 95 %] | 접촉 (CP 상한) | '
      '정체·시간초과 | 시간 중앙/IQM [s] | 이격 p5 [m] | 개입/에피소드 | 이탈>1m | 복귀>5s |')
    w('| --- | --- | --- | ---: | ---: | --- | --- | --- | ---: | --- | ---: | ---: | ---: '
      '| ---: |')
    for s in res['summary']:
        iv = _p(s['interventions_mean']) if s['interventions_n'] else UNMEASURED
        w(f'| {s["arm"]} | {s["simulator"]} | {s["safety_node"]} | {s["n"]} | {s["errors"]} | '
          f'{s["policies"]}/{s["seeds"]} | {_p(s["success_rate"], 1, True)} '
          f'[{_p(s["success_lo"], 1, True)}, {_p(s["success_hi"], 1, True)}] | '
          f'{s["collisions"]}/{s["n_valid"]} ({_p(s["collision_cp_upper"], 2, True)}) | '
          f'{s["timeouts"]} | {_p(s["time_median"])}/{_p(s["time_iqm"])} | '
          f'{_p(s["min_clearance_p5"], 3)} | {iv} | {s["dev_over"]} | {s["return_over"]} |')
    w('')
    w('## 3. 시드 집계 — 성공률 IQM (층화 부트스트랩 95 %)')
    w('')
    w('| 시뮬레이터 | 비교군 | run | 층 | IQM [95 % CI] | P(개선 > DWA) | 비고 |')
    w('| --- | --- | ---: | --- | --- | ---: | --- |')
    for sim, rows in res['seed_aggregate'].items():
        for r in rows:
            w(f'| {sim} | {r["arm"]} | {r["runs"]} | {len(r["tasks"])} | {_p(r["iqm"], 3)} '
              f'[{_p(r["lo"], 3)}, {_p(r["hi"], 3)}] | {_p(r.get("poi_vs_ref"), 3)} | '
              f'{r.get("note", "")} |')
    w('')
    w('## 4. 2D → Gazebo 진출 게이트')
    w('')
    w('| 체크포인트 | n | 접촉 | CP 95 % 상한 (≤ 0.5 %) | 짝 | Tango z (δ=2%p) | 판정 |')
    w('| --- | ---: | ---: | --- | ---: | ---: | --- |')
    for g in res['gate_2d']:
        w(f'| `{g["policy_id"]}` | {g["n"]} | {g["collisions"]} | {_p(g["cp_upper"], 3, True)} | '
          f'{g["pairs"]} | {_p(g["tango_z"])} | **{g["verdict"]}** |')
    w('')
    w('## 5. 보조 비교군 (탐색적 — 확증 아님)')
    w('')
    w('| 비교군 | n | 접촉 | SPRT 상태 (n) | FM NI p | Holm 보정 p |')
    w('| --- | ---: | ---: | --- | ---: | ---: |')
    for s in res['secondary']:
        w(f'| {s["arm"]} | {s["n"]} | {s["collisions"]} | {s["sprt"]} ({s["sprt_n"]}) | '
          f'{_p(s["fm_p"], 4)} | {_p(s.get("holm_p"), 4)} |')
    w('')
    w('## 6. 최악 난도 CVaR (2D, 레벨 = scenario_id)')
    w('')
    w('| 비교군 | α | 레벨 | 최악 k | plug-in (비관) | cross-fit (낙관) | 레벨당 최소 에피소드 | 충분 |')
    w('| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |')
    for v in res['cvar']:
        w(f'| {v["arm"]} | {v["alpha"]:.2f} | {v["n_levels"]} | {v["k"]} | {_p(v["plugin"], 3)} | '
          f'{_p(v["crossfit"], 3)} | {v["episodes_per_level_min"]} | '
          f'{"예" if v["enough"] else UNDECIDABLE} |')
    w('')
    w('## 7. 시뮬레이터 간 순위 상관 (SRCC)')
    w('')
    w('| 쌍 | 정책 N | 공통 시나리오 | ρ | τ | BW 95 % | 시나리오 부트스트랩 95 % | 순열 p | 판정 |')
    w('| --- | ---: | ---: | ---: | ---: | --- | --- | ---: | --- |')
    for s in res['srcc']:
        w(f'| {s["sim_a"]}–{s["sim_b"]} | {s["n_policies"]} | {s["n_scenarios"]} | '
          f'{_p(s["rho"], 3)} | {_p(s["tau"], 3)} | [{_p(s["bw_lo"])}, {_p(s["bw_hi"])}] | '
          f'[{_p(s["boot_lo"])}, {_p(s["boot_hi"])}] | {_p(s["perm_p"], 4)} | '
          f'**{s["verdict"]}** |')
    w('')
    w('같은 비교군의 시드들은 독립 정책이 아니다 — 구간은 낙관적이다. N < 10 이면 판정 불가로 둔다.')
    w('')
    w('## 8. 명세 4.7 점검 (Gazebo)')
    w('')
    w('| 비교군 | 30 회 묶음 | 0 접촉 묶음 | 판정 | 최대 이탈 [m] | 판정 | 최대 복귀 [s] | 판정 |')
    w('| --- | ---: | ---: | --- | ---: | --- | ---: | --- |')
    for s in res['spec']:
        w(f'| {s["arm"]} | {s["blocks"]} | {s["blocks_zero_contact"]} | {s["contact_verdict"]} | '
          f'{_p(s["dev_max"], 3)} | {s["dev_verdict"]} | {_p(s["return_max"])} | '
          f'{s["return_verdict"]} |')
    if res['spec']:
        w('')
        w(f'"30 회 0 접촉" 은 판정 기준이지 안전 근거가 아니다 — 참 접촉률 2 % 정책도 '
          f'{100 * res["spec"][0]["oc_pass_at_2pct"]:.1f} % 확률로 통과한다.')
    w('')
    w('## 9. Gazebo 짝 차이 (기술 통계, 시험 − DWA, Hodges–Lehmann 95 %)')
    w('')
    w('| 비교군 | 지표 | 짝 | HL [95 %] | 비고 |')
    w('| --- | --- | ---: | --- | --- |')
    for d in res['pairs_gazebo']:
        if 'hl' in d:
            w(f'| {d["arm"]} | {d["metric"]} | {d["pairs"]} | {_p(d["hl"], 3)} '
              f'[{_p(d["lo"], 3)}, {_p(d["hi"], 3)}] | |')
        else:
            w(f'| {d["arm"]} | {d["metric"]} | {d["pairs"]} | — | {d["verdict"]} |')
    w('')
    return '\n'.join(L) + '\n'
