#!/usr/bin/env python3
"""
2 라운드 분석 (prereg_r2.md §2·§5). 1 라운드 analyze_x.py 의 식·표를 그대로 불러 쓰고 J''·S'' 선택과 H4/H5/H6 만 더한다.
**이 파일은 튜닝·held-out C 결과를 보기 전에 커밋한다.**

    python3 analyze_r2.py select --tune /runs/r2/tuning/raw          # → selection_r2.json
    python3 analyze_r2.py equiv --raw /runs/r2/equiv/raw               # X03(dwa_R) 대 Y00 (§4)
    python3 analyze_r2.py tables --runs /runs --out <dir>              # 표·판정

## 선택 규칙 J'' (prereg_r2 §2)
- 튜닝-20 (20 경로) 에서 조건 c ∈ {잡음 없음 0, seed 1, 2, 3} 마다 J_c·S_c 를 1 라운드와 같은 식으로 낸다
  (J_c = 경로 평균 [CTE 최대/10 cm + |정지 오차|/10 cm + T/T_ref], 실패 경로 10; S_c = 성공 경로 수).
- 튜닝 O 6 장면 (1 회): J_O = 장면 평균 [실패 = 10; 성공 = T/T_ref + (장애물 최소 이격 < 5 cm 이면 1)],
  S_O = 성공 장면 수.
- S'' = S_0 + mean(S_1..3) + S_O,  J'' = J_0 + mean(J_1..3) + J_O.
- 순서: S'' 최대 → J'' 최소 → 이름 순. 격자 다섯 (DWB_·MPPI_·RPP_·dwa_textbook_T·dwa_y_) 모두 같은 함수.
- 조건 하나라도 기록이 모자라면 (20 경로 × 4 조건 + O 6) 'incomplete' 로 순위에서 뺀다.

## 확증 가설 (held-out C, prereg_r2 §5)
- H4 (시간 비열등, 먼저): ROS C1, 두 팔 모두 성공한 경로의 mean log(T_dwa_R2 / T_RPP선택). 95 % 상한 < log(1.005).
- H5 (시간 우위, H4 확증일 때만 판정): 같은 통계량, 95 % 상한 < 0. H4 가 확증 안 되면 '판정 안 함'.
- H6 (장애물 우회, 독립): O 에서 장면마다 반복 평균 성공률 (dwa_R2 − dwa_full) 의 평균. 장면 묶음 부트스트랩 95 % 하한 > 0.
부트스트랩: B = 10 000, seed 12345, 백분위 (1 라운드와 같음).
"""
import argparse
import collections
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze_x as ax  # noqa: E402

FAMILY_PREFIX = {'DWB': 'DWB_', 'MPPI': 'MPPI_', 'RPP': 'RPP_', 'TB': 'dwa_textbook_T',
                 'DWA_R2': 'dwa_y_'}
SETTINGS = (0, 1, 2, 3)
N_TUNE_PATHS = 20
N_TUNE_O = 6
TABLE_ARMS = ['dwa_full', 'dwa_R', 'dwa_R2', 'dwa_textbook_tuned', 'DWB_D00', 'DWB_tuned',
              'MPPI_P00', 'MPPI_tuned', 'RPP_R00', 'RPP_tuned', 'pp']
LOG_1005 = math.log(1.005)


def j_o(recs):
    """튜닝 O: (성공 장면 수, J_O)."""
    terms = []
    ok = 0
    for r in recs:
        if r['success']:
            ok += 1
            pen = 1.0 if r.get('obs_min_clear', 1.0) < 0.05 else 0.0
            terms.append(r['t_reach'] / ax.t_lb(r['length']) + pen)
        else:
            terms.append(10.0)
    return ok, float(np.mean(terms)) if terms else 10.0


def select(tune_raw):
    recs = ax.load_dir(tune_raw)
    path_recs = [r for r in recs if r['family'] in ax.HELDOUT]
    o_recs = [r for r in recs if r['family'] == 'obstacle']
    by = collections.defaultdict(lambda: collections.defaultdict(list))
    for r in path_recs:
        by[r['arm']][int(r.get('seed', 0))].append(r)
    byo = collections.defaultdict(list)
    for r in o_recs:
        byo[r['arm']].append(r)
    out = {'rule': "S'' = S_0 + mean(S_1..3) + S_O 최대 → J'' = J_0 + mean(J_1..3) + J_O 최소 → 이름 순",
           'n_records': len(recs)}
    for fam, prefix in FAMILY_PREFIX.items():
        cands = []
        for arm in sorted(set(by) | set(byo)):
            if not arm.startswith(prefix):
                continue
            per = {}
            complete = True
            for s in SETTINGS:
                rs = by[arm].get(s, [])
                if len({r['scene'] for r in rs}) < N_TUNE_PATHS:
                    complete = False
                per[s] = ax.tuning_score(rs) if rs else (0, 10.0)
            so, jo = j_o(byo.get(arm, []))
            if len({r['scene'] for r in byo.get(arm, [])}) < N_TUNE_O:
                complete = False
            s2 = per[0][0] + float(np.mean([per[s][0] for s in (1, 2, 3)])) + so
            j2 = per[0][1] + float(np.mean([per[s][1] for s in (1, 2, 3)])) + jo
            cands.append({'arm': arm, 'S2': s2, 'J2': j2, 'complete': complete,
                          'per_setting': {str(s): {'success': per[s][0], 'J': per[s][1]}
                                          for s in SETTINGS},
                          'O': {'success': so, 'J_O': jo}})
        ranked = sorted([c for c in cands if c['complete']], key=lambda c: (-c['S2'], c['J2'], c['arm']))
        out[fam] = {'chosen': ranked[0]['arm'] if ranked else None,
                    'ranking': ranked + [c for c in cands if not c['complete']]}
    if out['DWA_R2']['chosen']:
        out['DWA_R2']['chosen_y'] = out['DWA_R2']['chosen'].replace('dwa_y_', '')
    if out['TB']['chosen']:
        out['TB']['chosen_t'] = out['TB']['chosen'].replace('dwa_textbook_', '')
    return out


# ---------------------------------------------------------------------------------------------
# 확증 가설

def h4h5(c1, rpp_sel):
    d = ax.paired(c1, 'dwa_R2', rpp_sel, 't_reach', ax.logratio)
    m, lo, hi = ax.boot(d.values())
    k2, n2, _ = ax.success_rate(c1, 'dwa_R2')
    kr, nr, _ = ax.success_rate(c1, rpp_sel)
    h4 = bool(d) and hi < LOG_1005
    base = {'statistic': f'mean log(T_dwa_R2 / T_{rpp_sel}), 두 팔 모두 성공한 경로',
            'condition': 'ROS C1: held-out C 60, 잡음 없음, stateful false',
            'n_paths': len(d), 'mean': m, 'ci95': [lo, hi],
            'time_ratio_pct': [100 * (math.exp(x) - 1) for x in (m, lo, hi)] if d else None,
            'success_R2_RPP': [k2, n2, kr, nr]}
    out4 = dict(base, hypothesis='H4', rule='95 % 상한 < log(1.005)',
                decision='confirmed' if h4 else 'not confirmed')
    if h4:
        out5 = dict(base, hypothesis='H5', rule='H4 확증 뒤, 95 % 상한 < 0',
                    decision='confirmed' if hi < 0 else 'not confirmed')
    else:
        out5 = dict(base, hypothesis='H5', rule='H4 확증 뒤, 95 % 상한 < 0',
                    decision='not tested (H4 not confirmed)')
    return out4, out5


def h6(o):
    sc = collections.defaultdict(lambda: collections.defaultdict(list))
    for r in o:
        if r['arm'] in ('dwa_R2', 'dwa_full'):
            sc[r['scene']][r['arm']].append(r['success'])
    d = [float(np.mean(v['dwa_R2'])) - float(np.mean(v['dwa_full']))
         for v in sc.values() if v['dwa_R2'] and v['dwa_full']]
    m, lo, hi = ax.boot(d)
    sr = sum(r['success'] for r in o if r['arm'] == 'dwa_R2')
    nr = sum(1 for r in o if r['arm'] == 'dwa_R2')
    sf = sum(r['success'] for r in o if r['arm'] == 'dwa_full')
    nf = sum(1 for r in o if r['arm'] == 'dwa_full')
    return {'hypothesis': 'H6', 'statistic': '장면별 반복 평균 성공률 차 (dwa_R2 − dwa_full) 의 평균',
            'condition': 'ROS O: held-out C 매끈 앞 20 + 원판, 반복 3, 참값 트랙 차단',
            'n_scenes': len(d), 'mean': m, 'ci95': [lo, hi], 'rule': '장면 묶음 95 % 하한 > 0',
            'decision': 'confirmed' if d and lo > 0 else 'not confirmed',
            'runs_success_R2_full': [sr, nr, sf, nf]}


# ---------------------------------------------------------------------------------------------

def tables(args):
    R = args.runs
    with open(os.path.join(R, 'r2', 'tuning', 'selection_r2.json'), encoding='utf-8') as f:
        sel = json.load(f)
    rpp_sel = sel['RPP']['chosen']
    st = lambda s: os.path.join(R, 'r2', s, 'raw')   # noqa: E731
    c1 = ax.substitute_tuned(ax.load_dir(st('c1')), sel)
    c2 = ax.substitute_tuned(ax.load_dir(st('c2')), sel)
    c3 = ax.substitute_tuned(ax.load_dir(st('c3')), sel)
    o = ax.substitute_tuned(ax.load_dir(st('o')), sel)
    chain = ax.load_chain(os.path.join(R, 'r2', 'chain', 'raw')) \
        if os.path.isdir(os.path.join(R, 'r2', 'chain', 'raw')) else []
    os.makedirs(args.out, exist_ok=True)
    hyp = {}
    if c1:
        hyp['H4'], hyp['H5'] = h4h5(c1, rpp_sel)
    if o:
        hyp['H6'] = h6(o)
    md = ['## 확증 가설 (prereg_r2 §5)\n\n```json\n' + json.dumps(hyp, indent=1, ensure_ascii=False) +
          '\n```\n']
    md.append(ax.table_summary(c1, TABLE_ARMS, '표 R2-1. ROS C1 — held-out C 60, 잡음 없음, stateful false',
                               ref='dwa_full', extra_ref='dwa_R2'))
    md.append(ax.table_summary(c1, TABLE_ARMS, '표 R2-1 민감도. 통로 제외 52 경로',
                               ref='dwa_full', extra_ref='dwa_R2', family=('smooth', 'astar')))
    md.append(ax.table_summary(c2, TABLE_ARMS, '표 R2-2. ROS C2 — stateful true', ref='dwa_full',
                               extra_ref='dwa_R2'))
    md.append(ax.table_stateful(c1, c2, TABLE_ARMS, '표 R2-2b. stateful true 효과 (C2 − C1)'))
    md.append(ax.table_summary(c3, TABLE_ARMS + ['dwa_x_X00'], '표 R2-3. ROS C3 — 오도메트리 σ 0.02, 12 경로 × seed 1–5',
                               ref='dwa_full', extra_ref='dwa_R2'))
    md.append(ax.table_cond_diff(c3, c1, TABLE_ARMS, '표 R2-3b. 잡음 효과 (C3 − C1)', 'C3', 'C1'))
    md.append(ax.table_o(o, TABLE_ARMS, '표 R2-4. 정적 미지 장애물 O — 20 장면 × 반복 3'))
    md.append(table_o_vs(o, 'dwa_R2', TABLE_ARMS))
    md.append(ax.table_decomp_ros(c1, TABLE_ARMS, '표 R2-5. ROS 첫 진입·지나침 분해 — C1'))
    md.append(ax.table_decomp_ros(c3, TABLE_ARMS + ['dwa_x_X00'], '표 R2-5b. 첫 진입·지나침 — C3'))
    md.append(ros_resets_r2(c3, c1, R))
    md.append(ax.rate_report([('C1', c1), ('C2', c2), ('C3', c3), ('O', o)]))
    if chain:
        arms_c = ['dwa_R', 'dwa_R2', 'dwa_textbook_tuned']
        for nz, stf, name in ((0, 0, '잡음 없음, stateful false (C1 대응)'),
                              (0, 1, '잡음 없음, stateful true (C2 대응)'),
                              (1, 0, '잡음 seed 1–5, stateful false (C3 대응)')):
            sub = [r for r in chain if (r['nseed'] > 0) == bool(nz) and r['stateful'] == stf]
            md.append(ax.table_switch(sub, f'표 R2-6. chain_sim_x (팔 − dwa_full) — held-out C 60, {name}',
                                      arms=arms_c))
        md.append(ax.slow_runs(chain, ['dwa_full'] + arms_c, 0))
        txt, _ = ax.reset_analysis(chain, os.path.join(R, 'r2', 'chain'),
                                   os.path.join(R, 'r2', 'paths_c'),
                                   arms=('dwa_full', 'dwa_R', 'dwa_R2'))
        md.append(txt)
        md.append(ax.reset_counts(chain, os.path.join(R, 'r2', 'chain'), ['dwa_full'] + arms_c))
        md.append(ax.table_failures(chain, '부록. 실패 목록 — chain_sim_x (held-out C)'))
    for name, recs, fam in (('C1', c1, None), ('C2', c2, None), ('C3', c3, None),
                            ('O', o, ('obstacle',))):
        md.append(ax.table_failures(recs, f'부록. 실패 목록 — ROS {name}', family=fam))
    with open(os.path.join(args.out, 'tables_r2.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(md))
    with open(os.path.join(args.out, 'hypotheses_r2.json'), 'w', encoding='utf-8') as f:
        json.dump(hyp, f, indent=1, ensure_ascii=False)
    print('\n'.join(md))
    return 0


def table_o_vs(o, ref, arms):
    """O 성공률 장면별 짝 차 (팔 − ref), 실행 성공 수 (기술)."""
    lines = [f'### 표 R2-4b. O 성공률 장면별 짝 차 (팔 − {ref}, 장면 묶음 부트스트랩)', '',
             '| 팔 | 성공 실행 | 장면별 성공률 차 [%p] (n) |', '| --- | --- | --- |']
    sc = collections.defaultdict(lambda: collections.defaultdict(list))
    for r in o:
        sc[r['scene']][r['arm']].append(r['success'])
    for arm in arms:
        rs = [r for r in o if r['arm'] == arm]
        if not rs:
            continue
        d = [float(np.mean(v[arm])) - float(np.mean(v[ref])) for v in sc.values()
             if v[arm] and v[ref]]
        lines.append(f'| {arm} | {sum(r["success"] for r in rs)}/{len(rs)} | '
                     f'{ax.fmt_ci(*ax.boot(d), 100)} ({len(d)}) |')
    return '\n'.join(lines) + '\n'


def ros_resets_r2(c3, c1, R):
    """C3 느린 실행의 첫 진입 전 3 s 재설정 — X00(dwa_full 동치, 기준 C1 dwa_full)·dwa_R·dwa_R2 (플러그인 기록)."""
    return _ros_resets(c3, c1, R)


def _ros_resets(c3, c1, R, window=3.0):
    lines = ['### 표 R2-7. ROS C3 재설정 기제 — 느린 실행(C1 같은 팔 대비 +30 % 초과 또는 실패)의 첫 진입 전 3 s 재설정', '',
             '| 팔 | 묶음 | 실행 | 창 안 재설정 ≥ 1 | 창 안 국면 (상위) | 실행당 재설정 |',
             '| --- | --- | --- | --- | --- | --- |']
    ref_of = {'dwa_x_X00': 'dwa_full', 'dwa_R': 'dwa_R', 'dwa_R2': 'dwa_R2'}
    clean = {(r['arm'], r['scene']): r for r in c1}
    for arm, ref in ref_of.items():
        rows = []
        for r in c3:
            if r['arm'] != arm:
                continue
            c = clean.get((ref, r['scene']))
            if c is None or not c['success']:
                continue
            rs = ax.read_plugin_resets(R, os.path.join('r2', 'c3'), r)
            if rs is None:
                continue
            slow = (not r['success']) or r['t_reach'] / c['t_reach'] > 1.30
            tf = r['t_first_entry'] if r.get('t_first_entry', -1) >= 0 else r['t_end']
            win = [x for x in rs if tf - window <= x['t_rel'] <= tf]
            rows.append((slow, len(win), len(rs), collections.Counter(x.get('phase', '?') for x in win)))
        for grp, flag in (('느린', True), ('나머지', False)):
            g = [x for x in rows if x[0] == flag]
            if not g:
                lines.append(f'| {arm} | {grp} | 0 | – | – | – |')
                continue
            ph = collections.Counter()
            for x in g:
                ph.update(x[3])
            lines.append(f'| {arm} | {grp} | {len(g)} | {sum(1 for x in g if x[1] > 0)}/{len(g)} | '
                         f'{", ".join(f"{k} {v}" for k, v in ph.most_common(3)) or "–"} | '
                         f'{np.mean([x[2] for x in g]):.2f} |')
    return '\n'.join(lines) + '\n'


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['select', 'equiv', 'tables'])
    ap.add_argument('--tune', default='')
    ap.add_argument('--raw', default='')
    ap.add_argument('--runs', default='/runs')
    ap.add_argument('--out', default='')
    args = ap.parse_args(argv)
    if args.cmd == 'select':
        print(json.dumps(select(args.tune), indent=1, ensure_ascii=False))
        return 0
    if args.cmd == 'equiv':
        print(json.dumps(ax.equiv_report(args.raw, arm_a='dwa_y_Y00', arm_b='dwa_R'), indent=1,
                         ensure_ascii=False))
        return 0
    return tables(args)


if __name__ == '__main__':
    sys.exit(main())
