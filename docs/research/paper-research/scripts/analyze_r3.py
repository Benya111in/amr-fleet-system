#!/usr/bin/env python3
"""
3 라운드 분석 (prereg_r3.md §2·§5). 2 라운드 analyze_r2.py·1 라운드 analyze_x.py 의 식을 그대로 쓴다.
**이 파일은 3 라운드 튜닝·held-out D 결과를 보기 전에 커밋한다.**

    python3 analyze_r3.py select --r2 /runs/r2/tuning/raw --r3 /runs/r3/tuning/raw   # → selection_r3.json
    python3 analyze_r3.py equiv --raw /runs/r3/equiv/raw                             # (선택) Y11 대 Z00
    python3 analyze_r3.py tables --runs /runs --out <dir>

## 선택 (prereg_r3 §2): 2 라운드와 같은 J''·S'' (analyze_r2.j_o·analyze_x.tuning_score)
- DWB·MPPI·RPP: 각 누적 36 설정. 1 라운드 격자 12 (D00–D11 등) 는 2 라운드 튜닝 원자료, 새 24 (D12–D35 등) 는 3 라운드.
- DWA-R3: Z00–Z11 (3 라운드 원자료) + Y11 (2 라운드 원자료, 이름 dwa_y_Y11). 교과서 T06 고정.
- 순서: S'' 최대 → J'' 최소 → 이름 순. 기록이 모자란 설정은 순위에서 뺀다.

## 확증 가설 (held-out D, prereg_r3 §5)
- H7 (먼저): ROS C1, 두 팔 모두 성공한 경로의 mean log(T_dwa_R3 / T_RPP선택). 95 % 상한 < log(1.005).
- H8 (H7 확증일 때만): 같은 통계량, 95 % 상한 < 0. H7 이 확증 안 되면 '판정 안 함'.
- H9 (독립): ROS O, 장면별 반복 평균 성공률 (dwa_R3 − MPPI선택) 의 평균, 장면 묶음 95 % 하한 > 0.
  **안전 조건:** O 에서 dwa_R3 의 접촉(원판 이격 ≤ 0 또는 지도 충돌)이 한 실행이라도 있으면 H9 는 확증으로 쓰지 않는다
  ('not usable (contact)').
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
import analyze_r2 as a2  # noqa: E402
import analyze_x as ax  # noqa: E402

LOG_1005 = math.log(1.005)
FAMS = {'DWB': ('DWB_',), 'MPPI': ('MPPI_',), 'RPP': ('RPP_',), 'DWA_R3': ('dwa_z_Z', 'dwa_y_Y11')}
TABLE_ARMS = ['dwa_full', 'dwa_R', 'dwa_R2', 'dwa_R3', 'dwa_textbook_tuned', 'DWB_D00', 'DWB_tuned',
              'MPPI_P00', 'MPPI_tuned', 'RPP_R00', 'RPP_tuned', 'pp']


def select(r2_raw, r3_raw):
    recs = ax.load_dir(r2_raw) + ax.load_dir(r3_raw)
    path_recs = [r for r in recs if r['family'] in ax.HELDOUT]
    o_recs = [r for r in recs if r['family'] == 'obstacle']
    by = collections.defaultdict(lambda: collections.defaultdict(list))
    for r in path_recs:
        by[r['arm']][int(r.get('seed', 0))].append(r)
    byo = collections.defaultdict(list)
    for r in o_recs:
        byo[r['arm']].append(r)
    out = {'rule': "S'' = S_0 + mean(S_1..3) + S_O 최대 → J'' 최소 → 이름 순 (2 라운드와 같음)",
           'n_records': len(recs)}
    for fam, prefixes in FAMS.items():
        cands = []
        for arm in sorted(set(by) | set(byo)):
            if not any(arm.startswith(p) for p in prefixes):
                continue
            if fam == 'DWA_R3' and arm.startswith('dwa_y_') and arm != 'dwa_y_Y11':
                continue
            per = {}
            complete = True
            for s in a2.SETTINGS:
                rs = by[arm].get(s, [])
                if len({r['scene'] for r in rs}) < a2.N_TUNE_PATHS:
                    complete = False
                per[s] = ax.tuning_score(rs) if rs else (0, 10.0)
            so, jo = a2.j_o(byo.get(arm, []))
            if len({r['scene'] for r in byo.get(arm, [])}) < a2.N_TUNE_O:
                complete = False
            s2 = per[0][0] + float(np.mean([per[s][0] for s in (1, 2, 3)])) + so
            j2 = per[0][1] + float(np.mean([per[s][1] for s in (1, 2, 3)])) + jo
            cands.append({'arm': arm, 'S2': s2, 'J2': j2, 'complete': complete,
                          'round': 'r3' if (arm.startswith('dwa_z_') or _new(arm)) else 'r2',
                          'per_setting': {str(s): {'success': per[s][0], 'J': per[s][1]}
                                          for s in a2.SETTINGS},
                          'O': {'success': so, 'J_O': jo}})
        ranked = sorted([c for c in cands if c['complete']], key=lambda c: (-c['S2'], c['J2'], c['arm']))
        out[fam] = {'chosen': ranked[0]['arm'] if ranked else None, 'n_candidates': len(cands),
                    'ranking': ranked + [c for c in cands if not c['complete']]}
    ch = out['DWA_R3']['chosen'] or ''
    out['DWA_R3']['chosen_z'] = ch.replace('dwa_z_', '') if ch.startswith('dwa_z_') else 'Y11'
    return out


def _new(arm):
    try:
        n = int(arm.split('_')[1][1:])
    except (IndexError, ValueError):
        return False
    return arm.split('_')[0] in ('DWB', 'MPPI', 'RPP') and n >= 12


def h7h8(c1, rpp_sel):
    d = ax.paired(c1, 'dwa_R3', rpp_sel, 't_reach', ax.logratio)
    m, lo, hi = ax.boot(d.values())
    k3, n3, _ = ax.success_rate(c1, 'dwa_R3')
    kr, nr, _ = ax.success_rate(c1, rpp_sel)
    h7 = bool(d) and hi < LOG_1005
    base = {'statistic': f'mean log(T_dwa_R3 / T_{rpp_sel}), 두 팔 모두 성공한 경로',
            'condition': 'ROS C1: held-out D 60, 잡음 없음, stateful false', 'n_paths': len(d),
            'mean': m, 'ci95': [lo, hi],
            'time_ratio_pct': [100 * (math.exp(x) - 1) for x in (m, lo, hi)] if d else None,
            'success_R3_RPP': [k3, n3, kr, nr]}
    out7 = dict(base, hypothesis='H7', rule='95 % 상한 < log(1.005)',
                decision='confirmed' if h7 else 'not confirmed')
    out8 = dict(base, hypothesis='H8', rule='H7 확증 뒤, 95 % 상한 < 0',
                decision=('confirmed' if hi < 0 else 'not confirmed') if h7 else
                'not tested (H7 not confirmed)')
    return out7, out8


def h9(o, mppi_sel):
    sc = collections.defaultdict(lambda: collections.defaultdict(list))
    for r in o:
        if r['arm'] in ('dwa_R3', mppi_sel):
            sc[r['scene']][r['arm']].append(r['success'])
    d = [float(np.mean(v['dwa_R3'])) - float(np.mean(v[mppi_sel])) for v in sc.values()
         if v['dwa_R3'] and v[mppi_sel]]
    m, lo, hi = ax.boot(d)
    r3 = [r for r in o if r['arm'] == 'dwa_R3']
    contacts = sum(1 for r in r3 if r.get('obs_collision') or r.get('collision'))
    stat_ok = bool(d) and lo > 0
    if not stat_ok:
        dec = 'not confirmed'
    elif contacts > 0:
        dec = 'not usable (contact)'
    else:
        dec = 'confirmed'
    return {'hypothesis': 'H9', 'statistic': f'장면별 반복 평균 성공률 차 (dwa_R3 − {mppi_sel}) 의 평균',
            'condition': 'ROS O: held-out D 매끈 앞 20 + 원판, 반복 3, 참값 트랙 차단',
            'n_scenes': len(d), 'mean': m, 'ci95': [lo, hi],
            'rule': '장면 묶음 95 % 하한 > 0, 그리고 dwa_R3 접촉 0', 'dwa_R3_contacts': contacts,
            'decision': dec,
            'runs_success_R3_MPPI': [sum(r['success'] for r in r3), len(r3),
                                     sum(r['success'] for r in o if r['arm'] == mppi_sel),
                                     sum(1 for r in o if r['arm'] == mppi_sel)]}


def three_round_table(R):
    """세 라운드 종합 (기술): 라운드별 DWA 변형 대 각 Nav2·PP 팔."""
    rounds = [
        ('1R held-out B', '', 'tuning/selection.json', 'dwa_R'),
        ('2R held-out C', 'r2/', 'r2/tuning/selection_r2.json', 'dwa_R2'),
        ('3R held-out D', 'r3/', 'r3/tuning/selection_r3.json', 'dwa_R3'),
    ]
    lines = ['### 세 라운드 종합 — 라운드별 DWA 변형 대 각 팔 (팔 − DWA 변형, 기술)', '',
             '| 라운드 | DWA 변형 | 비교 팔 | C1 시간 % (n) | C3 시간 % (n) | C1 곡선 CTE 차 [cm] | C1 \\|정지 오차\\| 차 [cm] | '
             'O 성공 (변형 / 팔) |', '| --- | --- | --- | --- | --- | --- | --- | --- |']
    for name, pre, selfn, var in rounds:
        fn = os.path.join(R, selfn)
        if not os.path.exists(fn):
            continue
        with open(fn, encoding='utf-8') as f:
            sel = json.load(f)
        c1 = ax.substitute_tuned(ax.load_dir(os.path.join(R, pre + 'c1', 'raw')), sel)
        c3 = ax.substitute_tuned(ax.load_dir(os.path.join(R, pre + 'c3', 'raw')), sel)
        o = ax.substitute_tuned(ax.load_dir(os.path.join(R, pre + 'o', 'raw')), sel)
        if not c1:
            continue
        for arm in ('dwa_full', 'RPP_R00', 'RPP_tuned', 'DWB_tuned', 'MPPI_tuned', 'pp'):
            if not any(r['arm'] == arm for r in c1):
                continue
            d1 = ax.paired(c1, arm, var, 't_reach', ax.logratio)
            d3 = ax.paired(c3, arm, var, 't_reach', ax.logratio)
            dc = ax.paired(c1, arm, var, 'cte_curve_mean')
            de = ax.paired(c1, arm, var, 'stop_err')
            sv = sum(r['success'] for r in o if r['arm'] == var)
            sa = sum(r['success'] for r in o if r['arm'] == arm)
            nv = sum(1 for r in o if r['arm'] == var)
            na = sum(1 for r in o if r['arm'] == arm)
            lines.append(f'| {name} | {var} | {arm} | {ax.fmt_pct(*ax.boot(d1.values()))} ({len(d1)}) | '
                         f'{ax.fmt_pct(*ax.boot(d3.values()))} ({len(d3)}) | '
                         f'{ax.fmt_ci(*ax.boot(dc.values()), 100)} | {ax.fmt_ci(*ax.boot(de.values()), 100)} | '
                         f'{sv}/{nv} / {sa}/{na} |')
    return '\n'.join(lines) + '\n'


def tables(args):
    R = args.runs
    with open(os.path.join(R, 'r3', 'tuning', 'selection_r3.json'), encoding='utf-8') as f:
        sel = json.load(f)
    rpp_sel = sel['RPP']['chosen']
    mppi_sel = sel['MPPI']['chosen']
    st = lambda s: os.path.join(R, 'r3', s, 'raw')   # noqa: E731
    c1 = ax.substitute_tuned(ax.load_dir(st('c1')), sel)
    c2 = ax.substitute_tuned(ax.load_dir(st('c2')), sel)
    c3 = ax.substitute_tuned(ax.load_dir(st('c3')), sel)
    o = ax.substitute_tuned(ax.load_dir(st('o')), sel)
    chain = ax.load_chain(os.path.join(R, 'r3', 'chain', 'raw')) \
        if os.path.isdir(os.path.join(R, 'r3', 'chain', 'raw')) else []
    os.makedirs(args.out, exist_ok=True)
    hyp = {}
    if c1:
        hyp['H7'], hyp['H8'] = h7h8(c1, rpp_sel)
    if o:
        hyp['H9'] = h9(o, mppi_sel)
    md = ['## 확증 가설 (prereg_r3 §5)\n\n```json\n' + json.dumps(hyp, indent=1, ensure_ascii=False) +
          '\n```\n']
    md.append(ax.table_summary(c1, TABLE_ARMS, '표 R3-1. ROS C1 — held-out D 60, 잡음 없음, stateful false',
                               ref='dwa_full', extra_ref='dwa_R3'))
    md.append(ax.table_summary(c1, TABLE_ARMS, '표 R3-1 민감도. 통로 제외 52 경로', ref='dwa_full',
                               extra_ref='dwa_R3', family=('smooth', 'astar')))
    md.append(ax.table_summary(c2, TABLE_ARMS, '표 R3-2. ROS C2 — stateful true', ref='dwa_full',
                               extra_ref='dwa_R3'))
    md.append(ax.table_stateful(c1, c2, TABLE_ARMS, '표 R3-2b. stateful true 효과 (C2 − C1)'))
    md.append(ax.table_summary(c3, TABLE_ARMS + ['dwa_x_X00'], '표 R3-3. ROS C3 — 오도메트리 σ 0.02, 12 경로 × seed 1–5',
                               ref='dwa_full', extra_ref='dwa_R3'))
    md.append(ax.table_cond_diff(c3, c1, TABLE_ARMS, '표 R3-3b. 잡음 효과 (C3 − C1)', 'C3', 'C1'))
    md.append(ax.table_o(o, TABLE_ARMS, '표 R3-4. 정적 미지 장애물 O — 20 장면 × 반복 3'))
    md.append(a2.table_o_vs(o, 'dwa_R3', TABLE_ARMS).replace('R2-4b', 'R3-4b'))
    md.append(ax.table_decomp_ros(c1, TABLE_ARMS, '표 R3-5. ROS 첫 진입·지나침 분해 — C1'))
    md.append(ros_resets_r3(c3, c1, R))
    md.append(ax.rate_report([('C1', c1), ('C2', c2), ('C3', c3), ('O', o)]))
    md.append(three_round_table(R))
    if chain:
        arms_c = ['dwa_R', 'dwa_R2', 'dwa_R3', 'dwa_textbook_tuned']
        for nz, stf, name in ((0, 0, '잡음 없음, stateful false (C1 대응)'),
                              (0, 1, '잡음 없음, stateful true (C2 대응)'),
                              (1, 0, '잡음 seed 1–5, stateful false (C3 대응)')):
            sub = [r for r in chain if (r['nseed'] > 0) == bool(nz) and r['stateful'] == stf]
            md.append(ax.table_switch(sub, f'표 R3-6. chain_sim_x (팔 − dwa_full) — held-out D 60, {name}',
                                      arms=arms_c))
        md.append(ax.slow_runs(chain, ['dwa_full'] + arms_c, 0))
        txt, _ = ax.reset_analysis(chain, os.path.join(R, 'r3', 'chain'), os.path.join(R, 'r3', 'paths_d'),
                                   arms=('dwa_full', 'dwa_R', 'dwa_R2', 'dwa_R3'))
        md.append(txt.replace('held-out B', 'held-out D'))
        md.append(ax.reset_counts(chain, os.path.join(R, 'r3', 'chain'), ['dwa_full'] + arms_c))
        md.append(ax.table_failures(chain, '부록. 실패 목록 — chain_sim_x (held-out D)'))
    for name, recs, fam in (('C1', c1, None), ('C2', c2, None), ('C3', c3, None),
                            ('O', o, ('obstacle',))):
        md.append(ax.table_failures(recs, f'부록. 실패 목록 — ROS {name}', family=fam))
    with open(os.path.join(args.out, 'tables_r3.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(md))
    with open(os.path.join(args.out, 'hypotheses_r3.json'), 'w', encoding='utf-8') as f:
        json.dump(hyp, f, indent=1, ensure_ascii=False)
    print('\n'.join(md))
    return 0


def ros_resets_r3(c3, c1, R, window=3.0):
    """C3 느린 실행의 첫 진입 전 3 s 재설정 + 실행당 재설정 (국면별) — X00·dwa_R·dwa_R2·dwa_R3."""
    lines = ['### 표 R3-7. ROS C3 재설정 — 느린 실행(같은 팔 C1 대비 +30 % 초과 또는 실패)의 첫 진입 전 3 s, 실행당 국면별', '',
             '| 팔 | 실행 | 느린 | 느린 실행 중 창 안 재설정 ≥ 1 | 실행당 재설정 | accel | decel | turn·cruise |',
             '| --- | --- | --- | --- | --- | --- | --- | --- |']
    ref_of = {'dwa_x_X00': 'dwa_full', 'dwa_R': 'dwa_R', 'dwa_R2': 'dwa_R2', 'dwa_R3': 'dwa_R3'}
    clean = {(r['arm'], r['scene']): r for r in c1}
    for arm, ref in ref_of.items():
        n = slow = slow_win = 0
        ph = collections.Counter()
        tot = 0
        for r in c3:
            if r['arm'] != arm:
                continue
            rs = ax.read_plugin_resets(R, os.path.join('r3', 'c3'), r)
            if rs is None:
                continue
            n += 1
            tot += len(rs)
            ph.update(x.get('phase', '?') for x in rs)
            c = clean.get((ref, r['scene']))
            if c is None or not c['success']:
                continue
            if (not r['success']) or r['t_reach'] / c['t_reach'] > 1.30:
                slow += 1
                tf = r['t_first_entry'] if r.get('t_first_entry', -1) >= 0 else r['t_end']
                slow_win += any(tf - window <= x['t_rel'] <= tf for x in rs)
        if n:
            lines.append(f'| {arm} | {n} | {slow} | {slow_win}/{slow} | {tot / n:.2f} | {ph["accel"] / n:.2f} | '
                         f'{ph["decel"] / n:.2f} | {(ph["turn"] + ph["cruise"]) / n:.2f} |')
    return '\n'.join(lines) + '\n'


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['select', 'tables'])
    ap.add_argument('--r2', default='')
    ap.add_argument('--r3', default='')
    ap.add_argument('--runs', default='/runs')
    ap.add_argument('--out', default='')
    args = ap.parse_args(argv)
    if args.cmd == 'select':
        print(json.dumps(select(args.r2, args.r3), indent=1, ensure_ascii=False))
        return 0
    return tables(args)


if __name__ == '__main__':
    sys.exit(main())
