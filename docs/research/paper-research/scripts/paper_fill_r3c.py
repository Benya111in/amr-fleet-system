#!/usr/bin/env python3
"""
원고 빈칸 채움표 (prereg_r3c §1). main.tex 의 \\nb{} / \\ci{} 를 문서 순서대로 세고, 순번마다 값과 출처를 낸다.
원고는 읽기만 한다 (고치지 않는다).

    python3 paper_fill_r3c.py --tex /home/CAPYI_AI/paper/cc-dwa/main.tex --values <analysis>/values_r3c.json \
        --out <analysis>/paper_fill_r3c.md
"""
import argparse
import json
import math
import re


def f1(x, nd=1):
    return '–' if x is None or (isinstance(x, float) and math.isnan(x)) else f'{x:.{nd}f}'


def sg(x, nd=1):
    return '–' if x is None or (isinstance(x, float) and math.isnan(x)) else f'{x:+.{nd}f}'


def ci(lo, hi, nd=1):
    return f'[{sg(lo, nd)}, {sg(hi, nd)}]'


def tokens(tex):
    body = tex[tex.index('\\begin{document}'):]
    out = []
    for m in re.finditer(r'\\nb\{\}|\\ci\{\}', body):
        ctx = body[max(0, m.start() - 60):m.start() + 12].replace('\n', ' ')
        out.append((m.group(0), ctx))
    return out


def build(V):
    """순번 → (값, 출처). 순번은 1 부터 (문서 순서)."""
    R3 = V['R3']
    sel = V['selection']
    dwb, mppi, rpp = sel['DWB'], sel['MPPI'], sel['RPP']
    hyp = R3.get('hypotheses', {})
    h7 = hyp.get('H7', {})
    h9 = hyp.get('H9', {})
    t1 = R3['table1']
    o3 = R3['o']
    e4 = V.get('E4', {})
    e4a = e4.get('arms', {})
    T2 = V['E1E3']
    m = {}
    src_h7 = 'R3 hypotheses_r3.json H7 (time_ratio_pct)'
    if h7.get('time_ratio_pct'):
        m[1] = (sg(h7['time_ratio_pct'][0], 2), src_h7)
        m[2] = (ci(h7['time_ratio_pct'][1], h7['time_ratio_pct'][2], 2), src_h7)
    m[3] = (str(o3['dwa_R3']['success']), 'R3 표 R3-4 (r3/o, dwa_R3 성공 실행)')
    m[4] = (str(o3[mppi]['success']), f'R3 표 R3-4 ({mppi})')
    r_m = e4.get(f'ratio_dwa_R3_over_{mppi}')
    m[5] = (f1(r_m['ratio'], 2) if r_m else '–', 'E4 (r3c/e4, 경로 평균 비 dwa_R3/MPPI_sel)')
    # 표 1
    rows = [('dwa_R3', None), ('dwa_textbook_tuned', 'dwa_textbook_tuned'), (dwb, dwb), (mppi, mppi), (rpp, rpp)]
    k = 6
    for arm, d in rows:
        s = t1[arm]
        src = f'R3 원자료 r3/c1 ({arm}), analyze_r3c 정의'
        m[k] = (f"{s['success']}/60" if arm == 'dwa_R3' else str(s['success']), src)
        k += 1
        m[k] = (f1(s['t_tref_med'], 3), src + ' T/T_ref 경로 중앙')
        k += 1
        if d is not None:
            dd = R3['c1_delta'][arm]
            m[k] = (sg(dd['pct'][0]), f'R3 원자료 C1, 짝 기준 dwa_R3 ({arm} − dwa_R3, n {dd["n"]})')
            k += 1
            m[k] = (ci(dd['pct'][1], dd['pct'][2]), 'B 10 000 경로 묶음')
            k += 1
        m[k] = (f1(s['cte_curve_cm_med'], 2), src + ' 곡선 CTE 평균 경로 중앙 [cm]')
        k += 1
        m[k] = (f1(s['stop_err_cm_med'], 1), src + ' |정지 오차| 경로 중앙 [cm]')
        k += 1
        if d is not None:
            dd = R3['c3_delta'][arm]
            m[k] = (sg(dd['pct'][0]), f'R3 원자료 r3/c3 (seed 평균 뒤 짝, n {dd["n"]})')
            k += 1
            m[k] = (ci(dd['pct'][1], dd['pct'][2]), 'B 10 000 경로 묶음')
            k += 1
        m[k] = (str(o3[arm]['success']), f'R3 표 R3-4 ({arm}, 성공 실행 /60)')
        k += 1
        if d is not None:
            m[k] = (str(o3[arm]['contact']), f'R3 r3/o ({arm}) 접촉 실행 수 (원판 이격 ≤ 0 또는 지도 충돌)')
            k += 1
        a = e4a.get(arm, {})
        m[k] = (f1(a.get('mean_ms'), 2), f'E4 r3c/e4 주기당 평균 [ms] ({arm})')
        k += 1
    assert k == 56, k
    p2 = V['E5'].get('P2', {})
    m[56] = (f1(p2.get('chain_slope_med'), 2), 'E1 chain V-meas 출발 v_m 0.3→0.6 기울기 ÷ a (중앙, r3c/chain)')
    e2 = V.get('E2') or {}
    m[57] = (f1(e2.get('accel2_per_run'), 2), 'E2 X00 출발 2 s 가속 재설정/실행 (r3c/e2)')
    m[58] = (f1(e2.get('t09_minus_E3_dwa_R3'), 2), 'E2 X00 t0.9 중앙 − E3 dwa_R3 t0.9 중앙 [s]')
    sw = R3['x00_c3_decel_window']
    m[59] = (str(sw['slow']), 'R3 r3/c3 dwa_x_X00 느린 실행 (C1 dwa_full 대비 +30 % 또는 실패)')
    m[60] = (str(sw['with_decel_reset_in_window']), 'R3 r3/c3 X00 느린 실행 중 첫 진입 전 3 s 감속 재설정')
    m[61] = (f1(R3['chain']['dwa_R2']['accel_resets_per_run'], 2), 'R3 chain dwa_R2 (Y11) 잡음 없음 가속 재설정/실행')
    m[62] = (f1(R3['dwa_R2_c1_resets'].get('accel2_per_run'), 2), 'R3 r3/c1 dwa_R2 출발 2 s 가속 재설정/실행')
    y = V['selection_yaml']
    m[63] = (str(y['K']), 'tuning_selection_r3.json 선택 설정 research.reset_persist_cycles')
    m[64] = (f1(y['D'], 1), '선택 설정 research.goal_extension_dist')
    m[65] = (f1(y['S'], 1), '선택 설정 research.blocked_detour_shift')
    rates = [v['cmd_rate_min'] for v in V['rates'].values()]
    m[66] = (f'{math.floor(10 * min(rates)) / 10:.1f}', 'R3 tuning·c1·c2·c3·o + R3-C e2·c1·c3·o·e4eq·e4 cmd_rate_hz 최소 (내림)')
    if h7.get('time_ratio_pct'):
        m[67] = (sg(h7['time_ratio_pct'][0], 2), src_h7)
        m[68] = (ci(h7['time_ratio_pct'][1], h7['time_ratio_pct'][2], 2), src_h7)
    se = [R3['stop_err_diff_vs_R3'][a]['mean'] for a in (dwb, mppi, rpp)]
    m[69] = (f1(min(se), 1), f'R3 C1 |정지 오차| 짝 차 (Nav2 선택 − dwa_R3) 평균의 최소 (DWB/MPPI/RPP = '
             f'{", ".join(f1(x, 1) for x in se)} cm)')
    m[70] = (f1(max(se), 1), '같은 값의 최대')
    cc = R3['cte_curve_diff_vs_R3'][rpp]['mean']
    m[71] = (f1(-cc, 2) if cc is not None else '–', f'R3 C1 곡선 CTE 짝 차 dwa_R3 − {rpp} 평균 [cm]')
    c3 = R3['c3_vs_c1']['dwa_R3']
    m[72] = (sg(c3['pct'][0]) if c3.get('pct') else '–', 'R3 dwa_R3 C3 − C1 (경로 짝, seed 평균)')
    m[73] = (str(R3['slow_c3_dwa_R3']['slow']), 'R3 r3/c3 dwa_R3 느린 실행 (같은 팔 C1 대비)')
    m[74] = (str(o3['dwa_R3']['success']), 'R3 r3/o dwa_R3 성공 실행')
    m[75] = (f1(o3['dwa_R3']['min_clear_cm_min'], 1), 'R3 r3/o dwa_R3 원판 최소 이격 최솟값 (전체 60 실행) [cm]')
    m[76] = (str(o3[mppi]['success']), f'R3 r3/o {mppi} 성공 실행')
    m[77] = (str(o3[rpp]['contact']), f'R3 r3/o {rpp} 접촉 실행')
    m[78] = (str(o3[dwb]['contact']), f'R3 r3/o {dwb} 접촉 실행')
    if h9:
        m[79] = (sg(h9['mean'], 3), 'R3 hypotheses_r3.json H9 (장면별 성공률 차, 비율)')
        m[80] = (ci(h9['ci95'][0], h9['ci95'][1], 3), 'R3 H9 장면 묶음 부트스트랩')

    # 표 2 (E1 + E3 같은 배치)
    def t2(vid):
        return T2.get(vid, {})

    def eq(vid, txt):
        st = t2(vid).get('status')
        return txt + (' (= CC-DWA, 돌지 않음)' if st and st != 'run' else '')
    cc_ = t2('CC-DWA')['c1']
    m[81] = (f1(cc_['t09_med'], 2), 'E3 r3c/c1 dwa_R3 t0.9 중앙 [s]')
    m[82] = (f1(cc_.get('accel2_per_run'), 2), 'E3 dwa_R3 출발 2 s 가속 재설정/실행')
    m[83] = (str(t2('CC-DWA')['slow_c3']['slow']), 'E3 r3c/c3 dwa_R3 느린 실행 /60')
    m[84] = (str(t2('CC-DWA')['o']['success']), 'E3 r3c/o dwa_R3 성공 /60')
    k = 85
    for vid in ('V-meas', 'V-thr', 'V-mdl-n', 'V-mdl-i', 'V-noext', 'V-nodet'):
        r = t2(vid)
        d = r.get('delta_c1')
        m[k] = (eq(vid, sg(d['pct'][0]) if d else '0 (= CC-DWA)'), f'E1 {vid} C1 Δ시간 % (짝 기준 E3 dwa_R3, n '
                f'{d["n"] if d else "–"}, 구간 {ci(*d["pct"][1:]) if d else "–"})')
        k += 1
        m[k] = (eq(vid, f1(r['c1']['t09_med'], 2)), f'E1 {vid} C1 t0.9 중앙 [s]')
        k += 1
        if vid != 'V-meas':
            m[k] = (eq(vid, f1(r['c1'].get('accel2_per_run'), 2)), f'E1 {vid} 출발 2 s 가속 재설정/실행')
            k += 1
        m[k] = (eq(vid, str(r['slow_c3']['slow'])), f'E1 {vid} C3 느린 실행 /60 (같은 팔 C1 대비)')
        k += 1
        m[k] = (eq(vid, str(r['o']['success'])), f'E1 {vid} O 성공 /60 (접촉 {r["o"]["contact"]})')
        k += 1
    assert k == 114, k
    thr = t2('V-thr')
    m[114] = (eq('V-thr', f1(thr['c1'].get('accel2_per_run'), 2)), 'E1 V-thr 출발 2 s 가속 재설정/실행')
    m[115] = (eq('V-thr', f1(thr.get('delta_t09_med'), 2)), 'E1 V-thr t0.9 중앙 − E3 dwa_R3 [s]')
    m[116] = (eq('V-thr', sg(thr['delta_c1']['pct'][0]) if thr.get('delta_c1') else '0'), 'E1 V-thr C1 Δ시간 %')
    m[117] = (eq('V-mdl-n', f1(t2('V-mdl-n')['c1'].get('accel2_per_run'), 2)), 'E1 V-mdl-n 출발 2 s 가속 재설정/실행')
    m[118] = ('0.01', 'params_r3.F1_FIT model_delay (analysis/f1_fit_r3.json, 튜닝-20 R2 ROS 궤적)')
    m[119] = ('0.06', 'params_r3.F1_FIT model_tau')
    m[120] = (eq('V-mdl-i', f1(t2('V-mdl-i')['c1'].get('accel2_per_run'), 2)), 'E1 V-mdl-i 출발 2 s 가속 재설정/실행')
    g1src = 'CC-DWA' if y['accel_hold'] and not y['reset_model'] else ('V-hold' if not y['accel_hold'] else 'V-G1only')
    m[121] = (f1(t2(g1src)['c1'].get('accel2_per_run'), 2), f'E3/E1 {g1src} (유예만·모형 없음) 출발 2 s 가속 재설정/실행')
    m[122] = (str(cc_['entry_yaw_over_005']), f'E3 dwa_R3 C1 진입 yaw 오차 > 0.05 rad 경로 (/{cc_["entry_yaw_n"]})')
    ne = t2('V-noext')['c1']
    m[123] = (eq('V-noext', str(ne['entry_yaw_over_005'])), f'E1 V-noext C1 진입 yaw 오차 > 0.05 rad (/{ne["entry_yaw_n"]})')
    m[124] = (eq('V-nodet', str(t2('V-nodet')['o']['success'])), 'E1 V-nodet O 성공 /60')
    vm = t2('V-meas').get('delta_c1')
    m[125] = (sg(vm['pct'][0]) if vm else '–', 'E1 V-meas C1 Δ시간 %')
    tg = R3['testbed_gap']
    m[126] = (f1(tg['time_pct'][0], 1) if tg.get('time_pct') else '–',
              f'R3 dwa_R3 ROS C1 − chain 잡음 없음 경로 짝 log 비 평균 % (구간 {ci(*tg["time_pct"][1:])}, '
              f'경로 |차| 최대 {f1(tg["time_abs_pct_max"], 1)} %)' if tg.get('time_pct') else 'R3 chain·C1')
    m[127] = (f1(tg['cte_cm'][0], 2) if tg.get('cte_cm') else '–',
              f'R3 dwa_R3 ROS − chain CTE 평균 짝 차 [cm] (경로 |차| 최대 {f1(tg.get("cte_abs_cm_max"), 2)})')
    a = e4a.get('dwa_R3', {})
    m[128] = (f1(a.get('n_samples_med'), 0), 'E4 dwa_R3 dwa/stats[1] 주기당 후보 수 중앙')
    m[129] = (f1(a.get('rollout_steps_med'), 0), 'E4 dwa_R3 굴림 스텝 중앙 (⌈T_sim(v)/0.1⌉)')
    m[130] = (f1(a.get('mean_ms'), 2), 'E4 dwa_R3 주기당 평균 [ms] (경로 평균의 평균)')
    m[131] = (f1(100 * a['mean_ms'] / 50.0, 1) if a.get('mean_ms') else '–', '평균 ÷ 50 ms')
    r_t = e4.get('ratio_dwa_R3_over_dwa_textbook_tuned')
    m[132] = (f1(r_t['ratio'], 2) if r_t else '–', 'E4 비 dwa_R3/교과서 (경로 묶음 부트스트랩 구간 '
              f'{ci(*r_t["ci"], 2) if r_t else "–"})')
    m[133] = (f1(r_m['ratio'], 2) if r_m else '–', 'E4 비 dwa_R3/MPPI_sel (구간 '
              f'{ci(*r_m["ci"], 2) if r_m else "–"})')
    m[134] = (f1(r_m['ratio'], 2) if r_m else '–', 'E4 비 dwa_R3/MPPI_sel')
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tex', required=True)
    ap.add_argument('--values', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    with open(args.tex, encoding='utf-8') as f:
        tex = f.read()
    with open(args.values, encoding='utf-8') as f:
        V = json.load(f)
    toks = tokens(tex)
    m = build(V)
    n_nb = sum(1 for t, _ in toks if t.startswith('\\nb'))
    n_ci = len(toks) - n_nb
    lines = ['# 원고 빈칸 채움표 (R3-C)', '',
             f'원고: `{args.tex}` (읽기만). 빈칸 표지 {len(toks)} 개 (nb {n_nb}, ci {n_ci} — ci 하나는 □□ 두 칸) '
             '를 문서 순서로 센다. 값의 출처는 오른쪽 열이다.', '',
             '| # | 표지 | 앞 문맥 | 값 | 출처 |', '| --- | --- | --- | --- | --- |']
    missing = []
    for i, (t, ctx) in enumerate(toks, 1):
        v, s = m.get(i, ('(채울 수 없음)', '사상 없음'))
        if i not in m:
            missing.append(i)
        c = ctx.replace('|', '\\|')
        lines.append(f'| {i} | `{t}` | …{c} | **{v}** | {s} |')
    lines += ['', f'채우지 못한 순번: {missing if missing else "없음"}']
    with open(args.out, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines[-3:]))


if __name__ == '__main__':
    main()
