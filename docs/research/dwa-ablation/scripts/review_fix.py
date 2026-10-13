#!/usr/bin/env python3
"""
리뷰 반영 사후 분석 — M3 시간 비용 분해와 잡음 취약성 재계산 (확증 아님).

    python3 review_fix.py --raw /runs/raw --traces /runs/review_fix/traces \
        --probe /runs/traces/probe_chain_20 --paths /runs/paths --out <dir>

1. M3 끔 − 켬 주행 시간 차를 "목표 0.10 m 첫 진입까지(도착 지연)" 와 "첫 진입 뒤 판정 통과까지
   (지나침·복귀)" 로 나눈다. 궤적은 chain_sim 을 trace_dir 로 다시 돌린 것이다 (원자료와 지표가
   비트 단위로 같은지 먼저 확인한다).
2. 오도메트리 잡음에 따른 우리 DWA 의 시간 증가를 경로 60 개·12 개 부분집합·실행 단위로 다시 계산하고,
   느린 실행(+30 % 초과)의 추가 시간을 같은 방식으로 나눈다. ROS 시험대는 궤적이 없어 요약 지표만 본다.
"""
import argparse
import collections
import csv
import glob
import json
import math
import os

import numpy as np

from analyze import boot, load, per_path

NOISE_IDS = ('S01_pwc', 'S02_pwl', 'S03_pwc', 'S04_pwl', 'S05_pwc', 'S06_pwl', 'N1', 'N4', 'N6',
             'A05', 'A06', 'A12')
SKIP = ('cycle_us_mean', 'cycle_us_p95', 'cycle_us_max')
R_GOAL = 0.10
SLOW = 1.30


def read_goals(paths_dir):
    """경로 파일에서 장면별 목표 자세를 읽는다."""
    goals = {}
    for fn in glob.glob(os.path.join(paths_dir, 'scenes_*.txt')):
        with open(fn, encoding='utf-8') as f:
            tok = f.read().split()
        i = 0
        while i < len(tok):
            if tok[i] != 'scene':
                i += 1
                continue
            sid = tok[i + 1]
            n = int(tok[i + 4])
            base = i + 5 + 3 * (n - 1)
            goals[sid] = tuple(float(x) for x in tok[base:base + 3])
            i = base + 3
    return goals


def read_trace(fn):
    with open(fn, encoding='utf-8') as f:
        rows = [[float(x) for x in r] for r in list(csv.reader(f))[1:]]
    return np.asarray(rows)


def arrival(tr, goal):
    """
    참값 자세가 목표 0.10 m 안에 처음 든 시각·속도와, 그 뒤 목표 방향으로 지나친 최대 거리.

    cmd_jump: 첫 진입 전 3 s 동안 제어기 명령 v 가 한 번에 오른 최대 폭 (감속 중 창 중심 재설정의 정황).
    gap: 같은 구간의 실제 속도 − 명령 최댓값 (재설정 임계 0.3 m/s 와 견준다).
    """
    gx, gy, gth = goal
    d = np.hypot(tr[:, 1] - gx, tr[:, 2] - gy)
    idx = np.nonzero(d <= R_GOAL)[0]
    if len(idx) == 0:
        return None
    k = int(idx[0])
    along = (tr[k:, 1] - gx) * math.cos(gth) + (tr[k:, 2] - gy) * math.sin(gth)
    done = np.nonzero(tr[:, 9] > 0.5)[0]
    k_end = int(done[0]) if len(done) else len(tr)
    w_after = np.abs(tr[k:k_end, 5]) if k_end > k else np.zeros(1)
    m = (tr[:, 0] >= tr[k, 0] - 3.0) & (tr[:, 0] <= tr[k, 0])
    jump = float(np.diff(tr[m, 6]).max()) if m.sum() > 1 else 0.0
    gap = float((tr[m, 4] - tr[m, 6]).max()) if m.any() else 0.0
    return {'t_first': float(tr[k, 0]), 'v_first': float(tr[k, 4]),
            'overshoot': float(max(0.0, along.max())), 'w_after_max': float(w_after.max()),
            'cmd_jump': jump, 'gap': gap}


def check_identical(new_fn, old_fn):
    """다시 돌린 기록이 원자료와 (계산 시간을 빼고) 같은지."""
    def key(r):
        return (r['arm'], r['scene'], r['seed'])
    old = {key(r): r for r in load(old_fn)}
    new = load(new_fn)
    bad = [key(r) for r in new if any(r[k] != old[key(r)][k] for k in r if k not in SKIP)]
    return len(new), len(bad)


def q(vals, p):
    return float(np.percentile(np.asarray(vals, dtype=float), p))


def m3_decomposition(clean, traces, goals, probe_dir, out):
    rec = {(r['arm'], r['scene']): r for r in clean}
    rows = []
    for (arm, sc), r in sorted(rec.items()):
        if arm != 'dwa_no_m3':
            continue
        on = rec[('dwa_full', sc)]
        if not (r['success'] and on['success']):
            continue
        a_on = arrival(read_trace(f'{traces}/clean/dwa_full__{sc}__s1.csv'), goals[sc])
        a_off = arrival(read_trace(f'{traces}/clean/dwa_no_m3__{sc}__s1.csv'), goals[sc])
        d_tot = r['t_reach'] - on['t_reach']
        d_arr = a_off['t_first'] - a_on['t_first']
        rows.append({'scene': sc, 'ratio': r['t_reach'] / on['t_reach'], 'd_tot': d_tot,
                     'd_arr': d_arr, 'd_after': d_tot - d_arr,
                     'log_first': math.log(a_off['t_first'] / a_on['t_first']),
                     'v_first_on': a_on['v_first'], 'v_first_off': a_off['v_first'],
                     'os_on': a_on['overshoot'], 'os_off': a_off['overshoot'],
                     'w_off': a_off['w_after_max']})
    n = len(rows)
    tot = sum(x['d_tot'] for x in rows)
    aft = sum(x['d_after'] for x in rows)
    lf, lo, hi = boot([x['log_first'] for x in rows])
    shares = [x['d_after'] / x['d_tot'] for x in rows if x['d_tot'] > 0]
    res = {
        'n': n,
        'ratio_min_med_max': [min(x['ratio'] for x in rows), q([x['ratio'] for x in rows], 50),
                              max(x['ratio'] for x in rows)],
        'd_tot_med': q([x['d_tot'] for x in rows], 50),
        'd_arr_med_p10_p90': [q([x['d_arr'] for x in rows], p) for p in (50, 10, 90)],
        'd_after_med': q([x['d_after'] for x in rows], 50),
        'share_after_sum': aft / tot,
        'share_after_path_med_p10': [q(shares, 50), q(shares, 10)],
        'first_entry_pct_ci': [100 * (math.exp(v) - 1) for v in (lf, lo, hi)],
        'v_first_med_on_off': [q([x['v_first_on'] for x in rows], 50),
                               q([x['v_first_off'] for x in rows], 50)],
        'overshoot_gt_0.10_on_off': [sum(x['os_on'] > R_GOAL for x in rows),
                                     sum(x['os_off'] > R_GOAL for x in rows)],
        'overshoot_off_med_p90': [q([x['os_off'] for x in rows], 50),
                                  q([x['os_off'] for x in rows], 90)],
        'w_after_off_med': q([x['w_off'] for x in rows], 50),
    }
    probe = {}
    for arm in ('dwa_full', 'dwa_no_m3'):
        tr = read_trace(f'{probe_dir}/{arm}__P_straight30__s1.csv')
        a = arrival(tr, (tr[0, 1] + 30.0, tr[0, 2], 0.0))
        done = tr[np.nonzero(tr[:, 9] > 0.5)[0][0], 0]
        v90 = tr[np.nonzero(tr[:, 4] >= 0.9)[0][0], 0]
        probe[arm] = dict(a, t_done_row=float(done), t_v90=float(v90))
    res['probe'] = probe
    out['m3'] = res
    out['m3_rows'] = rows


def noise_effect(clean, noise, arm, subset=None):
    a = per_path(noise, arm, 't_reach', True)
    b = per_path(clean, arm, 't_reach', True)
    common = sorted(s for s in set(a) & set(b) if subset is None or s in subset)
    m, lo, hi = boot([math.log(a[s] / b[s]) for s in common])
    return {'n_paths': len(common), 'pct_ci': [100 * (math.exp(v) - 1) for v in (m, lo, hi)]}


def run_ratios(clean, noise, arm):
    c = {r['scene']: r for r in clean if r['arm'] == arm}
    out = []
    for r in noise:
        if r['arm'] != arm:
            continue
        cr = c.get(r['scene'])
        if cr and cr['success'] and r['success']:
            out.append((r['t_reach'] / cr['t_reach'], r, cr))
    return out


def summarize_runs(rr):
    rat = np.asarray([x[0] for x in rr])
    slow = [x for x in rr if x[0] > SLOW]
    norm = [x for x in rr if x[0] <= SLOW]

    def med(grp, k):
        v = [x[1][k] for x in grp if x[1][k] > -0.5]
        return q(v, 50) if v else float('nan')
    nr = np.asarray([x[0] for x in norm])
    return {
        'n_runs': len(rr), 'arith_pct': 100 * (rat.mean() - 1),
        'median_pct': 100 * (np.median(rat) - 1), 'n_slow': len(slow),
        'slow_range_pct': ([100 * (min(x[0] for x in slow) - 1),
                            100 * (max(x[0] for x in slow) - 1)] if slow else None),
        'n_slow_ge47': int(sum(x[0] >= 1.47 for x in slow)),
        'normal_range_pct': [100 * (nr.min() - 1), 100 * (nr.max() - 1)],
        'normal_median_pct': 100 * (np.median(nr) - 1),
        'normal_within_1pct': int(sum(abs(nr - 1) <= 0.01)),
        'normal_le_6.5pct': int(sum(nr <= 1.065)),
        'cte_max_med_slow_norm_cm': [100 * med(slow, 'cte_max'), 100 * med(norm, 'cte_max')],
        't_v90_med_slow_norm_clean': [med(slow, 't_v90'), med(norm, 't_v90'),
                                      q([x[2]['t_v90'] for x in rr], 50)],
    }


def noise_decomposition(clean, noise, traces, goals):
    """chain_sim 잡음 실행의 추가 시간을 도착 지연과 첫 진입 뒤로 나눈다."""
    rr = run_ratios(clean, noise, 'dwa_full')
    rows = []
    for ratio, r, cr in rr:
        a_c = arrival(read_trace(f'{traces}/clean/dwa_full__{r["scene"]}__s1.csv'),
                      goals[r['scene']])
        fn = f'{traces}/noise/s{r["seed"]}/dwa_full__{r["scene"]}__s{r["seed"]}.csv'
        a_n = arrival(read_trace(fn), goals[r['scene']])
        d_tot = r['t_reach'] - cr['t_reach']
        d_arr = a_n['t_first'] - a_c['t_first']
        rows.append({'scene': r['scene'], 'seed': r['seed'], 'ratio': ratio, 'd_tot': d_tot,
                     'd_arr': d_arr, 'd_after': d_tot - d_arr, 'os_n': a_n['overshoot'],
                     'v_first_n': a_n['v_first'], 'v_first_c': a_c['v_first'],
                     't_v90_n': r['t_v90'], 't_v90_c': cr['t_v90'],
                     'jump_n': a_n['cmd_jump'], 'jump_c': a_c['cmd_jump'], 'gap_c': a_c['gap']})
    slow = [x for x in rows if x['ratio'] > SLOW]
    rest = [x for x in rows if x['ratio'] <= SLOW]

    def part(grp):
        tot = sum(x['d_tot'] for x in grp)
        return {'n': len(grp), 'sum_d_tot': tot, 'sum_d_arr': sum(x['d_arr'] for x in grp),
                'share_after': (tot - sum(x['d_arr'] for x in grp)) / tot if tot else float('nan'),
                'd_arr_med': q([x['d_arr'] for x in grp], 50),
                'd_after_med': q([x['d_after'] for x in grp], 50),
                'overshoot_gt_0.10': sum(x['os_n'] > R_GOAL for x in grp),
                'overshoot_med': q([x['os_n'] for x in grp], 50),
                'v_first_med': q([x['v_first_n'] for x in grp], 50),
                't_v90_med_noise_clean': [q([x['t_v90_n'] for x in grp], 50),
                                          q([x['t_v90_c'] for x in grp], 50)],
                'jump_ge_0.15': sum(x['jump_n'] >= 0.15 for x in grp)}
    clean_rows = {x['scene']: x for x in rows}.values()
    jump = [x for x in rows if x['jump_n'] >= 0.15]
    extra = {'clean_jump_ge_0.15': sum(x['jump_c'] >= 0.15 for x in clean_rows),
             'clean_jump_max': max(x['jump_c'] for x in clean_rows),
             'clean_paths': len(clean_rows),
             'clean_gap_med_max': [q([x['gap_c'] for x in clean_rows], 50),
                                   max(x['gap_c'] for x in clean_rows)],
             'after_gt2_with_jump': [sum(x['d_after'] > 2 for x in jump), len(jump)],
             'after_gt2_without_jump': [sum(x['d_after'] > 2 for x in rows if x['jump_n'] < 0.15),
                                        len(rows) - len(jump)]}
    return {'slow': part(slow), 'rest': part(rest), 'all': part(rows), 'jump': extra}, rows


def fmt(v, nd=1, sign=True):
    return f'{v:+.{nd}f}' if sign else f'{v:.{nd}f}'


def write_md(o, fn):
    m = o['m3']
    p = m['probe']
    on, off = p['dwa_full'], p['dwa_no_m3']
    L = ['# 리뷰 반영 사후 분석 (`scripts/review_fix.py`, 확증 아님)', '',
         f'궤적 재실행 동일성: 잡음 없음 {o["ident"]["clean"]}, 잡음 {o["ident"]["noise"]} '
         '(n, 원자료와 다른 기록 수 — 계산 시간 제외)', '',
         '## R1. M3 끔 − 켬 시간 분해 (chain_sim 사슬 20 Hz, held-out, 둘 다 성공)', '',
         f'- 탐침 직선 30 m: 목표 0.10 m 첫 진입 켬 {on["t_first"]:.2f} s · 끔 {off["t_first"]:.2f} s '
         f'(도착 지연 {off["t_first"] - on["t_first"]:+.2f} s), 판정 통과(궤적 done) 켬 '
         f'{on["t_done_row"]:.2f} s · 끔 {off["t_done_row"]:.2f} s, 끔의 첫 진입 속도 '
         f'{off["v_first"]:.2f} m/s (켬 {on["v_first"]:.2f}), 지나친 거리 {off["overshoot"]:.2f} m, '
         f'복귀 중 |ω| 최대 {off["w_after_max"]:.2f} rad/s, 0 → 0.9 m/s {on["t_v90"]:.2f} → '
         f'{off["t_v90"]:.2f} s',
         f'- 경로 수 {m["n"]}; 시간비 최소·중앙·최대 '
         + ' / '.join(f'{v:.2f}' for v in m['ratio_min_med_max'])
         + f'; 추가 시간 중앙 {m["d_tot_med"]:.1f} s',
         f'- 도착 지연(첫 진입 시각 차) 중앙 {m["d_arr_med_p10_p90"][0]:+.2f} s '
         f'[p10 {m["d_arr_med_p10_p90"][1]:+.2f}, p90 {m["d_arr_med_p10_p90"][2]:+.2f}], '
         f'첫 진입 시각 log 비 평균 {fmt(m["first_entry_pct_ci"][0])} % '
         f'[{fmt(m["first_entry_pct_ci"][1])}, {fmt(m["first_entry_pct_ci"][2])}]',
         f'- 첫 진입 뒤 시간 차 중앙 {m["d_after_med"]:+.1f} s; '
         f'추가 시간 합 가운데 첫 진입 뒤 몫 {100 * m["share_after_sum"]:.1f} % '
         f'(경로별 몫 중앙 {100 * m["share_after_path_med_p10"][0]:.1f} %, '
         f'p10 {100 * m["share_after_path_med_p10"][1]:.1f} %)',
         f'- 첫 진입 속도 중앙 켬 {m["v_first_med_on_off"][0]:.2f} · '
         f'끔 {m["v_first_med_on_off"][1]:.2f} m/s; '
         f'0.10 m 넘게 지나친 경로 켬 {m["overshoot_gt_0.10_on_off"][0]} · 끔 '
         f'{m["overshoot_gt_0.10_on_off"][1]}; '
         f'끔의 지나친 거리 중앙 {m["overshoot_off_med_p90"][0]:.2f} m '
         f'(p90 {m["overshoot_off_med_p90"][1]:.2f}); '
         f'첫 진입 뒤 |ω| 최대 중앙 {m["w_after_off_med"]:.2f} rad/s',
         '', '## R2. 오도메트리 잡음에 따른 시간 증가 (잡음 − 잡음 없음, 같은 팔)', '',
         '경로 단위: 경로마다 seed 평균 t_reach 의 log 비, 경로 묶음 부트스트랩 95 % '
         '(analyze.py 와 같은 규칙).', '',
         '| 시험대 | 팔 | 경로 | 시간 % [95 %] |', '| --- | --- | --- | --- |']
    for (tb, arm, lab), v in o['noise_path'].items():
        c = v['pct_ci']
        L.append(f'| {tb} | {arm} | {v["n_paths"]} ({lab}) | '
                 f'{fmt(c[0])} [{fmt(c[1])}, {fmt(c[2])}] |')
    L += ['', '실행 단위 (우리 DWA, 잡음 실행 ÷ 같은 경로 잡음 없음 실행):', '',
          '| 시험대 | 실행 | 산술 평균 % | 중앙 % | +30 % 초과 (범위) | 그중 +47 % 이상 '
          '| 나머지 범위 · 중앙 · ±1 % 안 · +6.5 % 이하 '
          '| CTE 최대 중앙 느림/나머지 [cm] | t_v90 중앙 느림/나머지/잡음 없음 [s] |',
          '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for tb, s in o['noise_runs'].items():
        L.append(f'| {tb} | {s["n_runs"]} | {fmt(s["arith_pct"])} | {fmt(s["median_pct"])} | '
                 f'{s["n_slow"]} ({fmt(s["slow_range_pct"][0])}–{fmt(s["slow_range_pct"][1])}) | '
                 f'{s["n_slow_ge47"]} | '
                 f'{fmt(s["normal_range_pct"][0])}–{fmt(s["normal_range_pct"][1])} · '
                 f'{fmt(s["normal_median_pct"])} · {s["normal_within_1pct"]} · '
                 f'{s["normal_le_6.5pct"]} | {s["cte_max_med_slow_norm_cm"][0]:.1f} / '
                 f'{s["cte_max_med_slow_norm_cm"][1]:.1f} | '
                 + ' / '.join(f'{v:.2f}' for v in s['t_v90_med_slow_norm_clean']) + ' |')
    L += ['', 'chain_sim 잡음 실행의 추가 시간 분해 (궤적, 첫 진입 = 참값 자세 목표 0.10 m):', '',
          '| 묶음 | 실행 | 추가 시간 합 [s] | 첫 진입 뒤 몫 | 도착 지연 중앙 [s] | 첫 진입 뒤 중앙 [s] '
          '| 0.10 m 넘게 지나침 | 지나친 거리 중앙 [m] | 첫 진입 속도 중앙 [m/s] | t_v90 중앙 잡음/없음 [s] '
          '| 명령 v 상승 ≥ 0.15 (첫 진입 전 3 s) |',
          '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for k in ('slow', 'rest', 'all'):
        s = o['noise_decomp'][k]
        L.append(f'| {k} | {s["n"]} | {s["sum_d_tot"]:.1f} | {100 * s["share_after"]:.1f} % | '
                 f'{s["d_arr_med"]:+.2f} | {s["d_after_med"]:+.2f} | {s["overshoot_gt_0.10"]} | '
                 f'{s["overshoot_med"]:.2f} | {s["v_first_med"]:.2f} | '
                 f'{s["t_v90_med_noise_clean"][0]:.2f} / {s["t_v90_med_noise_clean"][1]:.2f} | '
                 f'{s["jump_ge_0.15"]} |')
    j = o['noise_decomp']['jump']
    L += ['', '- 잡음 없는 같은 경로: 명령 v 상승 ≥ 0.15 '
          f'{j["clean_jump_ge_0.15"]}/{j["clean_paths"]} '
          f'(최대 {j["clean_jump_max"]:.3f} m/s), 첫 진입 전 3 s 의 실제 속도 − 명령 최댓값 중앙 '
          f'{j["clean_gap_med_max"][0]:.3f} · 최대 {j["clean_gap_med_max"][1]:.3f} m/s (재설정 임계 0.3)',
          f'- 첫 진입 뒤 추가 > 2 s 인 실행: 명령 상승 있음 {j["after_gt2_with_jump"][0]}/'
          f'{j["after_gt2_with_jump"][1]}, '
          f'없음 {j["after_gt2_without_jump"][0]}/{j["after_gt2_without_jump"][1]}']
    with open(fn, 'w', encoding='utf-8') as f:
        f.write('\n'.join(L) + '\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--raw', required=True)
    ap.add_argument('--traces', required=True, help='review_fix/traces (clean/, noise/s<k>/)')
    ap.add_argument('--traced-raw', required=True, help='review_fix/raw (궤적 재실행 기록)')
    ap.add_argument('--probe', required=True)
    ap.add_argument('--paths', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    goals = read_goals(a.paths)
    clean = [r for r in load(f'{a.raw}/chain_clean.jsonl')
             if r['arm'] in ('dwa_full', 'dwa_no_m3')]
    noise = load(f'{a.raw}/chain_noise.jsonl')
    ros_c = [r for f in sorted(glob.glob(f'{a.raw}/ros_heldout_w*.jsonl')) for r in load(f)]
    ros_n = [r for f in sorted(glob.glob(f'{a.raw}/ros_noise_w*.jsonl')) for r in load(f)]
    o = {'ident': {
        'clean': check_identical(f'{a.traced_raw}/clean_traced.jsonl',
                                 f'{a.raw}/chain_clean.jsonl'),
        'noise': check_identical(f'{a.traced_raw}/noise_traced.jsonl',
                                 f'{a.raw}/chain_noise.jsonl')}}
    m3_decomposition(clean, a.traces, goals, a.probe, o)
    np_ = collections.OrderedDict()
    chain_all = load(f'{a.raw}/chain_clean.jsonl')
    for arm in ('dwa_full', 'dwa_textbook', 'pp'):
        np_[('chain_sim', arm, '전체')] = noise_effect(chain_all, noise, arm)
        np_[('chain_sim', arm, '12 부분집합')] = noise_effect(chain_all, noise, arm, NOISE_IDS)
    for arm in ('dwa_full', 'dwa_textbook', 'pp', 'DWB_D00', 'DWB_D02', 'MPPI_P00', 'MPPI_P10',
                'RPP_R00', 'RPP_R03'):
        np_[('ROS', arm, '12 부분집합')] = noise_effect(ros_c, ros_n, arm)
    o['noise_path'] = np_
    o['noise_runs'] = {
        'ROS (12 × 5)': summarize_runs(run_ratios(ros_c, ros_n, 'dwa_full')),
        'chain_sim (60 × 5)': summarize_runs(run_ratios(chain_all, noise, 'dwa_full'))}
    o['noise_decomp'], rows = noise_decomposition(chain_all, noise, a.traces, goals)
    os.makedirs(a.out, exist_ok=True)
    write_md(o, f'{a.out}/review_fix.md')
    js = dict(o, noise_path={' / '.join(k): v for k, v in o['noise_path'].items()},
              noise_decomp_rows=rows)
    with open(f'{a.out}/review_fix.json', 'w', encoding='utf-8') as f:
        json.dump(js, f, ensure_ascii=False, indent=1)


if __name__ == '__main__':
    main()
