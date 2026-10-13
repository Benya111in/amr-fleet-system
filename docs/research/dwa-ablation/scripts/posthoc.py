#!/usr/bin/env python3
"""
사후 분석 (사전 등록 analyze.py 밖 — results.md 「사전 등록과 다르게 한 것」 에 적는다).

    python3 posthoc.py --raw /runs/raw --out <dir>

1. 좁은 통로 "통로 안 관통": 성공 ∧ 참값 최소 여유 ≤ (통로 폭 − 0.40)/2 + 0.02 m (통로 안을 지났다면
   여유가 반드시 이 값 이하). 사전 등록은 관통 = 성공으로 정했는데, 랙 바깥으로 돌아 목표에 닿은
   실행(최소 여유 약 0.42 m)도 성공으로 세어져 관통을 과대평가한다. CTE 최대로 가르면 통로를 지난 뒤
   목표에서 되돌아 돈 실행까지 섞여 쓰지 않는다.
2. 실패 원인 표 (팔 × 집합 × 원인).
3. 도착 시간 예측 오차 |T − T_pred| / T (사전 등록 §5 의 보조 지표 — analyze.py 에서 빠졌다).
4. ROS 실행의 실제 명령 주기·부하·계산 시간 분포.
"""
import argparse
import collections
import json
import os

import numpy as np

from analyze import HELDOUT, load, substitute_tuned
from make_paths import NARROW_CASES

WIDTH = {c[0]: c[1] for c in NARROW_CASES}

ARMS = ['dwa_full', 'dwa_textbook', 'pp', 'DWB_D00', 'DWB_tuned', 'MPPI_P00', 'MPPI_tuned',
        'RPP_R00', 'RPP_tuned', 'dwa_no_m1', 'dwa_no_m1c', 'dwa_no_m1e', 'dwa_no_m2',
        'dwa_m2_trap', 'dwa_no_m3', 'dwa_no_m4', 'dwa_no_m4a', 'dwa_no_m5']


def narrow_pass(recs, title):
    lines = [f'### {title}', '',
             '| 팔 | 성공 (사전 등록 "관통") | 통로 안 관통 | 우회 성공 | 실패 |',
             '| --- | --- | --- | --- | --- |']
    for arm in ARMS:
        rs = [r for r in recs if r['arm'] == arm and r['family'] == 'narrow']
        if not rs:
            continue
        ok = sum(r['success'] for r in rs)
        inside = sum(1 for r in rs if r['success'] and 0.0 <= r['min_clear'] <=
                     (WIDTH[r['scene']] - 0.40) / 2 + 0.02)
        lines.append(f'| {arm} | {ok}/{len(rs)} | {inside}/{len(rs)} | {ok - inside} | '
                     f'{len(rs) - ok} |')
    return '\n'.join(lines) + '\n'


def failures(recs, title):
    lines = [f'### {title}', '', '| 팔 | 매끈 | 좁은 통로 | A* |', '| --- | --- | --- | --- |']
    for arm in ARMS:
        rs = [r for r in recs if r['arm'] == arm and r['family'] in HELDOUT]
        if not rs:
            continue
        cells = []
        for fam in HELDOUT:
            c = collections.Counter(
                (r['fail'] if r['fail'] != 'none' else 'collision') for r in rs
                if r['family'] == fam and not r['success'])
            n = sum(1 for r in rs if r['family'] == fam)
            cells.append(', '.join(f'{k} {v}' for k, v in sorted(c.items())) + f' (/{n})'
                         if c else f'0 (/{n})')
        lines.append(f'| {arm} | ' + ' | '.join(cells) + ' |')
    return '\n'.join(lines) + '\n'


def eta(recs, title):
    lines = [f'### {title}', '',
             '보조 지표. 예측 모형(SpeedProfile)은 우리 DWA·PP 에 맞춰져 있어 우리 팔에 유리하게 편향된다.', '',
             '| 팔 | 성공 실행 수 | |T − T_pred| / T 중앙 [%] | p90 [%] |', '| --- | --- | --- | --- |']
    for arm in ARMS:
        e = [abs(r['t_reach'] - r['t_pred']) / r['t_reach'] for r in recs
             if r['arm'] == arm and r['success'] and r['family'] in HELDOUT and r['t_reach'] > 0]
        if e:
            lines.append(f'| {arm} | {len(e)} | {100 * np.median(e):.1f} | '
                         f'{100 * np.percentile(e, 90):.1f} |')
    return '\n'.join(lines) + '\n'


def ros_timing(recs, title):
    lines = [f'### {title}', '',
             '| 팔 | 실행 | 명령 주기 중앙 [Hz] | 18 Hz 미만 실행 | 계산 CPU 중앙 / p90 [ms/주기] | '
             '우리 팔 cycle_ms 중앙 / 최대 | loadavg(1 분) 범위 |',
             '| --- | --- | --- | --- | --- | --- | --- |']
    for arm in ARMS:
        rs = [r for r in recs if r['arm'] == arm]
        if not rs:
            continue
        hz = [r['cmd_rate_hz'] for r in rs]
        cpu = [r['cpu_ms_per_cycle'] for r in rs if r['cpu_ms_per_cycle'] >= 0]
        cyc = [r['cycle_us_mean'] / 1000 for r in rs if r['cycle_us_mean'] > 0]
        cmx = [r['cycle_us_max'] / 1000 for r in rs if r['cycle_us_max'] > 0]
        ld = [r['load_start'][0] for r in rs] + [r['load_end'][0] for r in rs]
        own = f'{np.median(cyc):.2f} / {max(cmx):.2f}' if cyc else '–'
        lines.append(
            f'| {arm} | {len(rs)} | {np.median(hz):.1f} | {sum(1 for h in hz if h < 18)} | '
            f'{np.median(cpu):.2f} / {np.percentile(cpu, 90):.2f} | {own} | '
            f'{min(ld):.0f}–{max(ld):.0f} |')
    return '\n'.join(lines) + '\n'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--raw', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    chosen_f = os.path.join(args.raw, 'tuning_selection.json')
    with open(chosen_f, encoding='utf-8') as f:
        chosen = json.load(f)
    ros = substitute_tuned(load(os.path.join(args.raw, 'ros_heldout.jsonl')), chosen)
    rosn = substitute_tuned(load(os.path.join(args.raw, 'ros_noise.jsonl')), chosen)
    cc = load(os.path.join(args.raw, 'chain_clean.jsonl'))
    cn = load(os.path.join(args.raw, 'chain_noise.jsonl'))
    md = [narrow_pass(ros, '사후 1a. 좁은 통로 관통 — ROS held-out'),
          narrow_pass(cc, '사후 1b. 좁은 통로 관통 — chain_sim held-out 잡음 없음'),
          narrow_pass(cn, '사후 1c. 좁은 통로 관통 — chain_sim 잡음 (8 통로 × seed 5)'),
          narrow_pass(rosn, '사후 1d. 좁은 통로 관통 — ROS 잡음 (3 통로 × seed 5)'),
          failures(ros, '사후 2a. 실패 원인 — ROS held-out'),
          failures(cc, '사후 2b. 실패 원인 — chain_sim held-out 잡음 없음'),
          failures(cn, '사후 2c. 실패 원인 — chain_sim 잡음 (seed 5 합)'),
          failures(rosn, '사후 2d. 실패 원인 — ROS 잡음 (seed 5 합)'),
          eta(ros, '사후 3a. 도착 시간 예측 오차 — ROS held-out'),
          eta(cc, '사후 3b. 도착 시간 예측 오차 — chain_sim held-out'),
          ros_timing(ros, '사후 4a. ROS 실행 주기·부하·계산 — held-out'),
          ros_timing(rosn, '사후 4b. ROS 실행 주기·부하·계산 — 잡음')]
    with open(os.path.join(args.out, 'posthoc.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(md))
    print('\n'.join(md))


if __name__ == '__main__':
    main()
