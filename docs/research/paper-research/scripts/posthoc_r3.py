#!/usr/bin/env python3
"""
3 라운드 **사후** 분해 (prereg_r3 §5 "사후 분해도 같은 방식으로"). 확증이 아니다. 1·2 라운드와 같은 식
(posthoc_x 의 구간 정의, full_r1r2 의 표 함수)을 held-out D 에 쓴다.

    python3 posthoc_r3.py --runs /home/CAPYI_AI/paper_research/runs --out <dir>

내용: (1) 구간 분해 — DWA-R3 대 RPP 선택·dwa_full·DWA-R2, DWA-R2 대 RPP (경로 종류별), (2) 첫 진입 yaw 오차,
(3) 0→0.9 m/s (세 라운드), (4) ROS C1 실행당 재설정 (국면별, 플러그인 기록: dwa_R·dwa_R2·dwa_R3), (5) O 원인 (세 라운드).
"""
import argparse
import collections
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze_x as ax  # noqa: E402
import full_r1r2 as fr  # noqa: E402

fr.ROUNDS['r3'] = {'stage': 'r3/', 'paths': 'r3/paths_d', 'sel': 'r3/tuning/selection_r3.json',
                   'R': 'dwa_R3', 'tun': 'r3/tuning/raw'}


def resets_c1(R, rel, arms):
    recs = ax.load_dir(os.path.join(R, rel, 'raw'))
    lines = [f'#### ROS 실행당 재설정 (국면별, {rel} — 연구 플러그인 기록)', '',
             '| 팔 | 실행 | 재설정/실행 | accel | decel | turn·cruise | 출발 2 s 안 accel |',
             '| --- | --- | --- | --- | --- | --- | --- |']
    for arm in arms:
        n = 0
        ph = collections.Counter()
        early = 0
        for r in recs:
            if r['arm'] != arm:
                continue
            rs = ax.read_plugin_resets(R, rel, r)
            if rs is None:
                continue
            n += 1
            ph.update(x.get('phase', '?') for x in rs)
            early += sum(1 for x in rs if x.get('phase') == 'accel' and x['t_rel'] <= 2.0)
        if n:
            tot = sum(ph.values())
            lines.append(f'| {arm} | {n} | {tot / n:.2f} | {ph["accel"] / n:.2f} | {ph["decel"] / n:.2f} | '
                         f'{(ph["turn"] + ph["cruise"]) / n:.2f} | {early / n:.2f} |')
    return '\n'.join(lines) + '\n'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    R = args.runs
    sel3 = fr.sel_of(R, 'r3')
    rpp3 = sel3['RPP']['chosen']
    md = ['# 3 라운드 사후 분해 (held-out D) — 확증 아님', '',
          '1·2 라운드와 같은 방법이다 (`posthoc_x.py` 구간, `full_r1r2.py` 표). 값은 기준 팔 전체 시간 대비 %p.', '']
    md += ['## 1. 구간 분해 (ROS C1)', '']
    Z = fr.zones(R, 'r3', 'c1', ('dwa_R3', 'dwa_R2', 'dwa_R', 'dwa_full', rpp3, 'RPP_R00', 'pp'))
    md.append(fr.decomp_table(Z, 'dwa_R3', rpp3, '3 라운드 (held-out D)'))
    md.append(fr.decomp_table(Z, 'dwa_R3', 'dwa_R2', '3 라운드 (held-out D)'))
    md.append(fr.decomp_table(Z, 'dwa_R3', 'dwa_full', '3 라운드 (held-out D)'))
    md.append(fr.decomp_table(Z, 'dwa_R2', rpp3, '3 라운드 (held-out D)'))
    md.append(fr.decomp_table(Z, 'pp', rpp3, '3 라운드 (held-out D)'))
    md += ['## 2. 목표 첫 진입 yaw 오차', '']
    md.append(fr.entry_yaw(R, 'r3', 'c1', ('dwa_full', 'dwa_R', 'dwa_R2', 'dwa_R3', rpp3, 'pp')))
    md += ['## 3. 출발 가속 (세 라운드)', '']
    labs = {}
    for rnd, rel in (('1R C1', 'c1'), ('2R C1', 'r2/c1'), ('3R C1', 'r3/c1')):
        s = fr.sel_of(R, {'1R C1': 'r1', '2R C1': 'r2', '3R C1': 'r3'}[rnd])
        labs[rnd] = ax.substitute_tuned(fr.load_stage(R, rel), s)
    md.append(fr.accel_table(labs, ['dwa_full', 'dwa_R', 'dwa_R2', 'dwa_R3', 'RPP_R00', 'RPP_tuned', 'pp',
                                    'DWB_tuned', 'MPPI_tuned']))
    md += ['## 4. 재설정 (ROS)', '']
    md.append(resets_c1(R, 'r2/c1', ('dwa_R', 'dwa_R2')))
    md.append(resets_c1(R, 'r3/c1', ('dwa_R', 'dwa_R2', 'dwa_R3')))
    md += ['## 5. O 원인 (세 라운드)', '']
    ol = {}
    for rnd, rel, key in (('1R', 'o', 'r1'), ('2R', 'r2/o', 'r2'), ('3R', 'r3/o', 'r3')):
        ol[rnd] = ax.substitute_tuned(fr.load_stage(R, rel), fr.sel_of(R, key))
    md.append(fr.o_table(ol, ['dwa_full', 'dwa_R', 'dwa_R2', 'dwa_R3', 'MPPI_tuned', 'RPP_tuned',
                              'DWB_tuned', 'pp', 'dwa_textbook_tuned']))
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, 'posthoc_r3.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(md) + '\n')
    print('\n'.join(md))


if __name__ == '__main__':
    main()
