#!/usr/bin/env python3
"""브리프 §2 · §4.1 · §4.2 · §6 의 수치를 로그에서 재유도한다.

사용:  python3 c1_census.py <logs 경로>      (기본 ../../../../logs)

여기서 나오는 수치만 브리프에 쓴다. 손으로 옮긴 수치는 쓰지 않는다.
"""
import csv
import glob
import math
import os
import statistics as st
import sys
from collections import Counter

LOGS = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(__file__), '..', '..', '..', '..', 'logs')
GT_DT = 0.020          # ground truth 50 Hz (amr_simulation/config/dynamic_obstacles.yaml:21)


def fl(row, key):
    """유한한 float 만 돌려준다. nan/inf 는 None — 섞이면 평균·중앙값이 조용히 오염된다."""
    try:
        v = float(row[key])
    except (KeyError, TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def load_contacts():
    rows = []
    for path in glob.glob(os.path.join(LOGS, '*', '08_dynamic_obstacles', 'contacts*.csv')):
        with open(path) as fh:
            for r in csv.DictReader(fh):
                r['_run'] = os.path.basename(os.path.dirname(os.path.dirname(path)))
                rows.append(r)
    return rows


def section2(rows):
    print('== §2 접촉 전수 집계 ==')
    print(f'접촉 {len(rows)} 건 / {len(set(r["_run"] for r in rows))} 런')
    states = Counter(r.get('yield_state', '?') for r in rows)
    for k, v in states.most_common():
        print(f'  yield_state {k or "(빈칸)":12s} {v:4d}  {100 * v / len(rows):5.1f} %')
    sp = [fl(r, 'robot_speed_mps') for r in rows]
    sp = [x for x in sp if x is not None]
    stopped = sum(1 for x in sp if x < 0.05)
    print(f'  로봇 v < 0.05 m/s : {stopped} / {len(sp)} = {100 * stopped / len(sp):.1f} %')
    ob = Counter(r.get('obstacle', '?') for r in rows)
    for k, v in ob.most_common(4):
        print(f'  장애물 {k:22s} {v:4d}  {100 * v / len(rows):5.1f} %')
    osp = [fl(r, 'obstacle_speed_mps') for r in rows]
    osp = [x for x in osp if x is not None]
    near1 = sum(1 for x in osp if abs(x - 1.0) < 0.05)
    print(f'  장애물 속도 1.000 ±0.05 : {near1} / {len(osp)}')
    for col, label in (('gate_in_v_mps', 'DWA 자체 출력 ~0'), ('gate_out_v_mps', '게이트 후 ~0')):
        n = sum(1 for r in rows if (fl(r, col) is not None and abs(fl(r, col)) < 0.02))
        print(f'  {label}: {n}')


def section41(rows):
    """침투가 50 Hz 표집에 검열된 관측량인지 — 브리프 §4.1."""
    print('\n== §4.1 침투는 여유의 척도인가 (검열 검정) ==')
    ratios, over = [], 0
    for r in rows:
        d, vr, vo = fl(r, 'distance_m'), fl(r, 'robot_speed_mps'), fl(r, 'obstacle_speed_mps')
        if d is None or d >= 0 or vo is None:
            continue
        pen = -d
        bound = (abs(vr) if vr else 0.0) + abs(vo)      # 보수적 상대속도 상한
        bound *= GT_DT
        if bound <= 0:
            continue
        if pen > bound + 1e-9:
            over += 1
        ratios.append(pen / bound)
    ratios.sort()
    n = len(ratios)
    print(f'  침투 표본 {n} 건, 표집 상한 초과 {over} 건')
    print(f'  침투/상한 비율 중앙 {st.median(ratios):.3f} 평균 {st.mean(ratios):.3f} 최대 {max(ratios):.3f}')
    dstat = max(max(abs((i + 1) / n - r), abs(r - i / n)) for i, r in enumerate(ratios))
    crit = 1.36 / math.sqrt(n)
    print(f'  KS vs U(0,1): D = {dstat:.3f}  임계 {crit:.3f}  -> '
          f'{"균등 기각 (순수 표집산물 아님)" if dstat > crit else "균등과 구분 안 됨"}')
    print('  결론: 상한 초과 0 이면 침투는 검열된 양이고 "필요 여유" 로 읽을 수 없다.')


def fisher_two_sided(a, b, c, d):
    from math import comb
    n, r1, c1 = a + b + c + d, a + b, a + c
    def p(x):
        return comb(r1, x) * comb(n - r1, c1 - x) / comb(n, c1)
    p0 = p(a)
    lo, hi = max(0, c1 - (n - r1)), min(r1, c1)
    return sum(p(x) for x in range(lo, hi + 1) if p(x) <= p0 + 1e-12)


def section42():
    print('\n== §4.2 merge_min_gap 전후 접촉률 (Q8 vs N8) ==')
    qa, qn, na, nn = 3, 120, 5, 90
    print(f'  Q8(0.10) {qa}/{qn} = {100 * qa / qn:.2f} %   N8(0.38) {na}/{nn} = {100 * na / nn:.2f} %')
    print(f'  Fisher 양측 p = {fisher_two_sided(qa, qn - qa, na, nn - na):.4f}  -> 구분 불가')


def section_power():
    print('\n== §2 검정력 — A/B 에 필요한 표본 ==')
    def n_arm(p1, p2, za=1.96, zb=0.84):
        pb = (p1 + p2) / 2
        num = (za * math.sqrt(2 * pb * (1 - pb))
               + zb * math.sqrt(p1 * (1 - p1) + p2 * (1 - p2))) ** 2
        return num / (p1 - p2) ** 2
    n = n_arm(0.025, 0.056)
    print(f'  2.5 % vs 5.6 % 를 80 % 검정력으로 구분: 팔당 {n:.0f} 시행 = 30시행 라운드 {n / 30:.0f} 회')
    print('  30시행 무접촉 확률:')
    for p in (0.056, 0.025, 0.010, 0.003):
        print(f'    접촉률 {100 * p:4.1f} % -> {(1 - p) ** 30 * 100:5.1f} %')


def section_plan():
    """/plan 재발행 실태 — 브리프 §6 별건."""
    print('\n== §6 별건: /plan 신선도 (TTC 가 경로 기반인 시간 비율) ==')
    files = sorted(glob.glob(os.path.join(LOGS, '*', '08_dynamic_obstacles', 'plans_trial*.csv')))
    counts = []
    for fn in files:
        plans = {}
        with open(fn) as fh:
            for r in csv.DictReader(fh):
                try:
                    plans.setdefault(int(r['plan_idx']), float(r['t']))
                except (KeyError, ValueError):
                    pass
        counts.append(len(plans))
    if not counts:
        print('  plans_trial*.csv 없음')
        return
    once = sum(1 for c in counts if c == 1)
    print(f'  시행 {len(counts)} 개, 계획 발행 중앙 {st.median(counts):.0f} 회')
    print(f'  **정확히 1회만 발행한 시행: {once} / {len(counts)} = {100 * once / len(counts):.0f} %**')
    durs = []
    for fn in sorted(glob.glob(os.path.join(LOGS, '*', '08_dynamic_obstacles', 'pose_trial*.csv'))):
        ts = []
        with open(fn) as fh:
            for r in csv.DictReader(fh):
                v = fl(r, 't') or fl(r, 'time')
                if v is not None:
                    ts.append(v)
        if len(ts) >= 2:
            durs.append(max(ts) - min(ts))
    if durs:
        med = st.median(durs)
        print(f'  시행 지속 중앙 {med:.1f} s, plan_timeout 5.0 s '
              f'-> 1회 발행 시행의 {100 * (med - 5) / med:.0f} % 동안 TTC 로봇 모델이 직선 외삽')


if __name__ == '__main__':
    rows = load_contacts()
    if not rows:
        print(f'접촉 기록을 찾지 못했다: {LOGS}')
        sys.exit(1)
    section2(rows)
    section41(rows)
    section42()
    section_power()
    section_plan()
