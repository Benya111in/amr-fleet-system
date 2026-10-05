#!/usr/bin/env python3
"""R0 판정 — 브리프 §6 의 결정 관문을 자료로 답한다.

사용:  python3 c2_r0.py <logs/R0a 같은 실행 경로> [...]

답해야 할 것 (브리프 §3 의 가설을 주기 단위로 검정한다):
  G1  argmin 이 평탄한가          — cost_span 이 0 에 가까운 주기의 비율
  G2  후보가 서로 구별되는가      — disp_span 분포 (§3.1 은 v≈0 에서 0.075 m 를 예측)
  G3  창이 붕괴하는가             — v_hi - v_lo 분포
  G4  VO 가 전면 포화하는가       — n_selectable == 0 인 주기의 비율
  G5  **조기 반환이 지배하는가**  — nearest_obs < R_vo 인 주기의 비율 ← 결정 관문
  G6  게이트가 계획기를 지우는가  — planner_v > 0.05 인데 cmd_v < 0.05 인 주기

G5 가 "이미 포화" 로 나오면 R1/R2 의 값어치가 0 이고 R3(참 호 접촉 판정)이 먼저다.
"""
import csv
import glob
import math
import os
import statistics as st
import sys

R_VO = 0.361 + 0.25 + 0.30      # robot_radius + obstacle radius + vo_margin (nav2_params.yaml)
STOPPED = 0.05                  # [m/s] 사실상 정지
ENGAGED = 3.0                   # [m] 이 안이면 "조우 중" — 결정이 실제로 필요한 구간


def load(paths):
    rows = []
    for base in paths:
        for fn in sorted(glob.glob(os.path.join(base, '08_dynamic_obstacles', 'stats_trial*.csv'))):
            trial = os.path.basename(fn)
            with open(fn) as fh:
                for r in csv.DictReader(fh):
                    r['_trial'] = trial
                    r['_run'] = os.path.basename(base)
                    rows.append(r)
    return rows


def num(r, k):
    try:
        v = float(r[k])
    except (KeyError, TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def pct(n, d):
    return f'{100 * n / d:.1f} %' if d else 'n/a'


def quant(xs, qs=(0.05, 0.5, 0.95)):
    if not xs:
        return 'n/a'
    xs = sorted(xs)
    return ' / '.join(f'{xs[min(len(xs) - 1, int(q * len(xs)))]:.4f}' for q in qs)


def main(paths):
    rows = load(paths)
    if not rows:
        print('stats_trial*.csv 를 찾지 못했다. R0 계측이 들어간 실행인지 확인하라.')
        return 1
    if 'cost_span' not in rows[0]:
        print('R0 열이 없다 — 옛 실행이다.')
        return 1

    trials = len({(r['_run'], r['_trial']) for r in rows})
    print(f'주기 {len(rows)} 개 / 시행 {trials} 개 / 실행 {len({r["_run"] for r in rows})} 개')
    print(f'  (접촉은 30 시행당 약 1 건 — 여기서는 표본이 {len(rows) // max(1, trials)} 배 많다)\n')

    # 위험 구간만 따로 본다: 동적 장애물이 R_vo 안에 있고 로봇이 거의 멈춘 주기
    # "조우 중" = 동적 장애물이 ENGAGED 안. 결정이 실제로 필요한 구간이고,
    # 여기서 R_vo 안 비율을 물어야 한다. (예전 판에서는 위험 구간을 R_vo 로 정의해 놓고
    # 그 안에서 R_vo 비율을 물어 동어반복이었다.)
    eng = [r for r in rows if 0 <= (num(r, 'nearest_obs_m') if num(r, 'nearest_obs_m') is not None
                                    else -1) < ENGAGED]
    near = [r for r in eng if (num(r, 'nearest_obs_m') or 1e9) < R_VO]
    slow = [r for r in rows if abs(num(r, 'cmd_v') or 0.0) < STOPPED]
    crit = [r for r in eng if abs(num(r, 'cmd_v') or 0.0) < STOPPED]
    print(f'조우 중 주기 (동적 장애물 < {ENGAGED:.1f} m): {len(eng)} ({pct(len(eng), len(rows))})')
    print(f'  그중 R_vo = {R_VO:.3f} m 안             : {len(near)} ({pct(len(near), len(eng))})'
          '   <- 조기 반환 지배 여부')
    print(f'로봇이 사실상 정지한 주기            : {len(slow)} ({pct(len(slow), len(rows))})')
    print(f'**조우 중 + 정지 (위험 구간)**       : {len(crit)} ({pct(len(crit), len(rows))})\n')

    def block(label, sample):
        if not sample:
            print(f'-- {label}: 표본 없음\n')
            return
        n = len(sample)
        print(f'-- {label} (n = {n}) --')
        cs = [num(r, 'cost_span') for r in sample]
        cs = [x for x in cs if x is not None and x >= 0]
        flat = sum(1 for x in cs if x < 1e-6)
        print(f'  G1 cost_span  p05/p50/p95 = {quant(cs)}   0 에 붙은 주기 {pct(flat, len(cs))}')
        ds = [num(r, 'disp_span_m') for r in sample]
        ds = [x for x in ds if x is not None]
        print(f'  G2 disp_span  p05/p50/p95 = {quant(ds)} m')
        vs = [(num(r, 'v_hi') or 0) - (num(r, 'v_lo') or 0) for r in sample]
        print(f'  G3 창 폭 v    p05/p50/p95 = {quant(vs)} m/s')
        ns = [num(r, 'n_selectable') for r in sample]
        ns = [x for x in ns if x is not None]
        zero = sum(1 for x in ns if x == 0)
        print(f'  G4 n_selectable == 0 (VO 전면 포화) : {zero} / {len(ns)} = {pct(zero, len(ns))}')
        no = [num(r, 'nearest_obs_m') for r in sample]
        no = [x for x in no if x is not None and x >= 0]
        inside = sum(1 for x in no if x < R_VO)
        print(f'  G5 nearest_obs < R_vo               : {inside} / {len(no)} = {pct(inside, len(no))}'
              f'   (거리 p05/p50/p95 = {quant(no)} m)')
        killed = sum(1 for r in sample
                     if (num(r, 'planner_v') or 0) > STOPPED and abs(num(r, 'cmd_v') or 0) < STOPPED)
        print(f'  G6 계획기가 냈는데 하류가 지운 주기 : {killed} ({pct(killed, n)})')
        esc = sum(1 for r in sample if (num(r, 'escaping') or 0) > 0.5)
        ll = sum(1 for r in sample if (num(r, 'leave_lane') or 0) > 0.5)
        print(f'     escaping {pct(esc, n)}   leave_lane {pct(ll, n)}\n')

    block('전체 주기', rows)
    block(f'조우 중 (< {ENGAGED:.1f} m)', eng)
    block('위험 구간 (조우 중 + 정지)', crit)

    print('== 결정 관문 ==')
    if crit:
        no = [num(r, 'nearest_obs_m') for r in eng]
        no = [x for x in no if x is not None and x >= 0]
        inside = sum(1 for x in no if x < R_VO) / len(no) if no else 0
        ns = [num(r, 'n_selectable') for r in crit]
        sat = sum(1 for x in ns if x == 0) / len(ns) if ns else 0
        ds = [num(r, 'disp_span_m') for r in crit]
        ds = [x for x in ds if x is not None]
        med_disp = st.median(ds) if ds else float('nan')
        print(f'  조우 중 주기 가운데 R_vo 안 비율  = {100 * inside:.1f} %  (동어반복 아님)')
        print(f'  위험 구간에서 VO 전면 포화 비율   = {100 * sat:.1f} %')
        print(f'  위험 구간 disp_span 중앙          = {med_disp:.4f} m')
        if inside > 0.8:
            print('  -> **조기 반환이 지배한다. R3(참 호 접촉 판정)이 먼저다.**')
            print('     R1/R2 로 후보를 넓혀도 술어가 반경 안에서 정보를 주지 않는다.')
        elif med_disp < 0.15:
            print('  -> **후보 표현력이 결손이다. R1+R2 가 먼저다.**')
        else:
            print('  -> 둘 다 지배적이지 않다. 브리프 §6 을 다시 본다.')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:] or glob.glob(os.path.join(
        os.path.dirname(__file__), '..', '..', '..', '..', 'logs', 'R0*'))))
