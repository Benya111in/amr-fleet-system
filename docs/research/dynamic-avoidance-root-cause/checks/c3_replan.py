#!/usr/bin/env python3
"""명세 4.7 "경로를 재계획(Replanning)하여 회피" 가 **실제로 동작하는가**.

사용:  python3 c3_replan.py <끈 실행...> -- <켠 실행...>
  예)  python3 c3_replan.py logs/R25a logs/R25c logs/R25d -- logs/RP2a

구현했다고 충족이 아니다. 사슬이 발화해야 한다:
  1. TTC 가 **경로 기반**인가          — plan 재발행 간격으로 간접 확인
  2. TTC 가 admit_ttc 아래로 내려가는가 — avoidance.csv 의 min_ttc_s
  3. **/plan 이 실제로 바뀌는가**       — plans_trial*.csv 의 plan_idx 개수  <- 핵심
  4. 경로가 바뀐 시행에서 거동이 다른가 — 최근접 거리·이탈

3 이 안 되면 구현은 여전히 무효다. 앞선 설계가 실패한 지점이 정확히
"A* 가 자기를 부른 장애물을 보지 못한 채 다시 계획한다" 였다.
"""
import csv
import glob
import math
import os
import statistics as st
import sys
from math import comb


def fisher(a, b, c, d):
    n, r1, c1 = a + b + c + d, a + b, a + c
    def p(x):
        return comb(r1, x) * comb(n - r1, c1 - x) / comb(n, c1)
    p0 = p(a)
    lo, hi = max(0, c1 - (n - r1)), min(r1, c1)
    return sum(p(x) for x in range(lo, hi + 1) if p(x) <= p0 + 1e-12)


def num(row, key):
    try:
        v = float(row[key])
    except (KeyError, TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def plans_per_trial(runs):
    """시행마다 /plan 이 몇 번 발행됐는가. 1 이면 재계획이 한 번도 없었다는 뜻."""
    out = []
    for base in runs:
        for fn in sorted(glob.glob(os.path.join(base, '08_dynamic_obstacles',
                                                'plans_trial*.csv'))):
            idx = set()
            with open(fn) as fh:
                for r in csv.DictReader(fh):
                    try:
                        idx.add(int(r['plan_idx']))
                    except (KeyError, ValueError):
                        pass
            if idx:
                out.append(len(idx))
    return out


def min_ttc(runs):
    out = []
    for base in runs:
        for fn in glob.glob(os.path.join(base, '08_dynamic_obstacles', 'avoidance.csv')):
            with open(fn) as fh:
                for r in csv.DictReader(fh):
                    v = num(r, 'min_ttc_s')
                    if v is not None and v >= 0:
                        out.append(v)
    return out


def contacts(runs):
    n = 0
    for base in runs:
        for fn in glob.glob(os.path.join(base, '08_dynamic_obstacles', 'contacts*.csv')):
            with open(fn) as fh:
                n += len(list(csv.DictReader(fh)))
    return n


def report(label, runs, admit_ttc):
    pp = plans_per_trial(runs)
    tt = min_ttc(runs)
    print(f'\n== {label} ({len(runs)} 실행) ==')
    if not pp:
        print('  plans_trial*.csv 없음')
        return None
    once = sum(1 for n in pp if n == 1)
    multi = sum(1 for n in pp if n > 1)
    print(f'  시행 {len(pp)} · 접촉 {contacts(runs)}')
    print(f'  [3] /plan 발행 횟수: 중앙 {st.median(pp):.0f} · 최대 {max(pp)}')
    print(f'      **1 회뿐인 시행 {once}/{len(pp)} = {100 * once / len(pp):.0f} %**'
          f'   재계획한 시행 {multi} ({100 * multi / len(pp):.0f} %)')
    if tt:
        tt.sort()
        below = sum(1 for x in tt if x <= admit_ttc)
        print(f'  [2] 시행별 min TTC: p05 {tt[len(tt) // 20]:.2f} · 중앙 {st.median(tt):.2f} s')
        print(f'      admit_ttc {admit_ttc} 이하인 시행 {below}/{len(tt)}'
              f' = {100 * below / len(tt):.0f} %   <- 재진입 기회')
    return {'trials': len(pp), 'once': once, 'multi': multi, 'contacts': contacts(runs)}


def main(argv):
    if '--' not in argv:
        print(__doc__)
        return 1
    k = argv.index('--')
    off, on = argv[:k], argv[k + 1:]
    admit = 2.5
    a = report('재계획 끔 (기준선)', off, admit)
    b = report('재계획 켬', on, admit)
    if not a or not b:
        return 1
    print('\n== 판정 ==')
    p = fisher(a['multi'], a['trials'] - a['multi'], b['multi'], b['trials'] - b['multi'])
    print(f"  재계획한 시행 비율: {100 * a['multi'] / a['trials']:.0f} % -> "
          f"{100 * b['multi'] / b['trials']:.0f} %   Fisher p = {p:.4f}")
    if b['multi'] > a['multi'] and p < 0.05:
        print('  -> **경로가 실제로 바뀐다. 명세 4.7 "재계획하여 회피" 가 동작한다.**')
    elif b['multi'] <= a['multi']:
        print('  -> **경로가 바뀌지 않는다. 구현이 무효다** — 사슬 어딘가가 끊겼다.')
        print('     확인 순서: predicted_obstacles 발행 -> ttc_path_based -> admit_ttc 도달')
        print('     -> 전역 코스트맵 마킹 -> A* 가 다른 경로를 내는가')
    else:
        print('  -> 방향은 맞으나 표본이 부족하다. 라운드를 더 돌린다.')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
