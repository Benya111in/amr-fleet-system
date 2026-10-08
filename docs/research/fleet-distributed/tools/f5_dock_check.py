#!/usr/bin/env python3
"""
F5 도크 배치 확인 분석 (b4-wire, 사전 등록: notebook entries/*_b4-wire_f5-dock-layout-fleetsim.md).

  python3 f5_dock_check.py --old OLD/runs.jsonl --new NEW/runs.jsonl [--p4 P4-hold-01/out/runs.jsonl] --out OUT.json

운동학 플릿 모의(fleetsim) 두 실행 묶음 — 옛 배치(fd17e49, 도크 구역 ±1 m · approach_m 1.5)와 새 배치(도크 ±0.75 m ·
approach 2.0) — 를 같은 비교군 · 고장 · 손실 · 시드로 대조한다. 표준 라이브러리만.
- 주 판정: 새 배치, hold 비교군(A1h · A5h), F0 · 손실 0 의 '도크 구역 동시 점유가 있는 실행' 수. 두 비교군 모두 0/30
  이면 '해소(운동학 모형 한정)', 아니면 '미달 — 남은 위험'.
- 보조: F6 같은 수, 옛 배치 수, 사건 수, 교차로 · 통로 사건 수(배치 변경이 다른 구역 사건을 늘리지 않았나).
- 처리량(보조): 완료 작업 수의 시드 짝 차(새 − 옛) 평균 · 부트스트랩 95 % CI(10 000, 시드 0) · 부호 수.
  CI 가 0 을 포함하면 '가르지 못함'.
- 재현 확인(선택 --p4): 옛 배치 재실행의 digest 가 P4-hold-01 의 같은 칸 digest 와 같은지.
"""

import argparse
import collections
import json
import math
import random
import sys

HOLD = ('A1h_nearest_hold', 'A5h_cbaa_hold')


def load(path):
    with open(path, encoding='utf-8') as f:
        return [json.loads(line) for line in f if line.strip()]


def key(r):
    return (r['arm'], r['fault'], float(r['loss']), int(r['seed']))


def dock_events(r):
    return sum(n for z, n in (r.get('cooccupancy_zones') or {}).items() if z.startswith('dock_'))


def other_events(r):
    return sum(n for z, n in (r.get('cooccupancy_zones') or {}).items()
               if not z.startswith('dock_'))


def zero_upper(n):
    """0 건일 때 정확 이항 95 % 단측 상한 (1 − 0.05^(1/n))."""
    return 1.0 - 0.05 ** (1.0 / n)


def boot_ci(xs, n_boot=10000, seed=0):
    rng = random.Random(seed)
    m = len(xs)
    means = sorted(sum(xs[rng.randrange(m)] for _ in range(m)) / m for _ in range(n_boot))
    return means[int(0.025 * n_boot)], means[int(0.975 * n_boot) - 1]


def cells(rows):
    out = collections.defaultdict(list)
    for r in rows:
        out[(r['arm'], r['fault'], float(r['loss']))].append(r)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--old', required=True)
    ap.add_argument('--new', required=True)
    ap.add_argument('--p4', default='')
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    old, new = load(a.old), load(a.new)
    co, cn = cells(old), cells(new)
    table = []
    for c in sorted(set(co) | set(cn)):
        ro, rn = co.get(c, []), cn.get(c, [])
        row = {'arm': c[0], 'fault': c[1], 'loss': c[2], 'n_old': len(ro), 'n_new': len(rn)}
        for tag, rs in (('old', ro), ('new', rn)):
            row[f'dock_runs_{tag}'] = sum(1 for r in rs if dock_events(r) > 0)
            row[f'dock_events_{tag}'] = sum(dock_events(r) for r in rs)
            row[f'other_events_{tag}'] = sum(other_events(r) for r in rs)
            row[f'completed_mean_{tag}'] = (sum(r['completed'] for r in rs) / len(rs)) if rs else None
        po = {int(r['seed']): r for r in ro}
        pn = {int(r['seed']): r for r in rn}
        seeds = sorted(set(po) & set(pn))
        d = [pn[s]['completed'] - po[s]['completed'] for s in seeds]
        if d:
            lo, hi = boot_ci(d)
            row['completed_diff'] = {
                'n': len(d), 'mean': sum(d) / len(d), 'ci95': [lo, hi],
                'better': sum(x > 0 for x in d), 'worse': sum(x < 0 for x in d),
                'equal': sum(x == 0 for x in d),
                'verdict': '가르지 못함' if lo <= 0.0 <= hi else ('새 배치가 많다' if lo > 0 else
                                                                  '새 배치가 적다')}
        if rn and row['dock_runs_new'] == 0:
            row['dock_zero_upper95'] = zero_upper(len(rn))
        table.append(row)
    prim = [t for t in table if t['arm'] in HOLD and t['fault'] == 'F0' and t['loss'] == 0.0]
    ok = bool(prim) and all(t['n_new'] >= 30 and t['dock_runs_new'] == 0 for t in prim)
    verdict = ('해소 (운동학 모형 한정): 새 배치 hold 비교군 F0 · 손실 0 도크 동시 점유 실행 0/30'
               if ok else '미달 — 남은 위험 (주 판정 칸에 도크 동시 점유가 남았거나 n 부족)')
    repro = None
    if a.p4:
        p4 = {key(r): r['digest'] for r in load(a.p4)}
        both = [(key(r), r['digest']) for r in old if key(r) in p4]
        repro = {'compared': len(both), 'equal': sum(p4[k] == d for k, d in both)}
    out = {'primary_cells': prim, 'primary_verdict': verdict, 'table': table,
           'old_rerun_vs_p4_digest': repro}
    with open(a.out, 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print(verdict)
    for t in table:
        cd = t.get('completed_diff') or {}
        print(f"{t['arm']:22s} {t['fault']} {t['loss']:g} 도크 실행 {t['dock_runs_old']}/{t['n_old']} → "
              f"{t['dock_runs_new']}/{t['n_new']} (사건 {t['dock_events_old']} → {t['dock_events_new']}), "
              f"그 밖 사건 {t['other_events_old']} → {t['other_events_new']}, 완료 차 "
              f"{cd.get('mean', math.nan):+.2f} {cd.get('ci95')} {cd.get('verdict')}")
    if repro:
        print(f"옛 배치 재실행 digest = P4-hold-01: {repro['equal']}/{repro['compared']}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
