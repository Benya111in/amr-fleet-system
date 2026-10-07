#!/usr/bin/env python3
"""
C4 동적 장애물 스트레스 집계 (사전 등록: 노트 `2026-10-07_1137_b1-gzc_c4-dynamic-stress.md`) — 팔별 수만, 짝 비교 없음.

    analyze_c4.py --c4 /home/CAPYI_AI/runs/bonus1/b1-gzc/c4 --out <json>   (마크다운 표는 stdout)
"""

import argparse
import csv
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import analyze_c as A  # noqa: E402

ENCOUNTER_M = 2.0


def load(c4, arm):
    """팔 하나의 exec_index → 행 (첫 시도 우선)."""
    rows = {}
    for a in A.attempts(os.path.join(c4, 'runs', arm)):
        f = os.path.join(a, 'gzbench.csv')
        if os.path.exists(f):
            with open(f, encoding='utf-8', newline='') as fh:
                for r in csv.DictReader(fh):
                    rows.setdefault(int(r['exec_index']), r)
    return [rows[k] for k in sorted(rows)]


def summary(rows):
    """팔 요약."""
    ok = [r for r in rows if A.excl(r) != 'E1']
    n = len(rows)
    dyn = [r for r in ok if A.num(r.get('contacts_dynamic')) > 0]
    sta = [r for r in ok if A.num(r.get('contacts_static')) > 0]
    enc = [r for r in ok if A.num(r.get('min_clear_dynamic_m')) <= ENCOUNTER_M]
    out = {'n': n, 'excl': {k: sum(1 for r in rows if A.excl(r) == k)
                            for k in ('E1', 'E2', 'E3', 'E4')},
           'contact_dynamic_eps': {'x': len(dyn), 'n': len(ok),
                                   'wilson': A.wilson(len(dyn), len(ok)),
                                   'ids': [r['scenario_id'] for r in dyn]},
           'contact_static_eps': {'x': len(sta), 'n': len(ok),
                                  'wilson': A.wilson(len(sta), len(ok))},
           'encounters_le2m': len(enc),
           'min_clear_dynamic_q': A.q([A.num(r.get('min_clear_dynamic_m')) for r in ok]),
           'outcome_cross': {}}
    for r in rows:
        k = f"{r.get('outcome_contract')}|{r.get('outcome_system')}"
        out['outcome_cross'][k] = out['outcome_cross'].get(k, 0) + 1
    for col in A.RATE_COLS:
        x = sum(A.success(r, col, 'all') for r in rows)
        out[col + '_all'] = {'x': x, 'n': n, 'wilson': A.wilson(x, n)}
    for col in ('interventions', 'zone_stop_s'):
        v = [A.num(r.get(col)) for r in ok]
        v = [x for x in v if x == x]
        out[col] = {'mean': sum(v) / len(v) if v else None, 'sum': sum(v) if v else None}
    for col in ('max_dev_m', 'return_max_s', 'nav_time_s', 'rtf'):
        out[col + '_q'] = A.q([A.num(r.get(col)) for r in ok], (5, 50, 95, 100))
    return out


def main():
    """명령행."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--c4', default='/home/CAPYI_AI/runs/bonus1/b1-gzc/c4')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    res = {arm: summary(load(a.c4, arm)) for arm in ('dwa', 'rl')}
    with open(a.out, 'w', encoding='utf-8') as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    lines = ['| 팔 | n | 제외 E1/E2/E3/E4 | 동적 접촉 에피소드 | 정적 접촉 에피소드 | 조우 ≤ 2 m | '
             '동적 최소 이격 5/50/95/100 % [m] | strict_gt | nav2compat | Nav2 성공 | 개입 평균 | '
             '최대 이탈 5/50/95/100 % [m] | 복귀 최대 5/50/95/100 % [s] |',
             '|' + '---|' * 13]
    for arm, s in res.items():
        ex = '/'.join(str(s['excl'][k]) for k in ('E1', 'E2', 'E3', 'E4'))
        rt = (lambda e: f"{e['x']}/{e['n']}")
        im = s['interventions']['mean']
        cells = [arm.upper(), s['n'], ex, rt(s['contact_dynamic_eps']),
                 rt(s['contact_static_eps']),
                 s['encounters_le2m'],
                 ' / '.join('—' if v is None else f'{v:.2f}' for v in s['min_clear_dynamic_q']),
                 rt(s['strict_gt_all']), rt(s['nav2compat_gt_all']), rt(s['system_success_all']),
                 '—' if im is None else f'{im:.1f}',
                 ' / '.join('—' if v is None else f'{v:.2f}' for v in s['max_dev_m_q']),
                 ' / '.join('—' if v is None else f'{v:.1f}' for v in s['return_max_s_q'])]
        lines.append('| ' + ' | '.join(str(c) for c in cells) + ' |')
    for arm, s in res.items():
        lines.append(f"\n- {arm.upper()} 결과 분류: " + ', '.join(
            f'{k} {v}' for k, v in sorted(s['outcome_cross'].items())))
    sys.stdout.write('\n'.join(lines) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
