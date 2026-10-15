#!/usr/bin/env python3
"""
보너스 4 P5 결과 표 — 형식만 (지표 · 판정은 p5_analyze.py).

원자료 루트의 metrics.json · analysis/<실험>.json 을 읽어 p5.md 에 붙일 마크다운 표를 낸다. 표준 라이브러리만.

  python3 p5_report.py runs <원자료 루트> [--exp C1]    갈래별 표 (완료 · makespan · 안전 · 장애 반응 · RTF · 부하)
  python3 p5_report.py events <원자료 루트>             실제 자세 동시 점유 · 허가 없는 진입 · 해제 게이트 사건 표 (분산 먼저)
  python3 p5_report.py exp <원자료 루트> <실험>          짝 비교 · 판정 요약
"""

import argparse
import json
import os
import sys


def load_runs(root, exp=None):
    out = []
    for name in sorted(os.listdir(root)):
        p = os.path.join(root, name, 'metrics.json')
        if not os.path.isfile(p):
            continue
        with open(p, encoding='utf-8') as f:
            m = json.load(f)
        m['_tag'] = name
        if exp is None or m.get('exp') == exp:
            out.append(m)
    return out


def fmt(v, nd=1):
    if v is None:
        return '—'
    if isinstance(v, float):
        return f'{v:.{nd}f}'
    return str(v)


def runs_table(ms):
    head = ('| 태그 | 모드 | 유효 | 완료 | makespan s | 원판 동시 점유 | 믿은 겹침 | 허가 없는 진입 s | 중복 픽업 | '
            '낮은 epoch 수용 | 확정 실패 | 교착 | RTF | 시작 load |')
    lines = [head, '|' + '---|' * 14]
    for m in ms:
        lines.append('| ' + ' | '.join([
            m['_tag'], str(m.get('mode')), '예' if m.get('valid') else f"아니오 ({m.get('why')})",
            f"{fmt(m.get('completed'))}/{fmt(m.get('n_tasks'))}", fmt(m.get('makespan')),
            str(len(m.get('occupancy_disc') or [])),
            str(len(m.get('held_overlaps') or {})) if m.get('mode') == 'distributed' else '—',
            fmt(round(sum((m.get('unauth_s') or {}).values()), 1))
            if m.get('mode') == 'distributed' else '—',
            str(len(m.get('duplicate_pickups') or {})),
            str(len(m.get('low_epoch_accepted') or [])),
            fmt(m.get('commit_failures')), fmt(m.get('deadlocks')), fmt(m.get('rtf'), 3),
            ' '.join((m.get('start_load') or ['?'])[:1])]) + ' |')
    return '\n'.join(lines)


def events_table(ms):
    lines = ['| 태그 | 모드 | 종류 | 구역 | 시각(t_base 뒤 s) | 로봇 | 깊이 m | 자세 · 단계 |',
             '|' + '---|' * 8]
    for m in sorted(ms, key=lambda m: (m.get('mode') != 'distributed', m['_tag'])):
        tb = m.get('t_base') or 0.0
        for e in m.get('occupancy_disc') or []:
            lines.append(f"| {m['_tag']} | {m.get('mode')} | 원판 동시 점유 | {e['zone']} | "
                         f"{e['t0'] - tb:.1f}–{e['t1'] - tb:.1f} | {','.join(e['robots'])} | "
                         f"{fmt(e.get('depth_m'), 3)} | {e.get('poses', '')} |")
        for e in (m.get('unauth_events') or [])[:20]:
            lines.append(f"| {m['_tag']} | {m.get('mode')} | 허가 없는 진입 | {e['zone']} | "
                         f"{e['t'] - tb:.1f} | {e['robot']} | — | {e.get('pose')} {e.get('mode')} "
                         f"{e.get('phase')} |")
        for e in m.get('release_inside') or []:
            lines.append(f"| {m['_tag']} | {m.get('mode')} | 안에서 해제 | {e['zone']} | "
                         f"{e['t'] - tb:.1f} | {e['robot']} | {fmt(e.get('clear_m'), 3)} | "
                         f"{e.get('gt')} |")
    return '\n'.join(lines)


def exp_text(root, exp):
    p = os.path.join(root, 'analysis', f'{exp}.json')
    if not os.path.isfile(p):
        return f'(analysis/{exp}.json 없음)'
    with open(p, encoding='utf-8') as f:
        r = json.load(f)
    lines = [f"### {exp} — 유효 짝 {r.get('n_pairs_valid')}, 갈래 {r.get('labels')} (차 = b − a)", '',
             '| 지표 | n | 평균 차 | 95 % CI | +/−/0 | 부호 p | 짝 (a, b) |', '|' + '---|' * 7]
    for mt, c in (r.get('compare') or {}).items():
        ci = c.get('ci95') or [None, None]
        lines.append(f"| {mt} | {c['n']} | {fmt(c.get('mean'), 2)} | "
                     f"[{fmt(ci[0], 2)}, {fmt(ci[1], 2)}] | "
                     f"{c['pos']}/{c['neg']}/{c['zero']} | {c['sign_p']:.4f} | "
                     f"{[(row[1], row[2]) for row in c['rows']]} |")
    for k, v in (r.get('decision') or {}).items():
        lines.append(f'- 판정 {k}: {v}')
    for key in ('aa', 'divergence', 'outage', 'outage_compare', 'holm', 'docking', 'invalid'):
        if r.get(key):
            lines.append(f'- {key}: `{json.dumps(r[key], ensure_ascii=False, default=str)[:900]}`')
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['runs', 'events', 'exp'])
    ap.add_argument('root')
    ap.add_argument('exp', nargs='?')
    ap.add_argument('--exp', dest='exp_opt')
    a = ap.parse_args(argv)
    if a.cmd == 'exp':
        print(exp_text(a.root, a.exp))
        return 0
    ms = load_runs(a.root, a.exp_opt)
    print(runs_table(ms) if a.cmd == 'runs' else events_table(ms))
    return 0


if __name__ == '__main__':
    sys.exit(main())
