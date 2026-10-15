#!/usr/bin/env python3
r"""
보너스 4 P5b 분석 (b4-p5b — 사전 등록, 결과 전에 커밋하고 sha256 을 노트 항목에 적는다). 표준 라이브러리 + PyYAML.

구역 진입 몸체 판정(`effed5b`) 뒤 Gazebo 재확인의 판정 계산. 갈래 지표는 P5 분석기(`p5_analyze.run_metrics`, 바꾸지 않음)가
낸 metrics.json 을 그대로 쓰고, 사건 기제 분류는 P5 리뷰 재확인 도구(`p5_review_check.incidents` — 허가 없는 로봇이 사건
15 s 전부터 끝까지 그 구역을 요청 · 막힘(`gate` 사건)했는가)를 그대로 쓴다. 기준(P5)과 새 값은 **같은 함수**로 센다.

  python3 p5b_analyze.py safety --root <P5b 원자료> --sets SC2,C3 \\
      --baseline-root <P5 원자료> --baseline-sets C2:distributed,C3 --out <디렉터리>
  python3 p5b_analyze.py central --root <P5b 원자료> --sets C1:central,F5L --out <디렉터리>
  python3 p5b_analyze.py f5l --root <P5b 원자료> --out <디렉터리>

갈래 고르기: 드라이버의 pairs/<실험>_p<k>.done 에 적힌 태그만 (판정에 쓴 짝 — `.done.pre_*` 는 고치기 전 코드라 뺀다).
'<실험>:<모드>' 면 그 모드 갈래만.

판정 (노트 「P5b 사전 등록」과 같다)
- 안전 불변식 (분산): 실제 자세 원판 동시 점유 · 믿은 허가 겹침 · 픽업 중복 · 낮은 epoch 수용 중 하나라도 > 0 이면 **미달**.
  0 이면 노출(구역 진입 E)로 rule of three 상한 3/E.
- 비율 비교 (P5 같은 묶음 대비): 원판 사건 x_new / 진입 E_new 대 x_old / E_old — 조건부 정확 이항 검정 단측
  p = P(X ≤ x_new), X ~ Bin(x_new + x_old, E_new / (E_new + E_old)). p < 0.05 면 "P5 보다 적다",
  아니면 "가르지 못함".
- 결정점 정지 침범 (분산): `gate_stops_before_grant` 의 clear_m < 0 비율, 새 대 P5 — Fisher 정확 검정 단측 (새 쪽이 적다).
- 기술만 (판정 없음): 기제별 사건 수, 중심 기준 사건, 허가 없는 진입 초, 막힘 거리 0 사건(걸친 미보유 구역 — 새 판정에만
  있는 동작), 완료 · makespan · 교착 · RTF. P5 와 시각 · 부하가 다르고 짝이 아니다 — 차이를 판정하지 않는다.
"""

import argparse
import collections
import json
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', '..', 'tests', 'integration', 'fleet_p5'))
import p5_analyze as A  # noqa: E402
import p5_review_check as RC  # noqa: E402

ALPHA = 0.05


# ---------------------------------------------------------------------- 통계
def cond_binom_lower(x_new, e_new, x_old, e_old):
    """조건부 정확 이항 검정 단측 p = P(X ≤ x_new | X + Y = n), 율이 같다는 귀무 아래 (새 쪽이 적은가)."""
    n = x_new + x_old
    if n == 0 or e_new <= 0 or e_old <= 0:
        return 1.0
    p = e_new / (e_new + e_old)
    return min(1.0, sum(math.comb(n, k) * p ** k * (1 - p) ** (n - k) for k in range(x_new + 1)))


def fisher_lower(a, n1, c, n2):
    """Fisher 정확 검정 단측 p: 표 [[a, n1 − a], [c, n2 − c]] 에서 첫 행 사건 수가 a 이하일 확률 (초기하)."""
    k = a + c
    n = n1 + n2
    if n == 0 or k == 0:
        return 1.0
    total = math.comb(n, k)
    lo = max(0, k - n2)
    return min(1.0, sum(math.comb(n1, i) * math.comb(n2, k - i) for i in range(lo, a + 1)) / total)


def rule_of_three(e):
    """사건 0 · 노출 e 의 95 % 단측 상한 율 (3/e)."""
    return None if not e else 3.0 / e


# ---------------------------------------------------------------------- 갈래
def parse_sets(text):
    """'SC2,C3' · 'C2:distributed' → [(실험, 모드 또는 None)]."""
    out = []
    for tok in text.split(','):
        tok = tok.strip()
        if tok:
            exp, _, mode = tok.partition(':')
            out.append((exp, mode or None))
    return out


def select_arms(root, sets):
    """pairs/<실험>_p<k>.done 의 태그 중 (모드가 맞는) 것 [(실험, 태그)] — 짝 번호 · 갈래 순."""
    out = []
    pdir = os.path.join(root, 'pairs')
    for exp, mode in sets:
        names = sorted((n for n in os.listdir(pdir) if n.startswith(f'{exp}_p')
                        and n.endswith('.done') and n[len(exp) + 2:-5].isdigit()),
                       key=lambda n: int(n[len(exp) + 2:-5])) if os.path.isdir(pdir) else []
        for n in names:
            with open(os.path.join(pdir, n), encoding='utf-8') as f:
                tags = json.load(f)['tags']
            for key in sorted(tags, key=str):
                tag = tags[key]
                m = A.read_json(os.path.join(root, tag, 'metrics.json'), {})
                if mode is None or m.get('mode') == mode:
                    out.append((exp, tag))
    return out


def zero_distance_blocks(events):
    """막힘 거리 0 사건: `gate` 의 blocking 이 비지 않고 first_entry_m == 0 (허가 없이 이미 걸친 구역)."""
    n = 0
    for e in events:
        ev = e.get('ev') or {}
        if e.get('k') == 'agent_event' and ev.get('event') == 'gate' and ev.get('blocking') \
                and ev.get('first_entry_m') is not None and float(ev['first_entry_m']) <= 1e-6:
            n += 1
    return n


def head_commit(path):
    """갈래 HEAD.txt 첫 줄 (없으면 None)."""
    if not os.path.isfile(path):
        return None
    with open(path, encoding='utf-8') as f:
        return f.readline().strip()


def arm_row(root, exp, tag):
    """갈래 한 줄 (metrics.json + 기제 분류)."""
    m = A.read_json(os.path.join(root, tag, 'metrics.json'), {})
    row = {'exp': exp, 'tag': tag, 'mode': m.get('mode'), 'label': m.get('label'),
           'pair': m.get('pair'), 'valid': bool(m.get('valid')),
           'commit': head_commit(os.path.join(root, tag, 'HEAD.txt')),
           'entries': m.get('zone_entries') or 0,
           'disc': len(m.get('occupancy_disc') or []),
           'center': len(m.get('occupancy_center') or []),
           'disc_events': m.get('occupancy_disc') or [],
           'held_overlaps': len(m.get('held_overlaps') or {}),
           'dup_pickups': len(m.get('duplicate_pickups') or {}),
           'low_epoch_accepted': len(m.get('low_epoch_accepted') or []),
           'unauth_s': round(sum((m.get('unauth_s') or {}).values()), 2),
           'completed': m.get('completed'), 'n_tasks': m.get('n_tasks'),
           'makespan': m.get('makespan'), 'deadlocks': m.get('deadlocks'),
           'unresolved': m.get('unresolved'), 'rtf': m.get('rtf'),
           'start_load': m.get('start_load'), 'end_load': m.get('end_load')}
    stops = m.get('gate_stops_before_grant') or []
    row['gate_stops'] = len(stops)
    row['gate_stops_neg'] = sum(1 for s in stops if (s.get('clear_m') or 0.0) < 0.0)
    row['gate_stop_min'] = min((s['clear_m'] for s in stops if s.get('clear_m') is not None),
                               default=None)
    row['gate_pass'] = m.get('gate_pass_without_stop')
    row['mech'] = []
    row['zero_blocks'] = 0
    if m.get('mode') == 'distributed' and os.path.isfile(os.path.join(root, tag, 'events.jsonl')):
        events = A.read_jsonl(os.path.join(root, tag, 'events.jsonl'))
        row['zero_blocks'] = zero_distance_blocks(events)
        if row['disc']:
            gt = A.read_gt(os.path.join(root, tag, 'gt.csv'))
            held = A.timeline(events, 'agent_state', 'held')
            row['mech'] = RC.incidents(root, tag, events, gt, held)
    return row


def totals(rows):
    """갈래 줄 합."""
    keys = ('entries', 'disc', 'center', 'held_overlaps', 'dup_pickups', 'low_epoch_accepted',
            'gate_stops', 'gate_stops_neg', 'zero_blocks', 'deadlocks', 'unresolved')
    t = {k: sum(int(r.get(k) or 0) for r in rows) for k in keys}
    t['arms'] = len(rows)
    t['invalid_arms'] = [r['tag'] for r in rows if not r['valid']]
    t['unauth_s'] = round(sum(r['unauth_s'] for r in rows), 2)
    t['mech'] = dict(collections.Counter(x['mech'] for r in rows for x in r['mech']))
    t['mech_s'] = {k: round(sum(x['dur'] for r in rows for x in r['mech'] if x['mech'] == k), 2)
                   for k in t['mech']}
    t['completed'] = sum(int(r['completed'] or 0) for r in rows)
    t['n_tasks'] = sum(int(r['n_tasks'] or 0) for r in rows)
    t['gate_stop_min'] = min((r['gate_stop_min'] for r in rows if r['gate_stop_min'] is not None),
                             default=None)
    rtf = [r['rtf'] for r in rows if r['rtf'] is not None]
    t['rtf_range'] = [min(rtf), max(rtf)] if rtf else None
    ms = [r['makespan'] for r in rows if r['makespan'] is not None]
    t['makespan_median'] = statistics.median(ms) if ms else None
    return t


def safety_verdict(new, old):
    """사전 등록 판정 문구 (가르지 못함을 '같다' 로 쓰지 않는다)."""
    out = {}
    bad = {k: new[k] for k in ('disc', 'held_overlaps', 'dup_pickups', 'low_epoch_accepted')
           if new[k] > 0}
    if bad:
        out['invariant'] = f'미달 ({", ".join(f"{k} {v}" for k, v in bad.items())})'
    else:
        ub = rule_of_three(new['entries'])
        out['invariant'] = ('충족 (원판 동시 점유 0 · 믿은 허가 겹침 0 · 픽업 중복 0 · 낮은 epoch 수용 0, '
                            f'노출 진입 {new["entries"]}, 95 % 상한 {ub:.4f}/진입)' if ub else
                            '판정 불가 (노출 0)')
    p = cond_binom_lower(new['disc'], new['entries'], old['disc'], old['entries'])
    out['rate_p'] = p
    out['rate'] = (f'P5 보다 적다 (단측 p = {p:.4g}' if p < ALPHA else f'가르지 못함 (단측 p = {p:.4g}') + \
        f'; 새 {new["disc"]}/{new["entries"]} 대 P5 {old["disc"]}/{old["entries"]})'
    q = fisher_lower(new['gate_stops_neg'], new['gate_stops'], old['gate_stops_neg'],
                     old['gate_stops'])
    out['stop_p'] = q
    out['stop'] = (f'P5 보다 적다 (단측 p = {q:.4g}' if q < ALPHA else f'가르지 못함 (단측 p = {q:.4g}') + \
        f'; 새 {new["gate_stops_neg"]}/{new["gate_stops"]} 대 P5 ' \
        f'{old["gate_stops_neg"]}/{old["gate_stops"]})'
    return out


def cmd_safety(a):
    new_rows = [arm_row(a.root, e, t) for e, t in select_arms(a.root, parse_sets(a.sets))]
    old_rows = [arm_row(a.baseline_root, e, t)
                for e, t in select_arms(a.baseline_root, parse_sets(a.baseline_sets))]
    new, old = totals(new_rows), totals(old_rows)
    res = {'sets': a.sets, 'baseline_sets': a.baseline_sets, 'new': new, 'baseline': old,
           'verdict': safety_verdict(new, old) if new['arms'] else '갈래 없음',
           'new_rows': new_rows, 'baseline_rows': old_rows}
    return res


def cmd_central(a):
    rows = [arm_row(a.root, e, t) for e, t in select_arms(a.root, parse_sets(a.sets))]
    return {'sets': a.sets, 'totals': totals(rows), 'rows': rows}


def dock_counts(m):
    """도크 구역 사건: 고정 지표(옛 큰 상자) 원판 · 중심, 그 배치 자기 상자 원판."""
    own = [e for e in (m.get('occupancy_disc') or [])
           if str(e.get('zone', '')).startswith('dock_')]
    return {'fixed_disc': len(m.get('occupancy_dock_fixed_disc') or []),
            'fixed_center': len(m.get('occupancy_dock_fixed_center') or []),
            'own_disc': len(own),
            'dock_n': m.get('dock_n') or 0, 'dock_within': m.get('dock_within_2cm_1deg') or 0,
            'dock_err_med': m.get('dock_pos_err_med'), 'dock_err_max': m.get('dock_pos_err_max')}


def f5l_not_worse(per, cmp_c):
    """
    F5L 사전 등록 문구 조건 → (새 배치가 나쁘지 않다, 완료 회귀).

    짝 차는 b − a = central_old − central_new (갈래 이름 사전순). 완료 회귀 = 부호 검정 p < 0.05 이고 옛 > 새.
    "나쁘지 않다" = 두 배치 자료가 있고, 도크 고정 지표 원판 사건 합 새 ≤ 옛, 완료 회귀 없음.
    """
    new, old = per.get('central_new', {}), per.get('central_old', {})
    regress = bool(cmp_c and cmp_c['sign_p'] < ALPHA and cmp_c['pos'] > cmp_c['neg'])
    ok = bool(new) and bool(old) and new.get('fixed_disc', 0) <= old.get('fixed_disc', 0) \
        and not regress
    return ok, regress


def cmd_f5l(a):
    """F5L: 짝(`.done`)마다 옛 · 새 배치 도크 사건 · 도킹, 합과 사전 등록 문구."""
    summ = A.exp_summary('F5L', a.root)
    per = collections.defaultdict(lambda: collections.Counter())
    rows = []
    for exp, tag in select_arms(a.root, [('F5L', None)]):
        m = A.read_json(os.path.join(a.root, tag, 'metrics.json'), {})
        c = dock_counts(m)
        rows.append(dict(c, tag=tag, label=m.get('label'), pair=m.get('pair'),
                         completed=m.get('completed'), makespan=m.get('makespan')))
        for k in ('fixed_disc', 'fixed_center', 'own_disc', 'dock_n', 'dock_within'):
            per[m.get('label')][k] += c[k]
    cmp_c = (summ.get('compare') or {}).get('completed')
    ok, regress = f5l_not_worse(per, cmp_c)
    return {'n_pairs_valid': summ.get('n_pairs_valid'), 'decision': summ.get('decision'),
            'compare': {k: (summ.get('compare') or {}).get(k) for k in ('completed', 'makespan')},
            'per_layout': {k: dict(v) for k, v in per.items()}, 'rows': rows,
            'not_worse': ('새 배치가 나쁘지 않다 (도크 고정 지표 원판 사건 새 ≤ 옛, 완료 회귀 없음)' if ok else
                          '"나쁘지 않다" 를 쓸 수 없다 (조건 중 하나 이상 어긋남 또는 짝 없음)'),
            'completion_regression': regress}


def md(res, kind):
    """짧은 요약 (원본은 JSON)."""
    if kind == 'safety':
        n, o, v = res['new'], res['baseline'], res['verdict']
        lines = [f"# P5b 안전 재확인 — 새 {res['sets']} ({n['arms']} 갈래) 대 P5 {res['baseline_sets']} "
                 f"({o['arms']} 갈래)", '']
        for k in ('entries', 'disc', 'center', 'held_overlaps', 'dup_pickups',
                  'low_epoch_accepted', 'unauth_s', 'gate_stops', 'gate_stops_neg',
                  'gate_stop_min', 'zero_blocks', 'mech', 'mech_s', 'completed', 'n_tasks',
                  'deadlocks', 'unresolved', 'rtf_range', 'makespan_median', 'invalid_arms'):
            lines.append(f'- {k}: 새 {n.get(k)} · P5 {o.get(k)}')
        if isinstance(v, dict):
            for k in ('invariant', 'rate', 'stop'):
                lines.append(f'- 판정 {k}: {v[k]}')
        return '\n'.join(lines) + '\n'
    return json.dumps(res, ensure_ascii=False, indent=1, default=str)[:4000] + '\n'


def main(argv=None):
    ap = argparse.ArgumentParser(description='P5b 분석')
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('safety')
    s.add_argument('--root', required=True)
    s.add_argument('--sets', default='SC2,C3')
    s.add_argument('--baseline-root', default='/home/CAPYI_AI/runs/b24/b4-p5/raw')
    s.add_argument('--baseline-sets', default='C2:distributed,C3')
    s.add_argument('--out', default='')
    c = sub.add_parser('central')
    c.add_argument('--root', required=True)
    c.add_argument('--sets', default='C1:central,F5L')
    c.add_argument('--out', default='')
    f = sub.add_parser('f5l')
    f.add_argument('--root', required=True)
    f.add_argument('--out', default='')
    a = ap.parse_args(argv)
    res = {'safety': cmd_safety, 'central': cmd_central, 'f5l': cmd_f5l}[a.cmd](a)
    text = md(res, a.cmd)
    if a.out:
        os.makedirs(a.out, exist_ok=True)
        with open(os.path.join(a.out, f'p5b_{a.cmd}.json'), 'w', encoding='utf-8') as fh:
            json.dump(res, fh, ensure_ascii=False, indent=1, default=str)
        with open(os.path.join(a.out, f'p5b_{a.cmd}.md'), 'w', encoding='utf-8') as fh:
            fh.write(text)
    print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
