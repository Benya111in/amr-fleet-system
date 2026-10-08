"""
P4 결과 표 (plan.py 의 사전 등록 규칙 그대로) — 마크다운 + JSON.

  python3 -m amr_fleet.fleetsim.report aa   --runs A/runs.jsonl --out DIR
  python3 -m amr_fleet.fleetsim.report main --runs M/runs.jsonl [--aa DIR/aa.json] --out DIR
  python3 -m amr_fleet.fleetsim.report hold --runs M/runs.jsonl H/runs.jsonl --out DIR  (리뷰 B1)

표의 수치는 모두 같은 seed 쌍의 실행 단위 값이다 (작업 단위 아님).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics as st
from typing import Dict, List, Optional

from amr_fleet.fleetsim import plan
from amr_fleet.fleetsim.analysis import (
    compare, divergence, fmt, holm, index, load_runs, mde, metric, paired, pearson, zero_upper,
)

SHORT = {a: a.split('_', 1)[0] for a in plan.MAIN_ARMS + plan.DIAG_ARMS + plan.HOLD_ARMS}


def _ci(c: dict, nd: int = 1) -> str:
    if c.get('n', 0) < 2:
        return '-'
    lo, hi = c['ci95']
    return f"{fmt(c['mean_diff'], nd)} [{fmt(lo, nd)}, {fmt(hi, nd)}]"


# ---------------------------------------------------------------------- A/A
def aa_report(rows: List[dict]) -> dict:
    idx = index(rows)
    out = {'cells': []}
    for arm in plan.AA_ARMS:
        for loss, fault in plan.AA_CELLS:
            seeds = sorted(s for (a, lo, f, s, r) in idx if a == arm and lo == loss
                           and f == fault and r == 0 and (a, lo, f, s, 1) in idx)
            if not seeds:
                continue
            cell = {'arm': arm, 'loss': loss, 'fault': fault, 'n': len(seeds), 'metrics': {}}
            for m in plan.AA_METRICS:
                x0 = [metric(idx[(arm, loss, fault, s, 0)], m) for s in seeds]
                x1 = [metric(idx[(arm, loss, fault, s, 1)], m) for s in seeds]
                pair = [(a, b) for a, b in zip(x0, x1) if a is not None and b is not None]
                if len(pair) < 3:
                    continue
                a, b = [p[0] for p in pair], [p[1] for p in pair]
                c = paired(a, b)
                cell['metrics'][m] = {
                    'n': c['n'], 'mean0': c['mean_a'], 'mean1': c['mean_b'],
                    'mean_diff': c['mean_diff'], 'ci95': c['ci95'], 'sd_diff': c['sd_diff'],
                    'sd_between': st.pstdev(a) if len(a) > 1 else 0.0,
                    'rho': pearson(a, b), 'mde30': mde(c['sd_diff'], 30),
                    'mde8': mde(c['sd_diff'], 8), 'k_zero': c['k_zero']}
            divs = []
            for s in seeds:
                t0 = idx[(arm, loss, fault, s, 0)].get('trace')
                t1 = idx[(arm, loss, fault, s, 1)].get('trace')
                if t0 is not None and t1 is not None:
                    d = divergence(t0, t1)
                    divs.append(math.inf if d is None else d)
            if divs:
                fin = sorted(divs)
                never = sum(1 for d in divs if math.isinf(d))
                cell['divergence'] = {'median': fin[len(fin) // 2], 'min': fin[0],
                                      'max': fin[-1], 'never': never}
            out['cells'].append(cell)
    return out


def _row(*cells) -> str:
    return '| ' + ' | '.join(str(c) for c in cells) + ' |'


def _head(*cells) -> List[str]:
    return [_row(*cells), _row(*(['---'] * len(cells)))]


def aa_markdown(rep: dict) -> str:
    lines = ['# A/A 산포 (rep 0 대 rep 1: 작업 열·고장 같고 통신 난수·실행 잡음만 다름)', '']
    lines += _head('비교군', '손실', '고장', '지표', 'n', '평균 rep0', '평균 쌍차 [95% CI]', '쌍차 SD',
                   'ρ', 'MDE n=30', 'MDE n=8', '같은 값 쌍')
    for c in rep['cells']:
        for m, v in c['metrics'].items():
            lo, hi = v['ci95']
            lines.append(_row(
                SHORT.get(c['arm'], c['arm']), f"{c['loss']:g}", c['fault'], m, v['n'],
                fmt(v['mean0']), f"{fmt(v['mean_diff'])} [{fmt(lo)}, {fmt(hi)}]",
                fmt(v['sd_diff']), fmt(v['rho']), fmt(v['mde30']), fmt(v['mde8']), v['k_zero']))
    lines += ['']
    lines += _head('비교군', '손실', '고장', '사건열 발산 시각 중앙값 [s]', '최소', '최대', '끝까지 같음')
    for c in rep['cells']:
        d = c.get('divergence')
        if d:
            lines.append(_row(SHORT.get(c['arm'], c['arm']), f"{c['loss']:g}", c['fault'],
                              fmt(d['median'], 1), fmt(d['min'], 1), fmt(d['max'], 1),
                              d['never']))
    return '\n'.join(lines) + '\n'


def aa_floor(rep: Optional[dict], metric_name: str, loss: float, fault: str) -> Optional[float]:
    """같은 지표·셀의 A/A MDE(n=30) 최댓값 (비교군들 중) — 없으면 None."""
    if not rep:
        return None
    vals = [c['metrics'][metric_name]['mde30'] for c in rep['cells']
            if c['loss'] == loss and c['fault'] == fault and metric_name in c['metrics']]
    return max(vals) if vals else None


# ---------------------------------------------------------------------- 본 스윕
def safety(rows: List[dict]) -> List[dict]:
    out = []
    for arm in plan.MAIN_ARMS + plan.DIAG_ARMS:
        rs = [r for r in rows if r['arm'] == arm]
        if not rs:
            continue
        n = len(rs)
        cooc_runs = sum(1 for r in rs if r['cooccupancy_events'] > 0)
        dup_runs = sum(1 for r in rs if r['dup_tasks'] > 0)
        dl_runs = sum(1 for r in rs if r['deadlocks'] > 0)
        out.append({
            'arm': arm, 'n': n,
            'cooc_events': sum(r['cooccupancy_events'] for r in rs), 'cooc_runs': cooc_runs,
            'cooc_upper': zero_upper(n) if cooc_runs == 0 else None,
            'dup_tasks': sum(r['dup_tasks'] for r in rs), 'dup_runs': dup_runs,
            'dup_upper': zero_upper(n) if dup_runs == 0 else None,
            'deadlocks': sum(r['deadlocks'] for r in rs), 'deadlock_runs': dl_runs,
            'dl_upper': zero_upper(n) if dl_runs == 0 else None,
            'long_waits': sum(r['long_waits'] for r in rs),
            'stuck_end_runs': sum(1 for r in rs if r['stuck_end'] > 0),
        })
    return out


def primary(idx) -> List[dict]:
    res = []
    for p in plan.PRIMARY:
        c = compare(idx, p['a'], p['b'], p['loss'], p['fault'], p['metric'])
        c.update({'id': p['id'], 'kind': p['kind'], 'pred': p['pred']})
        res.append(c)
    adj = holm([c.get('wilcoxon_p', 1.0) for c in res])
    for c, pa in zip(res, adj):
        c['p_holm'] = pa
        lo, hi = c['ci95'] if c['n'] >= 2 else (-math.inf, math.inf)
        if c['n'] < 2:
            c['decision'] = '데이터 없음'
        elif c['kind'] == 'superiority':
            ok = pa < 0.05 and lo > 0 and c['k_pos'] >= plan.SUPERIORITY_MIN_K
            c['decision'] = '우월' if ok else '판정 불가'
        else:
            mg = plan.EQUIV_MARGIN[c['metric']]
            eq = -mg < lo and hi < mg
            diff = pa < 0.05 and (lo > 0 or hi < 0)
            c['decision'] = ('차이 있으나 margin 안' if eq and diff else f'동등(±{mg:g})' if eq
                             else '차이' if diff else '판정 불가')
    return res


def explore(idx, aa: Optional[dict]) -> List[dict]:
    out = []
    for loss in plan.MAIN_LOSSES:
        for fault in plan.MAIN_FAULTS:
            for arm in plan.MAIN_ARMS:
                if arm == plan.EXPLORE_VS:
                    continue
                for m in plan.EXPLORE_METRICS:
                    c = compare(idx, plan.EXPLORE_VS, arm, loss, fault, m)
                    if c['n'] < 2:
                        continue
                    fl = aa_floor(aa, m, loss, fault)
                    c['below_floor'] = fl is not None and abs(c['mean_diff']) < fl
                    c['floor'] = fl
                    out.append(c)
    return out


def factorial(idx) -> List[dict]:
    f = plan.FACTORIAL
    out = []
    for loss in plan.MAIN_LOSSES:
        for fault in plan.MAIN_FAULTS:
            for m in ('completed', 'latency_rmean'):
                seeds = sorted(s for (a, lo, fa, s, r) in idx if a == f['CC'] and lo == loss
                               and fa == fault and r == 0
                               and all((f[k], loss, fault, s, 0) in idx for k in f))
                vals = {k: [metric(idx[(f[k], loss, fault, s, 0)], m) for s in seeds] for k in f}
                if not seeds or any(v is None for k in f for v in vals[k]):
                    continue
                n = len(seeds)
                alloc = [0.5 * ((vals['DC'][i] - vals['CC'][i]) + (vals['DV'][i] - vals['CV'][i]))
                         for i in range(n)]
                traf = [0.5 * ((vals['CV'][i] - vals['CC'][i]) + (vals['DV'][i] - vals['DC'][i]))
                        for i in range(n)]
                inter = [(vals['DV'][i] - vals['DC'][i]) - (vals['CV'][i] - vals['CC'][i])
                         for i in range(n)]
                z = [0.0] * n
                out.append({'loss': loss, 'fault': fault, 'metric': m, 'n': n,
                            'means': {k: st.mean(vals[k]) for k in f},
                            'alloc': paired(z, alloc), 'traffic': paired(z, traf),
                            'interaction': paired(z, inter)})
    return out


def qcurves(rows: List[dict], bin_mult: int = 2) -> Dict:
    """(손실, 고장, 비교군) → 구간(60 s × bin_mult)별 평균 완료 수 — Q(t) 겹쳐 보기용 (기술 통계)."""
    out: Dict = {}
    for loss in (0.05, 0.2):
        for fault in plan.MAIN_FAULTS:
            for arm in ('A1_nearest_central', 'A6_cbaa_vote', 'A9_hybrid_vote'):
                rs = [r for r in rows if r['arm'] == arm and r['loss'] == loss
                      and r['fault'] == fault]
                if not rs:
                    continue
                nb = len(rs[0]['q_bins']) // bin_mult
                out[(loss, fault, arm)] = [
                    st.mean(sum(r['q_bins'][k * bin_mult:(k + 1) * bin_mult]) for r in rs)
                    for k in range(nb)]
    return out


def cell_means(rows: List[dict]) -> Dict:
    out: Dict = {}
    for r in rows:
        key = (r['arm'], r['loss'], r['fault'])
        out.setdefault(key, []).append(r)
    return {k: {'n': len(v), 'completed': st.mean(x['completed'] for x in v),
                'latency_rmean': st.mean(x['latency_rmean'] for x in v),
                'after': st.mean(x['completed_after_fault'] for x in v),
                'dup': sum(x['dup_tasks'] for x in v),
                'msgs': st.mean(x['msgs_sent'] for x in v),
                'stuck_runs': sum(1 for x in v if x['stuck_end'] > 0),
                'fallback': [x['first_fallback_s'] for x in v
                             if x['first_fallback_s'] is not None]}
            for k, v in out.items()}


def _pct100(x: Optional[float]) -> str:
    return '-' if x is None else f'{100.0 * x:.2f} %'


def main_markdown(rows: List[dict], aa: Optional[dict]) -> (str, dict):
    idx = index(rows)
    saf = safety(rows)
    pri = primary(idx)
    exp = explore(idx, aa)
    fac = factorial(idx)
    cm = cell_means(rows)
    L = ['# P4 본 스윕 결과 (plan.py 사전 등록 규칙)', '',
         '## 안전·정확성 (실행 단위, 0 건이면 실행당 1 건 이상 확률의 95 % 상한)', '']
    L += _head('비교군', '실행', '동시 점유 사건 (실행)', '상한', '중복 실행 작업 (실행)', '상한',
               '교착 순환 (실행)', '상한', '120 s 넘는 대기', '끝에 고착 실행')
    for s in saf:
        L.append(_row(SHORT[s['arm']], s['n'], f"{s['cooc_events']} ({s['cooc_runs']})",
                      _pct100(s['cooc_upper']), f"{s['dup_tasks']} ({s['dup_runs']})",
                      _pct100(s['dup_upper']), f"{s['deadlocks']} ({s['deadlock_runs']})",
                      _pct100(s['dl_upper']), s['long_waits'], s['stuck_end_runs']))
    L += ['', '## 주 비교 (Holm 보정 4 개)', '']
    L += _head('id', 'B − A', '셀', '지표', 'n', '평균 A', '평균 B', '쌍차 [95% CI]', 'd_z',
               'k+/k−/k0', 'Holm p', '판정', '예측')
    for c in pri:
        L.append(_row(c['id'], f"{SHORT[c['b']]} − {SHORT[c['a']]}",
                      f"손실 {c['loss']:g} {c['fault']}", c['metric'], c['n'], fmt(c['mean_a']),
                      fmt(c['mean_b']), _ci(c), fmt(c['dz']),
                      f"{c['k_pos']}/{c['k_neg']}/{c['k_zero']}", fmt(c['p_holm'], 4),
                      f"**{c['decision']}**", c['pred']))
    L += ['', '## 셀별 평균 (쌍 30)', '']
    L += _head('비교군', '손실', '고장', '완료', '고장 뒤 완료', '제한 평균 지연 [s]', '중복 작업 합',
               '끝 고착 실행', '메시지/실행', '폴백 전환 시각 중앙값')
    for (arm, loss, fault), v in sorted(cm.items(),
                                        key=lambda kv: (kv[0][2], kv[0][1], kv[0][0])):
        fb = sorted(v['fallback'])
        L.append(_row(SHORT[arm], f'{loss:g}', fault, fmt(v['completed']), fmt(v['after']),
                      fmt(v['latency_rmean'], 1), v['dup'], v['stuck_runs'], fmt(v['msgs'], 0),
                      fmt(fb[len(fb) // 2], 1) if fb else '-'))
    L += ['', f'## 탐색: 각 비교군 − {SHORT[plan.EXPLORE_VS]} '
          '(p 를 주장하지 않는다, † = A/A 바닥 아래)', '']
    L += _head('손실', '고장', '비교군', '지표', '쌍차 [95% CI]', 'd_z', 'k+/k−/k0')
    for c in exp:
        if c['metric'] not in ('completed', 'latency_rmean', 'completed_after_fault'):
            continue
        mark = ' †' if c['below_floor'] else ''
        L.append(_row(f"{c['loss']:g}", c['fault'], SHORT[c['b']], c['metric'], _ci(c) + mark,
                      fmt(c['dz']), f"{c['k_pos']}/{c['k_neg']}/{c['k_zero']}"))
    L += ['', '## 2×2 요인 (할당 중앙/협상 × 교통 중앙/투표: A2, A4, A5, A6)', '']
    L += _head('손실', '고장', '지표', 'CC', 'CV', 'DC', 'DV', '할당 효과 [CI]', '교통 효과 [CI]',
               '상호작용 [CI]')
    for f in fac:
        mm = f['means']
        L.append(_row(f"{f['loss']:g}", f['fault'], f['metric'], fmt(mm['CC']), fmt(mm['CV']),
                      fmt(mm['DC']), fmt(mm['DV']), _ci(f['alloc']), _ci(f['traffic']),
                      _ci(f['interaction'])))
    L += ['', '## Q(t): 120 s 구간별 평균 완료 수 (무장애 · 고장 + 폴백 끔 · 고장 + 폴백 켬)', '']
    qs = qcurves(rows)
    if qs:
        nb = len(next(iter(qs.values())))
        L += _head('손실', '고장', '비교군', *[f'{120 * k}–{120 * (k + 1)}' for k in range(nb)])
        for (loss, fault, arm), q in sorted(qs.items()):
            L.append(_row(f'{loss:g}', fault, SHORT[arm], *[fmt(x) for x in q]))
    data = {'safety': saf, 'primary': pri, 'factorial': fac,
            'qcurves': {'|'.join(map(str, k)): v for k, v in qs.items()},
            'explore': [{k: v for k, v in c.items()} for c in exp],
            'cell_means': {'|'.join(map(str, k)): v for k, v in cm.items()}}
    return '\n'.join(L) + '\n', data


# ---------------------------------------------------------------------- 리뷰 B1: hold 의미 (P4-5)
def _below(r: dict) -> int:
    # P4 배치의 결정점 미달 구역(도크 넷)으로 센다 — 이 보고는 P4 실행(fd17e49 배치)을 다시 만드는 것이다.
    # 지금 배치에는 미달 구역이 없어 새 실행에서는 이 칸이 '도크 구역 사건' 을 뜻한다 (plan.py 주석).
    z = r.get('cooccupancy_zones') or {}
    return sum(n for k, n in z.items() if k in plan.P4_BELOW_DECISION_ZONES)


def hold_safety(rows: List[dict]) -> List[dict]:
    """(비교군, 고장, 손실) → 동시 점유 실행 · 사건 · 그중 도크(결정점 미달) · 고장 전 사건 · 끝 고착."""
    out = []
    arms = plan.HOLD_ARMS + sorted(set(plan.HOLD_TWINS.values()))
    for arm in arms:
        for fault in plan.MAIN_FAULTS:
            for loss in plan.MAIN_LOSSES:
                rs = [r for r in rows if r['arm'] == arm and r['fault'] == fault
                      and r['loss'] == loss]
                if not rs:
                    continue
                n = len(rs)
                runs = sum(1 for r in rs if r['cooccupancy_events'] > 0)
                ev = sum(r['cooccupancy_events'] for r in rs)
                pre_known = all('cooccupancy_pre_fault' in r for r in rs)
                out.append({
                    'arm': arm, 'fault': fault, 'loss': loss, 'n': n, 'cooc_runs': runs,
                    'cooc_events': ev, 'cooc_upper': zero_upper(n) if runs == 0 else None,
                    'cooc_dock': sum(_below(r) for r in rs) if pre_known else None,
                    'cooc_pre': (sum(r['cooccupancy_pre_fault'] for r in rs)
                                 if pre_known else None),
                    'cooc_s': sum(r['cooccupancy_s'] for r in rs),
                    'stuck_runs': sum(1 for r in rs if r['stuck_end'] > 0),
                    'dup_runs': sum(1 for r in rs if r['dup_tasks'] > 0),
                    'completed': st.mean(r['completed'] for r in rs)})
    return out


def hold_markdown(rows: List[dict]) -> (str, dict):
    idx = index(rows)
    saf = hold_safety(rows)
    L = ['# 리뷰 B1: 현행 hold 의미(latched, 거부 목록) 대 P4 1 차 허가 목록 모형', '',
         '같은 셀 · 같은 시드 1–30. hold 비교군 = A1h · A2h · A5h, 짝 = A1 · A2 · A5 (P4-main-01).', '',
         '## 동시 점유 (실행 단위) — 도크 = 요청 거리가 결정점 거리 미달인 구역(p0.md §5)', '']
    L += _head('비교군', '고장', '손실', 'n', '동시 점유 실행', '사건', '그중 도크', '고장 전 사건',
               '점유 시간 합 [s]', '0 건 상한', '끝 고착 실행', '중복 실행', '평균 완료')
    for s in saf:
        L.append(_row(SHORT[s['arm']], s['fault'], f"{s['loss']:g}", s['n'], s['cooc_runs'],
                      s['cooc_events'], '-' if s['cooc_dock'] is None else s['cooc_dock'],
                      '-' if s['cooc_pre'] is None else s['cooc_pre'], fmt(s['cooc_s'], 1),
                      _pct100(s['cooc_upper']), s['stuck_runs'], s['dup_runs'],
                      fmt(s['completed'])))
    L += ['', '## hold 의미 − 허가 목록 (같은 seed 쌍, p 를 주장하지 않는다)', '']
    L += _head('쌍', '고장', '손실', '지표', '평균 허가 목록 → hold', '쌍차 [95% CI]', 'k+/k−/k0')
    pairs = []
    for h, a in plan.HOLD_TWINS.items():
        for fault in plan.MAIN_FAULTS:
            for loss in plan.MAIN_LOSSES:
                for m in ('completed', 'completed_after_fault', 'latency_rmean',
                          'cooccupancy_events'):
                    c = compare(idx, a, h, loss, fault, m)
                    if c['n'] < 2:
                        continue
                    pairs.append(c)
                    L.append(_row(f'{SHORT[h]} − {SHORT[a]}', fault, f'{loss:g}', m,
                                  f"{fmt(c['mean_a'])} → {fmt(c['mean_b'])}", _ci(c),
                                  f"{c['k_pos']}/{c['k_neg']}/{c['k_zero']}"))
    p = plan.HOLD_PRIMARY
    c = compare(idx, p['a'], p['b'], p['loss'], p['fault'], p['metric'])
    if c['n'] >= 2:
        lo = c['ci95'][0]
        ok = c['wilcoxon_p'] < 0.05 and lo > 0 and c['k_pos'] >= plan.SUPERIORITY_MIN_K
        c['decision'] = '우월' if ok else '판정 불가'
    else:
        c['decision'] = '데이터 없음'
    c.update({'id': p['id'], 'pred': p['pred']})
    L += ['', '## P1h 재확인 (단일 비교, 판정 규칙은 P1 과 같음 — Holm 가족 밖)', '']
    L += _head('id', 'B − A', '셀', 'n', '평균 A → B', '쌍차 [95% CI]', 'd_z', 'k+/k−/k0',
               'Wilcoxon p', '판정', '예측')
    L.append(_row(c['id'], f"{SHORT[c['b']]} − {SHORT[c['a']]}",
                  f"손실 {c['loss']:g} {c['fault']} {c['metric']}", c['n'],
                  f"{fmt(c['mean_a'])} → {fmt(c['mean_b'])}", _ci(c), fmt(c['dz']),
                  f"{c['k_pos']}/{c['k_neg']}/{c['k_zero']}", fmt(c['wilcoxon_p'], 4),
                  f"**{c['decision']}**", c['pred']))
    f = plan.FACTORIAL_HOLD
    fac = []
    for loss in plan.MAIN_LOSSES:
        for fault in plan.MAIN_FAULTS:
            seeds = sorted(sd for (a, lo, fa, sd, r) in idx if a == f['CC'] and lo == loss
                           and fa == fault and r == 0
                           and all((f[k], loss, fault, sd, 0) in idx for k in f))
            if not seeds:
                continue
            vals = {k: [metric(idx[(f[k], loss, fault, sd, 0)], 'completed') for sd in seeds]
                    for k in f}
            cooc = {k: sum(1 for sd in seeds
                           if idx[(f[k], loss, fault, sd, 0)]['cooccupancy_events'] > 0)
                    for k in f}
            n = len(seeds)
            traf = [0.5 * ((vals['CV'][i] - vals['CC'][i]) + (vals['DV'][i] - vals['DC'][i]))
                    for i in range(n)]
            alloc = [0.5 * ((vals['DC'][i] - vals['CC'][i]) + (vals['DV'][i] - vals['CV'][i]))
                     for i in range(n)]
            inter = [(vals['DV'][i] - vals['DC'][i]) - (vals['CV'][i] - vals['CC'][i])
                     for i in range(n)]
            z = [0.0] * n
            fac.append({'loss': loss, 'fault': fault, 'n': n,
                        'means': {k: st.mean(vals[k]) for k in f}, 'cooc_runs': cooc,
                        'alloc': paired(z, alloc), 'traffic': paired(z, traf),
                        'interaction': paired(z, inter)})
    L += ['', '## 2×2 를 hold 의미로 (CC = A2h, CV = A4, DC = A5h, DV = A6; 완료 수, 괄호 = 동시 점유 실행)',
          '']
    L += _head('손실', '고장', 'CC', 'CV', 'DC', 'DV', '할당 효과 [CI]', '교통 효과 [CI]',
               '상호작용 [CI]')
    for x in fac:
        mm, cc = x['means'], x['cooc_runs']
        L.append(_row(f"{x['loss']:g}", x['fault'],
                      *[f'{fmt(mm[k])} ({cc[k]})' for k in ('CC', 'CV', 'DC', 'DV')],
                      _ci(x['alloc']), _ci(x['traffic']), _ci(x['interaction'])))
    qs = {}
    for loss in (0.2,):
        for fault in ('F1', 'F5'):
            for arm in ('A1_nearest_central', 'A1h_nearest_hold', 'A9_hybrid_vote'):
                rs = [r for r in rows if r['arm'] == arm and r['loss'] == loss
                      and r['fault'] == fault]
                if rs:
                    nb = len(rs[0]['q_bins']) // 2
                    qs[(loss, fault, arm)] = [st.mean(sum(r['q_bins'][2 * k:2 * k + 2])
                                                      for r in rs) for k in range(nb)]
    if qs:
        nb = len(next(iter(qs.values())))
        L += ['', '## Q(t): 120 s 구간별 평균 완료 수', '']
        L += _head('손실', '고장', '비교군', *[f'{120 * k}–{120 * (k + 1)}' for k in range(nb)])
        for (loss, fault, arm), q in sorted(qs.items()):
            L.append(_row(f'{loss:g}', fault, SHORT[arm], *[fmt(x) for x in q]))
    data = {'safety': saf, 'pairs': pairs, 'primary': c, 'factorial': fac,
            'qcurves': {'|'.join(map(str, k)): v for k, v in qs.items()}}
    return '\n'.join(L) + '\n', data


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('which', choices=['aa', 'main', 'hold'])
    ap.add_argument('--runs', required=True, nargs='+')
    ap.add_argument('--aa', default=None)
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    rows = load_runs(a.runs)
    os.makedirs(a.out, exist_ok=True)
    if a.which == 'aa':
        rep = aa_report(rows)
        with open(os.path.join(a.out, 'aa.json'), 'w', encoding='utf-8') as f:
            json.dump(rep, f, ensure_ascii=False, indent=1, default=str)
        md = aa_markdown(rep)
        with open(os.path.join(a.out, 'aa.md'), 'w', encoding='utf-8') as f:
            f.write(md)
        print(md)
        return 0
    if a.which == 'hold':
        md, data = hold_markdown(rows)
        with open(os.path.join(a.out, 'hold.md'), 'w', encoding='utf-8') as f:
            f.write(md)
        with open(os.path.join(a.out, 'hold.json'), 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=1, default=str)
        print(md)
        return 0
    aa = None
    if a.aa:
        with open(a.aa, encoding='utf-8') as f:
            aa = json.load(f)
    md, data = main_markdown(rows, aa)
    with open(os.path.join(a.out, 'main.md'), 'w', encoding='utf-8') as f:
        f.write(md)
    with open(os.path.join(a.out, 'main.json'), 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=1, default=str)
    print(md)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
