#!/usr/bin/env python3
"""
paper_research 분석 (prereg.md §4·§6, contract.md §2). 원본 dwa-ablation/scripts/analyze.py 의 사본이다.

원본에서 그대로 가져온 것 (식·규칙 동일): t_lb(= T_ref), 설정 하나의 J (tuning_score), 경로별 값
(per_path, seed·반복은 경로 안에서 먼저 평균), 경로 묶음 부트스트랩 (B = 10 000, seed 12345, 백분위 95 %),
짝 차 (paired), 표 꼴 (table_summary·table_ablation·table_failures), 튜닝 팔 이름 바꾸기 (substitute_tuned).

    python3 analyze_x.py dryrun --raw /runs/dryrun/raw --orig /home/CAPYI_AI/runs/dwa-abl/raw
    python3 analyze_x.py equiv  --raw /runs/equiv/raw
    python3 analyze_x.py select --raw /runs/tuning/raw > /runs/tuning/selection.json
    python3 analyze_x.py tables --runs /runs --out <dir>

## 튜닝 선택 규칙 J' (prereg §4) — 튜닝 결과를 보기 전에 이 파일로 고정한다
- 설정마다 튜닝 4 경로를 네 조건에서 돈다: 잡음 없음(seed 0) 1 회, 오도메트리 잡음 σ 0.02 seed 1·2·3.
- 조건 c 의 J_c = 경로 평균 [CTE 최대/10 cm + |정지 오차|/10 cm + T/T_ref], 실패 경로 항 = 10 (원본 J 그대로).
- J' = J_0 + mean(J_1, J_2, J_3).
- 성공 수도 같은 꼴로 센다: S' = S_0 + mean(S_1, S_2, S_3) (S_c = 조건 c 의 성공 경로 수, 0–4).
  prereg 는 "성공 경로 수 최대가 먼저" 라고만 적었다 — 네 조건을 합치는 방식은 J' 와 같은 가중으로 정했다
  (results.md 「사전 등록과 다르게 한 것」에 해석으로 적는다).
- 순서: S' 최대 → J' 최소 → 이름 순. DWB·MPPI·RPP·DWA-R(dwa_x_*)·교과서(dwa_textbook_T*, prereg §8 A1)
  다섯 격자 모두 같은 함수.
- 네 조건 × 4 경로 중 하나라도 기록이 없으면 그 설정은 'incomplete' 로 표시하고 순위에서 뺀다 (재실행으로 채운다).

## 확증 가설 (prereg §6.1)
- H1r: chain_sim_x, held-out B, 잡음 없음, stateful false. 두 팔 모두 성공한 경로의
  mean log(T_no_m3 / T_full). 95 % 하한 > 0 이면 확증.
- H2: ROS C3. 경로마다 "두 팔 모두 성공한 seed 의 log(T_R / T_full) 평균", 그 12 경로 평균. 95 % 상한 < 0 이면 확증.
  (민감도: 원본 analyze 식 — 경로마다 seed 평균 시간의 log 비 — 도 함께 낸다.)
- H3: ROS C1. 두 팔 모두 성공한 경로의 mean log(T_R / T_RPP_R00). 95 % 상한 < log(1.005) 이면 확증.
나머지는 모두 기술(구간)이다.

## 재설정 기제 (prereg §5 재설정 기록, 원본 results.md §6 b 의 검증)
- 느린 잡음 실행 = 같은 팔·같은 stateful·같은 경로의 잡음 없는 실행보다 T 가 30 % 넘게 긴 실행 (원본 정의) 또는 실패.
- 첫 진입 = 궤적(trace)의 참값 위치가 경로 끝 0.10 m 안에 처음 든 시각.
- 재설정 = reset_log 의 행 (측정값 중심으로 돌아간 주기). 첫 진입 전 3 s 안의 재설정 수와 그 국면을 느린·나머지 실행으로 나눠 센다.
"""
import argparse
import collections
import csv
import glob
import json
import math
import os
import sys

import numpy as np

B = 10000
SEED = 12345
V_MAX, A_MAX, J_MAX = 1.0, 1.0, 2.0
HELDOUT = ('smooth', 'narrow', 'astar')
TABLE1_ARMS = ['dwa_full', 'dwa_R', 'dwa_textbook', 'dwa_textbook_tuned', 'DWB_D00', 'DWB_tuned',
               'MPPI_P00', 'MPPI_tuned', 'RPP_R00', 'RPP_tuned', 'pp']
CHAIN_ARMS = ['dwa_x_X02', 'dwa_x_X04', 'dwa_x_X05', 'dwa_x_X06', 'dwa_R', 'dwa_no_m3']
CHAIN_LABEL = {'dwa_x_X02': 'R1 K = 4 (X02)', 'dwa_x_X04': 'R2 저크 일관 (X04)',
               'dwa_x_X05': 'R3 PP 후보 (X05)', 'dwa_x_X06': 'R4 t_c 0.25 (X06)',
               'dwa_R': 'DWA-R (선택)', 'dwa_no_m3': 'M3 끔 (H1r)'}
FAMILY_PREFIX = {'DWB': 'DWB_', 'MPPI': 'MPPI_', 'RPP': 'RPP_', 'DWA_R': 'dwa_x_',
                 'TB': 'dwa_textbook_T'}
TUNING_SETTINGS = (0, 1, 2, 3)
LOG_1005 = math.log(1.005)


def load(path):
    """path(.jsonl) 와 같은 이름의 작업자 조각 path[:-6]_w*.jsonl 을 함께 읽는다 (원본 그대로)."""
    files = ([path] if os.path.exists(path) else []) + sorted(glob.glob(path[:-6] + '_w*.jsonl'))
    out = []
    for fn in files:
        with open(fn, encoding='utf-8') as f:
            out += [json.loads(line) for line in f if line.strip()]
    return out


def load_dir(d, dedupe=True):
    """디렉터리의 *.jsonl 전부. 같은 (scene, arm, seed, rep, stateful) 가 둘이면 먼저 것만 (재개 중복 방지)."""
    out = []
    seen = set()
    for fn in sorted(glob.glob(os.path.join(d, '*.jsonl'))):
        with open(fn, encoding='utf-8') as f:
            for line in f:
                if not line.strip():
                    continue
                r = json.loads(line)
                r['_file'] = os.path.basename(fn)
                key = (r['scene'], r['arm'], r.get('seed', 0), r.get('rep', 0),
                       r.get('stateful', 0), r.get('nseed', None))
                if dedupe and key in seen:
                    continue
                seen.add(key)
                out.append(r)
    return out


def load_chain(d):
    """chain_sim_x 호출 파일 <arm>__<fam>__<clean|sN>__st<0|1>.jsonl → 기록에 nseed·stateful 를 붙인다."""
    out = []
    for fn in sorted(glob.glob(os.path.join(d, '*.jsonl'))):
        base = os.path.basename(fn)[:-6]
        arm, fam, cond, st = base.split('__')
        nseed = 0 if cond == 'clean' else int(cond[1:])
        with open(fn, encoding='utf-8') as f:
            for line in f:
                if line.strip():
                    r = json.loads(line)
                    r['nseed'] = nseed
                    r['stateful'] = int(st[2:]) if 'stateful' not in r else int(r['stateful'])
                    r['_call'] = base
                    out.append(r)
    return out


def t_lb(length):
    """정지→정지 최소 시간 (곡률 무시): L/v + v/a + a/j. 본문에서는 T_ref 라 부른다."""
    return length / V_MAX + V_MAX / A_MAX + A_MAX / J_MAX


# ---------------------------------------------------------------------------------------------
# 튜닝 선택 (prereg §4)

def tuning_score(recs):
    """원본 그대로: (성공 수, J). J = 경로 평균 [CTE 최대/10 cm + |정지 오차|/10 cm + T/T_lb], 실패 10."""
    ok = 0
    terms = []
    for r in recs:
        if r['success']:
            ok += 1
            terms.append(100 * max(r['cte_max'], 0.0) / 10.0 + 100 * r['stop_err'] / 10.0 +
                         r['t_reach'] / t_lb(r['length']))
        else:
            terms.append(10.0)
    return ok, float(np.mean(terms)) if terms else float('inf')


def jprime(recs_by_seed, n_paths=4):
    """조건별 (S_c, J_c) → (S', J', 완전 여부, 조건별 값)."""
    per = {}
    complete = True
    for s in TUNING_SETTINGS:
        rs = recs_by_seed.get(s, [])
        if len({r['scene'] for r in rs}) < n_paths:
            complete = False
        per[s] = tuning_score(rs) if rs else (0, 10.0)
    s_prime = per[0][0] + float(np.mean([per[s][0] for s in (1, 2, 3)]))
    j_prime = per[0][1] + float(np.mean([per[s][1] for s in (1, 2, 3)]))
    return s_prime, j_prime, complete, {str(s): {'success': per[s][0], 'J': per[s][1]}
                                        for s in TUNING_SETTINGS}


def select(raw):
    recs = load_dir(raw)
    n_paths = len({r['scene'] for r in recs})
    by = collections.defaultdict(lambda: collections.defaultdict(list))
    for r in recs:
        by[r['arm']][int(r.get('seed', 0))].append(r)
    out = {'rule': "S' = S_0 + mean(S_1..3) 최대 → J' = J_0 + mean(J_1..3) 최소 → 이름 순",
           'n_paths': n_paths, 'n_records': len(recs)}
    for fam, prefix in FAMILY_PREFIX.items():
        cands = []
        for arm in sorted(by):
            if not arm.startswith(prefix):
                continue
            sp, jp, complete, per = jprime(by[arm], n_paths)
            cands.append({'arm': arm, 'S_prime': sp, 'J_prime': jp, 'complete': complete,
                          'n': sum(len(v) for v in by[arm].values()), 'per_setting': per})
        ranked = sorted([c for c in cands if c['complete']],
                        key=lambda c: (-c['S_prime'], c['J_prime'], c['arm']))
        out[fam] = {'chosen': ranked[0]['arm'] if ranked else None,
                    'ranking': ranked + [c for c in cands if not c['complete']]}
    if out['DWA_R']['chosen']:
        out['DWA_R']['chosen_x'] = out['DWA_R']['chosen'].replace('dwa_x_', '')
    if out['TB']['chosen']:
        out['TB']['chosen_t'] = out['TB']['chosen'].replace('dwa_textbook_', '')
    return out


# ---------------------------------------------------------------------------------------------
# 경로별 값과 부트스트랩 (원본 그대로)

SENTINEL_KEYS = ('cte_straight_mean', 'cte_curve_mean', 'cte_straight_max', 'cte_curve_max',
                 'settle', 'min_clear', 'cpu_ms_per_cycle', 't_v90', 't_first_entry',
                 'v_first_entry', 'overshoot_max')


def per_path(recs, arm, key, only_success=False, family=None):
    """팔 하나의 경로별 값 (seed·반복 평균). key 는 기록 키 또는 함수."""
    acc = collections.defaultdict(list)
    for r in recs:
        if r['arm'] != arm or r['family'] not in (family or HELDOUT):
            continue
        if only_success and not r['success']:
            continue
        v = key(r) if callable(key) else r.get(key)
        if v is None or (isinstance(v, float) and (math.isnan(v) or v < -0.5 and
                                                   key in SENTINEL_KEYS)):
            continue
        acc[r['scene']].append(v)
    return {k: float(np.mean(v)) for k, v in acc.items()}


def boot(values, stat=np.mean):
    v = np.asarray(list(values), dtype=float)
    if len(v) == 0:
        return (float('nan'),) * 3
    rng = np.random.default_rng(SEED)
    idx = rng.integers(0, len(v), size=(B, len(v)))
    bs = stat(v[idx], axis=1)
    return float(stat(v)), float(np.percentile(bs, 2.5)), float(np.percentile(bs, 97.5))


def paired(recs, arm_a, arm_b, key, transform=None, only_success=True, family=None):
    """경로별 (a − b) 또는 transform(a, b). 두 팔이 모두 값이 있는 경로만."""
    a = per_path(recs, arm_a, key, only_success, family)
    b = per_path(recs, arm_b, key, only_success, family)
    common = sorted(set(a) & set(b))
    f = transform or (lambda x, y: x - y)
    return {s: f(a[s], b[s]) for s in common}


def fmt_ci(m, lo, hi, scale=1.0, nd=1):
    if any(math.isnan(x) for x in (m, lo, hi)):
        return '–'
    return f'{scale * m:+.{nd}f} [{scale * lo:+.{nd}f}, {scale * hi:+.{nd}f}]'


def fmt_pct(m, lo, hi):
    if any(math.isnan(x) for x in (m, lo, hi)):
        return '–'
    return (f'{100 * (math.exp(m) - 1):+.1f} [{100 * (math.exp(lo) - 1):+.1f}, '
            f'{100 * (math.exp(hi) - 1):+.1f}]')


def med(vals, scale=1.0, nd=1):
    v = list(vals)
    return f'{scale * float(np.median(v)):.{nd}f}' if v else '–'


def success_rate(recs, arm, family=None):
    pp = per_path(recs, arm, 'success', False, family)
    return sum(1 for v in pp.values() if v >= 0.5), len(pp), pp


def logratio(x, y):
    return math.log(x / y)


def run_success(recs, arm, family=None):
    rs = [r for r in recs if r['arm'] == arm and r['family'] in (family or HELDOUT)]
    return sum(r['success'] for r in rs), len(rs)


# ---------------------------------------------------------------------------------------------
# 표

def table_summary(recs, arms, title, ref='dwa_full', family=None, extra_ref=None):
    """팔별 요약 (경로 중앙값) + 기준 팔과의 짝 차 구간 (원본 꼴 + 실행 성공 수·0→0.9 m/s)."""
    fam = family or HELDOUT
    lines = [f'### {title}', '',
             '| 팔 | 성공 매끈/통로/A* (실행) | 시간 T/T_ref 중앙 | CTE 직선 평균/최대 [cm] | '
             'CTE 곡선 평균/최대 [cm] | 정지 오차 부호 중앙 [p10, p90] [cm] | 정착 [s] | '
             '명령 저크 p99 [m/s³] | 0→0.9 m/s [s] | 통로 최소 여유 [cm] | 계산 [ms/주기] | 18 Hz 미만 |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for arm in arms:
        if not any(r['arm'] == arm for r in recs):
            continue
        sr = []
        for f1 in fam:
            k, n, _ = success_rate(recs, arm, (f1,))
            sr.append(f'{k}/{n}')
        ks, ns = run_success(recs, arm, fam)
        tr = per_path(recs, arm, lambda r: r['t_reach'] / t_lb(r['length']), True, fam)
        csm = per_path(recs, arm, 'cte_straight_mean', True, fam)
        csx = per_path(recs, arm, 'cte_straight_max', True, fam)
        ccm = per_path(recs, arm, 'cte_curve_mean', True, fam)
        ccx = per_path(recs, arm, 'cte_curve_max', True, fam)
        st = per_path(recs, arm, 'stop_along', True, fam)
        se = per_path(recs, arm, 'settle', True, fam)
        jk = per_path(recs, arm, 'cmd_jerk_p99', False, fam)
        tv = per_path(recs, arm, 't_v90', False, fam)
        mc = per_path(recs, arm, 'min_clear', False, ('narrow',))
        if any(r.get('testbed') == 'ros' for r in recs):
            cp = per_path(recs, arm, 'cpu_ms_per_cycle', False, fam)
        else:
            cp = per_path(recs, arm, lambda r: r['cycle_us_mean'] / 1000.0, False, fam)
        low = sum(1 for r in recs if r['arm'] == arm and r['family'] in fam and
                  r.get('cmd_rate_hz', 20.0) < 18.0)
        stv = list(st.values())
        stop = (f'{100 * np.median(stv):+.1f} [{100 * np.percentile(stv, 10):+.1f}, '
                f'{100 * np.percentile(stv, 90):+.1f}]') if stv else '–'
        lines.append(
            f'| {arm} | {" / ".join(sr)} ({ks}/{ns}) | {med(tr.values(), nd=3)} | '
            f'{med(csm.values(), 100)} / {med(csx.values(), 100)} | '
            f'{med(ccm.values(), 100)} / {med(ccx.values(), 100)} | {stop} | '
            f'{med(se.values(), nd=2)} | {med(jk.values())} | {med(tv.values(), nd=2)} | '
            f'{med(mc.values(), 100)} | {med(cp.values(), nd=2)} | {low} |')
    for r0 in [ref] + ([extra_ref] if extra_ref else []):
        if not any(r['arm'] == r0 for r in recs):
            continue
        lines += ['', f'기준 {r0} 와의 경로별 짝 차 (팔 − 기준), 평균 [부트스트랩 95 %], n = 둘 다 '
                  '값이 있는 경로 수:', '',
                  '| 팔 | 성공률 차 [%p] (n) | 시간 비 % (n) | CTE 곡선 평균 차 [cm] (n) | '
                  '|정지 오차| 차 [cm] (n) | CTE 직선 최대 차 [cm] (n) | 저크 p99 차 [m/s³] (n) | '
                  '0→0.9 m/s 차 [s] (n) |',
                  '| --- | --- | --- | --- | --- | --- | --- | --- |']
        for arm in arms:
            if arm == r0 or not any(r['arm'] == arm for r in recs):
                continue
            d_s = paired(recs, arm, r0, 'success', only_success=False, family=fam)
            d_t = paired(recs, arm, r0, 't_reach', logratio, family=fam)
            d_c = paired(recs, arm, r0, 'cte_curve_mean', family=fam)
            d_e = paired(recs, arm, r0, 'stop_err', family=fam)
            d_x = paired(recs, arm, r0, 'cte_straight_max', family=fam)
            d_j = paired(recs, arm, r0, 'cmd_jerk_p99', only_success=False, family=fam)
            d_v = paired(recs, arm, r0, 't_v90', only_success=False, family=fam)
            lines.append(
                f'| {arm} | {fmt_ci(*boot(d_s.values()), 100)} ({len(d_s)}) | '
                f'{fmt_pct(*boot(d_t.values()))} ({len(d_t)}) | '
                f'{fmt_ci(*boot(d_c.values()), 100)} ({len(d_c)}) | '
                f'{fmt_ci(*boot(d_e.values()), 100)} ({len(d_e)}) | '
                f'{fmt_ci(*boot(d_x.values()), 100)} ({len(d_x)}) | '
                f'{fmt_ci(*boot(d_j.values()))} ({len(d_j)}) | '
                f'{fmt_ci(*boot(d_v.values()), nd=2)} ({len(d_v)}) |')
    return '\n'.join(lines) + '\n'


IDENT_KEYS = ('t_end', 'cte_mean', 'cte_max', 'stop_along', 'stop_lat', 'min_clear')


def table_switch(recs, title, arms=CHAIN_ARMS, ref='dwa_full'):
    """표 2: 스위치 팔 − dwa_full 짝 차 (chain_sim_x). 원본 table_ablation 꼴 + 저크·가속·정지 부호."""
    lines = [f'### {title}', '',
             '| 팔 | 성공 팔·기준 | 시간 비 % (n) | CTE 곡선 평균 차 [cm] | CTE 직선 최대 차 [cm] | '
             '|정지 오차| 차 [cm] | 정지 오차 부호 중앙 팔/기준 [cm] | 저크 p99 차 [m/s³] | '
             '0→0.9 m/s 차 [s] | 통로 최소 여유 차 [cm] | 결과 동일 실행 |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    full = {(r['scene'], r.get('nseed', 0)): r for r in recs if r['arm'] == ref}
    for arm in arms:
        rs = [r for r in recs if r['arm'] == arm]
        if not rs:
            continue
        same = 0
        for r in rs:
            f = full.get((r['scene'], r.get('nseed', 0)))
            if f and all(abs(r[k] - f[k]) < 1e-9 for k in IDENT_KEYS):
                same += 1
        k_a, n_a, _ = success_rate(recs, arm)
        k_r, n_r, _ = success_rate(recs, ref)
        d_t = paired(recs, arm, ref, 't_reach', logratio)
        d_c = paired(recs, arm, ref, 'cte_curve_mean')
        d_x = paired(recs, arm, ref, 'cte_straight_max')
        d_e = paired(recs, arm, ref, 'stop_err')
        d_j = paired(recs, arm, ref, 'cmd_jerk_p99', only_success=False)
        d_v = paired(recs, arm, ref, 't_v90', only_success=False)
        d_m = paired(recs, arm, ref, 'min_clear', only_success=False, family=('narrow',))
        sa = per_path(recs, arm, 'stop_along', True)
        sr = per_path(recs, ref, 'stop_along', True)
        lines.append(
            f'| {CHAIN_LABEL.get(arm, arm)} | {k_a}/{n_a} · {k_r}/{n_r} | '
            f'{fmt_pct(*boot(d_t.values()))} ({len(d_t)}) | {fmt_ci(*boot(d_c.values()), 100)} | '
            f'{fmt_ci(*boot(d_x.values()), 100)} | {fmt_ci(*boot(d_e.values()), 100)} | '
            f'{med(sa.values(), 100, 2)} / {med(sr.values(), 100, 2)} | '
            f'{fmt_ci(*boot(d_j.values()))} | {fmt_ci(*boot(d_v.values()), nd=2)} | '
            f'{fmt_ci(*boot(d_m.values()), 100)} | {same}/{len(rs)} |')
    return '\n'.join(lines) + '\n'


def table_failures(recs, title, family=None):
    fam = family or HELDOUT
    lines = [f'### {title}', '', '| 팔 | 실패 경로 (원인) |', '| --- | --- |']
    by = collections.defaultdict(list)
    for r in recs:
        if not r['success'] and r['family'] in fam:
            why = r['fail'] if r['fail'] != 'none' else (
                'obs_collision' if r.get('obs_collision') else
                'collision' if r['collision'] else '?')
            tag = r['scene']
            if r.get('nseed'):
                tag += f"/s{r['nseed']}"
            elif r.get('testbed') == 'ros' and r.get('seed'):
                tag += f"/s{r['seed']}"
            if r.get('rep'):
                tag += f"/r{r['rep']}"
            by[r['arm']].append(f'{tag}({why})')
    for arm in sorted(by):
        lines.append(f'| {arm} | {", ".join(by[arm])} |')
    return '\n'.join(lines) + '\n'


def substitute_tuned(recs, chosen):
    """선택된 설정 팔을 'DWB_tuned' 등의 이름으로 복제한다 (원래 이름도 남김) — 원본 그대로."""
    out = list(recs)
    for fam, info in chosen.items():
        if fam not in ('DWB', 'MPPI', 'RPP') or not isinstance(info, dict) or \
                not info.get('chosen'):
            continue
        for r in recs:
            if r['arm'] == info['chosen']:
                out.append(dict(r, arm=f'{fam}_tuned'))
    return out


# ---------------------------------------------------------------------------------------------
# 확증 가설

def h1r(chain):
    recs = [r for r in chain if r['nseed'] == 0 and r['stateful'] == 0]
    d = paired(recs, 'dwa_no_m3', 'dwa_full', 't_reach', logratio)
    m, lo, hi = boot(d.values())
    k1, n1, _ = success_rate(recs, 'dwa_no_m3')
    k0, n0, _ = success_rate(recs, 'dwa_full')
    return {'hypothesis': 'H1r', 'statistic': 'mean log(T_no_m3/T_full), 두 팔 모두 성공한 경로',
            'condition': 'chain_sim_x 20 Hz, held-out B, 잡음 없음, stateful false',
            'n_paths': len(d), 'mean': m, 'ci95': [lo, hi],
            'time_ratio_pct': [100 * (math.exp(x) - 1) for x in (m, lo, hi)] if d else None,
            'rule': '95 % 하한 > 0', 'decision': 'confirmed' if d and lo > 0 else 'not confirmed',
            'success_noM3_full': [k1, n1, k0, n0]}


def h2_stat(c3, arm_r='dwa_R', arm_f='dwa_full'):
    """경로마다 두 팔 모두 성공한 seed 의 log(T_R/T_full) 평균 (seed 짝은 같은 noise_seed 번호)."""
    t = collections.defaultdict(dict)
    for r in c3:
        if r['arm'] in (arm_r, arm_f) and r['success']:
            t[(r['scene'], int(r['seed']))][r['arm']] = r['t_reach']
    per = collections.defaultdict(list)
    for (scene, seed), v in t.items():
        if arm_r in v and arm_f in v:
            per[scene].append(math.log(v[arm_r] / v[arm_f]))
    return {s: float(np.mean(v)) for s, v in per.items()}


def h2(c3):
    d = h2_stat(c3)
    m, lo, hi = boot(d.values())
    alt = paired(c3, 'dwa_R', 'dwa_full', 't_reach', logratio)   # 원본 식 (민감도)
    ma, loa, hia = boot(alt.values())
    kr, nr = run_success(c3, 'dwa_R')
    kf, nf = run_success(c3, 'dwa_full')
    return {'hypothesis': 'H2', 'statistic': '12 경로의 (seed 짝 log(T_R/T_full) 평균) 의 평균',
            'condition': 'ROS C3: 오도메트리 σ 0.02, seed 1–5, stateful false',
            'n_paths': len(d), 'mean': m, 'ci95': [lo, hi],
            'time_ratio_pct': [100 * (math.exp(x) - 1) for x in (m, lo, hi)] if d else None,
            'rule': '95 % 상한 < 0', 'decision': 'confirmed' if d and hi < 0 else 'not confirmed',
            'run_success_R_full': [kr, nr, kf, nf],
            'sensitivity_orig_formula': {'n_paths': len(alt), 'mean': ma, 'ci95': [loa, hia]}}


def h3(c1):
    d = paired(c1, 'dwa_R', 'RPP_R00', 't_reach', logratio)
    m, lo, hi = boot(d.values())
    kr, nr, _ = success_rate(c1, 'dwa_R')
    kp, np_, _ = success_rate(c1, 'RPP_R00')
    return {'hypothesis': 'H3', 'statistic': 'mean log(T_R/T_RPP_R00), 두 팔 모두 성공한 경로',
            'condition': 'ROS C1: 잡음 없음, stateful false, held-out B 60',
            'n_paths': len(d), 'mean': m, 'ci95': [lo, hi],
            'time_ratio_pct': [100 * (math.exp(x) - 1) for x in (m, lo, hi)] if d else None,
            'rule': '95 % 상한 < log(1.005)',
            'decision': 'confirmed' if d and hi < LOG_1005 else 'not confirmed',
            'success_R_RPP': [kr, nr, kp, np_]}


# ---------------------------------------------------------------------------------------------
# stateful 효과·O 장면·재설정 기제

def table_stateful(c1, c2, arms, title):
    """같은 팔의 C2(stateful true) − C1(false) 경로별 짝 차."""
    lines = [f'### {title}', '',
             '| 팔 | 성공 C2 · C1 | 시간 비 % (n) | |정지 오차| 차 [cm] (n) | 정지 오차 부호 중앙 C2/C1 [cm] | '
             '0.10 m 넘게 지나친 경로 C2/C1 |', '| --- | --- | --- | --- | --- | --- |']
    for arm in arms:
        a = [dict(r, arm='x_c2') for r in c2 if r['arm'] == arm]
        b = [dict(r, arm='x_c1') for r in c1 if r['arm'] == arm]
        if not a or not b:
            continue
        both = a + b
        d_t = paired(both, 'x_c2', 'x_c1', 't_reach', logratio)
        d_e = paired(both, 'x_c2', 'x_c1', 'stop_err')
        k2, n2, _ = success_rate(both, 'x_c2')
        k1, n1, _ = success_rate(both, 'x_c1')
        s2 = per_path(both, 'x_c2', 'stop_along', True)
        s1 = per_path(both, 'x_c1', 'stop_along', True)
        o2 = sum(1 for r in a if r.get('overshoot_max', -1) > 0.10)
        o1 = sum(1 for r in b if r.get('overshoot_max', -1) > 0.10)
        lines.append(f'| {arm} | {k2}/{n2} · {k1}/{n1} | {fmt_pct(*boot(d_t.values()))} '
                     f'({len(d_t)}) | {fmt_ci(*boot(d_e.values()), 100)} ({len(d_e)}) | '
                     f'{med(s2.values(), 100)} / {med(s1.values(), 100)} | {o2}/{o1} |')
    return '\n'.join(lines) + '\n'


def o_cause(r):
    """O 장면 실행 결과 분류: success / collision / blocked(장애물 앞 정지) / timeout / aborted_other."""
    if r['success']:
        return 'success'
    if r.get('obs_collision') or r.get('collision'):
        return 'collision'
    near = 0.0 <= r.get('obs_end_clear', 9.0) < 0.8
    stopped = 0.0 <= r.get('v_last5_max', 9.0) < 0.05
    if near and stopped:
        return 'blocked'
    return r['fail'] if r['fail'] in ('timeout', 'aborted', 'rejected') else 'other'


def table_o(o, arms, title):
    fam = ('obstacle',)
    lines = [f'### {title}', '',
             '| 팔 | 성공 실행 (장면 다수결) | 원인: 충돌 / 장애물 앞 정지 / 시간 초과 / 중단(기타) | '
             '장애물 최소 이격 중앙 [p10] [cm] (성공) | 최소 이격 최솟값 [cm] (전체) | T/T_ref 중앙 (성공) | '
             'CTE 최대 중앙 [cm] (성공) |',
             '| --- | --- | --- | --- | --- | --- | --- |']
    for arm in arms:
        rs = [r for r in o if r['arm'] == arm]
        if not rs:
            continue
        cause = collections.Counter(o_cause(r) for r in rs)
        k, n, _ = success_rate(o, arm, fam)
        clr = [r['obs_min_clear'] for r in rs if r['success']]
        allc = [r['obs_min_clear'] for r in rs if math.isfinite(r['obs_min_clear'])]
        tr = per_path(o, arm, lambda r: r['t_reach'] / t_lb(r['length']), True, fam)
        cx = per_path(o, arm, 'cte_max', True, fam)
        clr_s = (f'{100 * np.median(clr):.1f} [{100 * np.percentile(clr, 10):.1f}]'
                 if clr else '–')
        lines.append(f'| {arm} | {cause["success"]}/{len(rs)} ({k}/{n}) | {cause["collision"]} / '
                     f'{cause["blocked"]} / {cause["timeout"]} / '
                     f'{cause["aborted"] + cause["other"] + cause["rejected"]} | {clr_s} | '
                     f'{100 * min(allc):.1f} | {med(tr.values(), nd=3)} | {med(cx.values(), 100)} |'
                     if allc else f'| {arm} | – | – | – | – | – | – |')
    lines += ['', '기준 dwa_R 와의 짝 차 (팔 − dwa_R, 장면별 반복 평균, 두 팔 모두 성공한 반복만):', '',
              '| 팔 | 시간 비 % (n) | 장애물 최소 이격 차 [cm] (n) | 성공률 차 [%p] (n) |',
              '| --- | --- | --- | --- |']
    for arm in arms:
        if arm == 'dwa_R' or not any(r['arm'] == arm for r in o):
            continue
        d_t = paired(o, arm, 'dwa_R', 't_reach', logratio, family=fam)
        d_c = paired(o, arm, 'dwa_R', 'obs_min_clear', family=fam)
        d_s = paired(o, arm, 'dwa_R', 'success', only_success=False, family=fam)
        lines.append(f'| {arm} | {fmt_pct(*boot(d_t.values()))} ({len(d_t)}) | '
                     f'{fmt_ci(*boot(d_c.values()), 100)} ({len(d_c)}) | '
                     f'{fmt_ci(*boot(d_s.values()), 100)} ({len(d_s)}) |')
    return '\n'.join(lines) + '\n'


def path_ends(paths_dir):
    ends = {}
    for fn in ('heldout_smooth.json', 'heldout_narrow.json', 'heldout_astar.json'):
        p = os.path.join(paths_dir, fn)
        if os.path.exists(p):
            with open(p, encoding='utf-8') as f:
                for e in json.load(f):
                    ends[e['id']] = e['points'][-1]
    return ends


def read_trace(fn):
    with open(fn, encoding='utf-8') as f:
        rd = csv.DictReader(f)
        return [{k: float(v) for k, v in row.items()} for row in rd]


def first_entry(trace, end):
    for row in trace:
        if math.hypot(row['x'] - end[0], row['y'] - end[1]) <= 0.10:
            return row['t']
    return None


def read_resets(fn):
    """reset_log CSV → 장면별 행 목록. 장면 열(scene)이 있으면 그것으로, 없으면 전체를 '' 로."""
    out = collections.defaultdict(list)
    if not os.path.exists(fn):
        return out
    with open(fn, encoding='utf-8') as f:
        rd = csv.DictReader(f)
        for row in rd:
            out[row.get('scene', '')].append(row)
    return out


def reset_analysis(chain, chain_root, paths_dir, arms=('dwa_full', 'dwa_x_X02', 'dwa_R'),
                   window=3.0, slow_ratio=1.30):
    """잡음 실행의 느림(> +30 % 또는 실패) 과 첫 진입 전 3 s 재설정의 관계 (stateful 별)."""
    ends = path_ends(paths_dir)
    clean = {(r['arm'], r['scene'], r['stateful']): r for r in chain if r['nseed'] == 0}
    rows = []
    reset_cache = {}
    for r in chain:
        if r['nseed'] == 0 or r['arm'] not in arms:
            continue
        c = clean.get((r['arm'], r['scene'], r['stateful']))
        if c is None or not c['success']:
            continue
        slow = (not r['success']) or r['t_reach'] / c['t_reach'] > slow_ratio
        call = r['_call']
        if call not in reset_cache:
            reset_cache[call] = read_resets(os.path.join(chain_root, 'resets', f'{call}.csv'))
        rs = reset_cache[call].get(r['scene'], reset_cache[call].get('', []))
        tr_fn = os.path.join(chain_root, 'traces', call,
                             f"{r['arm']}__{r['scene']}__s{r.get('seed', 1)}.csv")
        t_first = None
        if os.path.exists(tr_fn) and r['scene'] in ends:
            t_first = first_entry(read_trace(tr_fn), ends[r['scene']])
        t_end_win = t_first if t_first is not None else r['t_end']
        win = [x for x in rs if t_end_win - window <= float(x['t']) <= t_end_win]
        rows.append({'arm': r['arm'], 'stateful': r['stateful'], 'scene': r['scene'],
                     'nseed': r['nseed'], 'slow': slow, 'success': r['success'],
                     't_first': t_first, 'n_resets': len(rs), 'n_win': len(win),
                     'win_phases': collections.Counter(x.get('phase', '?') for x in win),
                     'phases': collections.Counter(x.get('phase', '?') for x in rs),
                     'reasons': collections.Counter(x.get('reason', '?') for x in rs)})
    lines = ['### 재설정 기제 — 잡음 실행의 느림과 첫 진입 전 3 s 재설정 (chain_sim_x, held-out B, seed 1–5)', '',
             '느린 실행 = 같은 팔·stateful·경로의 잡음 없는 실행 대비 T > +30 % 또는 실패. '
             '창 = 목표 0.10 m 첫 진입 전 3 s (첫 진입이 없으면 끝 전 3 s).', '',
             '| 팔 | stateful | 묶음 | 실행 | 창 안 재설정 ≥ 1 | 창 안 재설정 평균 | 창 안 국면 (상위) | '
             '실행당 재설정 평균 (전체) |',
             '| --- | --- | --- | --- | --- | --- | --- | --- |']
    for arm in arms:
        for st in (0, 1):
            for grp, sel in (('느린', True), ('나머지', False)):
                g = [x for x in rows if x['arm'] == arm and x['stateful'] == st and
                     x['slow'] == sel]
                if not g:
                    lines.append(f'| {arm} | {st} | {grp} | 0 | – | – | – | – |')
                    continue
                ph = collections.Counter()
                for x in g:
                    ph.update(x['win_phases'])
                top = ', '.join(f'{k} {v}' for k, v in ph.most_common(3)) or '–'
                lines.append(
                    f'| {arm} | {st} | {grp} | {len(g)} | {sum(1 for x in g if x["n_win"] > 0)}/'
                    f'{len(g)} | {np.mean([x["n_win"] for x in g]):.2f} | {top} | '
                    f'{np.mean([x["n_resets"] for x in g]):.2f} |')
    return '\n'.join(lines) + '\n', rows


def reset_counts(chain, chain_root, arms):
    """실행당 재설정 수 (국면별) — 잡음 없음·잡음, stateful false."""
    lines = ['### 실행당 재설정 수 (국면별 평균, chain_sim_x, stateful false)', '',
             '| 팔 | 조건 | 실행 | 재설정/실행 | accel | cruise | decel | turn | 이유 threshold/hard/persist |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    cache = {}
    for arm in arms:
        for cond, pick in (('잡음 없음', lambda r: r['nseed'] == 0),
                           ('잡음 seed 1–5', lambda r: r['nseed'] > 0)):
            rs = [r for r in chain if r['arm'] == arm and r['stateful'] == 0 and pick(r)]
            if not rs:
                continue
            ph = collections.Counter()
            rz = collections.Counter()
            tot = 0
            for r in rs:
                call = r['_call']
                if call not in cache:
                    cache[call] = read_resets(os.path.join(chain_root, 'resets', f'{call}.csv'))
                xs = cache[call].get(r['scene'], cache[call].get('', []))
                tot += len(xs)
                ph.update(x.get('phase', '?') for x in xs)
                rz.update(x.get('reason', '?') for x in xs)
            n = len(rs)
            lines.append(f'| {arm} | {cond} | {n} | {tot / n:.2f} | {ph["accel"] / n:.2f} | '
                         f'{ph["cruise"] / n:.2f} | {ph["decel"] / n:.2f} | {ph["turn"] / n:.2f} | '
                         f'{rz["threshold"]}/{rz["hard"]}/{rz["persist"]} |')
    return '\n'.join(lines) + '\n'


def slow_runs(chain, arms, stateful=0):
    """잡음 실행 중 느린 실행 수 (> +30 % 또는 실패) — 원본 §6 의 이봉 꼴."""
    clean = {(r['arm'], r['scene']): r for r in chain if r['nseed'] == 0 and
             r['stateful'] == stateful}
    lines = [f'### 잡음 실행의 느린 실행 (chain_sim_x, stateful {"true" if stateful else "false"})', '',
             '| 팔 | 잡음 실행 | 느린 (> +30 %) | 실패 | 실행 log 시간비 중앙 | 경로 단위 시간 비 % (잡음 − 잡음 없음) |',
             '| --- | --- | --- | --- | --- | --- |']
    for arm in arms:
        rs = [r for r in chain if r['arm'] == arm and r['nseed'] > 0 and r['stateful'] == stateful]
        if not rs:
            continue
        lr = []
        slow = fail = 0
        for r in rs:
            c = clean.get((arm, r['scene']))
            if c is None or not c['success']:
                continue
            if not r['success']:
                fail += 1
                continue
            x = r['t_reach'] / c['t_reach']
            lr.append(math.log(x))
            slow += x > 1.30
        both = [dict(r, arm='x_noise') for r in rs] + \
            [dict(r, arm='x_clean') for r in chain if r['arm'] == arm and r['nseed'] == 0 and
             r['stateful'] == stateful]
        d = paired(both, 'x_noise', 'x_clean', 't_reach', logratio)
        lines.append(f'| {arm} | {len(rs)} | {slow} | {fail} | '
                     f'{100 * (math.exp(np.median(lr)) - 1) if lr else float("nan"):+.1f} % | '
                     f'{fmt_pct(*boot(d.values()))} ({len(d)}) |')
    return '\n'.join(lines) + '\n'


def rate_report(sets):
    lines = ['### 실시간 품질 (ROS): 실제 명령 주기와 부하', '',
             '| 조건 | 실행 | cmd_rate_hz 최소 / 중앙 | 18 Hz 미만 | 시작 loadavg(1 분) 범위 | '
             '참값 트랙 구독자 (≠0 실행) | commit · dirty |',
             '| --- | --- | --- | --- | --- | --- | --- |']
    for name, recs in sets:
        if not recs:
            continue
        rates = [r['cmd_rate_hz'] for r in recs]
        loads = [r['load_start'][0] for r in recs]
        subs = sum(1 for r in recs if r.get('tracks_subs', 0) != 0)
        cd = sorted({f"{r.get('commit')}·{r.get('dirty')}" for r in recs})
        lines.append(f'| {name} | {len(recs)} | {min(rates):.1f} / {np.median(rates):.1f} | '
                     f'{sum(1 for v in rates if v < 18.0)} | {min(loads):.1f}–{max(loads):.1f} | '
                     f'{subs} | {", ".join(cd)} |')
    return '\n'.join(lines) + '\n'


def table_decomp_ros(recs, arms, title, family=None):
    """prereg §8 A2: ROS 첫 진입(참값 0.10 m)·지나침·복귀 분해 (chain 표 3 과 같은 정의)."""
    fam = family or HELDOUT
    lines = [f'### {title}', '',
             '| 팔 | 성공 실행 | 첫 진입 [s] 중앙 | 첫 진입 뒤 판정까지 [s] 중앙 [p90] | 첫 진입 속도 중앙 [m/s] | '
             '0.10 m 넘게 지나친 실행 | 지나침 중앙 [cm] (지나친 실행) |',
             '| --- | --- | --- | --- | --- | --- | --- |']
    for arm in arms:
        rs = [r for r in recs if r['arm'] == arm and r['family'] in fam and r['success'] and
              r.get('t_first_entry', -1) >= 0]
        if not rs:
            continue
        post = [r['t_reach'] - r['t_first_entry'] for r in rs]
        over = [r['overshoot_max'] for r in rs if r['overshoot_max'] > 0.10]
        lines.append(f'| {arm} | {len(rs)} | {med([r["t_first_entry"] for r in rs], nd=2)} | '
                     f'{med(post, nd=2)} [{np.percentile(post, 90):.2f}] | '
                     f'{med([r["v_first_entry"] for r in rs], nd=2)} | {len(over)} | '
                     f'{med(over, 100)} |')
    for ref in ('dwa_full', 'dwa_R'):
        if not any(r['arm'] == ref for r in recs):
            continue
        lines += ['', f'첫 진입 시각 짝 차 (팔 − {ref}, log 비 → %, 두 팔 모두 성공):', '',
                  '| 팔 | 첫 진입 시간 비 % (n) | 첫 진입 뒤 시간 차 [s] (n) |', '| --- | --- | --- |']
        for arm in arms:
            if arm == ref:
                continue
            d1 = paired(recs, arm, ref, 't_first_entry', logratio, family=fam)
            d2 = paired(recs, arm, ref, lambda r: r['t_reach'] - r['t_first_entry']
                        if r.get('t_first_entry', -1) >= 0 else None, family=fam)
            if d1:
                lines.append(f'| {arm} | {fmt_pct(*boot(d1.values()))} ({len(d1)}) | '
                             f'{fmt_ci(*boot(d2.values()), nd=2)} ({len(d2)}) |')
    return '\n'.join(lines) + '\n'


def read_plugin_resets(runs_root, stage, rec):
    """플러그인 재설정 기록 (작업·팔마다 한 파일): 이 실행 시간 창의 행."""
    jid = rec['_file'][:-6]
    fn = os.path.join(runs_root, stage, 'resets', f'{jid}__{rec["arm"]}.csv')
    if not os.path.exists(fn):
        return None
    t0 = rec['t_acc_abs']
    t1 = t0 + rec['t_end']
    out = []
    with open(fn, encoding='utf-8') as f:
        for row in csv.DictReader(f):
            t = float(row['t'])
            if t0 - 0.1 <= t <= t1 + 0.1:
                row['t_rel'] = t - t0
                out.append(row)
    return out


def ros_reset_analysis(noise_recs, clean_recs, runs_root, stage, title, window=3.0):
    """
    prereg §8 A2: C3(C4) 느린 실행(같은 경로 잡음 없는 C1 대비 +30 % 초과 또는 실패)에서 첫 진입 전 3 s 안
    재설정 수 — X00(원본과 동치, 기준 C1 은 dwa_full)·dwa_R(기준 C1 dwa_R).
    """
    ref_arm = {'dwa_x_X00': 'dwa_full', 'dwa_R': 'dwa_R'}
    clean = {(r['arm'], r['scene']): r for r in clean_recs}
    lines = [f'### {title}', '',
             '| 팔 | 묶음 | 실행 | 창 안 재설정 ≥ 1 | 창 안 재설정 평균 | 창 안 국면 (상위) | 실행당 재설정 (전체) |',
             '| --- | --- | --- | --- | --- | --- | --- |']
    rows = []
    for arm, ref in ref_arm.items():
        for r in noise_recs:
            if r['arm'] != arm:
                continue
            c = clean.get((ref, r['scene']))
            if c is None or not c['success']:
                continue
            rs = read_plugin_resets(runs_root, stage, r)
            if rs is None:
                continue
            slow = (not r['success']) or r['t_reach'] / c['t_reach'] > 1.30
            tf = r['t_first_entry'] if r.get('t_first_entry', -1) >= 0 else r['t_end']
            win = [x for x in rs if tf - window <= x['t_rel'] <= tf]
            rows.append({'arm': arm, 'slow': slow, 'n_win': len(win), 'n_all': len(rs),
                         'ph': collections.Counter(x.get('phase', '?') for x in win)})
        for grp, sel in (('느린', True), ('나머지', False)):
            g = [x for x in rows if x['arm'] == arm and x['slow'] == sel]
            if not g:
                lines.append(f'| {arm} | {grp} | 0 | – | – | – | – |')
                continue
            ph = collections.Counter()
            for x in g:
                ph.update(x['ph'])
            top = ', '.join(f'{k} {v}' for k, v in ph.most_common(3)) or '–'
            lines.append(f'| {arm} | {grp} | {len(g)} | {sum(1 for x in g if x["n_win"] > 0)}/{len(g)} | '
                         f'{np.mean([x["n_win"] for x in g]):.2f} | {top} | '
                         f'{np.mean([x["n_all"] for x in g]):.2f} |')
    return '\n'.join(lines) + '\n'


def table_cond_diff(a_recs, b_recs, arms, title, name_a, name_b):
    """같은 팔의 조건 a − 조건 b 경로별 짝 차 (seed 평균 먼저)."""
    lines = [f'### {title}', '',
             f'| 팔 | 성공 실행 {name_a} · {name_b} | 시간 비 % (n) | CTE 곡선 평균 차 [cm] | |정지 오차| 차 [cm] |',
             '| --- | --- | --- | --- | --- |']
    for arm in arms:
        a = [dict(r, arm='x_a') for r in a_recs if r['arm'] == arm]
        b = [dict(r, arm='x_b') for r in b_recs if r['arm'] == arm]
        if not a or not b:
            continue
        both = a + b
        d_t = paired(both, 'x_a', 'x_b', 't_reach', logratio)
        d_c = paired(both, 'x_a', 'x_b', 'cte_curve_mean')
        d_e = paired(both, 'x_a', 'x_b', 'stop_err')
        lines.append(f'| {arm} | {sum(r["success"] for r in a)}/{len(a)} · '
                     f'{sum(r["success"] for r in b)}/{len(b)} | {fmt_pct(*boot(d_t.values()))} '
                     f'({len(d_t)}) | {fmt_ci(*boot(d_c.values()), 100)} | '
                     f'{fmt_ci(*boot(d_e.values()), 100)} |')
    return '\n'.join(lines) + '\n'


def table_m(m, arms, title):
    """prereg §8 A4: 움직이는 원판 장면 — 기술만 (판정 없음). DWA 팔만 추적 참값을 받는다."""
    fam = ('moving',)
    lines = [f'### {title}', '',
             '| 팔 | 목표 도달 실행 | 접촉 실행 (|v|>0.05 만) | 접촉 실행 (정지 중 포함) | 접촉 횟수 합 (이동/전체) | '
             '최소 이격 중앙 [p10] [cm] | 최소 이격 최솟값 [cm] | T/T_ref 중앙 (도달) | 주행 중 정지 횟수 중앙 |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for arm in arms:
        rs = [r for r in m if r['arm'] == arm and r['family'] in fam]
        if not rs:
            continue
        mc = [r['mov_min_clear'] for r in rs if r.get('mov_min_clear', -1) > -0.99 or
              r.get('contact_any', 0)]
        tr = [r['t_reach'] / t_lb(r['length']) for r in rs if r['reached']]
        lines.append(
            f'| {arm} | {sum(r["reached"] for r in rs)}/{len(rs)} | '
            f'{sum(1 for r in rs if r.get("contact_moving", 0) > 0)}/{len(rs)} | '
            f'{sum(1 for r in rs if r.get("contact_any", 0) > 0)}/{len(rs)} | '
            f'{sum(r.get("contact_moving", 0) for r in rs)}/{sum(r.get("contact_any", 0) for r in rs)} | '
            + (f'{100 * np.median(mc):.1f} [{100 * np.percentile(mc, 10):.1f}] | {100 * min(mc):.1f} | '
               if mc else '– | – | ') +
            f'{med(tr, nd=3)} | {med([r.get("n_stops", 0) for r in rs], nd=0)} |')
    lines += ['', '짝 차 (팔 − dwa_full, 장면별 반복 평균):', '',
              '| 팔 | 최소 이격 차 [cm] (n) | 시간 비 % (도달한 반복, n) | 접촉(정지 포함) 실행 비율 차 [%p] (n) |',
              '| --- | --- | --- | --- |']
    mm = [dict(r, success=r['reached']) for r in m]
    for arm in arms:
        if arm == 'dwa_full':
            continue
        d_c = paired(mm, arm, 'dwa_full', 'mov_min_clear', only_success=False, family=fam)
        d_t = paired(mm, arm, 'dwa_full', 't_reach', logratio, family=fam)
        d_k = paired(mm, arm, 'dwa_full', lambda r: float(r.get('contact_any', 0) > 0),
                     only_success=False, family=fam)
        lines.append(f'| {arm} | {fmt_ci(*boot(d_c.values()), 100)} ({len(d_c)}) | '
                     f'{fmt_pct(*boot(d_t.values()))} ({len(d_t)}) | '
                     f'{fmt_ci(*boot(d_k.values()), 100)} ({len(d_k)}) |')
    return '\n'.join(lines) + '\n'


# ---------------------------------------------------------------------------------------------
# 하네스 확인 (dryrun) · 동치 (equiv)

def dryrun_report(raw, orig_raw):
    new = load_dir(raw)
    old = load(os.path.join(orig_raw, 'ros_tuning.jsonl'))
    oldi = {(r['scene'], r['arm']): r for r in old}
    rows = []
    for r in new:
        o = oldi.get((r['scene'], r['arm']))
        if o is None:
            continue
        rows.append({'arm': r['arm'], 'scene': r['scene'], 'ok_new': r['success'],
                     'ok_old': o['success'], 't_new': r['t_reach'], 't_old': o['t_reach'],
                     'cte_new': r['cte_mean'], 'cte_old': o['cte_mean'],
                     'cmax_new': r['cte_max'], 'cmax_old': o['cte_max'],
                     'stop_new': r['stop_err'], 'stop_old': o['stop_err']})
    both = [x for x in rows if x['ok_new'] and x['ok_old']]
    dt = [100 * (x['t_new'] / x['t_old'] - 1) for x in both]
    dc = [100 * (x['cte_new'] - x['cte_old']) for x in both]
    dm = [100 * (x['cmax_new'] - x['cmax_old']) for x in both]
    ds = [100 * (x['stop_new'] - x['stop_old']) for x in both]
    succ_mismatch = [(x['arm'], x['scene'], x['ok_old'], x['ok_new']) for x in rows
                     if x['ok_new'] != x['ok_old']]
    # 원본 선택 규칙(J, 잡음 없음 1 회)을 새 실행에 적용하면 같은 설정이 뽑히는가
    sel_new = {}
    by = collections.defaultdict(list)
    for r in new:
        by[r['arm']].append(r)
    byo = collections.defaultdict(list)
    for r in old:
        byo[r['arm']].append(r)
    for fam in ('DWB', 'MPPI', 'RPP'):
        def rank(d):
            c = []
            for arm, rs in d.items():
                if arm.startswith(fam + '_'):
                    ok, j = tuning_score(rs)
                    c.append((-ok, j, arm))
            return [a for _, _, a in sorted(c)]
        sel_new[fam] = {'new_top3': rank(by)[:3], 'orig_top3': rank(byo)[:3]}
    return {'n_pairs': len(rows), 'n_both_success': len(both),
            'success_mismatch': succ_mismatch,
            'time_pct': {'median': float(np.median(dt)), 'p10': float(np.percentile(dt, 10)),
                         'p90': float(np.percentile(dt, 90)),
                         'abs_median': float(np.median(np.abs(dt))),
                         'max_abs': float(np.max(np.abs(dt)))},
            'cte_mean_cm': {'median': float(np.median(dc)), 'max_abs': float(np.max(np.abs(dc)))},
            'cte_max_cm': {'median': float(np.median(dm)), 'max_abs': float(np.max(np.abs(dm)))},
            'stop_err_cm': {'median': float(np.median(ds)), 'max_abs': float(np.max(np.abs(ds)))},
            'by_arm_family_time_pct': {
                fam: float(np.median([100 * (x['t_new'] / x['t_old'] - 1) for x in both
                                      if x['arm'].startswith(fam)]))
                for fam in ('DWB', 'MPPI', 'RPP', 'dwa_full', 'pp', 'dwa_textbook')
                if any(x['arm'].startswith(fam) for x in both)},
            'selection_orig_rule': sel_new, 'rows': rows}


EQUIV_TIME_PCT = 1.0     # prereg §3.3: 시간 ±1 %
EQUIV_CTE_CM = 0.2       # prereg §3.3: CTE ±0.2 cm (경로 평균 CTE, 반복 평균끼리)


def equiv_report(raw, arm_a='dwa_x_X00', arm_b='dwa_full', raw_b=None):
    """
    prereg §3.3 ROS 동치: 튜닝 4 경로마다 실시간 반복(rep) 평균끼리 비교.
    통과 = 모든 경로에서 성공이 같고 |시간 차| ≤ 1 % 이고 |경로 평균 CTE 차| ≤ 0.2 cm.
    CTE 최대·정지 오차는 기술로만 낸다.
    """
    recs = load_dir(raw)
    if raw_b:   # 다른 실행 묶음의 같은 팔과 비교 (trace_check·simx_check: 기록 없는 equiv_ros 의 X00 대비)
        recs = [r for r in recs if r['arm'] == arm_a] + \
            [dict(r, arm=arm_b) for r in load_dir(raw_b) if r['arm'] == arm_a]
    out = {'criterion': f'경로마다 반복 평균: |ΔT|/T ≤ {EQUIV_TIME_PCT} %, |Δ cte_mean| ≤ '
                        f'{EQUIV_CTE_CM} cm, 성공 동일', 'paths': {}}
    ok_all = True
    for scene in sorted({r['scene'] for r in recs}):
        a = [r for r in recs if r['scene'] == scene and r['arm'] == arm_a]
        b = [r for r in recs if r['scene'] == scene and r['arm'] == arm_b]
        sa = [r['success'] for r in a]
        sb = [r['success'] for r in b]
        ta = [r['t_reach'] for r in a if r['success']]
        tb = [r['t_reach'] for r in b if r['success']]
        ca = [r['cte_mean'] for r in a if r['success']]
        cb = [r['cte_mean'] for r in b if r['success']]
        ma = [r['cte_max'] for r in a if r['success']]
        mb = [r['cte_max'] for r in b if r['success']]
        ea = [r['stop_err'] for r in a if r['success']]
        eb = [r['stop_err'] for r in b if r['success']]
        res = {'n': [len(a), len(b)], 'success': [sum(sa), sum(sb)]}
        if ta and tb:
            dt = 100 * (np.mean(ta) / np.mean(tb) - 1)
            dc = 100 * (np.mean(ca) - np.mean(cb))
            res.update({'t_mean': [float(np.mean(ta)), float(np.mean(tb))],
                        't_sd': [float(np.std(ta, ddof=1)) if len(ta) > 1 else 0.0,
                                 float(np.std(tb, ddof=1)) if len(tb) > 1 else 0.0],
                        'time_diff_pct': float(dt),
                        'cte_mean_cm': [100 * float(np.mean(ca)), 100 * float(np.mean(cb))],
                        'cte_diff_cm': float(dc),
                        'cte_max_cm': [100 * float(np.mean(ma)), 100 * float(np.mean(mb))],
                        'stop_err_cm': [100 * float(np.mean(ea)), 100 * float(np.mean(eb))],
                        'cmd_rate_min': min(r['cmd_rate_hz'] for r in a + b)})
            passed = (sum(sa) == sum(sb) == len(a) == len(b) and abs(dt) <= EQUIV_TIME_PCT and
                      abs(dc) <= EQUIV_CTE_CM)
        else:
            passed = False
        res['pass'] = bool(passed)
        ok_all = ok_all and passed
        out['paths'][scene] = res
    out['pass'] = bool(ok_all)
    return out


# ---------------------------------------------------------------------------------------------

def tables(args):
    R = args.runs
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(R, 'tuning', 'selection.json'), encoding='utf-8') as f:
        sel = json.load(f)
    c1 = substitute_tuned(load_dir(os.path.join(R, 'c1', 'raw')), sel)
    c2 = substitute_tuned(load_dir(os.path.join(R, 'c2', 'raw')), sel)
    c3 = substitute_tuned(load_dir(os.path.join(R, 'c3', 'raw')), sel)
    c4 = substitute_tuned(load_dir(os.path.join(R, 'c4', 'raw')), sel) \
        if os.path.isdir(os.path.join(R, 'c4', 'raw')) else []
    mm = substitute_tuned(load_dir(os.path.join(R, 'm', 'raw')), sel) \
        if os.path.isdir(os.path.join(R, 'm', 'raw')) else []
    mc = substitute_tuned(load_dir(os.path.join(R, 'm_clr', 'raw')), sel) \
        if os.path.isdir(os.path.join(R, 'm_clr', 'raw')) else []
    o = substitute_tuned(load_dir(os.path.join(R, 'o', 'raw')), sel)
    chain = load_chain(os.path.join(R, 'chain', 'raw'))
    paths_dir = os.path.join(R, 'paths')
    md = []
    hyp = {'H1r': h1r(chain) if chain else None, 'H2': h2(c3) if c3 else None,
           'H3': h3(c1) if c1 else None}
    md.append('## 확증 가설 (prereg §6.1)\n\n```json\n' + json.dumps(hyp, indent=1,
                                                                ensure_ascii=False) + '\n```\n')
    md.append(table_summary(c1, TABLE1_ARMS, '표 1. 고전 제어기 비교 — ROS C1 (잡음 없음, stateful false), '
                            'held-out B 60', ref='dwa_full', extra_ref='dwa_R'))
    md.append(table_summary([r for r in c1 if r.get('cmd_rate_hz', 20.0) >= 18.0], TABLE1_ARMS,
                            '표 1 민감도 A. 18 Hz 미만 실행을 뺀 같은 표 (prereg §7)',
                            ref='dwa_full', extra_ref='dwa_R'))
    md.append(table_summary(c1, TABLE1_ARMS, '표 1 민감도 B. 좁은 통로(held-out A 와 같은 8 경로)를 뺀 '
                            '52 경로 (prereg 밖, 결과 보기 전 결정)', ref='dwa_full',
                            extra_ref='dwa_R', family=('smooth', 'astar')))
    md.append(table_summary(c3, TABLE1_ARMS, '표 1b. ROS C3 — 오도메트리 σ 0.02, 12 경로 × seed 1–5',
                            ref='dwa_full', extra_ref='dwa_R'))
    md.append(table_summary(c2, TABLE1_ARMS, '표 1c. ROS C2 — 잡음 없음, stateful true, held-out B 60',
                            ref='dwa_full', extra_ref='dwa_R'))
    md.append(table_stateful(c1, c2, TABLE1_ARMS, '표 1d. stateful true 효과 (같은 팔 C2 − C1, ROS)'))
    md.append(table_o(o, TABLE1_ARMS, '표 5. 정적 미지 장애물 장면 O — ROS, 20 장면 × 반복 3'))
    if c4:
        md.append(table_summary(c4, TABLE1_ARMS + ['dwa_x_X00'], '표 1e. ROS C4 — 오도메트리 σ 0.02 + 위치 잡음 '
                                '(σ 0.01 m · 0.005 rad · τ 1 s), 12 경로 × seed 1–5 (prereg §8 A3)',
                                ref='dwa_full', extra_ref='dwa_R'))
        md.append(table_cond_diff(c4, c3, TABLE1_ARMS, '표 1f. 위치 잡음 효과 (같은 팔 C4 − C3)', 'C4', 'C3'))
    md.append(table_cond_diff(c3, c1, TABLE1_ARMS + ['dwa_x_X00'],
                              '표 1g. 오도메트리 잡음 효과 (같은 팔 C3 − C1, C3 12 경로)', 'C3', 'C1'))
    md.append(table_decomp_ros(c1, TABLE1_ARMS, '표 3r. ROS 첫 진입·지나침 분해 — C1 (prereg §8 A2)'))
    md.append(table_decomp_ros(c2, TABLE1_ARMS, '표 3r-2. ROS 첫 진입·지나침 분해 — C2 stateful true'))
    md.append(table_decomp_ros(c3, TABLE1_ARMS + ['dwa_x_X00'], '표 3r-3. ROS 첫 진입·지나침 분해 — C3'))
    md.append(ros_reset_analysis(c3, c1, R, 'c3', '표 6r. ROS 재설정 기제 — C3 느린 실행의 첫 진입 전 3 s '
                                 '재설정 (prereg §8 A2; X00 의 잡음 없는 기준은 C1 dwa_full)'))
    if c4:
        md.append(ros_reset_analysis(c4, c1, R, 'c4', '표 6r-2. ROS 재설정 기제 — C4'))
    if mm:
        sel_m = ['dwa_full', 'dwa_no_m67', 'dwa_R', 'DWB_tuned', 'MPPI_tuned', 'RPP_tuned']
        md.append(table_m(mm, sel_m, '표 7. 움직이는 원판 장면 M — ROS, 20 장면 × 반복 3 (prereg §8 A4, 기술). '
                          'DWA 팔만 추적 참값을 받는다 — Nav2 팔은 코스트맵만 쓰는 참고값'))
    if mc:
        sel_m = ['dwa_full', 'dwa_no_m67', 'dwa_R', 'DWB_tuned', 'MPPI_tuned', 'RPP_tuned']
        md.append(table_m(mc, sel_m, '표 7b. M_clr — 같은 장면, 맞지 않은 빔을 11.99 m 로 (코스트맵 자국 지움; '
                          '사전 등록과 다름, 실행 전 결정)'))
    md.append(rate_report([('C1', c1), ('C2', c2), ('C3', c3), ('C4', c4), ('O', o), ('M', mm),
                           ('M_clr', mc)]))
    if chain:
        for nz, st, name in ((0, 0, '잡음 없음, stateful false'), (0, 1, '잡음 없음, stateful true'),
                             (1, 0, '잡음 seed 1–5, stateful false'),
                             (1, 1, '잡음 seed 1–5, stateful true')):
            sub = [r for r in chain if (r['nseed'] > 0) == bool(nz) and r['stateful'] == st]
            md.append(table_switch(sub, f'표 2. 스위치 (팔 − dwa_full) — chain_sim_x, held-out B 60, {name}'))
        arms_c = ['dwa_full'] + CHAIN_ARMS
        md.append(table_summary([r for r in chain if r['nseed'] == 0 and r['stateful'] == 0],
                                arms_c, '부록 A. chain_sim_x 팔별 요약 (잡음 없음, stateful false)'))
        md.append(slow_runs(chain, arms_c[:-1], 0))
        md.append(slow_runs(chain, arms_c[:-1], 1))
        txt, rows = reset_analysis(chain, os.path.join(R, 'chain'), paths_dir,
                                   arms=('dwa_full', 'dwa_x_X02', 'dwa_R'))
        md.append(txt)
        md.append(reset_counts(chain, os.path.join(R, 'chain'), arms_c))
        # M3 끔의 stateful 의존 (prereg §6.2-5 예상)
        for st in (0, 1):
            sub = [r for r in chain if r['nseed'] == 0 and r['stateful'] == st]
            d = paired(sub, 'dwa_no_m3', 'dwa_full', 't_reach', logratio)
            md.append(f'- M3 끔 − 켬 시간 비 (chain, 잡음 없음, stateful {st}): '
                      f'{fmt_pct(*boot(d.values()))} (n {len(d)})\n')
        md.append(table_failures(chain, '부록 C. 실패 목록 — chain_sim_x'))
    md.append(table_failures(c1, '부록 B1. 실패 목록 — ROS C1'))
    md.append(table_failures(c2, '부록 B2. 실패 목록 — ROS C2'))
    md.append(table_failures(c3, '부록 B3. 실패 목록 — ROS C3'))
    md.append(table_failures(o, '부록 B4. 실패 목록 — ROS O', family=('obstacle',)))
    if c4:
        md.append(table_failures(c4, '부록 B5. 실패 목록 — ROS C4'))
    with open(os.path.join(args.out, 'tables.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(md))
    with open(os.path.join(args.out, 'hypotheses.json'), 'w', encoding='utf-8') as f:
        json.dump(hyp, f, indent=1, ensure_ascii=False)
    print('\n'.join(md))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['select', 'equiv', 'dryrun', 'tables'])
    ap.add_argument('--raw', default='')
    ap.add_argument('--orig', default='/home/CAPYI_AI/runs/dwa-abl/raw')
    ap.add_argument('--runs', default='/runs')
    ap.add_argument('--out', default='')
    ap.add_argument('--raw-b', default='', help='equiv: 다른 묶음의 같은 팔 (trace_check·simx_check)')
    args = ap.parse_args(argv)
    if args.cmd == 'select':
        print(json.dumps(select(args.raw), indent=1, ensure_ascii=False))
        return 0
    if args.cmd == 'equiv':
        print(json.dumps(equiv_report(args.raw, raw_b=args.raw_b or None), indent=1,
                         ensure_ascii=False))
        return 0
    if args.cmd == 'dryrun':
        res = dryrun_report(args.raw, args.orig)
        rows = res.pop('rows')
        print(json.dumps(res, indent=1, ensure_ascii=False))
        if args.out:
            with open(args.out, 'w', encoding='utf-8') as f:
                json.dump(dict(res, rows=rows), f, indent=1)
        return 0
    return tables(args)


if __name__ == '__main__':
    sys.exit(main())
