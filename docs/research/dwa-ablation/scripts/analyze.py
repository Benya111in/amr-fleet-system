#!/usr/bin/env python3
"""
DWA 절제·고전 비교 분석 (사전 등록 prereg.md §5 — 이 파일의 sha256 을 prereg 에 적는다).

    python3 analyze.py select --raw /runs/raw            # 튜닝 집합에서 Nav2 제어기 설정 고르기
    python3 analyze.py tables --raw /runs/raw --out <dir> # 표 1–4 + 확증 H1 + 부록 표

원자료 (raw 디렉터리의 JSONL, 실행마다 한 줄):
  ros_tuning.jsonl    ROS 시험대, 튜닝 집합 4 경로 × (DWB·MPPI·RPP 격자 36 + 우리 팔)
  ros_heldout.jsonl   ROS 시험대, held-out 60 경로 × 표 1 팔 + 절제 확인 팔
  ros_noise.jsonl     ROS 시험대, 잡음 부분집합 12 경로 × 표 1 팔 × seed 1–5
  chain_clean.jsonl   chain_sim, held-out × 절제 팔 (결정적)
  chain_noise.jsonl   chain_sim, held-out × 절제 팔 × seed 1–5 (오도메트리·위치·LiDAR 잡음)
  chain_ideal.jsonl   chain_sim 이상 플랜트 (하류 사슬 없음)
  chain_rate.jsonl    chain_sim 제어 주기 10/40 Hz (M3 설명 모형 확인)
  chain_probe.jsonl   chain_sim 기전 탐침 직선 30 m (사슬·이상 × M3 × 10/20/40 Hz)
통계: 경로 = 표본 단위. 짝 차는 같은 경로의 두 팔 (잡음 seed 는 경로 안에서 먼저 평균).
구간 = 경로 묶음 재표집 부트스트랩 (B = 10 000, seed 12345) 백분위 95 %.
"""
import argparse
import collections
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
TABLE1_ARMS = ['dwa_full', 'dwa_textbook', 'DWB_D00', 'DWB_tuned', 'MPPI_P00', 'MPPI_tuned',
               'RPP_R00', 'RPP_tuned', 'pp']
ABL_ARMS = ['dwa_no_m1', 'dwa_no_m1c', 'dwa_no_m1e', 'dwa_no_m2', 'dwa_m2_trap', 'dwa_no_m3',
            'dwa_no_m4', 'dwa_no_m4a', 'dwa_no_m5', 'dwa_textbook']
FAMILY_PREFIX = {'DWB': 'DWB', 'MPPI': 'MPPI', 'RPP': 'RPP'}


def load(path):
    """path(.jsonl) 와 같은 이름의 작업자 조각 path[:-6]_w*.jsonl 을 함께 읽는다."""
    files = ([path] if os.path.exists(path) else []) + sorted(glob.glob(path[:-6] + '_w*.jsonl'))
    out = []
    for fn in files:
        with open(fn, encoding='utf-8') as f:
            out += [json.loads(line) for line in f if line.strip()]
    return out


def t_lb(length):
    """정지→정지 최소 시간 하한 (곡률 무시): L/v + v/a + a/j."""
    return length / V_MAX + V_MAX / A_MAX + A_MAX / J_MAX


# ---------------------------------------------------------------------------------------------
# 튜닝 선택 (prereg §3.2)

def tuning_score(recs):
    """
    설정 하나의 튜닝 집합 기록 → (성공 수, J). J 가 작을수록 좋다.

    J = 경로 평균 [CTE 최대/10 cm + |정지 오차|/10 cm + T/T_lb], 실패 경로 항은 10.
    """
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


def select(raw):
    recs = load(os.path.join(raw, 'ros_tuning.jsonl'))
    by = collections.defaultdict(list)
    for r in recs:
        by[r['arm']].append(r)
    out = {}
    for fam in FAMILY_PREFIX:
        cands = []
        for arm, rs in sorted(by.items()):
            if not arm.startswith(fam + '_'):
                continue
            ok, j = tuning_score(rs)
            cands.append({'arm': arm, 'n': len(rs), 'success': ok, 'J': j})
        cands.sort(key=lambda c: (-c['success'], c['J'], c['arm']))
        out[fam] = {'chosen': cands[0]['arm'] if cands else None, 'ranking': cands}
    for arm in ('dwa_full', 'pp', 'dwa_textbook'):
        if arm in by:
            ok, j = tuning_score(by[arm])
            out.setdefault('reference', []).append({'arm': arm, 'success': ok, 'J': j,
                                                    'n': len(by[arm])})
    return out


# ---------------------------------------------------------------------------------------------
# 경로별 값과 부트스트랩

def per_path(recs, arm, key, only_success=False, family=None):
    """팔 하나의 경로별 값 (seed 평균). key 는 기록 키 또는 함수."""
    acc = collections.defaultdict(list)
    for r in recs:
        if r['arm'] != arm or r['family'] not in (family or HELDOUT):
            continue
        if only_success and not r['success']:
            continue
        v = key(r) if callable(key) else r[key]
        if v is None or (isinstance(v, float) and (math.isnan(v) or v < -0.5 and key in (
                'cte_straight_mean', 'cte_curve_mean', 'cte_straight_max', 'cte_curve_max',
                'settle', 'min_clear', 'cpu_ms_per_cycle', 't_v90'))):
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


def med(vals, scale=1.0, nd=1):
    v = list(vals)
    return f'{scale * float(np.median(v)):.{nd}f}' if v else '–'


def success_rate(recs, arm, family=None):
    pp = per_path(recs, arm, 'success', False, family)
    return sum(1 for v in pp.values() if v >= 0.5), len(pp), pp


def logratio(x, y):
    return math.log(x / y)


# ---------------------------------------------------------------------------------------------
# 표

def table_summary(recs, arms, title, ref='dwa_full'):
    """팔별 요약 (경로 중앙값) + 기준 팔과의 짝 차 구간."""
    lines = [f'### {title}', '',
             '| 팔 | 성공 매끈/통로/A* | 시간 T/T_lb 중앙 | CTE 직선 평균/최대 [cm] | '
             'CTE 곡선 평균/최대 [cm] | 정지 오차 부호 중앙 [p10, p90] [cm] | 정착 [s] | '
             '명령 저크 p99 [m/s³] | 통로 최소 여유 [cm] | 계산 [ms/주기] |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for arm in arms:
        if not any(r['arm'] == arm for r in recs):
            continue
        sr = []
        for fam in HELDOUT:
            k, n, _ = success_rate(recs, arm, (fam,))
            sr.append(f'{k}/{n}')
        tr = per_path(recs, arm, lambda r: r['t_reach'] / t_lb(r['length']), True)
        csm = per_path(recs, arm, 'cte_straight_mean', True)
        csx = per_path(recs, arm, 'cte_straight_max', True)
        ccm = per_path(recs, arm, 'cte_curve_mean', True)
        ccx = per_path(recs, arm, 'cte_curve_max', True)
        st = per_path(recs, arm, 'stop_along', True)
        se = per_path(recs, arm, 'settle', True)
        jk = per_path(recs, arm, 'cmd_jerk_p99', False)
        mc = per_path(recs, arm, 'min_clear', False, ('narrow',))
        if any(r.get('testbed') == 'ros' for r in recs):
            cp = per_path(recs, arm, 'cpu_ms_per_cycle', False)
        else:
            cp = per_path(recs, arm, lambda r: r['cycle_us_mean'] / 1000.0, False)
        stv = list(st.values())
        stop = (f'{100 * np.median(stv):+.1f} [{100 * np.percentile(stv, 10):+.1f}, '
                f'{100 * np.percentile(stv, 90):+.1f}]') if stv else '–'
        lines.append(
            f'| {arm} | {" / ".join(sr)} | {med(tr.values(), nd=3)} | '
            f'{med(csm.values(), 100)} / {med(csx.values(), 100)} | '
            f'{med(ccm.values(), 100)} / {med(ccx.values(), 100)} | {stop} | '
            f'{med(se.values(), nd=2)} | {med(jk.values())} | {med(mc.values(), 100)} | '
            f'{med(cp.values(), nd=2)} |')
    lines += ['', f'기준 {ref} 와의 경로별 짝 차 (팔 − 기준), 평균 [부트스트랩 95 %], n = 둘 다 '
              '값이 있는 경로 수:', '',
              '| 팔 | 성공률 차 [%p] (n) | log 시간비 → 시간 비 % (n) | CTE 곡선 평균 차 [cm] (n) | '
              '|정지 오차| 차 [cm] (n) |',
              '| --- | --- | --- | --- | --- |']
    for arm in arms:
        if arm == ref or not any(r['arm'] == arm for r in recs):
            continue
        d_s = paired(recs, arm, ref, 'success', only_success=False)
        d_t = paired(recs, arm, ref, 't_reach', logratio)
        d_c = paired(recs, arm, ref, 'cte_curve_mean')
        d_e = paired(recs, arm, ref, 'stop_err')
        m, lo, hi = boot(d_t.values())
        tpct = '–' if math.isnan(m) else \
            f'{100 * (math.exp(m) - 1):+.1f} [{100 * (math.exp(lo) - 1):+.1f}, ' \
            f'{100 * (math.exp(hi) - 1):+.1f}]'
        lines.append(
            f'| {arm} | {fmt_ci(*boot(d_s.values()), 100)} ({len(d_s)}) | {tpct} ({len(d_t)}) | '
            f'{fmt_ci(*boot(d_c.values()), 100)} ({len(d_c)}) | '
            f'{fmt_ci(*boot(d_e.values()), 100)} ({len(d_e)}) |')
    return '\n'.join(lines) + '\n'


def table_ablation(recs, title):
    """표 2: 전체 대 하나 끔 짝 차. 같은 경로에서 완전히 같은 결과(효과 0)인 경로 수도 센다."""
    lines = [f'### {title}', '',
             '| 끈 수정 (팔) | 성공 끔/켬 | 성공률 차 [%p] | 시간 비 % (n) | CTE 곡선 평균 차 [cm] | '
             'CTE 직선 최대 차 [cm] | |정지 오차| 차 [cm] | 통로 최소 여유 차 [cm] | '
             '결과 동일 경로 |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    full = {(r['scene'], r.get('seed', 0)): r for r in recs if r['arm'] == 'dwa_full'}
    for arm in ABL_ARMS:
        rs = [r for r in recs if r['arm'] == arm]
        if not rs:
            continue
        same = 0
        for r in rs:
            f = full.get((r['scene'], r.get('seed', 0)))
            if f and all(abs(r[k] - f[k]) < 1e-9 for k in (
                    't_end', 'cte_mean', 'cte_max', 'stop_along', 'stop_lat', 'min_clear')):
                same += 1
        k_off, n_off, _ = success_rate(recs, arm)
        k_on, n_on, _ = success_rate(recs, 'dwa_full')
        d_s = paired(recs, arm, 'dwa_full', 'success', only_success=False)
        d_t = paired(recs, arm, 'dwa_full', 't_reach', logratio)
        m, lo, hi = boot(d_t.values())
        tpct = '–' if math.isnan(m) else \
            f'{100 * (math.exp(m) - 1):+.1f} [{100 * (math.exp(lo) - 1):+.1f}, ' \
            f'{100 * (math.exp(hi) - 1):+.1f}] ({len(d_t)})'
        d_c = paired(recs, arm, 'dwa_full', 'cte_curve_mean')
        d_x = paired(recs, arm, 'dwa_full', 'cte_straight_max')
        d_e = paired(recs, arm, 'dwa_full', 'stop_err')
        d_m = paired(recs, arm, 'dwa_full', 'min_clear', only_success=False, family=('narrow',))
        lines.append(
            f'| {arm} | {k_off}/{n_off} · {k_on}/{n_on} | {fmt_ci(*boot(d_s.values()), 100)} | '
            f'{tpct} | {fmt_ci(*boot(d_c.values()), 100)} | {fmt_ci(*boot(d_x.values()), 100)} | '
            f'{fmt_ci(*boot(d_e.values()), 100)} | {fmt_ci(*boot(d_m.values()), 100)} | '
            f'{same}/{len(rs)} |')
    return '\n'.join(lines) + '\n'


def table_m3(chain_clean, chain_ideal, probe, rate):
    """표 3: 하류 사슬 유무 × M3 — 탐침 직선의 a_eff, held-out 시간 비."""
    lines = ['### 표 3. M3(창 중심) 효과와 하류 사슬', '',
             '탐침 직선 30 m (정지 출발): a_eff = 0.9 m/s ÷ (0 → 0.9 m/s 시간), 한계 a = 1.0 m/s².', '',
             '| 플랜트 | 제어 주기 | a_eff 켬 [m/s²] | a_eff 끔 [m/s²] | 끔/켬 | 주행 시간 켬 / 끔 [s] |',
             '| --- | --- | --- | --- | --- | --- |']
    for plant in ('chain', 'ideal'):
        for hz in (10.0, 20.0, 40.0):
            on = [r for r in probe if r['arm'] == 'dwa_full' and r['plant'] == plant and
                  abs(r['rate'] - hz) < 1e-6]
            off = [r for r in probe if r['arm'] == 'dwa_no_m3' and r['plant'] == plant and
                   abs(r['rate'] - hz) < 1e-6]
            if not on or not off:
                continue
            a_on = 0.9 / on[0]['t_v90'] if on[0]['t_v90'] > 0 else float('nan')
            a_off = 0.9 / off[0]['t_v90'] if off[0]['t_v90'] > 0 else float('nan')
            t_on, t_off = on[0]['t_reach'], off[0]['t_reach']
            lines.append(f'| {plant} | {hz:.0f} Hz | {a_on:.3f} | {a_off:.3f} | '
                         f'{a_off / a_on:.2f} | {t_on:.2f} / {t_off:.2f} |')
    lines += ['', 'held-out 전 경로: M3 끔 − 켬 시간 비 (둘 다 성공한 경로), 평균 [95 %]:', '',
              '| 조건 | 성공 끔·켬 | 시간 비 % (n) | |정지 오차| 차 [cm] |', '| --- | --- | --- | --- |']
    for name, recs in (('사슬 20 Hz', chain_clean), ('이상 플랜트 20 Hz', chain_ideal),
                       ('사슬 10 Hz', [r for r in rate if abs(r['rate'] - 10) < 1e-6]),
                       ('사슬 40 Hz', [r for r in rate if abs(r['rate'] - 40) < 1e-6])):
        if not recs:
            continue
        d_t = paired(recs, 'dwa_no_m3', 'dwa_full', 't_reach', logratio)
        m, lo, hi = boot(d_t.values())
        d_e = paired(recs, 'dwa_no_m3', 'dwa_full', 'stop_err')
        k1, n1, _ = success_rate(recs, 'dwa_no_m3')
        k0, n0, _ = success_rate(recs, 'dwa_full')
        tpct = '–' if math.isnan(m) else \
            f'{100 * (math.exp(m) - 1):+.1f} [{100 * (math.exp(lo) - 1):+.1f}, ' \
            f'{100 * (math.exp(hi) - 1):+.1f}] ({len(d_t)})'
        lines.append(f'| {name} | {k1}/{n1} · {k0}/{n0} | {tpct} | '
                     f'{fmt_ci(*boot(d_e.values()), 100)} |')
    return '\n'.join(lines) + '\n'


def table_testbed(ros, chain, arms=('dwa_full', 'dwa_textbook', 'pp', 'dwa_no_m1', 'dwa_no_m2',
                                    'dwa_no_m3')):
    """두 시험대 차이: 같은 팔·같은 경로 (ROS − chain)."""
    lines = ['### 표 4. 두 시험대 차이 (같은 팔·같은 경로, ROS − chain_sim)', '',
             '| 팔 | 성공 ROS / chain | 시간 비 % (n) | CTE 곡선 평균 차 [cm] | 정지 오차(부호) 차 [cm] |',
             '| --- | --- | --- | --- | --- |']
    for arm in arms:
        a = [r for r in ros if r['arm'] == arm]
        b = [r for r in chain if r['arm'] == arm]
        if not a or not b:
            continue
        both = [dict(r, arm='x_ros') for r in a] + [dict(r, arm='x_chain') for r in b]
        d_t = paired(both, 'x_ros', 'x_chain', 't_reach', logratio)
        m, lo, hi = boot(d_t.values())
        tpct = '–' if math.isnan(m) else \
            f'{100 * (math.exp(m) - 1):+.1f} [{100 * (math.exp(lo) - 1):+.1f}, ' \
            f'{100 * (math.exp(hi) - 1):+.1f}] ({len(d_t)})'
        d_c = paired(both, 'x_ros', 'x_chain', 'cte_curve_mean')
        d_s = paired(both, 'x_ros', 'x_chain', 'stop_along')
        ka, na, _ = success_rate(both, 'x_ros')
        kb, nb, _ = success_rate(both, 'x_chain')
        lines.append(f'| {arm} | {ka}/{na} · {kb}/{nb} | {tpct} | '
                     f'{fmt_ci(*boot(d_c.values()), 100)} | {fmt_ci(*boot(d_s.values()), 100)} |')
    return '\n'.join(lines) + '\n'


def table_failures(recs, title):
    lines = [f'### {title}', '', '| 팔 | 실패 경로 (원인) |', '| --- | --- |']
    by = collections.defaultdict(list)
    for r in recs:
        if not r['success'] and r['family'] in HELDOUT:
            why = r['fail'] if r['fail'] != 'none' else ('collision' if r['collision'] else '?')
            by[r['arm']].append(f"{r['scene']}{'/s' + str(r['seed']) if r.get('seed') else ''}"
                                f"({why})")
    for arm in sorted(by):
        lines.append(f'| {arm} | {", ".join(by[arm])} |')
    return '\n'.join(lines) + '\n'


def h1(chain_clean, chain_ideal):
    """확증 주장 H1 (prereg §5.1)."""
    d = paired(chain_clean, 'dwa_no_m3', 'dwa_full', 't_reach', logratio)
    m, lo, hi = boot(d.values())
    di = paired(chain_ideal, 'dwa_no_m3', 'dwa_full', 't_reach', logratio)
    mi, loi, hii = boot(di.values())
    k1, n1, _ = success_rate(chain_clean, 'dwa_no_m3')
    k0, n0, _ = success_rate(chain_clean, 'dwa_full')
    return {
        'statistic': 'mean over held-out paths of log(T_noM3 / T_full), both succeeded',
        'n_paths': len(d), 'mean': m, 'ci95': [lo, hi],
        'time_ratio_pct': [100 * (math.exp(x) - 1) for x in (m, lo, hi)] if d else None,
        'decision': 'confirmed' if d and lo > 0 else 'not confirmed',
        'success_noM3_full': [k1, n1, k0, n0],
        'ideal_plant': {'n_paths': len(di), 'mean': mi, 'ci95': [loi, hii]},
    }


def substitute_tuned(recs, chosen):
    """선택된 설정 팔을 'DWB_tuned' 등의 이름으로 복제한다 (원래 이름도 남김)."""
    out = list(recs)
    for fam, info in chosen.items():
        if fam not in FAMILY_PREFIX or not info.get('chosen'):
            continue
        for r in recs:
            if r['arm'] == info['chosen']:
                out.append(dict(r, arm=f'{fam}_tuned'))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['select', 'tables'])
    ap.add_argument('--raw', required=True)
    ap.add_argument('--out', default='')
    args = ap.parse_args(argv)
    if args.cmd == 'select':
        res = select(args.raw)
        print(json.dumps(res, indent=1))
        return 0
    os.makedirs(args.out, exist_ok=True)
    chosen_f = os.path.join(args.raw, 'tuning_selection.json')
    chosen = json.load(open(chosen_f, encoding='utf-8')) if os.path.exists(chosen_f) else {}
    ros = substitute_tuned(load(os.path.join(args.raw, 'ros_heldout.jsonl')), chosen)
    ros_noise = substitute_tuned(load(os.path.join(args.raw, 'ros_noise.jsonl')), chosen)
    cc = load(os.path.join(args.raw, 'chain_clean.jsonl'))
    cn = load(os.path.join(args.raw, 'chain_noise.jsonl'))
    ci = load(os.path.join(args.raw, 'chain_ideal.jsonl'))
    cr = load(os.path.join(args.raw, 'chain_rate.jsonl'))
    cp = load(os.path.join(args.raw, 'chain_probe.jsonl'))
    md = []
    md.append(table_summary(ros, TABLE1_ARMS, '표 1. 고전 제어기 비교 — ROS 시험대, held-out 60 경로, 잡음 없음'))
    md.append(table_summary(ros_noise, TABLE1_ARMS,
                            '표 1b. 같은 비교 — ROS 시험대, 잡음 부분집합 12 경로 × seed 5 (오도메트리 σ 0.02)'))
    md.append(table_summary([r for r in ros if r.get('cmd_rate_hz', 20.0) >= 18.0], TABLE1_ARMS,
                            '표 1 민감도. 제어 주기가 부하로 18 Hz 아래로 떨어진 실행을 뺀 같은 표'))
    md.append(table_ablation(cc, '표 2. 절제 (전체 − 하나 끔) — chain_sim, held-out 60 경로, 잡음 없음'))
    md.append(table_ablation(cn, '표 2b. 절제 — chain_sim, held-out 60 경로 × seed 5 (잡음 조건)'))
    md.append(table_ablation(ros, '표 2c. 절제 확인 — ROS 시험대 (M1·M2·M3·교과서 팔만)'))
    md.append(table_m3(cc, ci, cp, cr))
    md.append(table_testbed(ros, cc))
    md.append(table_summary(cc, ['dwa_full', 'dwa_textbook', 'pp'] + ABL_ARMS[:-1],
                            '부록 A. chain_sim 팔별 요약 (held-out, 잡음 없음)'))
    md.append(table_failures(ros, '부록 B. 실패 목록 — ROS held-out'))
    md.append(table_failures(cc, '부록 C. 실패 목록 — chain_sim held-out'))
    hyp = h1(cc, ci)
    md.append('### 확증 주장 H1\n\n```json\n' + json.dumps(hyp, indent=1) + '\n```\n')
    with open(os.path.join(args.out, 'tables.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(md))
    with open(os.path.join(args.out, 'h1.json'), 'w', encoding='utf-8') as f:
        json.dump(hyp, f, indent=1)
    print('\n'.join(md))
    return 0


if __name__ == '__main__':
    sys.exit(main())
