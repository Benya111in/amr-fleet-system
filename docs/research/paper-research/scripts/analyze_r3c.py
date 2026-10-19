#!/usr/bin/env python3
"""
R3-C 분석 (prereg_r3c.md §1·§3·§4). **확증 가설이 없다 — 모든 수치는 기술이다.** results_r3.md 가 커밋된 뒤에만 돈다.

    python3 analyze_r3c.py --runs /home/CAPYI_AI/paper_research/runs --out <analysis 디렉터리>
        → <out>/tables_r3c.md, <out>/values_r3c.json, <out>/fig2_startup_D.dat

정의 (prereg_r3c §4, 1–3 라운드와 같음):
- t0.9: 궤적에서 첫 명령 v_c > 0.01 m/s 시각부터 측정 v_m ≥ 0.9 m/s 첫 시각까지 (ROS: odometry/filtered,
  chain: 서보 출력). 그림 2 추출(/home/CAPYI_AI/paper/cc-dwa/tools/fig_startup_extract.py)과 같은 식.
- 가속 재설정: 플러그인 재설정 기록의 phase = accel 이고 수락 뒤 2 s 안 (t_rel ≤ 2.0, posthoc_r3.resets_c1 과 같음).
- 느린 실행: 같은 팔·같은 경로의 C1(같은 배치) 대비 +30 % 초과 또는 실패 (analyze_r3 표 R3-7 과 같음).
- Δ시간: 두 팔 모두 성공한 경로의 mean log(T_팔/T_기준) → %, 경로 묶음 부트스트랩 B 10 000 (seed 12345,
  analyze_x.boot). C3 는 경로 안 seed 평균을 먼저 낸다 (analyze_x.per_path).
- 진입 방위 오차: 참값이 목표 0.10 m 안에 처음 든 순간 |yaw 오차| > 0.05 rad 인 경로 수 (full_r1r2.entry_yaw 와 같음).
- 접촉·최소 이격·곡선 CTE·정지 오차: analyze_x 정의 그대로. |정지 오차| = stop_err (성공 실행) 경로 중앙.
- E4: 감싼 팔(tm_*) 주기 기록에서 실행 창 [t_acc, t_res] 의 dt_us. 경로 평균 → 비율은 경로 묶음 부트스트랩
  (같은 경로 색인으로 두 팔을 함께 뽑아 평균의 비).
"""
import argparse
import collections
import csv
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze_x as ax  # noqa: E402
import params_r3c as pc  # noqa: E402

B, SEED = ax.B, ax.SEED
FAMS3 = ('smooth', 'narrow', 'astar')


# ---------------------------------------------------------------------------------------------
# 읽기

def stage(R, rel):
    d = os.path.join(R, rel, 'raw')
    return ax.load_dir(d) if os.path.isdir(d) else []


def fnum(x):
    return float(x) if x not in ('', None) else float('nan')


_trace_cache = {}


def read_trace_ros(R, rel, r):
    fn = os.path.join(R, rel, 'traces', r['_file'][:-6],
                      f"{r['arm']}__{r['scene']}__s{r.get('seed', 0)}__r{r.get('rep', 0)}.csv")
    if fn in _trace_cache:
        return _trace_cache[fn]
    if not os.path.exists(fn):
        return None
    rows = []
    with open(fn, encoding='utf-8') as f:
        for x in csv.DictReader(f):
            if x['v_meas'] == '':
                continue
            rows.append((float(x['t']), fnum(x['v_meas']), fnum(x['v_cmd']), fnum(x['x']), fnum(x['y']),
                         fnum(x['yaw'])))
    _trace_cache[fn] = rows
    return rows


def read_trace_chain(root, r):
    fn = os.path.join(root, 'traces', r['_call'], f"{r['arm']}__{r['scene']}__s{r.get('seed', 1)}.csv")
    if not os.path.exists(fn):
        return None
    rows = []
    with open(fn, encoding='utf-8') as f:
        for x in csv.DictReader(f):
            rows.append((float(x['t']), float(x['v']), float(x['v_cmd']), float(x['x']), float(x['y']),
                         float(x['theta'])))
    return rows


def t_cmd0(rows):
    for t, vm, vc, *_ in rows:
        if not math.isnan(vc) and vc > 0.01:
            return t
    return None


def t09(rows, thr=0.9):
    """첫 명령 > 0.01 → v_m ≥ thr 첫 시각 (fig_startup_extract / fig_startup_stats 와 같다)."""
    if not rows:
        return None
    t0 = t_cmd0(rows)
    if t0 is None:
        return None
    for t, vm, vc, *_ in rows:
        if t >= t0 and vm >= thr:
            return t - t0
    return None


def slope36(rows):
    """출발 구간 v_m 0.3 → 0.6 m/s 기울기 [m/s²] (P2)."""
    t0 = t_cmd0(rows) if rows else None
    if t0 is None:
        return None
    t3 = t6 = None
    for t, vm, *_ in rows:
        if t < t0:
            continue
        if t3 is None and vm >= 0.3:
            t3 = t
        if t6 is None and vm >= 0.6:
            t6 = t
            break
    if t3 is None or t6 is None or t6 <= t3:
        return None
    return 0.3 / (t6 - t3)


def start_gap_max(rows, win=2.0):
    """출발 2 s 안 v_c − v_m 최대 (ROS 궤적: v_c 는 마지막 방출 명령 = 다음 주기의 v_c^{k-1})."""
    t0 = t_cmd0(rows) if rows else None
    if t0 is None:
        return None
    g = [vc - vm for t, vm, vc, *_ in rows if t0 <= t <= t0 + win and not math.isnan(vc)]
    return max(g) if g else None


def entry_yaw(rows, t_first, goal_yaw):
    if t_first is None or t_first < 0 or not rows:
        return None
    for t, vm, vc, x, y, yaw in rows:
        if t >= t_first - 1e-6 and not math.isnan(yaw):
            return abs((yaw - goal_yaw + math.pi) % (2 * math.pi) - math.pi)
    return None


def path_goals(R):
    g = {}
    for fn in ('heldout_smooth.json', 'heldout_narrow.json', 'heldout_astar.json'):
        with open(os.path.join(R, 'r3', 'paths_d', fn), encoding='utf-8') as f:
            for e in json.load(f):
                g[e['id']] = e['points'][-1]
    return g


def resets_of(R, rel, r):
    rs = ax.read_plugin_resets(R, rel, r)
    return rs


def accel2(rs):
    return sum(1 for x in rs if x.get('phase') == 'accel' and x['t_rel'] <= 2.0)


# ---------------------------------------------------------------------------------------------
# 요약 함수

def boot_ci(vals):
    return ax.boot(list(vals))


def pct(m):
    return 100.0 * (math.exp(m) - 1.0)


def fmt_pct(t):
    return ax.fmt_pct(*t)


def delta_time(recs, arm, ref, family=None):
    d = ax.paired(recs, arm, ref, 't_reach', ax.logratio, family=family)
    m, lo, hi = boot_ci(d.values())
    return {'n': len(d), 'mean_log': m, 'ci_log': [lo, hi],
            'pct': [pct(m), pct(lo), pct(hi)] if d else None}


def med(v):
    v = [x for x in v if x is not None and not (isinstance(x, float) and math.isnan(x))]
    return float(np.median(v)) if v else None


def arm_c1_summary(recs, arm, R=None, rel=None, goals=None):
    rs = [r for r in recs if r['arm'] == arm and r['family'] in FAMS3]
    if not rs:
        return None
    ok = [r for r in rs if r['success']]
    out = {'runs': len(rs), 'success': sum(r['success'] for r in rs),
           'success_by_family': {f: [sum(r['success'] for r in rs if r['family'] == f),
                                     sum(1 for r in rs if r['family'] == f)] for f in FAMS3},
           't_tref_med': med([r['t_reach'] / ax.t_lb(r['length']) for r in ok]),
           'cte_curve_cm_med': med([100 * r['cte_curve_mean'] for r in ok if r['cte_curve_mean'] >= 0]),
           'stop_err_cm_med': med([100 * r['stop_err'] for r in ok]),
           'stop_along_cm_med': med([100 * r['stop_along'] for r in ok]),
           'cmd_rate_min': min(r['cmd_rate_hz'] for r in rs) if rs and 'cmd_rate_hz' in rs[0] else None}
    if R is not None:
        t9 = []
        ey = []
        gaps = []
        resets = []
        acc2 = []
        hard_acc = 0
        for r in rs:
            tr = read_trace_ros(R, rel, r)
            t9.append(t09(tr))
            gaps.append(start_gap_max(tr))
            if goals and r['success'] and r.get('t_first_entry', -1) >= 0:
                ey.append(entry_yaw(tr, r['t_first_entry'], goals[r['scene']][2]))
            rr = resets_of(R, rel, r)
            if rr is not None:
                resets.append(len(rr))
                acc2.append(accel2(rr))
                hard_acc += sum(1 for x in rr if x.get('reason') == 'hard' and x.get('phase') == 'accel')
        t9v = [x for x in t9 if x is not None]
        out.update({'t09_med': med(t9v), 't09_n': len(t9v), 't09_by_scene': {r['scene']: t for r, t in zip(rs, t9)},
                    'start_gap_max': {'med': med(gaps), 'p10': _pctl(gaps, 10), 'p90': _pctl(gaps, 90),
                                      'max': max([g for g in gaps if g is not None], default=None)},
                    'entry_yaw_over_005': sum(1 for e in ey if e is not None and e > 0.05),
                    'entry_yaw_n': sum(1 for e in ey if e is not None)})
        if resets:
            out.update({'resets_per_run': float(np.mean(resets)), 'accel2_per_run': float(np.mean(acc2)),
                        'reset_logged_runs': len(resets), 'hard_accel_resets': hard_acc})
    return out


def _pctl(v, q):
    v = [x for x in v if x is not None]
    return float(np.percentile(v, q)) if v else None


def slow_count(c3, c1, arm, ref_arm=None):
    clean = {(r['arm'], r['scene']): r for r in c1}
    ref_arm = ref_arm or arm
    n = slow = fail = 0
    for r in c3:
        if r['arm'] != arm:
            continue
        n += 1
        c = clean.get((ref_arm, r['scene']))
        if c is None or not c['success']:
            continue
        if not r['success']:
            slow += 1
            fail += 1
        elif r['t_reach'] / c['t_reach'] > 1.30:
            slow += 1
    return {'runs': n, 'slow': slow, 'fail': fail}


def o_summary(o, arm):
    rs = [r for r in o if r['arm'] == arm]
    if not rs:
        return None
    cause = collections.Counter(ax.o_cause(r) for r in rs)
    allc = [r['obs_min_clear'] for r in rs if math.isfinite(r['obs_min_clear'])]
    return {'runs': len(rs), 'success': sum(r['success'] for r in rs),
            'contact': sum(1 for r in rs if r.get('obs_collision') or r.get('collision')),
            'cause': dict(cause), 'min_clear_cm_min': 100 * min(allc) if allc else None,
            'min_clear_cm_med_success': med([100 * r['obs_min_clear'] for r in rs if r['success']])}


# ---------------------------------------------------------------------------------------------
# chain

def chain_load(root):
    d = os.path.join(root, 'raw')
    return ax.load_chain(d) if os.path.isdir(d) else []


def chain_arm(chain, root, arm, R3=False):
    rs = [r for r in chain if r['arm'] == arm and r['nseed'] == 0 and r['stateful'] == 0]
    t9 = []
    sl = []
    for r in rs:
        tr = read_trace_chain(root, r)
        t9.append(t09(tr))
        sl.append(slope36(tr))
    acc = []
    cache = {}
    for r in rs:
        call = r['_call']
        if call not in cache:
            cache[call] = ax.read_resets(os.path.join(root, 'resets', f'{call}.csv'))
        xs = cache[call].get(r['scene'], [])
        acc.append(sum(1 for x in xs if x.get('phase') == 'accel'))
    return {'runs': len(rs), 'success': sum(r['success'] for r in rs), 't09_med': med(t9),
            'slope36_med': med(sl), 'accel_resets_per_run': float(np.mean(acc)) if acc else None}


# ---------------------------------------------------------------------------------------------
# E4

def timing_rows(R, rel, jid, tarm):
    fn = os.path.join(R, rel, 'timing', f'{jid}__{tarm}.csv')
    if not os.path.exists(fn):
        return None
    t, dt = [], []
    with open(fn, encoding='utf-8') as f:
        for line in f:
            if line.startswith('#') or line.startswith('t_ros'):
                continue
            a = line.strip().split(',')
            if len(a) >= 2:
                t.append(float(a[0]))
                dt.append(float(a[1]))
    return np.array(t), np.array(dt)


def e4_per_run(R, rel, recs):
    out = []
    cache = {}
    for r in recs:
        if not r['arm'].startswith('tm_'):
            continue
        key = (r['_file'][:-6], r['arm'])
        if key not in cache:
            cache[key] = timing_rows(R, rel, *key)
        tr = cache[key]
        if tr is None:
            continue
        t, dt = tr
        m = (t >= r['t_acc_abs'] - 1e-6) & (t <= r['t_res_abs'] + 1e-6)
        x = dt[m] / 1000.0
        if not len(x):
            continue
        out.append({'arm': r['arm'][3:], 'scene': r['scene'], 'n': int(len(x)), 'mean_ms': float(x.mean()),
                    'p50_ms': float(np.median(x)), 'p99_ms': float(np.percentile(x, 99)),
                    'max_ms': float(x.max()), 'success': r['success'],
                    'n_samples_med': r.get('n_samples_med', -1), 'rollout_steps_med': r.get('rollout_steps_med', -1),
                    'cmd_rate_hz': r['cmd_rate_hz']})
    return out


def ratio_boot(a, b):
    """경로별 평균 a·b (같은 경로) → mean(a)/mean(b) 와 경로 묶음 부트스트랩 95 %."""
    common = sorted(set(a) & set(b))
    va = np.array([a[s] for s in common])
    vb = np.array([b[s] for s in common])
    if not len(common):
        return None
    rng = np.random.default_rng(SEED)
    idx = rng.integers(0, len(common), size=(B, len(common)))
    rs = va[idx].mean(axis=1) / vb[idx].mean(axis=1)
    return {'n': len(common), 'ratio': float(va.mean() / vb.mean()),
            'ci': [float(np.percentile(rs, 2.5)), float(np.percentile(rs, 97.5))]}


def e4_summary(rows, arms5):
    out = {'arms': {}}
    by = collections.defaultdict(lambda: collections.defaultdict(list))
    for x in rows:
        by[x['arm']][x['scene']].append(x)
    pm = {}
    for arm in arms5:
        xs = [x for s in by[arm].values() for x in s]
        if not xs:
            continue
        pm[arm] = {s: float(np.mean([x['mean_ms'] for x in v])) for s, v in by[arm].items()}
        allx = [x for x in xs]
        out['arms'][arm] = {'runs': len(xs), 'paths': len(pm[arm]),
                            'mean_ms': float(np.mean(list(pm[arm].values()))),
                            'p50_ms_med': med([x['p50_ms'] for x in allx]),
                            'p99_ms_med': med([x['p99_ms'] for x in allx]),
                            'max_ms': max(x['max_ms'] for x in allx),
                            'cycles': sum(x['n'] for x in allx),
                            'n_samples_med': med([x['n_samples_med'] for x in allx if x['n_samples_med'] >= 0]),
                            'rollout_steps_med': med([x['rollout_steps_med'] for x in allx
                                                      if x['rollout_steps_med'] >= 0]),
                            'cmd_rate_min': min(x['cmd_rate_hz'] for x in allx)}
    if 'dwa_R3' in pm:
        for arm in arms5[1:]:
            if arm in pm:
                out[f'ratio_dwa_R3_over_{arm}'] = ratio_boot(pm['dwa_R3'], pm[arm])
    return out


def e4eq_report(recs, arms5):
    """
    prereg_r3c §3 E4 동치: 경로마다 반복 평균, 감쌈(tm_X) 대 안 감쌈(X) 시간 ±1 %, CTE ±0.2 cm, 명령 주기 ≥ 19.7 Hz.
    시간·CTE 는 모든 실행의 t_end·cte_mean 반복 평균 (성공이면 t_end = t_reach, 시간 초과는 상한 시각) 이고, 성공 수가
    같아야 한다. 'pass_success_only' 는 두 팔 모두 모든 반복이 성공한 경로만 보는 엄격판 (analyze_x.equiv_report 꼴).
    """
    rep = {'criterion': '경로마다 반복 평균: 성공 수 같음, 성공 실행의 |ΔT|/T ≤ 1 %·|Δ cte_mean| ≤ 0.2 cm '
                        '(성공이 없으면 실패 실행의 t_end ±1 %), 명령 주기 ≥ 19.7 Hz', 'arms': {}}
    ok_all = True
    for arm in arms5:
        sub = [r for r in recs if r['arm'] in (arm, f'tm_{arm}')]
        res = {}
        ok = True
        ok_s = True
        for sc in sorted({r['scene'] for r in sub}):
            a = [r for r in sub if r['scene'] == sc and r['arm'] == f'tm_{arm}']
            b = [r for r in sub if r['scene'] == sc and r['arm'] == arm]
            rate = min(r['cmd_rate_hz'] for r in a + b)
            d = {'n': [len(a), len(b)], 'success': [sum(r['success'] for r in a), sum(r['success'] for r in b)],
                 'cmd_rate_min': rate,
                 't_end': [[round(r['t_end'], 3) for r in a], [round(r['t_end'], 3) for r in b]]}
            sa = [r for r in a if r['success']]
            sb = [r for r in b if r['success']]
            if sa and sb:
                ta, tb = np.mean([r['t_reach'] for r in sa]), np.mean([r['t_reach'] for r in sb])
                d['time_diff_pct'] = 100 * (ta / tb - 1)
                d['cte_diff_cm'] = 100 * (np.mean([r['cte_mean'] for r in sa]) - np.mean([r['cte_mean'] for r in sb]))
            else:
                d['time_diff_pct'] = 100 * (np.mean([r['t_end'] for r in a]) / np.mean([r['t_end'] for r in b]) - 1)
                d['cte_diff_cm'] = 0.0
            fa = [r['t_end'] for r in a if not r['success']]
            fb = [r['t_end'] for r in b if not r['success']]
            d['fail_t_end_diff_pct'] = 100 * (np.mean(fa) / np.mean(fb) - 1) if fa and fb else None
            p = (d['success'][0] == d['success'][1] and len(a) == len(b) and abs(d['time_diff_pct']) <= 1.0 and
                 abs(d['cte_diff_cm']) <= 0.2 and rate >= 19.7 and
                 (d['fail_t_end_diff_pct'] is None or abs(d['fail_t_end_diff_pct']) <= 1.0))
            d['pass'] = bool(p)
            d['all_success'] = d['success'][0] == d['success'][1] == len(a) == len(b)
            ok = ok and p
            ok_s = ok_s and (p if d['all_success'] else True)
            res[sc] = d
        rep['arms'][arm] = {'pass': bool(ok), 'pass_success_only': bool(ok_s), 'paths': res}
        ok_all = ok_all and ok
    rep['pass'] = bool(ok_all)
    return rep


def e4eq2_report(recs):
    """E4 동치 보충 (MPPI): 구성 순서 A/B × 반복 3 를 합친 감쌈 대 안 감쌈, 그리고 경로마다 먼저 돈 쪽·나중 쪽."""
    arms = sorted({r['arm'] for r in recs})
    tm = [a for a in arms if a.startswith('tm_')][0]
    base = tm[3:]
    out = {'arm': base, 'paths': {}}
    ok = True
    for sc in sorted({r['scene'] for r in recs}):
        rows = []
        by_job = collections.defaultdict(list)
        for r in recs:
            if r['scene'] == sc:
                by_job[r['_file']].append(r)
        for job, rs in by_job.items():
            rs = sorted(rs, key=lambda r: r['t_acc_abs'])
            for i, r in enumerate(rs):
                rows.append({'job': job[:-6], 'arm': r['arm'], 'run_order': i, 't_end': round(r['t_end'], 3),
                             'success': r['success'], 'cte_cm': round(100 * r['cte_mean'], 3)})
        a = [x for x in rows if x['arm'] == tm]
        b = [x for x in rows if x['arm'] == base]
        dt = 100 * (np.mean([x['t_end'] for x in a]) / np.mean([x['t_end'] for x in b]) - 1)
        dc = np.mean([x['cte_cm'] for x in a]) - np.mean([x['cte_cm'] for x in b])
        first = [x for x in rows if x['run_order'] == 0]
        second = [x for x in rows if x['run_order'] == 1]
        p = (sum(x['success'] for x in a) == sum(x['success'] for x in b) and abs(dt) <= 1.0 and abs(dc) <= 0.2)
        ok = ok and p
        out['paths'][sc] = {'time_diff_pct': float(dt), 'cte_diff_cm': float(dc), 'pass': bool(p),
                            'first_run_t': {'tm': [x['t_end'] for x in first if x['arm'] == tm],
                                            'base': [x['t_end'] for x in first if x['arm'] == base]},
                            'second_run_t': {'tm': [x['t_end'] for x in second if x['arm'] == tm],
                                             'base': [x['t_end'] for x in second if x['arm'] == base]},
                            'rows': rows}
    out['pass'] = bool(ok)
    return out


# ---------------------------------------------------------------------------------------------
# 그림 2 (E6)

def fig2(R, sel_rpp, arms_src, out_dat):
    """arms_src: {'hold': (rel, arm), 'thr': …, 'mdl': …, 'mid': …, 'rpp': …}. 경로 선택 규칙 prereg_r3c §3 E6."""
    t9 = {}
    rec_of = {}
    for key, (rel, arm) in arms_src.items():
        recs = [r for r in stage(R, rel) if r['arm'] == arm and r['family'] in FAMS3]
        t9[key] = {}
        for r in recs:
            v = t09(read_trace_ros(R, rel, r))
            if v is not None:
                t9[key][r['scene']] = v
                rec_of[(key, r['scene'])] = (rel, r)
    rule_keys = ('hold', 'thr', 'mdl', 'rpp')
    meds = {k: float(np.median(list(t9[k].values()))) for k in rule_keys}
    common = sorted(set.intersection(*[set(t9[k]) for k in rule_keys]))
    score = {s: sum(abs(t9[k][s] - meds[k]) for k in rule_keys) for s in common}
    best = sorted(common, key=lambda s: (score[s], s))[0]
    series = {}
    for key in ('rpp', 'thr', 'mdl', 'mid', 'hold'):
        rel, r = rec_of[(key, best)] if (key, best) in rec_of else (None, None)
        if r is None:
            series[key] = {}
            continue
        rows = read_trace_ros(R, rel, r)
        t0 = t_cmd0(rows)
        series[key] = {round(t - t0, 2): (vm, vc) for t, vm, vc, *_ in rows if -1e-9 <= t - t0 <= 2.6 + 1e-9}
    grid = sorted(set.intersection(*[set(s) for s in series.values() if s]))
    with open(out_dat, 'w', encoding='utf-8') as f:
        f.write('t rpp_m rpp_c thr_m thr_c mdl_m mdl_c mid_m mid_c hold_m hold_c\n')
        for t in grid:
            f.write(f'{t:.3f} ' + ' '.join(f'{series[k][t][0]:.4f} {series[k][t][1]:.4f}'
                                           for k in ('rpp', 'thr', 'mdl', 'mid', 'hold')) + '\n')
    return {'path': best, 'score': score[best], 'medians': meds, 'n_common': len(common),
            't09_on_path': {k: t9[k].get(best) for k in t9}, 'rows': len(grid),
            'sources': {k: list(v) for k, v in arms_src.items()}}


# ---------------------------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', default='/home/CAPYI_AI/paper_research/runs')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    R = args.runs
    os.makedirs(args.out, exist_ok=True)
    sel = pc.committed_selection()
    zsel = sel['DWA_R3']['chosen_z']
    dwb, mppi, rpp = sel['DWB']['chosen'], sel['MPPI']['chosen'], sel['RPP']['chosen']
    arms5 = ['dwa_R3', dwb, mppi, rpp, 'dwa_textbook_tuned']
    vlist = pc.variant_list(zsel)
    goals = path_goals(R)
    V = {'selection': {'chosen_z': zsel, 'DWB': dwb, 'MPPI': mppi, 'RPP': rpp},
         'variants': [{'id': v[0], 'arm': v[1], 'row': v[2], 'over': v[3], 'status': v[4]} for v in vlist]}
    research, top = pc.sel_spec(zsel)
    V['selection_yaml'] = {'research': research, 'top': top,
                           'K': research.get('reset_persist_cycles'), 'D': research.get('goal_extension_dist'),
                           'S': research.get('blocked_detour_shift'), 'accel_hold': research.get('accel_hold', False),
                           'reset_model': research.get('reset_model', False)}

    # --- R3 (재실행 없음) ---
    r3 = {s: stage(R, f'r3/{s}') for s in ('c1', 'c2', 'c3', 'o')}
    V['R3'] = {'table1': {}, 'o': {}, 'c3_delta': {}, 'c1_delta': {}, 'c2_delta': {}}
    for arm in arms5:
        V['R3']['table1'][arm] = arm_c1_summary(r3['c1'], arm, R, 'r3/c1', goals)
        V['R3']['o'][arm] = o_summary(r3['o'], arm)
        if arm != 'dwa_R3':
            V['R3']['c1_delta'][arm] = delta_time(r3['c1'], arm, 'dwa_R3')
            V['R3']['c3_delta'][arm] = delta_time(r3['c3'], arm, 'dwa_R3')
            V['R3']['c2_delta'][arm] = delta_time(r3['c2'], arm, 'dwa_R3')
        V['R3'].setdefault('c2', {})[arm] = arm_c1_summary(r3['c2'], arm)
        V['R3'].setdefault('c3_vs_c1', {})[arm] = _cond_diff(r3['c3'], r3['c1'], arm)
        V['R3'].setdefault('stop_err_diff_vs_R3', {})[arm] = _paired_ci(r3['c1'], arm, 'dwa_R3', 'stop_err', 100)
        V['R3'].setdefault('cte_curve_diff_vs_R3', {})[arm] = _paired_ci(r3['c1'], arm, 'dwa_R3',
                                                                         'cte_curve_mean', 100)
    V['R3']['slow_c3_dwa_R3'] = slow_count(r3['c3'], r3['c1'], 'dwa_R3')
    V['R3']['slow_c3_X00'] = slow_count(r3['c3'], r3['c1'], 'dwa_x_X00', 'dwa_full')
    V['R3']['x00_c3_decel_window'] = _slow_window(R, r3['c3'], r3['c1'], 'dwa_x_X00', 'dwa_full')
    V['R3']['dwa_R2_c1_resets'] = arm_c1_summary(r3['c1'], 'dwa_R2', R, 'r3/c1', goals)
    hyp_fn = os.path.join(HERE, '..', 'analysis', 'hypotheses_r3.json')
    if os.path.exists(hyp_fn):
        with open(hyp_fn, encoding='utf-8') as f:
            V['R3']['hypotheses'] = json.load(f)
    # C2 순위 (5 팔, 경로 중앙 T/T_ref)
    V['R3']['rank_c1'] = _rank(r3['c1'], arms5)
    V['R3']['rank_c2'] = _rank(r3['c2'], arms5)
    # R3 chain (dwa_R2·dwa_R3)
    ch3 = chain_load(os.path.join(R, 'r3', 'chain'))
    V['R3']['chain'] = {a: chain_arm(ch3, os.path.join(R, 'r3', 'chain'), a) for a in ('dwa_R2', 'dwa_R3')}
    V['R3']['testbed_gap'] = _testbed_gap(ch3, r3['c1'], 'dwa_R3')
    # 명령 주기 최소 (R3 전 단계)
    V['rates'] = {}
    for rel in ('r3/tuning', 'r3/c1', 'r3/c2', 'r3/c3', 'r3/o', 'r3c/e2', 'r3c/c1', 'r3c/c3', 'r3c/o', 'r3c/e4eq',
                'r3c/e4eq2', 'r3c/smoke_timed'):
        rs = stage(R, rel)
        if rs:
            V['rates'][rel] = {'runs': len(rs), 'cmd_rate_min': min(r['cmd_rate_hz'] for r in rs),
                               'below18': sum(1 for r in rs if r['cmd_rate_hz'] < 18.0),
                               'tracks_subs_nonzero': sum(1 for r in rs if r.get('tracks_subs', 0) != 0)}

    # --- E1 + E3 (같은 배치) ---
    c1, c3, o = stage(R, 'r3c/c1'), stage(R, 'r3c/c3'), stage(R, 'r3c/o')
    rows = [('CC-DWA', 'dwa_R3', 'CC-DWA', 'run')] + [(v[0], v[1], v[2], v[4]) for v in vlist]
    T2 = {}
    for vid, arm, row, st in rows:
        src = 'dwa_R3' if st != 'run' else arm
        s = arm_c1_summary(c1, src, R, 'r3c/c1', goals)
        if s is None:
            continue
        s.pop('t09_by_scene', None)
        T2[vid] = {'arm': arm, 'row': row, 'status': st, 'c1': s,
                   'delta_c1': delta_time(c1, src, 'dwa_R3') if src != 'dwa_R3' else None,
                   'delta_t09_med': (s['t09_med'] - T2['CC-DWA']['c1']['t09_med'])
                   if vid != 'CC-DWA' and s['t09_med'] is not None else None,
                   'slow_c3': slow_count(c3, c1, src), 'o': o_summary(o, src),
                   'delta_c3': delta_time(c3, src, 'dwa_R3') if src != 'dwa_R3' else None}
    V['E1E3'] = T2
    # E3 재현: R3 dwa_R3 C1 대비
    both = [dict(r, arm='x_e3') for r in c1 if r['arm'] == 'dwa_R3'] + \
        [dict(r, arm='x_r3') for r in r3['c1'] if r['arm'] == 'dwa_R3']
    d = ax.paired(both, 'x_e3', 'x_r3', 't_reach', ax.logratio)
    V['E3_repro'] = {'n': len(d), 'mean_abs_pct': float(np.mean([abs(pct(x)) for x in d.values()])) if d else None,
                     'mean_pct': pct(float(np.mean(list(d.values())))) if d else None,
                     'success_e3': sum(r['success'] for r in c1 if r['arm'] == 'dwa_R3'),
                     'success_r3': sum(r['success'] for r in r3['c1'] if r['arm'] == 'dwa_R3')}
    V['E3_repro']['pass'] = bool(d) and V['E3_repro']['mean_abs_pct'] <= 0.5 and \
        V['E3_repro']['success_e3'] == V['E3_repro']['success_r3']

    # --- E2 ---
    e2 = stage(R, 'r3c/e2')
    s2 = arm_c1_summary(e2, 'dwa_x_X00', R, 'r3c/e2', goals)
    if s2:
        s2.pop('t09_by_scene', None)
        s2['t09_minus_E3_dwa_R3'] = (s2['t09_med'] - T2['CC-DWA']['c1']['t09_med']) if 'CC-DWA' in T2 else None
    V['E2'] = s2

    # --- E1 chain ---
    root = os.path.join(R, 'r3c', 'chain')
    chc = chain_load(root)
    V['E1_chain'] = {}
    for vid, arm, row, st in rows:
        src = 'dwa_R3' if st != 'run' else arm
        V['E1_chain'][vid] = chain_arm(chc, root, src)
        V['E1_chain'][vid]['delta_c1'] = delta_time([r for r in chc if r['nseed'] == 0 and r['stateful'] == 0],
                                                    src, 'dwa_R3') if src != 'dwa_R3' else None
    # chain dwa_R3 이 R3 chain 과 같은가 (결정적)
    V['E1_chain_vs_R3_chain'] = _chain_same(chc, ch3, 'dwa_R3')

    # --- E4 ---
    eq = stage(R, 'r3c/e4eq')
    if eq:
        V['E4_equiv'] = e4eq_report(eq, arms5)
    eq2 = stage(R, 'r3c/e4eq2')
    if eq2:
        V['E4_equiv2'] = e4eq2_report(eq2)
    e4 = stage(R, 'r3c/e4')
    if e4:
        V['E4'] = e4_summary(e4_per_run(R, 'r3c/e4', e4), arms5)
        V['E4']['success'] = {a[3:]: sum(r['success'] for r in e4 if r['arm'] == a) for a in {r['arm'] for r in e4}}
        lf = os.path.join(R, 'r3c', 'e4', 'load.log')
        if os.path.exists(lf):
            ls = [float(line.split()[2]) for line in open(lf, encoding='utf-8') if len(line.split()) >= 5]
            V['E4']['load1'] = {'min': min(ls), 'max': max(ls), 'n': len(ls)} if ls else None

    # --- 계산 시간 (E4 취소 — 사용자 마무리 지시. R3 C1 원자료 cpu_ms_per_cycle 로 대체, 편차 D1) ---
    comp = {}
    pm = {}
    for a in arms5:
        pp = ax.per_path(r3['c1'], a, 'cpu_ms_per_cycle', False)
        if pp:
            pm[a] = pp
            comp[a] = {'paths': len(pp), 'mean_ms': float(np.mean(list(pp.values()))),
                       'median_ms': float(np.median(list(pp.values())))}
    V['compute'] = {'source': 'R3 r3/c1 cpu_ms_per_cycle (controller_server 누적 CPU 증가 − 정지 10 s 유휴 비율 × 시간, '
                              '÷ 명령 수; ros_bench_x.py)', 'arms': comp}
    for a in arms5[1:]:
        if 'dwa_R3' in pm and a in pm:
            V['compute'][f'ratio_dwa_R3_over_{a}'] = ratio_boot(pm['dwa_R3'], pm[a])
    if eq:   # 후보 수·굴림 스텝: e4eq 의 dwa_R3·tm_dwa_R3 (ros_bench_r3c, 튜닝-20 매끈 4 × 반복 3)
        xs = [r for r in eq if r['arm'] in ('dwa_R3', 'tm_dwa_R3') and r.get('n_samples_med', -1) >= 0]
        V['compute']['dwa_R3_candidates'] = {'runs': len(xs), 'n_samples_med': med([r['n_samples_med'] for r in xs]),
                                             'rollout_steps_med': med([r['rollout_steps_med'] for r in xs]),
                                             'source': 'r3c/e4eq dwa/stats[1], ⌈T_sim(v)/0.1⌉'}
        V['E4_equiv_timing_aux'] = e4_summary(e4_per_run(R, 'r3c/e4eq', eq), arms5)

    # --- E5 ---
    E5 = {}
    pj = os.path.join(R, 'r3c', 'e5', 'p13', 'probe_chain_r3c.json')
    if os.path.exists(pj):
        with open(pj, encoding='utf-8') as f:
            E5['P1_P3_chain'] = json.load(f)
    p4 = os.path.join(R, 'r3c', 'e5', 'p4.txt')
    if os.path.exists(p4):
        E5['P4'] = [line.strip() for line in open(p4, encoding='utf-8')]
    if 'CC-DWA' in T2:
        E5['P1_ros_start_gap_dwa_R3'] = T2['CC-DWA']['c1']['start_gap_max']
        E5['P1_ros_hard_accel_resets'] = {
            'c1': T2['CC-DWA']['c1'].get('hard_accel_resets'),
            'c3': _hard_accel(R, 'r3c/c3', c3, 'dwa_R3'), 'o': _hard_accel(R, 'r3c/o', o, 'dwa_R3')}
    if 'V-meas' in T2:
        E5['P2'] = {'formula_ratio': 0.27, 'a': 1.0,
                    'ros_slope_med': _slope_med(R, 'r3c/c1', c1, 'dwa_v_meas'),
                    'chain_slope_med': V['E1_chain'].get('V-meas', {}).get('slope36_med'),
                    'ros_slope_med_dwa_R3': _slope_med(R, 'r3c/c1', c1, 'dwa_R3'),
                    'chain_slope_med_dwa_R3': V['E1_chain'].get('CC-DWA', {}).get('slope36_med')}
    V['E5'] = E5

    # --- E6 ---
    vmap = {v[0]: (v[1], v[4]) for v in vlist}

    def src_of(vid):
        arm, st = vmap[vid]
        return ('r3c/c1', 'dwa_R3' if st != 'run' else arm)
    try:
        V['E6'] = fig2(R, rpp, {'hold': ('r3c/c1', 'dwa_R3'), 'thr': src_of('V-thr'), 'mdl': src_of('V-mdl-n'),
                                'mid': src_of('V-mdl-i'), 'rpp': ('r3/c1', rpp)},
                       os.path.join(args.out, 'fig2_startup_D.dat'))
    except (KeyError, IndexError, ValueError) as e:
        V['E6'] = {'error': repr(e)}

    with open(os.path.join(args.out, 'values_r3c.json'), 'w', encoding='utf-8') as f:
        json.dump(V, f, indent=1, ensure_ascii=False, default=float)
    with open(os.path.join(args.out, 'tables_r3c.md'), 'w', encoding='utf-8') as f:
        f.write(render(V))
    print(json.dumps({k: (v if k in ('selection', 'E3_repro') else '…') for k, v in V.items()}, ensure_ascii=False,
                     indent=1, default=float))
    return 0


def _paired_ci(recs, a, b, key, scale=1.0):
    d = ax.paired(recs, a, b, key)
    m, lo, hi = ax.boot(d.values())
    return {'n': len(d), 'mean': scale * m if d else None, 'ci': [scale * lo, scale * hi] if d else None}


def _cond_diff(a_recs, b_recs, arm):
    both = [dict(r, arm='x_a') for r in a_recs if r['arm'] == arm] + \
        [dict(r, arm='x_b') for r in b_recs if r['arm'] == arm]
    return delta_time(both, 'x_a', 'x_b')


def _rank(recs, arms):
    v = {}
    for a in arms:
        tr = ax.per_path(recs, a, lambda r: r['t_reach'] / ax.t_lb(r['length']), True)
        if tr:
            v[a] = float(np.median(list(tr.values())))
    return sorted(v, key=lambda a: v[a]), v


def _slow_window(R, c3, c1, arm, ref, window=3.0):
    """느린 실행 중 첫 진입 전 3 s 안 재설정(전체·감속) 이 있었던 수 (analyze_r3 표 R3-7 꼴)."""
    clean = {(r['arm'], r['scene']): r for r in c1}
    slow = any_w = dec_w = 0
    for r in c3:
        if r['arm'] != arm:
            continue
        c = clean.get((ref, r['scene']))
        if c is None or not c['success']:
            continue
        if not ((not r['success']) or r['t_reach'] / c['t_reach'] > 1.30):
            continue
        slow += 1
        rs = ax.read_plugin_resets(R, os.path.join('r3', 'c3'), r) or []
        tf = r['t_first_entry'] if r.get('t_first_entry', -1) >= 0 else r['t_end']
        win = [x for x in rs if tf - window <= x['t_rel'] <= tf]
        any_w += bool(win)
        dec_w += any(x.get('phase') == 'decel' for x in win)
    return {'slow': slow, 'with_reset_in_window': any_w, 'with_decel_reset_in_window': dec_w}


def _hard_accel(R, rel, recs, arm):
    n = runs = 0
    for r in recs:
        if r['arm'] != arm:
            continue
        rs = ax.read_plugin_resets(R, rel, r)
        if rs is None:
            continue
        runs += 1
        n += sum(1 for x in rs if x.get('reason') == 'hard' and x.get('phase') == 'accel')
    return {'runs': runs, 'hard_accel': n}


def _slope_med(R, rel, recs, arm):
    v = [slope36(read_trace_ros(R, rel, r)) for r in recs if r['arm'] == arm and r['family'] in FAMS3]
    return med(v)


def _testbed_gap(chain, c1, arm):
    """chain (잡음 없음, stateful 0) 대 ROS C1 의 경로별 짝 차: 시간 % (ROS/chain), CTE 평균 차 [cm]."""
    ch = [dict(r, arm='x_chain') for r in chain if r['arm'] == arm and r['nseed'] == 0 and r['stateful'] == 0]
    ro = [dict(r, arm='x_ros') for r in c1 if r['arm'] == arm]
    both = ch + ro
    dt = ax.paired(both, 'x_ros', 'x_chain', 't_reach', ax.logratio)
    dc = ax.paired(both, 'x_ros', 'x_chain', 'cte_mean')
    m, lo, hi = ax.boot(dt.values())
    mc, loc, hic = ax.boot(dc.values())
    return {'n': len(dt), 'time_pct': [pct(m), pct(lo), pct(hi)] if dt else None,
            'time_abs_pct_max': max(abs(pct(x)) for x in dt.values()) if dt else None,
            'cte_cm': [100 * mc, 100 * loc, 100 * hic] if dc else None,
            'cte_abs_cm_max': 100 * max(abs(x) for x in dc.values()) if dc else None}


def _chain_same(a, b, arm):
    ka = {(r['scene'], r['nseed'], r['stateful']): r for r in a if r['arm'] == arm}
    kb = {(r['scene'], r['nseed'], r['stateful']): r for r in b if r['arm'] == arm}
    common = set(ka) & set(kb)
    keys = ('success', 't_end', 'cte_mean', 'cte_max', 'stop_along', 'stop_lat', 'n_resets')
    same = sum(1 for k in common if all(ka[k].get(x) == kb[k].get(x) for x in keys))
    return {'common': len(common), 'identical_metrics': same}


# ---------------------------------------------------------------------------------------------
# 표 (tables_r3c.md)

def _f(x, nd=2, sign=False):
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return '–'
    return f'{x:+.{nd}f}' if sign else f'{x:.{nd}f}'


def _d(d, nd=1):
    if not d or not d.get('pct'):
        return '–'
    return f"{d['pct'][0]:+.{nd}f} [{d['pct'][1]:+.{nd}f}, {d['pct'][2]:+.{nd}f}] (n {d['n']})"


def render(V):
    R3 = V['R3']
    sel = V['selection']
    arms5 = ['dwa_R3', 'dwa_textbook_tuned', sel['DWB'], sel['MPPI'], sel['RPP']]
    e4a = V.get('compute', {}).get('arms', {})
    L = ['# R3-C 표 (analyze_r3c.py 생성 — 기술 통계)', '',
         f"선택: CC-DWA = dwa_R3 = {sel['chosen_z']}, DWB {sel['DWB']}, MPPI {sel['MPPI']}, RPP {sel['RPP']}.", '']
    L += ['## 표 1 재구성 — R3 원자료 (held-out D), Δ 는 제어기 − CC-DWA(dwa_R3) 짝 차', '',
          '| 제어기 | 성공 C1 (매끈/통로/A*) | T/T_ref 중앙 | Δ시간 C1 % [95 %] | 곡선 CTE 중앙 [cm] | |정지 오차| 중앙 [cm] | '
          'C3 Δ시간 % [95 %] | C2 Δ시간 % | O 성공 /60 (접촉) | O 최소 이격 최솟값 [cm] | CPU [ms/주기] (R3) |',
          '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for a in arms5:
        s = R3['table1'].get(a)
        o = R3['o'].get(a)
        if not s:
            continue
        sb = s['success_by_family']
        L.append(f"| {a} | {s['success']} ({sb['smooth'][0]}/{sb['narrow'][0]}/{sb['astar'][0]}) | "
                 f"{_f(s['t_tref_med'], 3)} | {_d(R3['c1_delta'].get(a))} | {_f(s['cte_curve_cm_med'])} | "
                 f"{_f(s['stop_err_cm_med'], 1)} | {_d(R3['c3_delta'].get(a))} | {_d(R3['c2_delta'].get(a))} | "
                 f"{o['success']} ({o['contact']}) | {_f(o['min_clear_cm_min'], 1)} | "
                 f"{_f(e4a.get(a, {}).get('mean_ms'))} |")
    L += ['', f"순위 (T/T_ref 경로 중앙, 빠른 순): C1 {R3['rank_c1'][0]}, C2 {R3['rank_c2'][0]}", '']
    L += ['## 표 2 — E1 변형 + E3 CC-DWA (같은 배치, held-out D)', '',
          '| 변형 | 팔 | 상태 | Δ시간 C1 % [95 %] | t0.9 중앙 [s] (n) | Δt0.9 [s] | 출발 2 s 가속 재설정 /실행 | '
          '재설정 전체 /실행 | C3 느린 /60 (실패) | C3 Δ시간 % | O 성공 /60 (접촉) | 진입 yaw > 0.05 rad | 성공 C1 |',
          '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for vid, r in V['E1E3'].items():
        c = r['c1']
        L.append(f"| {vid} | {r['arm']} | {r['status']} | {_d(r['delta_c1'])} | {_f(c['t09_med'])} ({c['t09_n']}) | "
                 f"{_f(r['delta_t09_med'], 2, True)} | {_f(c.get('accel2_per_run'))} | {_f(c.get('resets_per_run'))} | "
                 f"{r['slow_c3']['slow']} ({r['slow_c3']['fail']}) | {_d(r['delta_c3'])} | "
                 f"{r['o']['success']} ({r['o']['contact']}) | {c['entry_yaw_over_005']}/{c['entry_yaw_n']} | "
                 f"{c['success']}/{c['runs']} |")
    e3 = V['E3_repro']
    L += ['', f"E3 재현 (E3 dwa_R3 C1 대 R3 dwa_R3 C1): 경로별 |시간 차| 평균 {_f(e3['mean_abs_pct'], 3)} % "
          f"(평균 {_f(e3['mean_pct'], 3, True)} %, n {e3['n']}), 성공 {e3['success_e3']} 대 {e3['success_r3']} → "
          f"{'통과' if e3['pass'] else '기준 밖'} (기준 ≤ 0.5 %, 성공 같음)", '']
    L += ['## E1 chain (잡음 없음, stateful false)', '',
          '| 변형 | 성공 | t0.9 중앙 [s] | v_m 0.3→0.6 기울기 중앙 [m/s²] | 가속 재설정 /실행 | Δ시간 % (짝, dwa_R3 기준) |',
          '| --- | --- | --- | --- | --- | --- |']
    for vid, r in V['E1_chain'].items():
        L.append(f"| {vid} | {r['success']}/{r['runs']} | {_f(r['t09_med'])} | {_f(r['slope36_med'])} | "
                 f"{_f(r['accel_resets_per_run'])} | {_d(r.get('delta_c1'))} |")
    L += ['', f"E1 chain dwa_R3 대 R3 chain dwa_R3 (결정적): {V['E1_chain_vs_R3_chain']}", '']
    e2 = V.get('E2') or {}
    L += ['## E2 — X00 (배포 규칙 동치, K 0) × held-out D C1', '',
          f"성공 {e2.get('success')}/{e2.get('runs')}, t0.9 중앙 {_f(e2.get('t09_med'))} s "
          f"(E3 dwa_R3 대비 {_f(e2.get('t09_minus_E3_dwa_R3'), 2, True)} s), 출발 2 s 가속 재설정 "
          f"{_f(e2.get('accel2_per_run'))}/실행, 재설정 전체 {_f(e2.get('resets_per_run'))}/실행, "
          f"T/T_ref 중앙 {_f(e2.get('t_tref_med'), 3)}", '']
    L += ['## R3 원자료에서 내는 2.2 절 수치', '',
          f"- ② R3 C3 X00 느린 실행 {R3['x00_c3_decel_window']['slow']} 건 중 첫 진입 전 3 s 안 재설정 "
          f"{R3['x00_c3_decel_window']['with_reset_in_window']} 건, 감속 재설정 "
          f"{R3['x00_c3_decel_window']['with_decel_reset_in_window']} 건",
          f"- ③ dwa_R2 (Y11) 가속 재설정 /실행: chain {_f(R3['chain']['dwa_R2']['accel_resets_per_run'])}, "
          f"ROS C1 출발 2 s {_f(R3['dwa_R2_c1_resets'].get('accel2_per_run'))} "
          f"(전체 {_f(R3['dwa_R2_c1_resets'].get('resets_per_run'))})",
          f"- 시험대 간 차 (dwa_R3, ROS C1 − chain): {R3['testbed_gap']}", '']
    if 'E4_equiv' in V:
        L += ['## E4 동치', '', f"e4eq 통과 여부: { {a: v['pass'] for a, v in V['E4_equiv']['arms'].items()} }"]
    if 'E4_equiv2' in V:
        e2p = {sc: (round(d['time_diff_pct'], 3), round(d['cte_diff_cm'], 3))
               for sc, d in V['E4_equiv2']['paths'].items()}
        L += [f"e4eq2 (MPPI, 순서 엇갈림): 통과 {V['E4_equiv2']['pass']}, 경로별 (시간 %, CTE cm) {e2p}"]
    cp = V.get('compute', {})
    L += ['', '## 계산 시간 — R3 C1 cpu_ms_per_cycle (E4 취소, 편차 D1)', '',
          '| 팔 | 경로 | 경로 평균의 평균 [ms] | 경로 중앙 [ms] |', '| --- | --- | --- | --- |']
    for a in arms5:
        x = cp.get('arms', {}).get(a)
        if x:
            L.append(f"| {a} | {x['paths']} | {_f(x['mean_ms'], 3)} | {_f(x['median_ms'], 3)} |")
    for k, v in cp.items():
        if k.startswith('ratio_') and v:
            L.append(f"- {k}: {v['ratio']:.3f} [{v['ci'][0]:.3f}, {v['ci'][1]:.3f}] (n {v['n']})")
    if cp.get('dwa_R3_candidates'):
        L.append(f"- CC-DWA 후보·굴림: {cp['dwa_R3_candidates']}")
    if 'E4' in V:
        L += ['', '## E4 계산 시간 (held-out D C1 60, 레인 2 개 cpuset, 단독)', '',
              '| 팔 | 실행 | 경로 | 주기 | 평균 [ms] | p50 중앙 [ms] | p99 중앙 [ms] | 최대 [ms] | 후보 수 중앙 | 굴림 스텝 중앙 | 명령 주기 최소 |',
              '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
        for a in arms5:
            x = e4a.get(a)
            if x:
                L.append(f"| {a} | {x['runs']} | {x['paths']} | {x['cycles']} | {_f(x['mean_ms'], 3)} | "
                         f"{_f(x['p50_ms_med'], 3)} | {_f(x['p99_ms_med'], 3)} | {_f(x['max_ms'], 2)} | "
                         f"{_f(x['n_samples_med'], 0)} | {_f(x['rollout_steps_med'], 0)} | {_f(x['cmd_rate_min'])} |")
        for k, v in V['E4'].items():
            if k.startswith('ratio_') and v:
                L.append(f"- {k}: {v['ratio']:.3f} [{v['ci'][0]:.3f}, {v['ci'][1]:.3f}] (n {v['n']})")
        L.append(f"- 성공: {V['E4'].get('success')}, 1 분 부하: {V['E4'].get('load1')}")
    L += ['', '## E5', '', '```json', json.dumps(V['E5'], indent=1, ensure_ascii=False, default=float), '```', '']
    e6 = json.dumps(V.get('E6'), indent=1, ensure_ascii=False, default=float)
    L += ['## E6 그림 2 (held-out D)', '', '```json', e6, '```', '']
    L += ['## 실시간 품질', '', '| 단계 | 실행 | 명령 주기 최소 [Hz] | 18 Hz 미만 | 참값 트랙 구독 ≠ 0 |', '| --- | --- | --- | --- | --- |']
    for k, v in V['rates'].items():
        L.append(f"| {k} | {v['runs']} | {v['cmd_rate_min']:.2f} | {v['below18']} | {v['tracks_subs_nonzero']} |")
    return '\n'.join(L) + '\n'


if __name__ == '__main__':
    sys.exit(main())
