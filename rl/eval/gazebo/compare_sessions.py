#!/usr/bin/env python3
"""
부하 조건이 다른 세션을 나란히 — 부하에 민감한 [측정] (RTF, 지연, 주기, 플랜트, 벽시계) 과 단계 C 소요 추정.

    python3 rl/eval/gazebo/compare_sessions.py 공유:dwa=<dir> 공유:rl=<dir> 독점:dwa=<dir> 독점:rl=<dir> \
        [--csv gzbench.csv] [--out <json>]

인자 = <조건>:<팔>=<세션 디렉터리>[@<CSV 이름>]. 세션 디렉터리 = run_bench.sh 출력
(gzbench*.csv, probe_step_analysis.json, started.txt·ended.txt (호스트 run_session.sh), clock_hz.txt).
COORDINATION §8.1 (측정 시간대 독점, 22:59): 공유 부하 값은 지우지 않고 독점 값과 나란히 적는다 —
이 도구는 그 표를 만든다 (A5-5).

단계 C 소요 (팔당 주 100 + 스트레스 100, 50 에피소드 묶음 4 세션):
  에피소드 = 100·평균(주) + 100·평균(스트레스), ±2·100·√(sd²_주/n_주 + sd²_스/n_스) (평균 추정 불확실성 — 스모크 n 이 작다),
  세션 고정비 = 세션 전체 벽시계 − 에피소드 벽시계 합 (기동·수명주기·탐침·종료) × 4.
"""

import argparse
import csv
import datetime as dt
import json
import math
import os

import numpy as np


def num(x):
    """CSV 칸 → float (빈 칸 = nan)."""
    try:
        return float(x)
    except (TypeError, ValueError):
        return math.nan


def med(xs):
    """유한한 값의 중앙 (없으면 None)."""
    x = np.asarray([v for v in xs if math.isfinite(v)], dtype=float)
    return round(float(np.median(x)), 4) if x.size else None


def qs(xs, ps=(5, 50, 95)):
    """백분위 (유한한 값만)."""
    x = np.asarray([v for v in xs if math.isfinite(v)], dtype=float)
    return [round(float(np.percentile(x, p)), 4) for p in ps] if x.size else None


def read_iso(path):
    """started.txt·ended.txt (date -Iseconds) → datetime 또는 None."""
    try:
        return dt.datetime.fromisoformat(open(path).read().strip())
    except (OSError, ValueError):
        return None


def clock_rate(path):
    """ros2 topic hz /clock 의 마지막 'average rate' [Hz] 또는 None."""
    try:
        lines = [ln for ln in open(path) if 'average rate' in ln]
    except OSError:
        return None
    return round(float(lines[-1].split(':')[1]), 1) if lines else None


def session(path, csv_name):
    """세션 하나 → 부하 민감 지표 dict."""
    rows = list(csv.DictReader(open(os.path.join(path, csv_name), encoding='utf-8')))
    ok = [r for r in rows if r['setup_ok'] in ('1', 'True')]
    out = {'path': path, 'csv': csv_name, 'n': len(rows), 'n_ok': len(ok),
           'controller': rows[0]['controller'] if rows else None}
    out['rtf_q'] = qs([num(r['rtf']) for r in ok])
    out['clock_hz_end'] = clock_rate(os.path.join(path, 'clock_hz.txt'))
    for k in ('scan_to_cmd_p50_s', 'scan_to_cmd_p95_s', 'scan_age_p50_s', 'scan_age_p95_s',
              'chain_lag_p50_s', 'chain_lag_p95_s', 'ctrl_dt_sim_mean', 'ctrl_dt_wall_mean',
              'tick_dt_sim_mean', 'decision_period_mean_s', 'infer_ms_p95'):
        out[f'{k}_med'] = med([num(r.get(k)) for r in ok])
    dp = [num(r.get('decision_period_mean_s')) for r in ok]
    dp = [v for v in dp if math.isfinite(v)]
    out['decision_period_mean_range'] = [round(min(dp), 5), round(max(dp), 5)] if dp else None
    walls = {}
    for w in ('main', 'stress'):
        x = [num(r['wall_episode_s']) for r in rows if r['difficulty'] == w]
        x = [v for v in x if math.isfinite(v)]
        walls[w] = {'n': len(x), 'mean': round(float(np.mean(x)), 1) if x else None,
                    'sd': round(float(np.std(x, ddof=1)), 1) if len(x) > 1 else None}
    out['wall_episode'] = walls
    out['wall_setup_med'] = med([num(r['wall_setup_s']) for r in rows])
    out['wall_nav_med'] = med([num(r['wall_nav_s']) for r in rows])
    t_a = read_iso(os.path.join(path, 'started.txt'))
    t_b = read_iso(os.path.join(path, 'ended.txt'))
    ep_sum = float(np.nansum([num(r['wall_episode_s']) for r in rows]))
    if t_a and t_b:
        total = (t_b - t_a).total_seconds()
        out['session_wall_s'] = round(total, 1)
        out['session_fixed_s'] = round(total - ep_sum, 1)
    out['load_start_end'] = [rows[0].get('load_start'), rows[-1].get('load_end')] if rows else None
    ps = os.path.join(path, 'probe_step_analysis.json')
    if os.path.exists(ps):
        p = json.load(open(ps, encoding='utf-8'))
        out['plant'] = {k: {f: p.get(k, {}).get(f) for f in ('T_m_s', 'T_d_s', 'rmse')}
                        for k in ('plant_linear', 'plant_angular', 'observed_linear',
                                  'observed_angular') if k in p}
        out['chain_linear'] = p.get('chain_linear')
    return out


def stage_c(s, n_main=100, n_stress=100, n_sessions=4):
    """단계 C 팔 하나 소요 [h] (에피소드 + 세션 고정비)."""
    w = s['wall_episode']
    m, st = w.get('main', {}), w.get('stress', {})
    if not (m.get('mean') and st.get('mean')):
        return None
    ep = n_main * m['mean'] + n_stress * st['mean']
    var = (m['sd'] ** 2 / m['n'] if m.get('sd') else 0.0) + \
        (st['sd'] ** 2 / st['n'] if st.get('sd') else 0.0)
    unc = 2.0 * 100.0 * math.sqrt(var)
    fixed = (s.get('session_fixed_s') or 0.0) * n_sessions
    return {'episodes_h': round(ep / 3600, 2), 'pm2se_h': round(unc / 3600, 2),
            'fixed_h': round(fixed / 3600, 2), 'total_h': round((ep + fixed) / 3600, 2)}


def main():
    """명령행."""
    ap = argparse.ArgumentParser()
    ap.add_argument('sessions', nargs='+', help='<조건>:<팔>=<디렉터리>[@<CSV>]')
    ap.add_argument('--csv', default='gzbench.csv')
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    res = {}
    for spec in a.sessions:
        key, path = spec.split('=', 1)
        path, _, name = path.partition('@')
        s = session(path, name or a.csv)
        s['stage_c'] = stage_c(s)
        res[key] = s
    txt = json.dumps(res, ensure_ascii=False, indent=1)
    print(txt)
    if a.out:
        open(a.out, 'w', encoding='utf-8').write(txt)


if __name__ == '__main__':
    main()
