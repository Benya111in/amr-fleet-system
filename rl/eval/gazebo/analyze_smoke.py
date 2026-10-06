#!/usr/bin/env python3
"""
하네스 스모크·세션 요약 (gzbench.csv 들 → 표·판정·소요 추정) — A5-4 사전 등록 판정 H1–H4 와 전이 점검 3·4·6.

    python3 rl/eval/gazebo/analyze_smoke.py <세션 디렉터리>… [--out <json>] [--expect 20]

세션 디렉터리 = run_bench.sh 의 출력 (gzbench.csv, episodes_v1.csv, load.txt, lifecycle.txt, progress.log).
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))

CORE = ['outcome_contract', 'strict_gt', 'strict_est', 'nav2compat_gt', 'system_success',
        'final_dist_gt_m', 'final_dist_est_m', 'nav_time_s', 'path_length_m',
        'cte_active_mean_m', 'interventions', 'min_clear_static_m', 'loc_err_max_m',
        'loc_err_end_m', 'rtf', 'wall_episode_s', 'ctrl_dt_sim_mean', 'scan_to_cmd_p50_s']
CORE_RL = ['decisions', 'decision_period_mean_s', 'scan_age_p50_s']


def num(x):
    """CSV 칸 → float (빈 칸 = nan)."""
    try:
        return float(x)
    except (TypeError, ValueError):
        return math.nan


def q(x, ps=(5, 50, 95)):
    """백분위 (유한한 값만)."""
    x = np.asarray([v for v in x if math.isfinite(v)], dtype=float)
    return [round(float(np.percentile(x, p)), 4) for p in ps] if x.size else None


def rate(rows, key):
    """참 비율 'k/n'."""
    n = len(rows)
    k = sum(1 for r in rows if r.get(key) in ('1', 'True', 'true'))
    return f'{k}/{n}'


def session(path, expect, suffix=''):
    """세션 하나 요약 (suffix = recompute.py 가 쓴 CSV 를 읽을 때 '_recomputed')."""
    rows = list(csv.DictReader(open(os.path.join(path, f'gzbench{suffix}.csv'),
                                    encoding='utf-8')))
    ok = [r for r in rows if r['setup_ok'] in ('1', 'True')]
    err = [r for r in rows if r['setup_ok'] not in ('1', 'True')]
    ctrl = rows[0]['controller'] if rows else '?'
    out = {'path': path, 'controller': ctrl, 'n_rows': len(rows), 'n_error': len(err),
           'errors': [(r['exec_index'], r['scenario_id'], r['setup_error']) for r in err]}
    # H1
    out['H1_complete'] = len(rows) == expect and len(err) <= 2
    # H2
    need = CORE + (CORE_RL if ctrl != 'dwa' else [])
    missing = {}
    for r in ok:
        miss = [k for k in need if r.get(k, '') == '']
        if miss:
            missing[r['scenario_id']] = miss
    out['H2_missing'] = missing
    try:
        sys.path.insert(0, os.path.join(REPO, 'rl', 'eval'))
        from amr_rl_eval import schema
        schema.read_csv(os.path.join(path, f'episodes_v1{suffix}.csv'), strict=True)
        out['H2_schema'] = 'ok'
    except Exception as exc:  # noqa: BLE001 — 실패 사유를 그대로 남긴다
        out['H2_schema'] = f'fail: {exc}'[:300]
    out['H2_ok'] = not missing and out['H2_schema'] == 'ok'
    # H3
    succ = [r for r in ok if r['system_success'] in ('1', 'True')]
    good = [r for r in succ if num(r['final_dist_est_m']) <= 0.12]
    out['H3'] = f'{len(good)}/{len(succ)}'
    out['H3_ok'] = len(succ) == 0 or len(good) >= 0.9 * len(succ)
    # H4
    bad = [r['scenario_id'] for r in ok if not (num(r['loc_err_start_m']) <= 0.10) or
           r.get('lost_at_start') in ('1', 'True')]
    out['H4_bad'] = bad
    out['H4_ok'] = not bad
    # 기술 (집합별)
    out['by_set'] = {}
    for which in ('main', 'stress'):
        rs = [r for r in ok if r['difficulty'] == which]
        if not rs:
            continue
        oc = {}
        for r in rs:
            key = r['outcome_contract'] + ('*' if r.get('contract_note') else '')
            oc[key] = oc.get(key, 0) + 1
        osys = {}
        for r in rs:
            osys[r['outcome_system']] = osys.get(r['outcome_system'], 0) + 1
        out['by_set'][which] = {
            'n': len(rs), 'outcome_contract': oc, 'outcome_system': osys,
            'strict_gt': rate(rs, 'strict_gt'), 'strict_est': rate(rs, 'strict_est'),
            'nav2compat_gt': rate(rs, 'nav2compat_gt'),
            'system_success': rate(rs, 'system_success'),
            'final_ok_gt': rate(rs, 'final_ok_gt'),
            'final_dist_gt_q': q([num(r['final_dist_gt_m']) for r in rs]),
            'final_dist_est_q': q([num(r['final_dist_est_m']) for r in rs]),
            'nav_time_q': q([num(r['nav_time_s']) for r in rs]),
            't_strict_gt_q': q([num(r['t_strict_gt_s']) for r in rs]),
            'path_ratio_q': q([num(r['path_length_m']) / num(r['path_ref_m']) for r in rs]),
            'cte_active_mean_q': q([num(r['cte_active_mean_m']) for r in rs]),
            'max_dev_q': q([num(r['max_dev_m']) for r in rs]),
            'interventions_sum': int(np.nansum([num(r['interventions']) for r in rs])),
            'intervention_time_q': q([num(r['intervention_time_s']) for r in rs]),
            'zone_stop_s_sum': round(float(np.nansum([num(r['zone_stop_s']) for r in rs])), 2),
            'zone_critical_s_sum': round(float(np.nansum([num(r['zone_critical_s'])
                                                          for r in rs])), 2),
            'contacts_static': int(np.nansum([num(r['contacts_static']) for r in rs])),
            'contacts_dynamic': int(np.nansum([num(r['contacts_dynamic']) for r in rs])),
            'min_clear_static_q': q([num(r['min_clear_static_m']) for r in rs]),
            'near_goal_stop_s_q': q([num(r['near_goal_stop_s']) for r in rs]),
            'eps_near_goal_stop_ge2s': sum(1 for r in rs if num(r['near_goal_stop_max_s']) >= 2.0),
            'success_while_rotating': rate([r for r in rs if r['success_while_rotating'] != ''],
                                           'success_while_rotating'),
            'loc_collapse': rate(rs, 'loc_collapse'), 'lost_events': int(np.nansum(
                [num(r['lost_events']) for r in rs])),
        }
        if ctrl != 'dwa':
            out['by_set'][which].update({
                'decisions': int(np.nansum([num(r['decisions']) for r in rs])),
                'src_dwa': int(np.nansum([num(r['src_dwa']) for r in rs])),
                'src_align': int(np.nansum([num(r['src_align']) for r in rs])),
                'dwa_reasons': {k: int(np.nansum([num(r[f'dwa_{k}']) for r in rs]))
                                for k in ('state', 'policy_error', 'timeout', 'nonfinite',
                                          'handover')}})
    # 전이 점검 3·4·6 (에피소드 값의 분포)
    out['transfer'] = {
        'scan_to_cmd_p50_q': q([num(r['scan_to_cmd_p50_s']) for r in ok]),
        'scan_to_cmd_p95_q': q([num(r['scan_to_cmd_p95_s']) for r in ok]),
        'chain_lag_p50_q': q([num(r['chain_lag_p50_s']) for r in ok]),
        'chain_lag_p95_q': q([num(r['chain_lag_p95_s']) for r in ok]),
        'scan_age_p50_q': q([num(r['scan_age_p50_s']) for r in ok]),
        'scan_age_p95_q': q([num(r['scan_age_p95_s']) for r in ok]),
        'ctrl_dt_sim_mean_q': q([num(r['ctrl_dt_sim_mean']) for r in ok]),
        'ctrl_dt_wall_mean_q': q([num(r['ctrl_dt_wall_mean']) for r in ok]),
        'decision_period_mean_q': q([num(r['decision_period_mean_s']) for r in ok]),
        'tick_dt_sim_mean_q': q([num(r['tick_dt_sim_mean']) for r in ok]),
        'loc_err_end_q': q([num(r['loc_err_end_m']) for r in ok]),
        'loc_err_end_yaw_q': q([num(r['loc_err_end_yaw']) for r in ok]),
        'loc_err_max_q': q([num(r['loc_err_max_m']) for r in ok]),
        'strict_flip_gt_vs_est': sum(1 for r in ok if r['strict_gt'] != r['strict_est']),
        'final_ok_flip_gt_vs_est': sum(1 for r in ok if r['final_ok_gt'] != r['final_ok_est']),
        'v_max_gt': round(float(np.nanmax([num(r['v_max_gt']) for r in ok])), 3) if ok else None,
        'w_max_gt': round(float(np.nanmax([num(r['w_max_gt']) for r in ok])), 3) if ok else None,
        'infer_ms_p95_q': q([num(r['infer_ms_p95']) for r in ok]),
    }
    # 시간
    wall = [num(r['wall_episode_s']) for r in rows]
    out['timing'] = {
        'rtf_q': q([num(r['rtf']) for r in ok]),
        'wall_episode_mean_s': round(float(np.nanmean(wall)), 1) if wall else None,
        'wall_episode_q': q(wall), 'wall_setup_q': q([num(r['wall_setup_s']) for r in rows]),
        'wall_nav_q': q([num(r['wall_nav_s']) for r in rows]),
        'wall_by_set_mean_s': {w: round(float(np.nanmean(
            [num(r['wall_episode_s']) for r in rows if r['difficulty'] == w])), 1)
            for w in ('main', 'stress') if any(r['difficulty'] == w for r in rows)},
        'load_start_end': [rows[0]['load_start'], rows[-1]['load_end']] if rows else None,
    }
    lc = os.path.join(path, 'lifecycle.txt')
    if os.path.exists(lc):
        last = open(lc).read().strip().splitlines()[-1]
        out['timing']['lifecycle_last'] = last
    for name in ('started.txt', 'ended.txt'):
        f = os.path.join(path, name)
        if os.path.exists(f):
            out['timing'][name.split('.')[0]] = open(f).read().strip()
    return out


def main():
    """명령행."""
    ap = argparse.ArgumentParser()
    ap.add_argument('sessions', nargs='+')
    ap.add_argument('--out', default='')
    ap.add_argument('--expect', type=int, default=20)
    ap.add_argument('--suffix', default='')
    a = ap.parse_args()
    res = {os.path.basename(os.path.normpath(p)): session(p, a.expect, a.suffix)
           for p in a.sessions}
    txt = json.dumps(res, ensure_ascii=False, indent=1)
    print(txt)
    if a.out:
        open(a.out, 'w', encoding='utf-8').write(txt)


if __name__ == '__main__':
    main()
