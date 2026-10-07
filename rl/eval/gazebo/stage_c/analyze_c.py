#!/usr/bin/env python3
"""
단계 C 집계 — 원자료(c2/batches/*/a*/gzbench.csv, c2/sampler.tsv)에서 사전 등록한 표를 다시 계산한다 (numpy 만).

    analyze_c.py --c2 /home/CAPYI_AI/runs/bonus1/b1-gzc/c2 \
        --scenarios rl/eval/gazebo/scenarios_v1.json \
        [--isaac-eval <b1-ppo10-eval/eval/B1e_ppo10_s<S>_final_sets/eval.json>] \
        [--suffix ''] --out <json>

사전 등록: 노트 `entries/2026-10-07_1055_b1-gzc_gazebo-rl-vs-dwa.md` 「지표와 통계」. 요약:
- 팔 = 묶음 이름 접두사 (dwa_b*, rl_b*, rlho_b*). 같은 exec_index 가 여러 시도에 있으면 첫 시도의 행.
  `<묶음>_rerun` 이 있으면 그 묶음을 대신한다.
- 제외 분류 E1 하네스 오류 · E2 시작 위치추정 미확립 · E3 시뮬레이터 사고(watchdog, RTF < 0.5) · E4 주행 중 위치추정 붕괴.
  유효 짝 = 두 팔 모두 E1–E4 아님. 전체 짝 = 모든 시나리오, E1–E3 은 그 팔의 실패, E4 는 지면 진실 결과 그대로.
- 주 결론: 주 집합 유효 짝 strict_gt, 짝 차이 + 짝 부트스트랩 95 % (B 10 000, seed 20261007) + McNemar 정확 양측.
- 부결론: 구간만 (p 값 없음). 스트레스 집합은 목표 군집(1.0 m 단일 연결) 부트스트랩도.
--suffix 는 recompute.py 로 다시 계산한 CSV(gzbench<suffix>.csv)를 읽을 때.
"""

import argparse
import csv
import datetime as dt
import glob
import json
import math
import os
import sys

import numpy as np

ARMS = ('dwa', 'rl', 'rlho')
SETS = ('main', 'stress')
B = 10000
SEED = 20261007
CLUSTER_M = 1.0
COMPARE = (('rl', 'dwa'), ('rlho', 'dwa'), ('rlho', 'rl'))
RATE_COLS = ('strict_gt', 'strict_est', 'nav2compat_gt', 'system_success')


def num(x):
    """CSV 칸 → float (빈 칸·None → nan)."""
    try:
        return float(x)
    except (TypeError, ValueError):
        return float('nan')


def truthy(x):
    """CSV 참 값."""
    return str(x) in ('True', '1', 'true')


def attempts(bd):
    """시도 디렉터리 (번호 순)."""
    return sorted(glob.glob(os.path.join(bd, 'a*')), key=lambda p: int(os.path.basename(p)[1:]))


def load_arm(c2, arm, suffix=''):
    """팔 하나의 exec_index → 행 (첫 시도 우선, _rerun 묶음이 있으면 대신)."""
    rows, src = {}, {}
    for bd in sorted(glob.glob(os.path.join(c2, 'batches', f'{arm}_b*'))):
        name = os.path.basename(bd)
        if name.endswith('_rerun') or not name[len(arm) + 2:].isdigit():
            continue
        use = bd + '_rerun' if os.path.isdir(bd + '_rerun') else bd
        for a in attempts(use):
            f = os.path.join(a, f'gzbench{suffix}.csv')
            if not os.path.exists(f):
                continue
            with open(f, encoding='utf-8', newline='') as fh:
                for r in csv.DictReader(fh):
                    i = int(r['exec_index'])
                    if i not in rows:
                        r['_src'] = a
                        rows[i] = r
                        src[i] = os.path.basename(use)
    return rows


def excl(r):
    """제외 분류 (없으면 None) — 앞의 것이 우선."""
    if not truthy(r.get('setup_ok')) or r.get('outcome_contract') == 'error':
        return 'E1'
    if num(r.get('loc_err_start_m')) > 0.10 or truthy(r.get('lost_at_start')):
        return 'E2'
    if r.get('cancel_reason') == 'watchdog' or num(r.get('rtf')) < 0.5:
        return 'E3'
    if truthy(r.get('loc_collapse')) or num(r.get('lost_events')) > 0:
        return 'E4'
    return None


def wilson(x, n, z=1.959963984540054):
    """Wilson 95 % 구간."""
    if n == 0:
        return [None, None]
    p = x / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return [max(0.0, c - h), min(1.0, c + h)]


def mcnemar_exact(b, c):
    """맥니마(McNemar) 정확 검정의 양측 p."""
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    p = 2.0 * sum(math.comb(n, i) for i in range(k + 1)) / 2.0 ** n
    return min(1.0, p)


def boot_ci(d, stat='mean', clusters=None, b=B, seed=SEED):
    """짝 차이 d 의 점추정과 부트스트랩 95 % (시나리오 또는 군집 복원 추출)."""
    d = np.asarray(d, dtype=float)
    if d.size == 0:
        return {'n': 0, 'est': None, 'lo': None, 'hi': None}
    f = np.mean if stat == 'mean' else np.median
    rng = np.random.default_rng(seed)
    if clusters is None:
        idx = rng.integers(0, d.size, size=(b, d.size))
        bs = f(d[idx], axis=1)
    else:
        cl = np.asarray(clusters)
        uniq = np.unique(cl)
        members = [np.flatnonzero(cl == u) for u in uniq]
        bs = np.empty(b)
        for k in range(b):
            pick = rng.integers(0, len(uniq), size=len(uniq))
            bs[k] = f(d[np.concatenate([members[j] for j in pick])])
    return {'n': int(d.size), 'est': float(f(d)), 'lo': float(np.percentile(bs, 2.5)),
            'hi': float(np.percentile(bs, 97.5))}


def goal_clusters(scen_ids, scen):
    """목표 xy 단일 연결(≤ CLUSTER_M) 군집 번호."""
    xy = np.array([scen[i]['goal'][:2] for i in scen_ids], dtype=float)
    parent = list(range(len(scen_ids)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for i in range(len(xy)):
        dd = np.hypot(*(xy[i + 1:] - xy[i]).T) if i + 1 < len(xy) else []
        for j in np.flatnonzero(np.asarray(dd) <= CLUSTER_M):
            ra, rb = find(i), find(i + 1 + int(j))
            if ra != rb:
                parent[rb] = ra
    return [find(i) for i in range(len(xy))]


def success(r, col, mode):
    """에피소드 성공 지시 (전체: E1–E3 → 실패)."""
    e = excl(r)
    if mode == 'all' and e in ('E1', 'E2', 'E3'):
        return 0
    return 1 if truthy(r.get(col)) else 0


def q(x, ps=(5, 25, 50, 75, 95)):
    """백분위 (빈 → None)."""
    x = np.asarray([v for v in x if v == v], dtype=float)
    return [None] * len(ps) if x.size == 0 else [float(np.percentile(x, p)) for p in ps]


def arm_summary(rows):
    """팔·집합 하나의 기술 요약."""
    n = len(rows)
    ex = {k: sum(1 for r in rows if excl(r) == k) for k in ('E1', 'E2', 'E3', 'E4')}
    valid = [r for r in rows if excl(r) is None]
    ok = [r for r in rows if excl(r) not in ('E1',)]
    out = {'n': n, 'excl': ex, 'n_valid': len(valid)}
    for col in RATE_COLS:
        for mode, rs in (('valid', valid), ('all', rows)):
            x = sum(success(r, col, mode) for r in rs)
            out[f'{col}_{mode}'] = {'x': x, 'n': len(rs), 'rate': x / len(rs) if rs else None,
                                    'wilson': wilson(x, len(rs))}
    fd = [num(r['final_dist_gt_m']) for r in ok]
    out['final_dist_gt_q'] = q(fd)
    out['final_dist_gt_gt010'] = sum(1 for v in fd if v > 0.10)
    out['final_dist_gt_gt025'] = sum(1 for v in fd if v > 0.25)
    oc = {}
    for r in rows:
        k = f"{r.get('outcome_contract')}|{r.get('outcome_system')}"
        oc[k] = oc.get(k, 0) + 1
    out['outcome_cross'] = oc
    cont = [r for r in ok if num(r.get('contacts_static')) + num(r.get('contacts_dynamic')) > 0]
    out['contact_eps'] = {'x': len(cont), 'n': len(ok), 'wilson': wilson(len(cont), len(ok)),
                          'ids': [r['scenario_id'] for r in cont]}
    out['collision_outcome'] = sum(1 for r in rows if r.get('outcome_contract') == 'collision')
    out['near_goal_stop_ge2s'] = sum(1 for r in ok if num(r.get('near_goal_stop_max_s')) >= 2.0)
    succ = [r for r in ok if truthy(r.get('strict_gt'))]
    out['final_yaw_succ_q'] = q([abs(num(r['final_yaw_err_gt'])) for r in succ], (50, 95, 100))
    for col in ('interventions', 'intervention_time_s', 'zone_stop_s', 'zone_critical_s'):
        v = np.array([num(r.get(col)) for r in ok], dtype=float)
        v = v[np.isfinite(v)]
        out[col] = {'mean': float(v.mean()) if v.size else None,
                    'sum': float(v.sum()) if v.size else None,
                    'eps_gt0': int((v > 0).sum())}
    for col in ('nav_time_s', 'path_length_m', 'cte_active_mean_m', 'max_dev_m', 'rtf',
                'wall_episode_s', 'ctrl_dt_sim_mean', 'decision_period_mean_s',
                'tick_dt_sim_mean', 'scan_to_cmd_p50_s', 'scan_age_p50_s', 'loc_err_max_m'):
        out[col + '_q'] = q([num(r.get(col)) for r in ok])
    return out


def paired(rows_a, rows_b, ids, scen, clusters_on):
    """팔 a − 팔 b 짝 비교 (ids 순서대로)."""
    res = {}
    both = [i for i in ids if i in rows_a and i in rows_b]
    valid = [i for i in both if excl(rows_a[i]) is None and excl(rows_b[i]) is None]
    cl_all = dict(zip(both, goal_clusters(both, scen))) if clusters_on else None
    for col in RATE_COLS:
        for mode, sel in (('valid', valid), ('all', both)):
            xa = np.array([success(rows_a[i], col, mode) for i in sel])
            xb = np.array([success(rows_b[i], col, mode) for i in sel])
            d = xa - xb
            e = {'n': len(sel), 'a': int(xa.sum()), 'b': int(xb.sum()),
                 'a_only': int(((xa == 1) & (xb == 0)).sum()),
                 'b_only': int(((xa == 0) & (xb == 1)).sum()),
                 'diff_boot': boot_ci(d)}
            if clusters_on:
                e['diff_cluster_boot'] = boot_ci(d, clusters=[cl_all[i] for i in sel])
                e['n_clusters'] = len(set(cl_all[i] for i in sel))
            if col == 'strict_gt':
                e['mcnemar_p'] = mcnemar_exact(e['a_only'], e['b_only'])
            res[f'{col}_{mode}'] = e
    ok = [i for i in both if excl(rows_a[i]) != 'E1' and excl(rows_b[i]) != 'E1']
    fd = [num(rows_a[i]['final_dist_gt_m']) - num(rows_b[i]['final_dist_gt_m']) for i in ok]
    res['final_dist_gt_diff_median'] = boot_ci([v for v in fd if v == v], stat='median')
    bs = [i for i in ok
          if truthy(rows_a[i].get('strict_gt')) and truthy(rows_b[i].get('strict_gt'))]
    for col in ('nav_time_s', 'path_length_m'):
        d = [num(rows_a[i][col]) - num(rows_b[i][col]) for i in bs]
        d = [v for v in d if v == v]
        res[col + '_both_success_mean'] = boot_ci(d)
        res[col + '_both_success_median'] = boot_ci(d, stat='median')
    for col in ('cte_active_mean_m', 'max_dev_m', 'interventions', 'intervention_time_s',
                'zone_stop_s'):
        d = [num(rows_a[i].get(col)) - num(rows_b[i].get(col)) for i in ok]
        res[col + '_mean'] = boot_ci([v for v in d if v == v])
    return res


def parse_wall(s):
    """'%Y-%m-%dT%H:%M:%S%z' → epoch (실패 None)."""
    try:
        return dt.datetime.strptime(s, '%Y-%m-%dT%H:%M:%S%z').timestamp()
    except (TypeError, ValueError):
        return None


def load_join(rows, sampler):
    """에피소드 벽시계 구간의 표집 평균 (GPU 사용률·메모리·Isaac 수·loadavg 1 분)."""
    if not sampler:
        return
    ts = np.array([s[0] for s in sampler])
    for r in rows:
        t0 = parse_wall(r.get('started_wall'))
        w = num(r.get('wall_episode_s'))
        if t0 is None or w != w:
            continue
        m = (ts >= t0) & (ts <= t0 + w)
        if not m.any():
            m = np.abs(ts - t0) == np.abs(ts - t0).min()
        sel = [sampler[k] for k in np.flatnonzero(m)]
        r['_gpu_util'] = float(np.mean([s[2] for s in sel]))
        r['_gpu_mem'] = float(np.mean([s[3] for s in sel]))
        r['_n_isaac'] = float(np.mean([s[4] for s in sel]))
        r['_load1'] = float(np.mean([s[1] for s in sel]))


def read_sampler(path):
    """sampler.tsv → [(epoch, load1, gpu_util, gpu_mem, n_isaac)]."""
    out = []
    if not os.path.exists(path):
        return out
    with open(path, encoding='utf-8') as f:
        for line in f:
            p = line.rstrip('\n').split('\t')
            if len(p) < 5:
                continue
            try:
                g = p[3].split(',')
                out.append((float(p[0]), float(p[2].split(',')[0]), float(g[0]), float(g[1]),
                            float(p[4])))
            except (ValueError, IndexError):
                continue
    return out


def timing(arm_rows):
    """부하·실효 주기 기술."""
    out = {}
    for arm, rows in arm_rows.items():
        rs = [r for r in rows.values() if excl(r) != 'E1']
        rtf = np.array([num(r['rtf']) for r in rs])
        cdt = np.array([num(r['ctrl_dt_sim_mean']) for r in rs])
        e = {'n': len(rs), 'rtf_q': q(rtf), 'ctrl_dt_sim_q': q(cdt),
             'ctrl_dt_over_005rtf_q': q(cdt / (0.05 * rtf)),
             'decisions_per_sim_s_q': q(1.0 / cdt),
             'wall_episode_q': q([num(r['wall_episode_s']) for r in rs]),
             'gpu_util_q': q([r.get('_gpu_util', float('nan')) for r in rs]),
             'n_isaac_q': q([r.get('_n_isaac', float('nan')) for r in rs]),
             'load1_q': q([r.get('_load1', float('nan')) for r in rs]),
             'by_lane': {}}
        for r in rs:
            e['by_lane'].setdefault(r.get('_lane') or '?', []).append(num(r['rtf']))
        e['by_lane'] = {k: {'n': len(v), 'rtf_median': float(np.median(v))}
                        for k, v in e['by_lane'].items()}
        if arm == 'dwa' and len(rs) >= 6:
            t1, t2 = np.nanpercentile(rtf, [100 / 3, 200 / 3])
            terc = {}
            for name, lo, hi in (('low', -1, t1), ('mid', t1, t2), ('high', t2, 9)):
                sel = [r for r in rs if lo < num(r['rtf']) <= hi]
                terc[name] = {
                    'n': len(sel), 'rtf_range': [lo if lo > 0 else None, hi if hi < 9 else None],
                    'strict_gt': sum(1 for r in sel if truthy(r.get('strict_gt'))),
                    'nav_time_median_success': q([num(r['nav_time_s']) for r in sel
                                                  if truthy(r.get('strict_gt'))], (50,))[0]}
            e['rtf_tercile'] = terc
        if arm != 'dwa':
            dp = np.array([num(r['decision_period_mean_s']) for r in rs])
            e['decision_period_out_of_band'] = int(np.sum((dp < 0.099) | (dp > 0.101)))
        out[arm] = e
    return out


def main():
    """명령행."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--c2', default='/home/CAPYI_AI/runs/bonus1/b1-gzc/c2')
    ap.add_argument('--scenarios', required=True)
    ap.add_argument('--isaac-eval', default=None)
    ap.add_argument('--suffix', default='')
    ap.add_argument('--out', required=True)
    ap.add_argument('--interim', action='store_true', help='중간 집계 표시 (보고서에 쓰지 않는다)')
    a = ap.parse_args()
    with open(a.scenarios, encoding='utf-8') as f:
        sd = json.load(f)
    scen = {s['id']: s for s in sd['scenarios']}
    order = sd['exec_order']
    arm_rows = {}
    for arm in ARMS:
        rows = load_arm(a.c2, arm, a.suffix)
        for i, r in rows.items():
            if r['scenario_id'] != order[i]:
                raise ValueError(f'{arm} exec_index {i}: {r["scenario_id"]} != {order[i]}')
            r['_lane'] = ''
            head = os.path.join(r['_src'], 'HEAD.txt')
            if os.path.exists(head):
                for line in open(head, encoding='utf-8'):
                    if line.startswith('lane '):
                        r['_lane'] = 'l' + line.split()[1]
        arm_rows[arm] = rows
    sampler = read_sampler(os.path.join(a.c2, 'sampler.tsv'))
    for rows in arm_rows.values():
        load_join(rows.values(), sampler)
    res = {'interim': a.interim, 'generated': dt.datetime.now().isoformat(timespec='seconds'),
           'c2': a.c2, 'suffix': a.suffix,
           'counts': {arm: len(r) for arm, r in arm_rows.items()}, 'summary': {}, 'paired': {}}
    for arm, rows in arm_rows.items():
        res['summary'][arm] = {
            st: arm_summary([r for r in rows.values() if r['difficulty'] == st]) for st in SETS}
    for x, y in COMPARE:
        if not arm_rows[x] or not arm_rows[y]:
            continue
        res['paired'][f'{x}-{y}'] = {}
        for st in SETS:
            ids = [i for i in order if scen[i]['set'] == st]
            ra = {r['scenario_id']: r for r in arm_rows[x].values()}
            rb = {r['scenario_id']: r for r in arm_rows[y].values()}
            res['paired'][f'{x}-{y}'][st] = paired(ra, rb, ids, scen, clusters_on=True)
    p = res['paired'].get('rl-dwa', {}).get('main', {})
    if p:
        v, al = p['strict_gt_valid'], p['strict_gt_all']
        sig_v, sig_a = v['mcnemar_p'] < 0.05, al['mcnemar_p'] < 0.05
        sign = (lambda e: 0 if not e['est'] else int(np.sign(e['est'])))
        res['primary'] = {
            'valid': v, 'all': al,
            'verdict_valid': ('차이 있음' if sig_v else '차이를 보이지 못함'),
            'robust': sig_v == sig_a and sign(v['diff_boot']) == sign(al['diff_boot']),
            'complete': all(len(arm_rows[k]) == 400 for k in ('rl', 'dwa'))}
    res['timing'] = timing(arm_rows)
    if a.isaac_eval and os.path.exists(a.isaac_eval):
        with open(a.isaac_eval, encoding='utf-8') as f:
            ev = json.load(f)
        res['isaac'] = {k: {'strict': v['intervals']['strict_successes'], 'n': v['intervals']['n'],
                            'rate': v['intervals']['strict_successes'] / v['intervals']['n']}
                        for k, v in ev['by'].items()}
        res['isaac']['policy'] = ev.get('policy')
    with open(a.out, 'w', encoding='utf-8') as f:
        json.dump(res, f, ensure_ascii=False, indent=1, default=float)
    print(json.dumps({'counts': res['counts'], 'interim': a.interim,
                      'primary_complete': res.get('primary', {}).get('complete')},
                     ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
