"""
비교군 A (팀 DWA) Gazebo 기준선 — 기존 측정 로그에서 지표를 **출처와 함께** 다시 뽑는다.

로그는 읽기만 한다 (`/tmp/claude-0/.../scratchpad/wt-integ/logs`, COORDINATION.md §1).
handover1006.md §4 의 함정을 코드로 막는다:
  - run_integration.sh 는 HEAD.txt 를 쓰지 않는다 → 실행마다 HEAD.txt 유무·값을 표에 싣고 없으면 '추정'.
  - 태그 재사용 → HEAD.txt 수정 시각과 result.json started_at 의 차이를 싣는다 (같은 실행이면 수 초).
  - 느슨한 grep 중복 → launch.log 를 grep 하지 않는다. 시행 단위 CSV(avoidance.csv) 와 result.json 만 쓴다.
  - junit.xml 은 전수 회귀의 합본일 수 있다 → 시나리오별 result.json 의 checks 로 판정한다.
"""

import csv
from datetime import datetime, timezone
import hashlib
import json
import os

import numpy as np

from . import binomial, schema

DEFAULT_LOG_ROOT = ('/tmp/claude-0/-home-CAPYI/c8bfdadc-8698-4d09-aac1-e3c1224c9abc/'
                    'scratchpad/wt-integ/logs')

# performance.md §2 가 '0 / 240 시행' 에 쓴 8 실행 (런타임 src/ 동일한 세 HEAD).
PRIMARY_08 = ('S0a', 'S0b', 'S0c', 'S0d', 'S0e', 'FINa', 'FINb', 'regG')
# 같은 시나리오인데 240 에 넣지 않은 실행 — 제외 사유는 performance.md §2·§4.1 의 문구.
EXCLUDED_08 = {
    'S0f': 'S0d·S0e 와 같은 커밋, 측위 실패로 240 에서 제외 (performance.md §2·§4.1)',
    'itest_fx2': ('TTC 를 참조하는 BT(navigate_to_pose.xml) 구성 — 240 시행의 BT 와 다름 '
                  '(performance.md §2, handover1006.md §7.6)'),
}
CTE_07 = ('regG', 'E07')
SCEN_08 = '08_dynamic_obstacles'
SCEN_07 = '07_path_tracking_cte'
ACTOR_SEED_NOTE = ('월드 actor 루프 seed 42 (src/amr_simulation/worlds/gen_warehouse_world.py:63), '
                   '하네스 seed 7 (tests/integration/amr_itest/config.py:163)')


def _sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        h.update(f.read())
    return h.hexdigest()[:12]


def _parse_time(s):
    return datetime.fromisoformat(s).astimezone(timezone.utc)


def provenance(log_root, tag, scenario):
    """
    실행 하나의 출처 — HEAD.txt 유무·값, 미커밋 변경, 태그 재사용 의심, 부하, 실행 당시 판정.

    head_lag_s = result.json started_at − HEAD.txt 수정 시각. 같은 실행 래퍼가 썼으면 수 초이고,
    음수이거나 1 시간 넘게 크면 '재사용 의심' 으로 표시한다.
    """
    run = os.path.join(log_root, tag)
    res_path = os.path.join(run, scenario, 'result.json')
    with open(res_path, encoding='utf-8') as f:
        res = json.load(f)
    head_path = os.path.join(run, 'HEAD.txt')
    unc_path = os.path.join(run, 'uncommitted.txt')
    out = {'tag': tag, 'scenario': scenario, 'result_json': res_path,
           'result_sha': _sha256(res_path), 'started_at': res.get('started_at'),
           'harness_seed': res.get('settings', {}).get('seed'),
           'loadavg_start': res.get('host', {}).get('start', {}).get('loadavg', [None])[0],
           'loadavg_end': res.get('host', {}).get('end', {}).get('loadavg', [None])[0],
           'failed_checks': [c['name'] for c in res.get('checks', []) if not c.get('passed')]}
    out['harness_passed'] = not out['failed_checks']
    if os.path.exists(head_path):
        with open(head_path, encoding='utf-8') as f:
            out['head'] = f.read().strip().split()[0] if os.path.getsize(head_path) else ''
        mtime = datetime.fromtimestamp(os.path.getmtime(head_path), tz=timezone.utc)
        lag = (_parse_time(res['started_at']) - mtime).total_seconds() if res.get(
            'started_at') else None
        out['head_lag_s'] = lag
        out['head_status'] = ('있음' if lag is not None and 0 <= lag <= 3600 else '재사용 의심')
    else:
        out['head'], out['head_lag_s'], out['head_status'] = None, None, '없음 (커밋 추정)'
    if os.path.exists(unc_path):
        with open(unc_path, encoding='utf-8') as f:
            lines = [ln for ln in f.read().splitlines() if ln.strip()]
        out['uncommitted'] = lines
    else:
        out['uncommitted'] = None
    return out, res


def load_trials_08(log_root, tag):
    """avoidance.csv → 시행 dict 리스트 (숫자 열은 float, nearest 는 문자열)."""
    path = os.path.join(log_root, tag, SCEN_08, 'avoidance.csv')
    rows = []
    with open(path, newline='', encoding='utf-8') as f:
        for r in csv.DictReader(f):
            d = {}
            for k, v in r.items():
                if k == 'nearest':
                    d[k] = v
                    continue
                try:
                    d[k] = float(v)
                except (TypeError, ValueError):
                    d[k] = float('nan')
            rows.append(d)
    return rows, path


def trials_to_episodes(tag, prov, trials, episode_index):
    """시나리오 08 시행 → 결과 CSV 행 (schema v1). 짝짓기 키 scenario_seed = 왕복 순번."""
    out = []
    head = prov.get('head') or 'unknown'
    for t in trials:
        contacts = int(t['contacts'])
        if contacts > 0:
            outcome = 'collision'
        elif int(t['reached']) == 1:
            outcome = 'success'
        else:
            outcome = 'timeout'
        out.append({
            'schema_version': schema.SCHEMA_VERSION, 'arm': 'A_dwa',
            'policy_id': f'dwa@{head[:7]}', 'train_seed': None, 'simulator': 'gazebo',
            'scenario_set': 'gz_s08_actor42', 'scenario_id': SCEN_08,
            'scenario_seed': int(t['trial']), 'difficulty': 'dynamic_crossing',
            'episode': episode_index, 'run_tag': tag, 'commit': head, 'safety_node': 'on',
            'outcome': outcome, 'contacts': contacts, 'time_s': t['time_s'],
            'max_dev_m': t['max_deviation_m'], 'return_max_s': t['return_s'],
            'min_clearance_m': t['min_distance_m'], 'min_ttc_s': t['min_ttc_s'],
        })
    return out


def _pct(a, q):
    a = np.asarray(a, dtype=float)
    a = a[np.isfinite(a)]
    return float(np.percentile(a, q)) if a.size else float('nan')


def summarize_08(log_root=DEFAULT_LOG_ROOT, tags=PRIMARY_08):
    """
    시나리오 08 기준선 — 실행별 출처·요약과 묶음 지표, 결과 CSV 행.

    반환 dict: runs(실행별), pooled(묶음), episodes(schema 행), sources(파일·sha).
    """
    runs, episodes, pooled_trials, sources = [], [], [], []
    per_policy = {}
    for tag in tags:
        prov, res = provenance(log_root, tag, SCEN_08)
        trials, path = load_trials_08(log_root, tag)
        sources.append((path, _sha256(path)))
        key = prov.get('head') or 'unknown'
        idx = per_policy.get(key, 0)
        per_policy[key] = idx + 1
        episodes += trials_to_episodes(tag, prov, trials, idx)
        pooled_trials += trials
        col = {k: [t[k] for t in trials] for k in trials[0]}
        runs.append(dict(prov, n=len(trials), reached=int(sum(col['reached'])),
                         contacts=int(sum(col['contacts'])),
                         time_median=_pct(col['time_s'], 50), time_max=max(col['time_s']),
                         max_dev=max(col['max_deviation_m']), return_max=max(col['return_s']),
                         return_first_max=max(col['return_first_s']),
                         dev_first_over_1m=sum(x > 1.0 for x in col['max_dev_first_m']),
                         return_first_over_5s=sum(x > 5.0 for x in col['return_first_s']),
                         replanned=sum(x > 1 for x in col['n_plans']),
                         min_clearance=min(col['min_distance_m']),
                         min_ttc=min(col['min_ttc_s']),
                         summary=res['measurements'].get('summary', {})))
    col = {k: np.array([t[k] for t in pooled_trials], dtype=float)
           for k in pooled_trials[0] if k != 'nearest'}
    n = len(pooled_trials)
    x_contact = int((col['contacts'] > 0).sum())
    x_reached = int(col['reached'].sum())
    pooled = {
        'n': n, 'runs': len(tags), 'reached': x_reached,
        'reached_cp_lower': binomial.cp_lower(x_reached, n, 0.95),
        'contact_trials': x_contact, 'contact_rate': binomial.rate_summary(x_contact, n),
        'contact_ub_if_30_effective': binomial.cp_upper(x_contact, 30) if x_contact == 0 else None,
        'time_median': _pct(col['time_s'], 50), 'time_p90': _pct(col['time_s'], 90),
        'time_max': float(np.nanmax(col['time_s'])),
        'time_iqm': float(np.mean(np.sort(col['time_s'])[n // 4: n - n // 4])),
        'min_clearance_min': float(np.nanmin(col['min_distance_m'])),
        'min_clearance_p5': _pct(col['min_distance_m'], 5),
        'min_clearance_median': _pct(col['min_distance_m'], 50),
        'min_ttc_min': float(np.nanmin(col['min_ttc_s'])),
        'min_ttc_p5': _pct(col['min_ttc_s'], 5),
        'max_dev_max': float(np.nanmax(col['max_deviation_m'])),
        'max_dev_p95': _pct(col['max_deviation_m'], 95),
        'return_max': float(np.nanmax(col['return_s'])),
        'dev_first_over_1m': int((col['max_dev_first_m'] > 1.0).sum()),
        'return_first_over_5s': int((col['return_first_s'] > 5.0).sum()),
        'replanned': int((col['n_plans'] > 1).sum()),
        'harness_failed_runs': [r['tag'] for r in runs if not r['harness_passed']],
    }
    return {'runs': runs, 'pooled': pooled, 'episodes': episodes, 'sources': sources}


def summarize_excluded_08(log_root=DEFAULT_LOG_ROOT, tags=EXCLUDED_08):
    """240 에서 뺀 실행 — 접촉 수와 제외 사유."""
    out = []
    for tag, reason in tags.items():
        prov, _ = provenance(log_root, tag, SCEN_08)
        trials, path = load_trials_08(log_root, tag)
        out.append(dict(prov, n=len(trials), contacts=int(sum(t['contacts'] for t in trials)),
                        reason=reason, source=path, source_sha=_sha256(path)))
    return out


def summarize_07(log_root=DEFAULT_LOG_ROOT, tags=CTE_07):
    """
    시나리오 07 경로 추종 — CTE (직선·곡선 평균/p95/최대) 와 목표별 주행 시간.

    result.json 의 evaluation_cte_rows 를 cte_amr_01.csv 에서 다시 계산해 대조한다 (불일치면 표시).
    """
    out = []
    for tag in tags:
        prov, res = provenance(log_root, tag, SCEN_07)
        csv_path = os.path.join(log_root, tag, SCEN_07, 'cte_amr_01.csv')
        seg = {}
        with open(csv_path, newline='', encoding='utf-8') as f:
            for r in csv.DictReader(f):
                seg.setdefault(r['segment'], []).append(abs(float(r['cte'])))
        rows = {}
        for s, v in seg.items():
            v = np.array(v)
            rows[s] = {'n': int(v.size), 'mean': float(v.mean()),
                       'p95': float(np.percentile(v, 95)), 'max': float(v.max())}
        reported = {r['segment']: r for r in res['measurements'].get('evaluation_cte_rows', [])}
        agree = all(abs(rows[s]['mean'] - reported[s]['value']) < 1e-6
                    and rows[s]['n'] == reported[s]['count'] for s in reported if s in rows)
        goals = [(g['goal'], g['time_s'], g['status']) for g in res['measurements']['goals']]
        eta = res['measurements'].get('eta')
        out.append(dict(prov, cte=rows, cte_matches_result_json=agree, goals=goals, eta=eta,
                        source=csv_path, source_sha=_sha256(csv_path)))
    return out
