"""
A3b 재판정 분석 (numpy, Isaac 불필요) — eval_unbiased 결과 디렉터리들 → 표·진단 JSON.

    python3 rl/isaac/scripts/analyze_rejudge.py --root <rejudge 디렉터리> --assets <자산 캐시> --out <json>

입력 이름 규칙: <root>/A3b_<phi>_s<seed>_<range>[_dwell10]/{eval.json, episodes.npz}.
산출: 판정 지표 표(φ × 시드 × 경로 범위), dwell 표, 짝 비교(같은 첫 에피소드 경로 — 공통 난수), 목표 직전 정지
진단(사전 등록 분류 · 끝 D · 존 · 래치 · 원인), 실패 목표의 여유(목표점·풋프린트·직진 접근 거리 [계산]),
정체 끝 자세의 목표 기준 종·횡 오프셋.
"""

import argparse
import glob
import json
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from amr_isaac import eval_metrics as EV  # noqa: E402

NAME = re.compile(r'A3b_(lite|canonical)_s(\d+)_(3-15|3-30)(_dwell10)?$')


def load_runs(root):
    """{(phi, seed, range, dwell): (eval.json, episodes)}."""
    out = {}
    for d in sorted(glob.glob(os.path.join(root, 'A3b_*'))):
        m = NAME.match(os.path.basename(d))
        if not m or not os.path.exists(os.path.join(d, 'episodes.npz')):
            continue
        key = (m.group(1), int(m.group(2)), m.group(3), bool(m.group(4)))
        ev = json.load(open(os.path.join(d, 'eval.json')))
        ep = dict(np.load(os.path.join(d, 'episodes.npz')))
        out[key] = (ev, ep, d)
    return out


def pool_paths(assets):
    """held-out 풀 seed 1 의 경로 점 목록 (원래 번호)."""
    f = sorted(glob.glob(os.path.join(assets, 'paths_*_s1.npz')))[0]
    z = np.load(f)
    pts, offs = z['pts'], z['offs']
    return [pts[offs[i]:offs[i + 1]] for i in range(len(offs) - 1)], f


def goal_frame_offset(pose, path):
    """끝 자세의 목표 기준 (종 = 경로 끝 접선 방향 앞 +, 횡 = 왼쪽 +, 방위차)."""
    g = path[-1]
    j = max(0, path.shape[0] - 6)
    d = g - path[j]
    th = math.atan2(d[1], d[0])
    dx, dy = pose[0] - g[0], pose[1] - g[1]
    lon = math.cos(th) * dx + math.sin(th) * dy
    lat = -math.sin(th) * dx + math.cos(th) * dy
    dth = (pose[2] - th + math.pi) % (2 * math.pi) - math.pi
    return lon, lat, dth


def main():
    """분석."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', required=True)
    ap.add_argument('--assets', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    runs = load_runs(a.root)
    paths, pool_file = pool_paths(a.assets)
    geo = np.load(sorted(glob.glob(os.path.join(a.assets, 'warehouse_*.npz')))[0])
    obb = geo['obb']
    rep = {'pool_file': pool_file, 'runs': sorted(os.path.basename(v[2]) for v in runs.values())}
    # ---- 판정 지표 표
    table = []
    for (phi, seed, rng, dwell), (ev, ep, _d) in sorted(runs.items()):
        lay = EV.layered(ep)
        dg = EV.stop_diagnosis(ep)
        oc = ep['outcome']
        table.append({
            'phi': phi, 'seed': seed, 'range': rng, 'dwell_k': 10 if dwell else 1,
            'n': int(oc.size), 'unfinished': ev['unfinished'], 'path_ids_sha': ev['path_ids_sha'],
            'strict_success': lay['strict_success'], 'strict_count': int((oc == 0).sum()),
            'nav2_success': lay['nav2_compatible_success'],
            'nav2_count': int(ep['nav2_reach'].sum()),
            'collision': int((oc == 1).sum()), 'time_limit': int((oc == 2).sum()),
            'stall': int((oc == 3).sum()),
            'final_dist_all': lay['final_dist_all'], 'final_dist_fail': lay['final_dist_fail'],
            'fail_within_0p25m': lay['fail_within_0p25m'],
            'fail_within_0p5m': lay['fail_within_0p5m'],
            'intervention_per_ep': lay['intervention_per_ep'],
            'intervention_success_eps': lay['intervention_success_eps'],
            'intervention_share_from_fail': lay['intervention_share_from_fail'],
            'time_s_mean_success': ev['time_s_mean_success'], 'wall_s': ev['wall_s'],
            'stop_diag_counts': dg['counts'], 'stop_diag_n': dg['n']})
    rep['table'] = table
    # ---- 짝 비교 (같은 시드·범위·dwell, lite 대 canonical — 첫 에피소드 경로가 같다)
    pairs = []
    for (phi, seed, rng, dwell), (ev, ep, _d) in sorted(runs.items()):
        if phi != 'lite' or ('canonical', seed, rng, dwell) not in runs:
            continue
        ev2, ep2, _ = runs[('canonical', seed, rng, dwell)]
        same = bool(np.array_equal(ep['path_id_orig'], ep2['path_id_orig']))
        s1, s2 = ep['outcome'] == 0, ep2['outcome'] == 0
        pairs.append({'seed': seed, 'range': rng, 'dwell_k': 10 if dwell else 1,
                      'same_paths': same, 'both_success': int((s1 & s2).sum()),
                      'lite_only': int((s1 & ~s2).sum()), 'canonical_only': int((~s1 & s2).sum()),
                      'both_fail': int((~s1 & ~s2).sum()),
                      'mcnemar_discordant': int((s1 ^ s2).sum())})
    rep['paired'] = pairs
    # ---- 정지 진단 + 목표 여유 (실패 에피소드, 3–15 m·3–30 m, dwell 없음)
    cache = {}

    def clear(pid):
        if pid not in cache:
            cache[pid] = EV.goal_clearances(paths[pid], obb)
        return cache[pid]
    diag = []
    for (phi, seed, rng, dwell), (ev, ep, _d) in sorted(runs.items()):
        if dwell:
            continue
        dg = EV.stop_diagnosis(ep)
        fails = np.nonzero(ep['outcome'] != 0)[0]
        rows = []
        for i in fails:
            pid = int(ep['path_id_orig'][i])
            c = clear(pid)
            lon, lat, dth = goal_frame_offset(ep['pose_end'][i], paths[pid])
            row = {'i': int(i), 'path': pid, 'outcome': int(ep['outcome'][i]),
                   'd_end': float(ep['d_end'][i]), 'lon': lon, 'lat': lat, 'dtheta': dth,
                   'goal_point_clr': c['goal_point'], 'goal_fp_clr': c['goal_footprint'],
                   'goal_front_straight': c['goal_front_straight'],
                   'intervention': float(ep['cost'][i, 1])}
            for r in dg['episodes']:
                if r['i'] == int(i):
                    row.update({k: r[k] for k in ('class', 'frac_blocked', 'frac_policy_stop',
                                                  'mean_v_in', 'mean_v_out', 'D_end',
                                                  'guard_end', 'zone_end', 'prox_end',
                                                  'causes_end')})
            rows.append(row)
        diag.append({'phi': phi, 'seed': seed, 'range': rng, 'counts': dg['counts'],
                     'n_near_goal_fail': dg['n'], 'fails': rows})
    rep['diagnosis'] = diag
    # ---- 목표 여유 분포: held-out 풀 전체 대 실패 목표 (3–15 m 판정 범위의 첫 에피소드 경로)
    any_run = next(iter(runs.values()))[1]
    ref = runs.get(('lite', 0, '3-15', False), (None, any_run))[1]
    all_ids = np.unique(ref['path_id_orig'])
    fs = np.array([clear(int(p))['goal_front_straight'] for p in all_ids])
    fp = np.array([clear(int(p))['goal_footprint'] for p in all_ids])
    rep['goal_clearance_eval_3_15'] = {
        'paths': int(all_ids.size),
        'front_straight_lt_0p30': int((fs < 0.30).sum()),
        'front_straight_lt_0p25': int((fs < 0.25).sum()),
        'footprint_lt_0p30': int((fp < 0.30).sum()), 'footprint_lt_0p10': int((fp < 0.10).sum()),
        'front_straight_quantiles': [float(np.quantile(fs[np.isfinite(fs)], q))
                                     for q in (0.01, 0.05, 0.5)]}
    with open(a.out, 'w', encoding='utf-8') as f:
        json.dump(rep, f, indent=1, ensure_ascii=False)
    for row in table:
        print(f"{row['phi']:9s} s{row['seed']} {row['range']} k{row['dwell_k']:2d}  "
              f"strict {row['strict_count']:4d}/{row['n']} ({row['strict_success']:.4f})  "
              f"nav2 {row['nav2_count']:4d}  coll {row['collision']}  stall {row['stall']}  "
              f"tl {row['time_limit']}  iv/ep {row['intervention_per_ep']:.3f}  "
              f"diag {row['stop_diag_counts']}")
    for p in pairs:
        print('paired', p)
    print('goal clearance', rep['goal_clearance_eval_3_15'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
