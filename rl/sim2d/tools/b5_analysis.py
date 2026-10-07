#!/usr/bin/env python3
"""
B5 집계 (사전 등록 — 결과를 보기 전에 커밋한다; 노트 항목 2026-10-08_*_b1-b5_* 「판정 규칙」 과 같은 내용).

입력: --runs <dir> 아래 B5_dwa_W1/, B5_rl_s<N>_W1/ (N = 0..9) 의 episodes.npz (b5_crosseval.py 출력),
      --isaac <dir> 아래 B1e_ppo10_s<N>_final_sets/episodes.npz (b1-ppo10-eval, 같은 정책·같은 집합 행).
출력: --out 아래 analysis.json, analysis.md.

정의
  - 성공 = 계약 엄격 성공 (outcome 0: 목표 0.10 m ∧ |v| ≤ 0.05 m/s). Nav2 호환 = 에피소드 중 목표 0.25 m 안 도달.
  - 짝 = 같은 행 (같은 경로·같은 시작 yaw·같은 스캔 잡음열). DWA 는 결정론 단일 팔.
  - Δ(집합) = (1/10)Σ_s p_RL,s − p_DWA = 행 평균 짝 차의 시드 평균.
  - 구간 (사전 등록): 2 원 부트스트랩 — RL 시드 10 개 복원 추출 × 목표 군집(같은 끝점) 복원 추출, 독립, B 10 000,
    seed 20261008, 백분위 95 %. 성분으로 시드만·군집만 부트스트랩도 낸다.
판정
  - P1 (주 결론, 확증 하나): main_3_15 의 Δ. 2 원 구간이 0 을 빼면 "RL 이 높다/낮다", 아니면 "가르지 못함".
  - 방향 (Gazebo 대비): 두 쪽 구간이 모두 0 을 빼고 부호가 같으면 "같은 방향", 반대면 "반대 방향",
    한쪽이라도 0 을 포함하면 "방향 판정 불가" (점추정 부호는 기술로만 적는다).
    Gazebo 값 (c_gazebo.md §2·§3.2, 시드 7, 3–30 m): 주 −0.010 [−0.030, +0.010],
    스트레스 +0.126 [+0.060, +0.196].
    2D 쪽 비교 집합: 주 → main_3_15 (P1) 와 main_3_30, 스트레스 → stress_3_30 (Gazebo 와 같은 길이 범위, S1) 와
    stress_3_15.
  - 시드 7 (대표 정책, Gazebo 와 같은 체크포인트): 집합마다 짝 차와 목표 군집 부트스트랩 구간,
    main_3_15 에는 McNemar 정확 양측 (부결론 — Gazebo 주 결론과 같은 검정).
  - 시드 축: 시드별 Δ_s 와 부호 검정 (정확 양측, 0 은 뺀다) — 부결론.
  - 나머지 (Nav2 호환·충돌·정체·시간·개입·STOP 존·이탈·2D − Isaac) 는 기술 (구간만, p 값 없음).
"""

import argparse
import json
import os
import sys

import numpy as np
from scipy import stats

HERE = os.path.dirname(os.path.abspath(__file__))
RL = os.path.dirname(os.path.dirname(HERE))
SETS = ('main_3_15', 'stress_3_15', 'main_3_30', 'stress_3_30')
SEEDS = tuple(range(10))
REP_SEED = 7
B = 10000
BOOT_SEED = 20261008
GAZEBO = {'main': {'diff': -0.010, 'lo': -0.030, 'hi': 0.010, 'n': 199, 'p_mcnemar': 0.625,
                   'rl': '197/200', 'dwa': '198/199'},
          'stress': {'diff': 0.126, 'lo': 0.060, 'hi': 0.196, 'n': 199, 'p_mcnemar': None,
                     'rl': '174/200', 'dwa': '148/199'}}
DIR_PAIRS = (('main', 'main_3_15'), ('main', 'main_3_30'), ('stress', 'stress_3_30'),
             ('stress', 'stress_3_15'))


def load_ep(path):
    z = np.load(path, allow_pickle=False)
    return {k: z[k] for k in z.files}


def wilson(x, n, z=1.959963984540054):
    if n == 0:
        return (float('nan'), float('nan'))
    ph = x / n
    c = ph + z * z / (2 * n)
    h = z * np.sqrt(ph * (1 - ph) / n + z * z / (4 * n * n))
    d = 1 + z * z / n
    return (float((c - h) / d), float((c + h) / d))


def two_way(S, D, groups, reps=B, seed=BOOT_SEED, mode='both'):
    """S [ns, n] RL 0/1, D [n] DWA 0/1, groups [n] → (점추정, 하한, 상한). mode: both | seed | cluster."""
    S = np.asarray(S, dtype=np.float64)
    D = np.asarray(D, dtype=np.float64)
    _, inv = np.unique(groups, return_inverse=True)
    k = int(inv.max()) + 1
    ns = S.shape[0]
    A = np.stack([np.bincount(inv, weights=S[i], minlength=k) for i in range(ns)])   # [ns, k]
    Dk = np.bincount(inv, weights=D, minlength=k)
    nk = np.bincount(inv, minlength=k).astype(np.float64)
    est = float(A.sum() / (ns * nk.sum()) - Dk.sum() / nk.sum())
    rng = np.random.default_rng(seed)
    out = np.empty(reps)
    for r in range(reps):
        w = np.bincount(rng.integers(0, k, k), minlength=k) if mode in ('both', 'cluster') \
            else np.ones(k)
        m = np.bincount(rng.integers(0, ns, ns), minlength=ns) if mode in ('both', 'seed') \
            else np.ones(ns)
        tot = nk @ w
        out[r] = (m @ A @ w) / (ns * tot) - (Dk @ w) / tot
    lo, hi = np.percentile(out, [2.5, 97.5])
    return est, float(lo), float(hi)


def cluster_diff(d, groups, reps=B, seed=BOOT_SEED):
    """행별 짝 차 d [n] 의 평균과 목표 군집 부트스트랩 95 %."""
    d = np.asarray(d, dtype=np.float64)
    _, inv = np.unique(groups, return_inverse=True)
    k = int(inv.max()) + 1
    s = np.bincount(inv, weights=d, minlength=k)
    c = np.bincount(inv, minlength=k).astype(np.float64)
    rng = np.random.default_rng(seed)
    dr = rng.integers(0, k, size=(reps, k))
    boots = s[dr].sum(1) / c[dr].sum(1)
    lo, hi = np.percentile(boots, [2.5, 97.5])
    return float(d.mean()), float(lo), float(hi)


def mcnemar_exact(a, b):
    a, b = np.asarray(a, bool), np.asarray(b, bool)
    n10, n01 = int((a & ~b).sum()), int((~a & b).sum())
    p = 1.0 if n10 + n01 == 0 else float(stats.binomtest(n10, n10 + n01, 0.5).pvalue)
    return {'n10': n10, 'n01': n01, 'p_exact_two_sided': p}


def verdict(lo, hi, pos='RL 이 높다', neg='RL 이 낮다'):
    if lo > 0:
        return pos
    if hi < 0:
        return neg
    return '가르지 못함'


def direction(lo2, hi2, lo_g, hi_g):
    s2 = 1 if lo2 > 0 else (-1 if hi2 < 0 else 0)
    sg = 1 if lo_g > 0 else (-1 if hi_g < 0 else 0)
    if s2 == 0 or sg == 0:
        return '방향 판정 불가'
    return '같은 방향' if s2 == sg else '반대 방향'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', required=True)
    ap.add_argument('--isaac', required=True)
    ap.add_argument('--pool', required=True)
    ap.add_argument('--world-tag', default='W1')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    sys.path.insert(0, os.path.join(RL, 'isaac'))
    from amr_isaac import eval_sets as ES
    os.makedirs(a.out, exist_ok=True)
    dwa = load_ep(os.path.join(a.runs, f'B5_dwa_{a.world_tag}', 'episodes.npz'))
    rl = {s: load_ep(os.path.join(a.runs, f'B5_rl_s{s}_{a.world_tag}', 'episodes.npz'))
          for s in SEEDS}
    isa = {}
    for s in SEEDS:
        p = os.path.join(a.isaac, f'B1e_ppo10_s{s}_final_sets', 'episodes.npz')
        if os.path.exists(p):
            isa[s] = load_ep(p)
    z = np.load(a.pool)
    groups_all, _ = ES.goal_groups(z['pts'], z['offs'], dwa['pid'])
    # 행 정렬 검사
    for s in SEEDS:
        assert np.array_equal(rl[s]['pid'], dwa['pid'])
        assert np.array_equal(rl[s]['set'], dwa['set'])
        assert np.array_equal(rl[s]['yaw0'], dwa['yaw0'])
        assert (rl[s]['outcome'] >= 0).all(), f'시드 {s} unfinished'
    assert (dwa['outcome'] >= 0).all(), 'DWA unfinished'
    for s, e in isa.items():
        assert np.array_equal(e['path_id_orig'], dwa['pid']), f'Isaac 시드 {s} 행 순서가 다르다'
    res = {'world': a.world_tag, 'B': B, 'boot_seed': BOOT_SEED, 'sets': {}, 'gazebo': GAZEBO}
    md = [f'# B5 집계 ({a.world_tag})', '']
    md += ['| 집합 | n | 군집 | DWA 엄격 | RL 10 시드 평균 | Δ RL−DWA [2 원 95 %] | 시드만 | 군집만 | 판정 |',
           '|---|---|---|---|---|---|---|---|---|']
    for st in SETS:
        m = dwa['set'] == st
        n = int(m.sum())
        g = groups_all[m]
        D = dwa['outcome'][m] == 0
        S = np.stack([rl[s]['outcome'][m] == 0 for s in SEEDS])
        est, lo, hi = two_way(S, D, g)
        _, slo, shi = two_way(S, D, g, mode='seed')
        _, clo, chi = two_way(S, D, g, mode='cluster')
        ps = S.mean(1)
        ds = ps - D.mean()
        nz = ds[ds != 0]
        sign_p = float(stats.binomtest(int((nz > 0).sum()), len(nz), 0.5).pvalue) if len(nz) \
            else 1.0
        blk = {'n': n, 'goal_clusters': int(len(np.unique(g))),
               'dwa_success': int(D.sum()), 'dwa_rate': float(D.mean()),
               'dwa_wilson': wilson(int(D.sum()), n),
               'rl_rate_by_seed': {str(s): float(ps[i]) for i, s in enumerate(SEEDS)},
               'rl_rate_mean': float(ps.mean()),
               'delta': est, 'delta_ci_two_way': [lo, hi], 'delta_ci_seed_only': [slo, shi],
               'delta_ci_cluster_only': [clo, chi], 'verdict_two_way': verdict(lo, hi),
               'delta_by_seed': {str(s): float(ds[i]) for i, s in enumerate(SEEDS)},
               'seeds_above_dwa': int((ds > 0).sum()), 'seeds_below_dwa': int((ds < 0).sum()),
               'sign_test_p_two_sided': sign_p}
        # 시드 7
        r7 = rl[REP_SEED]['outcome'][m] == 0
        d7 = r7.astype(float) - D.astype(float)
        e7, l7, h7 = cluster_diff(d7, g)
        blk['seed7'] = {'rl_success': int(r7.sum()), 'rl_rate': float(r7.mean()),
                        'rl_wilson': wilson(int(r7.sum()), n), 'diff': e7,
                        'diff_ci_cluster': [l7, h7], **mcnemar_exact(r7, D)}
        # 시드별 McNemar (기술)
        blk['mcnemar_by_seed'] = {str(s): mcnemar_exact(S[i], D) for i, s in enumerate(SEEDS)}
        # Nav2 호환
        Dn = dwa['nav2'][m]
        Sn = np.stack([rl[s]['nav2'][m] for s in SEEDS])
        en, ln_, hn = two_way(Sn, Dn, g)
        blk['nav2'] = {'dwa': int(Dn.sum()), 'rl_mean': float(Sn.mean(1).mean()),
                       'rl_seed7': int(rl[REP_SEED]['nav2'][m].sum()),
                       'delta': en, 'delta_ci_two_way': [ln_, hn]}
        # 결말 분포

        def outc(o):
            kinds = (('success', 0), ('collision', 1), ('stall', 2), ('timeout', 3))
            return {k: int((o == v).sum()) for k, v in kinds}
        blk['outcomes_dwa'] = outc(dwa['outcome'][m])
        blk['outcomes_rl_sum10'] = outc(np.concatenate([rl[s]['outcome'][m] for s in SEEDS]))
        blk['outcomes_rl_seed7'] = outc(rl[REP_SEED]['outcome'][m])
        blk['collision_moving_dwa'] = int(dwa['collision_moving'][m].sum())
        blk['collision_moving_rl_sum10'] = int(sum(rl[s]['collision_moving'][m].sum()
                                                   for s in SEEDS))
        # 시간 (둘 다 성공한 짝, 시드별 평균 차의 시드 평균)
        tdiff = []
        for s in SEEDS:
            both = (rl[s]['outcome'][m] == 0) & D
            if both.any():
                tdiff.append(float((rl[s]['time_s'][m][both] - dwa['time_s'][m][both]).mean()))
        blk['time_both_success_diff_mean_over_seeds'] = float(np.mean(tdiff)) if tdiff else None
        blk['time_both_success_diff_by_seed'] = tdiff
        both7 = r7 & D
        blk['time_both_success_seed7'] = {
            'n': int(both7.sum()),
            'diff': cluster_diff(rl[REP_SEED]['time_s'][m][both7] - dwa['time_s'][m][both7],
                                 g[both7]) if both7.sum() > 1 else None}
        blk['time_success_mean'] = {
            'dwa': float(dwa['time_s'][m][D].mean()) if D.any() else None,
            'rl_mean': float(np.mean([rl[s]['time_s'][m][rl[s]['outcome'][m] == 0].mean()
                                      for s in SEEDS]))}
        # 안전·이탈 (에피소드 평균, 기술)

        def per_ep(e, mm):
            return {'interv_events': float(e['interv_events'][mm].mean()),
                    'interv_time_s': float((e['interv_sub'][mm] * 0.02).mean()),
                    'stop_zone_time_s': float((e['stop_sub'][mm] * 0.02).mean()),
                    'interv_cost': float(e['cost'][mm][:, 1].mean()),
                    'max_e_mean': float(e['max_e'][mm].mean()),
                    'travel_over_path': float((e['travel'][mm] / e['path_len'][mm]).mean()),
                    'spin_steps': float(e['spin_steps'][mm].mean())}
        blk['per_episode_dwa'] = per_ep(dwa, m)
        rows_rl = [per_ep(rl[s], m) for s in SEEDS]
        blk['per_episode_rl_mean'] = {k: float(np.mean([r[k] for r in rows_rl]))
                                      for k in rows_rl[0]}
        blk['per_episode_rl_seed7'] = per_ep(rl[REP_SEED], m)
        # 실패 꼴 (사전 등록 분류): 출발 교착 = 정체 ∧ 목표 쪽 진행 < 0.5 m, 목표 앞 정체 = 정체 ∧ 최소 목표 거리
        # ≤ 0.25 m, 중간 정체 = 그 밖의 정체, 그리고 충돌·시간 상한

        def modes(e, mm):
            st = e['outcome'][mm] == 2
            prog = e['d_start'][mm] - e['goal_min'][mm]
            near = e['goal_min'][mm] <= 0.25
            start = st & (prog < 0.5)
            return {'start_deadlock': int(start.sum()),
                    'near_goal_stall': int((st & near & ~start).sum()),
                    'mid_stall': int((st & ~near & ~start).sum()),
                    'collision': int((e['outcome'][mm] == 1).sum()),
                    'timeout': int((e['outcome'][mm] == 3).sum())}
        blk['fail_modes_dwa'] = modes(dwa, m)
        fm = [modes(rl[s], m) for s in SEEDS]
        blk['fail_modes_rl_sum10'] = {k: int(sum(f[k] for f in fm)) for k in fm[0]}
        blk['fail_modes_rl_seed7'] = modes(rl[REP_SEED], m)
        blk['dwa_no_valid_ticks'] = int(dwa['dwa_no_valid_ticks'][m].sum())
        blk['dwa_exceptions'] = int(dwa['dwa_exceptions'][m].sum())
        fail = dwa['outcome'][m] != 0
        blk['dwa_fail_final_dist'] = (
            [float(np.quantile(dwa['d_goal_end'][m][fail], q)) for q in (0.1, 0.5, 0.9)]
            if fail.any() else None)
        blk['dwa_fail_within_0p25'] = int((dwa['goal_min'][m][fail] <= 0.25).sum())
        # 2D − Isaac (같은 정책, 같은 행)
        if isa:
            gap = {}
            for s in SEEDS:
                if s not in isa:
                    continue
                io = isa[s]['outcome'][m] == 0
                r2 = rl[s]['outcome'][m] == 0
                gap[str(s)] = {'isaac_rate': float(io.mean()), 'twod_rate': float(r2.mean()),
                               'diff': float(r2.mean() - io.mean()),
                               'row_agree': float((io == r2).mean()),
                               'isaac_only': int((io & ~r2).sum()),
                               'twod_only': int((~io & r2).sum())}
            dd = np.array([v['diff'] for v in gap.values()])
            rng = np.random.default_rng(BOOT_SEED)
            bs = np.array([dd[rng.integers(0, len(dd), len(dd))].mean() for _ in range(B)])
            blk['twod_minus_isaac'] = {'by_seed': gap, 'mean': float(dd.mean()),
                                       'ci_seed': [float(np.percentile(bs, 2.5)),
                                                   float(np.percentile(bs, 97.5))]}
        res['sets'][st] = blk
        md.append(f'| {st} | {n} | {blk["goal_clusters"]} | {int(D.sum())}/{n} '
                  f'({D.mean():.4f}) | {ps.mean():.4f} | {est:+.4f} [{lo:+.4f}, {hi:+.4f}] | '
                  f'[{slo:+.4f}, {shi:+.4f}] | [{clo:+.4f}, {chi:+.4f}] | '
                  f'{blk["verdict_two_way"]} |')
    # 주 결론과 방향
    p1 = res['sets']['main_3_15']
    res['P1'] = {'set': 'main_3_15', 'delta': p1['delta'], 'ci': p1['delta_ci_two_way'],
                 'verdict': p1['verdict_two_way']}
    res['direction'] = {}
    for kind, st in DIR_PAIRS:
        b = res['sets'][st]
        gz = GAZEBO[kind]
        lo, hi = b['delta_ci_two_way']
        lo7, hi7 = b['seed7']['diff_ci_cluster']
        res['direction'][f'{kind}:{st}'] = {
            'twod_delta': b['delta'], 'twod_ci': [lo, hi], 'gazebo_diff': gz['diff'],
            'gazebo_ci': [gz['lo'], gz['hi']],
            'verdict_10seed': direction(lo, hi, gz['lo'], gz['hi']),
            'point_sign_agree_10seed': bool(np.sign(b['delta']) == np.sign(gz['diff'])),
            'twod_seed7_diff': b['seed7']['diff'], 'twod_seed7_ci': [lo7, hi7],
            'verdict_seed7': direction(lo7, hi7, gz['lo'], gz['hi']),
            'point_sign_agree_seed7': bool(np.sign(b['seed7']['diff']) == np.sign(gz['diff']))}
    md += ['', f'P1 (main_3_15): Δ {p1["delta"]:+.4f} [{p1["delta_ci_two_way"][0]:+.4f}, '
           f'{p1["delta_ci_two_way"][1]:+.4f}] → {p1["verdict_two_way"]}', '',
           '| 비교 | 2D Δ (10 시드) [95 %] | 2D 시드 7 [군집 95 %] | Gazebo (시드 7) | 판정 10 시드 | 판정 시드 7 |',
           '|---|---|---|---|---|---|']
    for key, v in res['direction'].items():
        c2, c7, cg = v['twod_ci'], v['twod_seed7_ci'], v['gazebo_ci']
        md.append(f'| {key} | {v["twod_delta"]:+.4f} [{c2[0]:+.4f}, {c2[1]:+.4f}] | '
                  f'{v["twod_seed7_diff"]:+.4f} [{c7[0]:+.4f}, {c7[1]:+.4f}] | '
                  f'{v["gazebo_diff"]:+.3f} [{cg[0]:+.3f}, {cg[1]:+.3f}] | '
                  f'{v["verdict_10seed"]} | {v["verdict_seed7"]} |')
    md += ['', '| 집합 | 시드 7 RL | DWA | 짝 차 [군집 95 %] | McNemar n10/n01 p |',
           '|---|---|---|---|---|']
    for st in SETS:
        b = res['sets'][st]
        s7 = b['seed7']
        md.append(f'| {st} | {s7["rl_success"]}/{b["n"]} | {b["dwa_success"]}/{b["n"]} | '
                  f'{s7["diff"]:+.4f} [{s7["diff_ci_cluster"][0]:+.4f}, '
                  f'{s7["diff_ci_cluster"][1]:+.4f}] | '
                  f'{s7["n10"]}/{s7["n01"]} p {s7["p_exact_two_sided"]:.4g} |')
    with open(os.path.join(a.out, 'analysis.json'), 'w') as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    with open(os.path.join(a.out, 'analysis.md'), 'w') as f:
        f.write('\n'.join(md) + '\n')
    print('\n'.join(md))


if __name__ == '__main__':
    main()
