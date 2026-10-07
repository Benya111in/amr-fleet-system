#!/usr/bin/env python3
"""
지도 변형 평가 보충 분석 (b1-mapgen) — **사전 등록 외**, 결과를 본 뒤에 쓴 것이다. 판정에 쓰지 않는다.

    docker run --rm --network none -v $SRC:/ws:ro -v $B:$B:ro -v $R/analysis:$R/analysis \
        -v /usr/share/fonts/google-noto-cjk:/fonts:ro -w /ws amr-ai/rl:0.3 \
        python3 rl/isaac/scripts/supp_mapgen.py --out $R/analysis/supp_mapgen.json \
        --fig $R/analysis/figs/mapgen_conditions

  1. 시드 하나 빼기 — P1map Δ 가 한 시드에 기대는가.
  2. 조건별 Δ 의 목표 군집 부트스트랩 (시드 고정 짝) — 차이가 "어느 목표가 들었나" 로 설명되는가 (b_dr.md §9 와 같은 방식).
  3. 실패 꼴 — 실패 에피소드의 끝 자세가 (a) 목표 0.25 m 안 (정밀 정지), (b) 새 상자(add) 또는 바뀐 물체 가까이, (c) 그 밖.
     충돌 에피소드는 끝 풋프린트가 새 상자와 겹치는지 원래 물체와 겹치는지.
  4. 지도(타일)별 엄격 성공 — 실패가 특정 변형 지도에 몰리는가.
  5. 경로가 명목과 같은 행 / 바뀐 행의 엄격 성공 (조건마다).
  6. 조건별 짝 열화 (같은 행의 명목 − 변형 엄격 성공) — 사전 등록 S1 은 6 조건 평균만 판정한다.
"""

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC = os.path.dirname(HERE)
sys.path.insert(0, ISAAC)
sys.path.insert(0, os.path.join(os.path.dirname(ISAAC), 'eval'))

import numpy as np  # noqa: E402

from amr_isaac import map_bundle as MB  # noqa: E402
from amr_isaac import mapgen as MG  # noqa: E402
from amr_isaac import mapgen_stats as MS  # noqa: E402
from amr_isaac import warehouse_geom as W  # noqa: E402

B = '/home/CAPYI_AI/runs/bonus1'
HALF_L, HALF_W = 0.30, 0.20


def load_all(root, seeds):
    """두 팔 실행 → {'on': [(s, ev, ep)], 'off': …}."""
    out = {}
    for arm in ('on', 'off'):
        out[arm] = [(s,) + MS.load_run(os.path.join(root, f'mg_{arm}_s{s}')) for s in seeds]
    return out


def primary_mask(ev, ep, ci):
    """조건 ci 의 1 차 모집단 마스크."""
    si = ev['sets'].index('main_3_15')
    return (ep['cond_index'] == ci) & (ep['set_index'] == si) & ep['main_variant'].astype(bool) \
        & ep['got'].astype(bool)


def goal_cluster_paired(on, off, ci, reps=4000, seed=0):
    """시드 고정, 같은 행 짝 — 목표 군집을 다시 뽑은 Δ 의 95 % 구간."""
    ev0, ep0 = on[0][1], on[0][2]
    m = primary_mask(ev0, ep0, ci)
    rows = ep0['bundle_row'][m]
    groups = ep0['goal_group'][m]

    def succ(runs):
        mat = []
        for _s, ev, ep in runs:
            mm = primary_mask(ev, ep, ci)
            if not np.array_equal(ep['bundle_row'][mm], rows):
                raise ValueError('행 순서가 시드마다 다르다')
            mat.append((ep['outcome'][mm] == 0).astype(float))
        return np.mean(mat, axis=0)

    def mat(runs):
        return np.array([(ep['outcome'][primary_mask(ev, ep, ci)] == 0).astype(float)
                         for _s, ev, ep in runs])
    d = succ(on) - succ(off)
    _u, inv = np.unique(groups, return_inverse=True)
    k = inv.max() + 1
    s = np.bincount(inv, weights=d, minlength=k)
    c = np.bincount(inv, minlength=k).astype(float)
    rng = np.random.default_rng(seed)
    draw = rng.integers(0, k, (reps, k))
    boots = s[draw].sum(1) / c[draw].sum(1)
    lo, hi = np.percentile(boots, [2.5, 97.5])
    # 두 단계: 팔마다 시드를 복원 추출한 뒤 목표 군집을 다시 뽑는다 (b_dr.md §9 와 같은 방식)
    mo, mf = mat(on), mat(off)
    rng2 = np.random.default_rng(seed + 1)
    tb = np.empty(reps)
    for r in range(reps):
        dd = mo[rng2.integers(0, mo.shape[0], mo.shape[0])].mean(0) - \
            mf[rng2.integers(0, mf.shape[0], mf.shape[0])].mean(0)
        g = rng2.integers(0, k, k)
        ss = np.bincount(inv, weights=dd, minlength=k)
        tb[r] = ss[g].sum() / c[g].sum()
    tlo, thi = np.percentile(tb, [2.5, 97.5])
    return {'diff': float(d.mean()), 'paired95': [float(lo), float(hi)],
            'two_level95': [float(tlo), float(thi)], 'goals': int(k)}


def failure_modes(runs, bundle, ci):
    """조건 ci 의 1 차 모집단 실패 꼴 (팔 합)."""
    out = {'fail': 0, 'near_goal_0p25': 0, 'near_added_0p8': 0, 'near_changed_object_0p8': 0,
           'other': 0, 'other_within_0p8_of_any_obstacle': 0, 'collisions': 0,
           'collision_with_added': 0, 'collision_with_original': 0, 'by_outcome': {}}
    nom = MB.tile_rects(bundle, 0)[0]
    for _s, ev, ep in runs:
        m = primary_mask(ev, ep, ci) & (ep['outcome'] != 0)
        for i in np.flatnonzero(m):
            t = int(ep['tile'][i])
            obb, _zr, gid = MB.tile_rects(bundle, t)
            off = bundle['tile_offset'][t]
            pose = np.array([ep['pose_end'][i, 0] - off[0], ep['pose_end'][i, 1] - off[1],
                             ep['pose_end'][i, 2]])
            out['fail'] += 1
            added = obb[gid == MG.ADDED]
            changed = obb[(gid >= 0)]
            d_add = MG.rect_point_distance(added, pose[None, :2]).min() if len(added) else np.inf
            # 바뀐 물체: 명목과 다른 행 (지터·크기·제거로 움직인 물체)
            moved = np.array([not any(np.allclose(r, q) for q in nom) for r in changed]) \
                if len(changed) else np.zeros(0, bool)
            d_chg = MG.rect_point_distance(changed[moved], pose[None, :2]).min() \
                if moved.any() else np.inf
            d_any = MG.rect_point_distance(obb, pose[None, :2]).min()
            if ep['d_end'][i] <= 0.25:
                cat = 'near_goal_0p25'
            elif d_add <= 0.8:
                cat = 'near_added_0p8'
            elif d_chg <= 0.8:
                cat = 'near_changed_object_0p8'
            else:
                cat = 'other'
            out[cat] += 1
            oc = {1: 'collision', 2: 'time_limit', 3: 'stall'}[int(ep['outcome'][i])]
            out['by_outcome'][f'{cat}|{oc}'] = out['by_outcome'].get(f'{cat}|{oc}', 0) + 1
            if cat == 'other' and d_any <= 0.8:
                out['other_within_0p8_of_any_obstacle'] += 1
            if ep['outcome'][i] == 1:
                out['collisions'] += 1
                if len(added) and W.footprint_hits_obb(pose, added, HALF_L, HALF_W):
                    out['collision_with_added'] += 1
                elif W.footprint_hits_obb(pose, obb[gid != MG.ADDED], HALF_L, HALF_W):
                    out['collision_with_original'] += 1
    return out


def per_tile(runs, ci, bundle):
    """조건 ci 의 지도(타일)별 엄격 성공 (팔의 시드 평균)."""
    tiles = np.flatnonzero(bundle['tile_cond'] == ci)
    res = {}
    for t in tiles:
        v = []
        for _s, ev, ep in runs:
            m = primary_mask(ev, ep, ci) & (ep['tile'] == t)
            v.append(float((ep['outcome'][m] == 0).mean()) if m.any() else np.nan)
        res[int(t)] = {'strict_mean': float(np.nanmean(v)), 'per_seed': v,
                       'n': int((primary_mask(runs[0][1], runs[0][2], ci) &
                                 (runs[0][2]['tile'] == t)).sum())}
    return res


def changed_split(runs, ci, bundle):
    """경로가 바뀐 행과 그대로인 행의 엄격 성공 (팔의 시드 평균)."""
    out = {}
    for name, want in (('changed', True), ('unchanged', False)):
        v = []
        for _s, ev, ep in runs:
            m = primary_mask(ev, ep, ci)
            ch = bundle['row_changed'][ep['bundle_row'][m]]
            sel = np.flatnonzero(m)[ch == want]
            v.append(float((ep['outcome'][sel] == 0).mean()) if sel.size else np.nan)
        out[name] = {'strict_mean': float(np.nanmean(v)), 'per_seed': v}
    return out


def nominal_stress(runs):
    """명목 지도 stress_3_15 엄격 성공 (시드별)."""
    out = []
    for _s, ev, ep in runs:
        m = (ep['cond_index'] == -1) & (ep['set_index'] == ev['sets'].index('stress_3_15'))
        out.append(float((ep['outcome'][m] == 0).mean()))
    return out


def figure(met, conds, path, nom_stress):
    """조건별 엄격 성공 (1 차 모집단·스트레스 유효) — 점 = 시드, 선 = 팔 평균. 기술 그림."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    for fp in ('/fonts/NotoSansCJK-Regular.ttc',):
        if os.path.exists(fp):
            font_manager.fontManager.addfont(fp)
            plt.rcParams['font.family'] = font_manager.FontProperties(fname=fp).get_name()
    names = ['nominal'] + list(conds)
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6))
    for ax, pop, title in ((axes[0], 'primary', '주 3–15 m · 1 차 모집단 (변형 주)'),
                           (axes[1], 'stress', '스트레스 3–15 m · 유효 행 전부 (기술)')):
        for arm, col, dx in (('on', '#1f77b4', -0.12), ('off', '#d62728', 0.12)):
            ys = []
            for k, c in enumerate(names):
                if c == 'nominal':
                    vals = [m['nominal']['strict'] for m in met[arm]] if pop == 'primary' \
                        else nom_stress[arm]
                else:
                    vals = [m['cond'][c][pop]['strict'] for m in met[arm]]
                vals = [v for v in vals if v is not None]
                ys.append(np.mean(vals) if vals else np.nan)
                ax.scatter([k + dx] * len(vals), vals, s=14, color=col, alpha=0.6)
            ax.plot(np.arange(len(names)) + dx, ys, '-o', color=col, ms=4,
                    label='DR 켬 (B1 s0–4)' if arm == 'on' else 'DR 끔 (B4 s0–4)')
        ax.set_xticks(range(len(names)))
        ax.set_xticklabels(names, rotation=30, fontsize=9)
        ax.set_title(title, fontsize=10)
        ax.set_ylabel('엄격 성공 (시드 점)', fontsize=9)
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=9, loc='lower left')
    fig.suptitle('지도 변형 (b1_map_suite_v1, 조건마다 지도 8 장) — DR 켬 대 끔, 평가 seed 4242',
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(path + '.png', dpi=130)
    fig.savefig(path + '.svg')


def main():
    """보충 분석."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default=f'{B}/b1-mapgen/eval')
    ap.add_argument('--seeds', default='0,1,2,3,4')
    ap.add_argument('--bundle', default=f'{B}/b1-mapgen/suite/map_bundle_v1.npz')
    ap.add_argument('--out', required=True)
    ap.add_argument('--fig', default='')
    a = ap.parse_args()
    seeds = [int(x) for x in a.seeds.split(',')]
    runs = load_all(a.root, seeds)
    bundle = MB.load_bundle(a.bundle)
    conds = runs['on'][0][1]['conditions']
    res = {'note': '사전 등록 외 (결과를 본 뒤) — 판정에 쓰지 않는다', 'loso': {}, 'goal_cluster': {},
           'failure_modes': {}, 'per_tile': {}, 'changed_split': {}}
    met = {arm: [MS.seed_metrics(ev, ep, bundle) for _s, ev, ep in runs[arm]]
           for arm in ('on', 'off')}

    def p1(m):
        return float(np.mean([m['cond'][c]['primary']['strict'] for c in conds]))
    on1 = [p1(m) for m in met['on']]
    off1 = [p1(m) for m in met['off']]
    for arm, vals in (('on', on1), ('off', off1)):
        for k, s in enumerate(seeds):
            a_on = [x for j, x in enumerate(on1) if not (arm == 'on' and j == k)]
            a_off = [x for j, x in enumerate(off1) if not (arm == 'off' and j == k)]
            d, ci = MS.boot_diff(a_on, a_off)
            p, _n = MS.perm_p(a_on, a_off)
            res['loso'][f'drop_{arm}_s{s}'] = {'diff': d, 'boot95': ci, 'perm_p': p}
    res['matched_degradation'] = {}
    for c in conds:
        blk = {}
        for arm in ('on', 'off'):
            v = [m['cond'][c]['matched_nominal_strict'] - m['cond'][c]['primary']['strict']
                 for m in met[arm]]
            blk[arm] = {'mean': float(np.mean(v)), 'per_seed': [round(x, 6) for x in v]}
        d, ci95 = MS.boot_diff([x for x in blk['off']['per_seed']],
                               [x for x in blk['on']['per_seed']])
        p, _n = MS.perm_p(blk['off']['per_seed'], blk['on']['per_seed'])
        blk['off_minus_on'] = {'diff': d, 'boot95': ci95, 'perm_p': p}
        res['matched_degradation'][c] = blk
    for ci, c in enumerate(conds):
        res['goal_cluster'][c] = goal_cluster_paired(runs['on'], runs['off'], ci)
        res['failure_modes'][c] = {arm: failure_modes(runs[arm], bundle, ci)
                                   for arm in ('on', 'off')}
        res['per_tile'][c] = {arm: per_tile(runs[arm], ci, bundle) for arm in ('on', 'off')}
        res['changed_split'][c] = {arm: changed_split(runs[arm], ci, bundle)
                                   for arm in ('on', 'off')}
    if a.fig:
        figure(met, conds, a.fig, {arm: nominal_stress(runs[arm]) for arm in ('on', 'off')})
    with open(a.out, 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    print(json.dumps({'loso': {k: round(v['diff'], 4) for k, v in res['loso'].items()},
                      'goal_cluster': res['goal_cluster']}, ensure_ascii=False, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
