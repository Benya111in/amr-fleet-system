#!/usr/bin/env python3
"""
B5 사전 점검 — Isaac 평가 집합(b1_eval_sets_v1.json, 풀 seed 1)을 2D 지도에서 그대로 쓸 수 있는가.

결과(성공률)를 보지 않는다. 지도 기하만 본다:
  1. 좌표계: 2D 지도(maps/warehouse.yaml, SLAM) 막힘 셀과 Isaac 기하(warehouse.sdf → 0.05 m 점유 격자) 의 상호 거리
  2. 집합 경로마다 2D 지도에서: 시작 풋프린트 여유(EDT), 경로 점 최소 EDT, 목표점 EDT,
     끝 접선 자세의 직진 여유 (Isaac 규칙과 같은 정의를 격자로 — 앞 변 점들을 1 cm 간격으로 밀며 첫 막힘 셀까지)
  3. 주(≥ 0.40 m)·스트레스 분할이 2D 지도에서도 같은가 (일치 표)
출력: --out 디렉터리에 mapcheck.json (요약), mapcheck_rows.npz (행별 값).
"""

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM2D = os.path.dirname(HERE)
RL = os.path.dirname(SIM2D)
sys.path.insert(0, SIM2D)
sys.path.insert(0, os.path.join(RL, 'isaac'))

from amr_sim2d.gridmap import GridMap  # noqa: E402
from amr_isaac import eval_sets as ES  # noqa: E402

HALF_L, HALF_W = 0.30, 0.20


def straight_clearance_grid(gm, pose, max_dist=5.0, step=0.005, lateral=0.01):
    """자세의 풋프린트를 앞으로 밀 때 첫 막힘 셀(셀 사각형)까지 거리 — 앞 변 점들의 행진 (격자 근사)."""
    x, y, th = pose
    c, s = np.cos(th), np.sin(th)
    ys = np.arange(-HALF_W, HALF_W + 1e-9, lateral)
    # 이미 겹침: 풋프린트 내부 점이 막힘 셀
    lx = np.arange(-HALF_L, HALF_L + 1e-9, 0.01)
    gx, gy = np.meshgrid(lx, ys)
    wx = x + c * gx - s * gy
    wy = y + s * gx + c * gy
    if _solid(gm, wx, wy).any():
        return 0.0
    d = np.arange(0.0, max_dist + step, step)
    fx = HALF_L + d[:, None]
    wx = x + c * fx - s * ys[None, :]
    wy = y + s * fx + c * ys[None, :]
    hit = _solid(gm, wx, wy).any(axis=1)
    k = np.nonzero(hit)[0]
    return float(d[k[0]]) if len(k) else float('inf')


def _solid(gm, wx, wy):
    mx, my = gm.world_to_cell(wx, wy)
    out = (mx < 0) | (my < 0) | (mx >= gm.W) | (my >= gm.H)
    inside = ~out
    res = out.copy()
    res[inside] = gm.solid[my[inside], mx[inside]] != 0
    return res


def _isaac_clear(pts, offs, obb, pids):
    """집합 행의 Isaac 직진 여유 (eval_sets.straight_approach_batch, 사각형 기하)."""
    poses = np.stack([ES.end_tangent_pose(pts[offs[p]:offs[p + 1]]) for p in pids])
    return ES.straight_approach_batch(poses, obb)


def edt_at(gm, xy):
    mx, my = gm.world_to_cell(xy[..., 0], xy[..., 1])
    ok = (mx >= 0) & (my >= 0) & (mx < gm.W) & (my < gm.H)
    out = np.zeros(mx.shape)
    out[ok] = gm.edt[my[ok], mx[ok]]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--eval-set', required=True)
    ap.add_argument('--pool', required=True, help='Isaac 풀 npz (paths_58eaa34ae5_s1.npz)')
    ap.add_argument('--geom', required=True, help='Isaac 기하 npz (warehouse_58eaa34ae5.npz)')
    ap.add_argument('--map', default=None, help='2D 지도 yaml (기본 maps/warehouse.yaml)')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    gm = GridMap.from_yaml(a.map) if a.map else GridMap.from_yaml()
    doc = ES.load_sets(a.eval_set)
    z = np.load(a.pool)
    pts, offs, length = z['pts'], z['offs'], z['length']
    ES.check_pool(doc, pts, offs, length)
    geom = np.load(a.geom)
    occ, o, r = geom['occ'], geom['origin'], float(geom['res'])
    from scipy.ndimage import distance_transform_edt
    # 1. 좌표계 대조
    ys, xs = np.nonzero(gm.solid)
    wx, wy = gm.cell_center(xs, ys)
    ix = np.floor((wx - o[0]) / r).astype(int)
    iy = np.floor((wy - o[1]) / r).astype(int)
    ok = (ix >= 0) & (iy >= 0) & (ix < occ.shape[1]) & (iy < occ.shape[0])
    d_sdf = distance_transform_edt(~occ) * r
    a2s = d_sdf[iy[ok], ix[ok]]
    oy_, ox_ = np.nonzero(occ)
    wx2, wy2 = o[0] + (ox_ + 0.5) * r, o[1] + (oy_ + 0.5) * r
    jx, jy = gm.world_to_cell(wx2, wy2)
    ok2 = (jx >= 0) & (jy >= 0) & (jx < gm.W) & (jy < gm.H)
    d_2d = distance_transform_edt(gm.solid == 0) * gm.res
    s2a = d_2d[jy[ok2], jx[ok2]]
    summary = {'map': gm.name, 'map_origin': [gm.ox, gm.oy], 'map_size_cells': [gm.W, gm.H],
               'map_res': gm.res, 'isaac_origin': o.tolist(),
               'isaac_size_cells': list(occ.shape[::-1]),
               'solid2d_to_isaac_occ_m': {'q50': float(np.quantile(a2s, 0.5)),
                                          'q99': float(np.quantile(a2s, 0.99)),
                                          'max': float(a2s.max()),
                                          'frac_gt_0.10': float((a2s > 0.10).mean()),
                                          'n': int(len(a2s))},
               'isaac_occ_to_solid2d_m': {'q50': float(np.quantile(s2a, 0.5)),
                                          'q99': float(np.quantile(s2a, 0.99)),
                                          'max': float(s2a.max()),
                                          'frac_gt_0.10': float((s2a > 0.10).mean()),
                                          'n_inside_2d_map': int(ok2.sum()),
                                          'n_total': int(len(ox_))}}
    # 2·3. 집합 행
    rows = {'set': [], 'pid': [], 'start_edt': [], 'path_min_edt': [], 'goal_edt': [],
            'clear2d': [], 'label_main': []}
    names = [s for s in doc['sets'] if not s.startswith('perturb')]
    for s in names:
        for pid in doc['sets'][s]:
            p = pts[offs[pid]:offs[pid + 1]]
            rows['set'].append(s)
            rows['pid'].append(int(pid))
            rows['start_edt'].append(float(edt_at(gm, p[0])))
            rows['path_min_edt'].append(float(edt_at(gm, p).min()))
            rows['goal_edt'].append(float(edt_at(gm, p[-1])))
            rows['clear2d'].append(straight_clearance_grid(gm, ES.end_tangent_pose(p)))
            rows['label_main'].append(s.startswith('main'))
    R = {k: np.asarray(v) for k, v in rows.items()}
    # Isaac 원 기하(사각형)로 잰 직진 여유 (집합을 나눈 값) — 2D 격자값과 비교
    obb = geom['obb']
    R['clear_isaac'] = _isaac_clear(pts, offs, obb, R['pid'])
    by = {}
    for s in names:
        m = R['set'] == s
        c2 = R['clear2d'][m]
        main2d = c2 >= ES.THRESHOLD
        by[s] = {'n': int(m.sum()),
                 'agree_split': float((main2d == R['label_main'][m]).mean()),
                 'main_in_2d': int(main2d.sum()),
                 'start_edt_min': float(R['start_edt'][m].min()),
                 'start_edt_lt_0.5': int((R['start_edt'][m] < 0.5).sum()),
                 'path_min_edt_min': float(R['path_min_edt'][m].min()),
                 'path_min_edt_lt_0.20': int((R['path_min_edt'][m] < 0.20).sum()),
                 'path_min_edt_lt_0.30': int((R['path_min_edt'][m] < 0.30).sum()),
                 'goal_edt_min': float(R['goal_edt'][m].min()),
                 'goal_edt_lt_0.36': int((R['goal_edt'][m] < 0.3606).sum()),
                 'clear2d_q': [float(np.quantile(np.where(np.isinf(c2), 99, c2), q))
                               for q in (0.0, 0.1, 0.5)]}
        ci = R['clear_isaac'][m]
        fin = np.isfinite(ci) & np.isfinite(c2)
        by[s]['clear2d_minus_isaac_m'] = (
            [float(np.quantile(c2[fin] - ci[fin], q)) for q in (0.05, 0.5, 0.95)] if fin.any()
            else None)
        by[s]['clear2d_zero'] = int((c2 == 0).sum())
    summary['sets'] = by
    np.savez_compressed(os.path.join(a.out, 'mapcheck_rows.npz'), **R)
    with open(os.path.join(a.out, 'mapcheck.json'), 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=1, ensure_ascii=False)
    print(json.dumps(summary, indent=1, ensure_ascii=False))


if __name__ == '__main__':
    main()
