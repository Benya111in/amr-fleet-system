#!/usr/bin/env python3
"""
paper_research held-out B 경로 집합 (docs/research/paper-research/prereg.md §3.2).

원본 docs/research/dwa-ablation/scripts/make_paths.py 의 사본이다. **바꾼 것은 기본 seed
(20261014 → 20261018) 와 이 머리말뿐이다** — 생성기·분포·파일 이름은 그대로다. 그래서 튜닝 집합
(tuning.json)·합성 지도(composite.*)·좁은 통로 8 개(heldout_narrow.json, 무작위 요소 없음)는
held-out A 와 비트 단위로 같고, 매끈 32 개와 창고 A* 쌍만 새로 뽑힌다.

    python3 make_paths_x.py --out /runs/paths --seed 20261018
    (ROS) plan_astar_x.py → heldout_astar.json, 그 뒤 make_paths_x.py --out /runs/paths --astar-scenes

이하 원본 설명:

만드는 것
- tuning.json: 튜닝 집합 4 경로 (closed_loop_eval.py synthetic 배치와 같은 직선 20 m · U 턴 R 2 ·
  S 자 R 3 · 좁은 통로 0.60 m). 우리 DWA 를 튜닝한 경로라 최종 비교에 쓰지 않는다.
- heldout_smooth.json: 무작위 매끈한 경로 32 개 (고정 seed). 절반은 구간별 상수 곡률(원호·직선),
  절반은 구간별 선형 곡률(클로소이드). |κ| ≤ 0.7 1/m (R ≥ 1.43 m), 길이 8–24 m, 경로의 서로 먼
  (호길이 3 m 초과) 두 점이 2.0 m 안으로 오면 버린다. 빈 동쪽 마당에서 시작한다.
- heldout_narrow.json: 좁은 통로 8 개 (폭 0.60/0.70, 길이 4/6 m, 직선·90° 회전·S 자 진입,
  전역 경로 편향 0–4 cm). 동쪽 마당 x ≥ 104 의 랙 쌍.
- astar_pairs.json: 창고 지도 위 시작·목표 쌍 24 개 (둘 다 장애물 여유 ≥ 0.45 m, 직선거리 ≥ 5 m).
  계획은 ROS 계획기(AStarPlanner)로 따로 한다 (plan_astar.py → heldout_astar.json, 앞 20 개 사용).
- composite.pgm/.yaml: 합성 창고(60 × 40 m, warehouse_map.build_warehouse) + 동쪽 마당
  (x 30–130 m) + 좁은 통로 랙. 0.05 m. 운동학 시험대(kinematic_sim) 지도.
- scenes_*.txt: chain_sim 장면 파일 (경로 + 근처 장애물 상자).
"""
import argparse
import hashlib
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'src', 'amr_navigation'))
from amr_navigation import warehouse_map  # noqa: E402

STEP = 0.05
MAP_X0, MAP_X1, MAP_Y0, MAP_Y1 = -30.0, 130.0, -30.0, 30.0
SMOOTH_ORIGIN = (70.0, 0.0)


# --- closed_loop_eval.py reference_paths('synthetic') 와 같은 식 (튜닝 집합) ---
def straight_path(x0, y0, yaw, length, step=STEP):
    n = int(round(length / step))
    return [(x0 + i * step * math.cos(yaw), y0 + i * step * math.sin(yaw), yaw)
            for i in range(n + 1)]


def append_straight(pts, length, step=STEP):
    x, y, th = pts[-1]
    for i in range(1, int(round(length / step)) + 1):
        pts.append((x + i * step * math.cos(th), y + i * step * math.sin(th), th))


def append_arc(pts, radius, angle, step=STEP):
    x, y, th = pts[-1]
    sgn = 1.0 if angle >= 0 else -1.0
    n = int(round(abs(angle) * radius / step))
    for i in range(1, n + 1):
        a = th + sgn * i * step / radius
        pts.append((x + sgn * radius * (math.sin(a) - math.sin(th)),
                    y - sgn * radius * (math.cos(a) - math.cos(th)), a))


def tuning_paths():
    straight = straight_path(-10.0, 0.0, 0.0, 20.0)
    uturn = [(-26.0, -4.0, math.pi / 2)]
    append_straight(uturn, 3.0)
    append_arc(uturn, 2.0, -math.pi)
    append_straight(uturn, 3.0)
    scurve = [(-18.0, -6.5, 0.0)]
    for _ in range(2):
        append_arc(scurve, 3.0, math.pi / 6)
        append_arc(scurve, 3.0, -math.pi / 3)
        append_arc(scurve, 3.0, math.pi / 6)
    append_straight(scurve, 1.0)
    narrow = straight_path(0.0, -6.0, -math.pi / 2, 8.5)
    return {'T1_straight': straight, 'T2_uturn': uturn, 'T3_scurve': scurve,
            'T4_narrow': narrow}


# --- 무작위 매끈한 경로 ---
def integrate_kappa(kappa_fn, length, x0, y0, th0, step=STEP):
    """곡률 κ(s) 를 정확 원호로 적분 (step 마다 그 구간 중점 곡률)."""
    pts = [(x0, y0, th0)]
    n = int(round(length / step))
    x, y, th = x0, y0, th0
    for i in range(n):
        k = kappa_fn((i + 0.5) * step)
        if abs(k) < 1e-9:
            x, y = x + step * math.cos(th), y + step * math.sin(th)
        else:
            th1 = th + k * step
            x += (math.sin(th1) - math.sin(th)) / k
            y -= (math.cos(th1) - math.cos(th)) / k
            th = th1
        pts.append((x, y, th))
    return pts


def self_close(pts, sep_arc=3.0, min_dist=2.0):
    xy = np.array([[p[0], p[1]] for p in pts])
    s = np.arange(len(pts)) * STEP
    sub = xy[::4]
    ss = s[::4]
    d = np.hypot(sub[:, None, 0] - sub[None, :, 0], sub[:, None, 1] - sub[None, :, 1])
    far = np.abs(ss[:, None] - ss[None, :]) > sep_arc
    return bool(np.any(d[far] < min_dist))


def smooth_paths(rng, n=32):
    out = {}
    i = 0
    tries = 0
    while len(out) < n:
        tries += 1
        kind = 'pwc' if i % 2 == 0 else 'pwl'      # 구간별 상수 / 구간별 선형 곡률
        length = float(rng.uniform(8.0, 24.0))
        knots_s = [0.0]
        while knots_s[-1] < length:
            knots_s.append(knots_s[-1] + float(rng.uniform(1.5, 3.0)))
        vals = []
        for _ in knots_s:
            if rng.uniform() < 0.3:
                vals.append(0.0)
            else:
                vals.append(float(rng.choice([-1.0, 1.0]) * rng.uniform(0.1, 0.7)))
        if kind == 'pwc':
            def kfn(s, ks=knots_s, v=vals):
                j = max(0, int(np.searchsorted(ks, s, side='right')) - 1)
                return v[j]
        else:
            def kfn(s, ks=knots_s, v=vals):
                return float(np.interp(s, ks, v))
        th0 = float(rng.uniform(-math.pi, math.pi))
        pts = integrate_kappa(kfn, length, SMOOTH_ORIGIN[0], SMOOTH_ORIGIN[1], th0)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        if self_close(pts) or min(xs) < 45.0 or max(xs) > 100.0 or abs(min(ys)) > 27.0 \
                or abs(max(ys)) > 27.0:
            continue
        turn = sum(abs(kfn((j + 0.5) * STEP)) * STEP for j in range(int(round(length / STEP))))
        out[f'S{i + 1:02d}_{kind}'] = {'points': pts, 'meta': {
            'kind': kind, 'length': length, 'knots': knots_s, 'kappa': vals,
            'total_turn_rad': turn, 'theta0': th0}}
        i += 1
    return out, tries


# --- 좁은 통로 ---
NARROW_CASES = [
    # id, 폭, 길이, 진입, 편향
    ('N1', 0.60, 4.0, 'straight', 0.00),
    ('N2', 0.60, 4.0, 'straight', 0.03),
    ('N3', 0.60, 6.0, 'straight', -0.02),
    ('N4', 0.70, 4.0, 'straight', 0.00),
    ('N5', 0.70, 6.0, 'straight', 0.04),
    ('N6', 0.60, 4.0, 'turn_right', 0.00),
    ('N7', 0.70, 4.0, 'turn_left', 0.03),
    ('N8', 0.60, 4.0, 's_bend', 0.02),
]
NARROW_AX = 108.0
NARROW_CY = [-24.0, -18.0, -12.0, -6.0, 0.0, 6.0, 12.0, 18.0]


def narrow_paths():
    out = {}
    boxes_all = []
    for (cid, width, length, entry, bias), cy in zip(NARROW_CASES, NARROW_CY):
        ax = NARROW_AX
        racks = [(ax, cy + width / 2, ax + length, cy + width / 2 + 1.0),
                 (ax, cy - width / 2 - 1.0, ax + length, cy - width / 2)]
        boxes_all += racks
        r = 1.5
        if entry == 'straight':
            pts = straight_path(ax - 3.0, cy, 0.0, 3.0)
        elif entry == 'turn_right':
            pts = [(ax - 1.0 - r, cy - r - 2.0, math.pi / 2)]
            append_straight(pts, 2.0)
            append_arc(pts, r, -math.pi / 2)
            append_straight(pts, 1.0)
        elif entry == 'turn_left':
            pts = [(ax - 1.0 - r, cy + r + 2.0, -math.pi / 2)]
            append_straight(pts, 2.0)
            append_arc(pts, r, math.pi / 2)
            append_straight(pts, 1.0)
        else:   # s_bend: 1.5 m 직선(옆 0.5 m) → 3 m 코사인 S → 통로
            d, lb = 0.5, 3.0
            pts = straight_path(ax - lb - 1.5, cy - d, 0.0, 1.5)
            n = int(round(lb / STEP))
            x0 = ax - lb
            for i in range(1, n + 1):
                x = x0 + i * STEP
                u = (x - x0) / lb
                y = cy - d / 2 * (1 + math.cos(math.pi * u))
                dy = d / 2 * math.pi / lb * math.sin(math.pi * u)
                pts.append((x, y, math.atan2(dy, 1.0)))
        append_straight(pts, length + 1.5)
        pts = [(x, y + bias, th) for x, y, th in pts]
        out[cid] = {'points': pts, 'meta': {'width': width, 'aisle_length': length,
                                            'entry': entry, 'bias': bias,
                                            'aisle': [ax, cy, ax + length]},
                    'racks': racks}
    return out, boxes_all


# --- 지도·상자 ---
def warehouse_boxes():
    """창고 정적 상자 + 외벽(두께 0.2) 을 축정렬 상자 목록으로 (x0, y0, x1, y1)."""
    hx, hy, t = warehouse_map.HALF_X, warehouse_map.HALF_Y, warehouse_map.WALL_T
    walls = [(-hx, -hy, hx, -hy + t), (-hx, hy - t, hx, hy), (-hx, -hy, -hx + t, hy),
             (hx - t, -hy, hx, hy)]
    return list(warehouse_map.static_boxes()) + walls


def composite_map(narrow_boxes, res=0.05):
    w = int(round((MAP_X1 - MAP_X0) / res))
    h = int(round((MAP_Y1 - MAP_Y0) / res))
    occ = np.zeros((h, w), dtype=bool)
    wh = warehouse_map.build_warehouse(res)
    ix0 = int(round((wh.origin[0] - MAP_X0) / res))
    iy0 = int(round((wh.origin[1] - MAP_Y0) / res))
    occ[iy0:iy0 + wh.height, ix0:ix0 + wh.width] = wh.occ
    xs = MAP_X0 + (np.arange(w) + 0.5) * res
    ys = MAP_Y0 + (np.arange(h) + 0.5) * res
    eps = 1e-9
    for x0, y0, x1, y1 in narrow_boxes:
        cols = (xs >= x0 - eps) & (xs <= x1 + eps)
        rows = (ys >= y0 - eps) & (ys <= y1 + eps)
        occ[np.ix_(rows, cols)] = True
    return warehouse_map.GridMap(occ=occ, resolution=res, origin=(MAP_X0, MAP_Y0))


def near_boxes(pts, boxes, margin=6.0):
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    bx0, bx1, by0, by1 = min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin
    return [b for b in boxes if b[2] >= bx0 and b[0] <= bx1 and b[3] >= by0 and b[1] <= by1]


def astar_pairs(rng, n=24):
    wh = warehouse_map.build_warehouse(0.05)
    dist = warehouse_map.distance_field(wh)
    pairs = []
    while len(pairs) < n:
        pts = []
        while len(pts) < 2:
            x = float(rng.uniform(-29.0, 29.0))
            y = float(rng.uniform(-19.0, 19.0))
            ix, iy = wh.world_to_cell(x, y)
            # 풋프린트 외접 반경 0.361 + 0.45 여유 → 셀 중심 거리 ≥ 0.81 m
            if wh.in_bounds(ix, iy) and dist[iy, ix] >= 0.81:
                pts.append((round(x, 2), round(y, 2)))
        if math.hypot(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]) < 5.0:
            continue
        pairs.append({'id': f'A{len(pairs) + 1:02d}', 'start': pts[0], 'goal': pts[1]})
    return pairs


def write_scenes(fname, entries):
    with open(fname, 'w', encoding='utf-8') as f:
        for e in entries:
            f.write(f"scene {e['id']} {e['family']}\npath {len(e['points'])}\n")
            for x, y, th in e['points']:
                f.write(f'{x:.6f} {y:.6f} {th:.6f}\n')
            f.write(f"boxes {len(e['boxes'])}\n")
            for b in e['boxes']:
                f.write(' '.join(f'{v:.6f}' for v in b) + '\n')
            f.write('end\n')


def sha256(fname):
    with open(fname, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=20261018)
    ap.add_argument('--astar-scenes', action='store_true',
                    help='plan_astar.py 가 만든 heldout_astar.json 에 상자를 붙여 장면 파일만 쓴다')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    if args.astar_scenes:
        nar, nboxes = narrow_paths()
        fj = os.path.join(args.out, 'heldout_astar.json')
        with open(fj, encoding='utf-8') as f:
            entries = json.load(f)
        for e in entries:
            e['boxes'] = near_boxes(e['points'], warehouse_boxes() + nboxes)
        fs = os.path.join(args.out, 'scenes_heldout_astar.txt')
        write_scenes(fs, entries)
        man = os.path.join(args.out, 'manifest.json')
        with open(man, encoding='utf-8') as f:
            summary = json.load(f)
        summary['files']['heldout_astar.json'] = sha256(fj)
        summary['files']['scenes_heldout_astar.txt'] = sha256(fs)
        summary['astar_lengths'] = [round(e['meta']['length'], 2) for e in entries]
        with open(man, 'w', encoding='utf-8') as f:
            json.dump(summary, f, indent=1)
        print(json.dumps(summary['astar_lengths']))
        return
    rng = np.random.default_rng(args.seed)
    wboxes = warehouse_boxes()
    nar, nboxes = narrow_paths()
    all_boxes = wboxes + nboxes

    tun = [{'id': k, 'family': 'tuning', 'points': v, 'boxes': near_boxes(v, all_boxes)}
           for k, v in tuning_paths().items()]
    smo, tries = smooth_paths(rng)
    smooth = [{'id': k, 'family': 'smooth', 'points': v['points'], 'meta': v['meta'],
               'boxes': near_boxes(v['points'], all_boxes)} for k, v in smo.items()]
    narrow = [{'id': k, 'family': 'narrow', 'points': v['points'], 'meta': v['meta'],
               'boxes': near_boxes(v['points'], all_boxes)} for k, v in nar.items()]
    pairs = astar_pairs(rng)
    files = {}
    for name, entries in (('tuning', tun), ('heldout_smooth', smooth),
                          ('heldout_narrow', narrow)):
        fj = os.path.join(args.out, f'{name}.json')
        with open(fj, 'w', encoding='utf-8') as f:
            json.dump(entries, f)
        fs = os.path.join(args.out, f'scenes_{name}.txt')
        write_scenes(fs, entries)
        files[os.path.basename(fj)] = sha256(fj)
        files[os.path.basename(fs)] = sha256(fs)
    fp = os.path.join(args.out, 'astar_pairs.json')
    with open(fp, 'w', encoding='utf-8') as f:
        json.dump({'pairs': pairs, 'boxes': wboxes}, f)
    files['astar_pairs.json'] = sha256(fp)
    # 기전 탐침 (표 3, held-out 아님): 빈 마당 직선 30 m — 정지에서 가속하는 a_eff 를 잰다
    probe = [{'id': 'P_straight30', 'family': 'probe',
              'points': straight_path(45.0, -27.0, 0.0, 30.0), 'boxes': []}]
    fpj = os.path.join(args.out, 'probe.json')
    with open(fpj, 'w', encoding='utf-8') as f:
        json.dump(probe, f)
    write_scenes(os.path.join(args.out, 'scenes_probe.txt'), probe)
    files['probe.json'] = sha256(fpj)
    files['scenes_probe.txt'] = sha256(os.path.join(args.out, 'scenes_probe.txt'))
    grid = composite_map(nboxes)
    yml = warehouse_map.write_map(grid, os.path.join(args.out, 'composite'))
    files['composite.pgm'] = sha256(os.path.join(args.out, 'composite.pgm'))
    files['composite.yaml'] = sha256(yml)
    summary = {'seed': args.seed, 'smooth_tries': tries, 'files': files,
               'smooth_lengths': [round(e['meta']['length'], 2) for e in smooth],
               'smooth_turn_rad': [round(e['meta']['total_turn_rad'], 2) for e in smooth]}
    with open(os.path.join(args.out, 'manifest.json'), 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=1)
    print(json.dumps(summary, indent=1))


if __name__ == '__main__':
    main()
