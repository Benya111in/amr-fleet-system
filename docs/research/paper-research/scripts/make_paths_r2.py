#!/usr/bin/env python3
"""
2 라운드 경로 집합 (prereg_r2.md §1). 1 라운드 사본 생성기 make_paths_x.py 를 모듈로 불러 쓰고 두 가지만 더한다.

1. **시드 의존 좁은 통로** (`--narrow-random`): 통로마다 폭 {0.60, 0.70}, 길이 {4, 6} m,
   진입 형태 {직선, 90° 우회전, 90° 좌회전, S 자}, 전역 경로 편향 부호 × U(0, 0.04) m 를 뽑는다.
   난수는 경로 생성 난수열과 따로 둔 numpy default_rng([seed, 1]) 이다. 그래서 같은 seed 면 매끈·A* 경로는
   make_paths_x 와 같다. 통로 자리(랙 x = 108, 중심 y = −24 … 18, 6 m 간격)와 경로·랙 기하 식은 원본 그대로다.
   이 옵션을 끄면 원본 고정 통로 8 개를 쓴다 — 출력이 make_paths_x 와 비트 단위로 같다 (확인: --check-r1).
2. **개수** (`--n-smooth --n-narrow --n-astar`): 튜닝-20 은 10/4/6, held-out C 는 32/8/20.
   A* 쌍은 n_astar + 4 개를 뽑는다 (원본 20 + 4 와 같은 꼴). 계획은 plan_astar_x.py 가 앞에서부터 n 개.

    python3 make_paths_r2.py --out /runs/r2/paths_c --seed 20261019 --narrow-random \\
        --n-smooth 32 --n-narrow 8 --n-astar 20
    python3 make_paths_r2.py --out /runs/r2/paths_tune --seed 20261020 --narrow-random \\
        --n-smooth 10 --n-narrow 4 --n-astar 6
    (ROS) plan_astar_x.py --pairs <out>/astar_pairs.json --out <out>/heldout_astar.json --n <n_astar>
    python3 make_paths_r2.py --out <out> --astar-scenes [--narrow-random --seed …]   (통로 상자를 장면에 붙이려면 같은 인자)

파일 이름은 1 라운드와 같다 (heldout_smooth.json·heldout_narrow.json·heldout_astar.json·scenes_heldout_*.txt·
composite.*). 집합마다 디렉터리가 다르고, 지도(composite)는 그 집합의 통로 랙을 담는다.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import make_paths_x as mp  # noqa: E402

WIDTHS = (0.60, 0.70)
LENGTHS = (4.0, 6.0)
ENTRIES = ('straight', 'turn_right', 'turn_left', 's_bend')


def random_cases(seed, n):
    """시드 의존 통로 사례 n 개 (id, 폭, 길이, 진입, 편향)."""
    rng = np.random.default_rng([seed, 1])
    out = []
    for i in range(n):
        w = float(rng.choice(WIDTHS))
        L = float(rng.choice(LENGTHS))
        entry = str(rng.choice(ENTRIES))
        bias = float((1.0 if rng.uniform() < 0.5 else -1.0) * rng.uniform(0.0, 0.04))
        out.append((f'N{i + 1}', w, L, entry, round(bias, 4)))
    return out


def narrow_set(cases):
    """make_paths_x.narrow_paths 를 주어진 사례로 돈다 (원본 함수가 전역 NARROW_CASES 를 읽으므로 잠시 바꾼다)."""
    saved = mp.NARROW_CASES
    mp.NARROW_CASES = list(cases)
    try:
        return mp.narrow_paths()
    finally:
        mp.NARROW_CASES = saved


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--narrow-random', action='store_true')
    ap.add_argument('--n-smooth', type=int, default=32)
    ap.add_argument('--n-narrow', type=int, default=8)
    ap.add_argument('--n-astar', type=int, default=20)
    ap.add_argument('--astar-scenes', action='store_true')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    if args.narrow_random:
        cases = random_cases(args.seed, args.n_narrow)
    else:
        cases = list(mp.NARROW_CASES[:args.n_narrow])
    nar, nboxes = narrow_set(cases)
    if args.astar_scenes:
        fj = os.path.join(args.out, 'heldout_astar.json')
        with open(fj, encoding='utf-8') as f:
            entries = json.load(f)
        for e in entries:
            e['boxes'] = mp.near_boxes(e['points'], mp.warehouse_boxes() + nboxes)
        fs = os.path.join(args.out, 'scenes_heldout_astar.txt')
        mp.write_scenes(fs, entries)
        man = os.path.join(args.out, 'manifest.json')
        with open(man, encoding='utf-8') as f:
            summary = json.load(f)
        summary['files']['heldout_astar.json'] = mp.sha256(fj)
        summary['files']['scenes_heldout_astar.txt'] = mp.sha256(fs)
        summary['astar_lengths'] = [round(e['meta']['length'], 2) for e in entries]
        with open(man, 'w', encoding='utf-8') as f:
            json.dump(summary, f, indent=1)
        print(json.dumps(summary['astar_lengths']))
        return
    rng = np.random.default_rng(args.seed)
    wboxes = mp.warehouse_boxes()
    all_boxes = wboxes + nboxes
    tun = [{'id': k, 'family': 'tuning', 'points': v, 'boxes': mp.near_boxes(v, all_boxes)}
           for k, v in mp.tuning_paths().items()]
    smo, tries = mp.smooth_paths(rng, args.n_smooth)
    smooth = [{'id': k, 'family': 'smooth', 'points': v['points'], 'meta': v['meta'],
               'boxes': mp.near_boxes(v['points'], all_boxes)} for k, v in smo.items()]
    narrow = [{'id': k, 'family': 'narrow', 'points': v['points'], 'meta': v['meta'],
               'boxes': mp.near_boxes(v['points'], all_boxes)} for k, v in nar.items()]
    pairs = mp.astar_pairs(rng, args.n_astar + 4)
    files = {}
    for name, entries in (('tuning', tun), ('heldout_smooth', smooth),
                          ('heldout_narrow', narrow)):
        fj = os.path.join(args.out, f'{name}.json')
        with open(fj, 'w', encoding='utf-8') as f:
            json.dump(entries, f)
        fs = os.path.join(args.out, f'scenes_{name}.txt')
        mp.write_scenes(fs, entries)
        files[os.path.basename(fj)] = mp.sha256(fj)
        files[os.path.basename(fs)] = mp.sha256(fs)
    fp = os.path.join(args.out, 'astar_pairs.json')
    with open(fp, 'w', encoding='utf-8') as f:
        json.dump({'pairs': pairs, 'boxes': wboxes}, f)
    files['astar_pairs.json'] = mp.sha256(fp)
    probe = [{'id': 'P_straight30', 'family': 'probe',
              'points': mp.straight_path(45.0, -27.0, 0.0, 30.0), 'boxes': []}]
    fpj = os.path.join(args.out, 'probe.json')
    with open(fpj, 'w', encoding='utf-8') as f:
        json.dump(probe, f)
    mp.write_scenes(os.path.join(args.out, 'scenes_probe.txt'), probe)
    files['probe.json'] = mp.sha256(fpj)
    files['scenes_probe.txt'] = mp.sha256(os.path.join(args.out, 'scenes_probe.txt'))
    grid = mp.composite_map(nboxes)
    yml = mp.warehouse_map.write_map(grid, os.path.join(args.out, 'composite'))
    files['composite.pgm'] = mp.sha256(os.path.join(args.out, 'composite.pgm'))
    files['composite.yaml'] = mp.sha256(yml)
    summary = {'seed': args.seed, 'smooth_tries': tries, 'files': files,
               'narrow_random': args.narrow_random, 'narrow_cases': cases,
               'counts': [args.n_smooth, args.n_narrow, args.n_astar],
               'smooth_lengths': [round(e['meta']['length'], 2) for e in smooth],
               'smooth_turn_rad': [round(e['meta']['total_turn_rad'], 2) for e in smooth]}
    with open(os.path.join(args.out, 'manifest.json'), 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=1)
    print(json.dumps(summary, indent=1))


if __name__ == '__main__':
    main()
