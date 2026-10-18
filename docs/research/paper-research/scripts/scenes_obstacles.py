#!/usr/bin/env python3
"""
정적 미지 장애물 장면 O (paper_research prereg.md §3.2) — held-out B 매끈 경로 앞 20 개에 원판 하나씩.

    python3 scenes_obstacles.py --paths /runs/paths --seed 20261018

규칙 (prereg §3.2 그대로):
- held-out B 매끈 경로(heldout_smooth.json) 가운데 **앞 20 개**.
- 경로 호길이 30–70 % 구간 U(0.3, 0.7)·L 의 경로점에서, 경로 법선 방향 U(−0.1, +0.1) m 떨어진 곳에
  지도에 없는 원판 하나, 반지름 U(0.25, 0.40) m.
- 원판 바깥으로 로봇이 지날 여유(≥ 0.8 m)가 없으면 다시 뽑는다. 판정: 원판 좌·우 법선 방향으로
  원판 가장자리에서 0.8 m 폭의 띠(중심 = 가장자리 + 0.4 m, 반경 0.4 m 원)가 지도 점유 셀과 겹치지 않는
  쪽이 하나라도 있어야 한다 (합성 지도 composite.yaml, 0.05 m). 동쪽 마당은 비어 있어 보통 첫 번에 된다.
- 뽑는 순서: 장면마다 (호길이 비율, 반지름, 법선 오프셋) 를 차례로, 다시 뽑으면 같은 순서로 다시.
  난수 = numpy default_rng(seed). seed 는 held-out B 와 같은 20261018 이지만 make_paths_x 와는 다른
  생성기 객체라 경로 생성 난수열과 섞이지 않는다.
- kinematic_sim 의 moving_obstacles 파라미터에 속도 0 으로 넣는다: "x,y,0,0,r" (LiDAR 에 보이고,
  참값 트랙 토픽은 하네스가 모든 팔에서 차단한다 — params_x.py).

출력 scenes_obstacles.json: ros_bench_x.py 가 그대로 읽는 경로 목록 형식(id, family='obstacle', points)에
obstacle {x, y, r, s, frac, offset}, moving_obstacles 문자열, 원래 경로 id(base), 다시 뽑은 횟수를 더한다.
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

N_SCENES = 20
FRAC = (0.30, 0.70)
RADIUS = (0.25, 0.40)
OFFSET = 0.10
PASS_ROOM = 0.80


def cumlen(xy):
    return np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1])))])


def point_at(pts, s_target):
    """호길이 s 의 경로점 (선형 보간) 과 접선 각."""
    xy = np.array([[p[0], p[1]] for p in pts])
    s = cumlen(xy)
    j = int(np.clip(np.searchsorted(s, s_target) - 1, 0, len(xy) - 2))
    u = (s_target - s[j]) / max(s[j + 1] - s[j], 1e-12)
    x = xy[j, 0] + u * (xy[j + 1, 0] - xy[j, 0])
    y = xy[j, 1] + u * (xy[j + 1, 1] - xy[j, 1])
    th = math.atan2(xy[j + 1, 1] - xy[j, 1], xy[j + 1, 0] - xy[j, 0])
    return x, y, th, float(s[-1])


def disc_free(grid, cx, cy, rad):
    """반경 rad 원이 점유 셀과 겹치지 않는가 (셀 중심 기준, 반 셀 여유)."""
    res = grid.resolution
    k = int(math.ceil((rad + res) / res))
    ix0, iy0 = grid.world_to_cell(cx, cy)
    for iy in range(iy0 - k, iy0 + k + 1):
        for ix in range(ix0 - k, ix0 + k + 1):
            if not grid.in_bounds(ix, iy):
                return False
            if grid.occ[iy, ix]:
                x, y = grid.cell_center(ix, iy)
                if math.hypot(x - cx, y - cy) <= rad + 0.5 * res:
                    return False
    return True


def room_ok(grid, ox, oy, r, th):
    nx, ny = -math.sin(th), math.cos(th)
    sides = []
    for sgn in (1.0, -1.0):
        c = r + 0.5 * PASS_ROOM
        sides.append(disc_free(grid, ox + sgn * c * nx, oy + sgn * c * ny, 0.5 * PASS_ROOM))
    return any(sides), sides


def sha256(fname):
    with open(fname, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--paths', required=True, help='heldout_smooth.json·composite.yaml 디렉터리')
    ap.add_argument('--seed', type=int, default=20261018)
    ap.add_argument('--out', default='')
    ap.add_argument('--n', type=int, default=N_SCENES,
                    help='앞에서부터 쓸 매끈 경로 수 (1 라운드 20; 2 라운드 튜닝 O 는 6)')
    args = ap.parse_args()
    with open(os.path.join(args.paths, 'heldout_smooth.json'), encoding='utf-8') as f:
        smooth = json.load(f)
    grid = warehouse_map.load_map(os.path.join(args.paths, 'composite.yaml'))
    rng = np.random.default_rng(args.seed)
    out = []
    for k, e in enumerate(smooth[:args.n]):
        tries = 0
        while True:
            tries += 1
            frac = float(rng.uniform(*FRAC))
            r = float(rng.uniform(*RADIUS))
            off = float(rng.uniform(-OFFSET, OFFSET))
            L = cumlen(np.array([[p[0], p[1]] for p in e['points']]))[-1]
            px, py, th, _ = point_at(e['points'], frac * L)
            ox, oy = px - off * math.sin(th), py + off * math.cos(th)
            ok, sides = room_ok(grid, ox, oy, r, th)
            if ok and disc_free(grid, ox, oy, r):
                break
            if tries > 1000:
                raise RuntimeError(f'{e["id"]}: 배치 실패')
        out.append({
            'id': f'O{k + 1:02d}', 'family': 'obstacle', 'base': e['id'], 'points': e['points'],
            'obstacle': {'x': ox, 'y': oy, 'r': r, 's': frac * L, 'frac': frac, 'offset': off,
                         'path_heading': th, 'room_left': sides[0], 'room_right': sides[1]},
            'moving_obstacles': f'{ox:.4f},{oy:.4f},0.0,0.0,{r:.4f}',
            'bench_obstacles': f'{ox:.4f},{oy:.4f},{r:.4f}',
            'length': L, 'tries': tries})
    fn = args.out or os.path.join(args.paths, 'scenes_obstacles.json')
    with open(fn, 'w', encoding='utf-8') as f:
        json.dump(out, f)
    print(json.dumps({'file': fn, 'sha256': sha256(fn), 'seed': args.seed,
                      'scenes': [{k: o[k] for k in ('id', 'base', 'moving_obstacles', 'tries')}
                                 for o in out]}, indent=1))


if __name__ == '__main__':
    main()
