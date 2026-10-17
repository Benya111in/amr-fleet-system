#!/usr/bin/env python3
"""
움직이는 장애물 장면 M (paper_research prereg.md §8 A4) — held-out B 매끈 경로 앞 20 개에 등속 원판 하나씩.

    python3 scenes_moving.py --paths /runs/paths --out /runs/scenes_m/scenes_moving.json

규칙:
- held-out B 매끈 경로(heldout_smooth.json) 앞 20 개. 원판 반지름 0.3 m, 속도 u ~ U(0.5, 1.2) m/s, 비반응 등속.
- 교차점: 경로 호길이 비율 U(0.3, 0.7) 의 경로점 P_c (O 장면과 같은 범위). 원판은 P_c 에서 경로 법선 방향으로
  경로를 가로지른다 (방향 ±, 반반).
- 시각 맞춤 (prereg "로봇이 공칭 속도로 갈 때 교차점에 동시에 닿도록"): 로봇 공칭 도달 시각
  t_c = s_c / v_nom + v_nom/(2a) + a/(2j) (v_nom 1.0 m/s, a 1.0 m/s², j 2.0 m/s³ — 정지 출발 저크 제한 램프의
  시간 지연 0.75 s; 곡률 감속은 넣지 않는다). 원판은 로봇이 움직이기 시작한 순간(kinematic_sim_x
  obstacle_clock = motion)에 P_c − u·n̂·t_c 에서 출발해 t_c 에 P_c 를 지난다.
- 다시 뽑기: 원판 출발점이 로봇 출발점에서 2.0 m 안이거나, 원판 궤적(0–t_c + 15 s)이 목표점 1.0 m 안을 지나면.
- kinematic_sim_x 인자: moving_obstacles "x,y,vx,vy,0.3", obstacle_clock motion, obstacle_period 2000 s
  (반주기 1000 s — 실행 중 되돌아오지 않는다).
- 난수: numpy default_rng([seed, 77]) — O 장면(scenes_obstacles.py, default_rng(seed)) 과 다른 난수열.
"""
import argparse
import hashlib
import json
import math
import os

import numpy as np

N_SCENES = 20
R_OBS = 0.30
SPEED = (0.5, 1.2)
FRAC = (0.30, 0.70)
V_NOM, A_NOM, J_NOM = 1.0, 1.0, 2.0
PERIOD = 2000.0


def cumlen(xy):
    return np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1])))])


def point_at(xy, s, s_target):
    j = int(np.clip(np.searchsorted(s, s_target) - 1, 0, len(xy) - 2))
    u = (s_target - s[j]) / max(s[j + 1] - s[j], 1e-12)
    x = xy[j, 0] + u * (xy[j + 1, 0] - xy[j, 0])
    y = xy[j, 1] + u * (xy[j + 1, 1] - xy[j, 1])
    th = math.atan2(xy[j + 1, 1] - xy[j, 1], xy[j + 1, 0] - xy[j, 0])
    return x, y, th


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--paths', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=20261018)
    args = ap.parse_args()
    with open(os.path.join(args.paths, 'heldout_smooth.json'), encoding='utf-8') as f:
        smooth = json.load(f)
    rng = np.random.default_rng([args.seed, 77])
    out = []
    for k, e in enumerate(smooth[:N_SCENES]):
        xy = np.array([[p[0], p[1]] for p in e['points']])
        s = cumlen(xy)
        L = float(s[-1])
        tries = 0
        while True:
            tries += 1
            frac = float(rng.uniform(*FRAC))
            u = float(rng.uniform(*SPEED))
            sgn = 1.0 if rng.uniform() < 0.5 else -1.0
            sc = frac * L
            px, py, th = point_at(xy, s, sc)
            nx, ny = -math.sin(th) * sgn, math.cos(th) * sgn        # 원판 진행 방향
            tc = sc / V_NOM + V_NOM / (2 * A_NOM) + A_NOM / (2 * J_NOM)
            x0, y0 = px - u * nx * tc, py - u * ny * tc
            ts = np.linspace(0.0, tc + 15.0, 400)
            gx, gy = xy[-1]
            d_goal = np.min(np.hypot(x0 + u * nx * ts - gx, y0 + u * ny * ts - gy))
            d_start = math.hypot(x0 - xy[0, 0], y0 - xy[0, 1])
            if d_start >= 2.0 and d_goal >= 1.0:
                break
            if tries > 1000:
                raise RuntimeError(e['id'])
        out.append({
            'id': f'M{k + 1:02d}', 'family': 'moving', 'base': e['id'], 'points': e['points'],
            'obstacle': {'x0': x0, 'y0': y0, 'vx': u * nx, 'vy': u * ny, 'r': R_OBS, 'speed': u,
                         'cross_s': sc, 'cross_frac': frac, 'cross_xy': [px, py], 't_cross': tc,
                         'side': sgn, 'd_start': d_start, 'd_goal_min': float(d_goal)},
            'moving_obstacles': f'{x0:.4f},{y0:.4f},{u * nx:.4f},{u * ny:.4f},{R_OBS:.2f}',
            'obstacle_period': PERIOD, 'length': L, 'tries': tries})
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w', encoding='utf-8') as f:
        json.dump(out, f)
    with open(args.out, 'rb') as f:
        h = hashlib.sha256(f.read()).hexdigest()
    print(json.dumps({'file': args.out, 'sha256': h, 'seed': [args.seed, 77],
                      'scenes': [{k: o[k] for k in ('id', 'base', 'moving_obstacles', 'tries')}
                                 | {'t_cross': round(o['obstacle']['t_cross'], 2)}
                                 for o in out]}, indent=1))


if __name__ == '__main__':
    main()
