#!/usr/bin/env python3
"""보행 차선 비용 마스크 생성 (Nav2 KeepoutFilter 용).

왜 필요한가
-----------
전역 계획이 보행 차선 **안을 따라** 달린다. A(-4,-5) → B(4,-9) 실측 계획 9.00 m 중
8.49 m (94 %) 가 어떤 보행 차선이든 몸 원통(로봇 0.361 + 사람 0.30 = 0.661 m) 안이고,
차선 밖 연속 구간이 0.25 m 짜리 두 개뿐이다. 그래서 로봇은 움직이면 회피 이득에 밀려
경로를 벗어나고(시나리오 07·08 이탈), 멈추면 작업자가 걸어 들어온다(08 접촉 9/60).
횡단 게이트를 켠 실측 g8a·g8b 가 이것을 그대로 보여 준다 — 로봇은 자기 경로 위
(dev_current 0.013~0.023)에 65 % 시간을 서 있었고, 그 자리가 차선에서 0.225 m 였다.

지역 계획기로는 풀 수 없다. 서 있을 곳이 경로 위에 없기 때문이다. 보행 차선을 통행
가능하되 비싼 구역으로 표시해 **전역 계획이 차선을 따라가지 않고 직각으로 건너게** 한다.

비용
----
마스크 점유값 90 → Nav2 표준 점유→비용 변환 `1 + 251·(v-1)/97` = **231**.
치명(254)도 외접(253)도 아니라 필요하면 통과할 수 있고, A* 간선 가중 κ=2.0 에서
같은 거리라면 차선 밖을 고른다. 차단이 아니라 기피다 — 목표가 차선 안에 있어도
계획이 실패하지 않는다.

`mode: scale` + `occupied_thresh 1.0` / `free_thresh 0.0` 으로 회색값이 그대로
점유값이 된다: 점유 = round((255 - 화소)/255 × 100). 화소 26 → 89.8 → 90.

차선 정의 (gen_warehouse_world.py 의 액터 궤적이 단일 출처)
-----------------------------------------------------------
worker_random   통로 격자 x∈{±3,±9,±15} × y∈{0,±6} 무작위 보행 (random_walk_nodes)
worker_crossing y=-7, x -6..6 횡단 (명세 4.7 시험 대상)
worker_slow_south y=-14.5, x -6..6
지게차·셔틀은 차량이고 통로 폭이 넓어 넣지 않는다 (사람과 달리 회피 규약이 다르다).

반폭 1.0 m 는 몸 원통 0.661 m 에 추적 가로 오차 p95 0.26 m 를 더한 값이다.
"""
from __future__ import annotations

import argparse
import os

import numpy as np
import yaml

# 보행 차선 (gen_warehouse_world.py 액터 궤적에서). (축, 중심, 시작, 끝)
LANES_H = [                      # 가로: y 고정, x 구간
    (-7.0, -6.0, 6.0),           # worker_crossing
    (-14.5, -6.0, 6.0),          # worker_slow_south
    (-6.0, -15.0, 15.0),         # worker_random 격자 행
    (0.0, -15.0, 15.0),
    (6.0, -15.0, 15.0),
]
LANES_V = [                      # 세로: x 고정, y 구간
    (-15.0, -6.0, 6.0), (-9.0, -6.0, 6.0), (-3.0, -6.0, 6.0),
    (3.0, -6.0, 6.0), (9.0, -6.0, 6.0), (15.0, -6.0, 6.0),
]
HALF_W = 1.0                     # [m] 반폭 = 몸 원통 0.661 + 추적 가로 오차 p95 0.26
OCC = 90                         # 점유값 → 비용 231
PIXEL = 26                       # (255-26)/255 × 100 = 89.8 → 90


def build(base_yaml: str, out_stem: str, half: float = HALF_W) -> None:
    with open(base_yaml, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    base_pgm = os.path.join(os.path.dirname(base_yaml), meta['image'])
    with open(base_pgm, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = (int(v) for v in line.split())

    img = np.full((h, w), 255, dtype=np.uint8)      # 255 = 자유
    # PGM 첫 행 = y 최대 → 행 r 의 월드 y = oy + (h - 1 - r) * res
    ys = oy + (h - 1 - np.arange(h)) * res
    xs = ox + np.arange(w) * res
    X, Y = np.meshgrid(xs, ys)
    for yc, x0, x1 in LANES_H:
        img[(np.abs(Y - yc) <= half) & (X >= x0 - half) & (X <= x1 + half)] = PIXEL
    for xc, y0, y1 in LANES_V:
        img[(np.abs(X - xc) <= half) & (Y >= y0 - half) & (Y <= y1 + half)] = PIXEL

    with open(out_stem + '.pgm', 'wb') as f:
        f.write(b'P5\n# amr: pedestrian lane cost mask (scripts/gen_pedestrian_lanes.py)\n')
        f.write(f'{w} {h}\n255\n'.encode())
        f.write(img.tobytes())
    with open(out_stem + '.yaml', 'w', encoding='utf-8') as f:
        f.write(
            f'image: {os.path.basename(out_stem)}.pgm\n'
            'mode: scale\n'                      # 회색값 → 점유값 (trinary 는 중간값을 미지로 만든다)
            f'resolution: {res}\n'
            f'origin: [{ox}, {oy}, 0]\n'
            'negate: 0\n'
            'occupied_thresh: 1.0\n'             # 전부 scale 구간으로 (0.65 면 90 이 100 으로 반올림)
            'free_thresh: 0.0\n'
            '# 보행 차선 비용 마스크 — scripts/gen_pedestrian_lanes.py 가 만든다 (손으로 고치지 말 것).\n'
            f'# 화소 {PIXEL} → 점유 {OCC} → Nav2 비용 231 (치명 254·외접 253 미만: 기피이지 차단이 아니다).\n'
            '# 차선 정의 출처: src/amr_simulation/worlds/gen_warehouse_world.py 의 액터 궤적.\n')
    painted = int((img == PIXEL).sum())
    print(f'{out_stem}.pgm  {w}x{h}  차선 셀 {painted} ({100 * painted / img.size:.1f} %)')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default='maps/warehouse.yaml', help='격자 규격을 따올 기준 지도')
    ap.add_argument('--out', default='maps/pedestrian_lanes', help='출력 stem (.pgm/.yaml)')
    ap.add_argument('--half-width', type=float, default=HALF_W)
    a = ap.parse_args()
    build(a.map, a.out, a.half_width)
