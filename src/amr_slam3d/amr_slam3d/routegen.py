"""
녹화 시나리오 경로 생성·검사 (rclpy 비의존) — 보너스 2 G 단계 (docs/research/slam3d/recordings.md).

창고(gen_warehouse_world.py, 60 x 40 m)의 트인 차선만 잇는 차선 그래프 위에서 경로를 만든다.
  가로 차선 y: 13.3 (북쪽 주 통로), 6 (통로 AB), 0 (통로 BC), −6 (열 C 남쪽), −15 (남쪽 홀)
  세로 차선 x: ±21.5 (랙 열 끝 바깥, 기둥 (±25, ±6) 안쪽) — 모든 가로 차선을 잇는다
               ±15, ±9, ±3 (랙 사이 4 m 틈 중앙) — y −6 ~ 13.3 (열 C·B·A 를 가로지른다)
               ±15, ±3 은 y −15 ~ −6 도 잇는다 (±9 는 기둥 (±10, −12) 에 0.75 m 까지 붙어서 뺀다)
  출발 마디 (18, −15): 스폰 (18, −16, π/2) 바로 북쪽. (15, −15)·(21.5, −15) 와 잇는다.
S1 경로 = 씨드 난수 걸음 (직전 마디로 되돌아가지 않음) 이 길이 L 에 닿으면 출발 마디로 가는 최단 경로를 붙인다 —
끝에서 출발 자리로 돌아오므로 루프 폐쇄 기회가 생긴다. 일직선으로 이어지는 마디는 합친다 (회전·정지 지점만 남긴다).
검사 (check_clearance): 정답 높이 대역 래스터(0.05–2.0 m)에서 각 직진 선분의 중심선이 장애물에서 open_r 이상, 회전 지점
원(rot_r)이 비어 있는지. 좁은 통로 안 선분은 aisle_r 로 따로 본다.
"""

from __future__ import annotations

import heapq
import math
import random
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

Point = Tuple[float, float]

H_LANES = (13.3, 6.0, 0.0, -6.0, -15.0)
END_X = (-21.5, 21.5)
GAP_X = (-15.0, -9.0, -3.0, 3.0, 9.0, 15.0)
SOUTH_GAP_X = (-15.0, -3.0, 3.0, 15.0)
START_NODE: Point = (18.0, -15.0)
SPAWN = (18.0, -16.0, math.pi / 2)
#: 좁은 통로 (gen_warehouse_world.py NARROW_*: 중심 (0, −10), 순폭 0.60, y −12 ~ −8)
AISLE_X = (-0.3, 0.3)
AISLE_Y = (-12.0, -8.0)


def _key(p: Point) -> Point:
    return (round(p[0], 3), round(p[1], 3))


def lane_graph() -> Dict[Point, List[Point]]:
    """차선 그래프: 마디 → 이웃 마디 (가로·세로 차선의 이웃 교차점끼리, 무향)."""
    vert = {x: (-15.0, 13.3) for x in END_X}
    for x in GAP_X:
        vert[x] = (-6.0, 13.3) if x not in SOUTH_GAP_X else (-15.0, 13.3)
    nodes = set()
    for y in H_LANES:
        for x, (y0, y1) in vert.items():
            if y0 - 1e-9 <= y <= y1 + 1e-9:
                nodes.add(_key((x, y)))
    nodes.add(_key(START_NODE))
    adj: Dict[Point, List[Point]] = {n: [] for n in nodes}

    def link(a: Point, b: Point) -> None:
        if b not in adj[a]:
            adj[a].append(b)
            adj[b].append(a)
    for y in H_LANES:
        row = sorted(n for n in nodes if abs(n[1] - y) < 1e-9)
        for a, b in zip(row, row[1:]):
            link(a, b)
    for x in vert:
        col = sorted((n for n in nodes if abs(n[0] - x) < 1e-9), key=lambda n: n[1])
        for a, b in zip(col, col[1:]):
            link(a, b)
    for n in adj:
        adj[n].sort()
    return adj


def _dist(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def shortest_path(adj: Dict[Point, List[Point]], a: Point, b: Point) -> List[Point]:
    """다익스트라 (마디 a → b, 양끝 포함). 동률은 마디 좌표 순으로 결정적."""
    best = {a: 0.0}
    prev: Dict[Point, Optional[Point]] = {a: None}
    q = [(0.0, a)]
    while q:
        d, n = heapq.heappop(q)
        if n == b:
            break
        if d > best[n] + 1e-12:
            continue
        for m in adj[n]:
            nd = d + _dist(n, m)
            if nd < best.get(m, math.inf) - 1e-12:
                best[m], prev[m] = nd, n
                heapq.heappush(q, (nd, m))
    if b not in prev:
        raise ValueError(f'{a} → {b} 경로 없음')
    out = [b]
    while prev[out[-1]] is not None:
        out.append(prev[out[-1]])
    return out[::-1]


def random_walk(seed: int, length_m: float, start: Point = START_NODE,
                adj: Optional[Dict[Point, List[Point]]] = None) -> List[Point]:
    """
    씨드 난수 걸음 (직전 마디로 되돌아가지 않는다, 막다른 곳만 예외) — 길이 ≥ length_m 이 되면 출발 마디로 돌아간다.

    반환: 마디 열 (처음·끝 = start).
    """
    adj = adj or lane_graph()
    rng = random.Random(seed)
    path = [_key(start)]
    total = 0.0
    while total < length_m:
        cur = path[-1]
        choices = [m for m in adj[cur] if len(path) < 2 or m != path[-2]] or adj[cur]
        nxt = rng.choice(choices)
        total += _dist(cur, nxt)
        path.append(nxt)
    back = shortest_path(adj, path[-1], _key(start))
    return path + back[1:]


def merge_collinear(path: Sequence[Point], tol: float = 1e-6) -> List[Point]:
    """일직선으로 이어지는 중간 마디를 뺀다 (같은 마디 반복도 뺀다). 되돌아가는 꺾임(180°)은 남긴다."""
    pts: List[Point] = []
    for p in path:
        if pts and _dist(pts[-1], p) < tol:
            continue
        pts.append((float(p[0]), float(p[1])))
    out = pts[:2]
    for p in pts[2:]:
        a, b = out[-2], out[-1]
        cross = (b[0] - a[0]) * (p[1] - b[1]) - (b[1] - a[1]) * (p[0] - b[0])
        dot = (b[0] - a[0]) * (p[0] - b[0]) + (b[1] - a[1]) * (p[1] - b[1])
        if abs(cross) < tol and dot > 0:
            out[-1] = p
        else:
            out.append(p)
    return out


def path_length(points: Sequence[Point]) -> float:
    """꺾은선 길이."""
    return float(sum(_dist(a, b) for a, b in zip(points, points[1:])))


def s1_waypoints(seed: int, length_m: float = 60.0) -> List[Point]:
    """S1 경로 웨이포인트 (스폰에서 출발 마디로 가는 첫 선분 포함, 일직선 합침)."""
    walk = random_walk(seed, length_m)
    return merge_collinear([SPAWN[:2]] + walk)[1:]


def map_waypoints() -> List[Point]:
    """
    긴 매핑 경로 (MAP) 웨이포인트.

    남쪽 홀 → 열 C 남쪽 → 통로 BC → 통로 AB → 북쪽 주 통로를 지그재그로 덮고, 서쪽 끝 차선과 남쪽 홀을 되짚어
    출발 자리로 돌아온다 (slam.md §6.1 의 322 m 경로와 같은 차선들 — 원래 웨이포인트 파일은 저장소에 없다).
    """
    w, e = END_X
    return [START_NODE, (w, -15.0), (w, -6.0), (e, -6.0), (e, 0.0), (w, 0.0), (w, 6.0), (e, 6.0),
            (e, 13.3), (w, 13.3), (w, -15.0), START_NODE]


def in_aisle(x, y, ext: float = 0.0):
    """점(또는 배열)이 좁은 통로 상자(순폭 x ±0.3, y −12 ~ −8)를 y 로 ext 만큼 넓힌 곳 안인가."""
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    return ((x >= AISLE_X[0] - 1e-9) & (x <= AISLE_X[1] + 1e-9)
            & (y >= AISLE_Y[0] - ext) & (y <= AISLE_Y[1] + ext))


def _samples(a: Point, b: Point, step: float = 0.02) -> Tuple[np.ndarray, np.ndarray]:
    n = max(2, int(math.ceil(_dist(a, b) / step)) + 1)
    return np.linspace(a[0], b[0], n), np.linspace(a[1], b[1], n)


def _lookup(field: np.ndarray, res: float, origin: Point, xs: np.ndarray,
            ys: np.ndarray) -> np.ndarray:
    col = np.floor((xs - origin[0]) / res).astype(int)
    row = np.floor((ys - origin[1]) / res).astype(int)
    ok = (row >= 0) & (row < field.shape[0]) & (col >= 0) & (col < field.shape[1])
    d = np.zeros(len(xs))
    d[ok] = field[row[ok], col[ok]]
    return d


#: 통로 상자를 y 로 넓히는 폭 — 랙 모서리 (±0.3, −8) 에서 open_r 0.66 안에 드는 중심선 구간 (√(0.66² − 0.3²) = 0.59)
AISLE_EXT = 0.6


def check_clearance(start: Point, waypoints: Sequence[Point], occupied: np.ndarray, res: float,
                    origin: Point, open_r: float = 0.66, rot_r: float = 0.46,
                    aisle_r: float = 0.25, aisle_ext: float = AISLE_EXT) -> Dict[str, object]:
    """
    직진 선분 중심선(간격 2 cm 표본)의 장애물 여유 ≥ 요구값, 회전 지점 여유 ≥ rot_r.

    요구값: 좁은 통로 상자(y 로 aisle_ext 넓힘) 안 표본은 aisle_r, 밖은 open_r. 회전 지점 = 통로 밖 웨이포인트
    (통로 안에서는 제자리 회전을 하지 않는다 — 시험이 이상 모델로 확인). occupied 는 행 0 = 최하단 y 인 bool 격자,
    거리는 셀 중심 기준 유클리드 거리변환.
    기본값: open_r = 외접 반경 0.36 + 0.30, rot_r = 0.36 + 0.10, aisle_r = 반폭 0.20 + 0.05.
    """
    from scipy import ndimage
    dist = ndimage.distance_transform_edt(~occupied) * res
    pts = [tuple(map(float, start))] + [tuple(map(float, p)) for p in waypoints]
    segs, turns, ok = [], [], True
    for a, b in zip(pts, pts[1:]):
        xs, ys = _samples(a, b)
        d = _lookup(dist, res, origin, xs, ys)
        need = np.where(in_aisle(xs, ys, aisle_ext), aisle_r, open_r)
        i = int(np.argmin(d - need))
        good = bool(d[i] >= need[i])
        ok &= good
        segs.append({'a': a, 'b': b, 'min_clearance': round(float(d.min()), 3),
                     'worst_margin': round(float(d[i] - need[i]), 3),
                     'worst_at': (round(float(xs[i]), 2), round(float(ys[i]), 2)),
                     'need_there': float(need[i]), 'ok': good})
    for p in pts[1:-1]:
        if bool(in_aisle(p[0], p[1], aisle_ext)):
            continue
        c = float(_lookup(dist, res, origin, np.array([p[0]]), np.array([p[1]]))[0])
        good = c >= rot_r
        ok &= good
        turns.append({'at': p, 'need': rot_r, 'clearance': round(c, 3), 'ok': bool(good)})
    return {'ok': bool(ok), 'segments': segs, 'turns': turns,
            'rule': {'open_r': open_r, 'rot_r': rot_r, 'aisle_r': aisle_r,
                     'aisle_ext': aisle_ext, 'res': res}}
