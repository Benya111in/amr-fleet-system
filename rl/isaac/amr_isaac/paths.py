"""
에피소드 경로 풀 — 창고 점유 격자 위의 A* 비용 최단 경로 (contract.md §10 "시작·목표").

계약 §10: "A* 경로는 리셋 때 한 번 계산하고 에피소드 동안 고정 … 같은 비용(N:27 κ_c 2.0, 대각 모서리 금지
N:34)". env 리셋 때마다 A* 를 돌리는 대신 같은 규칙의 최단 경로를 미리 풀로 만들어 두고 리셋 때 뽑는다
(같은 시작·목표면 같은 경로이므로 리셋 때 계산한 것과 같다). 생성물은 커밋하지 않는다
(/home/CAPYI_AI/runs/r1-isaac/ 에 캐시).

비용 (src/amr_navigation/config/nav2_params.yaml,
      src/amr_navigation/src/core/astar.cpp:70-90, :310-330)
  - 전역 코스트맵 팽창: inscribed 0.20 m (풋프린트 내접), cost_scaling 2.0, radius 1.2 (N:647-650)
    cost = 254 (점유), 253 (거리 ≤ 0.20), 252·exp(−2.0·(d − 0.20)) (d ≤ 1.2), 0 (그 밖)
  - 간선 = 길이 × (1 + κ·c/252), κ = 2.0, c = 도착 셀 비용, c ≥ 253 통행 불가, 대각은 두 직교 이웃이
    모두 통행 가능할 때만 (allow_corner_cutting False)
  - 해 = 비용 최단 (A* 와 같은 최적 비용; 동률 경로 선택은 다를 수 있다). scipy Dijkstra 를 목표에서
    역방향으로 돌려 한 번에 여러 시작점의 경로를 얻는다.
  - 평활: C++ 평활기(숏컷 + Gauss–Seidel, N:33-44) 대신 숏컷만 근사 — 현의 셀 비용 ≤ 원 구간 최대 + 10,
    현 비용 ≤ 1.05 × 원 비용, 현 길이 ≤ 10 m. 그 뒤 0.05 m 간격으로 재표본 (N:37 output_spacing).
"""

import math

import numpy as np

RES = 0.05
INSCRIBED = 0.20
INFL_RADIUS = 1.2
COST_SCALING = 2.0
KAPPA = 2.0
START_CLEARANCE = 0.5 + math.hypot(0.30, 0.20)   # 풋프린트 ≥ 0.5 m (contract §10), 모든 yaw 에 대해
GOAL_CLEARANCE = 0.40                            # 외접 반경 0.361 초과 (모든 도착 yaw 에서 비충돌)
L_MIN, L_MAX = 3.0, 30.0                         # contract §10
SPACING = 0.05


def inflation_cost(dist):
    """거리 [m] → 0–254 코스트 (nav2 InflationLayer::computeCost 와 같은 식)."""
    c = np.zeros(dist.shape, dtype=np.float64)
    m = dist <= INFL_RADIUS
    c[m] = 252.0 * np.exp(-COST_SCALING * (dist[m] - INSCRIBED))
    c[dist <= INSCRIBED] = 253.0
    c[dist <= 0.0] = 254.0
    return np.floor(c).astype(np.int32)


def costmap(occ, res=RES):
    """점유 격자 → (코스트 int, 장애물까지 거리 [m])."""
    from scipy.ndimage import distance_transform_edt
    dist = distance_transform_edt(~occ) * res
    dist[occ] = 0.0
    return inflation_cost(dist), dist


def reverse_graph(cost):
    """역방향 CSR 그래프: 간선 v→u 의 가중 = 정방향 u→v 의 가중 = 길이·(1 + κ·c_v/252)."""
    from scipy.sparse import csr_matrix
    ny, nx = cost.shape
    trav = cost < 253
    mult = 1.0 + KAPPA * cost / 252.0
    idx = np.arange(ny * nx).reshape(ny, nx)
    rows, cols, wts = [], [], []
    for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)):
        # v = (y, x), u = (y + dy, x + dx): 정방향 u → v
        ys = slice(max(0, -dy), ny - max(0, dy))
        xs = slice(max(0, -dx), nx - max(0, dx))
        yu = slice(max(0, dy), ny - max(0, -dy))
        xu = slice(max(0, dx), nx - max(0, -dx))
        ok = trav[ys, xs] & trav[yu, xu]
        if dx != 0 and dy != 0:
            # 대각: 두 직교 이웃 (y, x+dx), (y+dy, x) 가 모두 통행 가능 (u↔v 대칭)
            ok &= trav[ys, xu] & trav[yu, xs]
        ln = math.hypot(dx, dy) * RES
        rows.append(idx[ys, xs][ok])
        cols.append(idx[yu, xu][ok])
        wts.append(ln * mult[ys, xs][ok])
    rows = np.concatenate(rows)
    cols = np.concatenate(cols)
    wts = np.concatenate(wts)
    return csr_matrix((wts, (rows, cols)), shape=(ny * nx, ny * nx))


def cell_center(i, nx, origin, res=RES):
    """선형 인덱스 → 셀 중심 (x, y)."""
    y, x = divmod(int(i), nx)
    return origin[0] + (x + 0.5) * res, origin[1] + (y + 0.5) * res


def _chord_ok(p, q, cost, origin, max_cost, res=RES):
    """현 p→q 위 셀 비용이 모두 max_cost 이하이고 통행 가능한가, 그리고 현 비용."""
    d = math.hypot(q[0] - p[0], q[1] - p[1])
    n = max(2, int(math.ceil(d / (0.5 * res))) + 1)
    t = np.linspace(0.0, 1.0, n)
    xs = p[0] + t * (q[0] - p[0])
    ys = p[1] + t * (q[1] - p[1])
    ix = np.floor((xs - origin[0]) / res).astype(int)
    iy = np.floor((ys - origin[1]) / res).astype(int)
    if ix.min() < 0 or iy.min() < 0 or ix.max() >= cost.shape[1] or iy.max() >= cost.shape[0]:
        return False, math.inf
    c = cost[iy, ix]
    if c.max() >= 253 or c.max() > max_cost:
        return False, math.inf
    return True, d * float(np.mean(1.0 + KAPPA * c / 252.0))


def shortcut(cells_xy, cell_costs, cost, origin, max_len=10.0, ratio=0.05, margin=10):
    """격자 경로 [n, 2] 의 숏컷 근사 (모듈 머리말)."""
    n = len(cells_xy)
    seg = np.hypot(np.diff(cells_xy[:, 0]), np.diff(cells_xy[:, 1]))
    seg_cost = seg * (1.0 + KAPPA * cell_costs[1:] / 252.0)
    cum_cost = np.concatenate([[0.0], np.cumsum(seg_cost)])
    out = [cells_xy[0]]
    i = 0
    while i < n - 1:
        j_best = i + 1
        j = n - 1
        while j > i + 1:
            p, q = cells_xy[i], cells_xy[j]
            if math.hypot(q[0] - p[0], q[1] - p[1]) <= max_len:
                ok, ccost = _chord_ok(p, q, cost, origin, int(cell_costs[i:j + 1].max()) + margin)
                if ok and ccost <= (1.0 + ratio) * (cum_cost[j] - cum_cost[i]) + 1e-9:
                    j_best = j
                    break
            j -= max(1, (j - i) // 8) if j - i > 16 else 1
        out.append(cells_xy[j_best])
        i = j_best
    return np.asarray(out)


def resample(poly, spacing=SPACING):
    """폴리라인을 호길이 spacing 간격으로 (끝점 포함)."""
    seg = np.hypot(np.diff(poly[:, 0]), np.diff(poly[:, 1]))
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    L = cum[-1]
    n = max(1, int(math.floor(L / spacing + 1e-9)))
    s = np.concatenate([np.arange(n + 1) * spacing, [L]]) if n * spacing < L - 1e-9 else \
        np.arange(n + 1) * spacing
    x = np.interp(s, cum, poly[:, 0])
    y = np.interp(s, cum, poly[:, 1])
    return np.stack([x, y], axis=1)


def endpoint_cells(cost, dist):
    """
    경로 끝점 후보 셀 (평탄 색인) → (시작, 목표).

    시작: 장애물까지 거리 ≥ START_CLEARANCE (풋프린트 여유 ≥ 0.5 m, 모든 yaw). 목표: 거리 ≥ GOAL_CLEARANCE
    이고 통행 가능 (비용 < 253). tests/test_warehouse_paths.py 가 두 문턱을 창고 전체 격자에서 확인한다.
    """
    start = np.flatnonzero(dist.ravel() >= START_CLEARANCE)
    goal = np.flatnonzero((dist.ravel() >= GOAL_CLEARANCE) & (cost.ravel() < 253))
    return start, goal


def build_pool(occ, origin, n_goals=256, starts_per_goal=16, seed=0, smooth=True, log=print):
    """
    경로 풀 생성 — 반환 dict: pts [M, 2] float64 (이어 붙임), offs [n+1] int64, length [n].

    목표: 거리 ≥ GOAL_CLEARANCE 인 자유 셀 무작위, 시작: 거리 ≥ START_CLEARANCE 인 셀 중
    경로 길이 L ∈ [3, 30] m 인 것 무작위.
    """
    from scipy.sparse.csgraph import dijkstra
    rng = np.random.default_rng(seed)
    cost, dist = costmap(occ)
    ny, nx = cost.shape
    g = reverse_graph(cost)
    start_cells, goal_cells = endpoint_cells(cost, dist)
    paths, lengths, meta = [], [], []
    goals = rng.choice(goal_cells, size=n_goals, replace=False)
    for gi, goal in enumerate(goals):
        dcost, pred = dijkstra(g, directed=True, indices=int(goal), return_predecessors=True,
                               limit=L_MAX * (1.0 + KAPPA))
        cand = start_cells[np.isfinite(dcost[start_cells]) &
                           (dcost[start_cells] >= L_MIN) & (dcost[start_cells] <= L_MAX * 3.0)]
        rng.shuffle(cand)
        got = 0
        for s in cand[:starts_per_goal * 6]:
            seq = [int(s)]
            while seq[-1] != goal:
                p = pred[seq[-1]]
                if p < 0:
                    break
                seq.append(int(p))
            if seq[-1] != goal:
                continue
            xy = np.array([cell_center(i, nx, origin) for i in seq])
            if smooth:
                cc = cost.ravel()[np.asarray(seq)]
                xy = shortcut(xy, cc, cost, origin)
            pts = resample(xy)
            L = float(np.sum(np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))))
            if not (L_MIN <= L <= L_MAX):
                continue
            paths.append(pts)
            lengths.append(L)
            meta.append((int(goal), int(s)))
            got += 1
            if got >= starts_per_goal:
                break
        if log and (gi + 1) % 16 == 0:
            log(f'goal {gi + 1}/{n_goals}: paths {len(paths)}')
    offs = np.concatenate([[0], np.cumsum([len(p) for p in paths])]).astype(np.int64)
    return {'pts': np.concatenate(paths), 'offs': offs, 'length': np.asarray(lengths),
            'meta': np.asarray(meta, dtype=np.int64), 'origin': np.asarray(origin),
            'res': RES}
