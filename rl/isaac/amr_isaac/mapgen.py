"""
지도 변형 평가 (b1-mapgen, 완성 계획 D3 의 "지도 변형" 축) — Isaac 비의존 (numpy · scipy).

학습은 DR 켬·끔 두 팔 모두 창고 지도 한 장(정적 2D 사각형 129 개)으로만 했다. 학습 DR 에는 지도 축이 없다.
이 모듈은 그 지도를 바꾼 **변형 지도**를 결정론적으로 만들고, 기존 평가 집합의 시작·목표를 변형 지도에서
**같은 계획기·같은 여유 규칙으로 다시 계획**한다 (paths.build_pool 의 경로 한 개 절차 그대로).

물체 (움직이는 단위) — warehouse.sdf 의 모델 하나 + 그 모델에 붙은 표지판:
  - 랙 25 (랙 표지판 rack_marker_* 은 가장 가까운 랙에 붙인다), 기둥 8 (+ pillar_k_marker_j), 도크 상자 8,
    충전기 3 (+ charging_station_cN_marker) — 44 개
  - 고정: 바깥벽 4 (warehouse_walls), 벽에 붙은 도크 판 4 (dock_*_marker)
변형 종류 (suite 조건의 kind):
  jitter  물체마다 강체 이동 (dx, dy) ~ N(0, σ²) 와 물체 중심 둘레 회전 dθ ~ N(0, σθ²), 각 성분을 ±clip·σ 로 자른다
  scale   물체마다 크기 배율 s ~ U[lo, hi] — 물체 중심 기준으로 위치·반길이를 함께 (붙은 판은 면에 붙은 채로)
  remove  물체마다 확률 p 로 뺀다 (붙은 판 포함)
  add     새 상자 n 개 (반길이 U[half_lo, half_hi]², yaw U[0, π), 높이 [0, height]) 를 그 지도에 배정된 에피소드의
          **명목 경로 위**(호길이 비율 U[arc_lo, arc_hi], 경로 법선 방향 N(0, lateral_sigma²) 를 ±2σ 로 자름)에 놓는다.
          기존 장애물·다른 새 상자와 겹치면, 또는 그 지도의 시작·목표 점에서 keepout_m 안이면 다시 뽑는다
난수: 조건 c 의 지도 k 는 np.random.default_rng([cond.seed, k]). 물체는 이름 순서로 늘 같은 수의 난수를 쓴다.

재계획 (경로 한 개 = paths.build_pool 안쪽 반복과 같은 식, tests/test_mapgen.py 가 명목 지도에서 풀 경로를 비트 단위로
재현함을 확인한다):
  occ = warehouse_geom.rasterize(변형 사각형) → paths.costmap → paths.reverse_graph →
  목표 셀에서 scipy dijkstra
  (limit = L_MAX·(1 + κ)) → 시작 셀에서 선행자 사슬 → paths.shortcut → paths.resample.
  제외 코드 (그 에피소드를 이 조건에서 뺀다 — 두 팔이 같은 에피소드 집합):
    1 START_BLOCKED  시작 셀의 장애물 거리 < START_CLEARANCE (풀의 시작 규칙)
    2 GOAL_BLOCKED   목표 셀의 거리 < GOAL_CLEARANCE 또는 비용 ≥ 253 (풀의 목표 규칙)
    3 NO_PATH        목표에서 시작이 닿지 않거나 비용 거리 dcost ∉ [L_MIN, 3·L_MAX] (풀의 후보 규칙)
    4 LENGTH         평활 경로 길이 ∉ [L_MIN, L_MAX] (풀의 길이 규칙)
    5 TOO_MANY_PTS   경로 점 > path_pad (env 버퍼)
  재분류: 유효 경로의 끝 접선 자세에서 변형 사각형에 대한 직진 여유(eval_sets.straight_approach_batch) ≥ 0.40 m 면
  "변형 주", 아니면 "변형 좁음".
"""

import math
import re

import numpy as np

from . import eval_sets as ES
from . import paths as PA
from . import warehouse_geom as W

SUITE_FORMAT = 'amr_isaac_map_suite_v1'
BUNDLE_FORMAT = 'amr_isaac_map_bundle_v1'
KINDS = ('jitter', 'scale', 'remove', 'add')
CODES = ('valid', 'start_blocked', 'goal_blocked', 'no_path', 'length', 'too_many_pts')
VALID, START_BLOCKED, GOAL_BLOCKED, NO_PATH, LENGTH, TOO_MANY_PTS = range(6)
PATH_PAD = 640                 # env_cfg.AmrNavEnvCfg.path_pad
WALL_PLATES = ('dock_1_marker', 'dock_2_marker', 'dock_a_marker', 'dock_b_marker')
FIXED, ADDED = -1, -2          # 물체 번호: 고정(벽), 새 상자
LIDAR_Z = W.LIDAR_Z


# ---------------------------------------------------------------------------- 명목 지도와 물체
def inband_boxes(boxes, z_lo=W.Z_BAND_LO, z_hi=W.Z_BAND_HI):
    """상자 목록 → 몸체 높이 띠와 겹치는 상자 (warehouse_geom.obstacles_2d 와 같은 거르기·같은 순서)."""
    out = []
    for b in boxes:
        lo, hi = b.z_range()
        if hi < z_lo or lo > z_hi:
            continue
        out.append(b)
    return out


def nominal_layout(boxes=None):
    """
    명목 지도 → dict(obb [M, 5], zr [M, 2] (연직 범위), names [M]).

    obb 는 warehouse_geom.obstacles_2d(boxes) 와 같은 값·같은 순서다 (시험이 확인한다).
    """
    boxes = W.load_boxes() if boxes is None else boxes
    inb = inband_boxes(boxes)
    obb = W.obstacles_2d(boxes)
    if len(inb) != obb.shape[0]:
        raise ValueError('띠 안 상자 수와 2D 사각형 수가 다르다')
    zr = np.array([b.z_range() for b in inb], dtype=np.float64).reshape(-1, 2)
    return {'obb': obb, 'zr': zr, 'names': [b.name for b in inb]}


def model_of(name):
    """충돌체 이름 'model/link/collision' → 모델 이름."""
    return name.split('/')[0]


def object_groups(names, obb):
    """
    띠 안 상자 [M] → (물체 번호 [M] (FIXED = 고정), 물체 이름 목록, 물체 기준점 [G, 2]).

    물체 기준점 = 물체 이름과 같은 모델의 상자 중심 (랙·기둥·상자·충전기 본체). 규칙은 모듈 머리말.
    모르는 표지판 꼴이 나오면 실패한다 (창고가 바뀌면 조용히 넘어가지 않게).
    """
    models = [model_of(n) for n in names]
    obb = np.asarray(obb, dtype=np.float64)
    gname = []
    for m in models:
        if m.startswith('warehouse_walls') or m in WALL_PLATES:
            gname.append(None)
        elif re.fullmatch(r'pillar_\d+_marker_\d+', m):
            gname.append(m.split('_marker')[0])
        elif re.fullmatch(r'charging_station_c\d+_marker', m):
            gname.append(m[:-len('_marker')])
        elif m.startswith('rack_marker_'):
            gname.append('<rack>')
        elif 'marker' in m:
            raise ValueError(f'모르는 표지판 모델 {m}')
        else:
            gname.append(m)
    racks = [i for i, m in enumerate(models) if m.startswith('rack_') and
             not m.startswith('rack_marker_')]
    for i, g in enumerate(gname):
        if g == '<rack>':
            d = np.hypot(obb[racks, 0] - obb[i, 0], obb[racks, 1] - obb[i, 1])
            gname[i] = models[racks[int(np.argmin(d))]]
    groups = sorted({g for g in gname if g is not None})
    gid = np.array([FIXED if g is None else groups.index(g) for g in gname], dtype=np.int64)
    anchor = np.zeros((len(groups), 2))
    for k, g in enumerate(groups):
        main = [i for i, m in enumerate(models) if m == g]
        if len(main) != 1:
            raise ValueError(f'물체 {g} 의 본체 상자가 {len(main)} 개')
        anchor[k] = obb[main[0], :2]
    return gid, groups, anchor


# ---------------------------------------------------------------------------- 변형
def _rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def _clipped_normal(rng, sigma, clip, size=None):
    x = rng.normal(0.0, 1.0, size=size)
    return np.clip(x, -clip, clip) * sigma


def apply_jitter(obb, gid, anchor, rng, sigma_xy, sigma_yaw, clip=3.0):
    """물체마다 강체 이동·회전 (모듈 머리말). 반환 (obb, 물체별 (dx, dy, dθ) [G, 3])."""
    out = np.array(obb, dtype=np.float64, copy=True)
    G = anchor.shape[0]
    moves = np.zeros((G, 3))
    for g in range(G):
        dx, dy = _clipped_normal(rng, sigma_xy, clip, size=2)
        dth = float(_clipped_normal(rng, sigma_yaw, clip))
        moves[g] = (dx, dy, dth)
        m = gid == g
        rel = out[m, :2] - anchor[g]
        out[m, :2] = anchor[g] + rel @ _rot(dth).T + np.array([dx, dy])
        out[m, 4] = out[m, 4] + dth
    return out, moves


def apply_scale(obb, gid, anchor, rng, lo, hi):
    """물체마다 크기 배율 s ~ U[lo, hi] (물체 기준점 중심). 반환 (obb, s [G])."""
    out = np.array(obb, dtype=np.float64, copy=True)
    G = anchor.shape[0]
    s_all = np.zeros(G)
    for g in range(G):
        s = float(rng.uniform(lo, hi))
        s_all[g] = s
        m = gid == g
        out[m, :2] = anchor[g] + s * (out[m, :2] - anchor[g])
        out[m, 2:4] = s * out[m, 2:4]
    return out, s_all


def apply_remove(gid, rng, p, n_groups):
    """물체마다 확률 p 로 뺀다 → (남길 상자 [M] bool, 뺀 물체 [G] bool)."""
    removed = rng.random(n_groups) < p
    keep = np.ones(gid.shape[0], dtype=bool)
    for g in np.flatnonzero(removed):
        keep[gid == g] = False
    return keep, removed


def rect_point_distance(obb, pts):
    """사각형 [M, 5] 와 점 [P, 2] 사이 거리 [M, P] (안이면 0)."""
    obb = np.asarray(obb, dtype=np.float64).reshape(-1, 5)
    pts = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
    c, s = np.cos(obb[:, 4])[:, None], np.sin(obb[:, 4])[:, None]
    dx = pts[None, :, 0] - obb[:, 0:1]
    dy = pts[None, :, 1] - obb[:, 1:2]
    lx = np.abs(c * dx + s * dy) - obb[:, 2:3]
    ly = np.abs(-s * dx + c * dy) - obb[:, 3:4]
    return np.hypot(np.maximum(lx, 0.0), np.maximum(ly, 0.0))


def rects_overlap(a, b):
    """사각형 a [5] 와 사각형들 b [M, 5] 가 겹치는가 [M] (분리축, 경계 포함)."""
    b = np.asarray(b, dtype=np.float64).reshape(-1, 5)
    out = np.zeros(b.shape[0], dtype=bool)
    ca = _corners1(a)
    for i in range(b.shape[0]):
        cb = _corners1(b[i])
        sep = False
        for r in (a, b[i]):
            for th in (r[4], r[4] + 0.5 * math.pi):
                ax = np.array([math.cos(th), math.sin(th)])
                pa, pb = ca @ ax, cb @ ax
                if pa.max() < pb.min() or pb.max() < pa.min():
                    sep = True
                    break
            if sep:
                break
        out[i] = not sep
    return out


def _corners1(r):
    c, s = math.cos(r[4]), math.sin(r[4])
    loc = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]], dtype=np.float64) * r[2:4]
    return r[:2] + loc @ np.array([[c, s], [-s, c]])


def path_point_at(pts, frac):
    """경로 [n, 2] 의 호길이 비율 frac 위치 (x, y) 와 단위 접선."""
    seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    s = frac * cum[-1]
    i = int(np.clip(np.searchsorted(cum, s, side='right') - 1, 0, len(seg) - 1))
    u = (s - cum[i]) / max(seg[i], 1e-12)
    p = pts[i] + u * (pts[i + 1] - pts[i])
    t = (pts[i + 1] - pts[i]) / max(seg[i], 1e-12)
    return p, t


def apply_add(obb, rng, cond, host_paths, keep_pts, max_tries=20000):
    """
    새 상자 cond['n'] 개를 host_paths (명목 경로 목록) 위에 놓는다 → (새 사각형 [n', 5], 시도 수).

    keep_pts [P, 2] (그 지도의 시작·목표) 에서 keepout_m 안, 기존 사각형·다른 새 상자와 겹치는 후보는 버린다.
    max_tries 안에 n 개를 못 놓으면 놓은 만큼만 (요약에 적힌다).
    """
    n = int(cond['n'])
    new = []
    tries = 0
    base = np.asarray(obb, dtype=np.float64)
    while len(new) < n and tries < max_tries:
        tries += 1
        path = host_paths[int(rng.integers(0, len(host_paths)))]
        frac = float(rng.uniform(cond['arc_lo'], cond['arc_hi']))
        p, t = path_point_at(path, frac)
        lat = float(_clipped_normal(rng, cond['lateral_sigma'], 2.0))
        hx, hy = rng.uniform(cond['half_lo'], cond['half_hi'], size=2)
        yaw = float(rng.uniform(0.0, math.pi))
        cand = np.array([p[0] - t[1] * lat, p[1] + t[0] * lat, hx, hy, yaw])
        if keep_pts.shape[0] and rect_point_distance(cand, keep_pts).min() < cond['keepout_m']:
            continue
        others = np.concatenate([base, np.asarray(new).reshape(-1, 5)])
        if rects_overlap(cand, others).any():
            continue
        new.append(cand)
    return np.asarray(new, dtype=np.float64).reshape(-1, 5), tries


def make_variant(nominal, gid, anchor, cond, k, host_paths=(), keep_pts=None):
    """
    조건 cond 의 지도 k → dict(obb, zr, gid, info).

    nominal = nominal_layout(); host_paths·keep_pts 는 add 에만 쓴다 (그 지도에 배정된 에피소드의 명목 경로와
    시작·목표 점).
    """
    rng = np.random.default_rng([int(cond['seed']), int(k)])
    obb = np.array(nominal['obb'], dtype=np.float64, copy=True)
    zr = np.array(nominal['zr'], dtype=np.float64, copy=True)
    g = np.array(gid, copy=True)
    info = {'kind': cond['kind']}
    kind = cond['kind']
    if kind == 'jitter':
        obb, moves = apply_jitter(obb, gid, anchor, rng, float(cond['sigma_xy']),
                                  math.radians(float(cond['sigma_yaw_deg'])),
                                  float(cond.get('clip', 3.0)))
        info['shift_mean_m'] = float(np.hypot(moves[:, 0], moves[:, 1]).mean())
        info['shift_max_m'] = float(np.hypot(moves[:, 0], moves[:, 1]).max())
        info['yaw_abs_mean_deg'] = float(np.degrees(np.abs(moves[:, 2])).mean())
    elif kind == 'scale':
        obb, s = apply_scale(obb, gid, anchor, rng, float(cond['lo']), float(cond['hi']))
        info['scale_min'], info['scale_max'] = float(s.min()), float(s.max())
    elif kind == 'remove':
        keep, removed = apply_remove(gid, rng, float(cond['p']), anchor.shape[0])
        obb, zr, g = obb[keep], zr[keep], g[keep]
        info['removed_objects'] = int(removed.sum())
    elif kind == 'add':
        kp = np.zeros((0, 2)) if keep_pts is None else np.asarray(keep_pts, dtype=np.float64)
        new, tries = apply_add(obb, rng, cond, list(host_paths), kp)
        obb = np.concatenate([obb, new])
        zr = np.concatenate([zr, np.tile([0.0, float(cond['height'])], (new.shape[0], 1))])
        g = np.concatenate([g, np.full(new.shape[0], ADDED, dtype=np.int64)])
        info['added'] = int(new.shape[0])
        info['add_tries'] = int(tries)
    else:
        raise ValueError(f'모르는 변형 {kind!r} (가능: {KINDS})')
    return {'obb': obb, 'zr': zr, 'gid': g, 'info': info}


# ---------------------------------------------------------------------------- 재계획
class Planner:
    """한 지도의 비용 지도·역방향 그래프 (paths.build_pool 과 같은 계획기)."""

    def __init__(self, obb):
        """사각형 [M, 5] → 점유 격자·비용·거리·그래프."""
        self.occ, self.origin, self.res = W.rasterize(np.asarray(obb, dtype=np.float64))
        self.cost, self.dist = PA.costmap(self.occ)
        self.graph = PA.reverse_graph(self.cost)
        self.nx = self.cost.shape[1]
        self._goal = None
        self._dcost = self._pred = None

    def _dijkstra(self, goal):
        from scipy.sparse.csgraph import dijkstra
        if self._goal != goal:
            self._dcost, self._pred = dijkstra(self.graph, directed=True, indices=int(goal),
                                               return_predecessors=True,
                                               limit=PA.L_MAX * (1.0 + PA.KAPPA))
            self._goal = goal
        return self._dcost, self._pred

    def plan(self, goal, start):
        """목표·시작 셀 (평탄 색인) → (제외 코드, 경로 점 [n, 2] 또는 None, 길이)."""
        goal, start = int(goal), int(start)
        if self.dist.ravel()[start] < PA.START_CLEARANCE:
            return START_BLOCKED, None, math.nan
        if not (self.dist.ravel()[goal] >= PA.GOAL_CLEARANCE and self.cost.ravel()[goal] < 253):
            return GOAL_BLOCKED, None, math.nan
        dcost, pred = self._dijkstra(goal)
        dc = dcost[start]
        if not (np.isfinite(dc) and PA.L_MIN <= dc <= PA.L_MAX * 3.0):
            return NO_PATH, None, math.nan
        seq = [start]
        while seq[-1] != goal:
            p = pred[seq[-1]]
            if p < 0:
                break
            seq.append(int(p))
        if seq[-1] != goal:
            return NO_PATH, None, math.nan
        xy = np.array([PA.cell_center(i, self.nx, self.origin) for i in seq])
        cc = self.cost.ravel()[np.asarray(seq)]
        xy = PA.shortcut(xy, cc, self.cost, self.origin)
        pts = PA.resample(xy)
        L = float(np.sum(np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))))
        if not (PA.L_MIN <= L <= PA.L_MAX):
            return LENGTH, None, L
        if pts.shape[0] > PATH_PAD:
            return TOO_MANY_PTS, None, L
        return VALID, pts, L


def end_clearance(pts, obb):
    """경로 끝 접선 자세의 직진 여유 (eval_sets 의 주/스트레스 규칙과 같은 함수)."""
    return float(ES.straight_approach_batch(np.array([ES.end_tangent_pose(pts)]), obb)[0])


# ---------------------------------------------------------------------------- 타일·메시
def tile_offsets(n_tiles, spacing, cols=8, rows=7):
    """
    타일 번호 → 세계 좌표 원점 이동 [n, 2]. 타일 0 = (0, 0) (명목 지도 = 원래 자리), 나머지는 격자.

    격자 열 i ∈ [−cols/2, cols/2), 행 j ∈ [−rows//2, rows//2] 를 (0, 0) 을 빼고 행 우선으로 채운다.
    벽(60.4 × 40.4 m)이 타일마다 닫혀 있어 LiDAR 광선이 이웃 타일로 나가지 않는다 — 간격은 벽보다 커야 한다.
    """
    sx, sy = spacing
    if sx <= 60.4 + 2.0 or sy <= 40.4 + 2.0:
        raise ValueError(f'타일 간격 {spacing} 이 창고(60.4 × 40.4 m)에 비해 작다')
    cells = [(0, 0)]
    for j in range(-(rows // 2), rows // 2 + 1):
        for i in range(-(cols // 2), cols - cols // 2):
            if (i, j) != (0, 0):
                cells.append((i, j))
    if n_tiles > len(cells):
        raise ValueError(f'타일 {n_tiles} 개 > 격자 {len(cells)} 칸')
    return np.array([(i * sx, j * sy) for i, j in cells[:n_tiles]], dtype=np.float64)


def far_cell_index(offsets):
    """원점에서 가장 먼 타일 번호 (부동소수 이동 탐침용)."""
    return int(np.argmax(np.hypot(offsets[:, 0], offsets[:, 1])))


def box_mesh(obb, zr):
    """사각형 [M, 5] + 연직 범위 [M, 2] → (정점 float32, 삼각형 int32) — triangle_mesh 와 같은 규약."""
    boxes = []
    for (cx, cy, hx, hy, yaw), (lo, hi) in zip(np.asarray(obb), np.asarray(zr)):
        boxes.append(W.Box('v', np.array([cx, cy, 0.5 * (lo + hi)]),
                           np.array([2 * hx, 2 * hy, hi - lo]), W.rpy_matrix(0.0, 0.0, yaw)))
    if not boxes:
        return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.int32)
    return W.triangle_mesh(boxes)


def shift_obb(obb, off):
    """사각형을 off (dx, dy) 만큼 옮긴다."""
    out = np.array(obb, dtype=np.float64, copy=True)
    out[:, 0] += off[0]
    out[:, 1] += off[1]
    return out


def raycast_2d(origin, angles, obb, zr, max_range=25.0, z=LIDAR_Z):
    """
    2D 광선 [B] 의 첫 닿는 거리 (스캔 평면 z 를 지나는 사각형만, 없거나 max_range 초과면 nan) — Isaac 스캔 대조용.

    사각형 국소 좌표의 slab 법. origin (x, y), angles [B] (세계 각).
    """
    obb = np.asarray(obb, dtype=np.float64)
    zr = np.asarray(zr, dtype=np.float64)
    m = (zr[:, 0] <= z) & (zr[:, 1] >= z)
    obb = obb[m]
    ang = np.asarray(angles, dtype=np.float64)
    if obb.shape[0] == 0:
        return np.full(ang.shape, np.nan)
    dx, dy = np.cos(ang)[:, None], np.sin(ang)[:, None]           # [B, 1]
    c, s = np.cos(obb[:, 4])[None], np.sin(obb[:, 4])[None]       # [1, M]
    ox, oy = origin[0] - obb[None, :, 0], origin[1] - obb[None, :, 1]
    lox, loy = c * ox + s * oy, -s * ox + c * oy                  # 원점 (국소)
    ldx, ldy = c * dx + s * dy, -s * dx + c * dy                  # 방향 (국소)
    hx, hy = obb[None, :, 2], obb[None, :, 3]
    with np.errstate(divide='ignore', invalid='ignore'):
        tx1, tx2 = (-hx - lox) / ldx, (hx - lox) / ldx
        ty1, ty2 = (-hy - loy) / ldy, (hy - loy) / ldy
    big = 1e18
    tx1 = np.where(np.abs(ldx) < 1e-15, np.where(np.abs(lox) <= hx, -big, big), tx1)
    tx2 = np.where(np.abs(ldx) < 1e-15, np.where(np.abs(lox) <= hx, big, -big), tx2)
    ty1 = np.where(np.abs(ldy) < 1e-15, np.where(np.abs(loy) <= hy, -big, big), ty1)
    ty2 = np.where(np.abs(ldy) < 1e-15, np.where(np.abs(loy) <= hy, big, -big), ty2)
    tn = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
    tf = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
    hit = (tn <= tf) & (tf >= 0.0)
    t = np.where(hit, np.maximum(tn, 0.0), np.inf).min(axis=1)
    return np.where(t <= max_range, t, np.nan)
