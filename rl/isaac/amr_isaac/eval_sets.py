"""
B 단계 평가 집합 — 경로 끝 접선 자세의 풋프린트 앞 끝 직진 여유로 주 집합과 좁은 목표 스트레스 집합을 나눈다.

정의 (완성 계획 bonus1_completion_plan.md §6, 21:05 결정 + 21:55 정정; A3 노트 2026-10-06_1901 「목표 여유」):
  - 끝 접선 자세 = 경로 마지막 점, 방향 = 마지막 점에서 0.25 m 앞 점(경로 점 간격 0.05 m, 6 번째 앞 점) → 마지막 점
    (eval_metrics.goal_clearances 와 같은 규칙)
  - 직진 여유 = 그 자세의 풋프린트(0.60 × 0.40 m)를 앞(+x)으로 곧게 밀 때 정적 사각형에 처음 닿기까지 거리
    (= 정본 SafetyGate 스윕 풋프린트의 직진 접근 거리 D 의 기하값; 없으면 inf, 이미 겹치면 0)
  - **주 집합** = 직진 여유 ≥ 0.40 m (정지 문턱 0.30 + 여유 0.10), **스트레스 집합** = 0.40 m 미만
  - 학습은 경로 풀 전체(좁은 목표 포함)로 한다 — 이 모듈은 평가 집합만 정한다.

Isaac·Gazebo 비의존 (numpy). 스칼라 기준판은 eval_metrics.straight_approach, 이 모듈의 배치판은 그와 같은 값을
내야 한다 (tests/test_eval_sets.py 가 무작위 자세·실제 창고·경로 풀에서 대조한다).
"""

import hashlib
import json
import math

import numpy as np

from .eval_metrics import HALF_L, HALF_W

THRESHOLD = 0.40          # [m] 주 집합 문턱 (정지 문턱 0.30 + 0.10)
TANGENT_BACK = 5          # 끝 접선 = p[n-1-5] → p[n-1] (0.25 m, 간격 0.05 m)
MAX_DIST = 5.0            # straight_approach 의 max_dist 와 같다 (넘으면 inf)
SET_FORMAT = 'amr_isaac_eval_sets_v1'


def _corners(obb):
    """사각형 [M, 5] (cx, cy, hx, hy, yaw) → 꼭짓점 [M, 4, 2] (eval_metrics._rect_corners 와 같은 순서)."""
    obb = np.asarray(obb, dtype=np.float64)
    c, s = np.cos(obb[:, 4]), np.sin(obb[:, 4])
    sx = np.array([-1.0, 1.0, 1.0, -1.0])
    sy = np.array([-1.0, -1.0, 1.0, 1.0])
    lx = obb[:, 2:3] * sx                        # [M, 4]
    ly = obb[:, 3:4] * sy
    x = obb[:, 0:1] + c[:, None] * lx - s[:, None] * ly
    y = obb[:, 1:2] + s[:, None] * lx + c[:, None] * ly
    return np.stack([x, y], axis=-1)


def straight_approach_batch(poses, obb, half_l=HALF_L, half_w=HALF_W, max_dist=MAX_DIST):
    """
    자세 [K, 3] 마다 풋프린트를 앞으로 직진시킬 때 사각형 [M, 5] 에 처음 닿기까지 거리 [K].

    eval_metrics.straight_approach 의 배치판 (같은 두 경우: 장애물 꼭짓점이 앞 변에 닿음 / 앞 모서리 광선이
    장애물 변에 닿음). 어떤 장애물과 이미 겹치면 0, 닿는 것이 없거나 max_dist 초과면 inf.
    """
    poses = np.atleast_2d(np.asarray(poses, dtype=np.float64))
    oc = _corners(obb)                                           # [M, 4, 2]
    c = np.cos(poses[:, 2])[:, None, None]
    s = np.sin(poses[:, 2])[:, None, None]
    dx = oc[None, :, :, 0] - poses[:, None, None, 0]             # [K, M, 4]
    dy = oc[None, :, :, 1] - poses[:, None, None, 1]
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    # 겹침 (분리축 4 개: 로봇 x·y, 장애물 변 법선 2 개)
    sep = (lx.max(-1) < -half_l) | (lx.min(-1) > half_l) | \
          (ly.max(-1) < -half_w) | (ly.min(-1) > half_w)
    fp = np.array([[-half_l, -half_w], [half_l, -half_w], [half_l, half_w], [-half_l, half_w]])
    for i in (0, 1):
        ex = lx[..., i + 1] - lx[..., i]
        ey = ly[..., i + 1] - ly[..., i]
        nx, ny = -ey, ex                                         # 변 법선 [K, M]
        po = lx * nx[..., None] + ly * ny[..., None]             # 장애물 투영 [K, M, 4]
        pf = fp[None, None, :, 0] * nx[..., None] + fp[None, None, :, 1] * ny[..., None]
        sep |= (po.max(-1) < pf.min(-1)) | (pf.max(-1) < po.min(-1))
    overlap = (~sep).any(axis=1)                                 # [K]
    inf = np.inf
    # (1) 장애물 꼭짓점이 앞 변(|y| ≤ 반폭, x ≥ 반길이)
    ok = (np.abs(ly) <= half_w) & (lx >= half_l)
    best = np.where(ok, lx - half_l, inf).min(axis=(1, 2))
    # (2) 앞 모서리 (반길이, ±반폭) 에서 +x 광선이 장애물 변과 만남
    p0x, p0y = lx, ly
    p1x, p1y = np.roll(lx, -1, axis=-1), np.roll(ly, -1, axis=-1)
    for cy in (-half_w, half_w):
        cross = ((p0y - cy) * (p1y - cy) <= 0) & (p0y != p1y)
        den = np.where(cross, p1y - p0y, 1.0)
        t = (cy - p0y) / den
        xi = p0x + t * (p1x - p0x)
        hit = cross & (xi >= half_l)
        best = np.minimum(best, np.where(hit, xi - half_l, inf).min(axis=(1, 2)))
    best = np.where(best <= max_dist, best, inf)
    return np.where(overlap, 0.0, best)


def end_tangent_pose(path_pts):
    """경로 [n, 2] 의 끝 접선 자세 (x, y, θ) — eval_metrics.goal_clearances 와 같은 규칙."""
    p = np.asarray(path_pts, dtype=np.float64)
    g = p[-1]
    j = max(0, p.shape[0] - 1 - TANGENT_BACK)
    d = g - p[j]
    th = math.atan2(d[1], d[0]) if math.hypot(d[0], d[1]) > 1e-9 else 0.0
    return float(g[0]), float(g[1]), th


def pool_end_poses(pts, offs):
    """경로 풀 (pts [Σn, 2], offs [K+1]) → 끝 접선 자세 [K, 3]."""
    offs = np.asarray(offs)
    return np.array([end_tangent_pose(pts[offs[k]:offs[k + 1]]) for k in range(offs.shape[0] - 1)])


def front_clearance(pts, offs, obb, chunk=512):
    """경로 풀의 경로마다 끝 접선 자세의 직진 여유 [K] (메모리 때문에 chunk 단위)."""
    poses = pool_end_poses(pts, offs)
    out = np.empty(poses.shape[0])
    for a in range(0, poses.shape[0], chunk):
        out[a:a + chunk] = straight_approach_batch(poses[a:a + chunk], obb)
    return out


def classify(clearance, threshold=THRESHOLD):
    """직진 여유 → 'main'(≥ 문턱) / 'stress'(< 문턱) 불리언 쌍."""
    c = np.asarray(clearance, dtype=np.float64)
    main = c >= threshold
    return main, ~main


def pool_digest(pts, offs, length):
    """경로 풀 내용 해시 (npz 압축 시각과 무관하게 배열 내용으로)."""
    h = hashlib.sha256()
    for a in (pts, offs, length):
        a = np.ascontiguousarray(np.asarray(a))
        h.update(str(a.dtype).encode())
        h.update(str(a.shape).encode())
        h.update(a.tobytes())
    return h.hexdigest()[:16]


def balanced_sample(ids, n, rng):
    """
    후보 ids 에서 n 개 — 모두 쓰고 남는 것은 겹치지 않게 (각 원소 ⌊n/K⌋ 또는 ⌈n/K⌉ 번), 순서는 섞는다.

    K ≥ n 이면 비복원 추출과 같다.
    """
    ids = np.asarray(ids)
    K = ids.shape[0]
    if K == 0:
        raise ValueError('빈 후보에서 표본을 만들 수 없다')
    k, rem = divmod(n, K)
    out = np.concatenate([np.repeat(ids, k), rng.choice(ids, size=rem, replace=False)])
    rng.shuffle(out)
    return out


def _jnum(x):
    """JSON 안전 실수 (inf → 문자열 'inf')."""
    x = float(x)
    return x if math.isfinite(x) else ('inf' if x > 0 else '-inf')


def build_eval_sets(length, clearance, ranges=((3.0, 15.0), (3.0, 30.0)), n_main=2048,
                    n_stress=512, seed=20261006, threshold=THRESHOLD, n_perturb=1024):
    """
    평가 집합 — 범위마다 main_<lo>_<hi> (n_main) 와 stress_<lo>_<hi> (n_stress), 그리고 perturb_3_15.

    모든 표본은 경로 풀(원래 번호)에서 balanced_sample (주: 후보가 n 보다 많으면 비복원, 스트레스: 좁은 목표 경로를
    고르게 되풀이). 길이 범위는 env 의 select_paths_by_length 와 같다 (lo ≤ L ≤ hi).
    perturb_3_15 = main_3_15 의 앞 n_perturb 개 (교란 묶음의 각 조건이 쓰는 주 집합 부분, 무작위 순서의 앞부분이라
    무작위 부분 표본이다). 반환 dict: sets {이름: 경로 번호 목록}, stats {이름: 후보 수·서로 다른 경로·목표 수 …}.
    """
    length = np.asarray(length, dtype=np.float64)
    clearance = np.asarray(clearance, dtype=np.float64)
    rng = np.random.default_rng(seed)
    main_mask, stress_mask = classify(clearance, threshold)
    sets, stats = {}, {}
    for lo, hi in ranges:
        tag = f'{lo:g}_{hi:g}'
        inr = (length >= lo) & (length <= hi)
        for name, mask, n in (('main', main_mask, n_main), ('stress', stress_mask, n_stress)):
            cand = np.flatnonzero(inr & mask)
            ids = balanced_sample(cand, n, rng)
            sets[f'{name}_{tag}'] = ids
            stats[f'{name}_{tag}'] = {
                'n': int(ids.shape[0]), 'candidates': int(cand.shape[0]),
                'distinct_paths': int(np.unique(ids).shape[0]),
                'candidate_share_of_range': float(cand.shape[0] / max(int(inr.sum()), 1)),
                'clearance_min': _jnum(clearance[ids].min()),
                'clearance_median': _jnum(np.median(clearance[ids])),
                'clearance_inf_share': float(np.isinf(clearance[ids]).mean()),
                'length_mean': float(length[ids].mean())}
    if '3_15' in [f'{lo:g}_{hi:g}' for lo, hi in ranges]:
        sets['perturb_3_15'] = sets['main_3_15'][:n_perturb]
        stats['perturb_3_15'] = {'n': int(min(n_perturb, sets['main_3_15'].shape[0])),
                                 'prefix_of': 'main_3_15'}
    return sets, stats


def goal_groups(pts, offs, ids, decimals=3):
    """경로 번호 목록 → 목표(끝점) 묶음 번호 [n] 과 서로 다른 목표 수 (군집 표본 크기 보고용)."""
    offs = np.asarray(offs)
    ends = np.asarray(pts)[offs[1:] - 1]
    g = np.round(ends[np.asarray(ids)], decimals)
    _, inv = np.unique(g, axis=0, return_inverse=True)
    return inv.reshape(-1), int(inv.max() + 1) if inv.size else 0


def save_sets(path, sets, stats, meta):
    """집합 파일 (JSON) — 경로 번호는 풀 원래 번호."""
    doc = {'format': SET_FORMAT, **meta, 'stats': stats,
           'sets': {k: [int(x) for x in v] for k, v in sets.items()}}
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(doc, f, ensure_ascii=False, indent=1)
    return doc


def load_sets(path):
    """집합 파일 → dict (형식 확인)."""
    with open(path, encoding='utf-8') as f:
        doc = json.load(f)
    if doc.get('format') != SET_FORMAT:
        raise ValueError(f'평가 집합 형식이 아니다: {doc.get("format")!r}')
    return doc


def check_pool(doc, pts, offs, length):
    """집합 파일이 이 경로 풀로 만든 것인지 (내용 해시) — 다르면 ValueError."""
    d = pool_digest(pts, offs, length)
    if d != doc['pool_digest']:
        raise ValueError(f'경로 풀이 다르다: 집합 {doc["pool_digest"]} 대 풀 {d}')
    return d


def cluster_bootstrap_ci(x, groups, reps=2000, seed=0, conf=0.95):
    """
    군집 부트스트랩 백분위 구간 — 같은 군집(예: 같은 목표)의 에피소드를 한 덩어리로 복원 추출해 평균을 다시 잰다.

    x [n] (성공 0/1 등), groups [n] (군집 번호). 반환 (점추정, 하한, 상한). 군집이 하나면 구간 = 점추정.
    스트레스 집합은 경로 83·244 개, 목표 28·32 개뿐이라 이항 n 으로 잰 구간이 지나치게 좁다 — 이 구간을 함께 낸다.
    """
    x = np.asarray(x, dtype=np.float64)
    g = np.asarray(groups)
    est = float(x.mean()) if x.size else float('nan')
    if x.size == 0:
        return est, est, est
    _, inv = np.unique(g, return_inverse=True)
    k = int(inv.max()) + 1
    s = np.bincount(inv, weights=x, minlength=k)
    c = np.bincount(inv, minlength=k).astype(np.float64)
    if k == 1:
        return est, est, est
    rng = np.random.default_rng(seed)
    draw = rng.integers(0, k, size=(reps, k))
    boots = s[draw].sum(1) / c[draw].sum(1)
    lo, hi = np.percentile(boots, [100 * (1 - conf) / 2, 100 * (1 + conf) / 2])
    return est, float(lo), float(hi)
