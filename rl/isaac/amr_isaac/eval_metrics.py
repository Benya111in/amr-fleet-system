"""
평가 지표·진단의 순수 함수 (numpy, Isaac 비의존) — eval_unbiased.py 와 A3 분석이 쓴다.

정의는 결과를 보기 전에 A3 노트에서 정했다
(docs/research/rl-navigation/notebook/entries/2026-10-06_1901_b1-isaac-phi_rejudge-r7.md):
  - 층위 성공: 엄격(env 성공 종단) / Nav2 호환(첫 에피소드 중 목표 ≤ 0.25 m 인 비충돌 결정 스텝) / 최종 거리 분포
  - 정지 진단: 실패 에피소드 중 끝 거리 ≤ 0.5 m 인 것의 마지막 2 s(20 결정 스텝 × 5 safety 틱)
      φ 차단   = 틱의 ≥ 80 % 에서 정책 명령이 움직임(|v| > 0.02 또는 |ω| > 0.05) 이고 φ 가 꼭짓점 최고 점 속도를
                 50 % 이상 깎음
      정책 정지 = 틱의 ≥ 80 % 에서 정책 명령이 정지
      혼합     = 나머지
  - 목표 여유 [계산]: 목표점–정적 사각형 거리, 경로 끝 접선 자세의 풋프린트–사각형 거리, 그 자세에서 풋프린트를
    직진시킬 때 처음 닿기까지 거리(= 직진 접근 거리 D 의 기하값)
"""

import math

import numpy as np

HALF_L, HALF_W = 0.30, 0.20
MOVE_V, MOVE_W = 0.02, 0.05          # 정지 판정 문턱 (|v| ≤ 0.02 m/s 이고 |ω| ≤ 0.05 rad/s)
CUT_RATIO = 0.5                      # φ 가 점 속도를 이만큼 이상 깎으면 "깎음"
CLASS_FRAC = 0.8                     # 분류 문턱 (틱 비율)
NEAR_GOAL = 0.5                      # 진단 대상: 끝 거리 ≤ 0.5 m 인 실패
DIAG_LAST_STEPS = 20                 # 마지막 2 s
CLASSES = ('phi_blocked', 'policy_stopped', 'mixed')


def dist_stats(x):
    """거리 분포 요약 (n, 중앙, p90, 최대, 평균)."""
    x = np.asarray(x, dtype=np.float64)
    if x.size == 0:
        return {'n': 0}
    return {'n': int(x.size), 'median': float(np.median(x)), 'p90': float(np.percentile(x, 90)),
            'max': float(x.max()), 'mean': float(x.mean())}


def point_speed(v, w, a=HALF_L, b=HALF_W):
    """풋프린트 꼭짓점 최고 속력 (safety_gate.cpp maxPointSpeed, numpy)."""
    v = np.asarray(v, dtype=np.float64)
    w = np.asarray(w, dtype=np.float64)
    best = np.zeros(np.broadcast(v, w).shape)
    for x in (-a, a):
        for y in (-b, b):
            best = np.maximum(best, np.hypot(v - w * y, w * x))
    return best


def layered(ep):
    """
    층위 성공 요약.

    ep: outcome [n] (0 성공 1 충돌 2 시간 상한 3 정체), nav2_reach [n] bool, d_end [n], cost [n, 5].

    반환 dict — strict / nav2 성공률과 표준오차, 최종 거리 분포(전체·실패·정체), 개입 비용 분해.
    """
    out = ep['outcome']
    n = int(out.shape[0])
    succ = out == 0
    nav2 = np.asarray(ep['nav2_reach'], dtype=bool)
    d_end = np.asarray(ep['d_end'], dtype=np.float64)
    p_s = float(succ.mean()) if n else 0.0
    p_n = float(nav2.mean()) if n else 0.0
    iv = np.asarray(ep['cost'])[:, 1]
    res = {
        'n': n,
        'strict_success': p_s, 'strict_se': math.sqrt(p_s * (1 - p_s) / max(n, 1)),
        'nav2_compatible_success': p_n, 'nav2_se': math.sqrt(p_n * (1 - p_n) / max(n, 1)),
        'nav2_but_not_strict': int((nav2 & ~succ).sum()),
        'strict_but_not_nav2': int((succ & ~nav2).sum()),
        'final_dist_all': dist_stats(d_end),
        'final_dist_fail': dist_stats(d_end[~succ]),
        'final_dist_stall': dist_stats(d_end[out == 3]),
        'fail_within_0p5m': int(((~succ) & (d_end <= 0.5)).sum()),
        'fail_within_0p25m': int(((~succ) & (d_end <= 0.25)).sum()),
        'intervention_per_ep': float(iv.mean()) if n else None,
        'intervention_success_eps': float(iv[succ].mean()) if succ.any() else None,
        'intervention_fail_eps': float(iv[~succ].mean()) if (~succ).any() else None,
        'intervention_share_from_fail': float(iv[~succ].sum() / max(iv.sum(), 1e-12)),
    }
    return res


def classify_stop(pairs, steps, last=DIAG_LAST_STEPS):
    """
    한 에피소드의 정지 분류.

    pairs [K, 5, 4] = 마지막 K 결정 스텝 × 5 틱 × (v_ref, w_ref, v_out, w_out) (오래된 것 → 끝),
    steps = 에피소드 결정 스텝 수 (K 보다 짧은 에피소드의 앞 칸은 무효).

    반환 (분류, 지표 dict).
    """
    K = pairs.shape[0]
    n = min(last, K, int(steps))
    if n <= 0:
        return 'mixed', {'ticks': 0}
    p = pairs[K - n:].reshape(-1, 4)
    v_in, w_in, v_out, w_out = p[:, 0], p[:, 1], p[:, 2], p[:, 3]
    moving = (np.abs(v_in) > MOVE_V) | (np.abs(w_in) > MOVE_W)
    u_in = point_speed(v_in, w_in)
    u_out = point_speed(v_out, w_out)
    cut = u_out <= (1.0 - CUT_RATIO) * u_in
    f_block = float((moving & cut).mean())
    f_stop = float((~moving).mean())
    cls = 'phi_blocked' if f_block >= CLASS_FRAC else \
        'policy_stopped' if f_stop >= CLASS_FRAC else 'mixed'
    return cls, {'ticks': int(p.shape[0]), 'frac_blocked': f_block, 'frac_policy_stop': f_stop,
                 'mean_v_in': float(v_in.mean()), 'mean_v_out': float(v_out.mean()),
                 'mean_abs_w_in': float(np.abs(w_in).mean()),
                 'mean_abs_w_out': float(np.abs(w_out).mean())}


def stop_diagnosis(ep):
    """실패 ∧ 끝 거리 ≤ 0.5 m 에피소드의 분류 계수와 에피소드별 기록."""
    out = ep['outcome']
    d_end = ep['d_end']
    idx = np.nonzero((out != 0) & (d_end <= NEAR_GOAL))[0]
    counts = {c: 0 for c in CLASSES}
    rows = []
    for i in idx:
        cls, m = classify_stop(ep['diag_pairs'][i], ep['steps'][i])
        counts[cls] += 1
        last = -1
        rows.append({'i': int(i), 'class': cls, 'outcome': int(out[i]), 'd_end': float(d_end[i]),
                     'path_id_orig': int(ep['path_id_orig'][i]),
                     'D_end': float(ep['diag_D'][i, last]),
                     'guard_end': float(ep['diag_guard'][i, last]),
                     'zone_end': int(ep['diag_zone'][i, last]),
                     'prox_end': bool(ep['diag_prox'][i, last]),
                     'causes_end': int(ep['diag_causes'][i, last]), **m})
    return {'n': int(idx.size), 'counts': counts, 'episodes': rows}


def nav2_update(nav2, got, d_goal, collision, radius=0.25):
    """
    Nav2 호환 도달 누적 (torch) — 결정 스텝 하나.

    nav2·got·collision [N] bool (got = 첫 에피소드가 이미 끝난 env), d_goal [N] (전이 뒤 목표 직선거리).
    첫 에피소드 동안 목표 ≤ radius 인 **충돌이 아닌** 결정 스텝이 한 번이라도 있으면 True (속도·yaw 무관,
    Nav2 SimpleGoalChecker 기본 xy 0.25 m). 첫 에피소드가 끝난 env 는 더 바꾸지 않는다.
    """
    return nav2 | (~got & (d_goal <= radius) & ~collision)


def dwell_success(success, terminated, dwell, k):
    """
    평가 전용 dwell 판정 (torch).

    success·terminated [N] bool (env 의 이번 스텝 판정), dwell [N] long (연속 성공 스텝 수).

    k ≤ 1 이면 그대로. 아니면 성공 조건이 k 스텝 연속일 때만 성공 종단이고, 성공 조건만 맞은 스텝은 종단이 아니다.
    반환 (success', terminated', dwell').
    """
    if k <= 1:
        return success, terminated, dwell
    import torch
    dwell = torch.where(success, dwell + 1, torch.zeros_like(dwell))
    s2 = success & (dwell >= k)
    t2 = (terminated & ~success) | s2
    return s2, t2, dwell


# ======================================================================== 목표 여유 [계산]
def _rect_corners(cx, cy, hx, hy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([(cx + c * x - s * y, cy + s * x + c * y)
                     for x, y in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))])


def _point_rect_dist(p, cx, cy, hx, hy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = p[..., 0] - cx, p[..., 1] - cy
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    return np.hypot(np.maximum(np.abs(lx) - hx, 0.0), np.maximum(np.abs(ly) - hy, 0.0))


def point_clearance(p, obb):
    """점 p (2,) 와 정적 사각형들 obb [M, 5] (cx, cy, hx, hy, yaw) 사이 최소 거리."""
    p = np.asarray(p, dtype=np.float64)
    return float(min(_point_rect_dist(p, *o) for o in obb))


def footprint_clearance(pose, obb, half_l=HALF_L, half_w=HALF_W):
    """
    자세 (x, y, θ) 의 풋프린트와 사각형들 사이 최소 거리 (겹치면 0).

    떨어진 두 볼록 다각형의 최소 거리는 한쪽 꼭짓점과 다른 쪽 사이에서 난다.
    """
    x, y, th = pose
    fp = _rect_corners(x, y, half_l, half_w, th)
    best = math.inf
    for o in obb:
        oc = _rect_corners(*o)
        d1 = _point_rect_dist(fp, *o).min()
        d2 = _point_rect_dist(oc, x, y, half_l, half_w, th).min()
        inter = _rects_overlap(fp, oc)
        best = min(best, 0.0 if inter else float(min(d1, d2)))
    return best


def _rects_overlap(a, b):
    """두 볼록 사각형(꼭짓점 4 개)이 겹치는가 (분리축)."""
    for poly in (a, b):
        for i in range(4):
            e = poly[(i + 1) % 4] - poly[i]
            n = np.array([-e[1], e[0]])
            pa, pb = a @ n, b @ n
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


def straight_approach(pose, obb, half_l=HALF_L, half_w=HALF_W, max_dist=5.0):
    """
    자세 (x, y, θ) 에서 풋프린트를 앞(+x)으로 직진시킬 때 사각형에 처음 닿기까지 거리 (없으면 inf).

    = safety_gate 스윕 풋프린트 직진 접근 거리(SweptFootprint::straightApproach)의 기하값. 볼록 장애물과 직진하는
    사각형의 첫 접촉은 (1) 장애물 꼭짓점이 앞 변(|y| ≤ 반폭)에 닿거나 (2) 앞 모서리 두 점의 광선이 장애물 변에 닿는 것.
    이미 겹치면 0.
    """
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    best = math.inf
    for o in obb:
        oc = _rect_corners(*o)
        dx, dy = oc[:, 0] - x, oc[:, 1] - y
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        loc = np.stack([lx, ly], 1)
        if _rects_overlap(_rect_corners(0, 0, half_l, half_w, 0.0), loc):
            return 0.0
        ok = (np.abs(ly) <= half_w) & (lx >= half_l)
        if ok.any():
            best = min(best, float((lx[ok] - half_l).min()))
        for cy in (-half_w, half_w):
            for i in range(4):
                p0, p1 = loc[i], loc[(i + 1) % 4]
                if (p0[1] - cy) * (p1[1] - cy) > 0 or p0[1] == p1[1]:
                    continue
                t = (cy - p0[1]) / (p1[1] - p0[1])
                xi = p0[0] + t * (p1[0] - p0[0])
                if xi >= half_l:
                    best = min(best, float(xi - half_l))
    return best if best <= max_dist else math.inf


def goal_clearances(path_pts, obb):
    """
    경로 [n, 2] 의 목표 여유 3 종: 목표점–사각형, 끝 접선 자세의 풋프린트–사각형, 그 자세의 직진 접근 거리.

    끝 접선 = 마지막 점과 0.25 m 앞 점의 방향 (경로 점 간격 0.05 m).
    """
    p = np.asarray(path_pts, dtype=np.float64)
    g = p[-1]
    j = max(0, p.shape[0] - 6)
    d = g - p[j]
    th = math.atan2(d[1], d[0]) if np.hypot(*d) > 1e-9 else 0.0
    pose = (float(g[0]), float(g[1]), th)
    return {'goal_point': point_clearance(g, obb),
            'goal_footprint': footprint_clearance(pose, obb),
            'goal_front_straight': straight_approach(pose, obb), 'goal_heading': th}
