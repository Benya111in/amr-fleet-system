"""
지면 진실 충돌 판정과 safety gate 의 스윕 접근 거리 D — 배치 torch.

충돌 (contract.md §8: 접촉 = 즉시 종료, 경계 포함):
  - 로봇 풋프린트(0.60×0.40, base_link 중심 회전 사각형) vs 막힌 격자 셀(정사각형): 분리축 정리(SAT) 정확 판정.
  - vs 원 장애물: 원 중심의 풋프린트 여유거리 ≤ r.
  - vs 회전 사각형 장애물: 두 OBB 의 SAT.
스윕 접근 거리: src/amr_perception/src/safety_gate.cpp SweptFootprint::approach (직선·원호·꼬리) 이식.
  빔 창 순위 통계(robustBeams)·지연 보정·접촉 가드·도킹 예외는 옮기지 않았다 — D = 유효 빔 최솟값.
"""

import math

import torch

from ._contract import C

INF = math.inf
TWO_PI = 2.0 * math.pi

# safety_gate.hpp SafetyParams 기본값 (robot_params.yaml 의 같은 키가 이 값을 쓴다)
SAFETY = {
    'emergency_stop_distance': C.E_STOP_DIST,     # R:121
    'warning_zone_distance': C.WARNING_DIST,      # R:123
    'stop_release_distance': 0.50,                # safety_gate.hpp:100
    'zone_hysteresis': 0.05,                      # safety_gate.hpp:101
    'swept_margin': 0.0,                          # safety_gate.hpp:115
    'region_margin': 0.10,                        # safety_gate.hpp:116
    'motion_epsilon_v': 0.01,                     # safety_gate.hpp:117
    'motion_epsilon_w': 0.05,                     # safety_gate.hpp:118
    'reaction_latency': C.REACTION_LATENCY,       # R:135
    'max_deceleration': C.SAFETY_DECEL,
}
_WIN = {}


def _window(device, half):
    key = (str(device), half)
    if key not in _WIN:
        r = torch.arange(-half, half + 1, device=device)
        oy, ox = torch.meshgrid(r, r, indexing='ij')
        _WIN[key] = (ox.reshape(-1), oy.reshape(-1))
    return _WIN[key]


def rect_hits_grid(grid, map_meta, map_idx, pose, half_l=C.HALF_L, half_w=C.HALF_W):
    """
    풋프린트가 막힌 셀과 겹치는가 [N] bool (경계 접촉 포함, 지도 밖 = 막힘).

    중심 셀 둘레 (2h+1)² 셀을 SAT 4 축(세계 x·y, 로봇 u·v)으로 검사한다. h = ceil(외접 반경/res) + 1.
    """
    M, H, W = grid.shape
    meta = map_meta[map_idx]
    res = meta[:, 0]
    half = int(math.ceil(math.hypot(half_l, half_w) / float(map_meta[0, 0]))) + 1
    ox_, oy_ = _window(pose.device, half)
    x, y, th = pose[:, 0], pose[:, 1], pose[:, 2]
    ccx = torch.floor((x - meta[:, 1]) / res).long()
    ccy = torch.floor((y - meta[:, 2]) / res).long()
    cx = ccx[:, None] + ox_[None, :]
    cy = ccy[:, None] + oy_[None, :]
    inb = (cx >= 0) & (cx < W) & (cy >= 0) & (cy < H)
    flat = grid.reshape(-1)
    idx = map_idx[:, None] * (H * W) + cy.clamp(0, H - 1) * W + cx.clamp(0, W - 1)
    solid = torch.where(inb, flat[idx] != 0, torch.ones_like(inb))
    h = 0.5 * res[:, None]
    qx = meta[:, 1][:, None] + (cx.to(x.dtype) + 0.5) * res[:, None] - x[:, None]
    qy = meta[:, 2][:, None] + (cy.to(x.dtype) + 0.5) * res[:, None] - y[:, None]
    c = torch.cos(th)[:, None]
    s = torch.sin(th)[:, None]
    ac, as_ = c.abs(), s.abs()
    eps = 1e-9
    ok_x = qx.abs() <= h + half_l * ac + half_w * as_ + eps
    ok_y = qy.abs() <= h + half_l * as_ + half_w * ac + eps
    pu = c * qx + s * qy
    pv = -s * qx + c * qy
    ok_u = pu.abs() <= half_l + h * (ac + as_) + eps
    ok_v = pv.abs() <= half_w + h * (ac + as_) + eps
    return (solid & ok_x & ok_y & ok_u & ok_v).any(dim=1)


def rect_hits_obstacles(pose, obs, half_l=C.HALF_L, half_w=C.HALF_W):
    """풋프린트 vs 활성 장애물 (원·회전 사각형) 접촉 [N] bool."""
    K = obs['xy'].shape[1]
    if K == 0:
        return torch.zeros(pose.shape[0], dtype=torch.bool, device=pose.device)
    x, y, th = pose[:, 0:1], pose[:, 1:2], pose[:, 2:3]
    c, s = torch.cos(th), torch.sin(th)
    ox = obs['xy'][..., 0].to(pose.dtype)
    oy = obs['xy'][..., 1].to(pose.dtype)
    dx, dy = ox - x, oy - y
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    clr = torch.hypot((lx.abs() - half_l).clamp_min(0.0), (ly.abs() - half_w).clamp_min(0.0))
    hit_c = clr <= obs['r'].to(pose.dtype)
    # OBB vs OBB
    box = obs['box'].to(pose.dtype)
    cb, sb = torch.cos(obs['yaw']).to(pose.dtype), torch.sin(obs['yaw']).to(pose.dtype)
    bcx_l = 0.5 * (box[..., 0] + box[..., 1])
    bcy_l = 0.5 * (box[..., 2] + box[..., 3])
    bh_l = 0.5 * (box[..., 1] - box[..., 0])
    bh_w = 0.5 * (box[..., 3] - box[..., 2])
    bcx = ox + cb * bcx_l - sb * bcy_l - x
    bcy = oy + sb * bcx_l + cb * bcy_l - y
    axes = [(c, s), (-s, c), (cb, sb), (-sb, cb)]
    sep = torch.zeros_like(hit_c)
    for axx, axy in axes:
        ra = half_l * (c * axx + s * axy).abs() + half_w * (-s * axx + c * axy).abs()
        rb = bh_l * (cb * axx + sb * axy).abs() + bh_w * (-sb * axx + cb * axy).abs()
        dist = (bcx * axx + bcy * axy).abs()
        sep = sep | (dist > ra + rb + 1e-9)
    hit_r = ~sep
    hit = torch.where(obs['kind'] == 0, hit_c, hit_r) & obs['active']
    return hit.any(dim=1)


def footprint_obstacle_distance(pose, obs, half_l=C.HALF_L, half_w=C.HALF_W):
    """
    풋프린트 사각형과 활성 장애물 몸체 사이 최소 거리 [N] (장애물 없음 = inf, 진단용 — 학습 신호 아님).

    원: 사각형-점 거리 − r. 사각형: 두 볼록 다각형이 떨어져 있으면 최소 거리는 한쪽 꼭짓점과 다른 쪽 사이에서
    나므로 (꼭짓점 8 개 → 상대 사각형) 의 최솟값이다. 겹치면 값이 0 이 아닐 수 있다 — env 는 충돌한 스텝을 0 으로
    덮는다.
    """
    K = obs['xy'].shape[1]
    if K == 0:
        return torch.full((pose.shape[0],), float('inf'), dtype=pose.dtype, device=pose.device)
    x, y, th = pose[:, 0:1], pose[:, 1:2], pose[:, 2:3]
    c, s = torch.cos(th), torch.sin(th)
    ox = obs['xy'][..., 0].to(pose.dtype)
    oy = obs['xy'][..., 1].to(pose.dtype)

    def to_robot_box(wx, wy):
        dx, dy = wx - x[..., None], wy - y[..., None]
        lx = c[..., None] * dx + s[..., None] * dy
        ly = -s[..., None] * dx + c[..., None] * dy
        return torch.hypot((lx.abs() - half_l).clamp_min(0.0), (ly.abs() - half_w).clamp_min(0.0))
    d_circ = to_robot_box(ox[..., None], oy[..., None])[..., 0] - obs['r'].to(pose.dtype)
    box = obs['box'].to(pose.dtype)
    cb, sb = torch.cos(obs['yaw']).to(pose.dtype), torch.sin(obs['yaw']).to(pose.dtype)
    sx = torch.tensor([1.0, 1.0, -1.0, -1.0], dtype=pose.dtype, device=pose.device)
    sy = torch.tensor([1.0, -1.0, 1.0, -1.0], dtype=pose.dtype, device=pose.device)
    # 로봇 꼭짓점 [N,1,4] → 장애물 좌표 → 장애물 상자까지
    rcx = x + c * (sx * half_l) - s * (sy * half_w)
    rcy = y + s * (sx * half_l) + c * (sy * half_w)
    rx = rcx[:, None, :] - ox[..., None]
    ry = rcy[:, None, :] - oy[..., None]
    bx = cb[..., None] * rx + sb[..., None] * ry
    by = -sb[..., None] * rx + cb[..., None] * ry
    qx = torch.minimum(torch.maximum(bx, box[..., 0:1]), box[..., 1:2])
    qy = torch.minimum(torch.maximum(by, box[..., 2:3]), box[..., 3:4])
    d1 = torch.hypot(bx - qx, by - qy).min(dim=-1).values
    # 장애물 꼭짓점 [N,K,4] → 로봇 상자까지
    ex = torch.stack([box[..., 1], box[..., 1], box[..., 0], box[..., 0]], -1)
    ey = torch.stack([box[..., 3], box[..., 2], box[..., 3], box[..., 2]], -1)
    wx = ox[..., None] + cb[..., None] * ex - sb[..., None] * ey
    wy = oy[..., None] + sb[..., None] * ex + cb[..., None] * ey
    d2 = to_robot_box(wx, wy).min(dim=-1).values
    d = torch.where(obs['kind'] == 0, d_circ, torch.minimum(d1, d2)).clamp_min(0.0)
    d = torch.where(obs['active'], d, torch.full_like(d, float('inf')))
    return d.min(dim=1).values


# ------------------------------------------------------------------ 스윕 접근 거리
def _max_point_speed(a, b, v, w):
    best = torch.zeros_like(v)
    for x in (-a, a):
        for y in (-b, b):
            best = torch.maximum(best, torch.hypot(v - w * y, w * x))
    return best


def swept_approach(px, py, valid, v, w, p=SAFETY):
    """
    운동 (v, w) [N] 의 스윕 풋프린트 접근 거리 [N] — 유효 점 (px, py) [N,B] (로봇 좌표) 최솟값.

    safety_gate.cpp:67-195 SweptFootprint (생성자 + approach) 와 sweptRegion(:334-347) 의 이식.
    """
    a = C.HALF_L
    b = C.HALF_W + max(p['swept_margin'], 0.0)
    b_real = C.HALF_W
    u = _max_point_speed(a, b, v, w)
    brake = u * p['reaction_latency'] + u * u / (2.0 * p['max_deceleration'])
    envelope = p['emergency_stop_distance'] + brake
    total = torch.clamp(envelope, min=p['warning_zone_distance'] + p['zone_hysteresis']) + \
        p['region_margin']
    linear = v.abs() > p['motion_epsilon_v']
    rotating = w.abs() > p['motion_epsilon_w']
    empty = (~linear & ~rotating) | (u <= 0.0)
    w_safe = torch.where(w.abs() > 0.0, w, torch.ones_like(w))
    straight = (w.abs() < 1e-12) | ((v / w_safe).abs() > 1.0e5)
    R = torch.where(straight, torch.zeros_like(v), v / w_safe)
    u_safe = torch.where(u > 0.0, u, torch.ones_like(u))
    arc_time = torch.where(linear, torch.minimum(envelope, total), total) / u_safe
    arc_angle = torch.clamp(w.abs() * arc_time, max=TWO_PI)
    has_tail = linear & (envelope < total) & ~straight
    th_t = w * arc_time
    tx = R * torch.sin(th_t)
    ty = R * (1.0 - torch.cos(th_t))
    tail_off = v.abs() * arc_time
    vpos = v > 0.0

    def col(t):
        return t[:, None]

    def straight_approach(qx, qy, bb, vp):
        ahead = torch.where(vp, qx, -qx) - a
        ok = (qy.abs() <= bb) & (ahead >= 0.0)
        return torch.where(ok, ahead, torch.full_like(ahead, INF))

    beside = px.abs() <= a
    in_fp = beside & (py.abs() <= b_real)
    bb = torch.where(beside & (py.abs() <= b), torch.full_like(py, b_real), torch.full_like(py, b))
    s_str = straight_approach(px, py, bb, col(vpos))
    # 원호
    Rc = col(R)
    dyc = py - Rc
    rho = torch.hypot(px, dyc)
    th0 = torch.atan2(dyc, px)
    dirc = torch.where(col(w) > 0.0, -1.0, 1.0).to(px.dtype)
    best = torch.full_like(px, INF)
    rho2 = rho * rho

    def consider(best, x, y, ok):
        delta = torch.fmod(dirc * (torch.atan2(y - Rc, x) - th0), TWO_PI)
        delta = torch.where(delta < 0.0, delta + TWO_PI, delta)
        return torch.where(ok, torch.minimum(best, delta), best)
    for sx in (-a, a):
        h2 = rho2 - sx * sx
        h = torch.sqrt(h2.clamp_min(0.0))
        for y in (Rc + h, Rc - h):
            best = consider(best, torch.full_like(px, sx), y, (h2 >= 0.0) & (y.abs() <= bb))
    for sgn in (-1.0, 1.0):
        sy = sgn * bb
        h2 = rho2 - (sy - Rc) * (sy - Rc)
        h = torch.sqrt(h2.clamp_min(0.0))
        for x in (h, -h):
            best = consider(best, x, sy, (h2 >= 0.0) & (x.abs() <= a))
    s_arc = torch.where((best <= col(arc_angle)) & (rho >= 1e-9), rho * best,
                        torch.full_like(px, INF))
    # 꼬리: 원호 끝 자세에서 접선 직선
    c_t = torch.cos(col(th_t))
    s_t = torch.sin(col(th_t))
    qx = c_t * (px - col(tx)) + s_t * (py - col(ty))
    qy = -s_t * (px - col(tx)) + c_t * (py - col(ty))
    ahead = straight_approach(qx, qy, bb, col(vpos))
    s_tail = torch.where(col(has_tail) & torch.isfinite(ahead), col(tail_off) + ahead,
                         torch.full_like(px, INF))
    s_arc = torch.where(torch.isfinite(s_arc), s_arc, s_tail)
    s = torch.where(col(straight), s_str, s_arc)
    s = torch.where(s <= col(total), s, torch.full_like(s, INF))
    s = torch.where(in_fp, torch.zeros_like(s), s)
    s = torch.where(col(empty) | ~valid, torch.full_like(s, INF), s)
    return s.min(dim=1).values
