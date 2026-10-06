"""
배치 LiDAR 레이캐스팅 — 정적 격자(DDA, 셀 경계 정확 교차) + 동적 장애물(원·회전 사각형 해석해).

격자 레이캐스트는 두 구현이 있다:
  - raycast_grid_torch: 순수 torch (모든 광선을 함께 한 걸음씩). 기준·CPU 용.
  - raycast_grid_triton: Triton 커널 (광선 하나 = 레인 하나, while 루프로 블록 조기 종료). GPU 기본값.
시험 tests/test_raycast.py 가 둘을 해석해(축정렬 벽·원·사각형)와 서로 대조한다.

좌표: 격자 단위 위치 (x, y) = ((wx − ox)/res, (wy − oy)/res). 셀 (cx, cy) = (floor x, floor y).
지도 밖은 막힘(코스트맵 atOrLethal 과 같다). 시작 셀이 막혀 있으면 거리 0.
"""

import math

import torch

from ._contract import C

try:
    import triton
    import triton.language as tl
    HAVE_TRITON = True
except ImportError:      # pragma: no cover
    HAVE_TRITON = False

INF = math.inf


# ------------------------------------------------------------------ 격자 DDA (torch)
def raycast_grid_torch(grid, map_idx, x0, y0, dx, dy, max_t):
    """
    격자 DDA — grid [M,H,W] uint8, map_idx [N] long, x0·y0 [N,R] 격자 단위 원점, dx·dy [N,R] 단위 방향.

    반환: [N,R] 첫 막힌 셀 진입 거리(격자 단위), 없으면 inf. max_t 를 넘으면 inf.
    """
    M, H, W = grid.shape
    flat = grid.reshape(-1)
    base = (map_idx * (H * W))[:, None].expand_as(x0)
    cx = torch.floor(x0).long()
    cy = torch.floor(y0).long()
    sx = torch.where(dx > 0, 1, -1)
    sy = torch.where(dy > 0, 1, -1)
    inf = torch.full_like(x0, INF)
    tdx = torch.where(dx != 0, 1.0 / dx.abs(), inf)
    tdy = torch.where(dy != 0, 1.0 / dy.abs(), inf)
    fx = x0 - torch.floor(x0)
    fy = y0 - torch.floor(y0)
    tmx = torch.where(dx > 0, (1.0 - fx) * tdx, torch.where(dx < 0, fx * tdx, inf))
    tmy = torch.where(dy > 0, (1.0 - fy) * tdy, torch.where(dy < 0, fy * tdy, inf))
    t = torch.zeros_like(x0)
    out = inf.clone()
    active = torch.ones_like(x0, dtype=torch.bool)
    max_steps = int(2 * math.ceil(max_t)) + 4
    for _ in range(max_steps):
        inb = (cx >= 0) & (cx < W) & (cy >= 0) & (cy < H)
        idx = base + cy.clamp(0, H - 1) * W + cx.clamp(0, W - 1)
        occ = torch.where(inb, flat[idx] != 0, torch.ones_like(active))
        hit = active & occ & (t <= max_t)
        out = torch.where(hit, t, out)
        active = active & ~hit & (t <= max_t)
        if not bool(active.any()):
            break
        gx = tmx < tmy
        t = torch.where(gx, tmx, tmy)
        cx = torch.where(gx, cx + sx, cx)
        tmx = torch.where(gx, tmx + tdx, tmx)
        cy = torch.where(gx, cy, cy + sy)
        tmy = torch.where(gx, tmy, tmy + tdy)
    return out


# ------------------------------------------------------------------ 격자 DDA (Triton)
if HAVE_TRITON:
    @triton.jit
    def _dda_kernel(grid_ptr, base_ptr, x_ptr, y_ptr, dx_ptr, dy_ptr, out_ptr, n, H, W,
                    max_t, BLOCK: tl.constexpr):
        pid = tl.program_id(0)
        offs = pid * BLOCK + tl.arange(0, BLOCK)
        m = offs < n
        x0 = tl.load(x_ptr + offs, mask=m, other=0.0)
        y0 = tl.load(y_ptr + offs, mask=m, other=0.0)
        dx = tl.load(dx_ptr + offs, mask=m, other=1.0)
        dy = tl.load(dy_ptr + offs, mask=m, other=0.0)
        base = tl.load(base_ptr + offs, mask=m, other=0)
        fx0 = tl.floor(x0)
        fy0 = tl.floor(y0)
        cx = fx0.to(tl.int32)
        cy = fy0.to(tl.int32)
        sx = tl.where(dx > 0, 1, -1)
        sy = tl.where(dy > 0, 1, -1)
        big = 1e30
        adx = tl.abs(dx)
        ady = tl.abs(dy)
        tdx = tl.where(adx > 0, 1.0 / tl.where(adx > 0, adx, 1.0), big)
        tdy = tl.where(ady > 0, 1.0 / tl.where(ady > 0, ady, 1.0), big)
        fx = x0 - fx0
        fy = y0 - fy0
        tmx = tl.where(dx > 0, (1.0 - fx) * tdx, tl.where(dx < 0, fx * tdx, big))
        tmy = tl.where(dy > 0, (1.0 - fy) * tdy, tl.where(dy < 0, fy * tdy, big))
        t = tl.zeros([BLOCK], dtype=tl.float32)
        out = tl.full([BLOCK], big, dtype=tl.float32)
        active = m
        n_active = tl.sum(active.to(tl.int32), axis=0)
        while n_active > 0:
            inb = (cx >= 0) & (cx < W) & (cy >= 0) & (cy < H)
            ccx = tl.minimum(tl.maximum(cx, 0), W - 1)
            ccy = tl.minimum(tl.maximum(cy, 0), H - 1)
            occ = tl.load(grid_ptr + base + ccy * W + ccx, mask=active & inb, other=1)
            occ = (occ != 0) | (inb == 0)
            hit = active & occ & (t <= max_t)
            out = tl.where(hit, t, out)
            active = active & (hit == 0) & (t <= max_t)
            gx = tmx < tmy
            t = tl.where(gx, tmx, tmy)
            cx = tl.where(gx, cx + sx, cx)
            tmx = tl.where(gx, tmx + tdx, tmx)
            cy = tl.where(gx, cy, cy + sy)
            tmy = tl.where(gx, tmy, tmy + tdy)
            n_active = tl.sum(active.to(tl.int32), axis=0)
        tl.store(out_ptr + offs, out, mask=m)


def raycast_grid_triton(grid, map_idx, x0, y0, dx, dy, max_t, block=128):
    """raycast_grid_torch 와 같은 의미의 Triton 구현 (float32, CUDA)."""
    M, H, W = grid.shape
    shape = x0.shape
    base = (map_idx.to(torch.int64) * (H * W))[:, None].expand(shape).reshape(-1).contiguous()
    xf = x0.reshape(-1).float().contiguous()
    yf = y0.reshape(-1).float().contiguous()
    dxf = dx.reshape(-1).float().contiguous()
    dyf = dy.reshape(-1).float().contiguous()
    out = torch.empty_like(xf)
    n = xf.numel()
    grid_fn = (triton.cdiv(n, block),)
    _dda_kernel[grid_fn](grid, base, xf, yf, dxf, dyf, out, n, H, W, float(max_t), BLOCK=block)
    out = torch.where(out >= 1e29, torch.full_like(out, INF), out)
    return out.reshape(shape)


# ------------------------------------------------------------------ 동적 장애물
def raycast_obstacles(ox, oy, dx, dy, obs):
    """
    광선 [N,R] (월드 원점 ox·oy [N,1], 단위 방향 dx·dy [N,R]) 과 장애물 obs 의 첫 교차 거리 [N,R].

    obs: dict — 'xy' [N,K,2], 'yaw' [N,K], 'kind' [N,K] (0 원, 1 사각), 'r' [N,K],
    'box' [N,K,4] (x_min, x_max, y_min, y_max 몸체 좌표), 'active' [N,K] bool.
    원점이 장애물 안이면 0.
    """
    N, R = dx.shape
    K = obs['xy'].shape[1]
    out = torch.full((N, R), INF, dtype=dx.dtype, device=dx.device)
    if K == 0:
        return out
    cx = obs['xy'][..., 0].to(dx.dtype)[:, None, :]          # [N,1,K]
    cy = obs['xy'][..., 1].to(dx.dtype)[:, None, :]
    fx = ox[..., None] - cx                                   # [N,1,K]
    fy = oy[..., None] - cy
    ddx = dx[..., None]
    ddy = dy[..., None]
    # 원
    r = obs['r'].to(dx.dtype)[:, None, :]
    b = fx * ddx + fy * ddy                                   # [N,R,K]
    cc = (fx * fx + fy * fy) - r * r
    disc = b * b - cc
    t_c = -b - torch.sqrt(disc.clamp_min(0.0))
    t_c = torch.where((disc >= 0) & (t_c >= 0), t_c, torch.full_like(t_c, INF))
    t_c = torch.where(cc <= 0, torch.zeros_like(t_c), t_c)
    # 회전 사각형 (slab)
    c = torch.cos(obs['yaw']).to(dx.dtype)[:, None, :]
    s = torch.sin(obs['yaw']).to(dx.dtype)[:, None, :]
    lox = c * fx + s * fy
    loy = -s * fx + c * fy
    ldx = c * ddx + s * ddy
    ldy = -s * ddx + c * ddy
    box = obs['box'].to(dx.dtype)
    x_lo, x_hi = box[..., 0][:, None, :], box[..., 1][:, None, :]
    y_lo, y_hi = box[..., 2][:, None, :], box[..., 3][:, None, :]

    def slab(o, d, lo, hi):
        small = d.abs() < 1e-12
        d_s = torch.where(small, torch.ones_like(d), d)
        t1 = (lo - o) / d_s
        t2 = (hi - o) / d_s
        tn = torch.minimum(t1, t2)
        tf = torch.maximum(t1, t2)
        inside = (o >= lo) & (o <= hi)
        tn = torch.where(small, torch.where(inside, torch.full_like(tn, -INF),
                                            torch.full_like(tn, INF)), tn)
        tf = torch.where(small, torch.where(inside, torch.full_like(tf, INF),
                                            torch.full_like(tf, -INF)), tf)
        return tn, tf
    tnx, tfx = slab(lox, ldx, x_lo, x_hi)
    tny, tfy = slab(loy, ldy, y_lo, y_hi)
    tn = torch.maximum(tnx, tny)
    tf = torch.minimum(tfx, tfy)
    t_r = torch.where((tn <= tf) & (tf >= 0), tn.clamp_min(0.0), torch.full_like(tn, INF))
    kind = obs['kind'][:, None, :]
    t = torch.where(kind == 0, t_c, t_r)
    t = torch.where(obs['active'][:, None, :], t, torch.full_like(t, INF))
    return t.min(dim=-1).values


# ------------------------------------------------------------------ 스캔 합성
_BEAM = {}


def beam_angles(device):
    """빔 각 [720] float32 (계약 constants 의 angle_min + i·inc)."""
    key = str(device)
    if key not in _BEAM:
        i = torch.arange(C.LIDAR_SAMPLES, dtype=torch.float64)
        _BEAM[key] = (C.LIDAR_ANGLE_MIN + i * C.LIDAR_ANGLE_INC).to(device, torch.float32)
    return _BEAM[key]


def lidar_origin(pose):
    """센서 원점의 월드 좌표 (LiDAR, base_link (0.15, 0))."""
    c, s = torch.cos(pose[:, 2]), torch.sin(pose[:, 2])
    return pose[:, 0] + C.LIDAR_X * c - C.LIDAR_Y * s, pose[:, 1] + C.LIDAR_X * s + C.LIDAR_Y * c


def cast_scan(grid, map_meta, map_idx, pose, obs=None, noise_std=None, dropout=None,
              generator=None, use_triton=None):
    """
    자세 pose [N,3] 에서 720 빔 스캔 [N,720] float32 (inf = 무반사·무효).

    map_meta [M,4] (res, ox, oy, _). noise_std·dropout: [N] 또는 None (DR 축 5).
    거리 = min(격자 DDA, 장애물) → 잡음 → [range_min, range_max] 밖·드롭아웃은 inf.
    """
    if use_triton is None:
        use_triton = HAVE_TRITON and pose.is_cuda
    ang = beam_angles(pose.device)
    lx, ly = lidar_origin(pose)
    th = pose[:, 2].float()[:, None] + ang[None, :]
    dx = torch.cos(th)
    dy = torch.sin(th)
    meta = map_meta[map_idx]
    res = meta[:, 0].float()[:, None]
    gx = ((lx - meta[:, 1]) / meta[:, 0]).float()[:, None].expand_as(dx)
    gy = ((ly - meta[:, 2]) / meta[:, 0]).float()[:, None].expand_as(dx)
    max_cells = float(C.LIDAR_RANGE_MAX / float(map_meta[0, 0]))
    fn = raycast_grid_triton if use_triton else raycast_grid_torch
    r = fn(grid, map_idx, gx, gy, dx, dy, max_cells) * res
    if obs is not None and obs['xy'].shape[1] > 0:
        r_o = raycast_obstacles(lx.float()[:, None], ly.float()[:, None], dx, dy, obs)
        r = torch.minimum(r, r_o)
    if noise_std is not None:
        nz = torch.randn(r.shape, device=r.device, generator=generator)
        nz = nz * noise_std.float()[:, None]
        r = torch.where(torch.isfinite(r), r + nz, r)
    valid = torch.isfinite(r) & (r >= C.LIDAR_RANGE_MIN) & (r <= C.LIDAR_RANGE_MAX)
    if dropout is not None:
        drop = torch.rand(r.shape, device=r.device, generator=generator) < dropout.float()[:, None]
        valid = valid & ~drop
    return torch.where(valid, r, torch.full_like(r, INF))
