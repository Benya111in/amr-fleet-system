"""
동적 장애물 — 비반응 에이전트 (contract.md §10 "기본은 비반응").

모양 (src/amr_simulation/config/dynamic_obstacles.yaml):
  - 작업자: 원 r 0.30 (actor_radius)
  - 지게차: 사각 x −1.2…2.0 × y ±0.64 (model_footprints[0:4]), 진행 축 = 몸체 x
  - 다른 AMR(셔틀): 사각 ±0.30 × ±0.20 (model_footprints[4:8])
운동 (1 차: 등속 — Gazebo 의 사다리꼴 가감속·지게차 램프는 옮기지 않았다):
  - 왕복 선분 A↔B (끝에서 즉시 반전, 몸체는 돌지 않는다 — gen_warehouse_world.py 의 전진·후진 왕복과 같다)
  - 원 순환 (중심, 반경, 각속도)
배치 방식:
  - 'random': 로봇 A* 경로를 가로지르거나 마주 오는 선분을 env 마다 뽑는다 (DR 축 3).
  - 'gazebo': 팀 Gazebo 월드의 7 개 (gen_warehouse_world.py:28-35, worker_random 제외) 를 무작위 위상으로.
"""

import math

import torch

from . import contract_torch as T

KIND_CIRCLE, KIND_RECT = 0, 1
TYPE_WORKER, TYPE_FORKLIFT, TYPE_AMR = 0, 1, 2
MOT_PINGPONG, MOT_CIRCLE = 0, 1
WORKER_R = 0.30
FORKLIFT_BOX = (-1.2, 2.0, -0.64, 0.64)
AMR_BOX = (-0.30, 0.30, -0.20, 0.20)

# (이름, 종류, 운동, A 또는 중심, B 또는 (반경, 0), 속력) — gen_warehouse_world.py:28-35, :642-726
GAZEBO_AGENTS = (
    ('worker_straight_slow', TYPE_WORKER, MOT_PINGPONG, (-16.0, 6.0), (16.0, 6.0), 0.5),
    ('worker_straight_fast', TYPE_WORKER, MOT_PINGPONG, (22.0, -10.0), (22.0, 10.0), 1.2),
    ('worker_curve', TYPE_WORKER, MOT_CIRCLE, (-24.0, 0.0), (3.5, 0.0), 0.8),
    ('worker_crossing', TYPE_WORKER, MOT_PINGPONG, (-6.0, -7.0), (6.0, -7.0), 1.0),
    ('worker_slow_south', TYPE_WORKER, MOT_PINGPONG, (-6.0, -14.5), (6.0, -14.5), 0.3),
    ('forklift_main', TYPE_FORKLIFT, MOT_PINGPONG, (-20.0, 15.0), (20.0, 15.0), 1.5),
    ('shuttle_amr', TYPE_AMR, MOT_PINGPONG, (9.0, -6.0), (9.0, 11.0), 1.0),
)


class Obstacles:
    """env N × 슬롯 K 의 장애물 상태 (GPU 텐서)."""

    def __init__(self, n, k, device, dtype=torch.float64):
        """모든 슬롯 비활성."""
        self.n, self.k, self.device, self.dtype = n, k, device, dtype

        def z(*s, dt=dtype):
            return torch.zeros(n, k, *s, dtype=dt, device=device)
        self.active = z(dt=torch.bool)
        self.otype = z(dt=torch.long)
        self.kind = z(dt=torch.long)
        self.r = z()
        self.box = z(4)
        self.mot = z(dt=torch.long)
        self.a = z(2)
        self.b = z(2)
        self.speed = z()
        self.phase = z()           # 시간 오프셋 [s]

    def reset_slots(self, idx):
        """지정한 env 의 모든 슬롯을 비활성으로."""
        self.active[idx] = False

    def set_type(self, otype):
        """종류 otype [n,k] 에 맞춰 kind·r·box 를 채운다."""
        self.otype = otype
        self.kind = torch.where(otype == TYPE_WORKER, KIND_CIRCLE, KIND_RECT)
        self.r = torch.where(otype == TYPE_WORKER, torch.full_like(self.r, WORKER_R),
                             torch.zeros_like(self.r))
        fb = torch.tensor(FORKLIFT_BOX, dtype=self.dtype, device=self.device)
        ab = torch.tensor(AMR_BOX, dtype=self.dtype, device=self.device)
        self.box = torch.where((otype == TYPE_FORKLIFT)[..., None], fb, ab)

    def state(self, t):
        """시각 t [n] → dict(xy, vel, yaw, kind, r, box, active) (raycast·충돌·트랙 입력)."""
        tt = t[:, None] + self.phase
        ab = self.b - self.a
        L = torch.linalg.norm(ab, dim=-1)
        Ls = torch.where(L > 1e-6, L, torch.ones_like(L))
        u = ab / Ls[..., None]
        dist = torch.remainder(self.speed * tt, 2.0 * Ls)
        fwd = dist <= Ls
        d = torch.where(fwd, dist, 2.0 * Ls - dist)
        sgn = torch.where(fwd, 1.0, -1.0).to(self.dtype)
        moving = (L > 1e-6) & (self.speed > 0)
        xy_p = self.a + torch.where(moving, d, torch.zeros_like(d))[..., None] * u
        vel_p = torch.where(moving, sgn * self.speed, torch.zeros_like(d))[..., None] * u
        yaw_p = torch.where(L > 1e-6, torch.atan2(u[..., 1], u[..., 0]), torch.zeros_like(L))
        # 원 순환: a = 중심, b[...,0] = 반경
        rc = self.b[..., 0]
        rcs = torch.where(rc > 1e-6, rc, torch.ones_like(rc))
        om = self.speed / rcs
        ang = om * tt
        xy_c = self.a + rc[..., None] * torch.stack([torch.cos(ang), torch.sin(ang)], -1)
        vel_c = (rc * om)[..., None] * torch.stack([-torch.sin(ang), torch.cos(ang)], -1)
        circ = (self.mot == MOT_CIRCLE)[..., None]
        xy = torch.where(circ, xy_c, xy_p)
        vel = torch.where(circ, vel_c, vel_p)
        yaw = torch.where(self.mot == MOT_CIRCLE, ang + 0.5 * math.pi, yaw_p)
        return {'xy': xy, 'vel': vel, 'yaw': yaw, 'kind': self.kind, 'r': self.r,
                'box': self.box, 'active': self.active}

    # ------------------------------------------------------------ 배치
    def place_gazebo(self, idx, t0, gen):
        """지정한 env 에 Gazebo 월드 7 개를 무작위 위상으로."""
        m = len(GAZEBO_AGENTS)
        if self.k < m:
            raise ValueError(f'슬롯 {self.k} < Gazebo 장애물 {m}')
        n = idx.numel()
        dev, dt = self.device, self.dtype
        ot = torch.tensor([g[1] for g in GAZEBO_AGENTS], device=dev)
        mot = torch.tensor([g[2] for g in GAZEBO_AGENTS], device=dev)
        a = torch.tensor([g[3] for g in GAZEBO_AGENTS], dtype=dt, device=dev)
        b = torch.tensor([g[4] for g in GAZEBO_AGENTS], dtype=dt, device=dev)
        sp = torch.tensor([g[5] for g in GAZEBO_AGENTS], dtype=dt, device=dev)
        otype = self.otype.clone()
        otype[idx, :m] = ot
        self.set_type(otype)
        self.mot[idx, :m] = mot
        self.a[idx, :m] = a
        self.b[idx, :m] = b
        self.speed[idx, :m] = sp
        self.phase[idx, :m] = torch.rand(n, m, generator=gen, device=dev, dtype=dt) * 120.0
        self.active[idx] = False
        self.active[idx, :m] = True

    def place_random(self, idx, edt, map_meta, map_idx, path, start_xy, count, gen,
                     p_type=(0.7, 0.15, 0.15), speeds=((0.3, 1.5), (1.0, 1.5), (0.5, 1.0)),
                     half_len=(2.0, 6.0), min_start_dist=2.0):
        """
        지정한 env 에 경로를 가로지르는 왕복 장애물을 count [n] 개씩.

        경로 호길이 U[1, L] 점에서 진행 방향 기준 ±90°±45°(70 %) 또는 마주 오기(30 %) 선분을 잡고, 격자 EDT 로
        양 끝을 자유 공간 안으로 자른다 (몸체 반폭 + 0.05 m 여유). 선분이 1 m 보다 짧으면 서 있는 장애물(속력 0).
        t = 0 에 몸체 원점이 시작점에서 min_start_dist + 몸체 최대 반경 안이면 비활성.
        """
        n = idx.numel()
        if n == 0:
            return
        dev, dt, K = self.device, self.dtype, self.k
        pts, pn, cum, plen = path
        u = torch.rand(n, K, generator=gen, device=dev, dtype=dt)
        s = 1.0 + u * (plen[:, None] - 1.0).clamp_min(0.0)
        c = T.interpolate_at(pts, pn, cum, s)                                  # [n,K,2]
        c2 = T.interpolate_at(pts, pn, cum, s + 0.25)
        tang = c2 - c
        tyaw = torch.atan2(tang[..., 1], tang[..., 0])
        cross = torch.rand(n, K, generator=gen, device=dev, dtype=dt) < 0.7
        side = torch.where(torch.rand(n, K, generator=gen, device=dev) < 0.5, 1.0, -1.0).to(dt)
        jit = (torch.rand(n, K, generator=gen, device=dev, dtype=dt) - 0.5) * (0.5 * math.pi)
        ang = torch.where(cross, tyaw + side * 0.5 * math.pi + jit, tyaw + math.pi + 0.3 * jit)
        d = torch.stack([torch.cos(ang), torch.sin(ang)], -1)
        pt = torch.rand(n, K, generator=gen, device=dev, dtype=dt)
        otype = torch.full((n, K), TYPE_WORKER, device=dev, dtype=torch.long)
        otype = torch.where(pt > p_type[0], TYPE_FORKLIFT, otype)
        otype = torch.where(pt > p_type[0] + p_type[1], TYPE_AMR, otype)
        lo = torch.tensor([sp[0] for sp in speeds], dtype=dt, device=dev)[otype]
        hi = torch.tensor([sp[1] for sp in speeds], dtype=dt, device=dev)[otype]
        speed = lo + (hi - lo) * torch.rand(n, K, generator=gen, device=dev, dtype=dt)
        clear = torch.tensor([WORKER_R, 0.64, 0.20], dtype=dt, device=dev)[otype] + 0.05
        reach = torch.tensor([0.0, 2.0, 0.30], dtype=dt, device=dev)[otype]
        hl = half_len[0] + (half_len[1] - half_len[0]) * torch.rand(
            n, K, generator=gen, device=dev, dtype=dt)
        # 양 방향으로 0.1 m 씩 걸으며 EDT ≥ clear 인 마지막 점까지
        steps = torch.arange(0, int(half_len[1] / 0.1) + 1, device=dev, dtype=dt) * 0.1
        ends = []
        ok0 = _edt_at(edt, map_meta, map_idx, c) >= clear
        for sgn in (1.0, -1.0):
            q = c[..., None, :] + sgn * steps[None, None, :, None] * d[..., None, :]
            e = _edt_at(edt, map_meta, map_idx[:, None].expand(n, K), q)
            good = (e >= clear[..., None]) & (steps[None, None, :] <= hl[..., None])
            bad = ~good
            first_bad = torch.where(bad.any(-1), torch.argmax(bad.to(torch.int8), -1),
                                    torch.full_like(hl, steps.numel(), dtype=torch.long))
            ext = (first_bad - 1).clamp_min(0).to(dt) * 0.1
            ext = (ext - reach).clamp_min(0.0)
            ends.append(c + sgn * ext[..., None] * d)
        a_pt, b_pt = ends[1], ends[0]
        seg = torch.linalg.norm(b_pt - a_pt, dim=-1)
        speed = torch.where(seg >= 1.0, speed, torch.zeros_like(speed))
        phase = torch.rand(n, K, generator=gen, device=dev, dtype=dt) * 60.0
        cnt = torch.arange(K, device=dev)[None, :] < count[:, None]
        active = cnt & ok0
        otype_full = self.otype.clone()
        otype_full[idx] = otype
        self.set_type(otype_full)
        self.mot[idx] = MOT_PINGPONG
        self.a[idx] = a_pt
        self.b[idx] = b_pt
        self.speed[idx] = speed
        self.phase[idx] = phase
        self.active[idx] = active
        # 시작 근처 제거 (t = 0)
        st = self.state(torch.zeros(self.n, dtype=dt, device=dev))
        ext = torch.tensor([WORKER_R, math.hypot(2.0, 0.64), math.hypot(0.30, 0.20)], dtype=dt,
                           device=dev)[otype]                     # 몸체 원점에서 가장 먼 점
        near = torch.linalg.norm(st['xy'][idx] - start_xy[:, None, :], dim=-1) < \
            min_start_dist + ext
        self.active[idx] = self.active[idx] & ~near


def _edt_at(edt, map_meta, map_idx, xy):
    """EDT 텐서 [M,H,W] 를 월드 점 xy[...,2] 에서 읽는다 (지도 밖 0). map_idx 는 xy 앞 축과 같은 모양."""
    M, H, W = edt.shape
    meta = map_meta[map_idx]
    while meta.dim() < xy.dim():
        meta = meta.unsqueeze(-2)
    cx = torch.floor((xy[..., 0] - meta[..., 1]) / meta[..., 0]).long()
    cy = torch.floor((xy[..., 1] - meta[..., 2]) / meta[..., 0]).long()
    inb = (cx >= 0) & (cx < W) & (cy >= 0) & (cy < H)
    mi = map_idx
    while mi.dim() < cx.dim():
        mi = mi.unsqueeze(-1)
    v = edt[mi.expand_as(cx), cy.clamp(0, H - 1), cx.clamp(0, W - 1)]
    return torch.where(inb, v, torch.zeros_like(v)).to(xy.dtype)
