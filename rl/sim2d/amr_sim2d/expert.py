"""
특권 전문가 (후보 ④ 의 BC 시연자, 2차 물결 2b r6-dyn) — 장애물의 **미래 위치**(비반응 운동의 지면 진실)를 보는 표본 MPC.

배포 정책이 아니다: 학습 정책은 관측(LiDAR 섹터·트랙 위치·속도)만 보지만, 이 전문가는 env 의 장애물 매개변수에서
앞으로 H 초의 몸체 위치를 정확히 안다 (특권 정보 → 학생 BC, privileged teacher). 1차 물결의 BC 시연자
(heuristic.path_follower)는 장애물을 보지 않아 평가 분포에서 43 % 가 충돌한다 (dyn_obstacles.md §2).

결정 (0.1 s 마다, env 배치 전체):
  1. 후보 = 목표 속도 (v*, ω*) 격자 5 × 7 + 경로 추종기의 (v_f, ω_f) 와 (v_f/2, ω_f).
  2. 후보마다 "go_s 초 동안 그 목표로, 그 뒤 제동해 정지, 지평 H 까지 서 있음" 을 가속 한계(보수적 a 0.8, α 1.5)로
     굴리고, 매 dt 마다 (a) 여유 margin 만큼 키운 풋프린트 vs 그 시각의 장애물 몸체 (b) 정적 지도 EDT 를 본다.
     정지까지 포함하므로 "멈출 수 있는 안전" (ICS 회피) 이다 — 서 있다가 맞는 후보도 안전하지 않다.
  3. 추종기 후보가 안전하면 그것, 아니면 안전한 후보 가운데 (go 끝의 경로 진행 − 0.5·횡오차) 최대,
     안전한 후보가 없으면 첫 충돌 시각이 가장 늦은 후보 (같으면 느린 쪽).
  4. 목표 속도 → 가속도 행동 u (scripted._u 와 같은 규칙).
"""

import math

import torch

from . import contract_torch as T
from . import geometry2d as G
from . import kernels as K
from . import scripted as SP
from ._contract import C


class PrivilegedExpert:
    """env 의 지면 진실을 읽는 표본 MPC — __call__(obs, t) → u [N,2]."""

    def __init__(self, env, horizon=3.0, dt=0.2, go_s=1.0, margin=0.10, static_r=0.25,
                 v_grid=(0.0, 0.25, 0.5, 0.75, 1.0), w_grid=(-1.2, -0.6, -0.3, 0.0, 0.3, 0.6, 1.2),
                 a_lim=0.8, alpha_lim=1.5):
        """후보 격자·지평."""
        self.env = env
        self.H, self.dt, self.go_s = horizon, dt, go_s
        self.margin, self.static_r = margin, static_r
        self.a_lim, self.alpha_lim = a_lim, alpha_lim
        dev, fd = env.dev, env.fd
        vv, ww = torch.meshgrid(torch.tensor(v_grid, dtype=fd, device=dev),
                                torch.tensor(w_grid, dtype=fd, device=dev), indexing='ij')
        self.grid_v, self.grid_w = vv.reshape(-1), ww.reshape(-1)
        self.M = self.grid_v.numel() + 2
        self.steps = int(round(horizon / dt))
        self.stats = {'calls': 0, 'follow_safe': 0, 'other_safe': 0, 'none_safe': 0}

    @torch.no_grad()
    def targets(self, obs):
        """후보 목표 (v*, ω*) [N, M]."""
        vf, wf = SP._follow_target(obs.to(self.env.fd))
        N = obs.shape[0]
        gv = self.grid_v[None].expand(N, -1)
        gw = self.grid_w[None].expand(N, -1)
        v = torch.cat([vf[:, None], 0.5 * vf[:, None], gv], 1)
        w = torch.cat([wf[:, None], wf[:, None], gw], 1)
        return v, w

    @torch.no_grad()
    def __call__(self, obs, t):
        """관측(추종기 목표 계산용) + env 지면 진실 → 행동 u [N,2]."""
        env = self.env
        N, M, fd = env.N, self.M, env.fd
        v_t, w_t = self.targets(obs)
        pose = env.pose[:, None, :].expand(N, M, 3).reshape(-1, 3)
        v = env.v[:, None].expand(N, M).reshape(-1)
        w = env.w[:, None].expand(N, M).reshape(-1)
        vt, wt = v_t.reshape(-1), w_t.reshape(-1)
        op = {k: x.repeat_interleave(M, dim=0) for k, x in env._obst_params().items()}
        t0 = env.t.repeat_interleave(M)
        first_hit = torch.full((N * M,), math.inf, dtype=fd, device=env.dev)
        go_pose = pose
        hl, hw = C.HALF_L + self.margin, C.HALF_W + self.margin
        for k in range(1, self.steps + 1):
            tk = k * self.dt
            go = tk <= self.go_s + 1e-9
            tv = vt if go else torch.zeros_like(vt)
            tw = wt if go else torch.zeros_like(wt)
            dv = (tv - v).clamp(-self.a_lim * self.dt, self.a_lim * self.dt)
            dw = (tw - w).clamp(-self.alpha_lim * self.dt, self.alpha_lim * self.dt)
            v1, w1 = v + dv, w + dw
            x, y, th = T.integrate_arc(0.5 * (v + v1), 0.5 * (w + w1), self.dt)
            c, s = torch.cos(pose[:, 2]), torch.sin(pose[:, 2])
            pose = torch.stack([pose[:, 0] + c * x - s * y, pose[:, 1] + s * x + c * y,
                                T.wrap_angle(pose[:, 2] + th)], 1)
            v, w = v1, w1
            if go:
                go_pose = pose
            ob = K.obstacle_state(op, t0 + tk)
            hit = G.rect_hits_obstacles(pose, ob, half_l=hl, half_w=hw)
            hit = hit | self._static_hit(pose)
            first_hit = torch.where(hit & ~torch.isfinite(first_hit),
                                    torch.full_like(first_hit, tk), first_hit)
        first_hit = first_hit.reshape(N, M)
        safe = ~torch.isfinite(first_hit)
        # 점수: go 끝 자세의 경로 진행 − 0.5·횡오차
        gp = go_pose.reshape(N, M, 3)
        s_now = T.project_onto_path(env.path_pts, env.path_n, env.path_cum, env.pose[:, :2])[0]
        pts = env.path_pts.repeat_interleave(M, 0)
        proj = T.project_onto_path(pts, env.path_n.repeat_interleave(M), env.path_cum
                                   .repeat_interleave(M, 0), gp[..., :2].reshape(-1, 2))
        score = (proj[0].reshape(N, M) - s_now[:, None]) - 0.5 * proj[1].reshape(N, M)
        score = torch.where(safe, score, torch.full_like(score, -math.inf))
        best_safe = score.argmax(1)
        any_safe = safe.any(1)
        late = first_hit + 1e-3 * (1.0 - v_t / 1.0)          # 같으면 느린 쪽
        best_unsafe = late.argmax(1)
        pick = torch.where(safe[:, 0], torch.zeros_like(best_safe),
                           torch.where(any_safe, best_safe, best_unsafe))
        ix = torch.arange(N, device=env.dev)
        self.stats['calls'] += 1
        self.stats['follow_safe'] += int(safe[:, 0].sum())
        self.stats['other_safe'] += int((~safe[:, 0] & any_safe).sum())
        self.stats['none_safe'] += int((~any_safe).sum())
        return SP._u(v_t[ix, pick].to(obs.dtype), w_t[ix, pick].to(obs.dtype), obs)

    def _static_hit(self, pose):
        """정적 지도: 풋프린트 중심선의 세 점(앞·가운데·뒤)에서 EDT < static_r."""
        env = self.env
        c, s = torch.cos(pose[:, 2]), torch.sin(pose[:, 2])
        off = C.HALF_L - self.static_r
        hit = torch.zeros(pose.shape[0], dtype=torch.bool, device=pose.device)
        mi = torch.zeros(pose.shape[0], dtype=torch.long, device=pose.device)
        from .obstacles import _edt_at
        for o in (-off, 0.0, off):
            xy = torch.stack([pose[:, 0] + c * o, pose[:, 1] + s * o], 1)
            hit |= _edt_at(env.edt, env.map_meta, mi, xy) < self.static_r
        return hit
