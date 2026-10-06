"""
Isaac Lab DirectRLEnv — 창고 AMR 내비게이션, 계약 v1.2.0 비교군 B (연속 가속도 행동).

전이 한 번 (Δt = 0.1 s = 물리 0.02 s × 5):
  행동 raw → u = tanh(raw) → integrate_b (ray 클램프) → 명령 램프 (v_cmd → v_cmd', 주기 안 선형)
  → [50 Hz × 5] 프로파일러 기준 경로 → safety_lite φ (D = 근사 접근 거리) → 플랜트 (순수 지연 + 1 차 지연 +
  가속 한계, DR) → 정확 원호 적분 → 풋프린트 vs 정적 상자 충돌 판정 (접촉 = 그 자리 정지·종단)
  → Warp RayCaster 2D LiDAR (720 빔) → 계약 관측·보상·비용 (contract_torch).

보상 판은 cfg.reward_version ('1.2.0' = 계약 기본, '1.1.0' = 대조) 으로 고른다. 전이 전 거리 d(s) =
max(d_arc, 목표 직선거리) 는 관측과 같은 자세에서 캐시해 두고(build_obs → observe_with_cache) 다음 전이의
Φ(s)·진행 효용에 쓴다 — step_outcome 시점에는 버퍼의 자세가 이미 전이 뒤라 다시 계산할 수 없다.

계약과 다르게(또는 계약이 정하지 않아 여기서) 정한 것은 rl/isaac/README.md 「계약과의 차이」에 적는다.
"""

from __future__ import annotations

import math

import torch

from isaaclab.assets import RigidObject
from isaaclab.envs import DirectRLEnv
from isaaclab.sensors import RayCaster
import isaaclab.sim as sim_utils

from . import contract_torch as CT
from .assets import load_geometry, load_pool
from .contract_torch import C, F64
from .env_cfg import AmrNavEnvCfg, BODY_Z, WAREHOUSE_PRIM
from .env_math import (approach_distance, EpisodeLog, move_and_collide, reward_spec_for,
                       select_paths_by_length)

PHYS_DT = C.PROFILER_DT
N_SUB = int(round(C.DT / PHYS_DT))       # 5
MAX_DELAY = 4                            # 플랜트 순수 지연 최대 0.08 s = 4 서브스텝
OUTCOMES = ('success', 'collision', 'time_limit', 'stall')


class AmrNavEnv(DirectRLEnv):
    """창고 AMR 내비게이션 (비교군 B)."""

    cfg: AmrNavEnvCfg

    def __init__(self, cfg: AmrNavEnvCfg, render_mode: str | None = None, **kwargs):
        """버퍼·경로 풀·DR 준비."""
        super().__init__(cfg, render_mode, **kwargs)
        N, dev = self.num_envs, self.device
        self.obb = torch.tensor(self._geo['obb'], dtype=F64, device=dev)
        self.reward_spec = reward_spec_for(cfg.reward_version)
        pool = load_pool(seed=cfg.path_pool_seed, out_dir=cfg.assets_dir)
        # 경로 길이 범위 (구조 파라미터 — 2D 2a 와 같은 3–15 m 로 학습·평가할 때)
        p_pts, p_offs, p_len, self.pool_kept = select_paths_by_length(
            pool['pts'], pool['offs'], pool['length'], cfg.path_len_min, cfg.path_len_max)
        self.pool_pts = torch.tensor(p_pts, dtype=F64, device=dev)
        self.pool_offs = torch.tensor(p_offs, dtype=torch.long, device=dev)
        self.pool_n = self.pool_offs[1:] - self.pool_offs[:-1]
        self.pool_size = int(self.pool_n.shape[0])
        plen = torch.tensor(p_len, dtype=F64, device=dev)
        self.pool_order = torch.argsort(plen)
        self.pool_len_sorted = plen[self.pool_order]
        self.curriculum_lmax = None      # 경로 길이 상한 커리큘럼 (None = 끔), 학습 스크립트가 바꾼다
        self.curriculum_uniform = 0.2    # 커리큘럼 중에도 이 비율은 전체 풀에서 균일 (contract §10)
        # 리셋 때 v_cap: 확률 vcap_p_nominal 로 cfg.v_cap, 나머지는 U[vcap_lo, vcap_hi] (관측 블록
        # timing 에 v_cap/2.0 이 있어 정책이 안다). 학습 스크립트의 속도 상한 커리큘럼이 바꾼다.
        self.vcap_p_nominal = 1.0
        self.vcap_lo = self.vcap_hi = float(cfg.v_cap)
        P = cfg.path_pad
        if int(self.pool_n.max()) > P:
            raise ValueError(f'경로 점 {int(self.pool_n.max())} > path_pad {P}')
        z = lambda: torch.zeros(N, dtype=F64, device=dev)  # noqa: E731
        self.pose = torch.zeros(N, 3, dtype=F64, device=dev)
        self.path_pts = torch.zeros(N, P, 2, dtype=F64, device=dev)
        self.path_n = torch.ones(N, dtype=torch.long, device=dev)
        self.path_cum = torch.zeros(N, P, dtype=F64, device=dev)
        self.path_len = z()
        self.path_id = torch.zeros(N, dtype=torch.long, device=dev)
        self.time_limit = z()
        self.v_cmd, self.w_cmd = z(), z()
        self.v_meas, self.w_meas, self.a_v_meas, self.a_w_meas = z(), z(), z(), z()
        self.prev_actions = torch.zeros(N, C.N_PREV_ACTIONS, 2, dtype=F64, device=dev)
        self.v_cap = torch.full((N,), cfg.v_cap, dtype=F64, device=dev)
        self.zone = torch.zeros(N, dtype=torch.long, device=dev)
        self.speed_limit = torch.full((N,), math.inf, dtype=F64, device=dev)
        self.in_dev = torch.zeros(N, dtype=torch.bool, device=dev)
        self.dev_peak, self.dev_t = z(), z()
        self.d_prog = z()      # 전이 전 progress_distance 캐시 (observe_with_cache)
        self.stall_xy = torch.zeros(N, 2, dtype=F64, device=dev)
        self.stall_t = z()
        self.profiler = CT.ProfilerBatch(N, dev)
        self.plant_v, self.plant_w = z(), z()
        self.delay_buf = torch.zeros(N, MAX_DELAY + 1, 2, dtype=F64, device=dev)
        self.delay_steps = torch.full((N,), 2, dtype=torch.long, device=dev)
        self.plant_tm = torch.full((N,), cfg.dr.nominal_plant_time_constant_s, dtype=F64,
                                   device=dev)
        self.acc_scale = torch.ones(N, dtype=F64, device=dev)
        self.lidar_sigma = torch.full((N,), cfg.dr.nominal_lidar_sigma_m, dtype=F64, device=dev)
        self.beam_drop = z()
        self.scan = torch.full((N, C.LIDAR_SAMPLES), math.nan, dtype=F64, device=dev)
        self.scan_pts_w = torch.zeros(N, C.LIDAR_SAMPLES, 2, dtype=F64, device=dev)
        self.scan_valid = torch.zeros(N, C.LIDAR_SAMPLES, dtype=torch.bool, device=dev)
        T = cfg.max_tracks
        self.tracks = torch.zeros(N, T, 6, dtype=F64, device=dev)      # 동적 장애물 없음 = 전부 무효
        self.track_id = torch.zeros(N, T, dtype=torch.long, device=dev)
        self.scan_age, self.track_age = z(), z()
        self.prox_stop = torch.zeros(N, dtype=torch.bool, device=dev)   # safety_lite: 래치 없음
        self.min_ttc = torch.full((N,), math.inf, dtype=F64, device=dev)
        self.beam_ang = CT.beam_angles(dev, F64)
        # 결정 주기 안 버퍼
        self._sub = 0
        self._v0, self._w0, self._v1, self._w1 = z(), z(), z(), z()
        self._a_exec = torch.zeros(N, 2, dtype=F64, device=dev)
        self._pairs = torch.zeros(N, N_SUB, 4, dtype=F64, device=dev)
        self._pairs_n = torch.full((N,), N_SUB, dtype=torch.long, device=dev)
        self._collided = torch.zeros(N, dtype=torch.bool, device=dev)
        self._w_ref_prev = z()
        self._reward = torch.zeros(N, device=dev)
        # 라그랑주 승수 (비용 5 채널, contract §7) — 학습 스크립트가 쌍대 상승으로 갱신한다.
        # PPO 에 주는 보상 = 계약 보상 − Σ λ_i·c_i (기록·평가의 반환은 계약 보상 그대로).
        self.lagrange = torch.zeros(C.N_COSTS, dtype=F64, device=dev)
        self.last_out = None
        # 통계: 누적 계수 (학습 스크립트가 읽어 비운다) + 최근 창
        self.stat_counts = torch.zeros(len(OUTCOMES), dtype=torch.long, device=dev)
        self.stat_cost_sum = torch.zeros(C.N_COSTS, dtype=F64, device=dev)
        self.stat_episodes = torch.zeros((), dtype=torch.long, device=dev)
        self.ep_cost = torch.zeros(N, C.N_COSTS, dtype=F64, device=dev)
        self.ep_return = torch.zeros(N, dtype=F64, device=dev)
        self.stat_return_sum = torch.zeros((), dtype=F64, device=dev)
        self.stat_len_sum = torch.zeros((), dtype=F64, device=dev)
        self.stat_path_len_sum = torch.zeros((), dtype=F64, device=dev)
        self.ep_log = EpisodeLog(N, C.N_COSTS, dev)
        W = cfg.stats_window
        self.win = torch.zeros(W, len(OUTCOMES), dtype=torch.float32, device=dev)
        self.win_ptr = torch.zeros((), dtype=torch.long, device=dev)
        self.win_fill = torch.zeros((), dtype=torch.long, device=dev)
        self.lidar_pose_err = torch.zeros((), dtype=torch.float32, device=dev)

    # ------------------------------------------------------------------ 장면
    def _setup_scene(self):
        from pxr import UsdGeom, Vt, Gf
        self._geo = load_geometry(self.cfg.assets_dir)
        stage = sim_utils.get_current_stage()
        UsdGeom.Xform.Define(stage, WAREHOUSE_PRIM)
        mesh = UsdGeom.Mesh.Define(stage, WAREHOUSE_PRIM + '/static_colliders')
        verts = self._geo['verts']
        tris = self._geo['tris']
        mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in verts]))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(tris)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for i in tris.reshape(-1)]))
        mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.6, 0.6, 0.62)]))
        self.robot = RigidObject(self.cfg.robot)
        self.scene.rigid_objects['robot'] = self.robot
        self.lidar = RayCaster(self.cfg.lidar)
        self.scene.sensors['lidar'] = self.lidar
        # 충돌체가 하나도 없는 GPU 장면은 첫 step 에서 CUDA 700 (narrowphase) — 바닥 평면을 둔다
        ground = sim_utils.GroundPlaneCfg()
        ground.func('/World/ground', ground)
        self.scene.clone_environments(copy_from_source=False)
        light = sim_utils.DomeLightCfg(intensity=1500.0, color=(0.8, 0.8, 0.8))
        light.func('/World/Light', light)

    # ------------------------------------------------------------------ 상태 묶음
    def contract_state(self):
        """현재 버퍼 → contract_torch.BatchState (복사 없음)."""
        return CT.BatchState(
            pose=self.pose, path_pts=self.path_pts, path_n=self.path_n, scan=self.scan,
            v_meas=self.v_meas, w_meas=self.w_meas, a_v_meas=self.a_v_meas,
            a_w_meas=self.a_w_meas, v_cmd=self.v_cmd, w_cmd=self.w_cmd,
            prev_actions=self.prev_actions, v_cap=self.v_cap,
            scan_age=self.scan_age, track_age=self.track_age,
            tracks=self.tracks, track_id=self.track_id,
            zone=self.zone, speed_limit=self.speed_limit,
            proximity_stop=self.prox_stop, min_ttc=self.min_ttc,
            in_dev=self.in_dev, dev_peak=self.dev_peak, dev_t=self.dev_t,
            path_cum=self.path_cum)

    # ------------------------------------------------------------------ 행동
    def _pre_physics_step(self, actions: torch.Tensor):
        raw = torch.nan_to_num(actions.to(F64), nan=0.0).clamp(-20.0, 20.0)
        u = torch.tanh(raw)
        v1, w1, av, aw = CT.integrate_b(u, self.v_cmd, self.w_cmd, self.v_cap)
        self._v0.copy_(self.v_cmd)
        self._w0.copy_(self.w_cmd)
        self._v1.copy_(v1)
        self._w1.copy_(w1)
        self._a_exec = torch.stack([av, aw], dim=1)
        self._sub = 0
        self._collided.zero_()

    def _apply_action(self):
        self._sub += 1
        k = self._sub
        f = k / N_SUB
        vt = self._v0 + f * (self._v1 - self._v0)
        wt = self._w0 + f * (self._w1 - self._w0)
        v_ref, w_ref = self.profiler.step(vt, wt, PHYS_DT)
        # φ: 스캔 끝점(세계 좌표, 촬영 뒤 이동 보정)을 현재 자세로
        pts_r = CT.to_robot_frame(self.pose, self.scan_pts_w)
        d_main = approach_distance(pts_r, self.scan_valid, v_ref, w_ref)
        out = CT.safety_filter(v_ref, w_ref, d_main, self.min_ttc, self.prox_stop)
        self._pairs[:, k - 1] = torch.stack([v_ref, w_ref, out['v'], out['w']], dim=1)
        self.zone = out['zone']
        self.speed_limit = out['speed_limit']
        # 플랜트: 순수 지연 (서브스텝 단위) → 1 차 지연 → 가속 한계
        self.delay_buf = torch.roll(self.delay_buf, 1, dims=1)
        self.delay_buf[:, 0, 0] = out['v']
        self.delay_buf[:, 0, 1] = out['w']
        idx = self.delay_steps.view(-1, 1, 1).expand(-1, 1, 2)
        u_del = self.delay_buf.gather(1, idx).squeeze(1)
        alpha = 1.0 - torch.exp(-PHYS_DT / self.plant_tm)
        dv = (alpha * (u_del[:, 0] - self.plant_v)).clamp(
            -C.A_MAX * self.acc_scale * PHYS_DT, C.A_MAX * self.acc_scale * PHYS_DT)
        dw = (alpha * (u_del[:, 1] - self.plant_w)).clamp(
            -C.ALPHA_MAX * self.acc_scale * PHYS_DT, C.ALPHA_MAX * self.acc_scale * PHYS_DT)
        moving = ~self._collided
        self.plant_v = torch.where(moving, self.plant_v + dv, torch.zeros_like(dv))
        self.plant_w = torch.where(moving, self.plant_w + dw, torch.zeros_like(dw))
        # 이동·접촉 판정 (순수 함수, tests/test_env_math.py) — _collided 는 같은 텐서를 제자리 갱신
        self.pose, self.plant_v, self.plant_w, collided = move_and_collide(
            self.pose, self.plant_v, self.plant_w, self._collided, self.obb, PHYS_DT)
        self._collided.copy_(collided)
        self.v_meas = self.plant_v.clone()
        self.w_meas = self.plant_w.clone()
        self.a_v_meas = self.profiler.lin.a.clone()
        self.a_w_meas = (w_ref - self._w_ref_prev) / PHYS_DT
        self._w_ref_prev = w_ref.clone()
        if k == N_SUB:
            self._write_root_pose()

    def _write_root_pose(self, env_ids=None):
        p = self.pose if env_ids is None else self.pose[env_ids]
        half = 0.5 * p[:, 2]
        root = torch.stack([p[:, 0], p[:, 1], torch.full_like(p[:, 0], BODY_Z),
                            torch.cos(half), torch.zeros_like(half), torch.zeros_like(half),
                            torch.sin(half)], dim=1).to(torch.float32)
        self.robot.write_root_pose_to_sim(root, env_ids=env_ids)
        if env_ids is not None:
            self.robot.write_root_velocity_to_sim(
                torch.zeros(root.shape[0], 6, device=self.device), env_ids=env_ids)

    # ------------------------------------------------------------------ 종료·보상
    def step_outcome(self):
        """이번 전이의 보상·비용·종료 — 경로·이탈·직전 행동은 전이 전, 자세·v_meas 는 전이 뒤 버퍼."""
        return CT.reward_step(self.contract_state(), self.pose, self.v_meas, self._a_exec,
                              self._collided, self._pairs, self._pairs_n, d_pre=self.d_prog,
                              spec=self.reward_spec)

    def _get_dones(self):
        out = self.step_outcome()
        self.last_out = out
        self._reward = (out.reward - (out.cost * self.lagrange).sum(dim=1)).to(torch.float32)
        self.in_dev, self.dev_peak, self.dev_t = out.in_dev, out.dev_peak, out.dev_t
        self.prev_actions = torch.cat([self._a_exec.unsqueeze(1), self.prev_actions[:, :-1]], 1)
        self.v_cmd = self._v1.clone()
        self.w_cmd = self._w1.clone()
        self.ep_cost += out.cost
        self.ep_return += out.reward
        self.ep_log.accumulate(self.v_meas, self.w_meas, out.e, out.d_goal_post)
        t = self.episode_length_buf.to(F64) * C.DT
        moved = torch.hypot(self.pose[:, 0] - self.stall_xy[:, 0],
                            self.pose[:, 1] - self.stall_xy[:, 1]) >= C.STALL_RADIUS
        self.stall_xy = torch.where(moved.unsqueeze(1), self.pose[:, :2], self.stall_xy)
        self.stall_t = torch.where(moved, t, self.stall_t)
        stalled = (t - self.stall_t) > C.STALL_TIME
        over = t > self.time_limit + 1e-9
        terminated = out.terminated
        time_out = (over | stalled) & ~terminated
        done = terminated | time_out
        res = torch.stack([out.success, out.terminated & ~out.success, over & ~terminated,
                           stalled & ~over & ~terminated], dim=1)
        self._record(done, res)
        self.ep_log.finish(done, res, {
            'return': self.ep_return, 'path_len': self.path_len, 'd_start': self.path_len,
            'd_final': out.d_post, 'steps': self.episode_length_buf, 'cost': self.ep_cost,
            'path_id': self.path_id})
        return terminated, time_out

    def _record(self, done, res):
        n = done.sum()
        self.stat_counts += (res & done.unsqueeze(1)).sum(0)
        self.stat_cost_sum += (self.ep_cost * done.unsqueeze(1)).sum(0)
        self.stat_episodes += n
        dm = done.to(F64)
        self.stat_return_sum += (self.ep_return * dm).sum()
        self.stat_len_sum += (self.episode_length_buf.to(F64) * dm).sum()
        self.stat_path_len_sum += (self.path_len * dm).sum()
        # 최근 창 (동기화 없이: 끝난 env 를 차례로 원형 버퍼에)
        W = self.win.shape[0]
        ids = torch.nonzero(done, as_tuple=False).squeeze(-1)
        k = ids.numel()
        if k:
            pos = (self.win_ptr + torch.arange(k, device=self.device)) % W
            self.win[pos] = res[ids].to(torch.float32)
            self.win_ptr = (self.win_ptr + k) % W
            self.win_fill = torch.clamp(self.win_fill + k, max=W)
        fill = torch.clamp(self.win_fill, min=1).to(torch.float32)
        rates = self.win.sum(0) / fill
        self.extras['log'] = {f'{o}_rate': rates[i] for i, o in enumerate(OUTCOMES)}

    def _get_rewards(self):
        return self._reward

    # ------------------------------------------------------------------ 리셋
    def _reset_idx(self, env_ids):
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        n = len(env_ids)
        dev = self.device
        pid = torch.randint(0, self.pool_size, (n,), device=dev)
        if self.curriculum_lmax is not None:
            k = int(torch.searchsorted(self.pool_len_sorted,
                                       torch.tensor([self.curriculum_lmax], dtype=F64,
                                                    device=dev), right=True))
            k = max(k, 1)
            short = self.pool_order[torch.randint(0, k, (n,), device=dev)]
            use = torch.rand(n, device=dev) >= self.curriculum_uniform
            pid = torch.where(use, short, pid)
        self.path_id[env_ids] = pid
        cnt = self.pool_n[pid]
        P = self.cfg.path_pad
        ar = torch.arange(P, device=dev).unsqueeze(0)
        gidx = self.pool_offs[pid].unsqueeze(1) + torch.minimum(ar, (cnt - 1).unsqueeze(1))
        pts = self.pool_pts[gidx]                                   # [n, P, 2] 끝점 반복
        self.path_pts[env_ids] = pts
        self.path_n[env_ids] = cnt
        cum = CT.path_cumulative(pts, cnt)
        self.path_cum[env_ids] = cum
        L = cum.gather(1, (cnt - 1).unsqueeze(1)).squeeze(1)
        self.path_len[env_ids] = L
        self.time_limit[env_ids] = CT.time_limit(L)
        # 시작 자세: 경로 첫 점, yaw = 접선(0.5 m 앞 점 방향) + U[−0.5, 0.5]
        j = torch.clamp(torch.full_like(cnt, 10), max=cnt - 1)
        ahead = pts.gather(1, j.view(-1, 1, 1).expand(-1, 1, 2)).squeeze(1)
        tan = torch.atan2(ahead[:, 1] - pts[:, 0, 1], ahead[:, 0] - pts[:, 0, 0])
        noise = (torch.rand(n, device=dev, dtype=F64) * 2 - 1) * self.cfg.dr.yaw_noise_rad
        self.pose[env_ids] = torch.stack([pts[:, 0, 0], pts[:, 0, 1], CT.wrap_angle(tan + noise)],
                                         dim=1)
        for buf in (self.v_cmd, self.w_cmd, self.v_meas, self.w_meas, self.a_v_meas,
                    self.a_w_meas, self.dev_peak, self.dev_t, self.plant_v, self.plant_w,
                    self.stall_t, self._w_ref_prev):
            buf[env_ids] = 0.0
        self.prev_actions[env_ids] = 0.0
        self.delay_buf[env_ids] = 0.0
        self.in_dev[env_ids] = False
        self.zone[env_ids] = C.ZONE_CLEAR
        self.speed_limit[env_ids] = math.inf
        vc = self.vcap_lo + (self.vcap_hi - self.vcap_lo) * torch.rand(n, device=dev, dtype=F64)
        nominal = torch.rand(n, device=dev) < self.vcap_p_nominal
        self.v_cap[env_ids] = torch.where(nominal, torch.full_like(vc, self.cfg.v_cap), vc)
        self.stall_xy[env_ids] = pts[:, 0]
        self.ep_cost[env_ids] = 0.0
        self.ep_return[env_ids] = 0.0
        self.ep_log.reset(env_ids)
        self.profiler.reset(env_ids)
        self._sample_dr(env_ids)
        self._write_root_pose(env_ids)

    def _sample_dr(self, env_ids):
        n = len(env_ids)
        dev = self.device
        dr = self.cfg.dr

        def U(lo_hi):
            lo, hi = lo_hi
            return lo + (hi - lo) * torch.rand(n, device=dev, dtype=F64)
        if dr.enable:
            self.plant_tm[env_ids] = U(dr.plant_time_constant_s)
            d = torch.round(U(dr.plant_delay_s) / PHYS_DT).long().clamp(1, MAX_DELAY)
            self.delay_steps[env_ids] = d
            self.acc_scale[env_ids] = U(dr.accel_limit_scale)
            self.lidar_sigma[env_ids] = U(dr.lidar_sigma_m)
            self.beam_drop[env_ids] = U(dr.beam_dropout_p)
        else:
            self.plant_tm[env_ids] = dr.nominal_plant_time_constant_s
            self.delay_steps[env_ids] = int(round(dr.nominal_plant_delay_s / PHYS_DT))
            self.acc_scale[env_ids] = 1.0
            self.lidar_sigma[env_ids] = dr.nominal_lidar_sigma_m
            self.beam_drop[env_ids] = 0.0

    # ------------------------------------------------------------------ 관측
    def _read_scan(self):
        """스캔 거리 [N, 720] (무효 = nan) — RayCaster 데이터 또는 같은 Warp 메시 직접."""
        if self.cfg.lidar_source == 'sensor':
            data = self.lidar.data
            # RayCaster 의 pos_w 는 부착 강체 원점이다 — cfg.offset 은 광선 시작점에만 더해진다
            # (ray_caster.py _initialize_rays_impl). 몸체가 평면 운동이므로 yaw 로 돌려 더한다.
            q = data.quat_w.to(F64)
            yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
                              1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2))
            off = self.cfg.lidar.offset.pos
            origin = data.pos_w[:, :2].to(F64) + torch.stack([
                off[0] * torch.cos(yaw) - off[1] * torch.sin(yaw),
                off[0] * torch.sin(yaw) + off[1] * torch.cos(yaw)], dim=1)
            hits = data.ray_hits_w[..., :2].to(F64)
        else:
            from isaaclab.utils.warp import raycast_mesh
            o = self.sensor_origin()
            starts = torch.cat([o, torch.full_like(o[:, :1], 0.20)], 1).to(torch.float32)
            ang = (self.pose[:, 2:3] + self.beam_ang.unsqueeze(0))
            dirs = torch.stack([torch.cos(ang), torch.sin(ang), torch.zeros_like(ang)], -1)
            starts = starts.unsqueeze(1).expand_as(dirs).contiguous()
            hits = raycast_mesh(starts, dirs.to(torch.float32).contiguous(),
                                RayCaster.meshes[WAREHOUSE_PRIM],
                                max_dist=C.LIDAR_RANGE_MAX)[0][..., :2].to(F64)
            origin = o
        r = torch.hypot(hits[..., 0] - origin[:, 0:1], hits[..., 1] - origin[:, 1:2])
        r = torch.where(torch.isfinite(r), r, torch.full_like(r, math.nan))
        exp_o = self.sensor_origin()
        self.lidar_pose_err = (origin - exp_o).abs().max().to(torch.float32)
        return r

    def sensor_origin(self):
        """계약 자세로 계산한 LiDAR 원점 [N, 2]."""
        c, s = torch.cos(self.pose[:, 2]), torch.sin(self.pose[:, 2])
        return torch.stack([self.pose[:, 0] + C.LIDAR_X * c - C.LIDAR_Y * s,
                            self.pose[:, 1] + C.LIDAR_X * s + C.LIDAR_Y * c], dim=1)

    def _get_observations(self):
        r = self._read_scan()
        noise = torch.randn_like(r) * self.lidar_sigma.unsqueeze(1)
        r = r + noise
        drop = torch.rand_like(r) < self.beam_drop.unsqueeze(1)
        self.set_scan(torch.where(drop, torch.full_like(r, math.nan), r))
        return {'policy': self.build_obs()}

    def set_scan(self, r):
        """스캔 거리 [N, 720] (무효 = nan/inf/범위 밖) 를 넣고 세계 좌표 끝점을 갱신한다."""
        self.scan = r
        valid = CT.scan_valid(r)
        self.scan_valid = valid
        ang = self.pose[:, 2:3] + self.beam_ang.unsqueeze(0)
        o = self.sensor_origin()
        rr = torch.where(valid, r, torch.zeros_like(r))
        self.scan_pts_w = torch.stack([o[:, 0:1] + rr * torch.cos(ang),
                                       o[:, 1:2] + rr * torch.sin(ang)], dim=-1)

    def build_obs(self):
        """현재 버퍼의 계약 관측 [N, 291] (전이 전 거리 캐시 d_prog 도 갱신)."""
        obs, self.d_prog = CT.observe_with_cache(self.contract_state(), self.reward_spec)
        return obs

    def inject_state(self, st, ids):
        """
        시험용: contract_torch.BatchState(배치 = len(ids)) 를 env 버퍼 ids 에 넣는다.

        경로는 path_pad 로 끝점 반복 패딩, 트랙은 max_tracks 까지.
        """
        n, P = st.path_pts.shape[1], self.cfg.path_pad
        pts = torch.cat([st.path_pts, st.path_pts[:, -1:].expand(-1, P - n, -1)], 1)
        idx = torch.arange(P, device=self.device).unsqueeze(0)
        last = (st.path_n - 1).unsqueeze(1)
        last_pt = pts.gather(1, last.unsqueeze(-1).expand(-1, 1, 2))
        pts = torch.where((idx > last).unsqueeze(-1), last_pt.expand(-1, P, -1), pts)
        self.path_pts[ids] = pts
        self.path_n[ids] = st.path_n
        self.path_cum[ids] = CT.path_cumulative(pts, st.path_n)
        self.path_len[ids] = self.path_cum[ids].gather(1, last).squeeze(1)
        for name in ('pose', 'v_meas', 'w_meas', 'a_v_meas', 'a_w_meas', 'v_cmd', 'w_cmd',
                     'prev_actions', 'v_cap', 'zone', 'speed_limit', 'in_dev', 'dev_peak',
                     'dev_t', 'scan_age', 'track_age', 'min_ttc'):
            getattr(self, name)[ids] = getattr(st, name).to(getattr(self, name).dtype)
        self.prox_stop[ids] = st.proximity_stop
        T = st.tracks.shape[1]
        if T > self.tracks.shape[1]:
            raise ValueError(f'트랙 {T} > max_tracks {self.tracks.shape[1]}')
        self.tracks[ids] = 0.0
        self.track_id[ids] = 0
        self.tracks[ids, :T] = st.tracks
        self.track_id[ids, :T] = st.track_id
        full = torch.full((self.num_envs, C.LIDAR_SAMPLES), math.nan, dtype=F64,
                          device=self.device)
        full[ids] = st.scan
        keep = torch.ones(self.num_envs, dtype=torch.bool, device=self.device)
        keep[ids] = False
        full[keep] = self.scan[keep]
        self.set_scan(full)

    # ------------------------------------------------------------------ 통계
    def pop_stats(self):
        """누적 결과 계수를 읽고 비운다 → dict (CPU)."""
        out = {'episodes': int(self.stat_episodes)}
        for i, o in enumerate(OUTCOMES):
            out[o] = int(self.stat_counts[i])
        for i, name in enumerate(C.COST_NAMES):
            out[f'cost_{name}'] = float(self.stat_cost_sum[i])
        out['return_sum'] = float(self.stat_return_sum)
        out['len_steps_sum'] = float(self.stat_len_sum)
        out['path_len_sum'] = float(self.stat_path_len_sum)
        for t in (self.stat_counts, self.stat_cost_sum, self.stat_episodes, self.stat_return_sum,
                  self.stat_len_sum, self.stat_path_len_sum):
            t.zero_()
        return out
