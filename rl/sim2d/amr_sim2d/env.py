"""
GPU 배치 2D 내비게이션 환경 — 계약 v1.1.0 의 관측·행동·보상·비용·종료, 자동 리셋, 텐서 GPU 상주.

한 결정 스텝(0.1 s = LiDAR 주기) 의 순서 (contract.md §3, §5.4):
  1. 정책 행동 → 명령 적분 (B: integrate_b ray 클램프 / C: integrate_c, 마스크 밖이면 폴백)
  2. 50 Hz 서브스텝 5 회: 명령 램프 → 프로파일러 기준 모델 → safety φ (safety_lite, D = 스윕 접근 거리)
     → 플랜트 (순수 지연 T_d + 행동 지연, 1 차 지연 T_m, 가속 한계 배율) → 정확 원호 적분 → 지면 진실 충돌
  3. 새 스캔 (관측 지연만큼 과거 자세·장애물에서 레이캐스트) · 트랙 (누락·잡음)
  4. 보상·비용·이탈 상태 (reward_step), 성공·충돌(종단) / 시간 상한·정체(truncation)
  5. 관측 291 (+ C 면 마스크 100), 끝난 env 는 final_obs 를 남기고 리셋
계약 §5.4 문장은 "플랜트 → φ" 순서로 적었지만 §1·§7.1(φ 입력 = 프로파일러 출력)과 실제 사슬
(velocity_profiler → safety_node → DiffDrive) 은 "φ → 플랜트" 다. 여기서는 후자를 따른다 (REPORT 에 r0 요청).
"""

from dataclasses import dataclass, field
import math

import numpy as np
import torch

from . import contract_torch as T
from . import kernels as K
from . import obstacles as OB
from . import raycast as RC
from ._contract import C
from .gridmap import DEFAULT_MAP, GridMap, stack_maps
from .paths import load_or_build

INF = math.inf
SUB = int(round(C.DT / C.PROFILER_DT))       # 5 서브스텝
SUB_DT = C.PROFILER_DT
SPIN_W, SPIN_V = 1.0, 0.1                    # 진단: |ω| > 1.0 rad/s 이고 |v| < 0.1 m/s = 제자리 회전


def _resolve_spec(reward):
    """EnvConfig.reward → RewardSpec (None = v1.1.0, 문자열 = reward_variants 이름)."""
    if reward is None:
        return T.SPEC_V110
    if isinstance(reward, str):
        from amr_rl_contract import reward_variants
        return reward_variants.get(reward)
    return reward


@dataclass
class EnvConfig:
    """환경 설정. DR 범위는 rl/contract/amr_rl_contract/dr.py DR_AXES 기본값."""

    num_envs: int = 1024
    device: str = 'cuda'
    seed: int = 0
    arm: str = 'B'                              # 'B' 연속 가속도, 'C' 격자 + 마스크
    map_yaml: str = DEFAULT_MAP
    n_paths: int = 4096
    path_len: tuple = (3.0, 30.0)               # 풀에서 뽑는 경로 길이 범위 (커리큘럼 손잡이)
    obstacle_mode: str = 'random'               # random | gazebo | mixed | none
    p_gazebo: float = 0.2                       # mixed 에서 gazebo 배치 비율
    n_obstacles: tuple = (0, 8)                 # random 배치 개수 범위 (DR 축 3)
    obstacle_slots: int = 8
    dr: bool = True                             # False = 명목값 (Gazebo 기준값, 평가용)
    safety: bool = True                         # φ 를 전이에 넣는다 (contract §7.3)
    yaw_noise: float = 0.5                      # 시작 yaw = 경로 접선 + U[−a, a] (§10)
    v_cap: float = C.V_MAX_OP
    use_triton: bool = None
    compute_mask: bool = None                   # None = arm == 'C'
    compile: bool = True                        # torch.compile 고정 N 경로
    reward: object = None                       # RewardSpec 또는 후보 이름 (None = 계약 v1.1.0)
    truncate: bool = True                       # False = 정체·시간 상한을 플래그만 (분석용 연속 롤아웃)
    dr_ranges: dict = field(default_factory=lambda: {
        'plant_tm': (0.048, 0.112), 'plant_td': (0.02, 0.08), 'accel_scale': (0.84, 1.0),
        'obs_delay': (0.0, 0.15), 'action_delay': (0.0, 0.06),
        'track_drop': (0.0, 0.3), 'track_pos_sigma': (0.0, 0.10), 'track_vel_sigma': (0.0, 0.20),
        'lidar_sigma': (0.01, 0.06), 'beam_dropout': (0.0, 0.02)})
    nominal: dict = field(default_factory=lambda: {
        'plant_tm': 0.08, 'plant_td': 0.04, 'accel_scale': 1.0, 'obs_delay': 0.0,
        'action_delay': 0.0, 'track_drop': 0.0, 'track_pos_sigma': 0.0, 'track_vel_sigma': 0.0,
        'lidar_sigma': C.LIDAR_NOISE_STD, 'beam_dropout': 0.0})


class Sim2DEnv:
    """배치 환경. reset() → obs, step(action) → (obs, reward, terminated, truncated, info)."""

    HIST = 10        # 자세 이력 (서브스텝) — 관측 지연 ≤ 0.15 s = 7.5 서브스텝
    DBUF = 10        # 플랜트 명령 지연 버퍼 (서브스텝) — T_d + 행동 지연 ≤ 0.14 s

    def __init__(self, cfg: EnvConfig, log=print):
        """지도·경로 풀·상태 텐서를 만든다."""
        self.cfg = cfg
        self.spec = _resolve_spec(cfg.reward)
        self.N = N = cfg.num_envs
        self.dev = dev = torch.device(cfg.device)
        self.fd = torch.float64
        self.gen = torch.Generator(device=dev)
        self.gen.manual_seed(cfg.seed)
        self.use_triton = cfg.use_triton if cfg.use_triton is not None else (
            RC.HAVE_TRITON and dev.type == 'cuda')
        self.want_mask = cfg.compute_mask if cfg.compute_mask is not None else cfg.arm == 'C'
        # 지도
        self.gm = GridMap.from_yaml(cfg.map_yaml)
        self.grid, self.map_meta, _, _ = stack_maps([self.gm], dev)
        self.edt = torch.as_tensor(self.gm.edt, dtype=torch.float32, device=dev)[None]
        self.map_idx = torch.zeros(N, dtype=torch.long, device=dev)
        # 경로 풀
        pool = load_or_build(self.gm, cfg.n_paths, seed=cfg.seed, log=log)
        L = np.asarray(pool['length'])
        keep = np.where((L >= cfg.path_len[0]) & (L <= cfg.path_len[1]))[0]
        if len(keep) == 0:
            raise ValueError(f'경로 길이 {cfg.path_len} 인 풀 항목이 없다')
        self.pool_keep = torch.as_tensor(keep, device=dev)
        self.pool_L = torch.as_tensor(L, device=dev)
        self.pool_pts, self.pool_n, self.pool_cum, self.pool_len = T.path_tensors(
            pool['paths'], dev)
        self.P = self.pool_pts.shape[1]
        log(f'[env] 지도 {self.gm.name} {self.gm.W}x{self.gm.H}, 경로 풀 {len(L)} (사용 {len(keep)}, '
            f'P={self.P}), N={N}, arm={cfg.arm}, triton={self.use_triton}, cache={pool["cache"]}, '
            f'reward={self.spec.name}')
        self.obst = OB.Obstacles(N, cfg.obstacle_slots, dev)
        self._alloc()
        self._compile()
        self.av_lat, self.aw_lat = T.lattice(dev, self.fd)

    # ------------------------------------------------------------ 상태
    def _alloc(self):
        N, dev, fd = self.N, self.dev, self.fd

        def z(*s, dt=fd):
            return torch.zeros(N, *s, dtype=dt, device=dev)
        self.t = z()
        self.pose = z(3)
        self.v, self.w = z(), z()
        self.prof = T.ProfilerBatch(N, dev, fd)
        self.w_ref_prev = z()
        self.dbuf = z(self.DBUF, 2)
        self.hist = z(self.HIST, 3)
        self.tm, self.td, self.acc_scale = z(), z(), z()
        self.obs_delay = z(dt=torch.long)
        self.v_cmd, self.w_cmd = z(), z()
        self.prev_actions = z(C.N_PREV_ACTIONS, 2)
        self.v_cap = torch.full((N,), self.cfg.v_cap, dtype=fd, device=dev)
        self.path_pts = z(self.P, 2)
        self.path_n = z(dt=torch.long)
        self.path_cum = z(self.P)
        self.path_len = z()
        self.ranges = torch.full((N, C.LIDAR_SAMPLES), INF, dtype=torch.float32, device=dev)
        K = self.cfg.obstacle_slots
        self.trk = z(K, 4)
        self.trk_valid = z(K, dt=torch.bool)
        self.trk_dyn = z(K, dt=torch.bool)
        self.trk_id = torch.arange(K, device=dev)[None].repeat(N, 1)
        self.track_age = z()
        self.scan_age = z()
        self.zone = z(dt=torch.long)
        self.speed_limit = torch.full((N,), INF, dtype=fd, device=dev)
        self.prox = z(dt=torch.bool)
        self.min_ttc = torch.full((N,), INF, dtype=fd, device=dev)
        self.stop_count = z(dt=torch.long)
        self.intent = z(2)
        self.have_intent = z(dt=torch.bool)
        self.dev_in = z(dt=torch.bool)
        self.dev_peak, self.dev_t = z(), z()
        self.base_xy = z(2)
        self.base_t = z()
        self.t_limit = z()
        self.d_arc = z()
        self.d_goal = z()
        # 에피소드 진단 (학습 신호 아님): 시작 거리, 이동 거리, 제자리 회전 스텝, 최대 경로 이탈, 목표 최소 거리
        self.ep_d0, self.ep_travel, self.ep_maxe, self.ep_gmin = z(), z(), z(), z()
        self.ep_spin = z(dt=torch.long)
        self.lidar_sigma, self.beam_drop = z(), z()
        self.trk_drop, self.trk_psig, self.trk_vsig = z(), z(), z()
        self.ep_ret, self.ep_len = z(), z(dt=torch.long)
        self.ep_cost = z(C.N_COSTS)
        self.ep_fallback = z(dt=torch.long)
        self.mask = torch.ones(N, C.N_ACTIONS_C, dtype=torch.bool, device=dev)
        self.scan_px = torch.zeros(N, C.LIDAR_SAMPLES, dtype=torch.float32, device=dev)
        self.scan_py = torch.zeros_like(self.scan_px)
        self.scan_ok = torch.zeros_like(self.scan_px, dtype=torch.bool)

    def set_curriculum(self, path_len=None, n_obstacles=None):
        """
        커리큘럼 손잡이 (계약 §10: 구조 파라미터의 수동 단계) — 다음 리셋부터 적용.

        path_len: 풀에서 뽑을 경로 길이 범위, n_obstacles: random 배치 개수 범위.
        """
        if path_len is not None:
            keep = torch.nonzero((self.pool_L >= path_len[0]) & (self.pool_L <= path_len[1]))[:, 0]
            if keep.numel() == 0:
                raise ValueError(f'경로 길이 {path_len} 인 풀 항목이 없다')
            self.pool_keep = keep
            self.cfg.path_len = tuple(path_len)
        if n_obstacles is not None:
            self.cfg.n_obstacles = (int(n_obstacles[0]), int(n_obstacles[1]))

    def bstate(self):
        """현재 상태의 BState 뷰 (복사 없음)."""
        return T.BState(
            pose=self.pose, path_pts=self.path_pts, path_n=self.path_n, path_cum=self.path_cum,
            path_len=self.path_len, ranges=self.ranges, v_meas=self.v, w_meas=self.w,
            a_v_meas=self.prof.lin_a, a_w_meas=(self.prof.w_ref - self.w_ref_prev) / SUB_DT,
            v_cmd=self.v_cmd, w_cmd=self.w_cmd, prev_actions=self.prev_actions, v_cap=self.v_cap,
            scan_age=self.scan_age, track_age=self.track_age, trk=self.trk,
            trk_dyn=self.trk_dyn, trk_valid=self.trk_valid, trk_id=self.trk_id, zone=self.zone,
            speed_limit=self.speed_limit, proximity_stop=self.prox, min_ttc=self.min_ttc,
            dev_in=self.dev_in, dev_peak=self.dev_peak, dev_t=self.dev_t)

    # ------------------------------------------------------------ 난수
    def _u(self, n, lo, hi):
        return lo + (hi - lo) * torch.rand(n, generator=self.gen, device=self.dev, dtype=self.fd)

    def _dr(self, key, n):
        if self.cfg.dr:
            lo, hi = self.cfg.dr_ranges[key]
            return self._u(n, lo, hi)
        return torch.full((n,), float(self.cfg.nominal[key]), dtype=self.fd, device=self.dev)

    # ------------------------------------------------------------ 리셋
    def reset(self):
        """전체 리셋 → 관측 [N,291] (C 면 self.mask 도 갱신)."""
        idx = torch.arange(self.N, device=self.dev)
        self._reset_idx(idx)
        obs = self._observe()
        return obs

    def _reset_idx(self, idx):
        n = idx.numel()
        if n == 0:
            return
        dev, fd = self.dev, self.fd
        pi = self.pool_keep[torch.randint(0, self.pool_keep.numel(), (n,), generator=self.gen,
                                          device=dev)]
        self.path_pts[idx] = self.pool_pts[pi]
        self.path_n[idx] = self.pool_n[pi]
        self.path_cum[idx] = self.pool_cum[pi]
        self.path_len[idx] = self.pool_len[pi]
        p0 = self.pool_pts[pi, 0]
        tang = T.interpolate_at(self.pool_pts[pi], self.pool_n[pi], self.pool_cum[pi],
                                torch.full((n, 1), 0.25, dtype=fd, device=dev))[:, 0] - p0
        yaw = torch.atan2(tang[:, 1], tang[:, 0]) + self._u(n, -self.cfg.yaw_noise,
                                                            self.cfg.yaw_noise)
        pose = torch.cat([p0, T.wrap_angle(yaw)[:, None]], dim=1)
        self.pose[idx] = pose
        self.hist[idx] = pose[:, None, :]
        for name in ('t', 'v', 'w', 'w_ref_prev', 'v_cmd', 'w_cmd', 'dev_peak', 'dev_t',
                     'base_t', 'ep_ret'):
            getattr(self, name)[idx] = 0.0
        self.dbuf[idx] = 0.0
        self.prev_actions[idx] = 0.0
        self.prof.reset(idx)
        self.dev_in[idx] = False
        self.zone[idx] = C.ZONE_CLEAR
        self.speed_limit[idx] = INF
        self.prox[idx] = False
        self.min_ttc[idx] = INF
        self.stop_count[idx] = 0
        self.intent[idx] = 0.0
        self.have_intent[idx] = False
        self.base_xy[idx] = p0
        self.t_limit[idx] = T.time_limit(self.path_len[idx])
        self.ep_len[idx] = 0
        self.ep_cost[idx] = 0.0
        self.ep_fallback[idx] = 0
        # DR
        self.tm[idx] = self._dr('plant_tm', n)
        self.td[idx] = self._dr('plant_td', n) + self._dr('action_delay', n)
        self.acc_scale[idx] = self._dr('accel_scale', n)
        self.obs_delay[idx] = torch.round(self._dr('obs_delay', n) / SUB_DT).long().clamp(
            0, self.HIST - 1)
        self.lidar_sigma[idx] = self._dr('lidar_sigma', n)
        self.beam_drop[idx] = self._dr('beam_dropout', n)
        self.trk_drop[idx] = self._dr('track_drop', n)
        self.trk_psig[idx] = self._dr('track_pos_sigma', n)
        self.trk_vsig[idx] = self._dr('track_vel_sigma', n)
        # 장애물
        self._place_obstacles(idx)
        self._sense(idx)
        proj = T.project_onto_path(self.path_pts[idx], self.path_n[idx], self.path_cum[idx],
                                   self.pose[idx, :2])
        self.d_arc[idx] = (self.path_len[idx] - proj[0]).clamp_min(0.0)
        self.d_goal[idx] = torch.linalg.norm(
            self.pose[idx, :2] - T.goal_xy(self.path_pts[idx], self.path_n[idx]), dim=-1)
        self.ep_d0[idx] = torch.maximum(self.d_arc[idx], self.d_goal[idx])
        self.ep_gmin[idx] = self.d_goal[idx]
        self.ep_travel[idx] = 0.0
        self.ep_maxe[idx] = 0.0
        self.ep_spin[idx] = 0

    def _place_obstacles(self, idx):
        cfg = self.cfg
        n = idx.numel()
        self.obst.reset_slots(idx)
        if cfg.obstacle_mode == 'none' or cfg.obstacle_slots == 0:
            return
        if cfg.obstacle_mode == 'gazebo':
            gz = torch.ones(n, dtype=torch.bool, device=self.dev)
        elif cfg.obstacle_mode == 'mixed':
            gz = torch.rand(n, generator=self.gen, device=self.dev) < cfg.p_gazebo
        else:
            gz = torch.zeros(n, dtype=torch.bool, device=self.dev)
        ig, ir = idx[gz], idx[~gz]
        if ig.numel():
            self.obst.place_gazebo(ig, self.t[ig], self.gen)
        if ir.numel():
            cnt = torch.randint(cfg.n_obstacles[0], cfg.n_obstacles[1] + 1, (ir.numel(),),
                                generator=self.gen, device=self.dev)
            self.obst.place_random(
                ir, self.edt, self.map_meta, self.map_idx[ir],
                (self.path_pts[ir], self.path_n[ir], self.path_cum[ir], self.path_len[ir]),
                self.pose[ir, :2], cnt, self.gen)

    # ------------------------------------------------------------ 감지
    def _obst_params(self):
        o = self.obst
        return {'active': o.active, 'kind': o.kind, 'r': o.r, 'box': o.box, 'mot': o.mot,
                'a': o.a, 'b': o.b, 'speed': o.speed, 'phase': o.phase}

    def _sense(self, idx=None):
        """스캔·트랙 갱신 (관측 지연 반영). idx None = 전체 (컴파일 경로)."""
        full = idx is None
        if full:
            idx = slice(None)
            n = self.N
        else:
            n = idx.numel()
        dl = self.obs_delay[idx]
        pose_d = self.hist[idx].gather(1, dl[:, None, None].expand(-1, 1, 3))[:, 0]
        t_d = self.t[idx] - dl.to(self.fd) * SUB_DT
        op = {k: v[idx] for k, v in self._obst_params().items()}
        ob = (self._k['obst'] if full else K.obstacle_state)(op, t_d)
        ranges = RC.cast_scan(
            self.grid, self.map_meta, self.map_idx[idx], pose_d, None,
            noise_std=None, dropout=None, generator=self.gen, use_triton=self.use_triton)
        g, dev = self.gen, self.dev
        nz = torch.randn(n, C.LIDAR_SAMPLES, generator=g, device=dev)
        ud = torch.rand(n, C.LIDAR_SAMPLES, generator=g, device=dev)
        Kk = self.cfg.obstacle_slots
        nxy = torch.randn(n, Kk, 2, generator=g, device=dev, dtype=self.fd)
        nv = torch.randn(n, Kk, 2, generator=g, device=dev, dtype=self.fd)
        udr = torch.rand(n, Kk, generator=g, device=dev, dtype=self.fd)
        ray = self._k['ray_obst'] if full else RC.raycast_obstacles
        out = _sense_rest(ranges, pose_d, ob, self.lidar_sigma[idx], self.beam_drop[idx], nz, ud,
                          self.trk_psig[idx], self.trk_vsig[idx], self.trk_drop[idx], nxy, nv,
                          udr, ray)
        self.ranges[idx] = out['ranges']
        age = dl.to(self.fd) * SUB_DT
        self.scan_age[idx] = age
        self.track_age[idx] = age
        if Kk:
            self.trk[idx], self.trk_valid[idx], self.trk_dyn[idx] = out['trk']
        self.scan_px[idx], self.scan_py[idx], self.scan_ok[idx] = out['pts']

    # ------------------------------------------------------------ 스텝
    def _observe(self):
        st = self.bstate()
        proj = T.project_onto_path(self.path_pts, self.path_n, self.path_cum, self.pose[:, :2])
        obs = T.build_observation(st, proj)
        if self.want_mask:
            self.mask = T.compute_mask(st, proj)['mask']
        return obs

    def _compile(self):
        """고정 N 경로를 torch.compile 로 묶는다 (부분집합 리셋은 컴파일하지 않은 같은 함수)."""
        comp = self.cfg.compile
        wrap = (lambda f: torch.compile(f, dynamic=False)) if comp else (lambda f: f)
        # _sense_rest 전체를 묶으면 inductor 가 느린 커널(4096 env 114 ms)을 만든다 → 장애물 레이캐스트만 묶는다
        spec, trunc_on = self.spec, bool(self.cfg.truncate)

        def _post(st, P, a_prev, a_exec, collided, interv):
            return K.post_step(st, P, a_prev, a_exec, collided, interv, spec, trunc_on)
        self._k = {'substep': wrap(K.substep), 'post': wrap(_post),
                   'obst': wrap(K.obstacle_state), 'sense': _sense_rest,
                   'min_ttc': wrap(K.min_ttc), 'ray_obst': wrap(RC.raycast_obstacles),
                   'observe': wrap(_observe_all), 'mask': wrap(_mask_static)}
        self._kf = [torch.tensor(k / SUB, dtype=self.fd, device=self.dev)
                    for k in range(1, SUB + 1)]
        self._first = [torch.tensor(k == 1, device=self.dev) for k in range(1, SUB + 1)]

    _S_KEYS = ('pose', 'v', 'w', 'w_ref_prev', 'dbuf', 'hist', 'intent', 'have_intent',
               'stop_count', 'prox', 'zone', 'speed_limit')

    @torch.no_grad()
    def step(self, action):
        """행동 [N,2] (B, tanh 뒤 [−1,1]) 또는 [N] long (C) → 전이."""
        cfg, N, dev, fd = self.cfg, self.N, self.dev, self.fd
        if cfg.arm == 'B':
            v1, w1, av, aw = T.integrate_b(action.to(fd), self.v_cmd, self.w_cmd, self.v_cap)
            fb_used = torch.zeros(N, dtype=torch.bool, device=dev)
        else:
            a = action.long()
            ok = self.mask.gather(1, a.clamp(0, C.N_ACTIONS_C - 1)[:, None])[:, 0]
            a = torch.where(ok, a, torch.full_like(a, C.FALLBACK_INDEX))
            fb_used = a == C.FALLBACK_INDEX
            v1, w1, av, aw = T.integrate_c(a, self.v_cmd, self.w_cmd)
        a_exec = torch.stack([av, aw], dim=1)
        mt = self._k['min_ttc'](self.bstate()) if cfg.safety else torch.full_like(self.v, INF)
        S = {k: getattr(self, k) for k in self._S_KEYS}
        p = self.prof
        S.update({'lin_v': p.lin_v, 'lin_a': p.lin_a, 'ang_v': p.ang_v, 'ang_a': p.ang_a,
                  'w_ref': p.w_ref, 'curv': p.curv_mode,
                  'collided': torch.zeros(N, dtype=torch.bool, device=dev),
                  'coll_moving': torch.zeros(N, dtype=torch.bool, device=dev),
                  'interv': torch.zeros(N, dtype=fd, device=dev)})
        X = {'v_cmd': self.v_cmd, 'w_cmd': self.w_cmd, 'v1': v1, 'w1': w1,
             'scan_px': self.scan_px, 'scan_py': self.scan_py, 'scan_ok': self.scan_ok,
             'min_ttc': mt, 'tm': self.tm, 'td': self.td, 'acc_scale': self.acc_scale,
             't': self.t, 'obst': self._obst_params(), 'grid': self.grid,
             'map_meta': self.map_meta, 'map_idx': self.map_idx}
        for k in range(SUB):
            S = self._k['substep'](S, X, self._kf[k], self._first[k], bool(cfg.safety))
        for k in self._S_KEYS:
            setattr(self, k, S[k])
        p.lin_v, p.lin_a, p.ang_v, p.ang_a, p.w_ref, p.curv_mode = (
            S['lin_v'], S['lin_a'], S['ang_v'], S['ang_a'], S['w_ref'], S['curv'])
        collided, coll_moving = S['collided'], S['coll_moving']
        interv = S['interv'] / SUB
        self.t = self.t + C.DT
        if cfg.safety:
            self.min_ttc = mt
        a_prev = self.prev_actions[:, 0].clone()
        self.v_cmd, self.w_cmd = v1, w1
        self.prev_actions = torch.cat([a_exec[:, None, :], self.prev_actions[:, :-1]], dim=1)
        self._sense()
        Pd = {'d_arc': self.d_arc, 'd_goal': self.d_goal, 'base_xy': self.base_xy,
              'base_t': self.base_t, 't': self.t, 't_limit': self.t_limit}
        r = self._k['post'](self.bstate(), Pd, a_prev, a_exec, collided, interv)
        self.dev_in, self.dev_peak, self.dev_t = r['dev']
        self.d_arc, self.d_goal = r['d_arc'], r['d_goal']
        self.base_xy, self.base_t = r['base_xy'], r['base_t']
        self.ep_travel += self.v.abs() * C.DT
        self.ep_spin += ((self.w.abs() > SPIN_W) & (self.v.abs() < SPIN_V)).long()
        self.ep_maxe = torch.maximum(self.ep_maxe, r['proj'][1])
        self.ep_gmin = torch.minimum(self.ep_gmin, self.d_goal)
        obs, rew, term, trunc = r['obs'], r['reward'], r['terminated'], r['truncated']
        if self.want_mask:
            self.mask = self._k['mask'](self.bstate(), r['proj'])
        self.ep_ret += rew
        self.ep_len += 1
        self.ep_cost += r['cost']
        self.ep_fallback += fb_used.long()
        done = term | trunc
        info = {'cost': r['cost'].float(), 'success': r['success'], 'collision': collided,
                'collision_moving': coll_moving, 'stalled': r['stalled'],
                'timeout': r['timeout'], 'interv': interv, 'fallback': fb_used,
                'terms': r['terms'], 'a_prev': a_prev, 'a_exec': a_exec,
                # 전이 뒤 값 (리셋 전 사본) — 분석 도구가 다른 RewardSpec 으로 보상을 다시 계산할 때 쓴다
                'post': {'d_arc': r['d_arc'].clone(), 'd_goal': r['d_goal'].clone(),
                         'e': r['proj'][1].clone(), 'v': self.v.clone()}}
        di = torch.nonzero(done)[:, 0]
        if di.numel():
            info['done_idx'] = di
            info['final_obs'] = obs[di].clone()
            if self.want_mask:
                info['final_mask'] = self.mask[di].clone()
            info['episode'] = {
                'return': self.ep_ret[di].clone(), 'length': self.ep_len[di].clone(),
                'success': r['success'][di], 'collision': collided[di],
                'collision_moving': coll_moving[di], 'stalled': r['stalled'][di],
                'timeout': r['timeout'][di], 'cost': self.ep_cost[di].clone(),
                'path_len': self.path_len[di].clone(), 'time': self.t[di].clone(),
                'fallback': self.ep_fallback[di].clone(), 'd_start': self.ep_d0[di].clone(),
                'd_final': torch.maximum(self.d_arc[di], self.d_goal[di]),
                'travel': self.ep_travel[di].clone(), 'spin_steps': self.ep_spin[di].clone(),
                'max_e': self.ep_maxe[di].clone(), 'goal_min': self.ep_gmin[di].clone()}
            self._reset_idx(di)
            # 리셋된 env 의 관측: 부분집합 eager 대신 고정 N 컴파일 함수로 전체를 다시 계산해 고른다
            # (커널 발행 수가 부분집합 크기와 무관해진다)
            obs = torch.where(done[:, None], self._k['observe'](self.bstate()), obs)
            if self.want_mask:
                st = self.bstate().index(di)
                pj = T.project_onto_path(st.path_pts, st.path_n, st.path_cum, st.pose[:, :2])
                self.mask[di] = T.compute_mask(st, pj)['mask']
        return obs, rew.float(), term, trunc, info


def _mask_static(st, proj):
    """비교군 C 마스크 (고정 후보 수 — 컴파일 경로)."""
    return T.compute_mask(st, proj, static=True)['mask']


def _observe_all(st):
    """전체 env 관측 (투영 포함)."""
    proj = T.project_onto_path(st.path_pts, st.path_n, st.path_cum, st.pose[:, :2])
    return T.build_observation(st, proj)


def _sense_rest(ranges, pose_d, ob, lsig, bdrop, nz, ud, psig, vsig, pdrop, nxy, nv, udr,
                ray=RC.raycast_obstacles):
    """격자 거리 + 장애물 레이캐스트 + 잡음·드롭아웃 + 트랙 (난수는 밖에서)."""
    lx, ly = RC.lidar_origin(pose_d)
    ang = RC.beam_angles(pose_d.device)
    th = pose_d[:, 2].float()[:, None] + ang[None, :]
    dx, dy = torch.cos(th), torch.sin(th)
    if ob['xy'].shape[1] > 0:
        ranges = torch.minimum(ranges, ray(lx.float()[:, None], ly.float()[:, None], dx, dy, ob))
    r = torch.where(torch.isfinite(ranges), ranges + nz * lsig.float()[:, None], ranges)
    valid = torch.isfinite(r) & (r >= C.LIDAR_RANGE_MIN) & (r <= C.LIDAR_RANGE_MAX)
    valid = valid & ~(ud < bdrop.float()[:, None])
    r = torch.where(valid, r, torch.full_like(r, INF))
    px, py, ok = T.scan_points(r)
    out = {'ranges': r, 'pts': (px, py, ok)}
    if ob['xy'].shape[1] > 0:
        out['trk'] = K.track_obs(ob, psig, vsig, pdrop, nxy, nv, udr)
    return out
