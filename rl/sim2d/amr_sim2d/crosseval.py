"""
B5 교차 평가 — 같은 평가 집합(경로·시작 yaw 고정)에서 env 마다 첫 에피소드 하나만, RL 팔과 2D-DWA 팔을 같은 사슬로.

`FixedEpisodeEnv` 는 `Sim2DEnv` 를 고치지 않고 감싼다 (기존 기본 동작은 그대로):
  - env i = 경로 i · 시작 yaw i (집합 파일에서). 처음 리셋 뒤에는 리셋하지 않는다 — 끝난 env 는 그 자리에 세워 두고
    첫 에피소드만 기록한다 (Isaac `eval_unbiased.py` 의 "env 마다 첫 에피소드" 와 같은 추정량).
  - `step_rl(u)`: 정책 행동 → `integrate_b` → 명령 램프 → 서브스텝 5 회. `Sim2DEnv.step` 과 같은 계산
    (시험이 비트 대조).
  - `step_dwa(dwa)`: 팀 DWA 를 **20 Hz** 로 부른다 — 결정 주기 0.1 s 안에서 t (서브스텝 0 앞) 과
    t + 0.06 s (서브스텝 3 앞) 두 번, 각 명령을 그다음 서브스텝들에 0 차 유지로 넣는다
    (간격 0.06/0.04 s, 평균 0.05 s = controller_frequency 20 Hz).
    명령이 들어가는 자리는 RL 과 같다: 프로파일러 목표 → safety φ (safety_lite) → 플랜트.
  - 두 팔 모두 서브스텝마다 φ 의 보정 크기를 따로 재 개입 사건·시간·STOP 존 시간을 같은 정의로 센다.
난수: 스캔 잡음은 env 생성기에서 스텝마다 전 env 에 한 번에 뽑으므로, 리셋하지 않는 한 두 팔의 (env, 스텝) 잡음이 같다.
"""

import math
import os
import sys

import numpy as np
import torch

from . import contract_torch as T
from . import raycast as RC
from ._contract import C, RL_DIR
from .env import INF, SPIN_V, SPIN_W, SUB, EnvConfig, Sim2DEnv

# 보정 크기 √((Δv/1.0)² + (Δω/1.5)²) 가 이보다 크면 "개입 중" 서브스텝 (Gazebo 하네스 0.01 과 같은 문턱)
INTERV_EPS = 0.01


def export_world(geom_npz, out_dir, res=0.025, pad=0.0, name=None):
    """
    Isaac 기하를 해상도 res 로 다시 래스터화해 map_server 형식 PGM + YAML 로 쓴다 (반환: YAML 경로).

    입력 = Isaac 기하 npz (rl/isaac `warehouse_geom`: warehouse.sdf 충돌 상자 → 2D 사각형 129 개 `obb`).
    `warehouse_geom.rasterize` — 셀 중심이 사각형 + pad 안이면 점유 (점유 0, 자유 254, 미지 없음).
    Sim2DEnv 는 cfg.map_yaml 로 이 파일을 읽는다 (env 수정 없음).

    B5 주 세계 W1 = res 0.025 · pad 0: 셀 사각형 합집합과 실제 사각형의 차가 축마다 반 셀(0.0125 m) 이하.
    Isaac 이 경로 계획에 쓴 0.05 m · pad 0.025 격자(`occ`)는 장애물을 접근 방향으로 0–0.05 m 부풀린다
    (b5_mapcheck: 직진 여유 2D − Isaac 중앙 −0.05 m) — 그래서 그대로 쓰지 않는다.
    """
    sys.path.insert(0, os.path.join(RL_DIR, 'isaac'))
    from amr_isaac.warehouse_geom import rasterize
    z = np.load(geom_npz)
    o = np.asarray(z['origin'], dtype=np.float64)
    ny0, nx0 = np.asarray(z['occ']).shape
    r0 = float(z['res'])
    bounds = (o[0], o[0] + nx0 * r0, o[1], o[1] + ny0 * r0)
    occ, origin, _ = rasterize(np.asarray(z['obb']), res=res, bounds=bounds, pad=pad)
    img = np.where(occ, 0, 254).astype(np.uint8)[::-1]   # PGM 행 0 = 위
    name = name or f'world_isaac_obb_r{int(round(res * 1000)):03d}_p{int(round(pad * 1000)):03d}'
    os.makedirs(out_dir, exist_ok=True)
    pgm = os.path.join(out_dir, name + '.pgm')
    with open(pgm, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (img.shape[1], img.shape[0]))
        f.write(np.ascontiguousarray(img).tobytes())
    yml = os.path.join(out_dir, name + '.yaml')
    with open(yml, 'w', encoding='utf-8') as f:
        f.write(f'image: {name}.pgm\nmode: trinary\nresolution: {float(res)!r}\n'
                f'origin: [{float(origin[0])!r}, {float(origin[1])!r}, 0]\nnegate: 0\n'
                f'occupied_thresh: 0.65\nfree_thresh: 0.19\n'
                f'# 출처: {os.path.abspath(geom_npz)} (obb 129, rasterize res {res} pad {pad})\n')
    return yml


def start_yaw_isaac(path_xy, noise):
    """Isaac env 규칙: 접선 = 첫 점 → 10 번째 점(0.5 m 앞) 방향 + 잡음 (rl/isaac/amr_isaac/env.py:407-411)."""
    j = min(10, len(path_xy) - 1)
    d = path_xy[j] - path_xy[0]
    return math.atan2(d[1], d[0]) + noise


class FixedEpisodeEnv(Sim2DEnv):
    """평가 전용 env — 고정 경로·시작 yaw, env 마다 첫 에피소드."""

    def __init__(self, cfg: EnvConfig, paths, yaws, log=print):
        """paths: list[[k,2]] (len = num_envs), yaws: [num_envs] 시작 yaw."""
        if len(paths) != cfg.num_envs or len(yaws) != cfg.num_envs:
            raise ValueError('paths·yaws 수가 num_envs 와 다르다')
        self._armed = False
        super().__init__(cfg, log)
        dev = self.dev
        pts, n, cum, ln = T.path_tensors([np.asarray(p, dtype=np.float64) for p in paths], dev)
        self.pool_pts, self.pool_n, self.pool_cum, self.pool_len = pts, n, cum, ln
        self.pool_L = ln.clone()
        self.pool_keep = torch.arange(len(paths), device=dev)
        self.P = pts.shape[1]
        N = self.N
        self.path_pts = torch.zeros(N, self.P, 2, dtype=self.fd, device=dev)
        self.path_cum = torch.zeros(N, self.P, dtype=self.fd, device=dev)
        self.fixed_yaw = torch.as_tensor(np.asarray(yaws, dtype=np.float64), device=dev)
        self.fixed_paths = [np.asarray(p, dtype=np.float64) for p in paths]
        z = torch.zeros(N, dtype=torch.long, device=dev)
        self.ep_interv_sub = z.clone()       # 개입 중 서브스텝 수
        self.ep_interv_events = z.clone()    # 개입 사건 수 (상승 모서리)
        self.ep_stop_sub = z.clone()         # STOP 존 서브스텝 수
        self._interv_prev = torch.zeros(N, dtype=torch.bool, device=dev)

    # ------------------------------------------------------------ 리셋
    def _reset_idx(self, idx):
        if self._armed:
            return                            # 첫 에피소드만: 끝난 env 는 세워 둔다
        super()._reset_idx(idx)
        self.path_pts[idx] = self.pool_pts[idx]
        self.path_n[idx] = self.pool_n[idx]
        self.path_cum[idx] = self.pool_cum[idx]
        self.path_len[idx] = self.pool_len[idx]
        p0 = self.pool_pts[idx, 0]
        pose = torch.cat([p0, T.wrap_angle(self.fixed_yaw[idx])[:, None]], dim=1)
        self.pose[idx] = pose
        self.hist[idx] = pose[:, None, :]
        self.base_xy[idx] = p0
        self.t_limit[idx] = T.time_limit(self.path_len[idx])
        self._sense(idx)
        proj = T.project_onto_path(self.path_pts[idx], self.path_n[idx], self.path_cum[idx],
                                   self.pose[idx, :2])
        self.d_arc[idx] = (self.path_len[idx] - proj[0]).clamp_min(0.0)
        self.d_goal[idx] = torch.linalg.norm(
            self.pose[idx, :2] - T.goal_xy(self.path_pts[idx], self.path_n[idx]), dim=-1)
        self.ep_d0[idx] = torch.maximum(self.d_arc[idx], self.d_goal[idx])
        self.ep_gmin[idx] = self.d_goal[idx]

    def reset(self):
        """전체 리셋 (고정 경로·yaw) 뒤 무장 — 이후 리셋하지 않는다."""
        self._armed = False
        obs = super().reset()
        self._armed = True
        for k in ('ep_interv_sub', 'ep_interv_events', 'ep_stop_sub'):
            getattr(self, k).zero_()
        self._interv_prev.zero_()
        return obs

    # ------------------------------------------------------------ 스텝
    @torch.no_grad()
    def step_rl(self, action):
        """B 행동 [N,2] (tanh 뒤) — Sim2DEnv.step 과 같은 전이 + 서브스텝 개입 계측."""
        v1, w1, av, aw = T.integrate_b(action.to(self.fd), self.v_cmd, self.w_cmd, self.v_cap)
        a_exec = torch.stack([av, aw], dim=1)
        return self._step_core(v1, w1, a_exec, None)

    @torch.no_grad()
    def step_dwa(self, dwa, active):
        """
        2D-DWA 한 결정 주기 (0.1 s) — DWA 를 t 와 t + 0.06 s 에 부른다.

        dwa: dwa2d.DwaBatch (env 번호 = 이 env 의 번호), active: numpy bool [N] (아직 첫 에피소드 중인 env).
        반환 (obs, reward, terminated, truncated, info) + info['dwa'] (두 틱의 found·exception 등).
        """
        N, fd, dev = self.N, self.fd, self.dev
        idx = np.nonzero(active)[0]
        tick = {'found': np.ones(N, dtype=np.int64) * 2, 'exception': np.zeros(N, dtype=np.int64),
                'n_valid0': np.zeros(N, dtype=np.int64)}

        def call(pose, v, w):
            vc = torch.zeros(N, dtype=fd, device=dev)
            wc = torch.zeros(N, dtype=fd, device=dev)
            if len(idx):
                p = pose[idx].cpu().numpy()
                r = dwa.step(idx, p, v[idx].cpu().numpy(), w[idx].cpu().numpy())
                vc[idx] = torch.as_tensor(r['v'], dtype=fd, device=dev)
                wc[idx] = torch.as_tensor(r['w'], dtype=fd, device=dev)
                tick['found'][idx] -= (~r['found']).astype(np.int64)
                tick['exception'][idx] += r['exception'].astype(np.int64)
                tick['n_valid0'][idx] += (r['n_valid'] == 0).astype(np.int64)
            return vc, wc
        cmd = {}
        cmd['a'] = call(self.pose, self.v, self.w)

        def sched(k, S):
            if k == 3:
                cmd['b'] = call(S['pose'], S['v'], S['w'])
            return cmd['b'] if k >= 3 else cmd['a']
        v_prev = self.v_cmd
        w_prev = self.w_cmd
        out = self._step_core(None, None, None, sched, v_prev=v_prev, w_prev=w_prev, cmd=cmd)
        out[4]['dwa'] = {'found_ticks': tick['found'], 'exceptions': tick['exception'],
                         'no_valid_ticks': 2 - tick['found']}
        return out

    def _step_core(self, v1, w1, a_exec, sched, v_prev=None, w_prev=None, cmd=None):
        """Sim2DEnv.step 의 행동 뒤 부분 (같은 순서·같은 컴파일 커널). sched(k, S) → 서브스텝 k 의 명령 (v, ω)."""
        cfg, N, dev, fd = self.cfg, self.N, self.dev, self.fd
        fb_used = torch.zeros(N, dtype=torch.bool, device=dev)
        w_pre = self.w
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
        isub = torch.zeros(N, dtype=torch.long, device=dev)
        iev = torch.zeros(N, dtype=torch.long, device=dev)
        istop = torch.zeros(N, dtype=torch.long, device=dev)
        prev = self._interv_prev
        for k in range(SUB):
            Xk = X
            if sched is not None:
                vc, wc = sched(k, S)
                Xk = dict(X)
                Xk['v_cmd'], Xk['w_cmd'], Xk['v1'], Xk['w1'] = vc, wc, vc, wc
            before = S['interv']
            S = self._k['substep'](S, Xk, self._kf[k], self._first[k], bool(cfg.safety))
            flag = (S['interv'] - before) > INTERV_EPS ** 2
            isub += flag.long()
            iev += (flag & ~prev).long()
            istop += (S['zone'] == C.ZONE_STOP).long()
            prev = flag
        self._interv_prev = prev
        if sched is not None:
            v1, w1 = cmd['b']
            a_exec = torch.stack([(v1 - v_prev) / C.DT, (w1 - w_prev) / C.DT], dim=1)
        for k in self._S_KEYS:
            setattr(self, k, S[k])
        p.lin_v, p.lin_a, p.ang_v, p.ang_a, p.w_ref, p.curv_mode = (
            S['lin_v'], S['lin_a'], S['ang_v'], S['ang_a'], S['w_ref'], S['curv'])
        collided, coll_moving = S['collided'], S['coll_moving']
        coll_turning = collided & ~coll_moving & (w_pre.abs() > 0.1)
        interv = S['interv'] / SUB
        self.t = self.t + C.DT
        if cfg.obstacle_slots:
            sep = self._k['minsep'](self.pose, self._obst_params(), self.t)
            sep = torch.where(collided, torch.zeros_like(sep), sep)
            self.ep_minsep = torch.minimum(self.ep_minsep, sep)
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
        self.ep_interv_sub += isub
        self.ep_interv_events += iev
        self.ep_stop_sub += istop
        obs, rew, term, trunc = r['obs'], r['reward'], r['terminated'], r['truncated']
        self.ep_ret += rew
        self.ep_len += 1
        self.ep_cost += r['cost']
        self.ep_fallback += fb_used.long()
        done = term | trunc
        info = {'success': r['success'], 'collision': collided, 'collision_moving': coll_moving,
                'collision_turning': coll_turning, 'stalled': r['stalled'],
                'timeout': r['timeout'], 'done': done}
        return obs, rew.float(), term, trunc, info

    def episode_snapshot(self, di):
        """번호 di (tensor) 인 env 들의 에피소드 기록 (끝난 직후 부른다) → dict of numpy."""
        g = {
            'return': self.ep_ret, 'steps': self.ep_len, 'time_s': self.t,
            'path_len': self.path_len, 'd_start': self.ep_d0,
            'd_final': torch.maximum(self.d_arc, self.d_goal), 'd_goal_end': self.d_goal,
            'travel': self.ep_travel, 'spin_steps': self.ep_spin, 'max_e': self.ep_maxe,
            'goal_min': self.ep_gmin, 'interv_sub': self.ep_interv_sub,
            'interv_events': self.ep_interv_events, 'stop_sub': self.ep_stop_sub,
            'v_end': self.v, 'w_end': self.w}
        out = {k: v[di].detach().cpu().numpy().copy() for k, v in g.items()}
        out['pose_end'] = self.pose[di].detach().cpu().numpy().copy()
        out['cost'] = self.ep_cost[di].detach().cpu().numpy().copy()
        return out


def sensor_poses(pose):
    """base_link 자세 [N,3] → LiDAR 원점 자세 [N,3] (yaw 같음)."""
    lx, ly = RC.lidar_origin(pose)
    return torch.stack([lx, ly, pose[:, 2]], dim=1)
