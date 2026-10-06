"""
정본 φ env 결선 기록 — Isaac 폐루프에서 게이트가 받은 입력과 출력, 그리고 그 입력의 **독립 출처**를 남긴다.

    rl/isaac/run_isaac.sh wire rl/isaac/scripts/record_phi_wiring.py \
        --policy <policy.pt> --out <dir>

tools/phi_crosscheck/replay_env.py 가 이 기록으로 (1) 결선 불변식을 검사하고 (2) 에피소드마다 새 C++ SafetyGate 에
명세대로 만든 입력을 재생해 torch 출력과 대조한다 (A3d 노트 2026-10-06_2129).

기록 (env 0..N−1, 결정 스텝 k = 0..K−1, 서브스텝 j = 0..4):
  scan_r [K+1, N, 720]        결정 k 의 스캔 거리 = env.scan (set_scan 직후, 관측에 들어간 것과 같은 버퍼)
  scan_stamp_gate [K+1, N]    set_scan 직후 게이트의 scan_stamp (기대 0.1·k)
  plant_v/w [K, 5, N]         φ 를 부르는 순간의 env.plant_v/w (게이트에 넘긴 측정과 따로 읽는다)
  pairs [K, N, 5, 4]          스텝 뒤 env._pairs = (프로파일러 출력 v_ref, w_ref, φ 출력 v, w)
  arg_* [K, 5, N]             φ.step 에 실제로 넘어간 인자 (v_ref, w_ref, v_meas, w_meas) 와 now [K, 5]
  out_<채널> [K, 5, N]        torch 게이트 출력 (OUT_NAMES 중 estop 제외)
  reset_mask [K+1, N]         결정 k 직전에 리셋된 env (k = 0 = 첫 env.reset, k ≥ 1 = 스텝 k−1 의 종료 뒤)
  reset_fresh [K+1, N]        그 리셋 직후 게이트 상태가 새 게이트와 같은가 (리셋 안 한 env 는 True)
  obs_prox [K+1, N]           결정 k 의 관측에 든 근접 정지 비트 (contract 관측 safety 블록 5 번째)
  shadow_* [K, 5, N]          (선택) 가지치기 없는 그림자 게이트 — 같은 입력, 출력만 기록
행동: env 앞쪽 (1 − random_frac) 은 정책의 확률 행동 μ + σ·ε, 뒤쪽은 원출력 U[−2, 2].
"""

import argparse
import json
import math
import os
import sys
import time

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument('--policy', required=True)
ap.add_argument('--num_envs', type=int, default=64)
ap.add_argument('--steps', type=int, default=600)
ap.add_argument('--seed', type=int, default=777)
ap.add_argument('--random_frac', type=float, default=0.5)
ap.add_argument('--path_len', default='3:15')
ap.add_argument('--eval_cfg', action='store_true', help='평가 설정 (held-out 풀, DR 끔)')
ap.add_argument('--no_shadow', action='store_true', help='가지치기 없는 그림자 게이트를 끈다')
ap.add_argument('--out', required=True)
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

sys.path.insert(0, '/workspace/amr/rl/isaac')
import numpy as np  # noqa: E402
import torch  # noqa: E402

from amr_isaac import env_math as EM  # noqa: E402
from amr_isaac import policy_export as PE  # noqa: E402
from amr_isaac import safety_gate_torch as SG  # noqa: E402
from amr_isaac.env import AmrNavEnv  # noqa: E402
from amr_isaac.env_cfg import AmrNavEnvCfg, AmrNavEvalEnvCfg  # noqa: E402
from amr_rl_contract.observation import obs_slices  # noqa: E402

CHANNELS = ('v', 'w', 'zone', 'prox', 'causes', 'escaping', 'retreat', 'yield_escaping',
            'ttc_limited', 'cloud_stale', 'command_stale', 'speed_limit', 'angular_limit',
            'min_distance', 'lidar_distance', 'cloud_distance', 'contact_distance',
            'exclusion_distance', 'min_ttc')
SHADOW = ('v', 'w', 'prox', 'zone', 'min_distance', 'lidar_distance', 'contact_distance')
PROX_IDX = obs_slices()['safety'].start + 5          # zone one-hot 4, speed_limit, proximity_stop


class UnprunedGate(SG.SafetyGateBatch):
    """그림자: 빔 창 통계를 원본 가지치기 없이 "유한 중심 창들의 k 번째 값의 최소" 로 (헤더 :259-261 정의)."""

    def _robust(self, values, spec):
        s = SG.window_kth(values, spec)
        s = torch.where(torch.isfinite(values), s, torch.full_like(s, math.inf))
        return s.min(dim=1).values


def gate_state(g, ids):
    """게이트 상태 텐서 (env 축이 있는 것 전부) 의 ids 행."""
    out = {}
    for k, v in vars(g).items():
        if torch.is_tensor(v) and v.dim() >= 1 and v.shape[0] == g.n:
            out[k] = v[ids]
    return out


def same_state(a, fresh):
    """행마다 a == fresh (NaN 끼리 같다) → [n] bool, 다른 키 목록."""
    ok = None
    bad = []
    for k, f in fresh.items():
        x = a[k]
        f = f.expand_as(x)
        eq = (x == f)
        if x.is_floating_point():
            eq = eq | (torch.isnan(x) & torch.isnan(f))
        eq = eq.reshape(eq.shape[0], -1).all(dim=1)
        if not bool(eq.all()):
            bad.append(k)
        ok = eq if ok is None else ok & eq
    return ok, bad


def main():
    """기록."""
    os.makedirs(args.out, exist_ok=True)
    cfg = AmrNavEvalEnvCfg() if args.eval_cfg else AmrNavEnvCfg()
    cfg.safety_filter = 'canonical'
    cfg.scene.num_envs = args.num_envs
    cfg.seed = args.seed
    if args.path_len:
        cfg.path_len_min, cfg.path_len_max = (float(x) for x in args.path_len.split(':'))
    env = AmrNavEnv(cfg)
    dev = env.device
    N, K = env.num_envs, args.steps
    ck = torch.load(args.policy, map_location='cpu')
    pol = PE.PolicyB.from_rsl_state_dict(ck['state_dict'], ck.get('activation', 'elu')).to(dev)
    pol.eval()
    gen = torch.Generator(device=dev)
    gen.manual_seed(args.seed)
    n_rand = int(round(args.random_frac * N))
    rand_mask = torch.zeros(N, dtype=torch.bool, device=dev)
    rand_mask[N - n_rand:] = True

    phi = env.phi
    new_gate = EM.CanonicalPhi(1, dev).gate
    fresh = {k: v.clone() for k, v in gate_state(new_gate, slice(0, 1)).items()}
    shadow = None
    if not args.no_shadow:
        cap = SG.max_half_for_range(0.99 * EM.C.LIDAR_RANGE_MIN, EM.C.LIDAR_ANGLE_INC)
        shadow = UnprunedGate(N, dev, SG.DEPLOYED, n_beams=EM.C.LIDAR_SAMPLES,
                              angle_increment=EM.C.LIDAR_ANGLE_INC, circular=True,
                              origin=(EM.C.LIDAR_X, EM.C.LIDAR_Y), half_cap=cap)

    dec = [0]
    sub = [0]
    R = {
        'scan_r': np.full((K + 1, N, EM.C.LIDAR_SAMPLES), np.nan),
        'scan_stamp_gate': np.full((K + 1, N), np.nan),
        'scan_stamp_arg': np.full((K + 1,), np.nan),
        'scan_calls': np.zeros((K + 1,), np.int64),
        'plant_v': np.zeros((K, 5, N)), 'plant_w': np.zeros((K, 5, N)),
        'arg_v_ref': np.zeros((K, 5, N)), 'arg_w_ref': np.zeros((K, 5, N)),
        'arg_v_meas': np.zeros((K, 5, N)), 'arg_w_meas': np.zeros((K, 5, N)),
        'arg_now': np.zeros((K, 5)),
        'pairs': np.zeros((K, N, 5, 4)),
        'reset_mask': np.zeros((K + 1, N), bool),
        'reset_fresh': np.ones((K + 1, N), bool),
        'obs_prox': np.zeros((K + 1, N), np.float32),
        'random_env': rand_mask.cpu().numpy(),
    }
    for c in CHANNELS:
        R[f'out_{c}'] = np.zeros((K, 5, N))
    if shadow is not None:
        for c in SHADOW:
            R[f'shadow_{c}'] = np.zeros((K, 5, N))
    bad_keys = set()

    orig_set_scan = phi.set_scan
    orig_step = phi.step
    orig_reset_idx = env._reset_idx

    def set_scan(ranges, stamp):
        orig_set_scan(ranges, stamp)
        if shadow is not None:
            shadow.set_scan(phi.beams(ranges.to(SG.F64)), stamp)
            shadow.set_cloud(None, stamp, stamp)
        k = dec[0]
        R['scan_r'][k] = env.scan.detach().to(torch.float64).cpu().numpy()
        R['scan_stamp_gate'][k] = phi.gate.scan_stamp.detach().cpu().numpy()
        R['scan_stamp_arg'][k] = float(stamp)
        R['scan_calls'][k] += 1

    def step(v_ref, w_ref, v_meas, w_meas, now):
        k, j = dec[0], sub[0]
        R['plant_v'][k, j] = env.plant_v.detach().cpu().numpy()
        R['plant_w'][k, j] = env.plant_w.detach().cpu().numpy()
        out = orig_step(v_ref, w_ref, v_meas, w_meas, now)
        R['arg_v_ref'][k, j] = v_ref.detach().cpu().numpy()
        R['arg_w_ref'][k, j] = w_ref.detach().cpu().numpy()
        R['arg_v_meas'][k, j] = v_meas.detach().cpu().numpy()
        R['arg_w_meas'][k, j] = w_meas.detach().cpu().numpy()
        R['arg_now'][k, j] = float(now)
        for c in CHANNELS:
            R[f'out_{c}'][k, j] = out[c].detach().to(torch.float64).cpu().numpy()
        if shadow is not None:
            shadow.set_measured(env.plant_v, env.plant_w, now)
            shadow.set_command(v_ref, w_ref, now)
            so = shadow.evaluate(now)
            for c in SHADOW:
                R[f'shadow_{c}'][k, j] = so[c].detach().to(torch.float64).cpu().numpy()
        sub[0] += 1
        return out

    def reset_idx(env_ids):
        orig_reset_idx(env_ids)
        if env_ids is None:
            ids = torch.arange(N, device=dev)
        else:
            ids = torch.as_tensor(env_ids, device=dev).long()
        if shadow is not None:
            shadow.reset(ids)
        k = dec[0]
        R['reset_mask'][k, ids.cpu().numpy()] = True
        ok, bad = same_state(gate_state(phi.gate, ids), fresh)
        R['reset_fresh'][k, ids.cpu().numpy()] = ok.cpu().numpy()
        bad_keys.update(bad)

    phi.set_scan = set_scan
    phi.step = step
    env._reset_idx = reset_idx

    t0 = time.time()
    with torch.inference_mode():
        obs, _ = env.reset(seed=args.seed)
        R['obs_prox'][0] = obs['policy'][:, PROX_IDX].cpu().numpy()
        for k in range(K):
            dec[0], sub[0] = k, 0
            o = obs['policy']
            _a, mu, log_std = pol(o)
            eps = torch.randn(mu.shape, generator=gen, device=dev, dtype=mu.dtype)
            raw = mu + torch.exp(log_std) * eps
            u = torch.rand(mu.shape, generator=gen, device=dev, dtype=mu.dtype) * 4.0 - 2.0
            raw = torch.where(rand_mask.unsqueeze(1), u, raw)
            dec[0] = k
            # 스텝 안의 φ 호출은 결정 k, 스텝 뒤 리셋·관측(set_scan)은 결정 k+1 로 센다
            orig_get_dones = env._get_dones

            def get_dones(_k=k, _f=orig_get_dones):
                if sub[0] != 5:
                    raise RuntimeError(f'결정 {_k}: 서브스텝 {sub[0]} != 5')
                dec[0] = _k + 1
                return _f()
            env._get_dones = get_dones
            obs, _r, _t, _tr, _ = env.step(raw)
            env._get_dones = orig_get_dones
            R['pairs'][k] = env._pairs.detach().cpu().numpy()
            R['obs_prox'][k + 1] = obs['policy'][:, PROX_IDX].cpu().numpy()
    torch.cuda.synchronize()
    wall = time.time() - t0
    meta = {
        'policy': args.policy, 'num_envs': N, 'steps': K, 'seed': args.seed,
        'random_frac': args.random_frac, 'eval_cfg': bool(args.eval_cfg),
        'path_len': args.path_len, 'dr': bool(cfg.dr.enable), 'lidar_source': cfg.lidar_source,
        'phi_cloud_mode': cfg.phi_cloud_mode, 'shadow_unpruned': shadow is not None,
        'prox_obs_index': PROX_IDX, 'reset_state_bad_keys': sorted(bad_keys),
        'wall_s': round(wall, 1), 'loadavg': open('/proc/loadavg').read().split()[:3],
        'torch_max_alloc_MiB': round(torch.cuda.max_memory_allocated() / 2**20, 1),
        'phi_clock_end': env._phi_clock,
    }
    np.savez_compressed(os.path.join(args.out, 'wiring.npz'), **R)
    with open(os.path.join(args.out, 'record.json'), 'w') as f:
        json.dump(meta, f, indent=1, ensure_ascii=False)
    print('RECORD_PHI_WIRING ' + json.dumps(meta, ensure_ascii=False), flush=True)
    return 0


try:
    rc = main()
except Exception:
    import traceback
    traceback.print_exc()
    rc = 2
os._exit(rc)
