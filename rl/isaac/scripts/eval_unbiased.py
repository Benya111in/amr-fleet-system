"""
편향 없는 held-out 평가 — env 마다 리셋 직후 첫 에피소드 하나만 센다 (결정론 행동 tanh(mu)).

    rl/isaac/run_isaac.sh ev rl/isaac/scripts/eval_unbiased.py --policy <run>/policy_final.pt \
        --num_envs 2048 --seed 4242 --path_len 3:15 --tag final_heldout_3_15

eval_policy.py 는 끝난 에피소드를 n 개까지 세어 짧게 끝나는 결말 쪽으로 치우친다 (1차 물결 리뷰). 여기서는
rl/sim2d ppo.evaluate 와 같은 추정량을 쓴다:
  - 평가 seed 로 env.reset(seed) — 같은 seed 로 평가한 정책들은 첫 에피소드의 경로(시작·목표)가 같다
    (공통 난수; 경로 번호 해시 path_ids_sha 로 확인한다). 명목 동역학(DR 끔)이고 LiDAR 잡음은 명목 σ.
  - 시간 상한(min(120, 20 + 3L) s)까지 돌리면 모든 첫 에피소드가 끝난다 — 끝나지 않은 env 수 unfinished 를
    적고, 0 이 아니면 rc 1.
  - 결과: <out>/eval.json (요약), <out>/episodes.npz (에피소드별), <out>/obs_batch.npz (ONNX 왕복 시험용).
판정 대상·seed·경로 범위는 학습 전에 정한다 (runs/<id>/protocol.json).

safety φ (--phi): lite = 1·2차 물결 근사 (r7 정책이 이것으로 학습), canonical = 정본 SafetyGate 이식 (A3).
층위 성공 지표 (A3 노트 2026-10-06_1901 에서 결과 전에 정함):
  엄격      = env 성공 종단 (0.10 m ∧ |v| ≤ 0.05 m/s)
  Nav2 호환 = 첫 에피소드 동안 목표 직선거리 ≤ 0.25 m 인 결정 스텝이 충돌 스텝이 아닌 채로 한 번이라도 (속도 무관)
  최종 거리 = 에피소드가 끝난 스텝의 목표 직선거리 분포 (중앙·p90·최대, 실패만 따로)
  dwell-k   = --dwell_k k > 1 인 별도 실행: 성공 조건을 k 결정 스텝 연속 유지해야 성공 종단
진단 (episodes.npz 의 diag_*): 에피소드 끝의 마지막 --diag_steps 결정 스텝 동안 서브스텝별 φ 입력·출력
(v_ref, w_ref, v_out, w_out), φ 의 D·접촉 가드 거리·존·래치·원인, 목표 거리, 측정 속도.
"""

import argparse
import hashlib
import json
import math
import os
import sys
import time

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument('--policy', required=True)
ap.add_argument('--num_envs', type=int, default=2048)
ap.add_argument('--seed', type=int, default=4242)
ap.add_argument('--path_len', default='3:15', help="경로 길이 범위 'LO:HI' [m], 빈 값 = 풀 전체")
ap.add_argument('--train_pool', action='store_true', help='학습 풀(seed 0)에서 (기본 held-out seed 1)')
ap.add_argument('--tag', default='heldout')
ap.add_argument('--out', default=None)
ap.add_argument('--phi', default='lite', choices=('lite', 'canonical'), help='safety φ')
ap.add_argument('--phi_cloud', default='empty', choices=('empty', 'none'),
                help='정본 φ 의 깊이 점군 모델 (env_math.CanonicalPhi)')
ap.add_argument('--dwell_k', type=int, default=1, help='>1 이면 성공 조건 k 스텝 연속 유지 (평가 전용)')
ap.add_argument('--diag_steps', type=int, default=20, help='진단 링 버퍼 길이 (결정 스텝)')
ap.add_argument('--nav2_radius', type=float, default=0.25)
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

sys.path.insert(0, '/workspace/amr/rl/isaac')
import numpy as np  # noqa: E402
import torch  # noqa: E402

from amr_isaac import eval_metrics as EV  # noqa: E402
from amr_isaac import policy_export as PE  # noqa: E402
from amr_isaac.env import AmrNavEnv  # noqa: E402
from amr_isaac.env_cfg import AmrNavEvalEnvCfg  # noqa: E402
from amr_isaac.env_math import OUTCOME_CODES  # noqa: E402
from amr_rl_contract import constants as C  # noqa: E402


def summarize(ep, n_envs, unfinished):
    """에피소드별 배열 → 요약 (rl/sim2d ppo.episode_summary 와 같은 진단 이름)."""
    n = int(ep['outcome'].shape[0])
    out = {'episodes': n, 'unfinished': unfinished, 'num_envs': n_envs,
           'estimator': 'first_episode_per_env'}
    for i, o in enumerate(OUTCOME_CODES):
        out[o] = float((ep['outcome'] == i).mean())
    p = out['success']
    out['success_se'] = math.sqrt(p * (1 - p) / max(n, 1))
    succ = ep['outcome'] == 0
    t = ep['time_s']
    out['time_s_mean_all'] = float(t.mean())
    out['time_s_mean_success'] = float(t[succ].mean()) if succ.any() else None
    out['time_s_median_success'] = float(np.median(t[succ])) if succ.any() else None
    out['path_len_mean'] = float(ep['path_len'].mean())
    out['path_len_mean_success'] = float(ep['path_len'][succ].mean()) if succ.any() else None
    out['speed_mean_success'] = (float((ep['path_len'][succ] / t[succ]).mean())
                                 if succ.any() else None)
    out['return_mean'] = float(ep['return'].mean())
    steps = np.maximum(ep['steps'], 1)
    out['progress'] = float((1.0 - ep['d_final'] / np.maximum(ep['d_start'], 1e-6)).mean())
    out['travel_ratio'] = float((ep['travel'] / np.maximum(ep['path_len'], 1e-6)).mean())
    out['spin_frac'] = float((ep['spin'] / steps).mean())
    out['offpath_1m_frac'] = float((ep['cost'][:, 2] / steps).mean())
    out['max_e_mean'] = float(ep['maxe'].mean())
    out['max_e_max'] = float(ep['maxe'].max())
    out['goal_min_median'] = float(np.median(ep['gmin']))
    out['near_goal_0p5'] = float((ep['gmin'] <= 0.5).mean())
    for i, k in enumerate(C.COST_NAMES):
        out[f'cost_{k}_per_ep'] = float(ep['cost'][:, i].mean())
    # 층위 성공 지표와 목표 직전 정지 진단 (정의: amr_isaac/eval_metrics.py, A3 노트)
    out['layered'] = EV.layered(ep)
    dg = EV.stop_diagnosis(ep)
    out['stop_diagnosis'] = {'n': dg['n'], 'counts': dg['counts']}
    return out


def main():
    """평가."""
    out = args.out or os.path.join(os.path.dirname(args.policy), f'eval_{args.tag}')
    os.makedirs(out, exist_ok=True)
    ck = torch.load(args.policy, map_location='cpu')
    pol = PE.PolicyB.from_rsl_state_dict(ck['state_dict'], ck.get('activation', 'elu'))
    cfg = AmrNavEvalEnvCfg()
    cfg.safety_filter = args.phi
    cfg.phi_cloud_mode = args.phi_cloud
    cfg.success_dwell_steps = args.dwell_k
    if args.train_pool:
        cfg.path_pool_seed = 0
    cfg.scene.num_envs = args.num_envs
    cfg.seed = args.seed
    if args.path_len:
        cfg.path_len_min, cfg.path_len_max = (float(x) for x in args.path_len.split(':'))
    env = AmrNavEnv(cfg)
    pol = pol.to(env.device).eval()
    obs, _ = env.reset(seed=args.seed)
    N = env.num_envs
    first_path = env.pool_kept[env.path_id.cpu().numpy()]
    max_tl = float(env.time_limit.max())
    limit = int(math.ceil(max_tl / C.DT)) + 5
    log = env.ep_log
    got = torch.zeros(N, dtype=torch.bool, device=env.device)
    keys = list(log.fin)
    rec = {k: torch.zeros_like(v) for k, v in log.fin.items()}
    keep_obs, keep_out = [], []
    dev = env.device
    K = args.diag_steps
    # 층위 지표·진단 버퍼 (첫 에피소드)
    nav2 = torch.zeros(N, dtype=torch.bool, device=dev)
    d_end = torch.zeros(N, dtype=torch.float64, device=dev)
    pose_end = torch.zeros(N, 3, dtype=torch.float64, device=dev)
    ring = {'pairs': torch.zeros(K, N, 5, 4, dtype=torch.float64, device=dev)}
    for nm in ('d_goal', 'v_meas', 'w_meas', 'D', 'guard'):
        ring[nm] = torch.zeros(K, N, dtype=torch.float64, device=dev)
    for nm in ('zone', 'causes'):
        ring[nm] = torch.zeros(K, N, dtype=torch.long, device=dev)
    for nm in ('prox', 'escaping'):
        ring[nm] = torch.zeros(K, N, dtype=torch.bool, device=dev)
    diag = {nm: torch.zeros((N, K) + tuple(v.shape[2:]), dtype=v.dtype, device=dev)
            for nm, v in ring.items()}
    t0 = time.time()
    steps = 0
    with torch.inference_mode():
        while steps < limit:
            o = obs['policy']
            action, mu, log_std = pol(o)
            if len(keep_obs) < 8 and steps % 50 == 0:
                keep_obs.append(o[:64].cpu().numpy())
                keep_out.append(np.concatenate([action[:64].cpu().numpy(), mu[:64].cpu().numpy(),
                                                log_std[:64].cpu().numpy()], 1))
            obs, _r, _t, _tr, _ = env.step(mu)
            sd = env.step_diag
            slot = steps % K
            ring['pairs'][slot] = env._pairs
            for nm in ('d_goal', 'v_meas', 'w_meas', 'D', 'guard', 'zone', 'causes', 'prox',
                       'escaping'):
                ring[nm][slot] = sd[nm]
            nav2 = EV.nav2_update(nav2, got, sd['d_goal'], sd['collision'], args.nav2_radius)
            steps += 1
            new = (log.fin_count > 0) & ~got
            if bool(new.any()):
                for k in keys:
                    m = new.view(-1, *([1] * (rec[k].dim() - 1)))
                    rec[k] = torch.where(m, log.fin[k], rec[k])
                d_end = torch.where(new, sd['d_goal'], d_end)
                pose_end = torch.where(new.unsqueeze(1), sd['pose'], pose_end)
                order = [(slot + 1 + j) % K for j in range(K)]       # 오래된 것 → 마지막 스텝
                for nm, v in ring.items():
                    m = new.view(-1, *([1] * (v.dim() - 1)))
                    seq = v[order].transpose(0, 1)
                    diag[nm] = torch.where(m, seq, diag[nm])
                got |= new
                if bool(got.all()):
                    break
    torch.cuda.synchronize()
    wall = time.time() - t0
    unfinished = int((~got).sum())
    gm = got.cpu().numpy()
    ep = {k: v.cpu().numpy()[gm] for k, v in rec.items()}
    ep['nav2_reach'] = nav2.cpu().numpy()[gm]
    ep['d_end'] = d_end.cpu().numpy()[gm]
    ep['pose_end'] = pose_end.cpu().numpy()[gm]
    for nm, v in diag.items():
        ep[f'diag_{nm}'] = v.cpu().numpy()[gm]
    ep['path_id_orig'] = env.pool_kept[ep['path_id']]
    if not np.array_equal(ep['path_id_orig'], first_path[gm]):
        raise RuntimeError('첫 에피소드 경로 번호가 리셋 때와 다르다 (사본이 첫 에피소드가 아님)')
    res = summarize(ep, N, unfinished)
    res.update({
        'policy': args.policy, 'tag': args.tag, 'eval_seed': args.seed,
        'pool': 'train(seed0)' if args.train_pool else 'heldout(seed1)',
        'safety_filter': args.phi, 'phi_cloud_mode': args.phi_cloud if args.phi == 'canonical'
        else None, 'dwell_k': args.dwell_k, 'nav2_radius': args.nav2_radius,
        'diag_steps': K,
        'path_len_range': args.path_len, 'pool_paths': env.pool_size, 'dr': False,
        'deterministic': 'tanh(mu)', 'reward_version': cfg.reward_version,
        'contract_version': C.CONTRACT_VERSION, 'steps': steps, 'step_limit': limit,
        'wall_s': round(wall, 1), 'loadavg': open('/proc/loadavg').read().split()[:3],
        'torch_max_alloc_MiB': round(torch.cuda.max_memory_allocated() / 2**20, 1),
        'path_ids_sha': hashlib.sha256(
            np.ascontiguousarray(first_path).tobytes()).hexdigest()[:16],
        'policy_log_std': [float(x) for x in pol.log_std.flatten().tolist()],
    })
    np.savez_compressed(os.path.join(out, 'episodes.npz'), **ep)
    np.savez_compressed(os.path.join(out, 'obs_batch.npz'), obs=np.concatenate(keep_obs),
                        out=np.concatenate(keep_out))
    with open(os.path.join(out, 'eval.json'), 'w') as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    print('EVAL_UNBIASED ' + json.dumps(res, ensure_ascii=False), flush=True)
    return 0 if unfinished == 0 else 1


try:
    rc = main()
except Exception:
    import traceback
    traceback.print_exc()
    rc = 2
os._exit(rc)
