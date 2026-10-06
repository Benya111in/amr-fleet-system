"""
편향 없는 held-out 평가 — env 마다 리셋 직후 첫 에피소드 하나만 센다 (결정론 행동 tanh(mu)).

    rl/isaac/run_isaac.sh ev rl/isaac/scripts/eval_unbiased.py --policy <run>/policy_final.pt \
        --num_envs 2048 --seed 4242 --path_len 3:15 --tag final_heldout_3_15

B 단계 평가 집합·교란 묶음 (rl/isaac/README.md 「B 단계 실행자용」):
    S=rl/isaac/eval_sets
    ... eval_unbiased.py --policy <pt> --phi canonical --eval_set $S/b1_eval_sets_v1.json \
        --sets main_3_15,stress_3_15,main_3_30,stress_3_30 --out <dir>
    ... eval_unbiased.py --policy <pt> --phi canonical --eval_set $S/b1_eval_sets_v1.json \
        --sets perturb_3_15 --perturb $S/b1_perturb_suite_v1.json --out <dir>
  행 = (집합, 조건, 경로 번호) — 집합 × 조건의 모든 행을 --max_envs 개씩 라운드로 나눠 돈다. env i 는 그 라운드의 행 i 의
  경로(고정)와 교란(DR 끈 명목값 위에 덮어씀)으로 첫 에피소드 하나를 돈다. 결과는 (집합, 조건) 마다 따로 요약한다.

eval_policy.py 는 끝난 에피소드를 n 개까지 세어 짧게 끝나는 결말 쪽으로 치우친다 (1차 물결 리뷰). 여기서는
rl/sim2d ppo.evaluate 와 같은 추정량을 쓴다:
  - 평가 seed 로 env.reset(seed) — 같은 seed 로 평가한 정책들은 첫 에피소드의 경로(시작·목표)가 같다
    (공통 난수; 경로 번호 해시 path_ids_sha 로 확인한다). 명목 동역학(DR 끔)이고 LiDAR 잡음은 명목 σ.
  - 시간 상한(min(120, 20 + 3L) s)까지 돌리면 모든 첫 에피소드가 끝난다 — 끝나지 않은 env 수 unfinished 를
    적고, 0 이 아니면 rc 1.
  - 결과: <out>/eval.json (요약), <out>/episodes.npz (에피소드별), <out>/obs_batch.npz (ONNX 왕복 시험용).
판정 대상·seed·경로 범위는 학습 전에 정한다 (노트 항목).

safety φ (--phi): lite = 1·2차 물결 근사 (r7 정책이 이것으로 학습), canonical = 정본 SafetyGate 이식 (A3).
층위 성공 지표 (A3 노트 2026-10-06_1901 에서 결과 전에 정함):
  엄격      = env 성공 종단 (0.10 m ∧ |v| ≤ 0.05 m/s)
  Nav2 호환 = 첫 에피소드 동안 목표 직선거리 ≤ 0.25 m 인 결정 스텝이 충돌 스텝이 아닌 채로 한 번이라도 (속도 무관)
  최종 거리 = 에피소드가 끝난 스텝의 목표 직선거리 분포 (중앙·p90·최대, 실패만 따로)
  dwell-k   = --dwell_k k > 1 인 별도 실행: 성공 조건을 k 결정 스텝 연속 유지해야 성공 종단
집합별 요약에는 Wilson 95 % 구간(n 에피소드)과 **목표 군집 부트스트랩 95 % 구간**(같은 목표의 에피소드를 한 묶음으로
복원 추출 — 스트레스 집합은 서로 다른 목표가 28–32 개뿐이라 이항 n 보다 훨씬 넓다)을 함께 적는다.
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
ap.add_argument('--eval_set', default='',
                help='평가 집합 파일 (eval_sets.SET_FORMAT) — 주면 --path_len 무시')
ap.add_argument('--sets', default='', help="쉼표 목록 (빈 값 = perturb_* 를 뺀 전부)")
ap.add_argument('--perturb', default='', help='교란 묶음 파일 (perturb.SUITE_FORMAT)')
ap.add_argument('--conditions', default='', help='쉼표 목록 (빈 값 = 묶음의 전부)')
ap.add_argument('--max_envs', type=int, default=8192, help='집합 평가 한 라운드의 env 수 상한')
ap.add_argument('--boot_reps', type=int, default=2000)
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

sys.path.insert(0, '/workspace/amr/rl/isaac')
sys.path.insert(0, '/workspace/amr/rl/eval')
import numpy as np  # noqa: E402
import torch  # noqa: E402

from amr_isaac import eval_metrics as EV  # noqa: E402
from amr_isaac import eval_sets as ES  # noqa: E402
from amr_isaac import perturb as PT  # noqa: E402
from amr_isaac import policy_export as PE  # noqa: E402
from amr_isaac.assets import load_pool  # noqa: E402
from amr_isaac.env import AmrNavEnv  # noqa: E402
from amr_isaac.env_cfg import AmrNavEvalEnvCfg  # noqa: E402
from amr_isaac.env_math import OUTCOME_CODES  # noqa: E402
from amr_rl_contract import constants as C  # noqa: E402
from amr_rl_eval import binomial as BN  # noqa: E402


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


def interval_block(ep, groups):
    """엄격·Nav2 성공의 Wilson 95 % 와 목표 군집 부트스트랩 95 % 구간, 충돌 수."""
    succ = (ep['outcome'] == 0).astype(np.float64)
    nav2 = np.asarray(ep['nav2_reach'], dtype=np.float64)
    n = int(succ.shape[0])
    z = 1.959963984540054
    res = {'n': n, 'distinct_goals': int(np.unique(groups).shape[0]),
           'distinct_paths': int(np.unique(ep['path_id_orig']).shape[0]),
           'collisions': int((ep['outcome'] == 1).sum())}
    for nm, x in (('strict', succ), ('nav2', nav2)):
        k = int(x.sum())
        res[f'{nm}_successes'] = k
        res[f'{nm}_wilson95'] = list(BN.wilson_interval(k, n, z)) if n else None
        res[f'{nm}_goal_cluster_boot95'] = list(ES.cluster_bootstrap_ci(
            x, groups, reps=args.boot_reps, seed=0)) if n else None
    return res


def run_first_episodes(env, pol, keep_obs=None, keep_out=None):
    """env.reset(seed) 뒤 모든 env 의 첫 에피소드를 끝까지 → (에피소드별 numpy dict, unfinished, 스텝, 벽시계)."""
    obs, _ = env.reset(seed=args.seed)
    N = env.num_envs
    first_path = env.pool_kept[env.path_id.cpu().numpy()]
    max_tl = float(env.time_limit.max())
    limit = int(math.ceil(max_tl / C.DT)) + 5
    log = env.ep_log
    base = log.fin_count.clone()
    got = torch.zeros(N, dtype=torch.bool, device=env.device)
    keys = list(log.fin)
    rec = {k: torch.zeros_like(v) for k, v in log.fin.items()}
    dev = env.device
    K = args.diag_steps
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
            if keep_obs is not None and len(keep_obs) < 8 and steps % 50 == 0:
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
            new = (log.fin_count > base) & ~got
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
    ep = {k: v.cpu().numpy() for k, v in rec.items()}
    ep['nav2_reach'] = nav2.cpu().numpy()
    ep['d_end'] = d_end.cpu().numpy()
    ep['pose_end'] = pose_end.cpu().numpy()
    for nm, v in diag.items():
        ep[f'diag_{nm}'] = v.cpu().numpy()
    ep['path_id_orig'] = env.pool_kept[ep['path_id']]
    ep['got'] = gm
    if not np.array_equal(ep['path_id_orig'][gm], first_path[gm]):
        raise RuntimeError('첫 에피소드 경로 번호가 리셋 때와 다르다 (사본이 첫 에피소드가 아님)')
    return ep, first_path, unfinished, steps, limit, wall


def make_env(n_envs, path_len):
    """평가 env (DR 끔, 명목값)."""
    cfg = AmrNavEvalEnvCfg()
    cfg.safety_filter = args.phi
    cfg.phi_cloud_mode = args.phi_cloud
    cfg.success_dwell_steps = args.dwell_k
    if args.train_pool:
        cfg.path_pool_seed = 0
    cfg.scene.num_envs = n_envs
    cfg.seed = args.seed
    if path_len:
        cfg.path_len_min, cfg.path_len_max = (float(x) for x in path_len.split(':'))
    return AmrNavEnv(cfg), cfg


def common_meta(cfg, env, pol):
    """eval.json 공통 필드."""
    return {
        'policy': args.policy, 'tag': args.tag, 'eval_seed': args.seed,
        'pool': 'train(seed0)' if args.train_pool else 'heldout(seed1)',
        'safety_filter': args.phi, 'phi_cloud_mode': args.phi_cloud if args.phi == 'canonical'
        else None, 'dwell_k': args.dwell_k, 'nav2_radius': args.nav2_radius,
        'diag_steps': args.diag_steps, 'pool_paths': env.pool_size, 'dr': False,
        'deterministic': 'tanh(mu)', 'reward_version': cfg.reward_version,
        'contract_version': C.CONTRACT_VERSION,
        'loadavg': open('/proc/loadavg').read().split()[:3],
        'torch_max_alloc_MiB': round(torch.cuda.max_memory_allocated() / 2**20, 1),
        'policy_log_std': [float(x) for x in pol.log_std_summary()],
        'policy_kind': pol.kind,
    }


def main_single():
    """기존 방식: 길이 범위 안 무작위 경로, env 마다 첫 에피소드."""
    out = args.out or os.path.join(os.path.dirname(args.policy), f'eval_{args.tag}')
    os.makedirs(out, exist_ok=True)
    env, cfg = make_env(args.num_envs, args.path_len)
    pol = PE.load_policy(args.policy).to(env.device).eval()
    keep_obs, keep_out = [], []
    ep, first_path, unfinished, steps, limit, wall = run_first_episodes(env, pol, keep_obs,
                                                                        keep_out)
    gm = ep.pop('got')
    ep = {k: v[gm] for k, v in ep.items()}
    res = summarize(ep, env.num_envs, unfinished)
    res.update(common_meta(cfg, env, pol))
    res.update({'path_len_range': args.path_len, 'steps': steps, 'step_limit': limit,
                'wall_s': round(wall, 1),
                'path_ids_sha': hashlib.sha256(
                    np.ascontiguousarray(first_path).tobytes()).hexdigest()[:16]})
    np.savez_compressed(os.path.join(out, 'episodes.npz'), **ep)
    np.savez_compressed(os.path.join(out, 'obs_batch.npz'), obs=np.concatenate(keep_obs),
                        out=np.concatenate(keep_out))
    with open(os.path.join(out, 'eval.json'), 'w') as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    print('EVAL_UNBIASED ' + json.dumps(res, ensure_ascii=False), flush=True)
    return 0 if unfinished == 0 else 1


def main_sets():
    """B 단계: 평가 집합 × 교란 조건의 고정 행."""
    out = args.out or os.path.join(os.path.dirname(args.policy), f'eval_{args.tag}')
    os.makedirs(out, exist_ok=True)
    doc = ES.load_sets(args.eval_set)
    names = [s for s in args.sets.split(',') if s] or \
        [s for s in doc['sets'] if not s.startswith('perturb')]
    for s in names:
        if s not in doc['sets']:
            raise ValueError(f'집합 {s!r} 없음 (있는 것: {sorted(doc["sets"])})')
    if args.perturb:
        suite = PT.load_suite(args.perturb)
        conds = suite['conditions']
        want = [c for c in args.conditions.split(',') if c]
        if want:
            byname = {c['name']: c for c in conds}
            missing = [w for w in want if w not in byname]
            if missing:
                raise ValueError(f'조건 없음 {missing}')
            conds = [byname[w] for w in want]
    else:
        suite, conds = None, [{'name': 'nominal', 'group': 'nominal', 'params': {}}]
    rows = [(si, ci, int(pid)) for ci in range(len(conds)) for si, s in enumerate(names)
            for pid in doc['sets'][s]]
    total = len(rows)
    n_envs = min(args.max_envs, total)
    env, cfg = make_env(n_envs, '')
    if not np.array_equal(env.pool_kept, np.arange(env.pool_size)):
        raise RuntimeError('집합 평가는 경로 풀 전체(필터 없음)여야 한다')
    pool = load_pool(seed=cfg.path_pool_seed, out_dir=cfg.assets_dir, log=None)
    ES.check_pool(doc, pool['pts'], pool['offs'], pool['length'])
    pol = PE.load_policy(args.policy).to(env.device).eval()
    dev = env.device
    keep_obs, keep_out = [], []
    parts, rounds = [], []
    t_all = time.time()
    unfinished_total = 0
    for r0 in range(0, total, n_envs):
        chunk = rows[r0:r0 + n_envs]
        pad = n_envs - len(chunk)
        chunk_p = chunk + [chunk[-1]] * pad
        cidx = np.array([c for _, c, _ in chunk_p])
        env.forced_path_ids = torch.tensor([p for _, _, p in chunk_p], dtype=torch.long,
                                           device=dev)
        env.set_perturbation(PT.override_arrays(conds, cidx) if suite else None)
        ep, first_path, unf, steps, limit, wall = run_first_episodes(
            env, pol, keep_obs if not rounds else None, keep_out if not rounds else None)
        if not np.array_equal(first_path, np.array([p for _, _, p in chunk_p])):
            raise RuntimeError('리셋 경로가 지정한 집합 경로와 다르다')
        real = np.arange(n_envs) < len(chunk)
        unf_real = int((~ep['got'] & real).sum())
        unfinished_total += unf_real
        ep['set_index'] = np.array([s for s, _, _ in chunk_p])
        ep['cond_index'] = cidx
        if suite:
            snap = env.dr_snapshot()
        else:
            snap = env.dr_snapshot()
        parts.append({k: v[real] for k, v in ep.items()})
        rounds.append({'rows': len(chunk), 'pad': pad, 'steps': steps, 'step_limit': limit,
                       'wall_s': round(wall, 1), 'unfinished': unf_real, 'dr_snapshot': snap,
                       'loadavg': open('/proc/loadavg').read().split()[:3]})
        print('ROUND ' + json.dumps(rounds[-1], ensure_ascii=False), flush=True)
    ep = {k: np.concatenate([p[k] for p in parts]) for k in parts[0]}
    got = ep.pop('got')
    groups_all, _ = ES.goal_groups(pool['pts'], pool['offs'], ep['path_id_orig'])
    res = {'mode': 'eval_sets', 'eval_set_file': args.eval_set, 'pool_digest': doc['pool_digest'],
           'sets': names, 'perturb_file': args.perturb or None,
           'conditions': [c['name'] for c in conds], 'rows': total, 'max_envs': n_envs,
           'rounds': rounds, 'wall_s': round(time.time() - t_all, 1),
           'unfinished': unfinished_total, 'by': {}}
    res.update(common_meta(cfg, env, pol))
    for ci, c in enumerate(conds):
        for si, s in enumerate(names):
            m = (ep['set_index'] == si) & (ep['cond_index'] == ci) & got
            sub = {k: v[m] for k, v in ep.items()}
            key = s if not suite else f'{s}|{c["name"]}'
            if not m.any():
                res['by'][key] = {'episodes': 0}
                continue
            blk = summarize(sub, int(((ep['set_index'] == si) & (ep['cond_index'] == ci)).sum()),
                            int(((ep['set_index'] == si) & (ep['cond_index'] == ci) & ~got).sum()))
            blk['set'] = s
            blk['condition'] = c['name']
            blk['condition_group'] = c.get('group')
            blk['condition_params'] = PT.resolved_params(c)
            blk['intervals'] = interval_block(sub, groups_all[m])
            blk['path_ids_sha'] = hashlib.sha256(np.ascontiguousarray(
                sub['path_id_orig']).tobytes()).hexdigest()[:16]
            res['by'][key] = blk
    ep['got'] = got
    ep['goal_group'] = groups_all
    np.savez_compressed(os.path.join(out, 'episodes.npz'), **ep)
    if keep_obs:
        np.savez_compressed(os.path.join(out, 'obs_batch.npz'), obs=np.concatenate(keep_obs),
                            out=np.concatenate(keep_out))
    with open(os.path.join(out, 'eval.json'), 'w') as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    brief = {k: (v.get('layered', {}).get('strict_success'), v.get('episodes'))
             for k, v in res['by'].items()}
    print('EVAL_SETS ' + json.dumps({'out': out, 'unfinished': unfinished_total,
                                     'strict_success': brief}, ensure_ascii=False), flush=True)
    return 0 if unfinished_total == 0 else 1


try:
    rc = main_sets() if args.eval_set else main_single()
except Exception:
    import traceback
    traceback.print_exc()
    rc = 2
os._exit(rc)
