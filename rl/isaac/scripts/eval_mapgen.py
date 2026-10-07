"""
지도 변형 평가 (b1-mapgen) — env 마다 첫 에피소드 하나, 결정론 tanh(μ), 평가 seed 고정 (eval_unbiased.py 와 같은 추정량).

    rl/isaac/run_isaac.sh mg rl/isaac/scripts/eval_mapgen.py --policy <pt> --phi canonical \
        --eval_set /workspace/amr/rl/isaac/eval_sets/b1_eval_sets_v1.json \
        --bundle <map_bundle_v1.npz> --bundle_sha <sha256 앞부분> --max_envs 5120 --out <dir>

행 (라운드 = --max_envs 개씩, env i = 그 라운드 행 i):
  1. 명목 행 — --nominal_sets (기본 main_3_15,stress_3_15,main_3_30,stress_3_30) 의 모든 경로, 명목 지도(타일 0).
     eval_unbiased.py --eval_set (교란 없음) 과 **같은 행·같은 순서**다. --max_envs 가 같으면 첫 라운드가 그 평가와 같은
     조합이 되어 에피소드 배열이 비트 단위로 같아야 한다 (기본값 동작 대조 — 노트 항목의 P0).
  2. 변형 행 — bundle 의 유효 행 (조건 → 집합 → 집합 안 순번), 그 행의 타일(변형 지도)과 재계획 경로.
결과 키: 명목 = 집합 이름 (eval_unbiased 와 같음), 변형 = '<집합>|<조건>' (유효 행 전부) 와 '<집합>|<조건>|variant_main'
(변형 지도에서도 끝 직진 여유 ≥ 0.40 m 인 행 — 사전 등록 1 차 모집단).
탐침 선택지: --conditions none (명목 행만), --nominal_tile far (명목 행을 명목 지도의 먼 사본 타일에서 — 부동소수 이동 탐침),
--zero_policy (행동 0 — 파이프라인 시험, 정책 결과 없음), --rows_per_cond N (조건마다 앞 N 행만 — 스모크).
summarize·interval_block·run_first_episodes 는 eval_unbiased.py 의 것을 그대로 옮겼다 (그 파일을 바꾸지 않으려고).
부하 기록은 os.getloadavg() (eval_unbiased 의 기록과 같은 값, 소수 둘째 자리 문자열).
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
ap.add_argument('--seed', type=int, default=4242)
ap.add_argument('--tag', default='mapgen')
ap.add_argument('--out', default=None)
ap.add_argument('--phi', default='canonical', choices=('lite', 'canonical'), help='safety φ')
ap.add_argument('--phi_cloud', default='empty', choices=('empty', 'none'))
ap.add_argument('--dwell_k', type=int, default=1)
ap.add_argument('--diag_steps', type=int, default=20)
ap.add_argument('--nav2_radius', type=float, default=0.25)
ap.add_argument('--eval_set', required=True)
ap.add_argument('--nominal_sets', default='main_3_15,stress_3_15,main_3_30,stress_3_30')
ap.add_argument('--bundle', required=True)
ap.add_argument('--bundle_sha', default='', help='bundle sha256 앞부분 (다르면 rc 2)')
ap.add_argument('--conditions', default='', help="쉼표 목록, 빈 값 = 전부, 'none' = 변형 행 없음")
ap.add_argument('--nominal_tile', default='0', choices=('0', 'far'))
ap.add_argument('--rows_per_cond', type=int, default=0)
ap.add_argument('--zero_policy', action='store_true')
ap.add_argument('--max_envs', type=int, default=5120)
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
from amr_isaac import map_bundle as MB  # noqa: E402
from amr_isaac import mapgen as MG  # noqa: E402
from amr_isaac import policy_export as PE  # noqa: E402
from amr_isaac.assets import load_pool  # noqa: E402
from amr_isaac.env_cfg import AmrNavEvalEnvCfg  # noqa: E402
from amr_isaac.env_mapgen import AmrNavMapEnv  # noqa: E402
from amr_isaac.env_math import OUTCOME_CODES  # noqa: E402
from amr_rl_contract import constants as C  # noqa: E402
from amr_rl_eval import binomial as BN  # noqa: E402


def loadavg():
    """1·5·15 분 부하 (문자열 셋)."""
    return [f'{x:.2f}' for x in os.getloadavg()]


# ------------------------------------------------------------ eval_unbiased.py 에서 그대로 옮긴 부분
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
    """env.reset(seed) 뒤 모든 env 의 첫 에피소드를 끝까지 (eval_unbiased.py 와 같은 함수)."""
    with torch.inference_mode():
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


# ------------------------------------------------------------ 이 스크립트의 부분
class ZeroPolicy(torch.nn.Module):
    """행동 0 (파이프라인 시험용 — 로봇이 움직이지 않아 정체로 끝난다)."""

    kind = 'zero'

    def forward(self, o):
        """행동·평균·log σ 를 모두 0 으로 낸다."""
        z = torch.zeros(o.shape[0], 2, device=o.device, dtype=o.dtype)
        return z, z, z

    def log_std_summary(self):
        """정책 기록용."""
        return [0.0, 0.0]


def build_rows(doc, bundle, pool_size):
    """
    행 목록과 덧붙일 경로 → (rows, extra, set_names, cond_names).

    rows[i] = dict(set, cond (−1 = 명목), pid (env 풀 번호), pid_orig, tile, brow (bundle 행, −1),
    j (집합 안 순번), main_variant).
    extra[k] = (타일, bundle 행 또는 None, 원래 풀 번호 또는 None) — env 풀 번호 pool_size + k 의 경로.
    """
    nominal_sets = [s for s in args.nominal_sets.split(',') if s]
    for s in nominal_sets:
        if s not in doc['sets']:
            raise ValueError(f'집합 {s!r} 없음')
    bsets = bundle['set_names'].tolist()
    set_names = nominal_sets + [s for s in bsets if s not in nominal_sets]
    cond_names = bundle['cond_names'].tolist()
    if args.conditions == 'none':
        want = []
    elif args.conditions:
        want = args.conditions.split(',')
        bad = [c for c in want if c not in cond_names]
        if bad:
            raise ValueError(f'조건 없음 {bad}')
    else:
        want = cond_names
    rows, extra = [], []
    far = int(bundle['far_tile'])
    for si, s in enumerate(nominal_sets):
        for j, pid in enumerate(doc['sets'][s]):
            pid = int(pid)
            if args.nominal_tile == 'far':
                extra.append((far, None, pid))
                rows.append({'set': si, 'cond': -1, 'pid': pool_size + len(extra) - 1,
                             'pid_orig': pid, 'tile': far, 'brow': -1, 'j': j,
                             'main_variant': s.startswith('main')})
            else:
                rows.append({'set': si, 'cond': -1, 'pid': pid, 'pid_orig': pid, 'tile': 0,
                             'brow': -1, 'j': j, 'main_variant': s.startswith('main')})
    for cn in want:
        ci = cond_names.index(cn)
        idx = np.flatnonzero((bundle['row_cond'] == ci) & (bundle['row_code'] == MG.VALID))
        if args.rows_per_cond:
            idx = idx[:args.rows_per_cond]
        for r in idx:
            t = int(bundle['row_tile'][r])
            extra.append((t, int(r), None))
            rows.append({'set': set_names.index(bsets[int(bundle['row_set'][r])]), 'cond': ci,
                         'pid': pool_size + len(extra) - 1, 'pid_orig': int(bundle['row_pid'][r]),
                         'tile': t, 'brow': int(r), 'j': int(bundle['row_j'][r]),
                         'main_variant': bool(bundle['row_main_variant'][r])})
    return rows, extra, set_names, cond_names


ROW_KEYS = {'set': 'set_index', 'cond': 'cond_index', 'tile': 'tile', 'brow': 'bundle_row',
            'pid_orig': 'pid_orig', 'j': 'row_j', 'main_variant': 'main_variant'}


def main():
    """평가."""
    out = args.out or os.path.join(os.path.dirname(args.policy), f'eval_{args.tag}')
    os.makedirs(out, exist_ok=True)
    bundle = MB.load_bundle(args.bundle, expect_sha=args.bundle_sha or None)
    bsha = MB.file_sha256(args.bundle)
    doc = ES.load_sets(args.eval_set)
    if str(bundle['pool_digest']) != doc['pool_digest']:
        raise ValueError('bundle 과 평가 집합의 경로 풀이 다르다')
    cfg = AmrNavEvalEnvCfg()
    cfg.safety_filter = args.phi
    cfg.phi_cloud_mode = args.phi_cloud
    cfg.success_dwell_steps = args.dwell_k
    cfg.seed = args.seed
    pool = load_pool(seed=cfg.path_pool_seed, out_dir=cfg.assets_dir, log=None)
    ES.check_pool(doc, pool['pts'], pool['offs'], pool['length'])
    base_size = int(pool['length'].shape[0])
    rows, extra, set_names, cond_names = build_rows(doc, bundle, base_size)
    extra_paths = []
    for t, brow, pid in extra:
        if brow is not None:
            extra_paths.append((t, MB.row_path(bundle, brow)))
        else:
            o = pool['offs']
            extra_paths.append((t, pool['pts'][o[pid]:o[pid + 1]]))
    total = len(rows)
    n_envs = min(args.max_envs, total)
    cfg.scene.num_envs = n_envs
    env = AmrNavMapEnv(cfg, bundle, extra_paths)
    if env.n_base_paths != base_size:
        raise RuntimeError('env 풀 크기가 held-out 풀과 다르다')
    if not np.array_equal(env.pool_kept, np.arange(env.pool_size)):
        raise RuntimeError('집합 평가는 경로 풀 전체(필터 없음)여야 한다')
    pol = (ZeroPolicy() if args.zero_policy else PE.load_policy(args.policy)).to(env.device).eval()
    dev = env.device
    keep_obs, keep_out = [], []
    parts, rounds = [], []
    t_all = time.time()
    unfinished_total = 0
    for r0 in range(0, total, n_envs):
        chunk = rows[r0:r0 + n_envs]
        pad = n_envs - len(chunk)
        chunk_p = chunk + [chunk[-1]] * pad
        env.forced_path_ids = torch.tensor([r['pid'] for r in chunk_p], dtype=torch.long,
                                           device=dev)
        env.set_env_tiles([r['tile'] for r in chunk_p])
        env.set_perturbation(None)
        calls0 = env.collide_calls
        ep, first_path, unf, steps, limit, wall = run_first_episodes(
            env, pol, keep_obs if not rounds else None, keep_out if not rounds else None)
        if not np.array_equal(first_path, np.array([r['pid'] for r in chunk_p])):
            raise RuntimeError('리셋 경로가 지정한 행 경로와 다르다')
        real = np.arange(n_envs) < len(chunk)
        unf_real = int((~ep['got'] & real).sum())
        unfinished_total += unf_real
        for k, nm in ROW_KEYS.items():
            ep[nm] = np.array([r[k] for r in chunk_p])
        parts.append({k: v[real] for k, v in ep.items()})
        rounds.append({'rows': len(chunk), 'pad': pad, 'steps': steps, 'step_limit': limit,
                       'wall_s': round(wall, 1), 'unfinished': unf_real,
                       'collide_calls': env.collide_calls - calls0,
                       'tiles': sorted({int(r['tile']) for r in chunk}),
                       'dr_snapshot': env.dr_snapshot(), 'loadavg': loadavg()})
        print('ROUND ' + json.dumps({k: v for k, v in rounds[-1].items() if k != 'dr_snapshot'},
                                    ensure_ascii=False), flush=True)
    ep = {k: np.concatenate([p[k] for p in parts]) for k in parts[0]}
    got = ep.pop('got')
    groups_all, _ = ES.goal_groups(pool['pts'], pool['offs'], ep['pid_orig'])
    res = {'mode': 'eval_mapgen', 'eval_set_file': args.eval_set,
           'pool_digest': doc['pool_digest'],
           'bundle': args.bundle, 'bundle_sha256': bsha, 'suite_sha': str(bundle['suite_sha']),
           'sets': set_names, 'nominal_sets': [s for s in args.nominal_sets.split(',') if s],
           'conditions': cond_names, 'conditions_run': sorted({cond_names[r['cond']]
                                                               for r in rows if r['cond'] >= 0}),
           'nominal_tile': args.nominal_tile, 'rows_per_cond': args.rows_per_cond,
           'zero_policy': args.zero_policy, 'rows': total, 'max_envs': n_envs,
           'mesh_triangles_per_tile': env.mesh_counts, 'rounds': rounds,
           'wall_s': round(time.time() - t_all, 1), 'unfinished': unfinished_total, 'by': {}}
    res.update({
        'policy': args.policy, 'tag': args.tag, 'eval_seed': args.seed, 'pool': 'heldout(seed1)',
        'safety_filter': args.phi, 'phi_cloud_mode': args.phi_cloud if args.phi == 'canonical'
        else None, 'dwell_k': args.dwell_k, 'nav2_radius': args.nav2_radius,
        'diag_steps': args.diag_steps, 'pool_paths': env.pool_size,
        'pool_paths_base': env.n_base_paths, 'dr': False, 'deterministic': 'tanh(mu)',
        'reward_version': cfg.reward_version, 'contract_version': C.CONTRACT_VERSION,
        'loadavg': loadavg(),
        'torch_max_alloc_MiB': round(torch.cuda.max_memory_allocated() / 2**20, 1),
        'policy_log_std': [float(x) for x in pol.log_std_summary()], 'policy_kind': pol.kind})
    blocks = [(-1, si) for si in range(len(set_names))] + \
        [(ci, si) for ci in range(len(cond_names)) for si in range(len(set_names))]
    for ci, si in blocks:
        base_m = (ep['set_index'] == si) & (ep['cond_index'] == ci)
        if not base_m.any():
            continue
        subsets = [('', base_m)]
        if ci >= 0 and set_names[si].startswith('main'):
            subsets.append(('|variant_main', base_m & ep['main_variant']))
        for suffix, m0 in subsets:
            m = m0 & got
            key = set_names[si] if ci < 0 else f'{set_names[si]}|{cond_names[ci]}{suffix}'
            if not m.any():
                res['by'][key] = {'episodes': 0}
                continue
            sub = {k: v[m] for k, v in ep.items()}
            blk = summarize(sub, int(m0.sum()), int((m0 & ~got).sum()))
            blk['set'] = set_names[si]
            blk['condition'] = 'nominal' if ci < 0 else cond_names[ci]
            blk['intervals'] = interval_block(sub, groups_all[m])
            blk['path_ids_sha'] = hashlib.sha256(np.ascontiguousarray(
                sub['pid_orig']).tobytes()).hexdigest()[:16]
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
    print('EVAL_MAPGEN ' + json.dumps({'out': out, 'unfinished': unfinished_total,
                                       'strict_success': brief}, ensure_ascii=False), flush=True)
    return 0 if unfinished_total == 0 else 1


try:
    rc = main()
except Exception:
    import traceback
    traceback.print_exc()
    rc = 2
os._exit(rc)
