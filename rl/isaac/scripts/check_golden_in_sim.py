"""
Isaac 안에서의 계약 일치 시험 — golden 상태를 실제 env 버퍼(CUDA)에 주입하고 env 의 관측·보상 경로로 계산.

    rl/isaac/run_isaac.sh golden rl/isaac/scripts/check_golden_in_sim.py

tests/test_contract_golden.py 는 contract_torch 를 직접 부른다. 이 스크립트는 그 위의 env 배선
(경로 패딩·누적 호길이, 버퍼 이름, 전이 전 거리 캐시 d_prog = max(d_arc, 목표 직선거리), 보상 판 선택,
안전 개입 쌍 수, 트랙 버퍼)이 같은 값을 내는지 본다. 계약 v1.2.0 (golden 32 = decision 17 + step 12 + 기타 3).
  decision 17: inject_state → build_obs → golden obs (허용오차 tolerance.obs)
  step 12    : pre 주입 → build_obs (d(s) 캐시) → 자세·v_meas 를 post 로 → step_outcome → golden
               (보상 항 집합 = golden 의 항 집합: progress·dev 포함)
결과: $AMR_ISAAC_RUNS/golden/in_sim.json, rc 0 = 전부 일치.
"""

import argparse
import glob
import json
import os
import sys

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument('--out', default=os.path.join(os.environ.get(
    'AMR_ISAAC_RUNS', '/home/CAPYI_AI/runs/r7-isaac12'), 'golden', 'in_sim.json'))
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

sys.path.insert(0, '/workspace/amr/rl/isaac')
import torch  # noqa: E402

from amr_isaac import contract_torch as CT  # noqa: E402
from amr_isaac.env import AmrNavEnv  # noqa: E402
from amr_isaac.env_cfg import AmrNavEnvCfg  # noqa: E402
from amr_rl_contract import constants as C  # noqa: E402
from amr_rl_contract import reward as RW  # noqa: E402
from amr_rl_contract.observation import obs_slices  # noqa: E402

GOLDEN = '/workspace/amr/rl/contract/golden'


def main():
    """주입 시험."""
    files = sorted(glob.glob(os.path.join(GOLDEN, '*.json')))
    gold = [json.load(open(f, encoding='utf-8')) for f in files]
    dec = [g for g in gold if g['kind'] == 'decision']
    stp = [g for g in gold if g['kind'] == 'step']
    assert C.CONTRACT_VERSION == '1.2.0', C.CONTRACT_VERSION
    assert (len(gold), len(dec), len(stp)) == (32, 17, 12), (len(gold), len(dec), len(stp))
    assert all(g['contract_version'] == '1.2.0' for g in gold)
    cfg = AmrNavEnvCfg()
    cfg.scene.num_envs = max(len(dec), len(stp)) + 3
    cfg.dr.enable = False
    env = AmrNavEnv(cfg)
    env.reset()
    dev = env.device
    assert env.reward_spec is RW.SPEC, env.reward_spec
    res = {'decision': {}, 'step': {}, 'device': str(dev), 'contract_version': C.CONTRACT_VERSION,
           'reward_spec': str(env.reward_spec)}
    fails = []

    ids = torch.arange(len(dec), device=dev)
    st = CT.states_from_dicts([g['state'] for g in dec], device=dev)
    env.inject_state(st, ids)
    obs = env.build_obs()
    for i, g in enumerate(dec):
        exp = torch.tensor(g['expected']['obs'], dtype=torch.float64, device=dev)
        err = (obs[i].double() - exp).abs()
        bad = {k: float(err[sl].max()) for k, sl in obs_slices().items()
               if float(err[sl].max()) > g['tolerance']['obs']}
        res['decision'][g['name']] = {'max_err': float(err.max()), 'bad_blocks': bad}
        if bad:
            fails.append(g['name'])

    ids = torch.arange(len(stp), device=dev)
    pre = CT.states_from_dicts([g['pre'] for g in stp], device=dev)
    post = CT.states_from_dicts([g['post'] for g in stp], device=dev)
    env.inject_state(pre, ids)
    env.build_obs()                                   # d(s) 캐시 (d_prog) 를 pre 자세로
    env.pose[ids] = post.pose
    env.v_meas[ids] = post.v_meas
    env._a_exec[ids] = torch.tensor([g['action_exec'] for g in stp], dtype=torch.float64,
                                    device=dev)
    env._collided[ids] = torch.tensor([bool(g['collision']) for g in stp], device=dev)
    env._pairs[ids] = 0.0
    for i, g in enumerate(stp):
        pr = g['safety_pairs']
        env._pairs_n[i] = len(pr)
        for k, (a, b) in enumerate(pr):
            env._pairs[i, k] = torch.tensor([a[0], a[1], b[0], b[1]], dtype=torch.float64,
                                            device=dev)
    out = env.step_outcome()
    for i, g in enumerate(stp):
        e, tol = g['expected'], g['tolerance']
        errs = {'reward': abs(float(out.reward[i]) - e['reward'])}
        for k, v in e['terms'].items():
            errs[f'term_{k}'] = (abs(float(out.terms[k][i]) - v) if k in out.terms
                                 else float('inf'))
        errs['cost'] = float((out.cost[i].cpu() - torch.tensor(e['cost'],
                                                               dtype=torch.float64)).abs().max())
        terms_ok = set(out.terms) == set(e['terms'])
        flags_ok = (terms_ok and bool(out.terminated[i]) == e['terminated'] and
                    bool(out.success[i]) == e['success'] and
                    bool(out.in_dev[i]) == e['dev']['in_dev'] and
                    abs(float(out.dev_t[i]) - e['dev']['t_since_peak']) <= 1e-9)
        ok = (errs['reward'] <= tol['reward'] and errs['cost'] <= tol['cost'] and flags_ok and
              all(v <= tol['reward_terms'] for k, v in errs.items() if k.startswith('term_')))
        res['step'][g['name']] = {'ok': ok, 'flags_ok': flags_ok, 'terms_ok': terms_ok,
                                  'max_err': max(errs.values())}
        if not ok:
            fails.append(g['name'])
    res['fails'] = fails
    res['n_decision'] = len(dec)
    res['n_step'] = len(stp)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, 'w') as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    print('GOLDEN_IN_SIM ' + json.dumps({'fails': fails, 'n_decision': len(dec),
                                         'n_step': len(stp)}, ensure_ascii=False), flush=True)
    return 0 if not fails else 1


try:
    rc = main()
except Exception:
    import traceback
    traceback.print_exc()
    rc = 2
os._exit(rc)
