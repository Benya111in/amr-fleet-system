#!/usr/bin/env python3
"""
ONNX 왕복 확인 — torch 정책과 onnxruntime(CPU) 출력이 같은가 (onnx_io.md).

입력 표본: golden decision 17 사례의 obs(·mask) + env 에서 모은 관측. 배치 1 과 배치 N 둘 다.
메타데이터 필수 키(contract_version, arm, obs_dim, dt, normalization)도 확인한다.

    python3 rl/sim2d/tools/onnx_check.py --ckpt <model.pt> --onnx <policy.onnx>
"""

import argparse
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402
import torch  # noqa: E402

from amr_sim2d._contract import C, GOLDEN_DIR  # noqa: E402

REQUIRED = ('contract_version', 'arm', 'obs_dim', 'dt', 'normalization')


def golden_inputs():
    """Golden decision 사례의 (obs, mask)."""
    obs, mask = [], []
    for f in sorted(glob.glob(os.path.join(GOLDEN_DIR, 'd*.json'))):
        with open(f, encoding='utf-8') as fh:
            d = json.load(fh)
        obs.append(d['expected']['obs'])
        mask.append([1.0 if m else 0.0 for m in d['expected']['mask']])
    return np.array(obs, np.float32), np.array(mask, np.float32)


def env_inputs(env, steps=8):
    """Env 를 무작위 행동으로 몇 스텝 굴려 관측·마스크를 모은다."""
    obs = env.reset()
    o, m = [obs.cpu()], [env.mask.float().cpu()]
    g = torch.Generator(device=env.dev)
    g.manual_seed(7)
    for _ in range(steps):
        if env.cfg.arm == 'B':
            act = torch.rand(env.N, 2, device=env.dev, generator=g) * 2 - 1
        else:
            lg = torch.rand(env.N, 100, device=env.dev, generator=g)
            act = torch.where(env.mask, lg, torch.full_like(lg, -1.0)).argmax(1)
        obs, *_ = env.step(act)
        o.append(obs.cpu())
        m.append(env.mask.float().cpu())
    return torch.cat(o).numpy(), torch.cat(m).numpy()


def roundtrip(model, path, env=None, tol=1e-5):
    """Torch vs ORT 최대 오차와 메타데이터 검사 결과 dict."""
    import onnx
    import onnxruntime as ort
    model = model.eval().cpu()
    sess = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
    meta = {p.key: p.value for p in onnx.load(path).metadata_props}
    go, gm = golden_inputs()
    sets = {'golden17': (go, gm)}
    if env is not None:
        sets['env'] = env_inputs(env)
    opset = onnx.load(path).opset_import[0].version
    out = {'onnx': path, 'ort_version': ort.__version__, 'opset': opset,
           'inputs': [i.name for i in sess.get_inputs()],
           'outputs': [o.name for o in sess.get_outputs()],
           'meta_missing': [k for k in REQUIRED if k not in meta],
           'contract_version_ok': meta.get('contract_version') == C.CONTRACT_VERSION}
    worst = 0.0
    for name, (o, m) in sets.items():
        res = {}
        for bs in (1, len(o)):
            ob = o[:bs]
            mb = m[:bs]
            if model.arm == 'B':
                with torch.no_grad():
                    t = [x.numpy() for x in model(torch.from_numpy(ob))]
                r = sess.run(['action', 'mu', 'log_std'], {'obs': ob})
                errs = {k: float(np.abs(a - b).max()) for k, a, b in
                        zip(('action', 'mu', 'log_std'), t, r)}
            else:
                with torch.no_grad():
                    lg, act = model(torch.from_numpy(ob), torch.from_numpy(mb))
                r = sess.run(['logits', 'action'], {'obs': ob, 'mask': mb})
                valid = mb > 0.5
                errs = {'logits_valid': float(np.abs(lg.numpy() - r[0])[valid].max()),
                        'action_mismatch': int((act.numpy() != r[1]).sum()),
                        'action_in_mask': bool(mb[np.arange(bs), r[1]].all())}
            res[f'batch{bs}'] = errs
            worst = max(worst, max(v for k, v in errs.items() if isinstance(v, float)))
        out[name] = res
    out['max_abs_err'] = worst
    out['pass'] = bool(worst <= tol and not out['meta_missing'] and out['contract_version_ok'] and
                       all(v.get('action_mismatch', 0) == 0 for s in sets for v in
                           out[s].values()))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ckpt', required=True)
    ap.add_argument('--onnx', required=True)
    ap.add_argument('--export', action='store_true', help='ckpt 에서 onnx 를 새로 내보낸다')
    ap.add_argument('--git-commit', default='unknown')
    a = ap.parse_args()
    from amr_sim2d.ppo import ActorCriticB, ActorCriticC, export_onnx
    ck = torch.load(a.ckpt, map_location='cpu')
    model = ActorCriticB() if ck['arm'] == 'B' else ActorCriticC()
    model.load_state_dict(ck['model'])
    if a.export:
        export_onnx(model, a.onnx, {'algo': 'ppo', 'git_commit': a.git_commit,
                                    'train_steps': ck.get('steps', 0)})
    print(json.dumps(roundtrip(model, a.onnx), indent=1))


if __name__ == '__main__':
    main()
