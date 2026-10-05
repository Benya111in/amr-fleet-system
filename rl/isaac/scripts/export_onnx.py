#!/usr/bin/env python3
"""
학습 정책 → ONNX (onnx_io.md 비교군 B) 내보내기와 onnxruntime 왕복 일치 시험. amr-ai/rl:0.3 에서 돈다.

    R=/home/CAPYI_AI/runs/r1-isaac
    docker run --rm -v <wt>:/ws -v $R:$R -w /ws amr-ai/rl:0.3 \
        python3 rl/isaac/scripts/export_onnx.py --run $R/train/<run>

검사 (하나라도 어긋나면 rc 1):
  - 입력 obs [batch, 291] float32, 출력 action·mu·log_std [batch, 2] float32, opset 17, 필수 메타데이터
  - ORT(CPU) 출력 = torch PolicyB 출력 (1e-5): golden decision 관측 17 개, 학습 env 의 실제 관측
    (eval_policy.py 가 남긴 obs_batch.npz, 있으면), 표준정규 무작위 관측 256 개, 배치 1·N 모두
  - Isaac 안(torch 2.7, CUDA)에서 계산한 PolicyB 출력과 ORT 출력 차이 (obs_batch.npz 의 out) — 보고만
모델 파일은 커밋하지 않는다 (.gitignore *.onnx). 결과 <run>/onnx_check.json.
"""

import argparse
import glob
import json
import os
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), 'contract'))
from amr_isaac import policy_export as PE  # noqa: E402
from amr_rl_contract import constants as C  # noqa: E402

GOLDEN = os.path.join(os.path.dirname(os.path.dirname(HERE)), 'contract', 'golden')
TOL = 1e-5


def main():
    """내보내기 + 왕복 시험."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', required=True)
    ap.add_argument('--policy', default='policy_final.pt')
    ap.add_argument('--obs_batch', default=None)
    ap.add_argument('--algo', default='PPO (rsl_rl 3.1.2)')
    a = ap.parse_args()
    import onnx
    import onnxruntime as ort
    ck = torch.load(os.path.join(a.run, a.policy), map_location='cpu')
    pol = PE.PolicyB.from_rsl_state_dict(ck['state_dict'], ck.get('activation', 'elu')).eval()
    head = ''
    hp = os.path.join(a.run, 'HEAD.txt')
    if os.path.exists(hp):
        head = open(hp).read().split()[0]
    summ = {}
    sp = os.path.join(a.run, 'summary.json')
    if os.path.exists(sp):
        summ = json.load(open(sp))
    meta = {'contract_version': C.CONTRACT_VERSION, 'arm': 'B', 'obs_dim': '291', 'dt': '0.1',
            'normalization': 'inside', 'algo': a.algo,
            'seed': str(summ.get('seed', '')), 'git_commit': head,
            'train_steps': str(summ.get('env_steps', '')), 'simulator': 'isaac',
            'normalization_var_floor': str(PE.VAR_FLOOR)}
    stem = os.path.splitext(a.policy)[0]
    path = os.path.join(a.run, f'{stem}.onnx')
    PE.export_onnx(pol, path, meta)
    m = onnx.load(path)
    opset = [o.version for o in m.opset_import if o.domain in ('', 'ai.onnx')]
    res = {'onnx': path, 'opset': opset,
           'metadata': {p.key: p.value for p in m.metadata_props}, 'checks': {}}
    fails = []

    def chk(name, ok, val=None):
        res['checks'][name] = {'ok': bool(ok), 'value': val}
        if not ok:
            fails.append(name)

    sess = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
    ins = [(i.name, i.shape, i.type) for i in sess.get_inputs()]
    outs = [(o.name, o.shape, o.type) for o in sess.get_outputs()]
    chk('io_names', [i[0] for i in ins] == ['obs'] and
        [o[0] for o in outs] == ['action', 'mu', 'log_std'], {'in': ins, 'out': outs})
    chk('io_dtype', all(t == 'tensor(float)' for _n, _s, t in ins + outs))
    chk('obs_dim', ins[0][1][1] == 291, ins[0][1])
    chk('opset17', res['opset'] == [PE.OPSET], res['opset'])
    for k in ('contract_version', 'arm', 'obs_dim', 'dt', 'normalization'):
        chk(f'meta_{k}', k in res['metadata'])

    sets = {}
    gobs = []
    for f in sorted(glob.glob(os.path.join(GOLDEN, 'd*.json'))):
        gobs.append(json.load(open(f))['expected']['obs'])
    sets['golden_decision'] = np.asarray(gobs, dtype=np.float32)
    sets['random_normal'] = np.random.default_rng(0).standard_normal((256, 291)).astype(np.float32)
    obp = a.obs_batch or os.path.join(a.run, 'eval_heldout', 'obs_batch.npz')
    isaac_out = None
    if os.path.exists(obp):
        z = np.load(obp)
        sets['env_obs'] = z['obs'].astype(np.float32)
        isaac_out = z['out']
    for name, x in sets.items():
        with torch.no_grad():
            ref = [t.numpy() for t in pol(torch.from_numpy(x))]
        got = sess.run(['action', 'mu', 'log_std'], {'obs': x})
        err = max(float(np.abs(g - r).max()) for g, r in zip(got, ref))
        chk(f'ort_vs_torch_{name}', err <= TOL, err)
        one = sess.run(['action', 'mu', 'log_std'], {'obs': x[:1]})
        err1 = max(float(np.abs(g - r[:1]).max()) for g, r in zip(one, ref))
        chk(f'ort_batch1_{name}', err1 <= TOL, err1)
        if name == 'env_obs' and isaac_out is not None:
            g = np.concatenate(got, 1)
            res['isaac_cuda_vs_ort_max_abs'] = float(np.abs(g - isaac_out).max())
        if name == 'golden_decision':
            res['golden_actions'] = got[0].round(5).tolist()
    res['fails'] = fails
    with open(os.path.join(a.run, f'onnx_check_{stem}.json'), 'w') as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    print('ONNX_CHECK ' + json.dumps({'fails': fails, 'checks': {k: v['value'] for k, v in
                                      res['checks'].items() if 'ort' in k},
                                      'isaac_cuda_vs_ort': res.get('isaac_cuda_vs_ort_max_abs')}))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
