#!/usr/bin/env python3
"""
학습 정책 → ONNX (onnx_io.md 비교군 B) 내보내기와 onnxruntime 왕복 일치 시험. amr-ai/rl:0.3 에서 돈다.

    R=/home/CAPYI_AI/runs/r7-isaac12
    docker run --rm -v <wt>:/ws -v $R:$R -w /ws amr-ai/rl:0.3 \
        python3 rl/isaac/scripts/export_onnx.py --run $R/train/<run>

메타데이터는 학습 기록에서 만든다 (policy_export.onnx_metadata): algo·train_steps 는 정책 종류(파일 이름이
*_bc 이면 BC, 아니면 최종)·summary.json·bc.json 에서, 라그랑주 사실(lagrange_final·전부 0 이 된 반복·d₁·d₁ 이
계약 §7.2 훑기 값인지)은 summary.json·curve.jsonl 에서. --algo 로 덮어쓸 수 있지만 끝 λ 가 전부 0 인데 라그랑주를
주장하면 모델 파일을 쓰지 않고 rc 1 (algo_claim_consistent).

검사 (하나라도 어긋나면 rc 1):
  - 입력 obs [batch, 291] float32, 출력 action·mu·log_std [batch, 2] float32, opset 17, 필수 메타데이터
  - algo 문자열이 학습 기록과 모순되지 않는다 (algo_claim_consistent)
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
    ap.add_argument('--algo', default=None, help='없으면 학습 기록에서 만든다 (권장)')
    ap.add_argument('--kind', default='auto', choices=['auto', 'bc', 'final'])
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
    rows = []
    cp = os.path.join(a.run, 'curve.jsonl')
    if os.path.exists(cp):
        rows = [json.loads(line) for line in open(cp) if line.strip()]
    bc_info = {}
    bp = os.path.join(a.run, 'bc.json')
    if os.path.exists(bp):
        bc_info = json.load(open(bp))
    stem = os.path.splitext(a.policy)[0]
    kind = a.kind if a.kind != 'auto' else ('bc' if stem.endswith('_bc') else 'final')
    facts = PE.lagrange_facts(summ, rows)
    meta = PE.onnx_metadata(kind, summ, facts, head, C.CONTRACT_VERSION, algo=a.algo,
                            bc_info=bc_info)
    claim_ok = kind == 'bc' or PE.algo_claim_ok(meta['algo'], facts)
    if not claim_ok:
        # 틀린 algo 를 단 모델 파일을 남기지 않는다
        print('ONNX_CHECK ' + json.dumps({'fails': ['algo_claim_consistent'], 'algo': meta['algo'],
                                          'lagrange_final': facts['final']}))
        return 1
    path = os.path.join(a.run, f'{stem}.onnx')
    PE.export_onnx(pol, path, meta)
    m = onnx.load(path)
    opset = [o.version for o in m.opset_import if o.domain in ('', 'ai.onnx')]
    res = {'onnx': path, 'kind': kind, 'opset': opset,
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
    chk('algo_claim_consistent', claim_ok,
        {'algo': meta['algo'], 'lagrange_final': facts['final'],
         'all_zero_from_iteration': facts['all_zero_from_iteration']})

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
