#!/usr/bin/env python3
"""
학습 정책 ONNX 의 Python ORT·torch·C++ 플러그인 출력 일치 판정 (보너스 1 A2, amr-ai/rl:0.3).

    parity_check.py prep-isaac --runs s0=<run dir>,... --out DATA
    parity_check.py check --data DATA --cpp CPP --rsl RSL --runs s0=<run dir>,... --out report.json

prep-isaac: 학습 run 의 Isaac 평가 기록 관측(eval_*/obs_batch.npz)을 seed 마다 모아
  DATA/isaac_obs_<s>.npy 와, 최종 정책(final)으로 기록한 행의 Isaac CUDA 출력
  DATA/isaac_out_<s>.npy (action·mu·log_std 6 열, final 이 아닌 행은 NaN) 을 쓴다.
check: 아래 판정 (A2-2 노트 「설정」에 결과 전에 고정) 을 계산해 JSON 으로 쓰고 표를 출력한다.
  P1 C++ OnnxPolicy 대 Python ORT (같은 관측, 배치 1)   최대 |Δ| < 1e-5, 비트 동일 비율
  P2 Python ORT 대 torch PolicyB · rsl_rl ActorCritic (CPU)   최대 |Δ| < 1e-5 (CUDA·Isaac 기록은 보고)
  P3 C++ 반복(정순·역순·섞음) · 별개 프로세스 · Python ORT 반복   비트 동일
  P4 C++ 관측 대 numpy 기준 관측 (golden·무작위 상태)   최대 |Δ| ≤ 1e-5
  P5 ONNX 안 정규화 한 번 · 두 번 정규화 경로와의 차 > 1e-3
  P6 integrate_b 세 구현 (C++·numpy·Isaac torch) 최대 |Δ| ≤ 1e-12, tanh float32 대 float64 (보고)
"""

import argparse
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..', '..', '..'))
sys.path.insert(0, os.environ.get('RL_CONTRACT_DIR', os.path.join(REPO, 'rl', 'contract')))
sys.path.insert(0, os.environ.get('RL_ISAAC_DIR', os.path.join(REPO, 'rl', 'isaac')))

TOL = 1e-5
SETS = ('golden_ref', 'random_ref', 'normal', 'isaac')


def _runs(spec):
    return dict(kv.split('=', 1) for kv in spec.split(','))


def prep_isaac(a):
    """Isaac 평가 기록 관측을 seed 마다 모은다."""
    os.makedirs(a.out, exist_ok=True)
    summary = {}
    for s, run in _runs(a.runs).items():
        obs, out, src = [], [], []
        for f in sorted(glob.glob(os.path.join(run, 'eval_*', 'obs_batch.npz'))):
            z = np.load(f)
            o = z['obs'].astype(np.float32)
            final = os.path.basename(os.path.dirname(f)).startswith('eval_final')
            obs.append(o)
            out.append(z['out'].astype(np.float32) if final
                       else np.full((len(o), 6), np.nan, np.float32))
            src.append({'file': f, 'n': len(o), 'final_policy_output': final})
        np.save(os.path.join(a.out, f'isaac_obs_{s}.npy'), np.concatenate(obs))
        np.save(os.path.join(a.out, f'isaac_out_{s}.npy'), np.concatenate(out))
        summary[s] = src
    json.dump(summary, open(os.path.join(a.out, 'isaac_sources.json'), 'w'), indent=1)
    print(json.dumps({k: sum(x['n'] for x in v) for k, v in summary.items()}))


def _f32(path, width):
    return np.fromfile(path, dtype=np.float32).reshape(-1, width)


def _bits_equal(x, y):
    return int(np.sum(x.view(np.uint32) == y.view(np.uint32)))


def _maxabs(x, y):
    d = np.abs(x.astype(np.float64) - y.astype(np.float64))
    return float(np.nanmax(d)) if d.size else 0.0


def _ort_session(path, threads=1):
    import onnxruntime as ort
    so = ort.SessionOptions()
    so.intra_op_num_threads = threads
    so.inter_op_num_threads = 1
    so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    so.add_session_config_entry('session.intra_op.allow_spinning', '0')
    return ort.InferenceSession(path, so, providers=['CPUExecutionProvider'])


def _ort_rows(sess, x):
    """배치 1 로 한 행씩 (플러그인과 같은 호출 모양) → action·mu·log_std."""
    out = [sess.run(['action', 'mu', 'log_std'], {'obs': x[i:i + 1]}) for i in range(len(x))]
    return [np.concatenate([o[k] for o in out]) for k in range(3)]


def _graph_report(path):
    """ONNX 그래프에서 입력 직후 정규화 패턴을 센다."""
    import onnx
    m = onnx.load(path)
    ops = [n.op_type for n in m.graph.node]
    first_gemm = next((i for i, o in enumerate(ops) if o in ('Gemm', 'MatMul')), len(ops))
    pre = ops[:first_gemm]
    return {'ops_before_first_gemm': pre, 'n_clip_before_gemm': pre.count('Clip'),
            'n_sub_before_gemm': pre.count('Sub'), 'n_nodes': len(ops),
            'op_hist': {o: ops.count(o) for o in sorted(set(ops))}}


def check(a):
    """판정 계산."""
    import torch
    from amr_isaac import contract_torch as CT
    from amr_isaac import policy_export as PE
    torch.set_num_threads(1)
    data = {k: np.load(os.path.join(a.data, f'{k}_obs_ref.npy')) for k in ('golden', 'random')}
    normal = np.load(os.path.join(a.data, 'normal_obs.npy'))
    rep = {'tol': TOL, 'seeds': {}, 'fails': []}

    def chk(name, ok):
        if not ok:
            rep['fails'].append(name)

    for s, run in _runs(a.runs).items():
        onnx_path = os.path.join(run, 'policy_final.onnx')
        ck = torch.load(os.path.join(run, 'policy_final.pt'), map_location='cpu')
        pol = PE.PolicyB.from_rsl_state_dict(ck['state_dict'], ck.get('activation', 'elu')).eval()
        sess = _ort_session(onnx_path)
        rsl = np.load(os.path.join(a.rsl, f'rsl_{s}.npz')) if a.rsl else None
        sets = {'golden_ref': data['golden'], 'random_ref': data['random'], 'normal': normal,
                'isaac': np.load(os.path.join(a.data, f'isaac_obs_{s}.npy'))}
        isaac_out = np.load(os.path.join(a.data, f'isaac_out_{s}.npy'))
        r = {'onnx': onnx_path, 'graph': _graph_report(onnx_path), 'sets': {}}
        g = r['graph']
        chk(f'{s}:P5_graph_one_norm', g['n_clip_before_gemm'] == 1 and g['n_sub_before_gemm'] == 1)
        for name, x in sets.items():
            x = np.ascontiguousarray(x, dtype=np.float32)
            act, mu, logstd = _ort_rows(sess, x)
            act2 = _ort_rows(sess, x)[0]
            actn = sess.run(['action'], {'obs': x})[0]
            cpp = _f32(os.path.join(a.cpp, f'{s}_{name}_act.f32'), 2)
            with torch.no_grad():
                tb = [t.numpy() for t in pol(torch.from_numpy(x))]
                xn = PE.contract_normalize(torch.from_numpy(x), pol.mean, pol.var)
                twice = torch.tanh(pol.actor(PE.contract_normalize(xn, pol.mean, pol.var)))
            e = {'n': len(x),
                 'P1_cpp_vs_ort_max': _maxabs(cpp, act),
                 'P1_cpp_vs_ort_bit_identical': f'{_bits_equal(cpp, act)}/{act.size}',
                 'P2_torchB_vs_ort_action_max': _maxabs(tb[0], act),
                 'P2_torchB_vs_ort_mu_max': _maxabs(tb[1], mu),
                 'P2_torchB_vs_ort_logstd_max': _maxabs(tb[2], logstd),
                 'P3_ort_repeat_bit_identical': f'{_bits_equal(act, act2)}/{act.size}',
                 'ort_batchN_vs_batch1_max': _maxabs(actn, act),
                 'P5_twice_normalized_vs_ort_max': _maxabs(twice.numpy(), act),
                 'tanh_f32_vs_f64_max': _maxabs(np.tanh(mu.astype(np.float64)), act),
                 'action_abs_max': float(np.abs(act).max()),
                 'action_saturated_frac(|a|>0.999)': float(np.mean(np.abs(act) > 0.999))}
            chk(f'{s}:{name}:P1', e['P1_cpp_vs_ort_max'] < TOL)
            chk(f'{s}:{name}:P2_torchB', max(e['P2_torchB_vs_ort_action_max'],
                                             e['P2_torchB_vs_ort_mu_max']) < TOL)
            chk(f'{s}:{name}:P3_ort', e['P3_ort_repeat_bit_identical'].split('/')[0]
                == str(act.size))
            chk(f'{s}:{name}:P5_twice_distinguishable', e['P5_twice_normalized_vs_ort_max'] > 1e-3)
            if rsl is not None:
                stem = {'golden_ref': 'golden_obs_ref', 'random_ref': 'random_obs_ref',
                        'normal': 'normal_obs', 'isaac': f'isaac_obs_{s}'}[name]
                e['P2_rslrl_cpu_vs_ort_action_max'] = _maxabs(rsl[f'{stem}_act_cpu'], act)
                e['P2_rslrl_cpu_vs_ort_mu_max'] = _maxabs(rsl[f'{stem}_mu_cpu'], mu)
                chk(f'{s}:{name}:P2_rslrl', max(e['P2_rslrl_cpu_vs_ort_action_max'],
                                                e['P2_rslrl_cpu_vs_ort_mu_max']) < TOL)
                if f'{stem}_act_cuda' in rsl:
                    e['rslrl_cuda_vs_ort_action_max(보고)'] = _maxabs(rsl[f'{stem}_act_cuda'], act)
            if name == 'isaac':
                ok = ~np.isnan(isaac_out[:, 0])
                e['isaac_recorded_rows'] = int(ok.sum())
                if ok.any():
                    e['isaac_cuda_recorded_vs_ort_action_max(보고)'] = _maxabs(
                        isaac_out[ok, :2], act[ok])
                    e['isaac_cuda_recorded_vs_ort_mu_max(보고)'] = _maxabs(
                        isaac_out[ok, 2:4], mu[ok])
            rep_p = os.path.join(a.cpp, f'{s}_{name}_report.json')
            if os.path.exists(rep_p):
                cr = json.load(open(rep_p))
                e['P3_cpp_repeat'] = cr.get('repeat_bit_compare')
                e['cpp_run_ms'] = cr.get('run_ms')
                same = all(b['differing_values'] == 0 for b in cr.get('repeat_bit_compare', []))
                chk(f'{s}:{name}:P3_cpp_repeat', same)
            for tag in ('p2', 't4'):
                p = os.path.join(a.cpp, f'{s}_{name}_{tag}_act.f32')
                if os.path.exists(p):
                    o = _f32(p, 2)
                    e[f'P3_cpp_{tag}_bit_identical'] = f'{_bits_equal(o, cpp)}/{cpp.size}'
                    if tag == 'p2':
                        chk(f'{s}:{name}:P3_cpp_process', _bits_equal(o, cpp) == cpp.size)
            r['sets'][name] = e
        # 끝-끝: C++ 상태 → 관측 → 행동 대 numpy 관측 → Python ORT
        for name in ('golden', 'random'):
            cobs = _f32(os.path.join(a.cpp, f'{s}_{name}_e2e_obs.f32'), 291)
            cact = _f32(os.path.join(a.cpp, f'{s}_{name}_e2e_act.f32'), 2)
            ref = data[name]
            act = _ort_rows(sess, np.ascontiguousarray(ref))[0]
            dobs = np.abs(cobs.astype(np.float64) - ref.astype(np.float64))
            bad = np.where(dobs.max(axis=1) > TOL)[0]
            e = {'n': len(ref), 'P4_obs_max': float(dobs.max()),
                 'P4_obs_rows_over_tol': [int(i) for i in bad[:20]],
                 'P4_obs_rows_over_tol_n': int(len(bad)),
                 'P4_obs_bit_identical_values': f'{_bits_equal(cobs, ref)}/{ref.size}',
                 'e2e_action_max': _maxabs(cact, act),
                 'e2e_action_bit_identical': f'{_bits_equal(cact, act)}/{act.size}'}
            if len(bad):
                e['P4_worst'] = [{'row': int(i), 'col': int(dobs[i].argmax()),
                                  'cpp': float(cobs[i, dobs[i].argmax()]),
                                  'ref': float(ref[i, dobs[i].argmax()])} for i in bad[:5]]
            chk(f'{s}:{name}:P4', e['P4_obs_max'] <= TOL)
            chk(f'{s}:{name}:e2e', e['e2e_action_max'] < TOL)
            r['sets'][f'{name}_e2e'] = e
        rep['seeds'][s] = r
    # P6 적분
    rows = np.fromfile(os.path.join(a.data, 'integrate_in.f64')).reshape(-1, 5)
    ref = np.load(os.path.join(a.data, 'integrate_ref.npy'))
    cpp = np.fromfile(os.path.join(a.cpp, 'integrate.f64')).reshape(-1, 4) \
        if os.path.exists(os.path.join(a.cpp, 'integrate.f64')) else None
    t = torch.from_numpy(rows)
    ct = torch.stack(CT.integrate_b(t[:, :2], t[:, 2], t[:, 3], t[:, 4]), dim=1).numpy()
    rep['P6'] = {'n': len(rows), 'isaac_torch_vs_numpy_max': _maxabs(ct, ref)}
    if cpp is not None:
        rep['P6']['cpp_vs_numpy_max'] = _maxabs(cpp, ref)
        rep['P6']['cpp_vs_isaac_torch_max'] = _maxabs(cpp, ct)
        chk('P6_cpp', rep['P6']['cpp_vs_numpy_max'] <= 1e-12)
    chk('P6_isaac', rep['P6']['isaac_torch_vs_numpy_max'] <= 1e-12)
    rep['verdict'] = 'pass' if not rep['fails'] else 'fail'
    json.dump(rep, open(a.out, 'w'), indent=1, ensure_ascii=False)
    print(json.dumps({'verdict': rep['verdict'], 'fails': rep['fails']}, ensure_ascii=False))


def main():
    """하위 명령."""
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest='cmd', required=True)
    p = sp.add_parser('prep-isaac')
    p.add_argument('--runs', required=True)
    p.add_argument('--out', required=True)
    p.set_defaults(f=prep_isaac)
    c = sp.add_parser('check')
    for k in ('--data', '--cpp', '--runs', '--out'):
        c.add_argument(k, required=True)
    c.add_argument('--rsl', default='')
    c.set_defaults(f=check)
    a = ap.parse_args()
    a.f(a)


if __name__ == '__main__':
    main()
