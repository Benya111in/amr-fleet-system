"""
Golden 31 사례 재생 — torch 배치 구현이 계약 golden 과 허용오차 안에서 일치하는가.

견본은 rl/contract/tests/test_golden.py 의 test_decision_replay·test_step_replay 다. 여기서는 사례를 한 배치로
묶어(경로 길이·트랙 수가 서로 다른 env 를 함께) 돌린다. 두 설정: CPU float64, GPU(env 와 같은 혼합 정밀도:
상태 float64, 스캔·롤아웃 float32).
"""

import glob
import json
import math
import os

import numpy as np
import pytest
import torch

from amr_sim2d import contract_torch as T
from amr_sim2d._contract import C, GOLDEN_DIR
from amr_sim2d.state_io import bstate_from_states

from amr_rl_contract.state import State

FILES = sorted(glob.glob(os.path.join(GOLDEN_DIR, '*.json')))
CONFIGS = [('cpu', torch.float64, torch.float64)]
if torch.cuda.is_available():
    CONFIGS.append(('cuda', torch.float64, torch.float32))


def _load(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def _kind(kind):
    return [_load(f) for f in FILES if _load(f)['kind'] == kind]


def test_golden_files_present():
    assert len(FILES) == 31
    for d in map(_load, FILES):
        assert d['contract_version'] == C.CONTRACT_VERSION


@pytest.mark.parametrize('device,dtype,scan_dtype', CONFIGS)
def test_decision_replay_batched(device, dtype, scan_dtype):
    cases = _kind('decision')
    assert len(cases) == 17
    states = [State.from_dict(d['state']) for d in cases]
    b = bstate_from_states(states, device, dtype, scan_dtype)
    obs = T.build_observation(b).double().cpu().numpy()
    res = T.compute_mask(b, roll_dtype=scan_dtype)
    for i, d in enumerate(cases):
        e, tol, name = d['expected'], d['tolerance'], d['name']
        err = np.abs(obs[i] - np.array(e['obs']))
        assert err.max() <= tol['obs'], (name, int(err.argmax()), float(err.max()))
        assert res['mask'][i].cpu().tolist() == e['mask'], name
        assert res['fp_ok'][i].cpu().tolist() == e['fp_ok'], name
        assert res['jerk_ok'][i].cpu().tolist() == e['jerk_ok'], name
        assert res['speed_ok'][i].cpu().tolist() == e['speed_ok'], name
        assert bool(res['saturated'][i]) == e['saturated'], name
        for a, bb in zip(res['vo_time'][i].cpu().tolist(), e['vo_time']):
            assert (math.isinf(a) and bb is None) or abs(a - bb) <= tol['vo_time'], name
        assert np.allclose(res['fallback'][i].cpu().numpy(), e['fallback'], atol=tol['fallback'])


@pytest.mark.parametrize('device,dtype,scan_dtype', CONFIGS)
def test_step_replay_batched(device, dtype, scan_dtype):
    cases = _kind('step')
    assert len(cases) == 11
    pre = bstate_from_states([State.from_dict(d['pre']) for d in cases], device, dtype)
    post = bstate_from_states([State.from_dict(d['post']) for d in cases], device, dtype)
    d_pre = T.remaining_arc(pre.path_pts, pre.path_n, pre.path_cum, pre.path_len, pre.pose[:, :2])
    pj = T.project_onto_path(post.path_pts, post.path_n, post.path_cum, post.pose[:, :2])
    d_post = (post.path_len - pj[0]).clamp_min(0.0)
    gdist = torch.linalg.norm(post.pose[:, :2] - T.goal_xy(post.path_pts, post.path_n), dim=-1)
    S = max(1, max(len(d['safety_pairs']) for d in cases))
    vin = torch.zeros(len(cases), S, dtype=dtype)
    win, vout, wout = vin.clone(), vin.clone(), vin.clone()
    cnt = torch.zeros(len(cases), dtype=torch.long)
    for i, d in enumerate(cases):
        for j, (a, bb) in enumerate(d['safety_pairs']):
            vin[i, j], win[i, j], vout[i, j], wout[i, j] = a[0], a[1], bb[0], bb[1]
        cnt[i] = len(d['safety_pairs'])
    interv = T.intervention_cost(vin, win, vout, wout, cnt).to(device)
    a_exec = torch.tensor([d['action_exec'] for d in cases], dtype=dtype, device=device)
    coll = torch.tensor([d['collision'] for d in cases], device=device)
    r = T.reward_step(d_pre, d_post, pj[1], gdist, post.v_meas, pre.prev_actions[:, 0], a_exec,
                      coll, (pre.dev_in, pre.dev_peak, pre.dev_t), interv)
    for i, d in enumerate(cases):
        e, tol, name = d['expected'], d['tolerance'], d['name']
        assert float(r['reward'][i]) == pytest.approx(e['reward'], abs=tol['reward']), name
        for k, v in e['terms'].items():
            assert float(r['terms'][k][i]) == pytest.approx(v, abs=tol['reward_terms']), (name, k)
        assert np.allclose(r['cost'][i].cpu().numpy(), e['cost'], atol=tol['cost']), name
        assert (bool(r['terminated'][i]), bool(r['success'][i])) == (e['terminated'], e['success'])
        assert bool(r['dev'][0][i]) == e['dev']['in_dev'], name
        assert float(r['dev'][2][i]) == pytest.approx(e['dev']['t_since_peak'], abs=1e-9), name
        assert float(r['phi_pre'][i]) == pytest.approx(e['phi_pre'], abs=tol['reward_terms'])
        assert float(r['phi_post'][i]) == pytest.approx(e['phi_post'], abs=tol['reward_terms'])


def test_integrate_replay_batched():
    d = _kind('integrate')[0]['expected']
    f64 = torch.float64
    b = d['b']
    out = T.integrate_b(torch.tensor([r['u'] for r in b], dtype=f64),
                        torch.tensor([r['v_cmd'] for r in b], dtype=f64),
                        torch.tensor([r['w_cmd'] for r in b], dtype=f64),
                        torch.tensor([r['v_cap'] for r in b], dtype=f64))
    assert np.allclose(torch.stack(out, 1).numpy(), [r['out'] for r in b], atol=1e-9)
    c = d['c']
    out = T.integrate_c(torch.tensor([r['index'] for r in c]),
                        torch.tensor([r['v_cmd'] for r in c], dtype=f64),
                        torch.tensor([r['w_cmd'] for r in c], dtype=f64))
    assert np.allclose(torch.stack(out, 1).numpy(), [r['out'] for r in c], atol=1e-9)
    nc = d['nearest_cell']
    a = torch.tensor([r['a'] for r in nc], dtype=f64)
    assert T.nearest_cell(a[:, 0], a[:, 1]).tolist() == [r['cell'] for r in nc]
    bl = d['bc_label']
    assert len(bl) >= 7
    lab = T.bc_label(torch.tensor([r['dwa'][0] for r in bl], dtype=f64),
                     torch.tensor([r['dwa'][1] for r in bl], dtype=f64),
                     torch.tensor([r['v_cmd'] for r in bl], dtype=f64),
                     torch.tensor([r['w_cmd'] for r in bl], dtype=f64),
                     torch.tensor([r['mask'] for r in bl]))
    assert lab.tolist() == [r['out']['label'] for r in bl]


def test_profiler_replay_batched():
    d = _kind('profiler')[0]
    steps = d['expected']['steps']
    prof = T.ProfilerBatch(1, 'cpu')
    for stp in steps:
        v0, w0 = stp['cmd_from']
        v1, w1 = stp['cmd_to']
        subs = []
        for k in range(1, 6):
            f = k / 5
            vr, wr = prof.step(torch.tensor([v0 + f * (v1 - v0)], dtype=torch.float64),
                               torch.tensor([w0 + f * (w1 - w0)], dtype=torch.float64))
            subs.append([float(vr[0]), float(wr[0])])
        assert np.allclose(subs, stp['ref'], atol=d['tolerance']['profiler'])


def test_safety_replay_batched():
    rows = _kind('safety')[0]['expected']['rows']
    f64 = torch.float64

    def col(key):
        return torch.tensor([math.inf if r['in'][key] is None else r['in'][key] for r in rows],
                            dtype=f64)
    out = T.safety_filter(col('v'), col('w'), col('d_main'), col('min_ttc'),
                          torch.tensor([r['in']['proximity_stop'] for r in rows]))
    for i, r in enumerate(rows):
        assert float(out['v'][i]) == pytest.approx(r['out']['v'], abs=1e-9)
        assert float(out['w'][i]) == pytest.approx(r['out']['w'], abs=1e-9)
        assert int(out['zone'][i]) == r['out']['zone']
        exp = math.inf if r['out']['speed_limit'] is None else r['out']['speed_limit']
        assert float(out['speed_limit'][i]) == pytest.approx(exp)
