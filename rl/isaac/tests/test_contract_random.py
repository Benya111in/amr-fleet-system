"""
무작위 차분 시험 — contract_torch(배치) 와 기준 구현 amr_rl_contract(numpy, 한 건씩)를 같은 입력으로 비교.

golden 31 개가 덮지 못한 분기(투영 동률·꼭짓점, 이탈 종료 문턱 사이, 도달했지만 빠름, 곡률 모드 이탈,
트랙 정렬 동률, safety 존 경계)를 무작위로 훑는다. 변이 검사에서 golden 만으로는 놓친 변이를 잡으려고
더했다 (REPORT 「시험」).
"""

import math
import random

import numpy as np
import pytest
import torch

from amr_isaac import contract_torch as CT
from amr_rl_contract import constants as C
from amr_rl_contract import geometry as G
from amr_rl_contract import observation as OB
from amr_rl_contract import profiler as PR
from amr_rl_contract import reward as R
from amr_rl_contract import safety as S
from amr_rl_contract.state import DevState, SafetyState, State, Track

F64 = torch.float64


def _rand_path(rng, n_min=2, n_max=12):
    """꺾인 폴리라인 (가끔 중복 점·되돌아감)."""
    n = rng.randint(n_min, n_max)
    pts = [(rng.uniform(-5, 5), rng.uniform(-5, 5))]
    for _ in range(n - 1):
        r = rng.random()
        if r < 0.1:
            pts.append(pts[-1])                                  # 길이 0 구간
        elif r < 0.2 and len(pts) > 1:
            pts.append(pts[-2])                                  # 되돌아감 (동률 투영)
        else:
            pts.append((pts[-1][0] + rng.uniform(-2, 2), pts[-1][1] + rng.uniform(-2, 2)))
    return pts


def _bstate(path, pose):
    return CT.states_from_dicts([{'pose': list(pose), 'path': path,
                                  'scan_ranges': [None] * C.LIDAR_SAMPLES}])


def test_projection_matches_reference_incl_vertices():
    rng = random.Random(1)
    for _ in range(400):
        path = _rand_path(rng)
        if rng.random() < 0.4:                       # 꼭짓점·구간 위의 점 (동률 분기)
            i = rng.randrange(len(path))
            p = path[i]
        else:
            p = (rng.uniform(-6, 6), rng.uniform(-6, 6))
        st = _bstate(path, (p[0], p[1], 0.0))
        s, d, e, yaw = [float(t[0]) for t in CT.project_onto_path(
            st, torch.tensor([p], dtype=F64))]
        rs, rd, re_, ryaw = G.project_onto_path(path, p)
        assert (s, d, e) == pytest.approx((rs, rd, re_), abs=1e-9)
        assert math.cos(yaw - ryaw) == pytest.approx(1.0, abs=1e-9)


def test_dev_update_matches_reference():
    rng = random.Random(2)
    for _ in range(200):
        dev = DevState()
        ind = torch.tensor([False])
        pk = torch.tensor([0.0], dtype=F64)
        tt = torch.tensor([0.0], dtype=F64)
        for _ in range(40):
            e = rng.choice([rng.uniform(0, 0.5), rng.uniform(0.12, 0.33), rng.uniform(0, 1.5)])
            dev = R.dev_update(dev, e)
            ind, pk, tt = CT.dev_update(ind, pk, tt, torch.tensor([e], dtype=F64))
            assert bool(ind[0]) == dev.in_dev
            assert float(pk[0]) == pytest.approx(dev.peak, abs=1e-12)
            assert float(tt[0]) == pytest.approx(dev.t_since_peak, abs=1e-9)


def test_success_matches_reference():
    rng = random.Random(3)
    path = [(0.0, 0.0), (2.0, 0.0)]
    for _ in range(300):
        x = 2.0 + rng.uniform(-0.15, 0.15)
        y = rng.uniform(-0.15, 0.15)
        v = rng.choice([0.0, 0.03, 0.05, 0.051, 0.3, -0.06])
        col = rng.random() < 0.2
        st = State(pose=(x, y, 0.0), path=path, scan_ranges=[None] * 720, v_meas=v)
        got = CT.is_success(torch.tensor([[x, y, 0.0]], dtype=F64),
                            torch.tensor([[2.0, 0.0]], dtype=F64),
                            torch.tensor([v], dtype=F64), torch.tensor([col]))
        assert bool(got[0]) == R.is_success(st, col)


def test_profiler_random_sequences_match_reference():
    rng = random.Random(4)
    B = 24
    refs = [PR.ProfilerReference() for _ in range(B)]
    prof = CT.ProfilerBatch(B)
    for step in range(400):
        vt = [rng.choice([0.0, 0.03, rng.uniform(-0.2, 1.2)]) for _ in range(B)]
        wt = [rng.uniform(-1.6, 1.6) for _ in range(B)]
        out = [r.step(v, w) for r, v, w in zip(refs, vt, wt)]
        v_ref, w_ref = prof.step(torch.tensor(vt, dtype=F64), torch.tensor(wt, dtype=F64))
        for b in range(B):
            assert float(v_ref[b]) == pytest.approx(out[b][0], abs=1e-9), (step, b)
            assert float(w_ref[b]) == pytest.approx(out[b][1], abs=1e-9), (step, b)


def _rand_state(rng):
    path = _rand_path(rng, 2, 8)
    th = rng.uniform(-math.pi, math.pi)
    pose = (rng.uniform(-4, 4), rng.uniform(-4, 4), th)
    scan = []
    for _ in range(C.LIDAR_SAMPLES):
        u = rng.random()
        scan.append(None if u < 0.1 else rng.uniform(0.05, 26.0) if u < 0.6 else
                    rng.uniform(0.1, 3.0))
    tracks = []
    for i in range(rng.randint(0, 11)):
        d = rng.choice([2.0, rng.uniform(0.5, 11.0)])     # 거리 동률 (id 로 가른다)
        a = rng.uniform(-math.pi, math.pi)
        tracks.append(Track(rng.randint(0, 50), pose[0] + d * math.cos(a),
                            pose[1] + d * math.sin(a), rng.uniform(-1.5, 1.5),
                            rng.uniform(-1.5, 1.5), rng.random() < 0.6))
    ids = set()
    for t in tracks:                                      # id 는 서로 다르게
        while t.id in ids:
            t.id += 51
        ids.add(t.id)
    return State(
        pose=pose, path=path, scan_ranges=scan,
        v_meas=rng.uniform(0, 1.2), w_meas=rng.uniform(-1.5, 1.5),
        a_v_meas=rng.uniform(-1, 1), a_w_meas=rng.uniform(-2, 2),
        v_cmd=rng.uniform(0, 1.0), w_cmd=rng.uniform(-1.5, 1.5),
        prev_actions=[(rng.uniform(-1, 1), rng.uniform(-2, 2)) for _ in range(3)],
        v_cap=rng.choice([1.0, 2.0, 0.5]), scan_age=rng.uniform(0, 0.4),
        tracks=tracks, track_age=rng.uniform(0, 0.6),
        safety=SafetyState(rng.randint(0, 3), rng.choice([None, rng.uniform(0, 2.5)]),
                           rng.random() < 0.2, rng.choice([None, rng.uniform(0, 6)])),
        dev=DevState(*(lambda f: (True, f, rng.uniform(0, 8)) if rng.random() < 0.4
                       else (False, 0.0, 0.0))(rng.uniform(0.3, 2.0))))


def test_observation_random_states_match_reference():
    rng = random.Random(5)
    states = [_rand_state(rng) for _ in range(60)]
    bst = CT.states_from_dicts([s.to_dict() for s in states])
    obs = CT.build_observation(bst)
    for i, s in enumerate(states):
        ref = OB.build_observation(s)
        err = np.abs(obs[i].numpy().astype(np.float64) - ref.astype(np.float64))
        bad = {k: float(err[sl].max()) for k, sl in OB.obs_slices().items()
               if float(err[sl].max()) > 1e-5}
        assert not bad, (i, bad)


def test_reward_step_random_matches_reference():
    rng = random.Random(6)
    for _ in range(150):
        pre = _rand_state(rng)
        post = _rand_state(rng)
        post.path = pre.path
        if rng.random() < 0.3:                            # 목표 근처 (성공 분기)
            gx, gy = pre.path[-1]
            post.pose = (gx + rng.uniform(-0.12, 0.12), gy + rng.uniform(-0.12, 0.12), 0.0)
            post.v_meas = rng.choice([0.0, 0.04, 0.2])
        a = (rng.uniform(-1, 1), rng.uniform(-2, 2))
        col = rng.random() < 0.2
        pairs = [((rng.uniform(0, 1), rng.uniform(-1, 1)),
                  (rng.uniform(0, 1), rng.uniform(-1, 1)))
                 for _ in range(rng.randint(0, 5))]
        ref = R.step(pre, post, a, col, pairs)
        bp = CT.states_from_dicts([pre.to_dict()])
        bq = CT.states_from_dicts([post.to_dict()])
        pt = None
        if pairs:
            pt = torch.tensor([[x[0][0], x[0][1], x[1][0], x[1][1]] for x in pairs],
                              dtype=F64).unsqueeze(0)
        out = CT.reward_step(bp, bq.pose, bq.v_meas, torch.tensor([a], dtype=F64),
                             torch.tensor([col]), pt)
        assert float(out.reward[0]) == pytest.approx(ref.reward, abs=1e-9)
        assert np.allclose(out.cost[0].numpy(), ref.cost, atol=1e-12)
        assert (bool(out.terminated[0]), bool(out.success[0])) == (ref.terminated, ref.success)
        assert bool(out.in_dev[0]) == ref.dev.in_dev


def test_safety_filter_random_matches_reference():
    rng = random.Random(7)
    rows = []
    for _ in range(500):
        d = rng.choice([None, rng.uniform(0, 1.5), 0.3, 0.5, 1.0])
        ttc = rng.choice([None, rng.uniform(0, 3), 2.15])
        rows.append((rng.uniform(-0.5, 2.0), rng.uniform(-1.5, 1.5), d, ttc, rng.random() < 0.1))
    out = CT.safety_filter(*[torch.tensor([math.inf if r[k] is None else r[k] for r in rows],
                                          dtype=F64) for k in range(4)],
                           torch.tensor([r[4] for r in rows]))
    for i, r in enumerate(rows):
        ref = S.safety_filter(*r)
        assert float(out['v'][i]) == pytest.approx(ref['v'], abs=1e-12)
        assert float(out['w'][i]) == pytest.approx(ref['w'], abs=1e-12)
        assert int(out['zone'][i]) == ref['zone']


def test_integrate_b_random_matches_reference():
    from amr_rl_contract import kinematics as K
    rng = random.Random(8)
    rows = [((rng.uniform(-1.3, 1.3), rng.uniform(-1.3, 1.3)), rng.uniform(0, 2.0),
             rng.uniform(-1.6, 1.6), rng.choice([0.5, 1.0, 2.0, 3.0])) for _ in range(500)]
    out = CT.integrate_b(torch.tensor([r[0] for r in rows], dtype=F64),
                         torch.tensor([r[1] for r in rows], dtype=F64),
                         torch.tensor([r[2] for r in rows], dtype=F64),
                         torch.tensor([r[3] for r in rows], dtype=F64))
    for i, r in enumerate(rows):
        ref = K.integrate_b(*r)
        assert [float(o[i]) for o in out] == pytest.approx(list(ref), abs=1e-12)
