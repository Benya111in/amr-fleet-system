"""
B5 집계 스크립트(tools/b5_analysis.py)를 가짜 원자료로 돌려 본다 — 사전 등록 전에 집계가 도는지, 판정 규칙이 맞는지.

가짜 자료: DWA 는 주 집합 전부 성공·스트레스 절반 성공, RL 은 주 집합 시드마다 2 행 실패·스트레스 전부 성공.
그러면 Δ(main_3_15) < 0 이 작고, Δ(stress) > 0 이 크다.
"""

import json
import os
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(os.path.dirname(HERE), 'tools', 'b5_analysis.py')
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'tools'))

import b5_analysis as A  # noqa: E402


def _fake(tmp_path, n_main=40, n_stress=20, seed=0):
    rng = np.random.default_rng(seed)
    sets, pids = [], []
    for st, n in (('main_3_15', n_main), ('stress_3_15', n_stress), ('main_3_30', n_main),
                  ('stress_3_30', n_stress)):
        sets += [st] * n
        pids += list(range(len(pids), len(pids) + n))
    N = len(sets)
    sets = np.array(sets)
    pids = np.array(pids)
    # 풀: 행마다 경로 하나, 끝점을 8 개 목표로 묶는다
    pts, offs = [], [0]
    for i in range(N):
        g = i % 8
        p = np.stack([np.linspace(0, 3 + g, 10), np.full(10, float(g))], 1)
        pts.append(p)
        offs.append(offs[-1] + len(p))
    np.savez(tmp_path / 'pool.npz', pts=np.concatenate(pts), offs=np.array(offs),
             length=np.ones(N))
    yaw = rng.uniform(-0.5, 0.5, N)

    def ep(outcome):
        return dict(set=sets, pid=pids, yaw0=yaw, outcome=outcome, nav2=np.ones(N, bool),
                    collision_moving=np.zeros(N, bool), time_s=np.full(N, 10.0),
                    interv_events=np.zeros(N), interv_sub=np.zeros(N), stop_sub=np.zeros(N),
                    cost=np.zeros((N, 5)), max_e=np.zeros(N), travel=np.ones(N),
                    path_len=np.ones(N), spin_steps=np.zeros(N), dwa_no_valid_ticks=np.zeros(N),
                    dwa_exceptions=np.zeros(N), d_goal_end=np.full(N, 0.05),
                    goal_min=np.full(N, 0.05), d_start=np.full(N, 5.0))
    stress = np.char.startswith(sets.astype(str), 'stress')
    dwa = np.where(stress & (np.arange(N) % 2 == 0), 2, 0)
    runs = tmp_path / 'runs'
    for name, oc in [('B5_dwa_W1', dwa)]:
        os.makedirs(runs / name)
        np.savez(runs / name / 'episodes.npz', **ep(oc))
    isa = tmp_path / 'isaac'
    for s in range(10):
        oc = np.zeros(N, dtype=np.int64)
        main_idx = np.nonzero(~stress)[0]
        oc[rng.choice(main_idx, 2, replace=False)] = 3
        os.makedirs(runs / f'B5_rl_s{s}_W1')
        np.savez(runs / f'B5_rl_s{s}_W1' / 'episodes.npz', **ep(oc))
        os.makedirs(isa / f'B1e_ppo10_s{s}_final_sets')
        e = ep(np.zeros(N, dtype=np.int64))
        e['path_id_orig'] = pids
        np.savez(isa / f'B1e_ppo10_s{s}_final_sets' / 'episodes.npz', **e)
    return runs, isa, tmp_path / 'pool.npz', dwa, stress


def test_analysis_runs_on_fake_data(tmp_path):
    runs, isa, pool, dwa, stress = _fake(tmp_path)
    out = tmp_path / 'out'
    r = subprocess.run([sys.executable, TOOL, '--runs', str(runs), '--isaac', str(isa),
                        '--pool', str(pool), '--out', str(out)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr[-2000:]
    res = json.load(open(out / 'analysis.json'))
    m = res['sets']['main_3_15']
    assert m['dwa_rate'] == 1.0
    rates = [m['rl_rate_by_seed'][str(s)] for s in range(10)]
    assert m['delta'] == pytest.approx(np.mean(rates) - 1.0)
    assert m['delta'] < 0
    s3 = res['sets']['stress_3_30']
    assert s3['delta'] == pytest.approx(0.5)
    assert s3['verdict_two_way'] == 'RL 이 높다'
    d = res['direction']['stress:stress_3_30']
    assert d['verdict_10seed'] == '같은 방향'
    assert res['direction']['main:main_3_15']['verdict_10seed'] == '방향 판정 불가'
    assert s3['twod_minus_isaac']['mean'] == pytest.approx(0.0)
    # DWA 스트레스 실패 = 정체 ∧ goal_min 0.05 → 목표 앞 정체
    assert s3['fail_modes_dwa']['near_goal_stall'] == int((dwa[stress] == 2).sum()) // 2


def test_verdict_and_direction_rules():
    assert A.verdict(0.01, 0.02) == 'RL 이 높다'
    assert A.verdict(-0.02, -0.01) == 'RL 이 낮다'
    assert A.verdict(-0.01, 0.01) == '가르지 못함'
    assert A.direction(0.01, 0.1, 0.06, 0.19) == '같은 방향'
    assert A.direction(-0.1, -0.01, 0.06, 0.19) == '반대 방향'
    assert A.direction(-0.1, 0.01, 0.06, 0.19) == '방향 판정 불가'
    assert A.direction(0.01, 0.1, -0.03, 0.01) == '방향 판정 불가'


def test_two_way_bootstrap_point_and_degenerate():
    S = np.array([[1, 1, 0, 1], [1, 0, 1, 1]])
    D = np.array([1, 1, 1, 0])
    g = np.array([0, 0, 1, 2])
    est, lo, hi = A.two_way(S, D, g, reps=500)
    assert est == pytest.approx(S.mean() - D.mean())
    assert lo <= est <= hi
    e2, l2, h2 = A.two_way(np.ones((3, 4)), np.ones(4), g, reps=200)
    assert (e2, l2, h2) == (0.0, 0.0, 0.0)
