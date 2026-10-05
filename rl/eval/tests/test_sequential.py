"""0 충돌 순차 감시 — 이론편 「0충돌 실증은 순차 검정과 잘 맞는다」 수치 재현."""

import math

from amr_rl_eval import sequential as S
from conftest import load_calc
import numpy as np
import pytest


def test_ville_design_numbers():
    d = S.SPRTDesign(p0=0.02, p1=0.005, alpha=0.025)
    assert d.upper == pytest.approx(math.log(40))              # 경계 ln 40 = 3.69
    assert round(d.upper, 2) == 3.69
    assert round(d.step_ok, 5) == 0.01519                      # 무충돌 +0.01519
    assert round(d.step_fail, 3) == -1.386                     # 충돌 −1.386
    assert d.n_zero == 243                                     # 0 충돌이면 243 회
    assert round(d.cost_per_failure) == 92                     # 충돌 1 회마다 약 92 회


def test_delta_1pp_design_numbers():
    d = S.SPRTDesign(p0=0.01, p1=0.0025, alpha=0.025)
    assert d.n_zero == 489
    assert round(d.cost_per_failure) == 185


def test_wald_boundary_shorter():
    # §4 (2): Wald 경계(β = 0.2) 면 0 충돌 종료가 229 회
    d = S.SPRTDesign(p0=0.02, p1=0.005, alpha=0.025, boundary='wald')
    assert d.n_zero == 229


@pytest.mark.parametrize('p,p_ni,en', [
    # §4 (2) 표 (δ = 2 %, Ville, 상한 600, 4 만 회)
    (0.0, 1.0, 243), (0.0025, 0.966, 314), (0.005, 0.779, 399), (0.01, 0.302, 530),
    (0.02, 0.020, 595),
])
def test_ville_oc_delta2(p, p_ni, en):
    r = S.simulate_oc(S.SPRTDesign(), p, reps=40000, seed=11)
    assert r['p_ni'] == pytest.approx(p_ni, abs=0.01)
    assert r['expected_n'] == pytest.approx(en, rel=0.02)


def test_ville_type_one_error_below_alpha():
    # 경계 p = p0 에서 NI 선언 확률 ≤ α (Ville 부등식)
    r = S.simulate_oc(S.SPRTDesign(), 0.02, reps=40000, seed=5)
    assert r['p_ni'] <= 0.025


@pytest.mark.parametrize('p,p_ni', [(0.0025, 0.293), (0.005, 0.085), (0.01, 0.007)])
def test_ville_oc_delta1(p, p_ni):
    r = S.simulate_oc(S.SPRTDesign(p0=0.01, p1=0.0025), p, reps=40000, seed=3)
    assert r['p_ni'] == pytest.approx(p_ni, abs=0.01)


def test_wald_with_futility_size():
    # §4 (2): Wald 근사 경계는 경계 p = 2 % 에서 1 종 오류 0.026 (명목 0.025 를 넘을 수 있다)
    d = S.SPRTDesign(boundary='wald', futility=True)
    r = S.simulate_oc(d, 0.02, reps=40000, seed=7)
    assert r['p_ni'] == pytest.approx(0.026, abs=0.004)


def test_run_sprt_zero_failures_stops_at_243():
    st = S.run_sprt([0] * 1000)
    assert st.decision == 'ni' and st.n == 243 and st.failures == 0


def test_run_sprt_one_failure_extends_by_about_92():
    st = S.run_sprt([1] + [0] * 1000)
    assert st.decision == 'ni'
    assert st.n - 243 == pytest.approx(92, abs=2)


def test_run_sprt_inconclusive_at_cap():
    seq = ([0] * 80 + [1]) * 10
    st = S.run_sprt(seq)
    assert st.decision == 'inconclusive' and st.n == 600


def test_decision_frozen_after_stop():
    st = S.run_sprt([0] * 243)
    assert st.update(True) == 'ni' and st.n == 243


@pytest.mark.parametrize('p0,alpha,n', [(0.01, 0.05, 299), (0.01, 0.025, 368),
                                        (0.02, 0.05, 149), (0.02, 0.025, 183)])
def test_success_run(p0, alpha, n):
    # §4 (1): success-run e-process 종료 = 고정 zero-failure 검정
    assert S.success_run_n(p0, alpha) == n
    assert S.success_run_e(n, 0, p0) >= 1 / alpha > S.success_run_e(n - 1, 0, p0)
    assert S.success_run_e(n, 1, p0) == 0.0


def test_cross_check_calc6_sim_wald():
    """calc6.sim (Wald 경계 + 포기 경계) 과 같은 설계에서 우리 모의가 같은 OC 를 낸다."""
    ns = load_calc('calc6.py', ['sim'])
    ns['rng'] = np.random.default_rng(2)
    sa, un, inc, en, p90, n0, cost = ns['sim'](0.02, 0.005, 0.025, 0.2, 0.005, 600, reps=40000)
    d = S.SPRTDesign(boundary='wald', futility=True)
    r = S.simulate_oc(d, 0.005, reps=40000, seed=9)
    assert n0 == d.n_zero
    assert cost == pytest.approx(d.cost_per_failure)
    assert r['p_ni'] == pytest.approx(sa, abs=0.012)
    assert r['expected_n'] == pytest.approx(en, rel=0.03)
