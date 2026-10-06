"""보상·비용 시험 — PBRS 성질, 종단 Φ=0, Δd 를 쓰지 않는 이유, 궤적 순서, 이탈 상태기계."""

import importlib.util
import math
import os

import numpy as np
import pytest

from amr_rl_contract import constants as C
from amr_rl_contract import reward as R
from amr_rl_contract.state import DevState, empty_scan, State

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(
    __file__)))))
PATH = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]   # 20 m L 자


def _st(x, y, th=0.0, v=0.5, **kw):
    return State(pose=(x, y, th), path=PATH, scan_ranges=empty_scan(), v_meas=v, **kw)


def _rollout(xs, terminal_last, collision=False):
    """xs: [(x, y)] 상태열 → step 결과 목록 (마지막이 종단이면 collision 또는 성공)."""
    out = []
    states = [_st(x, y) for x, y in xs]
    if terminal_last and not collision:
        gx, gy = PATH[-1]
        states[-1] = _st(gx, gy, v=0.0)
    for k in range(len(states) - 1):
        last = k == len(states) - 2
        out.append(R.step(states[k], states[k + 1], (0.0, 0.0), collision and last))
    return states, out


def test_pbrs_telescopes_to_minus_phi0_on_termination():
    rng = np.random.default_rng(11)
    for collision in (False, True):
        xs = [(float(x), float(rng.uniform(-0.2, 0.2))) for x in np.linspace(0, 9.5, 60)]
        states, res = _rollout(xs, True, collision)
        shaped = sum(C.GAMMA ** t * r.terms['shape'] for t, r in enumerate(res))
        assert res[-1].phi_post == 0.0
        assert shaped == pytest.approx(-R.potential(states[0], False), abs=1e-9)


def test_pbrs_truncation_keeps_bootstrap_potential():
    xs = [(float(x), 0.0) for x in np.linspace(0, 4, 30)]
    states, res = _rollout(xs, False)
    T = len(res)
    shaped = sum(C.GAMMA ** t * r.terms['shape'] for t, r in enumerate(res))
    expect = C.GAMMA ** T * R.potential(states[-1], False) - R.potential(states[0], False)
    assert shaped == pytest.approx(expect, abs=1e-9)
    assert not res[-1].terminated


def test_terminal_potential_is_zero():
    st = _st(3.0, 0.0)
    assert R.potential(st, True) == 0.0
    assert R.potential(st, False) == pytest.approx(-C.KAPPA * 17.0)


def test_progress_reward_decomposition():
    # κ(d − d') = [γΦ(s') − Φ(s)] − κ(1 − γ)d'  (이론 보고서 「PBRS의 조건과 Δd의 정확한 분해」)
    rng = np.random.default_rng(2)
    for _ in range(50):
        a, b = sorted(rng.uniform(0, 19.9, 2))
        s, s2 = _st(*R.G.interpolate_at(PATH, a)), _st(*R.G.interpolate_at(PATH, b))
        d, d2 = 20.0 - a, 20.0 - b
        pbrs = C.GAMMA * R.potential(s2, False) - R.potential(s, False)
        assert C.KAPPA * (d - d2) == pytest.approx(pbrs - C.KAPPA * (1 - C.GAMMA) * d2, abs=1e-9)


def _chain_policy(n, shaping, crash_penalty, p_crash, gamma=C.GAMMA):
    """
    1 차원 사슬 MDP 가치 반복. 상태 i = 목표까지 i 칸(0.1 m).

    행동: 0 전진(안전, 1 칸), 1 돌진(2 칸, 확률 p_crash 로 충돌 종단), 2 정지.
    shaping: 'none' | 'pbrs' (Φ=−κd, 종단 0) | 'delta_d' (κ(d − d')). 반환: 상태별 최적 행동.
    """
    dc = 0.1

    def sh(d, d2, term):
        if shaping == 'pbrs':
            return gamma * (0.0 if term else -C.KAPPA * d2) + C.KAPPA * d
        if shaping == 'delta_d':
            return C.KAPPA * (d - d2)
        return 0.0

    V = np.zeros(n + 1)
    pol = np.zeros(n + 1, dtype=int)
    for _ in range(3 * n):
        Vn = V.copy()
        for i in range(1, n + 1):
            d = i * dc
            j = i - 1
            t = j == 0
            q0 = -C.TIME_COST + (C.GOAL_REWARD if t else 0.0) + sh(d, j * dc, t) + \
                (0.0 if t else gamma * V[j])
            j = max(0, i - 2)
            t = j == 0
            ok = -C.TIME_COST + (C.GOAL_REWARD if t else 0.0) + sh(d, j * dc, t) + \
                (0.0 if t else gamma * V[j])
            crash = -C.TIME_COST - crash_penalty + sh(d, d, True)
            q1 = (1.0 - p_crash) * ok + p_crash * crash
            q2 = -C.TIME_COST + sh(d, d, False) + gamma * V[i]
            q = [q0, q1, q2]
            Vn[i] = max(q)
            pol[i] = int(np.argmax(q))
        V = Vn
    return pol


def test_pbrs_preserves_optimal_policy_but_delta_d_does_not():
    n, p = 300, 0.002          # 30 m, 돌진 충돌 확률 0.2 %
    base = _chain_policy(n, 'none', C.COLLISION_PENALTY, p)
    pbrs = _chain_policy(n, 'pbrs', C.COLLISION_PENALTY, p)
    dd = _chain_policy(n, 'delta_d', C.COLLISION_PENALTY, p)
    assert np.array_equal(base, pbrs)
    assert not np.any(base[1:] == 1)              # 기저 보상: 위험한 돌진을 하지 않는다
    far = np.flatnonzero(dd == 1)
    # Δd 의 숨은 항 −κ(1−γ)d' (거리 비례 생존비) → 먼 곳에서 충돌 위험을 산다
    assert far.size > 0 and far.min() * 0.1 > 10.0


def _ret(rews, gamma):
    return sum(gamma ** t * r for t, r in enumerate(rews))


def test_trajectory_ordering_knox():
    """
    τ_crash ≺ τ_idle ≺ τ_succ, τ_loiter ≺ τ_succ — 할인(shaping 포함)과 무할인(기저) 모두.

    충돌은 **조기 충돌**(5 m 지점, 5 s)이다. 진행을 오래 모은 뒤의 늦은 충돌은 계약 v1.2.0 의 할인 가치에서
    대기보다 높아질 수 있다 (§6.3 알려진 대가) — test_reward_variants.py::test_late_crash_* 가 그 위치를 고정한다.
    """
    def run(xs, collision=False, success=False):
        states = [_st(x, 0.0) for x in xs]
        if success:
            states[-1] = State(pose=(10.0, 10.0, 0.0), path=PATH, scan_ranges=empty_scan())
        res = []
        for k in range(len(states) - 1):
            res.append(R.step(states[k], states[k + 1], (0.0, 0.0),
                              collision and k == len(states) - 2))
        shaped = [r.reward for r in res]
        base = [r.reward - r.terms['shape'] for r in res]
        return _ret(shaped, C.GAMMA), sum(base)

    go = list(np.linspace(0.0, 10.0, 101)) + [10.0] * 100
    succ = run(go, success=True)
    crash = run(list(np.linspace(0.0, 5.0, 51)), collision=True)
    idle = run([0.0] * (int(R.time_limit(PATH) / C.DT) + 1))   # 시간 상한까지
    loiter = run(list(np.linspace(0.0, 9.0, 91)) + [9.0, 9.05] * 300)
    for k in (0, 1):
        assert crash[k] < idle[k] < succ[k], (k, crash, idle, succ)
        assert loiter[k] < succ[k]


def test_cost_vector_semantics():
    pre = _st(5.0, 0.0)
    r = R.step(pre, _st(5.05, 1.2), (0.0, 0.0), False)
    assert r.cost[2] == 1.0 and r.dev.in_dev
    r = R.step(pre, _st(5.05, 0.08), (0.0, 0.0), False)
    assert r.cost == (0.0, 0.0, 0.0, 0.0, 0.0)
    r = R.step(pre, _st(5.05, 0.12), (0.0, 0.0), False)
    assert r.cost[4] == 1.0


def _load_metrics():
    path = os.path.join(REPO, 'tests', 'integration', 'amr_itest', 'metrics.py')
    spec = importlib.util.spec_from_file_location('itest_metrics', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_dev_machine_matches_integration_metrics():
    metrics = _load_metrics()
    rng = np.random.default_rng(7)
    for _ in range(40):
        n = 400
        e = np.abs(np.cumsum(rng.normal(0, 0.04, n)))
        times = [k * C.DT for k in range(n)]
        eps = metrics.deviation_episodes(times, list(e), C.DEV_OUT, C.DEV_BACK)
        dev = DevState()
        mine, cur = [], None
        for k in range(n):
            nd = R.dev_update(dev, float(e[k]))
            t = times[k]
            if not dev.in_dev and nd.in_dev:
                cur = [t, t, nd.peak]
            elif dev.in_dev and nd.in_dev:
                if nd.t_since_peak == 0.0:
                    cur[1], cur[2] = t, nd.peak
                assert nd.t_since_peak == pytest.approx(t - cur[1], abs=1e-9)
            elif dev.in_dev and not nd.in_dev:
                mine.append((cur[0], cur[1], cur[2], t))
                cur = None
            dev = nd
        if cur is not None:
            mine.append((cur[0], cur[1], cur[2], None))
        ref = [(x.t_start, x.t_peak, x.peak, x.t_end) for x in eps]
        assert len(mine) == len(ref)
        for a, b in zip(mine, ref):
            assert a[0] == pytest.approx(b[0]) and a[1] == pytest.approx(b[1])
            assert a[2] == pytest.approx(b[2])
            assert (a[3] is None and b[3] is None) or a[3] == pytest.approx(b[3])


def test_success_requires_stop_and_tolerance():
    st = State(pose=(10.0, 9.95, 0.0), path=PATH, scan_ranges=empty_scan(), v_meas=0.04)
    assert R.is_success(st)
    st.v_meas = 0.2
    assert not R.is_success(st)
    st.v_meas = 0.0
    st.pose = (10.0, 9.85, 0.0)
    assert not R.is_success(st)
    st.pose = (10.0, 10.0, 0.0)
    assert not R.is_success(st, collision=True)


def test_time_limit_and_progress_checker():
    assert R.time_limit(PATH) == pytest.approx(min(C.TIME_LIMIT_MAX, 20.0 + 3.0 * 20.0))
    pc = R.ProgressChecker((0.0, 0.0), 0.0)
    assert not pc.update((0.1, 0.0), 19.0)
    assert pc.update((0.2, 0.0), 20.5)
    assert not pc.update((0.35, 0.0), 21.0)    # 0.3 m 이상 → 기준 갱신
    assert not pc.update((0.35, 0.0), 40.0) and pc.update((0.35, 0.0), 41.5)


def test_intervention_cost_zero_when_untouched():
    assert R.intervention_cost([]) == 0.0
    assert R.intervention_cost([((0.5, 0.1), (0.5, 0.1))]) == 0.0
    assert R.intervention_cost([((1.0, 0.0), (0.0, 0.0))]) == pytest.approx(1.0)
    assert math.isfinite(R.intervention_cost([((0.3, 1.5), (0.0, 0.0))]))


def test_track_penalty_off_during_deviation_episode():
    pre = _st(5.0, 0.45, dev=DevState(True, 0.5, 1.0))
    r = R.step(pre, _st(5.05, 0.25), (0.0, 0.0), False)
    assert r.dev.in_dev and r.terms['track'] == 0.0
    pre = _st(5.0, 0.05)
    r = R.step(pre, _st(5.05, 0.25), (0.0, 0.0), False)
    assert not r.dev.in_dev and r.terms['track'] == pytest.approx(-C.TRACK_WEIGHT * 9.0)
