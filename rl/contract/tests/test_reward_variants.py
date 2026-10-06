"""
보상 설계 후보의 착취 단위 시험 (「이론」 211–213행: Knox 식 궤적 쌍 비교를 보상 코드의 자동 시험으로).

궤적은 20 m L 자 경로 위의 손으로 만든 상태열이다. 값은 두 가지로 잰다:
  - 할인: 학습 목적. 끝나지 않은 궤적은 마지막 상태에서 같은 정책을 계속한 연속 롤아웃(정확한 critic 의
    부트스트랩)까지 더한다 — truncation 은 bootstrap 이므로 학습자가 실제로 비교하는 값이다.
  - 무할인 기저(shaping 제외): 계약 에피소드(첫 정체·시간 상한 truncation 또는 종단)까지의 합.
v1.1.0 과 pbrs_eff 는 "대기 = 진행 후 정지" 가 같은 값(평평)임을 고정하고, 진행형 후보(prog*)는 엄격한
순서를 요구한다.
"""

import math

import numpy as np
import pytest

from amr_rl_contract import constants as C
from amr_rl_contract import reward as R
from amr_rl_contract import reward_variants as RV
from amr_rl_contract.state import empty_scan, State

PATH = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]   # 20 m
L = 20.0
STEP = 0.1                                      # 1 m/s × 0.1 s
TAIL = 3000                                     # γ^3000 ≈ 3e-7
PROG = ('prog25', 'prog50', 'prog50_st')
FLAT = ('v1.1.0', 'pbrs_eff')


def _arc(s):
    return R.G.interpolate_at(PATH, s)


def _st(xy, v=0.0):
    return State(pose=(float(xy[0]), float(xy[1]), 0.0), path=PATH, scan_ranges=empty_scan(),
                 v_meas=v)


def _value(xys, spec, collide_last=False, tail=None, actions=None):
    """
    상태열 xys 의 (할인 가치, 무할인 기저 에피소드 합, 결말).

    tail: 끝나지 않으면 이어 붙일 위치 생성기 k → xy (None = 마지막 위치에 정지). 정체 판정은
    ProgressChecker (N:86-87), 시간 상한은 reward.time_limit — 둘 다 truncation (spec.stall_terminal 이면
    정체는 종단). 할인 가치는 truncation 을 무시하고 종단 또는 TAIL 스텝까지 간다 (연속 롤아웃 부트스트랩).
    """
    t_lim = R.time_limit(PATH)
    pc = R.ProgressChecker(xys[0], 0.0)
    pre = _st(xys[0])
    disc, g, base_ep, ep_open, outcome = 0.0, 1.0, 0.0, True, 'open'
    n = len(xys) - 1
    total = n + (TAIL if not collide_last else 0)
    for k in range(total):
        if k < n:
            xy = xys[k + 1]
        else:
            xy = tail(k - n) if tail else xys[-1]
        moving = k < n
        post = _st(xy, v=0.5 if moving else 0.0)
        post.dev = pre.dev
        a = actions[k] if (actions and k < len(actions)) else (0.0, 0.0)
        t = (k + 1) * C.DT
        stalled = pc.update(xy, t)
        coll = collide_last and k == n - 1
        r = R.step(pre, post, a, coll, spec=spec, stalled=stalled)
        disc += g * r.reward
        g *= C.GAMMA
        if ep_open:
            base_ep += r.reward - r.terms['shape']
        if r.terminated:
            if ep_open:
                outcome = 'success' if r.success else ('collision' if coll else 'stall')
            break
        if ep_open and (stalled or t > t_lim):
            ep_open = False
            outcome = 'stall' if stalled else 'timeout'
        post.dev = r.dev
        post.prev_actions = [a] + list(pre.prev_actions[:2])
        pre = post
    return disc, base_ep, outcome


def _drive(s0, s1):
    k = max(1, int(round(abs(s1 - s0) / STEP)))
    return [_arc(s0 + (s1 - s0) * i / k) for i in range(k + 1)]


def _trajs():
    succ = _drive(0.0, L)
    tr = {
        'succ': (succ, False, None),
        'idle': ([_arc(0.0)], False, None),
        'partial': (_drive(0.0, 15.0), False, None),           # 5 m 전에 정지
        'crash': (_drive(0.0, 5.0), True, None),
        'drift': (_drive(0.0, 5.0) + [(5.0, 0.1 * i) for i in range(1, 31)], False, None),
        'circle': ([_arc(0.0)], False,
                   lambda k: (0.33 * math.sin(k * 0.3), 0.33 - 0.33 * math.cos(k * 0.3))),
        'overshoot': (succ[:-1] + [(10.0, 10.0 + 0.1 * i) for i in range(1, 11)], False,
                      lambda k: (10.0 + 0.33 * math.sin(k * 0.3),
                                 11.0 - 0.33 * math.cos(k * 0.3))),
    }
    # 1 m 우회 후 복귀해 성공 (Knox τ_swerve_1m)
    sw = []
    for xy in succ:
        x, y = xy
        bump = 1.0 * math.exp(-((x - 5.0) / 1.0) ** 2) if y == 0.0 else 0.0
        sw.append((x, y + bump))
    tr['swerve'] = (sw, False, None)
    return tr


@pytest.fixture(scope='module')
def values():
    out = {}
    tr = _trajs()
    for name, spec in RV.CANDIDATES.items():
        out[name] = {k: _value(xys, spec, coll, tail) for k, (xys, coll, tail) in tr.items()}
        spin_actions = [(0.0, C.ALPHA_MAX)] * 8 + [(0.0, 0.0)] * 8
        out[name]['spin'] = _value([_arc(0.0)] * 17, spec, actions=spin_actions)
    return out


def test_outcomes_are_what_the_trajectory_says(values):
    for name, v in values.items():
        assert v['succ'][2] == 'success' and v['swerve'][2] == 'success', name
        assert v['crash'][2] == 'collision', name
        assert v['idle'][2] == 'stall' and v['partial'][2] == 'stall', name
        assert v['circle'][2] == 'timeout', name        # 0.66 m 원은 정체 판정을 피한다


@pytest.mark.parametrize('name', list(RV.CANDIDATES))
def test_crash_idle_success_order_discounted_and_undiscounted(values, name):
    v = values[name]
    for k in (0, 1):
        assert v['crash'][k] < v['idle'][k] < v['succ'][k], (name, k, v['crash'], v['idle'])
        assert v['circle'][k] < v['succ'][k] and v['overshoot'][k] < v['succ'][k], name
        assert v['idle'][k] < v['swerve'][k], name                  # τ_wait_forever ≺ τ_swerve_1m
        assert v['spin'][k] <= v['idle'][k] + 1e-9, name            # 회전은 대기보다 낫지 않다


@pytest.mark.parametrize('name', FLAT)
def test_flat_designs_value_partial_progress_like_idle(values, name):
    """v1.1.0 의 결함 고정: 끝나지 않는 궤적의 할인 가치 = κ·d₀ − c/(1−γ), 진행량과 무관 (대기 = 직전 정지)."""
    v = values[name]
    flat = C.KAPPA * L - C.TIME_COST / (1.0 - C.GAMMA)
    for k in ('idle', 'partial'):
        assert v[k][0] == pytest.approx(flat, abs=0.02), (name, k, v[k][0])
    assert v['drift'][0] == pytest.approx(flat, abs=0.12)     # 경로 밖 정지도 같다 (추종 벌점 꺼짐)
    assert v['circle'][0] < flat                              # 배회는 추종 벌점만 더 낸다


@pytest.mark.parametrize('name', PROG)
def test_progress_designs_order_partial_progress(values, name):
    """진행형 후보: 대기 ≺ 직전 정지 ≺ 성공, 경로 밖 표류 ≺ 직전 정지, 원 배회 ≺ 직전 정지 (할인·무할인)."""
    v = values[name]
    kb = RV.get(name).progress_kappa
    # 할인: 15 m 진행 효용 κ_b·15 를 γ^~75 로 할인한 값의 일부. 무할인: κ_b·15 − c·150(주행 시간 비용) > 0
    # 이려면 κ_b > c/(v·Δt) = 0.2 /m (1 m/s) — 후보는 모두 그 위다.
    margins = (0.2 * kb * 15.0, 0.5 * (kb - 0.2) * 15.0)
    for k in (0, 1):
        assert v['idle'][k] + margins[k] < v['partial'][k] < v['succ'][k], (name, k, v['idle'],
                                                                            v['partial'])
        assert v['drift'][k] < v['partial'][k], (name, k)
        assert v['circle'][k] < v['partial'][k], (name, k)
        assert v['crash'][k] < v['idle'][k], (name, k)


@pytest.mark.parametrize('name', list(RV.CANDIDATES))
def test_suicide_window_holds_to_30m(name):
    """
    자살 하한 (「이론」 201행).

    진행 효용은 종단에서 돌려받지 않으므로 "진행 κ_b·d 를 모은 뒤 충돌" ≺ "대기" 가 C > c̄ + κ_b·d_max 를
    요구한다 (d_max 30 m, §10). 정체 종단이면 대기 영구보다 이른 정체가 이득이 아니도록 P_s ≥ c̄, 무할인으로
    충돌 ≺ 정체(20 s 대기 뒤)가 되도록 P_s < C − c·200.
    """
    sp = RV.get(name)
    cbar = sp.time_cost / (1.0 - C.GAMMA)
    assert sp.collision_penalty > cbar + sp.progress_kappa * 30.0
    if sp.stall_terminal:
        steps = C.STALL_TIME / C.DT
        assert cbar <= sp.stall_penalty < sp.collision_penalty - sp.time_cost * steps


def _chain_rush_states(kb, p, n=300, gamma=C.GAMMA):
    """
    §6.1 사슬 MDP 에 기저 진행 효용 κ_b·(d − d') 를 더한 최적 정책의 돌진 상태 (PBRS 는 최적 불변이라 뺀다).

    상태 i = 목표까지 i 칸(0.1 m), 행동: 전진 / 2 칸 돌진(확률 p 로 그 자리 충돌 종단) / 정지.
    """
    dc = 0.1
    V = np.zeros(n + 1)
    pol = np.zeros(n + 1, dtype=int)
    for _ in range(6 * n):
        Vn = V.copy()
        for i in range(1, n + 1):
            def mv(j):
                t = j == 0
                return (-C.TIME_COST + kb * (i - j) * dc + (C.GOAL_REWARD if t else 0.0) +
                        (0.0 if t else gamma * V[j]))
            crash = -C.TIME_COST - C.COLLISION_PENALTY
            q = [mv(i - 1), (1.0 - p) * mv(max(0, i - 2)) + p * crash,
                 -C.TIME_COST + gamma * V[i]]
            Vn[i] = max(q)
            pol[i] = int(np.argmax(q))
        V = Vn
    return np.flatnonzero(pol == 1)


@pytest.mark.parametrize('name', PROG)
def test_progress_utility_does_not_buy_crash_risk_within_30m(name):
    """κ_b 는 최적 정책을 바꾼다 — 그 크기의 상한: 0.2 % 충돌 돌진을 30 m 안 어디서도 고르지 않는다."""
    assert _chain_rush_states(RV.get(name).progress_kappa, 0.002).size == 0


def test_chain_rush_threshold_documents_kappa_window():
    """κ_b 0.8 까지는 돌진이 없고 1.0(= Δd, §6.1) 은 16 m 너머에서 고른다 — reward_v12.md 의 창."""
    assert _chain_rush_states(0.8, 0.002).size == 0
    r = _chain_rush_states(1.0, 0.002)
    assert r.size > 0 and r.min() * 0.1 > 16.0


def test_default_spec_is_contract_v110():
    """기본 RewardSpec = 계약 상수 (golden 이 이것으로 만들어졌다)."""
    sp = R.SPEC_V110
    assert (sp.kappa, sp.phi_dist, sp.progress_kappa, sp.dev_hinge, sp.stall_terminal) == \
        (C.KAPPA, 'arc', 0.0, 0.0, False)
    assert (sp.time_cost, sp.goal_reward, sp.collision_penalty) == \
        (C.TIME_COST, C.GOAL_REWARD, C.COLLISION_PENALTY)


def test_progress_term_is_delta_d_and_sums_to_net_progress():
    """진행 효용 = κ_b(d − d'): 같은 곳을 오가면 무할인 합이 0 (배회로 쌓이지 않는다), 충돌 스텝은 실제 d'."""
    sp = RV.PROG50
    a, b = _st(_arc(4.0)), _st(_arc(4.5))
    fwd = R.step(a, b, (0.0, 0.0), False, spec=sp).terms['progress']
    back = R.step(b, a, (0.0, 0.0), False, spec=sp).terms['progress']
    assert fwd == pytest.approx(0.25) and fwd + back == pytest.approx(0.0, abs=1e-12)
    crash = R.step(a, b, (0.0, 0.0), True, spec=sp)
    assert crash.terms['progress'] == pytest.approx(0.25) and crash.terms['shape'] == \
        pytest.approx(C.KAPPA * 16.0)


def test_eff_potential_sees_overshoot_beyond_path_end():
    """d_arc 는 끝점 너머에서 0 으로 평평하다 — 'eff' 는 목표 직선거리를 센다."""
    beyond = _st((10.0, 11.0))
    assert R.progress_distance(beyond, R.SPEC_V110) == pytest.approx(0.0, abs=1e-12)
    assert R.progress_distance(beyond, RV.PBRS_EFF) == pytest.approx(1.0)
    on = _st(_arc(7.0))
    assert R.progress_distance(on, RV.PBRS_EFF) == pytest.approx(13.0)


def test_stall_terminal_only_when_spec_says_so():
    pre, post = _st(_arc(3.0)), _st(_arc(3.0))
    r0 = R.step(pre, post, (0.0, 0.0), False, spec=RV.PROG50, stalled=True)
    assert not r0.terminated and 'stall' not in r0.terms
    r1 = R.step(pre, post, (0.0, 0.0), False, spec=RV.PROG50_ST, stalled=True)
    assert r1.terminated and r1.terms['stall'] == -10.0 and r1.phi_post == 0.0
    assert r1.terms['progress'] == 0.0          # 진행 효용은 종단에서 κ_b·d 를 돌려주지 않는다
    goal = _st(PATH[-1], v=0.0)
    r2 = R.step(pre, goal, (0.0, 0.0), False, spec=RV.PROG50_ST, stalled=True)
    assert r2.success and r2.terms['stall'] == 0.0           # 같은 스텝이면 성공이 이긴다
