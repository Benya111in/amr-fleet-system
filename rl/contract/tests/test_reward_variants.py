"""
보상 설계 후보의 착취 단위 시험 (「이론」 211–213행: Knox 식 궤적 쌍 비교를 보상 코드의 자동 시험으로).

궤적은 20 m L 자 경로 위의 손으로 만든 상태열이다. 값은 두 가지로 잰다:
  - 할인: 학습 목적. 끝나지 않은 궤적은 마지막 상태에서 같은 정책을 계속한 연속 롤아웃(정확한 critic 의
    부트스트랩)까지 더한다 — truncation 은 bootstrap 이므로 학습자가 실제로 비교하는 값이다.
  - 무할인 기저(shaping 제외): 계약 에피소드(첫 정체·시간 상한 truncation 또는 종단)까지의 합.
v1.1.0 과 pbrs_eff 는 "대기 = 진행 후 정지" 가 같은 값(평평)임을 고정하고, 진행형 후보(prog*)는 엄격한
순서를 요구한다.

충돌 궤적은 두 갈래로 본다 (리뷰 B2): 20 m 경로의 5 m 지점 **조기 충돌**은 모든 후보에서 할인·무할인 모두 대기보다
낮다. 진행을 오래 모은 뒤의 **늦은 충돌**은 진행형 후보에서 할인 가치가 대기보다 높아진다 — 할인 조건
κ_b·Σ_{t<T} γ^t Δd_t < γ^{T−1}(C − γ·c̄) 가 깨진다. 이것은 계약 §6.3 의 알려진 대가이고 아래 시험이 위반의
위치를 고정한다. 결정 수준(같은 지점까지 와서 정지 ≻ 충돌)은 C > γ·c̄ 만으로 모든 후보에서 성립한다.
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
PROG = ('prog25', 'prog50', 'prog50_st', 'v1.2.0')
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
def test_early_crash_idle_success_order_discounted_and_undiscounted(values, name):
    """조기 충돌(20 m 경로의 5 m) ≺ 대기 ≺ 성공 — 늦은 충돌은 test_late_crash_* 가 따로 본다."""
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


def test_candidates_spell_out_every_term():
    """후보는 기본값(계약 버전마다 바뀐다)에 기대지 않는다: 이름 → 항 값이 버전과 무관하게 고정."""
    want = {'v1.1.0': ('arc', 0.0, 0.0, False), 'pbrs_eff': ('eff', 0.0, 0.0, False),
            'prog25': ('eff', 0.25, 0.05, False), 'prog50': ('eff', 0.5, 0.05, False),
            'prog50_st': ('eff', 0.5, 0.05, True)}
    for name, w in want.items():
        sp = RV.get(name)
        assert (sp.phi_dist, sp.progress_kappa, sp.dev_hinge, sp.stall_terminal) == w, name


@pytest.mark.parametrize('name', list(RV.CANDIDATES))
def test_undiscounted_suicide_window_arithmetic(name):
    """
    무할인 계약 에피소드의 자살 창 — 산술만 본다 (궤적은 test_undiscounted_late_crash_below_idle).

    정체 truncation 으로 20 s 에 끝나는 대기의 무할인 기저는 −c·200 (= c̄ = 4 와 같은 값). 30 m 진행 효용
    κ_b·30 을 모은 뒤의 충돌이 그보다 낮으려면 C > c·200 + κ_b·d_max (κ_b 0.5 → 19). **할인 가치의 순서는
    이것으로 보장되지 않는다** (늦은 충돌의 벌점이 γ^T 로 줄어든다 — test_late_crash_*). 정체 종단이면 대기
    영구보다 이른 정체가 이득이 아니도록 P_s ≥ c̄, 무할인으로 충돌 ≺ 정체(20 s 대기 뒤)가 되도록 P_s < C − c·200.
    """
    sp = RV.get(name)
    cbar = sp.time_cost / (1.0 - C.GAMMA)
    steps = C.STALL_TIME / C.DT
    assert sp.collision_penalty > sp.time_cost * steps + sp.progress_kappa * 30.0
    if sp.stall_terminal:
        assert cbar <= sp.stall_penalty < sp.collision_penalty - sp.time_cost * steps


def _line(L):
    return [(0.0, 0.0), (L, 0.0)]


def _line_value(L, xs, spec, crash):
    """
    직선 L m 경로에서 위치열 xs(xs[0] = 0)를 행동 0 으로 따라간 (할인 가치, 무할인 기저 에피소드 합).

    crash: 마지막 전이가 충돌. 아니면 끝에서 정지해 TAIL 스텝(정체 종단이면 종단까지) 대기 — 할인 가치는
    truncation 을 넘는 연속 롤아웃(정확한 critic 부트스트랩), 무할인 기저는 첫 정체·시간 상한까지.
    """
    path = _line(L)
    t_lim = R.time_limit(path)
    pc = R.ProgressChecker((xs[0], 0.0), 0.0)
    pre = State(pose=(xs[0], 0.0, 0.0), path=path, scan_ranges=empty_scan())
    n = len(xs) - 1
    total = n if crash else n + TAIL
    disc, g, base, ep_open = 0.0, 1.0, 0.0, True
    for k in range(total):
        x = xs[k + 1] if k < n else xs[-1]
        post = State(pose=(x, 0.0, 0.0), path=path, scan_ranges=empty_scan(),
                     v_meas=1.0 if k < n else 0.0)
        post.dev = pre.dev
        t = (k + 1) * C.DT
        stalled = pc.update((x, 0.0), t)
        r = R.step(pre, post, (0.0, 0.0), crash and k == n - 1, spec=spec, stalled=stalled)
        disc += g * r.reward
        g *= C.GAMMA
        if ep_open:
            base += r.reward - r.terms['shape']
        if r.terminated:
            break
        if stalled or t > t_lim:
            ep_open = False
        post.dev = r.dev
        pre = post
    return disc, base


def _approach(x_end, v):
    n = max(1, int(round(x_end / (v * C.DT))))
    return [x_end * i / n for i in range(n + 1)]


def _idle_disc(L, spec):
    """경로 위 영구 대기의 할인 가치: 정체가 truncation 이면 닫힌 꼴 κ·L − c̄ (TAIL 절단 오차 없이)."""
    if not spec.stall_terminal:
        return spec.kappa * L - spec.time_cost / (1.0 - C.GAMMA)
    return _line_value(L, [0.0], spec, False)[0]


def _late_crash_minus_idle(L, v, spec):
    """속도 v 로 목표 0.5 m 앞까지 간 뒤 충돌 − 영구 대기 (할인)."""
    xs = _approach(L - 0.5, v)
    return _line_value(L, xs, spec, True)[0] - _idle_disc(L, spec), xs


@pytest.mark.parametrize('name', [n for n in RV.CANDIDATES if not RV.get(n).stall_terminal])
def test_discounted_crash_vs_idle_is_closed_form(name):
    """
    할인 (진행 뒤 충돌) − (영구 대기) = κ_b·Σ_{t<T} γ^t Δd_t − γ^{T−1}(C − γ·c̄)  (경로 위, 행동 0).

    T = 충돌 스텝을 포함한 스텝 수. PBRS 합은 두 궤적 모두 −Φ(s₀) 로 상쇄되고, 시간 비용 차는 γ^T·c̄ 다.
    늦은 충돌일수록 벌점 항이 γ^{T−1} 로 줄고 진행 효용 항은 남는다 — κ_b > 0 이면 충분히 늦은 충돌이
    대기보다 높아진다.
    """
    sp = RV.get(name)
    cbar = sp.time_cost / (1.0 - C.GAMMA)
    for L, v in ((6.0, 0.1), (15.0, 0.5), (30.0, 1.0), (30.0, 0.3)):
        d, xs = _late_crash_minus_idle(L, v, sp)
        T = len(xs) - 1
        bank = sum(C.GAMMA ** t * (xs[t + 1] - xs[t]) for t in range(T))
        penalty = C.GAMMA ** (T - 1) * (sp.collision_penalty - C.GAMMA * cbar)
        want = sp.progress_kappa * bank - penalty
        assert d == pytest.approx(want, abs=1e-9), (name, L, v)
        if not sp.progress_kappa:
            assert d < 0.0, (name, L, v)               # v1.1.0·pbrs_eff: 늘 대기보다 낮다


def test_late_crash_beats_idle_discounted_known_cost():
    """
    알려진 대가 고정 (계약 §6.3, reward_v12.md §3.4): v1.2.0 의 할인 Knox τ_crash ≺ τ_idle 은 늦은 충돌에서 깨진다.

    목표 0.5 m 앞 충돌이 대기보다 높아지는 최소 경로 길이 (0.5 m 격자): 1.0 m/s 26.5 m, 0.3 m/s 14.5 m.
    30 m 를 1 m/s 로 29.5 m 간 뒤의 충돌은 대기보다 +1.76. prog25 도 0.5 m/s 25 m 에서 깨진다.
    v1.1.0 은 30 m 안 어디서도 깨지지 않는다.
    """
    v12 = R.SPEC
    for L, v in ((26.5, 1.0), (14.5, 0.3)):
        assert _late_crash_minus_idle(L, v, v12)[0] > 0.0, (L, v)
        assert _late_crash_minus_idle(L - 0.5, v, v12)[0] < 0.0, (L, v)
    assert _late_crash_minus_idle(30.0, 1.0, v12)[0] == pytest.approx(1.76, abs=0.01)
    assert _late_crash_minus_idle(25.0, 0.5, RV.PROG25)[0] > 0.0
    assert _late_crash_minus_idle(24.5, 0.5, RV.PROG25)[0] < 0.0
    for v in (0.3, 1.0):
        assert _late_crash_minus_idle(30.0, v, R.SPEC_V110)[0] < 0.0


@pytest.mark.parametrize('name', list(RV.CANDIDATES))
def test_undiscounted_late_crash_below_idle(name):
    """무할인 계약 에피소드: 30 m 를 1 m/s 로 29.5 m 간 뒤의 충돌도 정체 truncation 으로 끝나는 대기보다 낮다."""
    sp = RV.get(name)
    crash = _line_value(30.0, _approach(29.5, 1.0), sp, True)[1]
    idle = _line_value(30.0, [0.0], sp, False)[1]
    assert crash < idle, (name, crash, idle)


@pytest.mark.parametrize('name', list(RV.CANDIDATES))
def test_stop_beats_crash_at_every_point(name):
    """
    결정 수준의 자살은 없다: 같은 접근 뒤 그 자리 정지 ≻ 그 자리 충돌 (30 m 경로, 1 m/s·0.3 m/s, 2.5–29.5 m).

    진행 효용은 두 궤적에서 같으므로 차이는 γ^{T−1}(C − γ·c̄) 이고 κ_b 와 무관하다 — C > γ·c̄ 이면 충분하다
    (정체 종단 후보는 정지 뒤 정체 벌점 γ^{T+200}·P_s 를 더 낸다).
    """
    sp = RV.get(name)
    cbar = sp.time_cost / (1.0 - C.GAMMA)
    for v in (1.0, 0.3):
        for xc in (2.5, 10.0, 20.0, 29.5):
            xs = _approach(xc, v)
            stop = _line_value(30.0, xs, sp, False)[0]
            crash = _line_value(30.0, xs, sp, True)[0]
            T = len(xs) - 1
            gap = C.GAMMA ** (T - 1) * (sp.collision_penalty - C.GAMMA * cbar)
            assert stop > crash, (name, v, xc)
            if not sp.stall_terminal:
                # TAIL 절단 오차 ≈ 1e-5
                assert stop - crash == pytest.approx(gap, abs=1e-4), (name, v, xc)


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


def test_default_spec_is_contract_v120_and_equals_prog50():
    """기본 RewardSpec = 계약 상수 v1.2.0 = G2 를 통과한 prog50 (golden 이 이것으로 만들어졌다)."""
    sp = R.SPEC
    assert sp.name == 'v1.2.0' == 'v' + C.CONTRACT_VERSION
    got = (sp.kappa, sp.phi_dist, sp.progress_kappa, sp.dev_hinge, sp.stall_terminal)
    assert got == (C.KAPPA, C.PHI_DIST, C.PROGRESS_KAPPA, C.DEV_HINGE, False)
    assert got == (1.0, 'eff', 0.5, 0.05, False)
    assert (sp.time_cost, sp.goal_reward, sp.collision_penalty) == \
        (C.TIME_COST, C.GOAL_REWARD, C.COLLISION_PENALTY)
    p = RV.PROG50
    assert (p.kappa, p.phi_dist, p.progress_kappa, p.dev_hinge, p.stall_terminal, p.stall_penalty,
            p.time_cost, p.goal_reward, p.collision_penalty) == \
        (sp.kappa, sp.phi_dist, sp.progress_kappa, sp.dev_hinge, sp.stall_terminal,
         sp.stall_penalty, sp.time_cost, sp.goal_reward, sp.collision_penalty)
    v = R.SPEC_V110                                  # 1차 물결 계약 (재현용)
    assert (v.phi_dist, v.progress_kappa, v.dev_hinge, v.stall_terminal) == \
        ('arc', 0.0, 0.0, False)


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


def test_dev_hinge_applies_beyond_1m_regardless_of_deviation_mode():
    """1 m 초과 이탈 힌지: 이탈 구간 안팎과 무관, 1 m 이하에서는 0 (회피 중 CTE 끔은 유지)."""
    from amr_rl_contract.state import DevState
    sp = RV.PROG50
    pre = _st(_arc(5.0))
    pre.dev = DevState(True, 1.3, 0.5)
    far = R.step(pre, _st((5.05, 1.2)), (0.0, 0.0), False, spec=sp)
    assert far.dev.in_dev and far.terms['dev'] == -sp.dev_hinge and far.terms['track'] == 0.0
    near = R.step(pre, _st((5.05, 0.9)), (0.0, 0.0), False, spec=sp)
    assert near.dev.in_dev and near.terms['dev'] == 0.0 and near.terms['track'] == 0.0
    out = R.step(_st(_arc(5.0)), _st((5.05, 1.05)), (0.0, 0.0), False, spec=sp)
    assert out.terms['dev'] == -sp.dev_hinge
    assert 'dev' not in R.step(pre, _st((5.05, 1.2)), (0.0, 0.0), False, spec=R.SPEC_V110).terms
