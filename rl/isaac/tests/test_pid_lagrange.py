"""
PID Lagrangian 순수 부분 시험 (amr_isaac.pid_lagrange, Isaac·rsl_rl 불필요).

- 예산 위 비용 → λ 증가, 예산 아래 → 0 으로 (투영), λ 상한·적분 반-와인드업, 정규화 (J − d)/d (예산 배율 불변)
- 미분항은 비용이 늘 때만, 지연 창
- GAE 가 rsl_rl 점화식과 같고 (손 계산), 결합 이익이 λ = 0 에서 표준화 이익과 비트 단위로 같다
- 에피소드 비용 합 추적기
"""

import math

import pytest
import torch

from amr_isaac import pid_lagrange as PL


def test_lambda_rises_above_budget_and_falls_to_zero_below():
    """J = 2d 가 이어지면 λ 가 늘고, J = 0 이 이어지면 0 으로 돌아온다 (I 는 0 아래로 가지 않는다)."""
    pid = PL.PIDLagrangian(kp=0.0, ki=0.1, kd=0.0, budget=0.5, lambda_max=10.0)
    lams = [pid.update(1.0) for _ in range(20)]
    assert all(b > a for a, b in zip(lams, lams[1:]))
    assert lams[-1] == pytest.approx(20 * 0.1 * 1.0)
    for _ in range(100):
        lam = pid.update(0.0)
    assert lam == 0.0 and pid.integral == 0.0


def test_lambda_cap_and_antiwindup():
    """λ 와 I 는 상한에서 멈춘다 — 비용이 예산 아래로 내려가면 바로 줄기 시작한다."""
    pid = PL.PIDLagrangian(kp=0.0, ki=1.0, kd=0.0, budget=0.2, lambda_max=3.0)
    for _ in range(50):
        lam = pid.update(10.0)
    assert lam == 3.0 and pid.integral == 3.0
    lam2 = pid.update(0.0)        # e = −1 → I = 2
    assert lam2 == pytest.approx(2.0)


def test_normalized_error_is_budget_scale_invariant():
    """같은 J/d 비율이면 예산 크기와 무관하게 같은 λ 궤적 (정규화)."""
    a = PL.PIDLagrangian(kp=0.5, ki=0.05, kd=0.2, budget=0.2, d_delay=3, ema_p=0.5, ema_d=0.3)
    b = PL.PIDLagrangian(kp=0.5, ki=0.05, kd=0.2, budget=1.0, d_delay=3, ema_p=0.5, ema_d=0.3)
    seq = [0.1, 0.5, 0.9, 0.3, 0.0, 1.5, 2.0, 0.2]
    la = [a.update(x * 0.2) for x in seq]
    lb = [b.update(x * 1.0) for x in seq]
    assert la == pytest.approx(lb, abs=1e-12)


def test_derivative_only_on_increase_with_delay():
    """미분항: 지연 창 안에서 J̄ 가 늘 때만 양수, 줄면 0."""
    pid = PL.PIDLagrangian(kp=0.0, ki=0.0, kd=1.0, budget=1.0, d_delay=2)
    pid.update(1.0)
    pid.update(2.0)
    assert pid.last['D'] == pytest.approx(1.0)
    pid.update(4.0)                # 창 [1, 2, 4] → 4 − 1
    assert pid.last['D'] == pytest.approx(3.0)
    pid.update(0.5)                # 창 [2, 4, 0.5] → 감소 → 0
    assert pid.last['D'] == 0.0


def test_proportional_term_and_init():
    """비례항 = K_p·e, λ_init 은 I 의 초기값."""
    pid = PL.PIDLagrangian(kp=2.0, ki=0.0, kd=0.0, budget=0.5, lambda_init=0.3)
    assert pid.lam == pytest.approx(0.3)
    lam = pid.update(0.75)         # e = 0.5 → 2·0.5 + 0.3
    assert lam == pytest.approx(1.3)


def test_invalid_budget_and_cap():
    """예산 0 (Slater 위반)·상한 0 은 거부."""
    with pytest.raises(ValueError):
        PL.PIDLagrangian(0.1, 0.1, 0.1, budget=0.0)
    with pytest.raises(ValueError):
        PL.PIDLagrangian(0.1, 0.1, 0.1, budget=0.5, lambda_max=0.0)


def test_gae_matches_hand_recursion():
    """T = 3, N = 2 손 계산 (종단에서 끊김)."""
    g, lam = 0.9, 0.5
    r = torch.tensor([[[1.0], [0.0]], [[0.0], [1.0]], [[2.0], [0.0]]])
    v = torch.tensor([[[0.5], [0.1]], [[0.2], [0.3]], [[0.4], [0.0]]])
    d = torch.tensor([[[0], [0]], [[1], [0]], [[0], [0]]], dtype=torch.uint8)
    last = torch.tensor([[1.0], [2.0]])
    ret, adv = PL.gae(r, v, d, last, g, lam)
    # env 0: t2: δ = 2 + .9·1 − .4 = 2.5, A2 = 2.5; t1 (종단): δ = 0 − .2 = −.2, A1 = −.2;
    #        t0: δ = 1 + .9·.2 − .5 = .68, A0 = .68 + .45·(−.2) = .59
    # env 1: t2: δ = 0 + .9·2 − 0 = 1.8; t1: δ = 1 + 0 − .3 = .7, A1 = .7 + .45·1.8 = 1.51;
    #        t0: δ = 0 + .9·.3 − .1 = .17, A0 = .17 + .45·1.51 = .8495
    exp = torch.tensor([[[0.59], [0.8495]], [[-0.2], [1.51]], [[2.5], [1.8]]])
    assert torch.allclose(adv, exp, atol=1e-6)
    assert torch.allclose(ret, adv + v)


def test_combine_advantages_lambda_zero_is_rsl_standardization():
    """λ = 0 → (A − mean)/(std + 1e-8) 와 비트 단위로 같다 (rsl_rl compute_returns 의 표준화)."""
    torch.manual_seed(0)
    a = torch.randn(64, 16, 1) * 3 + 1
    c = torch.randn(64, 16, 1) * 0.1
    ref = (a - a.mean()) / (a.std() + 1e-8)
    assert torch.equal(PL.combine_advantages(a, c, 0.0), ref)
    # λ 가 크면 −center(A_c) 쪽으로
    big = PL.combine_advantages(a, c, 1e6)
    assert torch.allclose(big, -(c - c.mean()), atol=1e-4)


def test_episode_cost_tracker():
    """끝난 에피소드의 비할인 합만 평균에 들어가고, pop 뒤 비워진다."""
    tr = PL.EpisodeCostTracker(3, 'cpu')
    tr.step(torch.tensor([1.0, 0.5, 0.0]), torch.tensor([False, False, False]))
    tr.step(torch.tensor([1.0, 0.5, 2.0]), torch.tensor([True, False, False]))
    tr.step(torch.tensor([0.0, 1.0, 0.0]), torch.tensor([False, True, False]))
    J, n = tr.pop()
    assert n == 2 and J == pytest.approx((2.0 + 2.0) / 2)
    assert tr.pop() == (None, 0)
    tr.step(torch.tensor([0.0, 0.0, 1.0]), torch.tensor([False, False, True]))
    J, n = tr.pop()
    assert n == 1 and J == pytest.approx(3.0) and not math.isnan(J)


def test_lambda_cap_applies_to_proportional_term():
    """적분이 0 이어도 비례항이 상한을 넘으면 λ = λ_max (변이 L2 가 살아남아 더한 시험)."""
    pid = PL.PIDLagrangian(kp=1.0, ki=0.0, kd=0.0, budget=0.2, lambda_max=3.0)
    lam = pid.update(10.0)                 # e = 49 → 비례 49
    assert pid.integral == 0.0 and pid.last['raw'] == pytest.approx(49.0) and lam == 3.0
